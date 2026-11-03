"""One lidar owner + geometry process. No chassis UART or T265 SDK imports.

Motion arrives from the existing owners; bounded queues carry original times.
Only display snapshots coalesce. Raw measurements and decisions remain logged.
"""
from collections import deque
import multiprocessing as mp
import os
import queue
import time
import uuid
import numpy as np
import serial
from async_debug_log import AsyncDebugLog
from .dynamic import DynamicFence
from .lidar_reader import LidarStream
from .lidar_timing import LidarTimingError
from .protocol import command_frame


def make_reader(config):
    mount=config['installation'];timing=dict(config['packet_timing'])
    timing['max_receive_gap_ns']=2**62
    return LidarStream(dict(bearing_sign=1,bearing_zero_deg=0.,physical_direction=1,
        center_raw_deg=mount['center_raw_deg'],fov_deg=mount['display_fov_deg'],
        packet_timing=timing,max_batch_span_ns=2**62,
        sector_limits=dict(max_span_ns=2**62,max_point_gap_ns=2**62,max_stall_ns=2**62)),
        sensor_epoch=uuid.uuid4().hex)


def run_worker(config,port,baud,root,inbox,display,replies,stop):
    # Spawn target: never inherits UART/SDK handles from the parent.
    display.cancel_join_thread()  # Raw records/final summary use the logger.
    engine=DynamicFence(config['installation']);log=AsyncDebugLog(root)
    lidar=None;reader=None;reader_totals={};pending=deque();times=deque(maxlen=2048)
    counters=dict(pose_ingested=0,primary_yaw_ingested=0,scans_received=0,
        scan_queue_drops=0,display_updates_coalesced=0)
    error=None;last_publish=0.;last_percentile=0.;p95=None;reference_request_id=None

    def event(name,**data):
        log.submit('lidar_worker_events.jsonl',dict(event=name,timestamp_monotonic_ns=time.monotonic_ns(),**data))

    def reply(kind,**data):
        replies.put_nowait(dict(kind=kind,**data))

    def consume():
        nonlocal reference_request_id
        for _ in range(1024):
            try:kind,payload=inbox.get_nowait()
            except queue.Empty:break
            if kind=='pose':engine.ingest_pose(payload);counters['pose_ingested']+=1
            elif kind=='yaw':
                engine.ingest_yaw(payload['time_ns'],payload['yaw_deg'],payload['source'])
                counters['primary_yaw_ingested']+=1
            elif kind=='invalidate':
                engine.invalidate_reference(payload['reason']);event('reference_invalidated',**payload)
            elif kind=='align':
                try:
                    requested=engine.request_initialization(payload['description'],physical_confirmed=payload['physical_confirmed'])
                    reference_request_id=payload['request_id']
                    reply('reference_search_started',request_id=reference_request_id)
                    event('field_reference_search_started',request_id=reference_request_id,initialization=requested)
                except ValueError as exc:reply('reference_rejected',request_id=payload['request_id'],error=str(exc))
            elif kind=='lidar_enabled':
                engine.set_lidar_enabled(payload['enabled'])
                event('operator_lidar_availability',enabled=payload['enabled'],reason=payload['reason'])

    def state():
        stats=dict(reader_totals)
        if reader:
            for k,v in reader.stats.items():stats[k]=stats.get(k,0)+v
        return dict(counters,scans=engine.scan_count,fits=engine.fit_count,
            scan_processing_ms_p95=p95,last=engine.snapshot(),reader_stats=stats,error=error,
            worker_pid=os.getpid(),worker_cpu_affinity=sorted(os.sched_getaffinity(0)),
            capture_chain_present=bool(engine.poses and engine.yaws and counters['scans_received']),
            have_tracked_pose=bool(engine.poses),have_primary_yaw=bool(engine.yaws),
            lidar_corrections_enabled=engine.lidar_corrections_enabled,
            logging_healthy=log.healthy(),accuracy_verified=False,full_speed_physical_validation=False)

    def publish(value):
        try:display.put_nowait(value)
        except queue.Full:
            try:display.get_nowait()
            except queue.Empty:return
            counters['display_updates_coalesced']+=1
            try:display.put_nowait(value)
            except queue.Full:pass

    try:
        lidar=serial.Serial(port,baud,timeout=.005,write_timeout=.2,exclusive=True)
        reader=make_reader(config)
        for mode,value in ((0xA0,0),(0xA2,3600)):
            packet=command_frame(mode,value)
            log.submit('lidar_tx.jsonl',dict(timestamp_monotonic_ns=time.monotonic_ns(),data_hex=packet.hex()))
            if lidar.write(packet)!=len(packet):raise RuntimeError('雷达启动命令写入不完整')
        reply('started',pid=os.getpid());event('worker_started',port=port,baud=baud,
            chassis_uart_owned=False,t265_sdk_owned=False)
        while not stop.is_set():
            consume()
            data=lidar.read(min(lidar.in_waiting or 1,65536));now=time.monotonic_ns()
            if data:
                log.submit('lidar_rx.jsonl',dict(timestamp_monotonic_ns=now,data_hex=data.hex()))
                try:scans=reader.feed(data,now)
                except LidarTimingError as exc:
                    event('lidar_segment_restart',reason=str(exc))
                    for k,v in reader.stats.items():reader_totals[k]=reader_totals.get(k,0)+v
                    reader=make_reader(config);scans=[]
                for scan in scans:
                    counters['scans_received']+=1
                    log.submit('dynamic_scans.jsonl',dict(scan_id=scan.scan_id,reference_ns=scan.reference_ns,
                        time_qualified=False,points=[vars(p).copy() for p in scan.points]))
                    if len(pending)>=4:
                        pending.popleft();counters['scan_queue_drops']+=1
                        event('bounded_scan_queue_overflow',dropped=counters['scan_queue_drops'])
                    pending.append(scan)
            consume()  # Motion received while parsing is available to this scan.
            if pending:
                scan=pending.popleft()
                if engine.reference is None and reference_request_id is not None:
                    result=engine.try_initialize(scan)
                    log.submit('dynamic_initialization.jsonl',result)
                    if result.get('reference') is not None:
                        reply('reference_set',request_id=reference_request_id,reference=result['reference'])
                        event('field_reference_set',request_id=reference_request_id,reference=result['reference'])
                        reference_request_id=None
                if engine.reference is not None:
                    try:
                        result=engine.observe(scan);times.append(result['compute_ms'])
                        log.submit('dynamic_localization.jsonl',result)
                    except ValueError as exc:event('scan_not_fitted',scan_id=scan.scan_id,reason=str(exc))
            if time.monotonic()-last_publish>=.05:
                if times and time.monotonic()-last_percentile>=1.:
                    p95=float(np.percentile(times,95));last_percentile=time.monotonic()
                publish(state());last_publish=time.monotonic()
    except Exception as exc:
        error=str(exc);event('worker_error',reason=error)
        try:reply('error',error=error)
        except queue.Full:pass
    finally:
        if lidar is not None:
            try:
                packet=command_frame(0xA1,0);lidar.write(packet)
                log.submit('lidar_tx.jsonl',dict(timestamp_monotonic_ns=time.monotonic_ns(),data_hex=packet.hex()))
            except (OSError,serial.SerialException) as exc:event('lidar_stop_write_failed',reason=str(exc))
            lidar.close()
        if times:p95=float(np.percentile(times,95))
        final=state();log.submit('lidar_worker_summary.json',final,atomic=True);log.close()
        final['logging_healthy']=log.healthy();final['logging_statistics']=log.statistics()
        # Keep final truth independent of the coalescing display queue.
        try:reply('stopped',summary=final)
        except queue.Full:pass


class RadarWorker:
    def __init__(self):
        self.context=mp.get_context('spawn');self.process=None
        self.inbox=self.context.Queue(maxsize=1024);self.display=self.context.Queue(maxsize=4)
        self.replies=self.context.Queue(maxsize=16);self.stop=self.context.Event()
        self.state={};self.input_drops=0;self.pending_reference=None;self.stopped_cleanly=False

    def start(self,config,port,baud,root):
        self.process=self.context.Process(target=run_worker,args=(config,port,baud,root,
            self.inbox,self.display,self.replies,self.stop),name='lidar-geometry-owner')
        self.process.start()

    def send(self,kind,payload):
        try:self.inbox.put_nowait((kind,payload));return True
        except queue.Full:self.input_drops+=1;return False

    def align(self,description,*,physical_confirmed=True):
        request_id=uuid.uuid4().hex
        if not self.send('align',dict(request_id=request_id,description=description,physical_confirmed=physical_confirmed)):
            raise ValueError('参考请求队列已满，未建立参考')
        self.pending_reference=request_id
        return request_id

    def poll(self):
        for _ in range(8):
            try:value=self.display.get_nowait()
            except queue.Empty:break
            if not self.stopped_cleanly:self.state=value
        results=[]
        for _ in range(16):
            try:message=self.replies.get_nowait()
            except queue.Empty:break
            if message['kind'] in ('reference_set','reference_rejected'):
                if message['request_id']!=self.pending_reference:continue
                self.pending_reference=None
            if message['kind']=='stopped':self.state=message['summary'];self.stopped_cleanly=True
            results.append(message)
        return results

    def close(self):
        self.stop.set()
        if self.process is not None:
            deadline=time.monotonic()+5
            while self.process.is_alive() and time.monotonic()<deadline:
                self.poll();self.process.join(timeout=.05)
            self.poll()
            if self.process.is_alive():
                self.process.terminate();self.process.join(timeout=1)
                self.state['error']='雷达进程未能正常退出';self.stopped_cleanly=False
        for channel in (self.inbox,self.display,self.replies):
            channel.cancel_join_thread();channel.close()
