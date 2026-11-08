"""Independent lidar reception, latest-scan fitting and motion publication.

No chassis UART or T265 SDK imports. Raw data retain their original timestamps;
only pending fit requests/control/display snapshots coalesce, without an age gate.
"""
from collections import deque
import multiprocessing as mp
import os
import queue
import threading
import time
import uuid
import numpy as np
import serial
from async_debug_log import AsyncDebugLog
from low_latency_scheduling import prepare_realtime, set_realtime_thread
from .dynamic import DynamicFence
from .lidar_reader import LidarStream
from .lidar_timing import LidarTimingError
from .latest_snapshot import LatestSnapshot, MotionInbox
from .protocol import command_frame
from .native_geometry import backend_status


def make_reader(config):
    mount=config['installation'];timing=dict(config['packet_timing'])
    timing['max_receive_gap_ns']=2**62
    return LidarStream(dict(bearing_sign=1,bearing_zero_deg=0.,physical_direction=1,
        center_raw_deg=mount['center_raw_deg'],fov_deg=mount['display_fov_deg'],
        packet_timing=timing,max_batch_span_ns=2**62,
        sector_limits=dict(max_span_ns=2**62,max_point_gap_ns=2**62,max_stall_ns=2**62)),
        sensor_epoch=uuid.uuid4().hex)


def offer_latest(channel, value):
    """Bounded mailbox, not a FIFO of old control/fit requests."""
    dropped = 0
    try:
        channel.put_nowait(value)
        return dropped
    except queue.Full:
        try:
            channel.get_nowait()
            dropped += 1
        except queue.Empty:
            return 1  # Multiprocessing feeder has not exposed the queued item yet.
    try:
        channel.put_nowait(value)
    except queue.Full:
        dropped += 1
    return dropped


def run_lidar_receiver(config, port, baud, root, scans, updates, stop):
    """Sole lidar serial owner; fitting/motion/GUI cannot delay the next read."""
    scans.cancel_join_thread()
    log = AsyncDebugLog(root)
    lidar = reader = None
    totals = {}
    state = dict(lidar_connected=False, lidar_error=None, lidar_open_attempts=0,
                 scans_received=0, scan_queue_drops=0, reader_stats={}, receiver_pid=os.getpid())
    next_open = last_publish = 0.

    def event(name, **data):
        log.submit('lidar_reader_events.jsonl', dict(event=name,
            timestamp_monotonic_ns=time.monotonic_ns(), **data))

    def publish():
        stats = dict(totals)
        if reader:
            for k, v in reader.stats.items():
                stats[k] = stats.get(k, 0) + v
        state['reader_stats'] = stats
        state['reader_logging_healthy'] = log.healthy()
        offer_latest(updates, dict(state))

    def disconnect(exc):
        nonlocal lidar, reader, next_open
        if lidar is not None:
            try: lidar.close()
            except (OSError, serial.SerialException): pass
        if reader:
            for k, v in reader.stats.items(): totals[k] = totals.get(k, 0) + v
        lidar = reader = None
        state.update(lidar_connected=False, lidar_error=str(exc))
        event('lidar_connection_error', reason=str(exc), motion_propagation_continues=True)
        next_open = time.monotonic() + 1.
        publish()

    try:
        while not stop.is_set():
            if lidar is None:
                if time.monotonic() < next_open:
                    stop.wait(.005)
                    continue
                state['lidar_open_attempts'] += 1
                try:
                    lidar = serial.Serial(port, baud, timeout=.005, write_timeout=.2, exclusive=True)
                    for mode, value in ((0xA0, 0), (0xA2, 3600)):
                        packet = command_frame(mode, value)
                        log.submit('lidar_tx.jsonl', dict(timestamp_monotonic_ns=time.monotonic_ns(), data_hex=packet.hex()))
                        if lidar.write(packet) != len(packet):
                            raise serial.SerialException('雷达启动命令写入不完整')
                    reader = make_reader(config)
                    state.update(lidar_connected=True, lidar_error=None)
                    event('lidar_connected', port=port, open_attempt=state['lidar_open_attempts'])
                    publish()
                except (OSError, serial.SerialException) as exc:
                    disconnect(exc)
                    continue
            try:
                data = lidar.read(min(lidar.in_waiting or 1, 65536))
            except (OSError, serial.SerialException) as exc:
                disconnect(exc)
                continue
            received = time.monotonic_ns()
            if data:
                log.submit('lidar_rx.jsonl', dict(timestamp_monotonic_ns=received, data_hex=data.hex()))
                try:
                    ready = reader.feed(data, received)
                except LidarTimingError as exc:
                    event('lidar_segment_restart', reason=str(exc))
                    for k, v in reader.stats.items(): totals[k] = totals.get(k, 0) + v
                    reader = make_reader(config)
                    ready = []
                for scan in ready:
                    state['scans_received'] += 1
                    log.submit('dynamic_scans.jsonl', dict(scan_id=scan.scan_id,
                        reference_ns=scan.reference_ns, time_qualified=False,
                        points=[vars(p).copy() for p in scan.points]))
                    state['scan_queue_drops'] += offer_latest(scans, scan)
            if time.monotonic() - last_publish >= .05:
                publish()
                last_publish = time.monotonic()
    except Exception as exc:
        state['lidar_error'] = str(exc)
        event('lidar_receiver_error', reason=str(exc))
    finally:
        if lidar is not None:
            try:
                packet = command_frame(0xA1, 0)
                lidar.write(packet)
                log.submit('lidar_tx.jsonl', dict(timestamp_monotonic_ns=time.monotonic_ns(), data_hex=packet.hex()))
            except (OSError, serial.SerialException) as exc: event('lidar_stop_write_failed', reason=str(exc))
            try: lidar.close()
            except (OSError, serial.SerialException): pass
        state['lidar_connected'] = False
        publish()
        log.submit('lidar_reader_summary.json', dict(state), atomic=True)
        log.close()
        state['reader_logging_healthy'] = log.healthy()
        state['reader_logging_statistics'] = log.statistics()
        offer_latest(updates, dict(state))


def run_worker(config,port,baud,root,inbox,display,replies,stop,control=None):
    # This process owns motion propagation. Its receiver child alone owns lidar.
    display.cancel_join_thread()
    engine=DynamicFence(config['installation']);log=AsyncDebugLog(root)
    context=mp.get_context('spawn')
    scans=context.Queue(maxsize=2);updates=context.Queue(maxsize=8)
    receiver=context.Process(target=run_lidar_receiver,
        args=(config,port,baud,root,scans,updates,stop),name='lidar-serial-receiver')
    fit_inbox=queue.Queue(maxsize=1);fit_done=queue.Queue(maxsize=8)
    fitting_stop=threading.Event()
    reader_state=dict(lidar_connected=False,lidar_error=None,lidar_open_attempts=0,
        scans_received=0,scan_queue_drops=0,reader_stats={})
    counters=dict(pose_ingested=0,primary_yaw_ingested=0,measured_wheel_ingested=0,
        fit_requests_coalesced=0,display_updates_coalesced=0,
        control_updates=0,control_updates_coalesced=0,control_capacity_errors=0)
    times=deque(maxlen=2048);error=None;last_publish=last_percentile=0.;p95=None
    reference_request_id=None;requested_lidar_enabled=True
    last_control_key=None
    scheduling=None

    def event(name,**data):
        log.submit('lidar_worker_events.jsonl',dict(event=name,timestamp_monotonic_ns=time.monotonic_ns(),**data))

    def reply(kind,**data):
        replies.put_nowait(dict(kind=kind,**data))

    def consume(first):
        nonlocal reference_request_id,requested_lidar_enabled
        changed=False
        for _ in range(256):
            if first is not None:
                kind,payload=first;first=None
            else:
                try:kind,payload=inbox.get_nowait()
                except queue.Empty:break
            changed=True
            if kind=='pose':engine.ingest_pose(payload);counters['pose_ingested']+=1
            elif kind=='yaw':
                engine.ingest_yaw(payload['time_ns'],payload['yaw_deg'],payload['source'])
                counters['primary_yaw_ingested']+=1
            elif kind=='wheel':engine.ingest_wheel(payload);counters['measured_wheel_ingested']+=1
            elif kind=='invalidate':
                engine.invalidate_reference(payload['reason']);event('reference_invalidated',**payload)
            elif kind=='align':
                try:
                    requested=engine.request_initialization(payload['description'],physical_confirmed=payload['physical_confirmed'])
                    reference_request_id=payload['request_id']
                    reply('reference_search_started',request_id=reference_request_id)
                    event('field_reference_search_started',request_id=reference_request_id,initialization=requested)
                except ValueError as exc:reply('reference_rejected',request_id=payload['request_id'],error=str(exc))
            elif kind=='field_start':
                reference=engine.align(payload['description'],physical_confirmed=False,
                    center_xy=payload['center_xy_m'],heading_deg=payload['yaw_deg'],
                    first_sample=payload.get('first_sample',True),round_token=payload.get('round_token'))
                reference_request_id=None
                # Pending observations belong to the previous placement.
                for channel in (scans,fit_inbox):
                    while True:
                        try:channel.get_nowait()
                        except queue.Empty:break
                event('field_reference_set',reference=reference)
            elif kind=='lidar_enabled':
                requested_lidar_enabled=bool(payload['enabled'])
                engine.set_lidar_enabled(requested_lidar_enabled and reader_state['lidar_connected'])
                event('operator_lidar_availability',enabled=payload['enabled'],reason=payload['reason'])
        with engine.lock:
            while engine.motion.events:
                change=engine.motion.events.popleft();name=change.pop('event');event(name,**change)
        return changed

    def publish_control():
        nonlocal last_control_key
        # Small, coherent latest value. No scan points, percentile calculation,
        # display queue or file write is on this publication path.
        value=dict(last=engine.control_snapshot(),have_tracked_pose=bool(engine.poses),
            have_primary_yaw=bool(engine.yaws),lidar_connected=reader_state['lidar_connected'],
            lidar_error=reader_state['lidar_error'])
        pose=value['last'];raw=engine.last_sample or {}
        # A future SDK Pose/gyro that has not changed the supported endpoint
        # must not turn the same 100Hz wheel endpoint into extra control frames.
        # Raw input histories and the full display diagnostics remain intact.
        key=(engine.revision,pose.get('ready'),pose.get('sample_monotonic_ns'),
            tuple(pose.get('center_xy_m',[])),pose.get('yaw_deg'),pose.get('source'),
            tuple(pose.get('current_wall_axes',[])),pose.get('fixes'),
            raw.get('tracker_confidence'),raw.get('mapper_confidence'),
            value['have_tracked_pose'],value['have_primary_yaw'],
            value['lidar_connected'],value['lidar_error'])
        if key==last_control_key:return
        try:
            counters['control_updates_coalesced']+=control.put_latest(value)
            counters['control_updates']+=1
            last_control_key=key
        except ValueError as exc:
            counters['control_capacity_errors']+=1
            event('control_snapshot_capacity_error',reason=str(exc))

    def state():
        return dict(counters,**reader_state,scans=engine.scan_count,fits=engine.fit_count,
            scan_processing_ms_p95=p95,last=engine.snapshot(),error=error,
            worker_pid=os.getpid(),worker_cpu_affinity=sorted(os.sched_getaffinity(0)),
            capture_chain_present=bool(engine.poses and engine.yaws and reader_state['scans_received']),
            have_tracked_pose=bool(engine.poses),have_primary_yaw=bool(engine.yaws),
            lidar_requested_enabled=requested_lidar_enabled,
            lidar_corrections_enabled=engine.lidar_corrections_enabled,
            logging_healthy=log.healthy(),motion_scheduling=scheduling,line_geometry=backend_status(),
            accuracy_verified=False,full_speed_physical_validation=False)

    def drain_reader():
        nonlocal reader_state
        dirty=False
        while True:
            try:value=updates.get_nowait()
            except queue.Empty:break
            changed=(value['lidar_connected']!=reader_state['lidar_connected'] or
                     value['lidar_error']!=reader_state['lidar_error'])
            reader_state=value
            if changed:
                dirty=True
                engine.set_lidar_enabled(requested_lidar_enabled and value['lidar_connected'])
                event('lidar_connected' if value['lidar_connected'] else 'lidar_connection_error',
                    reason=value['lidar_error'],motion_propagation_continues=True)
        return dirty

    def fit_loop():
        while not fitting_stop.is_set():
            try:scan,revision,request_id=fit_inbox.get(timeout=.05)
            except queue.Empty:continue
            try:
                with engine.lock:
                    if revision!=engine.revision:continue
                    initializing=engine.reference is None and request_id is not None
                    # Initialization has no control reference yet. Keep its
                    # three-scan state atomic against a new operator bind.
                    result=engine.try_initialize(scan) if initializing else None
                if not initializing:
                    if engine.reference is None:continue
                    result=engine.observe(scan,expected_revision=revision)
                fit_done.put((initializing,request_id,result),timeout=.05)
            except ValueError as exc:event('scan_not_fitted',scan_id=scan.scan_id,reason=str(exc))
            except queue.Full:event('fit_result_queue_full',scan_id=scan.scan_id)
            except Exception as exc:
                # A geometry failure cannot terminate motion propagation.
                event('scan_fit_error',scan_id=scan.scan_id,reason=str(exc))

    def run_fit():
        # Explicit developer profiling of the existing worker only. Disabled
        # in normal operation; no extra per-sample timers or log records.
        path=os.environ.get('RDK_LIDAR_FIT_PROFILE')
        if not path:return fit_loop()
        import cProfile
        profiler=cProfile.Profile()
        try:profiler.runcall(fit_loop)
        finally:profiler.dump_stats(path)

    fitter=threading.Thread(target=run_fit,name='lidar-fit',daemon=True)

    def drain_fits():
        nonlocal reference_request_id
        changed=False
        for _ in range(8):
            try:initializing,request_id,result=fit_done.get_nowait()
            except queue.Empty:break
            changed=True
            if initializing:
                log.submit('dynamic_initialization.jsonl',result)
                if result.get('reference') is not None and request_id==reference_request_id:
                    reply('reference_set',request_id=request_id,reference=result['reference'])
                    event('field_reference_set',request_id=request_id,reference=result['reference'])
                    reference_request_id=None
            else:
                times.append(result['compute_ms']);log.submit('dynamic_localization.jsonl',result)
                if result.get('position_reacquired'):
                    event('lidar_position_reacquired',scan_id=result['scan_id'],
                        reference_ns=result['reference_ns'],center_xy_m=result['candidate_center_xy_m'],
                        correction_m=result['correction_m'],motion_source=result['motion_source'])
        return changed

    try:
        receiver.start();fitter.start()
        # Start display/reply feeder threads under normal scheduling too.
        offer_latest(display,state())
        reply('started',pid=os.getpid());event('worker_started',port=port,baud=baud,
            chassis_uart_owned=False,t265_sdk_owned=False,receiver_pid=receiver.pid)
        scheduling=set_realtime_thread()
        event('motion_scheduling',**scheduling)
        while not stop.is_set():
            # Queue arrival wakes motion immediately; the 2ms idle timeout only
            # services scan/results/stop when there is no motion input.
            try:first=inbox.get(timeout=.002)
            except queue.Empty:first=None
            changed=consume(first)
            changed=drain_reader() or changed
            latest=None
            for _ in range(8):
                try:latest=scans.get_nowait()
                except queue.Empty:break
            if latest is not None:
                counters['fit_requests_coalesced']+=offer_latest(fit_inbox,
                    (latest,engine.revision,reference_request_id))
            changed=drain_fits() or changed
            if control is not None and changed:publish_control()
            if time.monotonic()-last_publish>=.05:
                if times and time.monotonic()-last_percentile>=1.:
                    p95=float(np.percentile(times,95));last_percentile=time.monotonic()
                counters['display_updates_coalesced']+=offer_latest(display,state())
                last_publish=time.monotonic()
    except Exception as exc:
        error=str(exc);event('worker_error',reason=error)
        try:reply('error',error=error)
        except queue.Full:pass
    finally:
        stop.set();fitting_stop.set()
        if fitter.ident is not None:fitter.join(timeout=2)
        drain_fits()
        if receiver.pid is not None:
            deadline=time.monotonic()+2
            while receiver.is_alive() and time.monotonic()<deadline:
                drain_reader();receiver.join(timeout=.02)
            drain_reader()
            if receiver.is_alive():receiver.terminate();receiver.join(timeout=1)
        if times:p95=float(np.percentile(times,95))
        final=state();log.submit('lidar_worker_summary.json',final,atomic=True);log.close()
        final['logging_healthy']=log.healthy() and reader_state.get('reader_logging_healthy',False)
        final['logging_statistics']=log.statistics()
        final['receiver_stopped_cleanly']=receiver.exitcode==0
        final['fit_thread_stopped']=not fitter.is_alive()
        try:reply('stopped',summary=final)
        except queue.Full:pass
        for channel in (scans,updates):channel.cancel_join_thread();channel.close()


class RadarWorker:
    def __init__(self,*,control_updates=False):
        self.context=mp.get_context('spawn');self.process=None
        self.inbox=MotionInbox();self.display=self.context.Queue(maxsize=4)
        self.replies=self.context.Queue(maxsize=16);self.stop=self.context.Event()
        self.control=LatestSnapshot(self.context) if control_updates else None
        self.state={};self.input_drops=0;self.pending_reference=None;self.stopped_cleanly=False
        self.scheduling_setup=None

    def start(self,config,port,baud,root):
        self.scheduling_setup=prepare_realtime()
        self.process=self.context.Process(target=run_worker,args=(config,port,baud,root,
            self.inbox,self.display,self.replies,self.stop,self.control),name='lidar-geometry-owner')
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
            if message['kind']=='stopped':
                self.state=message['summary']
                self.stopped_cleanly=bool(self.state.get('receiver_stopped_cleanly') and self.state.get('fit_thread_stopped'))
            results.append(message)
        return results

    def close(self):
        self.stop.set()
        if self.control is not None:self.control.wake()
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
