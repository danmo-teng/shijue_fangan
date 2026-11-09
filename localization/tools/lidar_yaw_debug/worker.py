"""Independent lidar reception, latest-scan fitting and motion publication.

No chassis UART or T265 SDK imports. Raw data retain their original timestamps;
only pending fit requests/control/display snapshots coalesce, without an age gate.
"""
from collections import deque
import multiprocessing as mp
import os
import socket
import select
import struct
import json
import queue
import signal
import threading
import time
import uuid
import numpy as np
import serial
from async_debug_log import AsyncDebugLog,ProcessDebugLog
from local_packet_receiver import LocalPacketReceiver,LocalDatagramSender,readable_nowait
from low_latency_scheduling import prepare_realtime, set_realtime_thread
from .dynamic import DynamicFence
from .native_lidar_reader import make_lidar_stream, NativeReaderError
from .lidar_timing import LidarTimingError
from .latest_snapshot import LatestSnapshot, MotionInbox
from .protocol import command_frame
from .native_geometry import backend_status
from .inputs import NativeFeedbackAdapter
from .control_packet import format_control_packet
from .native_motion import NativeDynamicFence,NativeMotionRuntime


def make_reader(config, *, native=True):
    mount=config['installation'];timing=dict(config['packet_timing'])
    timing['max_receive_gap_ns']=2**62
    return make_lidar_stream(dict(bearing_sign=1,bearing_zero_deg=0.,physical_direction=1,
        center_raw_deg=mount['center_raw_deg'],fov_deg=mount['display_fov_deg'],
        packet_timing=timing,max_batch_span_ns=2**62,
        sector_limits=dict(max_span_ns=2**62,max_point_gap_ns=2**62,max_stall_ns=2**62)),
        sensor_epoch=uuid.uuid4().hex, native=native)


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
    native_reader = True
    totals = {}
    state = dict(lidar_connected=False, lidar_error=None, lidar_open_attempts=0,
                 scans_received=0, scan_queue_drops=0, reader_stats={}, receiver_pid=os.getpid(),
                 reader_backend=None, reader_backend_error=None)
    next_open = last_publish = 0.

    def event(name, **data):
        log.submit('lidar_reader_events.jsonl', dict(event=name,
            timestamp_monotonic_ns=time.monotonic_ns(), **data))

    def publish():
        stats = dict(totals)
        if reader:
            for k, v in reader.stats.items():
                stats[k] = stats.get(k, 0) + v
            state.update(reader_backend=reader.backend, reader_backend_error=reader.backend_error)
        state['reader_stats'] = stats
        state['reader_logging_healthy'] = log.healthy()
        offer_latest(updates, dict(state))

    def publish_scan(scan):
        state['scans_received'] += 1
        log.submit('dynamic_scans.jsonl', dict(scan_id=scan.scan_id,
            reference_ns=scan.reference_ns, time_qualified=False,
            points=[vars(p).copy() for p in scan.points]))
        state['scan_queue_drops'] += offer_latest(scans, scan)

    def release_reader():
        nonlocal reader
        if reader:
            for k, v in reader.stats.items(): totals[k] = totals.get(k, 0) + v
            close = getattr(reader, 'close', None)
            if close is not None: close()
        reader = None

    def disconnect(exc):
        nonlocal lidar, reader, next_open
        if lidar is not None:
            try: lidar.close()
            except (OSError, serial.SerialException): pass
        release_reader()
        lidar = None
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
                    reader = make_reader(config, native=native_reader)
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
                    reader.feed(data, received, on_scan=publish_scan)
                except NativeReaderError as exc:
                    # Keep the same serial owner and already delivered scans.
                    # Do not replay this partially consumed batch or keep an
                    # incomplete sector across different implementations.
                    event('lidar_reader_fallback', reason=str(exc), backend='python',
                          motion_propagation_continues=True)
                    native_reader = False
                    release_reader()
                    reader = make_reader(config, native=False)
                    reader.backend_error = str(exc)
                except LidarTimingError as exc:
                    event('lidar_segment_restart', reason=str(exc))
                    release_reader()
                    reader = make_reader(config, native=native_reader)
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
        release_reader()
        log.submit('lidar_reader_summary.json', dict(state), atomic=True)
        log.close()
        state['reader_logging_healthy'] = log.healthy()
        state['reader_logging_statistics'] = log.statistics()
        offer_latest(updates, dict(state))


def run_geometry_jobs(connection):
    """One numerical owner; no UART, SDK, estimator state or control output."""
    from .scan_geometry import geometry_job
    try:
        while True:
            request=connection.recv()
            if request is None:break
            try:
                args,kwargs=request
                result=geometry_job(*args,**kwargs)
                connection.send((True,result,backend_status()))
            except Exception as exc:
                connection.send((False,str(exc),backend_status()))
    except (EOFError,OSError):pass
    finally:connection.close()


def run_worker(config,port,baud,root,inbox,display,replies,stop,control=None,sdk_policy=None,native_feedback=None,control_output=None,pose_input=None,wheel_policy_input=None):
    # This process owns motion propagation. Its receiver child alone owns lidar.
    # Own a process group before spawning any children. The external UART/SDK
    # owner stays outside it; a stuck shutdown can reclaim this whole subtree.
    os.setsid()
    display.cancel_join_thread()
    profile=None
    if os.environ.get('RDK_LIDAR_MOTION_PROFILE'):
        import cProfile
        profile=cProfile.Profile(timer=time.thread_time);profile.enable()
    engine=DynamicFence(config['installation']);log=ProcessDebugLog(root)
    context=mp.get_context('spawn')
    geometry_parent,geometry_child=context.Pipe()
    geometry_process=context.Process(target=run_geometry_jobs,args=(geometry_child,),name='lidar-numerical-owner')
    geometry_backend=None
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
    policy_output=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) if sdk_policy else None
    if policy_output is not None:policy_output.setblocking(False)
    last_policy=(False,'measured_wheel_input_enabled');policy_error_reported=False
    native_adapter=None;native_connection=None;native_reader=None
    native_runtime=None;native_runtime_error=None
    pose_connection=None;pose_reader=None;pose_error=None
    native_state=dict(enabled=native_feedback is not None,connected=False,
                      decode_errors=0,motion_input_drops=0,error=None)
    native_updates=0
    published_native_updates=0
    latest_raw_pose={}
    competition_wheel_policy=None
    last_output_packet=None
    direct_output=dict(packets_sent=0,output_drops=0,last_packet=None)
    direct_socket=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) if control_output else None
    if direct_socket is not None:direct_socket.setblocking(False)
    direct_sender=LocalDatagramSender(direct_socket,control_output['address']) if direct_socket is not None else None

    def compute_geometry(*args,**kwargs):
        nonlocal geometry_backend
        geometry_parent.send((args,kwargs))
        while not geometry_parent.poll(.05):
            if stop.is_set() or not geometry_process.is_alive():
                raise RuntimeError('lidar numerical process stopped; motion propagation continues')
        success,result,status=geometry_parent.recv()
        geometry_backend=status
        if not success:raise ValueError(result)
        return result

    engine.geometry=compute_geometry

    def event(name,**data):
        log.submit('lidar_worker_events.jsonl',dict(event=name,timestamp_monotonic_ns=time.monotonic_ns(),**data))

    def reply(kind,**data):
        replies.put_nowait(dict(kind=kind,**data))

    def native_send(kind,payload):
        nonlocal native_updates
        # Already in the motion process: do not serialize, send and receive
        # these measurements a second time through its own local socket.
        if kind=='wheel':engine.ingest_wheel(payload);counters['measured_wheel_ingested']+=1
        elif kind=='yaw':
            engine.ingest_yaw(payload['time_ns'],payload['yaw_deg'],payload['source'])
            counters['primary_yaw_ingested']+=1
        elif kind=='invalidate':engine.invalidate_reference(payload['reason'])
        else:raise ValueError('unsupported native motion input')
        # The input must never wait for a GUI/control consumer's mailbox lock.
        # The original publication owner observes this revision on its next
        # input/idle wake, publishing latest state without replaying samples.
        native_updates+=1
        return True

    def receive_native():
        nonlocal native_connection,native_reader
        try:
            if native_connection is None:
                if not readable_nowait(native_feedback.fileno()):return False
                native_connection,_=native_feedback.accept();native_connection.setblocking(False)
                native_connection.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,262144)
                native_reader=LocalPacketReceiver(native_connection,179)
                native_state['connected']=True
            packet=native_reader.recv()
            if not packet:
                native_connection.close();native_connection=None;native_state['connected']=False
                return False
            native_adapter.accept(packet)
            return True
        except (BlockingIOError,socket.timeout):return False
        except (ValueError,KeyError,TypeError,struct.error) as exc:
            native_state['decode_errors']+=1;native_state['error']=str(exc)
            event('native_motion_feedback_decode_error',reason=str(exc));return False
        except OSError as exc:
            native_state['error']=str(exc);event('native_motion_feedback_error',reason=str(exc))
            if native_connection is not None:native_connection.close();native_connection=None
            native_state['connected']=False
            return False

    def receive_pose():
        nonlocal latest_raw_pose,native_updates,pose_connection,pose_reader,pose_error
        try:
            if pose_connection is None:
                if not readable_nowait(pose_input['socket'].fileno()):return False
                pose_connection,_=pose_input['socket'].accept();pose_connection.setblocking(False)
                pose_connection.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,262144)
                pose_reader=LocalPacketReceiver(pose_connection,16384)
            data=pose_reader.recv()
            if not data:
                pose_connection.close();pose_connection=None;return False
            pose=json.loads(data)
            if pose['session_id']!=pose_input['session_id']:return False
            latest_raw_pose=pose
            engine.ingest_pose(pose);counters['pose_ingested']+=1
            native_updates+=1
            log.submit('pose_motion.jsonl',pose)
            return True
        except (BlockingIOError,socket.timeout):return False
        except (ValueError,KeyError,TypeError) as exc:
            pose_error=str(exc);event('pose_input_error',reason=str(exc));return False
        except OSError as exc:
            pose_error=str(exc);event('pose_input_error',reason=str(exc))
            if pose_connection is not None:pose_connection.close();pose_connection=None
            return False

    def restore_wheel_policy():
        if competition_wheel_policy is not None:
            engine.motion.set_wheel_suppressed(competition_wheel_policy['suppressed'],
                                               competition_wheel_policy['reason'])

    def receive_wheel_policy():
        nonlocal competition_wheel_policy
        if wheel_policy_input is None or native_runtime is not None:return False
        changed=False
        for _ in range(16):
            try:
                data=wheel_policy_input['socket'].recv(4096)
                packet=json.loads(data)
                if (not isinstance(packet,dict) or packet.get('schema')!=1 or packet.get('kind')!='competition_wheel_policy'
                        or packet.get('session_id')!=wheel_policy_input['session_id']
                        or not isinstance(packet.get('suppressed'),bool)):
                    continue
                packet.setdefault('reason','safe_zone_low_speed')
                if not isinstance(packet['reason'],str):continue
                competition_wheel_policy=packet
                with engine.lock:restore_wheel_policy()
                event('competition_wheel_policy_applied',**packet)
                changed=True
            except BlockingIOError:break
            except (OSError,ValueError,TypeError,KeyError) as exc:
                event('competition_wheel_policy_error',reason=str(exc))
        return changed

    def publish_sdk_policy():
        nonlocal last_policy,policy_error_reported
        if native_runtime is not None:return  # The same native motion owner publishes source decisions.
        if policy_output is None:return
        with engine.lock:
            pause=engine.motion.wheel_input_paused();reason=engine.motion.wheel_input_reason()
            requested=(pause,reason)
            if requested==last_policy:return
            evidence=engine.motion.wall_evidence
        packet=dict(schema=3,kind='wheel_source_policy',session_id=sdk_policy['session_id'],
            pause=pause,reason=reason,wall_motion_evidence=evidence,dependency_persists_after_pause=True)
        try:
            policy_output.sendto(json.dumps(packet,allow_nan=False).encode(),sdk_policy['address'])
            last_policy=requested;policy_error_reported=False
            event('wheel_source_policy_requested',**packet,sdk_application_confirmed=False)
        except OSError as exc:
            if not policy_error_reported:
                event('wheel_source_policy_transport_error',reason=str(exc),pause_requested=pause)
                policy_error_reported=True
            # Next real motion/fit change may retry; no periodic query/ACK,
            # data-age gate, MCU handshake, motor STOP or blocking wait.

    def consume(first):
        nonlocal reference_request_id,requested_lidar_enabled,latest_raw_pose
        changed=False
        for _ in range(16):
            if first is not None:
                kind,payload=first;first=None
            else:
                try:kind,payload=inbox.get_nowait()
                except queue.Empty:break
            changed=True
            if native_runtime is not None and kind in ('pose','yaw','wheel'):
                native_runtime.command(kind,payload)
                continue
            if kind=='pose':
                latest_raw_pose=payload
                engine.ingest_pose(payload);counters['pose_ingested']+=1
            elif kind=='yaw':
                engine.ingest_yaw(payload['time_ns'],payload['yaw_deg'],payload['source'])
                counters['primary_yaw_ingested']+=1
            elif kind=='wheel':engine.ingest_wheel(payload);counters['measured_wheel_ingested']+=1
            elif kind=='feedback_context' and native_adapter is not None:
                native_adapter.context(payload)
            elif kind=='feedback_context' and native_runtime is not None:
                engine.context(payload)
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
            if native_runtime is None:restore_wheel_policy()
            while engine.motion.events:
                change=engine.motion.events.popleft();name=change.pop('event');event(name,**change)
        return changed

    def publish_control():
        nonlocal last_control_key,last_output_packet
        if native_runtime is not None:return  # Never create a second control producer in Python.
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
        if direct_socket is not None:
            packet=format_control_packet(pose,latest_raw_pose,control_output['session_id'],
                direct_output['packets_sent'],last_output_packet,lidar_state=value)
            if pose.get('ready'):last_output_packet=packet
            if packet is not None:
                try:
                    direct_sender.send(json.dumps(packet,allow_nan=False).encode())
                    direct_output['packets_sent']+=1;direct_output['last_packet']=packet
                    log.submit('control_pose.jsonl',packet)
                except (BlockingIOError,FileNotFoundError,ConnectionRefusedError):
                    direct_output['output_drops']+=1
            # Native control has already reached its sole consumer. UI state
            # uses the existing 20Hz display snapshot; Event.notify handshakes
            # must not sit on the 100Hz control path.
            counters['control_updates']+=1
            last_control_key=key
            return
        try:
            coalesced=control.put_latest(value)
            if coalesced is not None:
                counters['control_updates_coalesced']+=coalesced
                counters['control_updates']+=1
            last_control_key=key
        except ValueError as exc:
            counters['control_capacity_errors']+=1
            event('control_snapshot_capacity_error',reason=str(exc))

    def state():
        native=engine.refresh_native() if native_runtime is not None else {}
        observed_counters=dict(counters)
        for key in ('pose_ingested','primary_yaw_ingested','measured_wheel_ingested','control_updates'):
            if key in native:observed_counters[key]=native[key]
        native_logs=native.get('logging',{})
        native_logs_healthy=all(not value.get('dropped') and not value.get('failed') for value in native_logs.values())
        last=(engine.snapshot(_native_state=native) if native_runtime is not None else engine.snapshot())
        return dict(observed_counters,**reader_state,scans=engine.scan_count,fits=engine.fit_count,
            scan_processing_ms_p95=p95,last=last,error=error,
            worker_pid=os.getpid(),worker_cpu_affinity=sorted(os.sched_getaffinity(0)),
            capture_chain_present=bool(engine.poses and engine.yaws and reader_state['scans_received']),
            have_tracked_pose=bool(engine.poses),have_primary_yaw=bool(engine.yaws),
            lidar_requested_enabled=requested_lidar_enabled,
            lidar_corrections_enabled=engine.lidar_corrections_enabled,
            logging_healthy=log.healthy() and native_logs_healthy,motion_scheduling=native.get('motion_scheduling',scheduling),line_geometry=geometry_backend or backend_status(),
            geometry_pid=geometry_process.pid,geometry_execution='independent_process_original_algorithm',
            native_feedback=dict(native_state,**(native.get('native_feedback',{}) if native_runtime is not None else
                (native_adapter.snapshot() if native_adapter else {}))),
            direct_output=native.get('direct_output') if native_runtime is not None else (dict(direct_output) if direct_socket is not None else None),
            pose_input_error=native.get('pose_input_error',pose_error),native_motion_error=native.get('error',native_runtime_error),
            motion_execution='native_cpp_owner' if native_runtime is not None else 'python_original_fallback',
            native_logging=native_logs,
            accuracy_verified=False,full_speed_physical_validation=False)

    def drain_reader():
        nonlocal reader_state
        dirty=False
        while readable_nowait(updates._reader.fileno()):
            try:value=updates.get_nowait()
            except queue.Empty:break
            changed=(value['lidar_connected']!=reader_state['lidar_connected'] or
                     value['lidar_error']!=reader_state['lidar_error'])
            reader_state=value
            if native_runtime is not None:
                native_runtime.command('lidar_state',dict(connected=value['lidar_connected'],error=value['lidar_error']))
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
                    have_reference=engine.reference is not None
                if initializing:
                    result=engine.try_initialize(scan,expected_revision=revision)
                else:
                    if not have_reference:continue
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
        geometry_process.start();geometry_child.close()
        if native_feedback is not None and pose_input is not None:
            try:
                def address(value):return '@'+value[1:] if value.startswith('\0') else value
                native_runtime=NativeMotionRuntime(dict(session_id=(control_output['session_id'] if control_output else pose_input['session_id']),
                    pose_session_id=pose_input['session_id'],
                    sdk_policy_session_id=sdk_policy['session_id'] if sdk_policy else pose_input['session_id'],
                    feedback_fd=native_feedback.fileno(),pose_fd=pose_input['socket'].fileno(),
                    control_address=address(control_output['address']) if control_output else '',
                    sdk_policy_address=address(sdk_policy['address']) if sdk_policy else '',
                    wheel_policy_fd=wheel_policy_input['socket'].fileno() if wheel_policy_input else -1,
                    log_root=str(root)))
                engine=NativeDynamicFence(config['installation'],native_runtime);engine.geometry=compute_geometry
            except Exception as exc:
                if native_runtime is not None:native_runtime.close()
                native_runtime=None;native_runtime_error=str(exc)
                engine=DynamicFence(config['installation']);engine.geometry=compute_geometry
                event('native_motion_start_failed_python_fallback',reason=str(exc),latency_validated=False)
        if native_feedback is not None and native_runtime is None:
            native_feedback.setblocking(False)
            native_adapter=NativeFeedbackAdapter(native_send,event,log)
        if pose_input is not None:pose_input['socket'].setblocking(False)
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
            try:first=inbox.get_nowait()
            except queue.Empty:first=None
            changed=consume(first)
            changed=receive_wheel_policy() or changed
            # Drain real queued inputs fairly, publishing actual endpoints
            # between them. No replay, stale-data rejection or synthetic ticks.
            # Housekeeping runs after a bounded quantum instead of after every
            # individual wire frame (there are two frames per wheel interval).
            for _ in range(16 if native_runtime is None else 0):
                incoming=False
                if native_feedback is not None:incoming=receive_native() or incoming
                if pose_input is not None:incoming=receive_pose() or incoming
                if wheel_policy_input is not None:
                    # Pose/session invalidation may replace the original
                    # Python fallback source; retain the current phase policy.
                    with engine.lock:restore_wheel_policy()
                if not incoming:break
                changed=True
                if control is not None:publish_control()
                publish_sdk_policy()
            if native_updates!=published_native_updates:
                published_native_updates=native_updates
                changed=True
            # Publish actual motion before probing the reader/scan/fit queues.
            # Those housekeeping calls may yield the GIL even when empty.
            # A completed posterior still gets its own update below; the
            # bounded/latest control mailbox never replays an input backlog.
            if changed:
                if control is not None:publish_control()
                publish_sdk_policy()
            housekeeping_changed=drain_reader()
            latest=None
            for _ in range(8):
                if not readable_nowait(scans._reader.fileno()):break
                try:incoming_scan=scans.get_nowait()
                except queue.Empty:break
                if latest is not None:counters['fit_requests_coalesced']+=1
                latest=incoming_scan
            if latest is not None:
                counters['fit_requests_coalesced']+=offer_latest(fit_inbox,
                    (latest,engine.revision,reference_request_id))
            housekeeping_changed=drain_fits() or housekeeping_changed
            if housekeeping_changed:
                if control is not None:publish_control()
                publish_sdk_policy()
            if time.monotonic()-last_publish>=.05:
                if times and time.monotonic()-last_percentile>=1.:
                    p95=float(np.percentile(times,95));last_percentile=time.monotonic()
                counters['display_updates_coalesced']+=offer_latest(display,state())
                last_publish=time.monotonic()
            if not changed and not housekeeping_changed and latest is None:
                waiting=[inbox._reader,updates._reader,scans._reader]
                if native_feedback is not None and native_runtime is None:waiting.append(native_connection or native_feedback)
                if pose_input is not None and native_runtime is None:waiting.append(pose_connection or pose_input['socket'])
                if wheel_policy_input is not None and native_runtime is None:waiting.append(wheel_policy_input['socket'])
                select.select(waiting,[],[],.002)
    except Exception as exc:
        error=str(exc);event('worker_error',reason=error)
        try:reply('error',error=error)
        except queue.Full:pass
    finally:
        if native_connection is not None:native_connection.close()
        if pose_connection is not None:pose_connection.close()
        native_state['connected']=False
        if pose_input is not None:pose_input['socket'].close()
        if native_feedback is not None:native_feedback.close()
        if wheel_policy_input is not None:wheel_policy_input['socket'].close()
        if policy_output is not None:policy_output.close()
        if direct_socket is not None:direct_socket.close()
        stop.set();fitting_stop.set()
        if fitter.ident is not None:fitter.join(timeout=2)
        if geometry_process.pid is not None:
            if fitter.is_alive():
                if geometry_process.is_alive():geometry_process.terminate()
                geometry_process.join(timeout=1)
                fitter.join(timeout=1)
            elif geometry_process.is_alive():
                try:geometry_parent.send(None)
                except (EOFError,OSError,BrokenPipeError):pass
            geometry_process.join(timeout=2)
            if geometry_process.is_alive():geometry_process.terminate();geometry_process.join(timeout=1)
        geometry_parent.close()
        drain_fits()
        if receiver.pid is not None:
            deadline=time.monotonic()+2
            while receiver.is_alive() and time.monotonic()<deadline:
                drain_reader();receiver.join(timeout=.02)
            drain_reader()
            if receiver.is_alive():receiver.terminate();receiver.join(timeout=1)
        if native_runtime is not None:
            native_runtime.close()
            engine.refresh_native()
            while engine.motion.events:
                change=engine.motion.events.popleft();name=change.pop('event');event(name,**change)
        if times:p95=float(np.percentile(times,95))
        final=state()
        final['receiver_stopped_cleanly']=receiver.exitcode==0
        final['fit_thread_stopped']=not fitter.is_alive()
        final['geometry_stopped_cleanly']=geometry_process.exitcode==0
        log.submit('lidar_worker_summary.json',final,atomic=True);log.close()
        final['logging_healthy']=final['logging_healthy'] and log.healthy() and reader_state.get('reader_logging_healthy',False)
        final['logging_statistics']=log.statistics()
        try:reply('stopped',summary=final)
        except queue.Full:pass
        for channel in (scans,updates):channel.cancel_join_thread();channel.close()
        if profile is not None:
            profile.disable();profile.dump_stats(os.environ['RDK_LIDAR_MOTION_PROFILE'])


class RadarWorker:
    def __init__(self,*,control_updates=False):
        self.context=mp.get_context('spawn');self.process=None
        self.inbox=MotionInbox();self.display=self.context.Queue(maxsize=4)
        self.replies=self.context.Queue(maxsize=16);self.stop=self.context.Event()
        self.control=LatestSnapshot(self.context) if control_updates else None
        self.state={};self.input_drops=0;self.pending_reference=None;self.stopped_cleanly=False
        self.scheduling_setup=None
        self.feedback_socket=None;self.feedback_address=None
        self.wheel_policy_input=None

    def enable_competition_wheel_policy(self,session_id):
        address='\0competition-wheel-policy-'+session_id
        channel=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
        channel.setblocking(False);channel.bind(address)
        self.wheel_policy_input=dict(socket=channel,session_id=session_id)
        return address

    def enable_native_feedback(self):
        # Bind before the sole C++ UART owner starts. No consumer-ready ACK.
        self.feedback_address='\0motion-feedback-'+uuid.uuid4().hex
        self.feedback_socket=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET)
        self.feedback_socket.bind(self.feedback_address);self.feedback_socket.listen(1)

    def start(self,config,port,baud,root,*,sdk_policy=None,control_output=None,pose_input=None):
        self.scheduling_setup=prepare_realtime()
        self.process=self.context.Process(target=run_worker,args=(config,port,baud,root,
            self.inbox,self.display,self.replies,self.stop,self.control,sdk_policy,self.feedback_socket,control_output,pose_input,self.wheel_policy_input),name='lidar-geometry-owner')
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
                self.stopped_cleanly=bool(self.state.get('receiver_stopped_cleanly') and self.state.get('fit_thread_stopped') and self.state.get('geometry_stopped_cleanly'))
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
                # The normal five-second drain has already expired. Killing
                # only the worker can leave its numerical/reader/log children
                # alive (including a suspended child). Reclaim only its owned
                # group, never the GUI, SDK/UART owner or caller's group.
                try:
                    if os.getpgid(self.process.pid)==self.process.pid:
                        os.killpg(self.process.pid,signal.SIGKILL)
                    else:self.process.kill()
                except ProcessLookupError:pass
                self.process.join(timeout=1)
                self.state['error']='雷达进程未能正常退出';self.stopped_cleanly=False
        for channel in (self.inbox,self.display,self.replies):
            channel.cancel_join_thread();channel.close()
        if self.feedback_socket is not None:self.feedback_socket.close()
        if self.wheel_policy_input is not None:self.wheel_policy_input['socket'].close()
