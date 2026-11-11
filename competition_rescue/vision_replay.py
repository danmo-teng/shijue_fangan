"""Bounded incident video from existing YOLO frames; never owns a camera/control.

Video FPS is a playback rate, not a claim of uniformly sampled sensor frames.
Each encoded frame has its original capture timestamp and state snapshot in JSONL.
"""
from collections import deque
import json
import math
import multiprocessing as mp
from pathlib import Path
import queue
import threading
import time


def _finite(value):
    if isinstance(value,float) and not math.isfinite(value): return None
    if isinstance(value,dict): return {k:_finite(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)): return [_finite(v) for v in value]
    return value


def _write_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False) + '\n')
    temporary.replace(path)


def replay_worker(root, session_id, inbox, stop, dropped, fps, pre_s, post_s, size,
                  native_frames=0):
    import cv2
    import numpy as np
    import signal
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # Parent closes/drains the recorder.
    cv2.setNumThreads(1)
    root = Path(root)
    manifest = dict(schema_version=1, sdk_session_id=session_id, complete=False,
        source='actual_YOLO_processed_camera_frames', playback_fps=fps,
        pre_seconds=pre_s, post_seconds=post_s, image_size=list(size),
        native_frames_per_clip=native_frames, native_complete=True, native_errors=[],
        pose_alignment='latest planner snapshot at enqueue, not capture-synchronized ground truth',
        clips=[], dropped_frames=0, error=None)
    manifest_path = root / 'vision_replay_manifest.json'
    ring = deque(maxlen=math.ceil(pre_s * fps) + 2)
    writer = frames = clip = None
    clip_end_ns = 0

    def persist():
        manifest['dropped_frames'] = dropped.value
        _write_json(manifest_path, manifest)

    def finish(reason):
        nonlocal writer, frames, clip
        if writer is None:
            return
        writer.release(); frames.close()
        check = cv2.VideoCapture(str(root/clip['video']))
        count = int(check.get(cv2.CAP_PROP_FRAME_COUNT)) if check.isOpened() else -1
        check.release()
        if count != clip['frames_written']:
            raise RuntimeError('replay video index/frame count mismatch after close')
        clip.update(complete=True, end_reason=reason)
        writer = frames = clip = None
        persist()

    def write_frame(bgr, meta):
        # Separate header strip preserves all of the camera image underneath.
        width, height = size
        view = np.zeros((height + 96, width, 3), dtype=np.uint8)
        view[96:] = bgr
        source_width, source_height = meta['source_size']
        sx, sy = width / source_width, height / source_height
        for d in meta.get('detections', []):
            x,y,w,h = d['bbox']
            color = (255,80,255) if d['class'].startswith('safe_') else (0,220,255)
            a,b = (round(x*sx),96+round(y*sy)), (round((x+w)*sx),96+round((y+h)*sy))
            cv2.rectangle(view,a,b,color,1)
            cv2.putText(view,f"{d['class']} {d['confidence']:.2f}",(a[0],max(108,a[1])),
                        cv2.FONT_HERSHEY_SIMPLEX,.42,color,1,cv2.LINE_AA)
        for polygon in meta.get('roi_polygons', []):
            if polygon:
                pts = np.array([(round(x*sx),96+round(y*sy)) for x,y in polygon],dtype=np.int32)
                cv2.polylines(view,[pts],True,(80,200,80),1)
        pose, stm = meta.get('pose',{}), meta.get('stm',{})
        lines = [f"Frame {meta['frame_sequence']}  capture {meta['capture_monotonic_ns']/1e9:.3f}s  {meta.get('upper_state','')}",
            f"XYZ {pose.get('x_m',0):+.3f} {pose.get('y_m',0):+.3f}  yaw {pose.get('yaw_deg',0):.1f}  camera {stm.get('camera_pitch_cdeg',0)/100:.1f} deg",
            f"STM {stm.get('mode','?')}  task/action {stm.get('task_id','?')}/{stm.get('action_id','?')}  {meta.get('reason','')}"]
        for i,line in enumerate(lines):
            cv2.putText(view,line,(8,24+30*i),cv2.FONT_HERSHEY_SIMPLEX,.48,(235,235,235),1,cv2.LINE_AA)
        writer.write(view)
        row = dict(meta, video_frame_index=clip['frames_written'],
            seconds_from_trigger=(meta['capture_monotonic_ns']-clip['trigger_monotonic_ns'])/1e9)
        frames.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
        if clip['first_capture_ns'] is None:
            clip['first_capture_ns'] = meta['capture_monotonic_ns']
        clip['frames_written'] += 1
        clip['last_capture_ns'] = meta['capture_monotonic_ns']

    def write_native(image, pixel_format, meta, event):
        # The original array already crossed the existing bounded replay IPC.
        # Only the first available image carrying an event is eligible, never
        # a resized/annotated prebuffer image or an invented trigger-frame match.
        if native_frames == 0:
            return
        saved = clip['native_frames']
        identity = (meta['frame_sequence'], meta['capture_monotonic_ns'])
        if (len(saved) >= native_frames or any(
                (entry['frame_sequence'], entry['capture_monotonic_ns']) == identity
                for entry in saved)):
            return
        stem = Path(clip['video']).stem + f'_native_{len(saved) + 1:02d}'
        entry = dict(data=stem+'.npy', metadata=stem+'.json', status='pending',
            frame_sequence=meta['frame_sequence'],
            capture_monotonic_ns=meta['capture_monotonic_ns'])
        # Count attempted frames too: a write error must not cause retries or
        # unbounded files on later events in this clip.
        saved.append(entry)
        try:
            np.save(root/entry['data'], image, allow_pickle=False)
            _write_json(root/entry['metadata'], dict(schema_version=1,
                sdk_session_id=session_id, data=entry['data'],
                provenance='original_decoded_camera_array_from_existing_YOLO_result',
                model_preprocessed_tensor=False, resized=False, annotated=False,
                pixel_format=pixel_format, shape=list(image.shape), dtype=str(image.dtype),
                data_nbytes=int(image.nbytes), source_size=meta['source_size'],
                frame_sequence=meta['frame_sequence'],
                camera_frame_id=meta.get('camera_frame_id'),
                capture_monotonic_ns=meta['capture_monotonic_ns'],
                snapshot_monotonic_ns=meta.get('snapshot_monotonic_ns'),
                event=event['event'], trigger_monotonic_ns=event['trigger_monotonic_ns'],
                event_frame_relation='first_available_enqueued_image_carrying_event; not exact trigger frame'))
            entry['status'] = 'saved'
        except Exception as error:
            entry.update(status='error', error=str(error))
            manifest['native_complete'] = False
            manifest['native_errors'].append(dict(video=clip['video'],
                frame_sequence=meta['frame_sequence'], error=str(error)))

    try:
        persist()
        while True:
            try:
                image, pixel_format, meta, triggers = inbox.get(timeout=.05)
            except queue.Empty:
                if stop.is_set(): break
                continue
            meta = _finite(meta)
            stamp = meta['capture_monotonic_ns']
            bgr = cv2.cvtColor(image,cv2.COLOR_YUV2BGR_NV12) if pixel_format=='nv12' else image
            bgr = cv2.resize(bgr,size,interpolation=cv2.INTER_AREA)
            while ring and stamp-ring[0][0]['capture_monotonic_ns'] > pre_s*1e9:
                ring.popleft()
            # Planner events survive skipped/full frame queues. Their original
            # occurrence time anchors the clip even if the next frame has a
            # different state. Overlapping incidents extend the same tail.
            for event in _finite(triggers):
                event_ns = event['trigger_monotonic_ns']
                if writer is not None and event_ns > clip_end_ns:
                    finish('post_window_finished')
                if writer is None:
                    number = len(manifest['clips']) + 1
                    stem = f'safe_zone_replay_{number:04d}'
                    clip = dict(video=stem+'.avi', frames=stem+'.jsonl', complete=False,
                        trigger=event['event'], trigger_monotonic_ns=event_ns,
                        events=[event], frames_written=0, first_capture_ns=None,
                        native_frames=[])
                    manifest['clips'].append(clip); persist()
                    writer = cv2.VideoWriter(str(root/clip['video']),cv2.VideoWriter_fourcc(*'MJPG'),fps,(size[0],size[1]+96))
                    if not writer.isOpened(): raise RuntimeError('MJPG video encoder unavailable')
                    frames = (root/clip['frames']).open('w',encoding='utf-8')
                    clip_end_ns = event_ns + round(post_s*1e9)
                    for old_meta,jpeg in ring:
                        if not event_ns-round(pre_s*1e9) <= old_meta['capture_monotonic_ns'] <= clip_end_ns:
                            continue
                        old = cv2.imdecode(jpeg,cv2.IMREAD_COLOR)
                        if old is None: raise RuntimeError('replay prebuffer decode failed')
                        write_frame(old,old_meta)
                else:
                    clip['events'].append(event)
                    clip_end_ns = max(clip_end_ns,event_ns+round(post_s*1e9))
                clip['post_window_end_ns'] = clip_end_ns
                write_native(image, pixel_format, meta, event)
                persist()
            if writer is not None and stamp > clip_end_ns:
                finish('post_window_finished')
            if writer is not None:
                write_frame(bgr,meta)
            ok, jpeg = cv2.imencode('.jpg',bgr,[cv2.IMWRITE_JPEG_QUALITY,85])
            if not ok: raise RuntimeError('replay prebuffer encoding failed')
            ring.append((meta,jpeg))
        finish('session_end')
        manifest['complete'] = dropped.value == 0
    except Exception as error:
        manifest['error'] = str(error)
    finally:
        if writer is not None: writer.release()
        if frames is not None: frames.close()
        try: persist()
        except OSError: pass  # Diagnostic failure never owns mission control.


class VisionReplay:
    def __init__(self, root, session_id, *, fps=8., pre_s=5., post_s=15., size=(960,768),
                 native_frames=0):
        if not isinstance(native_frames, int) or not 0 <= native_frames <= 3:
            raise ValueError('native_frames must be an integer in 0..3')
        self.root, self.session_id = Path(root), session_id
        self.context = mp.get_context('spawn')
        self.inbox = self.context.Queue(maxsize=4)
        self.stop = self.context.Event()
        self.dropped = self.context.Value('L',0,lock=False)  # Sole parent producer.
        self.process = self.context.Process(target=replay_worker,
            args=(self.root,session_id,self.inbox,self.stop,self.dropped,fps,pre_s,post_s,size,native_frames),
            name='competition-incident-replay',daemon=True)
        self.period_s = 1./fps
        self.next_frame_s = 0.
        self.event_lock = threading.Lock()
        self.pending_events = deque(maxlen=32)
        self.event_sequence = 0
        self.event_overflow_count = 0
        self.error = None
        self.started = False

    def start(self):
        try:
            self.process.start(); self.started = True
        except Exception as error:
            self.error = str(error)

    def due(self, now):
        return self.started and self.error is None and now >= self.next_frame_s

    def record_event(self, event):
        """Called by the planner, independently of image sampling or disk IO."""
        with self.event_lock:
            self.event_sequence += 1
            if len(self.pending_events) == self.pending_events.maxlen:
                self.event_overflow_count += 1
            self.pending_events.append(dict(event,event_id=self.event_sequence))

    def submit(self, image, pixel_format, metadata, now):
        if not self.due(now): return False
        self.next_frame_s = now + self.period_s
        if not self.process.is_alive():
            self.error = 'replay worker exited; see manifest'
            return False
        with self.event_lock:
            events = list(self.pending_events)
        try:
            self.inbox.put_nowait((image,pixel_format,metadata,events))
            if events:
                with self.event_lock:
                    while self.pending_events and self.pending_events[0]['event_id'] <= events[-1]['event_id']:
                        self.pending_events.popleft()
            return True
        except queue.Full:
            self.dropped.value += 1
            return False
        except (OSError,ValueError) as error:
            self.error = str(error)
            return False

    def request_close(self):
        self.stop.set()

    def close(self):
        self.request_close()
        if self.started:
            self.process.join(1.5)
            if self.process.is_alive():
                self.process.terminate();self.process.join(.5)
                self.error = 'replay close interrupted; files kept as incomplete'
        self.inbox.cancel_join_thread();self.inbox.close()
        native_error_count = None
        try:
            manifest = json.loads((self.root/'vision_replay_manifest.json').read_text())
            native_error_count = len(manifest.get('native_errors', []))
            self.error = self.error or manifest.get('error')
            with self.event_lock:
                pending_events = _finite(list(self.pending_events))
                overflow_count = self.event_overflow_count
            manifest.update(pending_events=pending_events,event_overflow_count=overflow_count)
            if pending_events or overflow_count:
                manifest['complete'] = False
            _write_json(self.root/'vision_replay_manifest.json',manifest)
        except (OSError,ValueError):
            self.error = self.error or 'replay manifest unavailable'
        return dict(dropped_frames=self.dropped.value,error=self.error,
                    native_error_count=native_error_count,
                    pending_events=len(self.pending_events),event_overflow_count=self.event_overflow_count,
                    manifest=str(self.root/'vision_replay_manifest.json'))
