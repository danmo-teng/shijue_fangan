#!/usr/bin/env python3
"""YOLO26 X5 live benchmark. No UART, mission, or motor control.

Decode follows D-Robotics rdk_model_zoo/ultralytics_yolo26 (LTRB, no DFL).
Independent latest-frame capture, optional preparation worker, and GUI process; bounded buffers.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import queue
import signal
import threading
import time
import traceback
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent


from rescue_vision.yolo26 import X5Yolo26 as Detector


def display_process(pixels, boxes, info, guard, quit_event, labels, fps, measure_start, deadline, directory):
    """Separate GUI process avoids BPU Python binding/GIL stalls; fixed shared memory."""
    cv2.setNumThreads(1)
    tensor = np.frombuffer(pixels, dtype=np.uint8).reshape(960, 640)
    detections = np.frombuffer(boxes, dtype=np.float32).reshape(8400, 6)
    shown_id, count = -1, 0
    last_frame = None
    next_display = 0.0
    display_times = []
    try:
        cv2.namedWindow('YOLO26 X5 benchmark - Q quit', cv2.WINDOW_NORMAL)
        while not quit_event.is_set():
            now = time.monotonic()
            if now < next_display:
                quit_event.wait(min(.005, next_display - now))
                continue
            next_display = now + 1 / fps
            snapshot = None
            with guard:
                if info[0] != shown_id and info[0] >= 0:
                    shown_id = info[0]
                    snapshot = (tensor.copy(), detections[:int(info[1])].copy(), list(info))
            if snapshot is not None:
                nv12, dets, meta = snapshot
                frame = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
                for x1,y1,x2,y2,score,cls in dets:
                    x1,y1,x2,y2 = map(int,(x1,y1,x2,y2))
                    cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)
                    cv2.putText(frame,f'{labels[int(cls)]} {score:.2f}',(x1,max(15,y1-4)),
                                cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1)
                cv2.putText(frame,f'Unique FPS {meta[4]:.0f} | BPU {meta[2]:.1f}ms | age {(now-meta[3])*1000:.1f}ms',
                            (8,23),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1)
                cv2.imshow('YOLO26 X5 benchmark - Q quit',frame)
                last_frame = frame
                if measure_start <= now < deadline:
                    count += 1
                    display_times.append(now - measure_start)
            if cv2.waitKey(1) & 0xff in (27,ord('q'),ord('Q')):
                quit_event.set()
    except Exception:
        Path(directory, 'display_error.txt').write_text(traceback.format_exc())
        quit_event.set()
    finally:
        if last_frame is not None:
            cv2.imwrite(str(Path(directory, 'last_preview.jpg')), last_frame)
        Path(directory, 'display.json').write_text(json.dumps({'frames':count, 'times_s':display_times}))
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description='YOLO26 640 X5真实新帧测速；不启动比赛、不发送串口')
    parser.add_argument('--model', type=Path, default=ROOT.parent / 'yolo26n/x5_output_640/yolo26n_6cls_640_bpu_bayese_640x640_nv12.bin')
    parser.add_argument('--labels', type=Path, default=ROOT / 'config/yolo_labels.txt')
    parser.add_argument('--device', default='/dev/video0')
    parser.add_argument('--camera-fps', type=int, default=180)
    parser.add_argument('--decode-fps', type=float, default=60)
    parser.add_argument('--pipeline', choices=('serial', 'threaded'), default='serial',
                        help='serial: one compute worker + independent capture/GUI; threaded: extra preprocessing worker')
    parser.add_argument('--preprocess', choices=('auto', 'vse', 'cpu'), default='auto')
    parser.add_argument('--score-thres', type=float, default=.2)
    parser.add_argument('--nms-thres', type=float, default=.45)
    parser.add_argument('--duration', type=float, default=60, help='测量秒数，不含预热；0运行到Q退出')
    parser.add_argument('--warmup', type=float, default=5)
    parser.add_argument('--display-fps', type=float, default=40)
    parser.add_argument('--no-display', action='store_true')
    parser.add_argument('--replay-dir', type=Path, help='仅离线图片回放，不代表相机实时帧率')
    parser.add_argument('--output', type=Path, default=ROOT / 'captures/yolo26_benchmark')
    args = parser.parse_args()
    if not (0 < args.score_thres < 1 and 0 < args.nms_thres < 1 and args.duration >= 0 and
            args.warmup >= 0 and args.decode_fps > 0 and args.display_fps > 0):
        parser.error('invalid threshold/timing')
    cv2.setNumThreads(1)  # Avoid competing OpenCV thread pools on Cortex-A55.
    from run_yolo_x5 import letterbox, bgr_to_nv12, nv12_letterbox
    from rescue_vision.vse import VseScaler
    detector = Detector(args.model, args.labels, args.score_thres, args.nms_thres)
    directory = args.output / (time.strftime('%Y%m%d_%H%M%S') + f'_{time.time_ns() % 1000000:06d}_{args.pipeline}')
    directory.mkdir(parents=True)
    scaler = camera = None
    images = []
    if args.replay_dir:
        for path in sorted(args.replay_dir.rglob('*.jpg'))[:16]:
            image = cv2.imread(str(path))
            if image is not None:
                images.append(image)
        if not images:
            raise ValueError('No replay JPG images')
        preprocess = 'cpu_replay'
    else:
        from rescue_vision.camera import LatestFrameCamera, resolve_camera_device
        preprocess = 'cpu'
        if args.preprocess != 'cpu':
            try:
                scaler = VseScaler(1280, 1024, 640, 512)
                preprocess = 'vse'
            except Exception as error:
                if args.preprocess == 'vse':
                    raise
                print(f'VSE unavailable, CPU fallback: {error}', flush=True)
        camera = LatestFrameCamera(resolve_camera_device(args.device), 1280, 1024, args.camera_fps,
                                   decoder='jpu', decode_fps=args.decode_fps,
                                   output_format='nv12' if scaler else 'bgr')
    metadata = {**vars(args), 'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), 'model_sha256': hashlib.sha256(args.model.read_bytes()).hexdigest(),
                'source': 'replay' if images else 'live_camera', 'preprocess_actual': preprocess,
                'input_shape': [1,3,640,640], 'output_shapes': detector.shapes,
                'labels': detector.labels, 'note': 'class order must match training; default uses repository six classes'}
    (directory / 'metadata.json').write_text(json.dumps(metadata, default=str, indent=2))
    print(f'Source={metadata["source"]}, pipeline={args.pipeline}, preprocess={preprocess}, threshold={args.score_thres}', flush=True)
    print(f'Logs: {directory}', flush=True)
    stop = threading.Event()
    prepared = queue.Queue(1)
    lock = threading.Lock()
    rows, errors = [], []
    drops = {'prepared': 0}
    gui = None
    gui_stop = None
    gui_buffers = None
    threads = []
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    start = time.monotonic()
    measure_start = start + args.warmup
    deadline = measure_start + args.duration if args.duration else math.inf

    def put_latest(q, value, key):
        while not stop.is_set():
            try:
                q.put_nowait(value)
                return
            except queue.Full:
                try:
                    q.get_nowait()
                    drops[key] += 1
                except queue.Empty:
                    pass

    def receive(q):
        while not stop.is_set():
            try:
                return q.get(timeout=.1)
            except queue.Empty:
                pass
        return None

    def finish(job, outputs):
        t = time.monotonic()
        detections = detector.decode(outputs)
        now = time.monotonic()
        job['postprocess_ms'] = (now - t) * 1000
        job['latency_ms'] = (now - job['capture_s']) * 1000
        job['completed_s'] = now
        job['detections'] = len(detections)
        with lock:
            if measure_start <= now < deadline:
                rows.append({k: v for k, v in job.items() if k != 'tensor'})
            recent_fps = sum(r['completed_s'] >= now - 1 for r in rows[-300:])
        if gui_buffers is not None:
            pixels, boxes, info, guard = gui_buffers
            if guard.acquire(False):
                try:
                    np.frombuffer(pixels, dtype=np.uint8)[:] = job['tensor'].reshape(-1)
                    np.frombuffer(boxes, dtype=np.float32).reshape(8400,6)[:len(detections)] = detections
                    info[:] = [job['frame_id'], len(detections), job['inference_ms'], job['capture_s'], recent_fps]
                finally:
                    guard.release()

    def producer():
        last_id = -1
        replay_id = 0
        next_replay = 0.0
        last_fresh = time.monotonic()
        while not stop.is_set():
            if camera:
                error = camera.check_error()
                if error:
                    raise RuntimeError(error)
                packet = camera.latest()
                if packet is None or packet.frame_id == last_id:
                    if time.monotonic() - last_fresh > 5:
                        raise RuntimeError('No new decoded camera frame for 5 seconds')
                    stop.wait(.001)
                    continue
                last_id = packet.frame_id
                image, capture = packet.image, packet.published_ns / 1e9
            else:
                now = time.monotonic()
                if now < next_replay:
                    stop.wait(next_replay - now)
                    continue
                next_replay = now + 1 / args.decode_fps
                image = images[replay_id % len(images)]
                replay_id += 1
                last_id, capture = replay_id, time.monotonic()
            last_fresh = time.monotonic()
            t = time.monotonic()
            if scaler:
                tensor, _, _ = nv12_letterbox(scaler.scale(image), 640, 512, 640, 640)
            else:
                canvas, _, _, _ = letterbox(image, 640, 640)
                tensor = bgr_to_nv12(canvas)
            job = {'frame_id': last_id, 'capture_s': capture, 'tensor': tensor,
                   'preprocess_ms': (time.monotonic() - t) * 1000}
            if args.pipeline == 'serial':
                t = time.monotonic()
                outputs = detector.run(tensor)
                job['inference_ms'] = (time.monotonic() - t) * 1000
                finish(job, outputs)
            else:
                put_latest(prepared, job, 'prepared')

    def inference():
        while not stop.is_set():
            job = receive(prepared)
            if job is None:
                return
            t = time.monotonic()
            outputs = detector.run(job['tensor'])
            job['inference_ms'] = (time.monotonic() - t) * 1000
            # Keep cheap postprocessing with its BPU worker: another Python
            # postprocess thread increased GIL/scheduling contention on this X5.
            finish(job, outputs)

    def guarded(function):
        try:
            function()
        except Exception:
            errors.append(traceback.format_exc())
            stop.set()

    last_report = start
    try:
        if camera:
            camera.start()
        start = time.monotonic()
        measure_start = start + args.warmup
        deadline = measure_start + args.duration if args.duration else math.inf
        if not args.no_display:
            import multiprocessing as mp
            context = mp.get_context('spawn')
            pixels = context.RawArray('B', 960*640)
            boxes = context.RawArray('f', 8400*6)
            info = context.RawArray('d', [-1,0,0,0,0])
            guard = context.Lock()
            gui_stop = context.Event()
            gui_buffers = (pixels, boxes, info, guard)
            gui = context.Process(target=display_process,
                args=(*gui_buffers, gui_stop, detector.labels, args.display_fps, measure_start, deadline, str(directory)))
            gui.start()
        for function in ([producer] if args.pipeline == 'serial' else [producer, inference]):
            thread = threading.Thread(target=guarded, args=(function,), daemon=True, name=function.__name__)
            threads.append(thread)
            thread.start()
        while not stop.is_set() and time.monotonic() < deadline:
            now = time.monotonic()
            with lock:
                count = len(rows)
                recent = sum(r['completed_s'] >= now - 1 for r in rows[-300:])
            if now - last_report >= 1:
                print(f'{"WARMUP" if now < measure_start else "MEASURE"} unique_fps_1s={recent} frames={count}', flush=True)
                last_report = now
            if gui is not None:
                if gui_stop.is_set():
                    break
                if not gui.is_alive():
                    raise RuntimeError(f'Display process exited unexpectedly: {gui.exitcode}')
            stop.wait(.002)
    except Exception:
        errors.append(traceback.format_exc())
    finally:
        end = min(time.monotonic(), deadline)
        stop.set()
        for thread in threads:
            thread.join(5)
        if any(t.is_alive() for t in threads):
            errors.append('Worker did not exit within 5 seconds')
        if camera:
            camera.stop()
        if scaler and not any(t.is_alive() for t in threads):
            scaler.close()
        if gui is not None:
            gui_stop.set()
            gui.join(5)
            if gui.is_alive():
                gui.terminate()
                gui.join()
                errors.append('Display process did not exit')
            if (directory/'display_error.txt').exists():
                errors.append((directory/'display_error.txt').read_text())
    elapsed = max(0.0, end - measure_start)
    rows = [r for r in rows if r['completed_s'] < end]
    complete_seconds = int(elapsed)
    windows = [sum(i <= r['completed_s'] - measure_start < i+1 for r in rows) for i in range(complete_seconds)]
    display_stats = json.loads((directory/'display.json').read_text()) if (directory/'display.json').exists() else {'frames':0}
    summary = {'source':metadata['source'], 'pipeline':args.pipeline, 'preprocess':preprocess,
               'seconds':elapsed, 'unique_results':len(rows), 'mean_fps':len(rows)/elapsed if elapsed else 0,
               'fps_1s':windows, 'min_fps_1s':min(windows) if windows else None,
               'displayed_new_frames':display_stats['frames'], 'queue_drops_including_warmup':drops,
               'camera_captured': camera.captured_count() if camera else None,
               'camera_decoded': camera.decoded_count() if camera else None, 'errors':errors}
    for key in ('preprocess_ms','inference_ms','postprocess_ms','latency_ms'):
        summary[key] = dict(zip(('p50','p95','p99'), map(float,np.percentile([r[key] for r in rows],[50,95,99])))) if rows else None
    summary['mean_detections'] = float(np.mean([r['detections'] for r in rows])) if rows else 0
    summary['display_fps'] = display_stats['frames'] / elapsed if elapsed else 0
    display_windows = [sum(i <= t < i+1 for t in display_stats.get('times_s', [])) for i in range(complete_seconds)]
    summary['display_min_fps_1s'] = min(display_windows) if display_windows and not args.no_display else None
    summary['max_result_gap_ms'] = float(np.max(np.diff([r['completed_s'] for r in rows])) * 1000) if len(rows) > 1 else None
    summary['stable_30fps'] = bool(not errors and elapsed >= 30 and windows and min(windows) >= 30)
    summary['live_30fps_verified'] = summary['stable_30fps'] and not images
    (directory/'summary.json').write_text(json.dumps(summary,indent=2))
    if rows:
        with (directory/'frames.csv').open('w') as file:
            writer=csv.DictWriter(file,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    print(json.dumps(summary,indent=2),flush=True)
    if errors:
        raise RuntimeError(errors[0])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
