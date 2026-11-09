#!/usr/bin/env python3
"""Live model comparison using the competition camera, worker and vision functions.

This entry owns no UART, planner, localization or competition command file.
"""
from __future__ import annotations

import argparse
from collections import Counter, deque
from dataclasses import asdict
import csv
import hashlib
import json
import multiprocessing as mp
import pickle
from pathlib import Path
import signal
import sys
import time
import traceback

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = ROOT.parent
sys.path.insert(0, str(PROJECT_ROOT / 'competition_rescue'))

# Import functions, never main(): no competition/controller process is started.
import run_competition_rescue as formal
from rescue_vision.camera import LatestFrameCamera, resolve_camera_device
from rescue_vision.config import load_config
from rescue_vision.localizer import ApproachRangeCalibration, GroundLocalizer
from rescue_vision.tracker import MultiFrameTracker
from rescue_vision.yolo26_pipeline import Yolo26Pipeline
from run_yolo_x5 import LABEL_ALIASES, load_labels

NEW_MODEL_DIR = PROJECT_ROOT.parent / 'x5_output_640_yolo26n'
DEFAULT_MODEL = NEW_MODEL_DIR / 'yolo26n_6cls_640_1109_bpu_bayese_640x640_nv12.bin'
# The supplied class order is unchanged; the new training uses different names.
BENCHMARK_LABEL_ALIASES = {'blueteam': 'safe_blue', 'redteam': 'safe_red'}
WINDOW = 'YOLO26-640 | competition vision | Q quit'


class PreviewSlot:
    """One paired image/result in shared memory; overwritten by newer results."""

    def __init__(self, context):
        self.pixels = context.RawArray('B', 1280 * 1536)
        self.metadata = context.RawArray('B', 1024 * 1024)
        self.info = context.RawArray('q', [0, 0])
        self.guard = context.Lock()

    def publish(self, image, pixel_format, vision, detections, metrics):
        payload = pickle.dumps((pixel_format, vision, detections, metrics))
        if len(payload) > len(self.metadata) or not self.guard.acquire(False):
            return False
        try:
            np.frombuffer(self.pixels, np.uint8)[:] = image.reshape(-1)
            np.frombuffer(self.metadata, np.uint8, count=len(payload))[:] = np.frombuffer(payload, np.uint8)
            self.info[1] = len(payload)
            self.info[0] += 1
        finally:
            self.guard.release()
        return True

    def receive(self, previous):
        with self.guard:
            if self.info[0] == previous:
                return None
            sequence = self.info[0]
            image = np.frombuffer(self.pixels, np.uint8).reshape(1536, 1280).copy()
            payload = bytes(np.frombuffer(self.metadata, np.uint8, count=self.info[1]))
        return sequence, image, *pickle.loads(payload)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def display_process(slot, stop, rois, directory, fps, measure_start, deadline):
    """Separate GUI process, matching the competition's nonblocking preview."""
    cv2.setNumThreads(1)
    last_frame = None
    times, latencies = [], []
    next_display = 0.0
    shown_sequence = 0
    output = formal.CompetitionOutput(formal.CompetitionState.WAIT_START, None, 'model comparison')
    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow(WINDOW, 640, 512)
        while not stop.is_set():
            now = time.monotonic()
            if now >= next_display:
                job = slot.receive(shown_sequence)
                if job is not None:
                    shown_sequence, image, pixel_format, vision, dets, metrics = job
                    # Preview only: halve NV12 before color conversion. The
                    # recognition worker still receives the full native image.
                    if pixel_format == 'nv12':
                        small = np.empty((768, 640), np.uint8)
                        small[:512] = image[:1024:2, ::2]
                        small[512:] = image[1024:].reshape(512, 640, 2)[::2, ::2].reshape(256, 640)
                        bgr = cv2.cvtColor(small, cv2.COLOR_YUV2BGR_NV12)
                    else:
                        bgr = image
                    view = formal.draw_overlay(bgr, vision, output, formal.PoseSnapshot(),
                        formal.StmSnapshot(), metrics['fps'], (640, 512), rois)
                    for item in dets:
                        x, y, w, h = item.bbox
                        a, b = (round(x * .5), round(y * .5)), (round((x+w)*.5), round((y+h)*.5))
                        color = (255, 100, 255) if item.class_name.startswith('safe_') else (0, 230, 0)
                        cv2.rectangle(view, a, b, color, 1)
                        cv2.putText(view, f'{item.class_name} {item.confidence:.2f}',
                            (a[0], max(65, a[1]-4)), cv2.FONT_HERSHEY_SIMPLEX, .48, color, 1, cv2.LINE_AA)
                    shown = time.monotonic()
                    age_ms = (shown - vision.observed_monotonic_s) * 1000
                    cv2.rectangle(view, (0, 0), (640, 56), (25, 25, 25), -1)
                    cv2.putText(view, f'NEW MODEL | recognition {metrics["fps"]:.1f} FPS | BPU {metrics["inference_ms"]:.1f} ms',
                        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .56, (0, 240, 240), 1, cv2.LINE_AA)
                    cv2.putText(view, f'Result {metrics["latency_ms"]:.1f} ms | preview {age_ms:.1f} ms | Q quit',
                        (8, 46), cv2.FONT_HERSHEY_SIMPLEX, .52, (240, 240, 240), 1, cv2.LINE_AA)
                    cv2.imshow(WINDOW, view)
                    last_frame = view
                    if measure_start <= shown < deadline:
                        times.append(shown - measure_start)
                        latencies.append(age_ms)
                    next_display = shown + 1 / fps
            if cv2.waitKey(1) & 0xff in (27, ord('q'), ord('Q')):
                stop.set()
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                stop.set()
    except Exception:
        Path(directory, 'display_error.txt').write_text(traceback.format_exc())
        stop.set()
    finally:
        if last_frame is not None:
            cv2.imwrite(str(Path(directory, 'last_preview.jpg')), last_frame)
        Path(directory, 'display.json').write_text(json.dumps({'frames':len(times), 'times_s':times,
            'capture_to_preview_ms':percentiles(latencies)}))
        cv2.destroyAllWindows()


def percentiles(values):
    return dict(zip(('p50', 'p95', 'p99'), map(float, np.percentile(values, [50, 95, 99])))) if values else None


def arguments():
    parser = argparse.ArgumentParser(description='新模型：复用正式比赛视觉进程、跟踪/ROI；不启动比赛或串口')
    parser.add_argument('--model', type=Path, default=DEFAULT_MODEL)
    parser.add_argument('--labels', type=Path, help='默认使用模型旁classes.txt；没有则使用正式类别表')
    parser.add_argument('--device', default='/dev/video0')
    parser.add_argument('--camera-fps', type=int, default=180)
    parser.add_argument('--decode-fps', type=float, default=60)
    parser.add_argument('--vision-fps', type=float, default=50)
    parser.add_argument('--pipeline', choices=('competition',), default='competition')
    parser.add_argument('--preprocess', choices=('auto', 'vse', 'cpu'), default='auto')
    parser.add_argument('--score-thres', type=float, default=formal.DEFAULT_YOLO_SCORE_THRESHOLD)
    parser.add_argument('--nms-thres', type=float, default=.45)
    parser.add_argument('--priority', type=int, default=0)
    parser.add_argument('--bpu-cores', type=int, nargs='+', default=[0])
    parser.add_argument('--side', choices=('red', 'blue'), default='red')
    parser.add_argument('--duration', type=float, default=60, help='测量秒数，不含预热；0运行到Q退出')
    parser.add_argument('--warmup', type=float, default=5)
    parser.add_argument('--display-fps', type=float, default=40)
    parser.add_argument('--no-display', action='store_true')
    parser.add_argument('--capture-roi', type=Path, default=ROOT / 'config/capture_roi.json')
    parser.add_argument('--output', type=Path, default=ROOT / 'captures/yolo26_benchmark')
    args = parser.parse_args()
    if not (0 < args.score_thres < 1 and 0 < args.nms_thres < 1 and args.duration >= 0 and
            args.warmup >= 0 and min(args.decode_fps, args.vision_fps, args.display_fps, args.camera_fps) > 0):
        parser.error('invalid threshold/timing')
    if args.labels is None:
        sibling = args.model.parent / 'classes.txt'
        args.labels = sibling if sibling.is_file() else ROOT / 'config/yolo_labels.txt'
    return args


def main():
    args = arguments()
    cv2.setNumThreads(1)
    source_labels = load_labels(args.labels)
    labels = [BENCHMARK_LABEL_ALIASES.get(name, name) for name in source_labels]
    config_path = ROOT / 'config/rescue_vision.json'
    config = load_config(config_path)
    if any(LABEL_ALIASES.get(name, name) not in config['classes'] for name in labels):
        raise ValueError(f'Unmapped training labels: {source_labels}')
    directory = args.output / (time.strftime('%Y%m%d_%H%M%S') + f'_{time.time_ns()%1000000:06d}_competition')
    directory.mkdir(parents=True)
    rois = formal.load_capture_rois(args.capture_roi)
    homography = ROOT / 'config/homography.txt'
    try:
        localizer = GroundLocalizer.load(homography, (1280, 1024))
    except (OSError, ValueError, TypeError):
        localizer = GroundLocalizer()
    range_calibration = ApproachRangeCalibration.load(ROOT / 'config/approach_range.json')
    tracker = MultiFrameTracker(config)
    mission = formal.CompetitionMission(formal.CompetitionSettings(side=args.side))
    # No live MCU input is fabricated: camera pitch/audit/manifest remain unknown.
    stm = formal.StmSnapshot()
    detector = camera = gui = None
    context = mp.get_context('spawn')
    stop = context.Event()
    preview = None if args.no_display else PreviewSlot(context)
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    rows, samples, errors = [], [], []
    recent = deque()
    dropped = {'submit_full':0, 'preview_busy':0}
    class_counts = Counter()
    recent_safe_bbox, safe_missing = None, 0
    last_id = sequence = 0
    last_report = next_vision = next_sample = 0.0
    start = measure_start = end = time.monotonic()
    deadline = float('inf')
    metadata = {**vars(args), 'source':'live_camera', 'pipeline':'competition',
        'model_sha256':sha256(args.model), 'source_labels':source_labels, 'worker_labels':labels,
        'semantic_labels':[LABEL_ALIASES.get(n, n) for n in labels],
        'label_mapping':BENCHMARK_LABEL_ALIASES, 'config_sha256':sha256(config_path),
        'capture_roi_sha256':sha256(args.capture_roi),
        'homography_calibrated':localizer.calibrated,
        'shared_code_sha256':{str(p.relative_to(PROJECT_ROOT)):sha256(p) for p in (
            Path(__file__), Path(formal.__file__), ROOT/'rescue_vision/yolo26.py',
            ROOT/'rescue_vision/yolo26_pipeline.py', ROOT/'rescue_vision/camera.py',
            ROOT/'rescue_vision/tracker.py')},
        'scope':'same recognition worker/thresholds/tracker/ROI/snapshot, no planner or control; MCU state unknown',
        'latency_reference':'host MJPEG reception to completed tracked vision snapshot; not exposure or physical display latency'}
    try:
        detector = Yolo26Pipeline(args.model, labels, args.score_thres, args.nms_thres,
            args.priority, args.bpu_cores, args.preprocess)
        metadata.update(output_shapes=detector.shapes, preprocess_actual=detector.preprocess)
        (directory/'metadata.json').write_text(json.dumps(metadata, default=str, ensure_ascii=False, indent=2))
        print(f'Model: {args.model}\nLabels: {source_labels} -> {metadata["semantic_labels"]}\nLogs: {directory}', flush=True)
        camera = LatestFrameCamera(resolve_camera_device(args.device), 1280, 1024,
            args.camera_fps, decoder='jpu', decode_fps=args.decode_fps, output_format='nv12')
        camera.start()
        start = time.monotonic()
        measure_start = start + args.warmup
        deadline = measure_start + args.duration if args.duration else float('inf')
        if not args.no_display:
            gui = context.Process(target=display_process, args=(preview, stop, rois,
                str(directory), args.display_fps, measure_start, deadline))
            gui.start()
        while not stop.is_set() and time.monotonic() < deadline:
            now = time.monotonic()
            error = camera.check_error()
            if error:
                raise RuntimeError(error)
            packet = camera.latest()
            if packet is not None and packet.frame_id != last_id and now >= next_vision:
                next_vision = now + 1 / args.vision_fps
                last_id = packet.frame_id
                if not detector.submit(packet):
                    dropped['submit_full'] += 1
            result = detector.receive()
            if result is not None:
                packet, detections, timing = result
                snapshot_start = time.monotonic()
                sequence += 1
                low_conf_green = any(item.class_name == 'green_supply' and
                    item.confidence < formal.GREEN_SUPPLY_SCORE_THRESHOLD for item in detections)
                detections = [item for item in detections if item.class_name != 'green_supply' or
                    item.confidence >= formal.GREEN_SUPPLY_SCORE_THRESHOLD]
                objects = [formal.make_detection(item, localizer) for item in detections]
                tracks = tracker.update(objects)
                detected_safe = formal.safe_bbox_from_detections(detections,
                    'safe_red' if args.side == 'red' else 'safe_blue')
                if detected_safe is not None:
                    recent_safe_bbox, safe_missing = detected_safe, 0
                    safe_bbox, safe_blocked = detected_safe, False
                elif recent_safe_bbox is not None and safe_missing < 2:
                    safe_missing += 1
                    safe_bbox, safe_blocked = recent_safe_bbox, True
                else:
                    recent_safe_bbox, safe_missing = None, 3
                    safe_bbox, safe_blocked = None, False
                vision = formal.make_vision_snapshot(detections, tracks, localizer, safe_bbox,
                    stm, mission, sequence, safe_blocked, rois, low_conf_green,
                    capture_quality_reason=formal.capture_image_quality(packet.image, packet.pixel_format,
                        rois, args.capture_roi.is_file()), observed_s=packet.published_ns/1e9,
                    range_calibration=range_calibration)
                finished = time.monotonic()
                recent.append(finished)
                while recent and recent[0] <= finished - 1:
                    recent.popleft()
                latency = (finished - packet.published_ns/1e9) * 1000
                if measure_start <= finished < deadline:
                    row = dict(frame_id=packet.frame_id, capture_s=packet.published_ns/1e9,
                        completed_s=finished, **timing, snapshot_ms=(finished-snapshot_start)*1000,
                        latency_ms=latency, detections=len(detections), tracks=len(vision.cargo))
                    rows.append(row)
                    class_counts.update(item.class_name for item in detections)
                    if finished >= next_sample:
                        samples.append(dict(sequence=sequence, frame_id=packet.frame_id,
                            capture_s=packet.published_ns/1e9,
                            detections=[asdict(item) for item in detections],
                            tracks=[asdict(item) for item in vision.cargo],
                            roi_decisions=vision.roi_decisions,
                            capture_observation_valid=vision.capture_observation_valid))
                        next_sample = finished + 1
                if gui is not None:
                    if not preview.publish(packet.image, packet.pixel_format, vision, detections,
                            dict(fps=len(recent), latency_ms=latency, inference_ms=timing['inference_ms'])):
                        dropped['preview_busy'] += 1
            if now - last_report >= 1:
                phase = 'WARMUP' if now < measure_start else 'MEASURE'
                print(f'{phase} unique_fps_1s={len(recent)} frames={len(rows)}', flush=True)
                last_report = now
            if gui is not None and not gui.is_alive():
                if not stop.is_set():
                    raise RuntimeError(f'Display exited unexpectedly: {gui.exitcode}')
                break
            time.sleep(.0005)  # Same parent loop cadence as the competition runner.
    except Exception:
        errors.append(traceback.format_exc())
    finally:
        end = min(time.monotonic(), deadline)
        stop.set()
        if detector is not None:
            detector.close()
        if camera is not None:
            camera.stop()
        if gui is not None:
            gui.join(5)
            if gui.is_alive():
                gui.terminate()
                gui.join()
                errors.append('Display process did not exit')
            if (directory/'display_error.txt').exists():
                errors.append((directory/'display_error.txt').read_text())
    elapsed = max(0, end - measure_start)
    windows = [sum(i <= r['completed_s']-measure_start < i+1 for r in rows) for i in range(int(elapsed))]
    display = json.loads((directory/'display.json').read_text()) if (directory/'display.json').exists() else {}
    summary = dict(source='live_camera', pipeline='competition', seconds=elapsed,
        model=str(args.model), model_sha256=metadata['model_sha256'], unique_results=len(rows),
        mean_fps=len(rows)/elapsed if elapsed else 0, fps_1s=windows, min_fps_1s=min(windows) if windows else None,
        stable_30fps=bool(not errors and elapsed >= 30 and windows and min(windows) >= 30),
        max_result_gap_ms=max(np.diff([r['completed_s'] for r in rows]))*1000 if len(rows)>1 else None,
        display_fps=display.get('frames', 0)/elapsed if elapsed else 0,
        capture_to_preview_ms=display.get('capture_to_preview_ms'),
        display_fps_1s=[sum(i <= t < i+1 for t in display.get('times_s', [])) for i in range(int(elapsed))],
        dropped_including_warmup=dropped, detection_class_frame_counts=dict(class_counts),
        camera_captured=camera.captured_count() if camera else None,
        camera_decoded=camera.decoded_count() if camera else None, errors=errors,
        latency_reference=metadata['latency_reference'])
    for key in ('preprocess_ms', 'inference_ms', 'postprocess_ms', 'snapshot_ms', 'latency_ms'):
        summary[key] = percentiles([r[key] for r in rows])
    (directory/'summary.json').write_text(json.dumps(summary, indent=2))
    if rows:
        with (directory/'frames.csv').open('w') as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    with (directory/'detections.jsonl').open('w') as file:
        for sample in samples:
            file.write(json.dumps(sample, ensure_ascii=False)+'\n')
    print(json.dumps(summary, indent=2), flush=True)
    if errors:
        raise RuntimeError(errors[0])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
