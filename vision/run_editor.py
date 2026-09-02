#!/usr/bin/env python3
from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

import cv2

from rescue_vision.camera import LatestFrameCamera
from rescue_vision.config import load_config
from rescue_vision.detector import TraditionalDetector
from rescue_vision.editor import ThresholdEditor
from rescue_vision.localizer import GroundLocalizer
from rescue_vision.tracker import MultiFrameTracker


def arguments() -> argparse.Namespace:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="RDK X5救援目标传统视觉阈值编辑器")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    parser.add_argument("--camera-fps", type=int)
    parser.add_argument("--vision-fps", type=float, default=90.0)
    parser.add_argument("--gui-fps", type=float, default=25.0)
    parser.add_argument("--config", default=str(root / "config" / "rescue_vision.json"))
    parser.add_argument("--homography", default=str(root / "config" / "homography.txt"))
    parser.add_argument("--captures", default=str(root / "captures"))
    return parser.parse_args()


def main() -> int:
    args = arguments()
    config = load_config(args.config)
    camera_config = config.setdefault("camera", {})
    args.width = int(args.width or camera_config.get("width", 1280))
    args.height = int(args.height or camera_config.get("height", 720))
    args.camera_fps = int(args.camera_fps or camera_config.get("fps", 180))
    camera_config.update({"width": args.width, "height": args.height, "fps": args.camera_fps})
    localizer = GroundLocalizer.load(args.homography, (args.width, args.height))
    detector = TraditionalDetector(config, localizer)
    tracker = MultiFrameTracker(config)
    camera = LatestFrameCamera(args.device, args.width, args.height, args.camera_fps)
    editor = ThresholdEditor(config, args.config, args.captures)
    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    camera.start()
    print(f"启动：MJPEG {args.width}x{args.height}@{args.camera_fps}，识别{args.vision_fps:.1f}Hz，GUI {args.gui_fps:.1f}Hz")
    print("单应标定：" + ("已加载" if localizer.calibrated else "未加载，当前只输出像素坐标"))

    vision_period = 1.0 / args.vision_fps
    gui_period = 1.0 / args.gui_fps
    next_vision = time.perf_counter()
    next_gui = next_vision
    last_frame_id = 0
    detections, debug, tracks = [], {"masks": {}, "rejected": []}, []
    last_frame = None
    report_time = next_vision
    report_decode = 0
    report_vision = 0
    report_gui = 0
    window_vision = 0
    window_gui = 0
    metrics = {}

    try:
        while running:
            error = camera.check_error()
            if error:
                print(error, file=sys.stderr)
                return 1
            now = time.perf_counter()
            packet = camera.latest()
            if packet is None:
                time.sleep(0.001)
                continue

            if now >= next_vision:
                next_vision += vision_period
                if now - next_vision > vision_period:
                    next_vision = now + vision_period
                if packet.frame_id != last_frame_id or editor.frozen:
                    editor.pull_parameters()
                    detector.update_config(config)
                    selected_frame = editor.choose_frame(packet.image)
                    started = time.perf_counter()
                    # The editor tunes one class at a time. Limiting detection
                    # to that class preserves 120 Hz while all mask stages and
                    # rejected-candidate reasons remain visible.
                    detections, debug = detector.detect(
                        selected_frame, [editor.selected_class], collect_debug=True
                    )
                    tracks = tracker.update(detections)
                    metrics["vision_ms"] = (time.perf_counter() - started) * 1000.0
                    metrics["frame_age_ms"] = (time.monotonic_ns() - packet.published_ns) / 1_000_000.0
                    last_frame = selected_frame
                    last_frame_id = packet.frame_id
                    window_vision += 1

            if now >= next_gui and last_frame is not None:
                next_gui = now + gui_period
                key = editor.render(last_frame, debug["masks"], detections, tracks, debug["rejected"], metrics)
                window_gui += 1
                if not editor.handle_key(key, last_frame):
                    break

            if now - report_time >= 1.0:
                elapsed = now - report_time
                decoded = camera.decoded_count()
                metrics["decode_fps"] = (decoded - report_decode) / elapsed
                metrics["vision_fps"] = window_vision / elapsed
                metrics["gui_fps"] = window_gui / elapsed
                print(f"decode={metrics['decode_fps']:.1f} vision={metrics['vision_fps']:.1f} GUI={metrics['gui_fps']:.1f} cost={metrics.get('vision_ms', 0):.2f}ms age={metrics.get('frame_age_ms', 0):.2f}ms confirmed={len(tracker.confirmed())}")
                report_time, report_decode = now, decoded
                window_vision = window_gui = 0
            time.sleep(0.0005)
    finally:
        camera.stop()
        editor.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
