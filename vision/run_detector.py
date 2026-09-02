#!/usr/bin/env python3
"""Headless runtime with tiered detection rates and latest-frame processing."""
from __future__ import annotations

import argparse
import json
import signal
import time
from pathlib import Path

from rescue_vision.camera import LatestFrameCamera
from rescue_vision.config import load_config
from rescue_vision.detector import TraditionalDetector
from rescue_vision.localizer import GroundLocalizer
from rescue_vision.tracker import MultiFrameTracker


def track_dict(track, namespace: str) -> dict:
    detection = track.last_detection
    return {
        "id": f"{namespace}{track.track_id}",
        "class": track.class_name,
        "confidence": round(track.confidence, 4),
        "state": track.state.value,
        "position": [round(value, 2) for value in track.position],
        "coordinate_system": "ground_mm" if detection.ground_xy_mm is not None else "image_px",
        "size_mm": None if detection.size_mm is None else [round(value, 2) for value in detection.size_mm],
        "hits": track.hits,
        "misses": track.misses,
    }


def main() -> int:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    parser.add_argument("--camera-fps", type=int)
    parser.add_argument("--danger-fps", type=float)
    parser.add_argument("--material-fps", type=float)
    parser.add_argument("--zone-fps", type=float)
    parser.add_argument("--config", default=str(root / "config" / "rescue_vision.json"))
    parser.add_argument("--homography", default=str(root / "config" / "homography.txt"))
    parser.add_argument("--output", default=str(root / "runtime_result.json"))
    parser.add_argument("--print-fps", type=float, default=2.0)
    args = parser.parse_args()

    config = load_config(args.config)
    runtime = config["runtime"]
    camera_config = config.setdefault("camera", {})
    width = int(args.width or camera_config.get("width", 1280))
    height = int(args.height or camera_config.get("height", 720))
    camera_fps = int(args.camera_fps or camera_config.get("fps", 180))
    camera_config.update({"width": width, "height": height, "fps": camera_fps})
    danger_classes = runtime.get("danger_classes", ["danger_cyan"])
    materials = [name for name in runtime["material_classes"] if name not in danger_classes]
    zones = runtime["zone_classes"]
    danger_fps = float(args.danger_fps or runtime.get("danger_fps", 120))
    material_fps = float(args.material_fps or runtime.get("material_fps", 90))
    zone_fps = float(args.zone_fps or runtime.get("zone_fps", 30))
    if not 60 <= danger_fps <= 120:
        parser.error("--danger-fps必须在60..120之间")
    if not 60 <= material_fps <= 90:
        parser.error("--material-fps必须在60..90之间")
    if not 20 <= zone_fps <= 30:
        parser.error("--zone-fps必须在20..30之间")
    danger_period = 1.0 / danger_fps
    material_period = 1.0 / material_fps
    zone_period = 1.0 / zone_fps
    localizer = GroundLocalizer.load(args.homography, (width, height))
    detector = TraditionalDetector(config, localizer)
    danger_tracker = MultiFrameTracker(config)
    material_tracker = MultiFrameTracker(config)
    zone_tracker = MultiFrameTracker(config)
    camera = LatestFrameCamera(args.device, width, height, camera_fps)
    output = Path(args.output)
    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    print(
        f"启动：{width}x{height} MJPEG@{camera_fps}，危险物资{danger_fps:g}Hz，"
        f"普通物资{material_fps:g}Hz，安全区{zone_fps:g}Hz，只保留最新帧"
    )
    if not localizer.calibrated:
        print("警告：当前分辨率没有匹配的单应标定，将输出图像像素坐标；请重新运行地面标定。")
    camera.start()
    next_danger = next_material = next_zone = next_output = next_print = time.perf_counter()
    last_danger_id = last_material_id = last_zone_id = 0
    danger_cost_ms = material_cost_ms = zone_cost_ms = 0.0
    report_time = time.perf_counter()
    report_decoded = camera.decoded_count()
    danger_cycles = material_cycles = zone_cycles = 0

    try:
        while running:
            error = camera.check_error()
            if error:
                print(error)
                return 1
            packet = camera.latest()
            if packet is None:
                time.sleep(0.001)
                continue
            now = time.perf_counter()
            material_due = now >= next_material and packet.frame_id != last_material_id
            danger_due = now >= next_danger and packet.frame_id != last_danger_id
            if material_due:
                # At each 60 Hz full-material tick, reuse the same HSV/Lab
                # conversion for danger and ordinary materials. The alternating
                # ticks only run the latency-critical danger class.
                next_material = now + material_period
                next_danger = now + danger_period
                started = time.perf_counter()
                detections, _ = detector.detect(packet.image, danger_classes + materials)
                danger_set = set(danger_classes)
                danger_tracker.update([item for item in detections if item.class_name in danger_set])
                material_tracker.update([item for item in detections if item.class_name not in danger_set])
                material_cost_ms = (time.perf_counter() - started) * 1000.0
                danger_cost_ms = material_cost_ms
                last_danger_id = last_material_id = packet.frame_id
                danger_cycles += 1
                material_cycles += 1
            elif danger_due:
                next_danger = now + danger_period
                started = time.perf_counter()
                detections, _ = detector.detect(packet.image, danger_classes)
                danger_tracker.update(detections)
                danger_cost_ms = (time.perf_counter() - started) * 1000.0
                last_danger_id = packet.frame_id
                danger_cycles += 1
            if now >= next_zone and packet.frame_id != last_zone_id:
                next_zone = now + zone_period
                started = time.perf_counter()
                detections, _ = detector.detect(packet.image, zones)
                zone_tracker.update(detections)
                zone_cost_ms = (time.perf_counter() - started) * 1000.0
                last_zone_id = packet.frame_id
                zone_cycles += 1
            if now >= next_output:
                next_output = now + 0.02
                danger_tracks = danger_tracker.confirmed()
                material_tracks = material_tracker.confirmed()
                zone_tracks = zone_tracker.confirmed()
                result = {
                    "timestamp_monotonic_ns": time.monotonic_ns(),
                    "frame_id": packet.frame_id,
                    "frame_age_ms": round((time.monotonic_ns() - packet.published_ns) / 1_000_000.0, 3),
                    "calibrated": localizer.calibrated,
                    "tracks": [track_dict(track, "D") for track in danger_tracks]
                    + [track_dict(track, "M") for track in material_tracks]
                    + [track_dict(track, "Z") for track in zone_tracks],
                }
                temporary = output.with_suffix(output.suffix + ".tmp")
                temporary.write_text(json.dumps(result, ensure_ascii=False), encoding="utf-8")
                temporary.replace(output)
            if now >= next_print:
                next_print = now + 1.0 / args.print_fps
                confirmed = len(danger_tracker.confirmed()) + len(material_tracker.confirmed()) + len(zone_tracker.confirmed())
                elapsed = max(now - report_time, 1e-6)
                decoded = camera.decoded_count()
                print(
                    f"frame={packet.frame_id} decode={(decoded-report_decoded)/elapsed:.1f}fps "
                    f"danger={danger_cycles/elapsed:.1f}fps/{danger_cost_ms:.2f}ms "
                    f"material={material_cycles/elapsed:.1f}fps/{material_cost_ms:.2f}ms "
                    f"zone={zone_cycles/elapsed:.1f}fps/{zone_cost_ms:.2f}ms confirmed={confirmed}"
                )
                report_time, report_decoded = now, decoded
                danger_cycles = material_cycles = zone_cycles = 0
            time.sleep(0.0005)
    finally:
        camera.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
