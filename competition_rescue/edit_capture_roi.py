#!/usr/bin/env python3
"""Edit the claw-capture polygon on the live competition YOLO image."""
from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = ROOT.parent
VISION_ROOT = PROJECT_ROOT / "vision"
sys.path.insert(0, str(VISION_ROOT))
sys.path.insert(0, str(ROOT))

from capture_roi import (  # noqa: E402
    IMAGE_HEIGHT,
    IMAGE_WIDTH,
    bbox_overlaps_capture_roi,
    load_capture_roi,
    save_capture_roi,
)
from rescue_vision.camera import LatestFrameCamera, resolve_camera_device  # noqa: E402
from rescue_vision.vse import VseScaler  # noqa: E402
from run_yolo_x5 import (  # noqa: E402
    DEFAULT_LABELS,
    DEFAULT_MODEL,
    DEFAULT_YOLO_SCORE_THRESHOLD,
    X5YoloV8,
    load_labels,
)


WINDOW_NAME = "capture_roi_editor"
DISPLAY_WIDTH = 960
DISPLAY_HEIGHT = 768
GREEN_SUPPLY_SCORE_THRESHOLD = 0.50


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="正式比赛夹内多边形区域标定")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--decoder", choices=("jpu", "software"), default="jpu")
    parser.add_argument("--camera-fps", type=int, default=180)
    parser.add_argument("--decode-fps", type=float, default=60.0)
    parser.add_argument("--vision-fps", type=float, default=50.0)
    parser.add_argument("--preprocess", choices=("auto", "vse", "cpu"), default="auto")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--labels", type=Path, default=DEFAULT_LABELS)
    parser.add_argument("--score-thres", type=float, default=DEFAULT_YOLO_SCORE_THRESHOLD)
    parser.add_argument("--nms-thres", type=float, default=0.45)
    parser.add_argument("--priority", type=int, default=0)
    parser.add_argument("--bpu-cores", type=int, nargs="+", default=[0, 1])
    parser.add_argument(
        "--output",
        type=Path,
        default=VISION_ROOT / "config/capture_roi.json",
    )
    return parser.parse_args()


def main() -> int:
    args = arguments()
    if args.vision_fps <= 0.0:
        raise ValueError("vision-fps must be positive")
    detector = X5YoloV8(
        args.model,
        load_labels(args.labels),
        args.score_thres,
        args.nms_thres,
        args.priority,
        args.bpu_cores,
    )
    scaler: VseScaler | None = None
    use_vse = args.preprocess in {"auto", "vse"} and args.decoder == "jpu"
    if args.preprocess == "vse" and args.decoder != "jpu":
        raise ValueError("VSE NV12 preprocessing requires JPU decoding")
    if use_vse:
        scale = min(
            detector.input_width / IMAGE_WIDTH,
            detector.input_height / IMAGE_HEIGHT,
        )
        content_width = max(2, int(round(IMAGE_WIDTH * scale)) // 2 * 2)
        content_height = max(2, int(round(IMAGE_HEIGHT * scale)) // 2 * 2)
        try:
            scaler = VseScaler(
                IMAGE_WIDTH, IMAGE_HEIGHT, content_width, content_height
            )
        except Exception:
            if args.preprocess == "vse":
                raise
            scaler = None

    camera = LatestFrameCamera(
        resolve_camera_device(args.device),
        IMAGE_WIDTH,
        IMAGE_HEIGHT,
        args.camera_fps,
        decoder=args.decoder,
        decode_fps=args.decode_fps,
        output_format="nv12" if scaler is not None else "bgr",
    )
    points = list(load_capture_roi(args.output))
    status = "Left:add  Right:undo  C:clear  R:reload  S:save  Q:quit"

    def mouse(event: int, x: int, y: int, _flags: int, _data) -> None:
        nonlocal status
        image_x = max(0, min(IMAGE_WIDTH - 1, round(x * IMAGE_WIDTH / DISPLAY_WIDTH)))
        image_y = max(0, min(IMAGE_HEIGHT - 1, round(y * IMAGE_HEIGHT / DISPLAY_HEIGHT)))
        if event == cv2.EVENT_LBUTTONDOWN:
            points.append((image_x, image_y))
            status = f"point added: ({image_x}, {image_y})"
        elif event == cv2.EVENT_RBUTTONDOWN and points:
            removed = points.pop()
            status = f"point removed: {removed}"

    running = True

    def stop(_signal, _frame) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    cv2.resizeWindow(WINDOW_NAME, DISPLAY_WIDTH, DISPLAY_HEIGHT)
    cv2.setMouseCallback(WINDOW_NAME, mouse)
    camera.start()
    last_frame_id = 0
    next_inference = time.monotonic()
    latest_image = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, 3), dtype=np.uint8)
    latest_detections = []
    try:
        while running:
            error = camera.check_error()
            if error:
                raise RuntimeError(error)
            packet = camera.latest()
            now = time.monotonic()
            if (
                packet is not None and
                packet.frame_id != last_frame_id and
                now >= next_inference
            ):
                last_frame_id = packet.frame_id
                next_inference = now + 1.0 / args.vision_fps
                latest_image = (
                    cv2.cvtColor(packet.image, cv2.COLOR_YUV2BGR_NV12)
                    if packet.pixel_format == "nv12" else packet.image.copy()
                )
                if packet.pixel_format == "nv12":
                    detections, _ = detector.infer_nv12(
                        packet.image, IMAGE_WIDTH, IMAGE_HEIGHT, scaler
                    )
                else:
                    detections, _ = detector.infer(packet.image)
                latest_detections = [
                    item for item in detections
                    if item.class_name != "green_supply" or
                    float(item.confidence) >= GREEN_SUPPLY_SCORE_THRESHOLD
                ]

            shown = cv2.resize(
                latest_image,
                (DISPLAY_WIDTH, DISPLAY_HEIGHT),
                interpolation=cv2.INTER_AREA,
            )
            scale_x = DISPLAY_WIDTH / IMAGE_WIDTH
            scale_y = DISPLAY_HEIGHT / IMAGE_HEIGHT
            polygon = tuple(points)
            for item in latest_detections:
                x, y, width, height = item.bbox
                inside = (
                    len(polygon) >= 3 and
                    bbox_overlaps_capture_roi(item.bbox, polygon)
                )
                color = (0, 255, 255) if inside else (0, 200, 0)
                p0 = (round(x * scale_x), round(y * scale_y))
                p1 = (
                    round((x + width) * scale_x),
                    round((y + height) * scale_y),
                )
                center = (
                    round((x + width * 0.5) * scale_x),
                    round((y + height * 0.5) * scale_y),
                )
                cv2.rectangle(shown, p0, p1, color, 2)
                cv2.circle(shown, center, 4, color, -1)
                cv2.putText(
                    shown,
                    f"{item.class_name} {item.confidence:.2f}",
                    (p0[0], max(18, p0[1] - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    color,
                    1,
                    cv2.LINE_AA,
                )
            if points:
                scaled = np.asarray(
                    [
                        (round(x * scale_x), round(y * scale_y))
                        for x, y in points
                    ],
                    dtype=np.int32,
                )
                cv2.polylines(
                    shown,
                    [scaled],
                    len(points) >= 3,
                    (255, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                for index, point in enumerate(scaled):
                    cv2.circle(shown, tuple(point), 5, (255, 255, 255), -1)
                    cv2.putText(
                        shown,
                        str(index + 1),
                        (int(point[0]) + 6, int(point[1]) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.45,
                        (255, 255, 255),
                        1,
                        cv2.LINE_AA,
                    )
            cv2.rectangle(shown, (0, 0), (DISPLAY_WIDTH, 48), (0, 0, 0), -1)
            cv2.putText(
                shown,
                status,
                (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                shown,
                f"points={len(points)}  yellow=bbox overlap > 80%",
                (10, 41),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            cv2.imshow(WINDOW_NAME, shown)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key in (ord("c"), ord("C")):
                points.clear()
                status = "polygon cleared"
            elif key in (ord("r"), ord("R")):
                points[:] = load_capture_roi(args.output)
                status = "polygon reloaded"
            elif key in (ord("s"), ord("S")):
                if len(points) < 3:
                    status = "at least three points are required"
                else:
                    save_capture_roi(args.output, points)
                    status = f"saved: {args.output}"
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        camera.stop()
        if scaler is not None:
            scaler.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
