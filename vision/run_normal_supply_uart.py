#!/usr/bin/env python3
"""Display and report only a confirmed ordinary supply to the F407.

This is intentionally separate from ``run_editor.py`` (threshold tuning) and
``run_detector.py`` (all competition classes).  It does no web publishing and
reports image coordinates without requiring ground-distance calibration.
"""
from __future__ import annotations

import argparse
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import cv2
import serial

from rescue_vision.camera import LatestFrameCamera, resolve_camera_device
from rescue_vision.config import load_config
from rescue_vision.detector import TraditionalDetector
from rescue_vision.localizer import GroundLocalizer
from rescue_vision.tracker import MultiFrameTracker
from rescue_vision.vision_protocol import (
    IMAGE_HEIGHT,
    IMAGE_WIDTH,
    NormalSupplyReport,
    config_frame,
)


def arguments() -> argparse.Namespace:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="普通物资视觉闭环 UART 调试")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--uart", default="/dev/ttyS1")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=1024)
    parser.add_argument("--camera-fps", type=int, default=180)
    parser.add_argument("--decoder", choices=("jpu", "software"), default="jpu")
    parser.add_argument("--decode-fps", type=float, default=60.0)
    parser.add_argument("--vision-fps", type=float, default=60.0)
    parser.add_argument("--uart-fps", type=float, default=30.0)
    parser.add_argument("--window-mode", choices=("fullscreen", "normal"), default="fullscreen")
    parser.add_argument("--display-width", type=int, help="fullscreen canvas width; default detects X11")
    parser.add_argument("--display-height", type=int, help="fullscreen canvas height; default detects X11")
    parser.add_argument("--team-color", choices=("red", "blue"), default="red")
    parser.add_argument("--start-zone", choices=(1, 2, 3, 4), type=int, default=1)
    parser.add_argument("--config", default=str(root / "config" / "rescue_vision.json"))
    parser.add_argument("--homography", default=str(root / "config" / "homography.txt"))
    return parser.parse_args()


def select_target(tracker: MultiFrameTracker, image_width: int):
    """Choose the largest confirmed ordinary item, then prefer image centre."""
    choices = []
    for track in tracker.confirmed():
        if track.class_name != "green_supply":
            continue
        x, y, width, height = track.last_detection.bbox
        centre_error = abs((x + width * 0.5) - image_width * 0.5)
        choices.append((-(width * height), centre_error, track.track_id, track))
    return min(choices, default=(0, 0, 0, None))[3]


def draw_overlay(image, track, report: NormalSupplyReport | None, vision_ms: float):
    view = image.copy()
    image_height, image_width = view.shape[:2]
    cv2.line(view, (image_width // 2, 0), (image_width // 2, image_height - 1), (120, 120, 120), 1, cv2.LINE_AA)
    cv2.line(view, (0, image_height // 2), (image_width - 1, image_height // 2), (120, 120, 120), 1, cv2.LINE_AA)
    title = "ordinary supply: searching"
    color = (0, 0, 255)
    if track is not None:
        detection = track.last_detection
        x, y, width, height = detection.bbox
        center = (x + width // 2, y + height // 2)
        cv2.rectangle(view, (x, y), (x + width, y + height), (0, 255, 0), 2)
        cv2.drawMarker(view, center, (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
        title = f"ordinary supply CONFIRMED score={track.confidence:.2f} area={width * height}px"
        color = (0, 255, 0)
        if report is not None:
            title += " image-position UART"
    cv2.putText(view, title, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 2, cv2.LINE_AA)
    cv2.putText(view, f"vision={vision_ms:.1f}ms  UART=TYPE 0x12 @30Hz  F fullscreen  Q/Esc quit",
                (12, image_height - 24), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (255, 255, 255), 1, cv2.LINE_AA)
    return view


def detect_display_size() -> tuple[int, int]:
    try:
        output = subprocess.check_output(
            ["xrandr", "--current"], text=True, stderr=subprocess.DEVNULL)
        match = re.search(r"current\s+(\d+)\s+x\s+(\d+)", output)
        if match:
            return int(match.group(1)), int(match.group(2))
    except (OSError, subprocess.SubprocessError, ValueError):
        pass
    return 1024, 768


def fit_complete_image(image, canvas_width: int, canvas_height: int):
    """Fit the entire image without cropping or aspect-ratio distortion."""
    image_height, image_width = image.shape[:2]
    scale = min(canvas_width / image_width, canvas_height / image_height)
    fitted_width = max(1, int(round(image_width * scale)))
    fitted_height = max(1, int(round(image_height * scale)))
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    fitted = cv2.resize(image, (fitted_width, fitted_height), interpolation=interpolation)
    canvas = cv2.copyMakeBorder(
        fitted,
        (canvas_height - fitted_height) // 2,
        canvas_height - fitted_height - (canvas_height - fitted_height) // 2,
        (canvas_width - fitted_width) // 2,
        canvas_width - fitted_width - (canvas_width - fitted_width) // 2,
        cv2.BORDER_CONSTANT,
        value=(0, 0, 0),
    )
    return canvas


def main() -> int:
    args = arguments()
    args.device = resolve_camera_device(args.device)
    if args.vision_fps <= 0 or args.uart_fps <= 0:
        raise ValueError("FPS must be positive")
    if (args.width, args.height) != (IMAGE_WIDTH, IMAGE_HEIGHT):
        raise ValueError(
            f"视觉闭环和UART协议固定为{IMAGE_WIDTH}x{IMAGE_HEIGHT}；"
            f"当前请求为{args.width}x{args.height}"
        )

    config = load_config(args.config)
    detected_width, detected_height = detect_display_size()
    display_width = args.display_width or detected_width
    display_height = args.display_height or detected_height
    if display_width <= 0 or display_height <= 0:
        raise ValueError("display width and height must be positive")
    config.setdefault("camera", {}).update({
        "width": args.width, "height": args.height, "fps": args.camera_fps,
    })
    localizer = GroundLocalizer.load(args.homography, (args.width, args.height))
    detector = TraditionalDetector(config, localizer)
    tracker = MultiFrameTracker(config)
    camera = LatestFrameCamera(args.device, args.width, args.height, args.camera_fps,
                               decoder=args.decoder, decode_fps=args.decode_fps)
    color = 0x11 if args.team_color == "red" else 0x12
    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    sequence = 0
    last_frame_id = 0
    last_track = None
    last_image = None
    vision_ms = 0.0
    next_vision = next_uart = time.perf_counter()
    vision_period = 1.0 / args.vision_fps
    uart_period = 1.0 / args.uart_fps

    with serial.Serial(args.uart, args.baud, timeout=0, write_timeout=0.05) as uart:
        # The F407 accepts mission configuration only after three consecutive
        # reports. Send it once before normal reports, using fresh sequences.
        for _ in range(3):
            uart.write(config_frame(sequence, color, args.start_zone))
            sequence = (sequence + 1) & 0xFF
            time.sleep(0.02)
        uart.flush()
        camera.start()
        window_name = "normal-supply UART closed loop"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        fullscreen = args.window_mode == "fullscreen"
        cv2.setWindowProperty(
            window_name, cv2.WND_PROP_FULLSCREEN,
            cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL,
        )
        if not fullscreen:
            cv2.resizeWindow(window_name, min(display_width, 960), min(display_height, 720))
        print(f"camera: MJPEG {args.width}x{args.height}@{args.camera_fps} FPS; "
              f"decoder={args.decoder}@{args.decode_fps:g} FPS")
        print(f"UART coordinates use native {IMAGE_WIDTH}x{IMAGE_HEIGHT} pixels; no scaling")
        print("distance calibration is optional; current UART reports image position only")
        print(f"display: complete-image fit into {display_width}x{display_height}; no cropping")
        print(f"UART {args.uart} {args.baud} 8N1; only green_supply is enabled")
        try:
            while running:
                error = camera.check_error()
                if error:
                    raise RuntimeError(error)
                packet = camera.latest()
                if packet is None:
                    time.sleep(0.001)
                    continue
                now = time.perf_counter()
                if now >= next_vision and packet.frame_id != last_frame_id:
                    next_vision = now + vision_period
                    started = time.perf_counter()
                    detections, _debug = detector.detect(packet.image, ["green_supply"])
                    tracker.update(detections)
                    last_track = select_target(tracker, args.width)
                    last_image = packet.image
                    last_frame_id = packet.frame_id
                    vision_ms = (time.perf_counter() - started) * 1000.0

                report = None
                if last_track is not None:
                    detection = last_track.last_detection
                    x, y, width, height = detection.bbox
                    report = NormalSupplyReport(
                        x_px=max(0, min(IMAGE_WIDTH - 1, x + width // 2)),
                        y_px=max(0, min(IMAGE_HEIGHT - 1, y + height // 2)),
                        distance_mm=0,
                        found=True,
                        # The current communication stage does not request a
                        # stop/grab action from image area. Keep NEAR cleared.
                        near=False,
                        distance_valid=False,
                    )

                if now >= next_uart:
                    next_uart = now + uart_period
                    uart.write((report or NormalSupplyReport()).to_frame(sequence))
                    sequence = (sequence + 1) & 0xFF

                if last_image is not None:
                    annotated = draw_overlay(last_image, last_track, report, vision_ms)
                    if fullscreen:
                        displayed = fit_complete_image(annotated, display_width, display_height)
                    else:
                        displayed = fit_complete_image(
                            annotated, min(display_width, 960), min(display_height, 720))
                    cv2.imshow(window_name, displayed)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord("q"), ord("Q")):
                        break
                    if key in (ord("f"), ord("F")):
                        fullscreen = not fullscreen
                        cv2.setWindowProperty(
                            window_name, cv2.WND_PROP_FULLSCREEN,
                            cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL,
                        )
                        if not fullscreen:
                            cv2.resizeWindow(
                                window_name, min(display_width, 960), min(display_height, 720))
                time.sleep(0.0005)
        finally:
            camera.stop()
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        raise SystemExit(1)
