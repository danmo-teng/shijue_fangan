#!/usr/bin/env python3
"""Camera/state-machine adapter for the ordinary-supply delivery test."""
from __future__ import annotations

import argparse
import json
import math
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = ROOT.parent
sys.path.insert(0, str(PROJECT_ROOT / "vision"))

from state_machine import MissionSettings, MissionState, PoseInput, RescueMission, VisionInput
from rescue_vision.camera import LatestFrameCamera, resolve_camera_device
from rescue_vision.config import load_config
from rescue_vision.detector import TraditionalDetector
from rescue_vision.localizer import GroundLocalizer
from rescue_vision.mission_protocol import Stm32Status, write_command_frame
from rescue_vision.vision_protocol import IMAGE_HEIGHT, IMAGE_WIDTH, config_frame
from rescue_vision.vse import VseScaler
from run_yolo_x5 import DEFAULT_LABELS, DEFAULT_MODEL, X5YoloV8, load_labels


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="普通物资抓取与安全区投送测试")
    parser.add_argument("--detector", choices=("yolo", "traditional"), default="yolo")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--decoder", choices=("jpu", "software"), default="jpu")
    parser.add_argument("--camera-fps", type=int, default=180)
    parser.add_argument("--decode-fps", type=float, default=60.0)
    parser.add_argument("--vision-fps", type=float, default=50.0)
    parser.add_argument("--preprocess", choices=("auto", "vse", "cpu"), default="auto")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--labels", type=Path, default=DEFAULT_LABELS)
    parser.add_argument("--score-thres", type=float, default=0.50)
    parser.add_argument("--nms-thres", type=float, default=0.45)
    parser.add_argument("--priority", type=int, default=0)
    parser.add_argument("--bpu-cores", type=int, nargs="+", default=[0, 1])
    parser.add_argument("--confirm-frames", type=int, default=3)
    parser.add_argument("--grab-timeout", type=float, default=2.0)
    parser.add_argument("--startup-timeout", type=float, default=30.0)
    parser.add_argument("--session", type=Path, default=PROJECT_ROOT / "rescue_map/runtime/session.json")
    parser.add_argument("--pose", type=Path, default=PROJECT_ROOT / "rescue_map/runtime/localization_result.json")
    parser.add_argument("--stm-status", type=Path, default=PROJECT_ROOT / "rescue_map/runtime/stm32_status.json")
    parser.add_argument("--command-file", type=Path, default=PROJECT_ROOT / "rescue_map/runtime/uart_command.bin")
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "vision/config/rescue_vision.json")
    parser.add_argument("--homography", type=Path, default=PROJECT_ROOT / "vision/config/homography.txt")
    parser.add_argument("--window-mode", choices=("fullscreen", "normal"), default="fullscreen")
    parser.add_argument("--display-fps", type=float, default=15.0)
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; 0 runs until stopped")
    return parser.parse_args()


def load_json(path: Path) -> dict | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError):
        return None


def load_pose(path: Path) -> PoseInput:
    data = load_json(path)
    if not data:
        return PoseInput()
    try:
        age_ms = (time.monotonic_ns() - int(data["timestamp_monotonic_ns"])) / 1_000_000.0
        pose = data["pose"]
        valid = data.get("quality") in {"GOOD", "DEGRADED"} and 0 <= age_ms <= 250
        return PoseInput(valid, float(pose["x_m"]), float(pose["y_m"]), float(pose["yaw_deg"]))
    except (KeyError, TypeError, ValueError):
        return PoseInput()


def load_stm_status(path: Path) -> Stm32Status:
    data = load_json(path)
    if not data:
        return Stm32Status()
    try:
        age_ms = max(0.0, (time.monotonic_ns() - int(data["timestamp_monotonic_ns"])) / 1_000_000.0)
        data = dict(data)
        data["age_ms"] = age_ms
        return Stm32Status.from_json(data)
    except (KeyError, TypeError, ValueError):
        return Stm32Status()


def largest(detections, class_name):
    matches = [item for item in detections if item.class_name == class_name]
    return max(matches, key=lambda item: item.bbox[2] * item.bbox[3], default=None)


def observation(detections, safe_class: str) -> VisionInput:
    target = largest(detections, "green_supply")
    safe = largest(detections, safe_class)
    if target is None:
        return VisionInput(safe_found=safe is not None, safe_bbox=None if safe is None else safe.bbox)
    x, y, width, height = target.bbox
    return VisionInput(
        target_found=True,
        target_x=max(0, min(IMAGE_WIDTH - 1, x + width // 2)),
        target_y=max(0, min(IMAGE_HEIGHT - 1, y + height // 2)),
        target_bbox=target.bbox,
        safe_found=safe is not None,
        safe_bbox=None if safe is None else safe.bbox,
    )


def display_size() -> tuple[int, int]:
    try:
        output = subprocess.check_output(["xrandr", "--current"], text=True, stderr=subprocess.DEVNULL)
        match = re.search(r"current\s+(\d+)\s+x\s+(\d+)", output)
        if match:
            return int(match.group(1)), int(match.group(2))
    except (OSError, subprocess.SubprocessError):
        pass
    return 1280, 1024


def fit_image(image, width: int, height: int):
    scale = min(width / image.shape[1], height / image.shape[0])
    resized = cv2.resize(image, (round(image.shape[1] * scale), round(image.shape[0] * scale)), interpolation=cv2.INTER_AREA)
    top = (height - resized.shape[0]) // 2
    left = (width - resized.shape[1]) // 2
    return cv2.copyMakeBorder(resized, top, height - resized.shape[0] - top, left, width - resized.shape[1] - left, cv2.BORDER_CONSTANT)


def draw(image, vision: VisionInput, output, pose: PoseInput, stm: Stm32Status):
    view = image.copy()
    cv2.line(view, (640, 0), (640, 1023), (110, 110, 110), 1)
    cv2.line(view, (0, 512), (1279, 512), (110, 110, 110), 1)
    if vision.target_bbox:
        x, y, w, h = vision.target_bbox
        cv2.rectangle(view, (x, y), (x + w, y + h), (0, 255, 0), 3)
        cv2.circle(view, (vision.target_x, vision.target_y), 8, (0, 255, 0), 2)
    if vision.safe_bbox:
        x, y, w, h = vision.safe_bbox
        cv2.rectangle(view, (x, y), (x + w, y + h), (255, 0, 255), 3)
    lines = [
        f"state={output.state.value}  {output.message}",
        f"target=({vision.target_x},{vision.target_y}) found={int(vision.target_found)} claw={int(stm.claw_visible)} age={stm.age_ms:.0f}ms",
        f"pose=({pose.x_m:+.2f},{pose.y_m:+.2f}) yaw={pose.yaw_deg:.1f} valid={int(pose.valid)}",
        "camera-down + target anywhere x confirm-frames = grab; F fullscreen; Q/Esc quit",
    ]
    for index, line in enumerate(lines):
        cv2.putText(view, line, (14, 34 + index * 30), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
    return view


def main() -> int:
    args = arguments()
    if args.vision_fps <= 0 or args.display_fps <= 0:
        raise ValueError("vision-fps和display-fps必须为正数")
    if not 0.0 < args.score_thres < 1.0 or not 0.0 < args.nms_thres < 1.0:
        raise ValueError("score-thres和nms-thres必须在0..1之间")
    session = load_json(args.session)
    if not session or session.get("side") not in {"red", "blue"}:
        raise RuntimeError(f"请先在rescue_map选择出发区和红蓝方：{args.session}")
    side = str(session["side"])
    settings = MissionSettings(
        side=side,
        confirmation_frames=args.confirm_frames,
        grab_timeout_s=args.grab_timeout,
    )
    mission = RescueMission(settings)
    config = load_config(args.config)
    localizer = GroundLocalizer.load(args.homography, (IMAGE_WIDTH, IMAGE_HEIGHT))
    traditional_detector = (
        TraditionalDetector(config, localizer)
        if args.detector == "traditional" else None
    )
    yolo_detector = None
    scaler = None
    if args.detector == "yolo":
        yolo_detector = X5YoloV8(
            args.model, load_labels(args.labels), args.score_thres, args.nms_thres,
            args.priority, args.bpu_cores,
        )
        use_vse = args.preprocess in {"auto", "vse"} and args.decoder == "jpu"
        if args.preprocess == "vse" and args.decoder != "jpu":
            raise ValueError("VSE NV12路径要求--decoder jpu")
        if use_vse:
            scale = min(yolo_detector.input_width / IMAGE_WIDTH,
                        yolo_detector.input_height / IMAGE_HEIGHT)
            content_width = max(2, int(round(IMAGE_WIDTH * scale)) // 2 * 2)
            content_height = max(2, int(round(IMAGE_HEIGHT * scale)) // 2 * 2)
            try:
                scaler = VseScaler(
                    IMAGE_WIDTH, IMAGE_HEIGHT, content_width, content_height
                )
            except Exception as error:
                if args.preprocess == "vse":
                    raise
                print(f"警告：VSE初始化失败，回退CPU预处理：{error}")
    camera = LatestFrameCamera(
        resolve_camera_device(args.device), IMAGE_WIDTH, IMAGE_HEIGHT, args.camera_fps,
        decoder=args.decoder, decode_fps=args.decode_fps,
        output_format="nv12" if scaler is not None else "bgr",
    )
    print(
        f"识别器={args.detector}，"
        f"置信度阈值={args.score_thres:.2f}，"
        f"预处理={'JPU NV12 + VSE' if scaler is not None else 'CPU BGR'}"
    )
    safe_class = "safe_red" if side == "red" else "safe_blue"
    team_color = 0x11 if side == "red" else 0x12
    deadline = time.monotonic() + args.startup_timeout
    while not load_pose(args.pose).valid:
        if time.monotonic() >= deadline:
            raise RuntimeError(
                "融合定位/串口转发器未就绪；请先运行rescue_map并确认T265为GOOD"
            )
        time.sleep(0.05)
    config_sequence = 0
    for _ in range(3):
        write_command_frame(args.command_file, config_frame(config_sequence, team_color, int(session["start_zone"])))
        config_sequence = (config_sequence + 1) & 0xFF
        time.sleep(0.04)
    report_sequence = 0
    mission_sequence = 0
    last_command_signature = None

    running = True
    def stop(_signal, _frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    screen_width, screen_height = display_size()
    window = "ordinary supply mission test"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    fullscreen = args.window_mode == "fullscreen"
    cv2.setWindowProperty(window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL)
    camera.start()
    started = time.perf_counter()
    next_vision = started
    next_display = next_vision
    last_id = 0
    latest_image = None
    latest_pixel_format = "bgr"
    latest_vision = VisionInput()
    latest_output = mission.step(latest_vision, PoseInput(), Stm32Status())
    try:
        while running:
            error = camera.check_error()
            if error:
                raise RuntimeError(error)
            packet = camera.latest()
            if packet is None:
                time.sleep(0.002)
                continue
            now = time.perf_counter()
            if args.duration > 0 and now - started >= args.duration:
                break
            if now >= next_vision and packet.frame_id != last_id:
                next_vision = now + 1.0 / args.vision_fps
                if yolo_detector is not None:
                    if packet.pixel_format == "nv12":
                        detections, _ = yolo_detector.infer_nv12(
                            packet.image, IMAGE_WIDTH, IMAGE_HEIGHT, scaler
                        )
                    else:
                        detections, _ = yolo_detector.infer(packet.image)
                else:
                    assert traditional_detector is not None
                    classes = ["green_supply"]
                    if mission.state == MissionState.ENTER_SAFE_ZONE:
                        classes.append(safe_class)
                    detections, _ = traditional_detector.detect(packet.image, classes)
                latest_vision = observation(detections, safe_class)
                latest_output = mission.step(latest_vision, load_pose(args.pose), load_stm_status(args.stm_status))
                if latest_output.command is not None:
                    packet_out = latest_output.command.to_frame(mission_sequence)
                    command_signature = (
                        latest_output.state.value,
                        latest_output.command.command,
                        latest_output.command.flags,
                        latest_output.command.target_x_mm,
                        latest_output.command.target_y_mm,
                        latest_output.command.heading_cdeg,
                    )
                    if command_signature != last_command_signature:
                        print(
                            "任务命令："
                            f"state={latest_output.state.value} "
                            f"cmd={latest_output.command.command} "
                            f"flags=0x{latest_output.command.flags:02X} "
                            f"target=({latest_output.command.target_x_mm},"
                            f"{latest_output.command.target_y_mm}) "
                            f"heading={latest_output.command.heading_cdeg / 100.0:.2f}° "
                            f"seq={mission_sequence}"
                        )
                        last_command_signature = command_signature
                    mission_sequence = (mission_sequence + 1) & 0xFF
                elif latest_output.report is not None:
                    packet_out = latest_output.report.to_frame(report_sequence)
                    report_sequence = (report_sequence + 1) & 0xFF
                else:
                    packet_out = None
                if packet_out is not None:
                    write_command_frame(args.command_file, packet_out)
                latest_image = packet.image
                latest_pixel_format = packet.pixel_format
                last_id = packet.frame_id
            if latest_image is not None and now >= next_display:
                next_display = now + 1.0 / args.display_fps
                display_image = (
                    cv2.cvtColor(latest_image, cv2.COLOR_YUV2BGR_NV12)
                    if latest_pixel_format == "nv12" else latest_image
                )
                shown = fit_image(draw(display_image, latest_vision, latest_output, load_pose(args.pose), load_stm_status(args.stm_status)), screen_width, screen_height)
                cv2.imshow(window, shown)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q"), ord("Q")):
                    break
                if key in (ord("f"), ord("F")):
                    fullscreen = not fullscreen
                    cv2.setWindowProperty(window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL)
            time.sleep(0.0005)
    finally:
        camera.stop()
        if scaler is not None:
            scaler.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        raise SystemExit(1)
