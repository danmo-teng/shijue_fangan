#!/usr/bin/env python3
"""Interactive UVC camera parameter test tool.

The tool opens a V4L2 camera, shows the live image, prints the negotiated
format, and allows common V4L2 controls to be changed while streaming.

Examples:
    python3 camera_parameter_test.py
    python3 camera_parameter_test.py --width 1280 --height 720 --fps 180
    python3 camera_parameter_test.py --format YUYV --width 1280 --height 1024 --fps 30
    python3 camera_parameter_test.py --set-ctrl auto_exposure=1 --exposure 15

This program intentionally lives outside the rescue-vision runtime. It is a
diagnostic tool and does not change the existing recognition code.
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np


CONTROL_ALIASES = {
    "exposure": "exposure_time_absolute",
    "focus": "focus_absolute",
    "white_balance": "white_balance_temperature",
    "sharpness": "sharpness",
    "brightness": "brightness",
    "contrast": "contrast",
    "saturation": "saturation",
    "gamma": "gamma",
    "backlight_compensation": "backlight_compensation",
}

LEGACY_DECREASE_KEYS = {
    ord("e"): ("exposure_time_absolute", -1),
    ord("f"): ("focus_absolute", -5),
    ord("w"): ("white_balance_temperature", -100),
    ord("k"): ("sharpness", -1),
    ord("b"): ("brightness", -1),
    ord("c"): ("contrast", -1),
    ord("v"): ("saturation", -1),
    ord("g"): ("gamma", -1),
}

SELECTABLE_CONTROLS = {
    ord("1"): ("exposure_time_absolute", "曝光"),
    ord("2"): ("focus_absolute", "焦距"),
    ord("3"): ("white_balance_temperature", "白平衡"),
    ord("4"): ("sharpness", "锐度"),
    ord("5"): ("brightness", "亮度"),
    ord("6"): ("contrast", "对比度"),
    ord("7"): ("saturation", "饱和度"),
    ord("8"): ("gamma", "伽马"),
}

CONTROL_DELTAS = {
    "exposure_time_absolute": 1,
    "focus_absolute": 5,
    "white_balance_temperature": 100,
    "sharpness": 1,
    "brightness": 1,
    "contrast": 1,
    "saturation": 1,
    "gamma": 1,
}

ADJUSTMENT_KEYS = {
    ord("["): -1,
    ord("]"): 1,
    ord("-"): -1,
    ord("="): 1,
}


@dataclass
class ControlInfo:
    name: str
    minimum: int | None = None
    maximum: int | None = None
    step: int = 1
    value: int | None = None


@dataclass
class CameraInfo:
    width: str = "?"
    height: str = "?"
    pixel_format: str = "?"
    fps: str = "?"

    def summary(self) -> str:
        return f"{self.pixel_format} {self.width}x{self.height}@{self.fps}"


class CaptureWorker:
    """Read frames independently so the GUI does not throttle camera capture."""

    def __init__(self, cap: cv2.VideoCapture) -> None:
        self.cap = cap
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="camera-capture", daemon=True)
        self._latest: np.ndarray | None = None
        self._latest_id = 0
        self._latest_time = 0.0
        self._captured = 0
        self._failed = 0

    def start(self) -> None:
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            now = time.monotonic()
            with self._lock:
                if ok and frame is not None:
                    self._captured += 1
                    self._latest = frame
                    self._latest_id = self._captured
                    self._latest_time = now
                else:
                    self._failed += 1
            if not ok and self._stop.wait(0.005):
                break

    def latest(self) -> tuple[int, np.ndarray | None, float]:
        with self._lock:
            return self._latest_id, self._latest, self._latest_time

    def stats(self) -> tuple[int, int]:
        with self._lock:
            return self._captured, self._failed

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)


def run_v4l2(device: str, *arguments: str) -> subprocess.CompletedProcess[str]:
    """Run v4l2-ctl without raising so unsupported controls are reportable."""

    return subprocess.run(
        ["v4l2-ctl", "-d", device, *arguments],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )


def parse_int(value: str) -> int | None:
    match = re.search(r"-?\d+", value)
    return int(match.group(0)) if match else None


def list_controls(device: str) -> dict[str, ControlInfo]:
    result = run_v4l2(device, "--list-ctrls")
    controls: dict[str, ControlInfo] = {}
    name_pattern = re.compile(r"^\s*(?P<name>[A-Za-z0-9_]+)\s+0x[0-9a-fA-F]+")
    for line in result.stdout.splitlines():
        name_match = name_pattern.search(line)
        value_match = re.search(r"\bvalue=(?P<value>-?\d+)", line)
        if not name_match or not value_match:
            continue
        range_match = re.search(
            r"\bmin=(?P<minimum>-?\d+)\s+max=(?P<maximum>-?\d+)\s+"
            r"step=(?P<step>-?\d+)",
            line,
        )
        controls[name_match.group("name")] = ControlInfo(
            name=name_match.group("name"),
            minimum=parse_int(range_match.group("minimum")) if range_match else None,
            maximum=parse_int(range_match.group("maximum")) if range_match else None,
            step=parse_int(range_match.group("step")) if range_match else 1,
            value=parse_int(value_match.group("value")),
        )
    return controls


def get_control(device: str, name: str) -> int | None:
    result = run_v4l2(device, f"--get-ctrl={name}")
    if result.returncode != 0:
        return None
    for line in result.stdout.splitlines():
        if ":" not in line:
            continue
        control_name, raw_value = line.split(":", 1)
        if control_name.strip() == name:
            return parse_int(raw_value)
    return None


def get_controls(device: str, names: Iterable[str]) -> dict[str, int]:
    values: dict[str, int] = {}
    for name in names:
        value = get_control(device, name)
        if value is not None:
            values[name] = value
    return values


def set_control(device: str, name: str, value: int) -> bool:
    result = run_v4l2(device, f"--set-ctrl={name}={value}")
    if result.returncode != 0:
        print(f"设置 {name}={value} 失败：{result.stdout.strip()}", file=sys.stderr)
        return False
    return True


def parse_custom_controls(items: list[str]) -> list[tuple[str, int]]:
    controls: list[tuple[str, int]] = []
    for item in items:
        if "=" not in item:
            raise ValueError(f"--set-ctrl 参数必须是 NAME=VALUE：{item}")
        name, raw_value = item.split("=", 1)
        name = name.strip()
        if not name:
            raise ValueError(f"控制项名称不能为空：{item}")
        try:
            value = int(raw_value.strip())
        except ValueError as exc:
            raise ValueError(f"控制项值必须是整数：{item}") from exc
        controls.append((name, value))
    return controls


def requested_controls(args: argparse.Namespace) -> list[tuple[str, int]]:
    controls: list[tuple[str, int]] = []

    # Apply automatic modes before their corresponding manual values.
    if args.auto_exposure is not None:
        controls.append(("auto_exposure", args.auto_exposure))
    if args.auto_white_balance is not None:
        controls.append(("white_balance_automatic", args.auto_white_balance))
    if args.auto_focus is not None:
        controls.append(("focus_automatic_continuous", args.auto_focus))

    for argument_name, control_name in CONTROL_ALIASES.items():
        value = getattr(args, argument_name)
        if value is not None:
            controls.append((control_name, value))

    controls.extend(parse_custom_controls(args.set_ctrl))
    return controls


def query_camera_info(device: str) -> CameraInfo:
    result = run_v4l2(device, "--get-fmt-video", "--get-parm")
    info = CameraInfo()
    for line in result.stdout.splitlines():
        if "Width/Height" in line:
            match = re.search(r"(\d+)\s*/\s*(\d+)", line)
            if match:
                info.width, info.height = match.group(1), match.group(2)
        elif "Pixel Format" in line:
            match = re.search(r"'([^']+)'", line)
            if match:
                info.pixel_format = match.group(1)
        elif "Frames per second" in line:
            match = re.search(r"([0-9]+(?:\.[0-9]+)?)", line)
            if match:
                info.fps = match.group(1)
    return info


def fourcc_text(value: float) -> str:
    code = int(value)
    if code <= 0:
        return "?"
    return "".join(chr((code >> (8 * index)) & 0xFF) for index in range(4))


def apply_capture_format(cap: cv2.VideoCapture, args: argparse.Namespace) -> dict[str, bool]:
    fourcc = cv2.VideoWriter_fourcc(*args.format)
    requested = [
        (cv2.CAP_PROP_FOURCC, float(fourcc)),
        (cv2.CAP_PROP_FRAME_WIDTH, float(args.width)),
        (cv2.CAP_PROP_FRAME_HEIGHT, float(args.height)),
        (cv2.CAP_PROP_FPS, float(args.fps)),
        (cv2.CAP_PROP_BUFFERSIZE, float(args.buffers)),
    ]
    results: dict[str, bool] = {}
    names = ["FOURCC", "width", "height", "fps", "buffers"]
    for name, (property_id, value) in zip(names, requested):
        results[name] = bool(cap.set(property_id, value))
    return results


def capture_properties(cap: cv2.VideoCapture) -> CameraInfo:
    return CameraInfo(
        width=f"{cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}",
        height=f"{cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f}",
        pixel_format=fourcc_text(cap.get(cv2.CAP_PROP_FOURCC)),
        fps=f"{cap.get(cv2.CAP_PROP_FPS):.2f}",
    )


def same_mode(left: CameraInfo, right: CameraInfo) -> bool:
    if (left.width, left.height, left.pixel_format) != (
        right.width,
        right.height,
        right.pixel_format,
    ):
        return False
    try:
        return abs(float(left.fps) - float(right.fps)) < 0.1
    except ValueError:
        return left.fps == right.fps


def clamp_control(value: int, info: ControlInfo | None) -> int:
    if info is None:
        return value
    if info.minimum is not None:
        value = max(info.minimum, value)
    if info.maximum is not None:
        value = min(info.maximum, value)
    if info.step > 1 and info.minimum is not None:
        value = info.minimum + round((value - info.minimum) / info.step) * info.step
        if info.maximum is not None:
            value = min(info.maximum, value)
    return value


def quality_metrics(frame: np.ndarray) -> tuple[float, float, float]:
    height, width = frame.shape[:2]
    roi = frame[height // 4 : 3 * height // 4, width // 4 : 3 * width // 4]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray = cv2.resize(gray, (160, 120), interpolation=cv2.INTER_AREA)
    mean_luma = float(gray.mean())
    clipped_ratio = float(np.mean((gray <= 3) | (gray >= 252)))
    sharpness = float(cv2.Laplacian(gray, cv2.CV_32F).var())
    return mean_luma, clipped_ratio, sharpness


def fit_for_display(frame: np.ndarray, max_width: int, max_height: int) -> np.ndarray:
    height, width = frame.shape[:2]
    scale = min(max_width / width, max_height / height)
    if scale <= 0:
        return frame
    new_size = (max(1, int(width * scale)), max(1, int(height * scale)))
    if new_size == (width, height):
        return frame
    interpolation = cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR
    return cv2.resize(frame, new_size, interpolation=interpolation)


def print_status(
    args: argparse.Namespace,
    cap: cv2.VideoCapture,
    v4l2_info: CameraInfo,
    controls: dict[str, int],
    capture_fps: float,
    display_fps: float,
    quality: tuple[float, float, float] | None,
) -> None:
    capture_info = capture_properties(cap)
    print(f"请求格式：{args.format} {args.width}x{args.height}@{args.fps}")
    print(f"OpenCV实际：{capture_info.summary()}")
    print(f"V4L2实际：{v4l2_info.summary()}")
    if capture_fps > 0:
        print(f"采集线程帧率：{capture_fps:.1f} FPS")
    if display_fps > 0:
        print(f"窗口显示帧率：{display_fps:.1f} FPS")
    if quality is not None:
        mean_luma, clipped_ratio, sharpness = quality
        print(
            f"画质指标：亮度={mean_luma:.1f}，过暗/过曝比例={clipped_ratio:.3f}，"
            f"清晰度(Laplacian)={sharpness:.1f}"
        )
    if controls:
        print("控制参数：" + " ".join(f"{name}={value}" for name, value in sorted(controls.items())))
    print("-" * 72)


def save_frame(frame: np.ndarray, save_dir: Path) -> Path | None:
    save_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    path = save_dir / f"frame_{timestamp}.png"
    if cv2.imwrite(str(path), frame):
        return path
    return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="打开V4L2摄像头并实时调试分辨率、格式、帧率和图像参数"
    )
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--format", choices=("MJPG", "YUYV"), default="MJPG")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=1024)
    parser.add_argument("--fps", type=int, default=350)
    parser.add_argument("--buffers", type=int, default=1)
    parser.add_argument("--display-width", type=int, default=1280)
    parser.add_argument("--display-height", type=int, default=720)
    parser.add_argument("--display-fps", type=float, default=30.0)
    parser.add_argument(
        "--save-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "captures",
    )

    parser.add_argument("--exposure", type=int, default=None)
    parser.add_argument("--focus", type=int, default=None)
    parser.add_argument("--white-balance", type=int, default=None)
    parser.add_argument("--sharpness", type=int, default=None)
    parser.add_argument("--brightness", type=int, default=None)
    parser.add_argument("--contrast", type=int, default=None)
    parser.add_argument("--saturation", type=int, default=None)
    parser.add_argument("--gamma", type=int, default=None)
    parser.add_argument("--backlight-compensation", type=int, default=None)
    parser.add_argument(
        "--auto-exposure",
        type=int,
        choices=(0, 1, 2, 3),
        default=None,
        help="直接设置V4L2 auto_exposure菜单值，例如本摄像头1=手动、3=光圈优先",
    )
    parser.add_argument("--auto-white-balance", type=int, choices=(0, 1), default=None)
    parser.add_argument("--auto-focus", type=int, choices=(0, 1), default=None)
    parser.add_argument(
        "--set-ctrl",
        action="append",
        default=[],
        metavar="NAME=VALUE",
        help="设置任意V4L2控制，可重复，例如 --set-ctrl contrast=50",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.width <= 0 or args.height <= 0 or args.fps <= 0 or args.display_fps <= 0:
        raise SystemExit("width、height、fps和display-fps必须大于0")
    if not shutil.which("v4l2-ctl"):
        raise SystemExit("找不到 v4l2-ctl，请先安装 v4l-utils")

    try:
        controls_to_set = requested_controls(args)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    backend = getattr(cv2, "CAP_V4L2", cv2.CAP_ANY)
    cap = cv2.VideoCapture(args.device, backend)
    if not cap.isOpened():
        raise SystemExit(f"无法打开摄像头：{args.device}")

    try:
        set_results = apply_capture_format(cap, args)
        print("参数设置结果：" + " ".join(f"{name}={'OK' if ok else '失败'}" for name, ok in set_results.items()))

        for name, value in controls_to_set:
            set_control(args.device, name, value)

        control_info = list_controls(args.device)
        control_names = set(CONTROL_ALIASES.values()) | {
            "auto_exposure",
            "white_balance_automatic",
            "focus_automatic_continuous",
        }
        control_values = get_controls(args.device, sorted(control_names))
        v4l2_info = query_camera_info(args.device)
        capture_info = capture_properties(cap)
        print(f"请求格式：{args.format} {args.width}x{args.height}@{args.fps}")
        print(f"OpenCV实际：{capture_info.summary()}")
        print(f"V4L2实际：{v4l2_info.summary()}")
        if not same_mode(v4l2_info, capture_info):
            print("提示：两个后端的显示精度不同，V4L2实际值通常更可信。")
        print("按 h 查看按键；q/Esc 退出；s 保存当前原始帧。")

        window = "Camera Parameter Test"
        worker: CaptureWorker | None = None
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window, args.display_width, args.display_height)
        worker = CaptureWorker(cap)
        worker.start()

        capture_fps = 0.0
        display_fps = 0.0
        capture_count_at_start = 0
        display_count = 0
        capture_fps_start = time.monotonic()
        display_fps_start = capture_fps_start
        last_quality_time = 0.0
        next_display_time = 0.0
        quality: tuple[float, float, float] | None = None
        last_frame: np.ndarray | None = None
        last_frame_id = 0
        selected_control = "exposure_time_absolute"
        status_until = 0.0
        status_text = ""

        while True:
            now = time.monotonic()
            captured, failed = worker.stats()
            capture_elapsed = now - capture_fps_start
            if capture_elapsed >= 1.0:
                capture_fps = (captured - capture_count_at_start) / capture_elapsed
                capture_count_at_start = captured
                capture_fps_start = now
            display_elapsed = now - display_fps_start
            if display_elapsed >= 1.0:
                display_fps = display_count / display_elapsed
                display_count = 0
                display_fps_start = now

            frame_id, frame, frame_time = worker.latest()
            if frame is None:
                status_text = "尚未读到画面；请检查摄像头格式/帧率组合。"
                status_until = now + 3.0
                key = cv2.waitKey(10) & 0xFF
                if key in (ord("q"), 27):
                    break
                continue

            last_frame = frame
            if frame_id != last_frame_id and now - last_quality_time >= 0.2:
                quality = quality_metrics(frame)
                last_quality_time = now
                last_frame_id = frame_id

            if now >= next_display_time:
                display = frame.copy()
                frame_age_ms = max(0.0, (now - frame_time) * 1000.0)
                lines = [
                    f"request: {args.format} {args.width}x{args.height}@{args.fps}",
                    f"source:   {capture_info.summary()}",
                    f"capture={capture_fps:.1f}fps  window={display_fps:.1f}fps  age={frame_age_ms:.1f}ms failed={failed}",
                ]
                if quality is not None:
                    mean_luma, clipped_ratio, sharpness = quality
                    lines.append(
                        f"luma={mean_luma:.1f} clipped={clipped_ratio:.3f} sharpness={sharpness:.1f}"
                    )
                shown_controls = [
                    ("exp", "exposure_time_absolute"),
                    ("focus", "focus_absolute"),
                    ("wb", "white_balance_temperature"),
                    ("sharp", "sharpness"),
                ]
                control_line = " ".join(
                    f"{label}={control_values[name]}"
                    for label, name in shown_controls
                    if name in control_values
                )
                if control_line:
                    lines.append(control_line)
                lines.extend(
                    [
                        f"selected={dict(SELECTABLE_CONTROLS.values()).get(selected_control, selected_control)}",
                        "1 exp 2 focus 3 white-balance 4 sharpness 5 brightness",
                        "6 contrast 7 saturation 8 gamma   [/] or -/= decrease/increase",
                        "p print  s save  h help  q/Esc quit",
                    ]
                )
                if status_text and now < status_until:
                    lines.append(status_text)
                elif status_text:
                    status_text = ""

                for index, line in enumerate(lines):
                    y = 28 + index * 25
                    cv2.putText(
                        display,
                        line,
                        (12, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.62,
                        (0, 255, 0) if index < 3 else (0, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )

                cv2.imshow(window, fit_for_display(display, args.display_width, args.display_height))
                next_display_time = now + 1.0 / args.display_fps
                display_count += 1
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("h"):
                print(
                    "按键：1-8选择参数，[ 或 - 减小，] 或 = 增大；"
                    "小写 e/f/w/k/b/c/v/g 仍可直接减小对应参数；p打印，s保存。"
                )
            elif key == ord("p"):
                v4l2_info = query_camera_info(args.device)
                print_status(args, cap, v4l2_info, control_values, capture_fps, display_fps, quality)
            elif key == ord("s"):
                if last_frame is not None:
                    saved = save_frame(last_frame.copy(), args.save_dir)
                    if saved is not None:
                        status_text = f"已保存：{saved}"
                        status_until = time.monotonic() + 3.0
                        print(status_text)
            elif key in SELECTABLE_CONTROLS:
                selected_control, label = SELECTABLE_CONTROLS[key]
                status_text = f"已选择：{label}；使用 [/] 或 -/= 调整"
                status_until = time.monotonic() + 2.0
            elif key in ADJUSTMENT_KEYS or key in LEGACY_DECREASE_KEYS:
                if key in ADJUSTMENT_KEYS:
                    name = selected_control
                    direction = ADJUSTMENT_KEYS[key]
                    delta = CONTROL_DELTAS.get(name, 1) * direction
                else:
                    name, delta = LEGACY_DECREASE_KEYS[key]
                current = control_values.get(name)
                if current is None:
                    status_text = f"摄像头不支持控制：{name}"
                    status_until = time.monotonic() + 3.0
                    print(status_text)
                    continue
                target = clamp_control(current + delta, control_info.get(name))
                if set_control(args.device, name, target):
                    actual = get_control(args.device, name)
                    if actual is not None:
                        control_values[name] = actual
                    status_text = f"{name}={control_values.get(name, target)}"
                    status_until = time.monotonic() + 2.0
                    print(status_text)
    finally:
        if worker is not None:
            worker.stop()
        cap.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
