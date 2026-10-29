#!/usr/bin/env python3
"""Interactive calibration map using the existing F407 MotionDebug protocol.

The F407 motion-debug branch has a different protocol from the competition
task firmware: TYPE=0x19 is a one-shot motion command and TYPE=0x1A is the
motion status.  This program owns the real F407 UART, forwards its ODOM bytes
through a pseudo-terminal to the existing T265 localizer, and keeps the full
T265/encoder CSV in the same run directory.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import pty
import re
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import serial
from PIL import Image, ImageDraw, ImageFont

TOOLS_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = TOOLS_ROOT.parents[1]
LOCALIZATION_ROOT = PROJECT_ROOT / "localization"
DEFAULT_CONFIG = LOCALIZATION_ROOT / "config" / "t265_debug_new_mount_candidate.conf"
DEFAULT_LOCALIZER_SCRIPT = LOCALIZATION_ROOT / "run_localization.sh"
DEFAULT_RUN_ROOT = PROJECT_ROOT / "rescue_map" / "runtime" / "history" / "t265_f407_motion_map"
VISION_ROOT = PROJECT_ROOT / "vision"
if str(VISION_ROOT) not in sys.path:
    sys.path.insert(0, str(VISION_ROOT))
if str(TOOLS_ROOT) not in sys.path:
    sys.path.insert(0, str(TOOLS_ROOT))

from f407_motion_protocol import (  # noqa: E402
    COMMAND_NAMES,
    MOTION_CMD_MOVE_DISTANCE,
    MOTION_CMD_STOP,
    MOTION_CMD_TURN_REL,
    MOTION_TURN_TIMEOUT_S,
    MOTION_STATE_DONE,
    MOTION_STATE_FAULT,
    MOTION_STATE_RUNNING,
    MOTION_STATE_STOPPED,
    STATE_NAMES,
    StreamParser,
    motion_move_frame,
    motion_stop_frame,
    motion_turn_frame,
)
from t265_f407_debug import PROTOCOL, analyze, write_debug_config  # noqa: E402


WINDOW_NAME = "t265_f407_motion_map"
FONT_REGULAR = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
FONT_MEDIUM = "/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc"
LOGICAL_WIDTH, LOGICAL_HEIGHT = 1280, 1024
FIELD_HALF_M = 1.5
START_HEADING_OFFSET_DEG = 45.0
DEBUG_COMMAND_FORWARD_SIGN = 1
DEBUG_COMMAND_LEFT_SIGN = 1
DEBUG_COMMAND_TURN_SIGN = -1
LOWER_FIRMWARE = "kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot@87aaba26f2c028680027b54df08d6ba825b047e9 (MotionDebug)"
MOTION_STATUS_TIMEOUT_MS = 1000
MOTION_PROTOCOL = dict(PROTOCOL, motion_command_type="0x19", motion_status_type="0x1A",
                       turn_angle_unit="signed 0.1 degree", move_arguments="physical forward/left mm",
                       wheel_body_axes="F=-(m1-m2)/sqrt(3), L=(m1+m2-2*m3)/3; signed wheel metres",
                       t265_heading="CCW positive, selected field start reference, wrapped -180..180 degrees",
                       motion_status_heading="F407 Location heading, CCW positive, local reset reference, modulo 360; not raw IMU yaw")


def read_json(path: Path | None) -> dict | None:
    if path is None:
        return None
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else None
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return None


def start_pose(zone: int) -> tuple[float, float, float]:
    headings = {1: 135.0, 2: 45.0, 3: 225.0, 4: 315.0}
    return 0.0, 0.0, (headings[zone] + START_HEADING_OFFSET_DEG) % 360.0


def config_values(path: Path) -> dict[str, str]:
    values = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if "=" in line:
            key, value = line.split("=", 1)
            values[key.strip()] = value.strip()
    return values


def wrap_degrees(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


SESSION_FILES = frozenset({"metadata.json", "localization.conf", "localization_result.json",
    "localization_debug.csv", "t265_raw_pose.csv", "localization_events.jsonl", "localizer.log",
    "f407_motion_status.json", "f407_motion_status.jsonl", "uart_rx.jsonl", "events.jsonl",
    "commands.jsonl", "analysis.json", "ground_truth.jsonl", "session_complete.json"})


def prune_completed_sessions(root: Path, current: Path, keep: int = 5) -> list[str]:
    """Only delete known files of explicitly completed sessions owned here.

    Legacy sessions need a terminal session_stopped event as completion proof.
    Active, unproven, symlinked, and user-extended directories are kept.
    Called after saving the current completed session, never on startup.
    """
    current_marker = read_json(current / "session_complete.json")
    if (not current_marker or current_marker.get("owner") != "t265_f407_motion_map" or
            current_marker.get("complete") is not True):
        return []
    completed = []
    for path in root.iterdir():
        if (path.is_symlink() or not path.is_dir() or
                not re.fullmatch(r"\d{8}_\d{6}_interactive(?:_\d+)?", path.name)):
            continue
        marker = read_json(path / "session_complete.json")
        metadata = read_json(path / "metadata.json")
        if (not metadata or
                metadata.get("mode") != "interactive_f407_motion_debug"):
            continue
        if marker:
            if marker.get("owner") != "t265_f407_motion_map" or marker.get("complete") is not True:
                continue
            index = marker.get("session_index")
            if not isinstance(index, int):
                continue
            order = (1, index)
        else:
            try:
                with (path / "events.jsonl").open("rb") as stream:
                    stream.seek(max(0, os.fstat(stream.fileno()).st_size - 65536))
                    last = json.loads(stream.read().splitlines()[-1])
                if last.get("event") != "session_stopped":
                    continue
                order = (0, (path / "events.jsonl").stat().st_mtime_ns)
            except (OSError, ValueError, IndexError, AttributeError):
                continue
        children = list(path.iterdir())
        if any(p.is_symlink() or not p.is_file() or p.name not in SESSION_FILES for p in children):
            continue
        completed.append((order, path))
    # Include current in the five, even if the system wall clock moved backwards.
    others = sorted(((order, p) for order, p in completed if p.resolve() != current.resolve()),
                    key=lambda item: (item[0], item[1].name), reverse=True)
    removed = []
    for _, path in others[max(0, keep - 1):]:
        # Recheck immediately before deleting, and never recursively remove.
        children = list(path.iterdir())
        if any(p.is_symlink() or not p.is_file() or p.name not in SESSION_FILES for p in children):
            continue
        for child in children:
            child.unlink()
        path.rmdir()
        removed.append(path.name)
    return removed


class TextPainter:
    def __init__(self, font_scale: float = 1.0) -> None:
        self.font_scale = max(1.0, float(font_scale))
        self.items: list[tuple] = []
        self.fonts: dict[tuple[int, bool], ImageFont.FreeTypeFont] = {}

    def add(self, value, xy, size=18, color=(230, 230, 230), bold=False, anchor="la", max_width=None) -> None:
        self.items.append((str(value), tuple(map(int, xy)),
                           max(12, int(round(size * self.font_scale))),
                           color, bold, anchor, max_width))

    def paint(self, canvas: np.ndarray) -> np.ndarray:
        image = Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(image)
        for value, xy, size, color, bold, anchor, max_width in self.items:
            if max_width is None and anchor.startswith("l"):
                max_width = image.width - xy[0] - 12
            while True:
                key = (size, bold)
                if key not in self.fonts:
                    path = FONT_MEDIUM if bold else FONT_REGULAR
                    try:
                        self.fonts[key] = ImageFont.truetype(path, size=size)
                    except OSError:
                        self.fonts[key] = ImageFont.load_default()
                if max_width is None or draw.textlength(value, font=self.fonts[key]) <= max_width or size <= 9:
                    break
                size -= 1
            draw.text(xy, value, font=self.fonts[key],
                      fill=(color[2], color[1], color[0]), anchor=anchor)
        self.items.clear()
        return cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)


class F407SerialBridge:
    """Read the real UART, parse motion telemetry, and feed ODOM to localizer."""

    def __init__(self, device: str, baud: int, status_path: Path,
                 status_log_path: Path) -> None:
        self.device = device
        self.baud = baud
        self.status_path = status_path
        self.status_log_path = status_log_path
        self.master_fd, self.slave_fd = pty.openpty()
        os.set_blocking(self.master_fd, False)
        self.slave_path = os.ttyname(self.slave_fd)
        self.serial: serial.Serial | None = None
        self.parser = StreamParser(on_motion_status=self._on_motion_status)
        self.latest_status: dict | None = None
        self.latest_status_timestamp_ns = 0
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.rx_log = None
        self.log_error: str | None = None

    def _on_motion_status(self, status: dict) -> None:
        timestamp_ns = time.monotonic_ns()
        document = dict(status)
        document["timestamp_monotonic_ns"] = timestamp_ns
        with self.lock:
            self.latest_status = document
            self.latest_status_timestamp_ns = timestamp_ns
        temporary = self.status_path.with_suffix(self.status_path.suffix + ".tmp")
        try:
            temporary.write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n",
                                 encoding="utf-8")
            temporary.replace(self.status_path)
            with self.status_log_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(document, ensure_ascii=False) + "\n")
        except OSError as error:
            self.log_error = str(error)

    def _forward_to_pty(self, data: bytes) -> None:
        try:
            os.write(self.master_fd, data)
        except (BlockingIOError, OSError):
            # The localizer may still be booting. Never let a full pseudo-
            # terminal buffer stop reading the real F407 UART.
            return

    def _read_loop(self) -> None:
        assert self.serial is not None
        while not self.stop_event.is_set():
            try:
                data = self.serial.read(512)
            except serial.SerialException:
                return
            if not data:
                continue
            # Retain the complete receive stream even if T265 never boots or
            # a frame fails CRC/decoding. Host monotonic time aligns it to CSV.
            if self.rx_log is not None:
                try:
                    self.rx_log.write(json.dumps({
                        "timestamp_monotonic_ns": time.monotonic_ns(),
                        "data_hex": data.hex(),
                    }) + "\n")
                except OSError as error:
                    self.log_error = str(error)
            self.parser.feed(data)
            self._forward_to_pty(data)

    def start(self) -> None:
        self.rx_log = (self.status_path.parent / "uart_rx.jsonl").open(
            "a", encoding="utf-8", buffering=1)
        self.serial = serial.Serial(
            self.device, self.baud, timeout=0.05, write_timeout=0.20
        )
        self.thread = threading.Thread(target=self._read_loop,
                                       name="f407-motion-uart", daemon=True)
        self.thread.start()

    def send(self, packet: bytes) -> None:
        with self.lock:
            if self.serial is None:
                raise RuntimeError("F407串口尚未打开")
            try:
                self.serial.write(packet)
                self.serial.flush()
            except serial.SerialException as error:
                raise RuntimeError(f"F407串口发送失败：{error}") from error

    def status_snapshot(self) -> tuple[dict | None, dict]:
        with self.lock:
            return self.latest_status, dict(self.parser.stats)

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        if self.serial is not None:
            self.serial.close()
        self.thread = None
        self.serial = None
        if self.rx_log is not None:
            self.rx_log.close()
            self.rx_log = None
        for fd in (self.master_fd, self.slave_fd):
            try:
                os.close(fd)
            except OSError:
                pass


class MotionMapApp:
    def __init__(self, options: argparse.Namespace) -> None:
        self.options = options
        self.screen_width, self.screen_height = self.detect_screen_size()
        self.render_scale = min(1.0, self.screen_width / LOGICAL_WIDTH,
                                self.screen_height / LOGICAL_HEIGHT)
        self.content_width = max(1, round(LOGICAL_WIDTH * self.render_scale))
        self.content_height = max(1, round(LOGICAL_HEIGHT * self.render_scale))
        self.content_offset_x = max(0, (self.screen_width - self.content_width) // 2)
        self.content_offset_y = max(0, (self.screen_height - self.content_height) // 2)
        self.map_left, self.map_top, self.map_size = 30, 60, 850
        self.zone = options.zone
        self.hitboxes: dict[str, tuple[int, int, int, int]] = {}
        self.message = "启动后仅监听，点击实验按钮发送运动指令"
        self.run_dir: Path | None = None
        self.localization_json: Path | None = None
        self.motion_status_path: Path | None = None
        self.bridge: F407SerialBridge | None = None
        self.localizer: subprocess.Popen | None = None
        self.localizer_log = None
        self.events_path: Path | None = None
        self.commands_path: Path | None = None
        self.command_sequence = 0
        self.last_sent_command: dict | None = None
        self.last_json_timestamp = 0
        self.last_status_timestamp = 0
        self.localization: dict = {}
        self.motion_status: dict = {}
        self.t265_points: list[tuple[float, float]] = []
        self.odom_points: list[tuple[float, float]] = []
        self.raw_points: list[tuple[float, float]] = []
        self.active_test: str | None = None
        self.active_command_sequence: int | None = None
        self.active_command_opcode: int | None = None
        self.active_command_sent_ns = 0
        self.last_matching_status_ns = 0
        self.return_phase: str | None = None
        self.return_distance_mm = 0
        self.distance_mm = 1000
        self.speed_mm_s = 300
        self.test_id = 0
        self.active_test_parameters: dict = {}
        self.session_config: dict[str, str] = {}
        self.measurement_buffer: str | None = None
        self.session_index = 0
        self.logging_failed = False

    @staticmethod
    def detect_screen_size() -> tuple[int, int]:
        try:
            output = subprocess.check_output(
                ["xrandr", "--current"], text=True, stderr=subprocess.DEVNULL
            )
            import re
            match = re.search(r"current\s+(\d+)\s+x\s+(\d+)", output)
            if match:
                return int(match.group(1)), int(match.group(2))
        except (OSError, subprocess.SubprocessError, ValueError):
            pass
        return LOGICAL_WIDTH, LOGICAL_HEIGHT

    def world_to_pixel(self, x_m: float, y_m: float) -> tuple[int, int]:
        return (
            int(round(self.map_left + (x_m + FIELD_HALF_M) / 3.0 * self.map_size)),
            int(round(self.map_top + (FIELD_HALF_M - y_m) / 3.0 * self.map_size)),
        )

    def world_rect(self, canvas, x0, y0, x1, y1, color, thickness=-1) -> None:
        left, bottom = self.world_to_pixel(min(x0, x1), min(y0, y1))
        right, top = self.world_to_pixel(max(x0, x1), max(y0, y1))
        cv2.rectangle(canvas, (left, top), (right, bottom), color, thickness, cv2.LINE_AA)

    def draw_field(self, canvas: np.ndarray, text: TextPainter) -> None:
        ml, mt, size = self.map_left, self.map_top, self.map_size
        cv2.rectangle(canvas, (ml, mt), (ml + size, mt + size), (218, 218, 218), -1)
        for coordinate in np.arange(-1.0, 1.01, 0.5):
            cv2.line(canvas, self.world_to_pixel(coordinate, -1.5),
                     self.world_to_pixel(coordinate, 1.5), (190, 190, 190), 1)
            cv2.line(canvas, self.world_to_pixel(-1.5, coordinate),
                     self.world_to_pixel(1.5, coordinate), (190, 190, 190), 1)
        for y0, y1, color, label in (
            (1.14, 1.50, (70, 70, 185), "红方安全区"),
            (-1.50, -1.14, (190, 125, 25), "蓝方安全区"),
        ):
            self.world_rect(canvas, -0.33, y0, 0.33, y1, (120, 45, 145), -1)
            inner_y0, inner_y1 = (1.20, 1.50) if y0 > 0 else (-1.50, -1.20)
            self.world_rect(canvas, -0.30, inner_y0, 0.30, inner_y1, color, -1)
            text.add(label, self.world_to_pixel(0.0, (y0 + y1) / 2), 17, (245, 245, 245), True, "mm")
        zones = {
            1: (-1.50, 1.20, -1.20, 1.50),
            2: (1.20, 1.20, 1.50, 1.50),
            3: (-1.50, -1.50, -1.20, -1.20),
            4: (1.20, -1.50, 1.50, -1.20),
        }
        for zone, rect in zones.items():
            self.world_rect(canvas, *rect, (215, 35, 220), -1)
            self.world_rect(canvas, *rect, (80, 30, 80), 2)
            text.add(zone, self.world_to_pixel((rect[0] + rect[2]) / 2,
                                               (rect[1] + rect[3]) / 2), 32, (20, 20, 20), True, "mm")
        cv2.rectangle(canvas, (ml, mt), (ml + size, mt + size), (35, 35, 35), 4)
        origin = self.world_to_pixel(0.0, 0.0)
        cv2.line(canvas, (origin[0] - 10, origin[1]), (origin[0] + 10, origin[1]), (70, 70, 70), 2)
        cv2.line(canvas, (origin[0], origin[1] - 10), (origin[0], origin[1] + 10), (70, 70, 70), 2)
        text.add("测试起点 (0,0)", (origin[0] + 30, origin[1] - 28), 15, (70, 70, 70))
        text.add("T265中心", (ml + 20, mt + size - 28), 15, (20, 150, 245), True)
        text.add("轮式中心", (ml + 150, mt + size - 28), 15, (220, 80, 220), True)
        text.add("raw origin", (ml + 280, mt + size - 28), 15, (50, 170, 80), True)

    def draw_path(self, canvas: np.ndarray, points: list[tuple[float, float]], color, width: int) -> None:
        if len(points) >= 2:
            pixels = np.asarray([self.world_to_pixel(x, y) for x, y in points], dtype=np.int32)
            cv2.polylines(canvas, [pixels], False, color, width, cv2.LINE_AA)

    def draw_trajectories(self, canvas: np.ndarray) -> None:
        self.draw_path(canvas, self.t265_points, (20, 150, 245), 3)
        self.draw_path(canvas, self.odom_points, (220, 80, 220), 2)
        self.draw_path(canvas, self.raw_points, (50, 170, 80), 2)

    def draw_robot(self, canvas: np.ndarray) -> None:
        pose = self.localization.get("pose", {})
        try:
            x, y, yaw = float(pose["x_m"]), float(pose["y_m"]), float(pose["yaw_deg"])
        except (KeyError, TypeError, ValueError):
            x, y, yaw = start_pose(self.zone)
        center = self.world_to_pixel(x, y)
        tip = self.world_to_pixel(x + 0.24 * math.cos(math.radians(yaw)),
                                  y + 0.24 * math.sin(math.radians(yaw)))
        cv2.circle(canvas, center, 22, (25, 25, 25), -1, cv2.LINE_AA)
        cv2.circle(canvas, center, 19, (20, 150, 245), 3, cv2.LINE_AA)
        cv2.arrowedLine(canvas, center, tip, (20, 150, 245), 5, cv2.LINE_AA, tipLength=0.30)
        odom = self.localization.get("wheel_odom", {})
        try:
            ox, oy = float(odom["x_m"]), float(odom["y_m"])
            odom_center = self.world_to_pixel(ox, oy)
            cv2.line(canvas, center, odom_center, (115, 115, 115), 1, cv2.LINE_AA)
            cv2.circle(canvas, odom_center, 9, (220, 80, 220), 2, cv2.LINE_AA)
        except (KeyError, TypeError, ValueError):
            pass
        tracking = self.localization.get("t265", {}).get("tracking_origin", {})
        try:
            raw_center = self.world_to_pixel(float(tracking["x_m"]), float(tracking["y_m"]))
            cv2.circle(canvas, raw_center, 7, (50, 170, 80), 2, cv2.LINE_AA)
            cv2.line(canvas, center, raw_center, (50, 170, 80), 1, cv2.LINE_AA)
        except (KeyError, TypeError, ValueError):
            pass

    def button(self, canvas, text: TextPainter, name: str, label: str,
               rect: tuple[int, int, int, int], selected=False, color=(65, 65, 70)) -> None:
        self.hitboxes[name] = rect
        x0, y0, x1, y1 = rect
        cv2.rectangle(canvas, (x0, y0), (x1, y1), color if selected else (45, 45, 48), -1)
        cv2.rectangle(canvas, (x0, y0), (x1, y1), (0, 225, 255) if selected else (105, 105, 110), 2)
        text.add(label, ((x0 + x1) // 2, (y0 + y1) // 2), 16, (245, 245, 245), selected, "mm",
                 max_width=x1-x0-8)

    def draw_panel(self, canvas: np.ndarray, text: TextPainter) -> None:
        x0 = 920
        cv2.rectangle(canvas, (900, 0), (LOGICAL_WIDTH, LOGICAL_HEIGHT), (26, 27, 30), -1)
        text.add("T265/F407 运动调试", (x0, 12), 22, (245, 245, 245), True)
        running = self.localizer is not None and self.localizer.poll() is None
        text.add("定位进程运行中" if running else "定位进程未运行", (x0, 68), 16,
                 (50, 220, 65) if running else (50, 80, 230), True)
        text.add(f"测试起点：(0,0)  初始航向 {start_pose(self.zone)[2]:.0f}°", (x0, 96), 15, (220, 220, 220))
        forward = self.session_config.get("camera_robot_forward_axis", "?")
        up = self.session_config.get("camera_robot_up_axis", "?")
        text.add(f"前向候选：{forward}", (x0, 123), 15, (220, 220, 220))
        up_world = self.localization.get("t265", {}).get("up_world")
        up_text = "待采集" if up_world is None else f"世界上向Y={up_world[1]:+.3f}"
        text.add(f"上向：{up} / {up_text}", (x0, 149), 13, (175, 175, 180))
        pose = self.localization.get("pose", {})
        try:
            text.add(f"T265中心：{float(pose['x_m']):+.3f} / {float(pose['y_m']):+.3f} m",
                     (x0, 180), 17, (20, 170, 245), True)
            text.add(f"T265方向：{float(pose['yaw_deg']):06.2f}°",
                     (x0, 207), 17, (20, 170, 245), True)
        except (KeyError, TypeError, ValueError):
            text.add("T265：等待数据", (x0, 190), 17, (170, 170, 175))
        odom = self.localization.get("wheel_odom", {})
        try:
            text.add(f"轮式中心：{float(odom['x_m']):+.3f} / {float(odom['y_m']):+.3f} m",
                     (x0, 240), 16, (220, 80, 220), True)
        except (KeyError, TypeError, ValueError):
            text.add("轮式中心：等待F407编码器", (x0, 240), 16, (180, 130, 180), True)
        comparison = self.localization.get("comparison", {})
        error_m = comparison.get("t265_vs_wheel_odom_distance_m")
        comparison_text = "T265-轮式：等待数据" if error_m is None else f"T265-轮式：{float(error_m):.3f} m"
        text.add(comparison_text,
                 (x0, 266), 16, (0, 205, 220))
        t265 = self.localization.get("t265", {})
        text.add(f"raw偏置：{float(t265.get('camera_offset_forward_m', 0.0)):+.4f} / {float(t265.get('camera_offset_left_m', 0.0)):+.4f} m",
                 (x0, 292), 15, (50, 170, 80))
        text.add(f"raw增量：{float(t265.get('tracking_origin_delta_forward_m', 0.0)):+.3f} / {float(t265.get('tracking_origin_delta_left_m', 0.0)):+.3f} m",
                 (x0, 316), 15, (50, 170, 80))

        status = self.motion_status
        state = int(status.get("state", 0)) if status else 0
        command = int(status.get("command", 0)) if status else 0
        fault = int(status.get("fault", 0)) if status else 0
        status_age = self.status_age(status)
        status_text = (f"F407：{STATE_NAMES.get(state, 'UNKNOWN')} / {COMMAND_NAMES.get(command, 'UNKNOWN')} age={status_age:.0f}ms"
                       if status else "F407：等待0x1A运动状态")
        text.add(status_text,
                 (x0, 350), 15, (245, 215, 150), True)
        unit = "0.1°" if command == MOTION_CMD_TURN_REL else "mm"
        heading = float(status.get("heading_cdeg", 0)) * 0.01
        progress_text = (f"进度={int(status.get('progress', 0))}{unit}  故障={fault}  IMU={heading:.1f}°"
                         if status else "运动进度及IMU航向：等待数据")
        text.add(progress_text,
                 (x0, 374), 15, (245, 215, 150))
        stats = self.bridge.status_snapshot()[1] if self.bridge else {}
        text.add(f"F407帧：ODOM={stats.get('odom_frames', 0)} STATUS={stats.get('motion_status_frames', 0)}",
                 (x0, 400), 14, (190, 190, 200))
        text.add(f"CRC={stats.get('crc_errors', 0)} 丢帧={stats.get('odom_sequence_gaps', 0)}/{stats.get('status_sequence_gaps', 0)}",
                 (x0, 423), 14, (190, 190, 200))

        text.add(f"参数：距离 {self.distance_mm} mm / 速度 {self.speed_mm_s} mm/s", (x0, 448), 16, (245, 245, 245), True)
        self.button(canvas, text, "dminus", "距离-100", (x0, 480, x0 + 78, 514))
        self.button(canvas, text, "dplus", "距离+100", (x0 + 84, 480, x0 + 162, 514))
        self.button(canvas, text, "sminus", "速度-50", (x0 + 168, 480, x0 + 246, 514))
        self.button(canvas, text, "splus", "速度+50", (x0 + 252, 480, x0 + 330, 514))
        self.button(canvas, text, "dfineminus", "细调-10mm", (x0, 520, x0 + 158, 550))
        self.button(canvas, text, "dfineplus", "细调+10mm", (x0 + 172, 520, x0 + 330, 550))

        self.button(canvas, text, "stop", "STOP", (x0, 558, x0 + 330, 590), False, (100, 55, 55))
        for col, angle in enumerate((90, 180, 360)):
            for row, sign in enumerate((1, -1)):
                value = sign * angle
                name = f"turn_{value}"
                self.button(canvas, text, name, f"{value:+d}°",
                            (x0 + col * 112, 595 + row * 37, x0 + col * 112 + 106, 627 + row * 37),
                            self.active_test == f"rotate_{value}deg", (75, 75, 105))
        for col, (name, label) in enumerate((("forward", "前进"), ("backward", "后退"),
                                            ("lateral", "左移"), ("right", "右移"))):
            self.button(canvas, text, name, label, (x0 + col * 84, 672, x0 + col * 84 + 78, 704),
                        self.active_test == name, (55, 110, 80))
        self.button(canvas, text, "returntest", "转向返航（非卷尺基准）", (x0, 711, x0 + 330, 741),
                    self.active_test == "turn_and_return", (55, 100, 135))
        command = self.last_sent_command
        text.add(f"最近发送：{command['name']} seq={command['sequence']}" if command else "最近发送：无",
                 (x0, 765), 15, (245, 215, 150))
        if command:
            text.add(command["description"], (x0, 788), 13, (220, 220, 220))
        text.add(f"实验{self.test_id}：{self.active_test or '无'} / {self.return_phase or ''}",
                 (x0, 814), 16, (0, 215, 255), True)
        text.add("V/B前后 L/H左右 O/P/0正转 1/2/3反转", (x0, 843), 13, (175, 175, 180))
        text.add("I/K距离±100 ,/.±10 U/J速度 S停车 Q退出", (x0, 865), 13, (175, 175, 180))
        text.add("M记录停稳点；G录入独立实测；Y转向返航", (x0, 887), 13, (175, 175, 180))
        text.add("指令目标和DONE不代表实测距离/转角", (x0, 909), 13, (175, 175, 180))
        text.add(f"日志：{self.run_dir.name if self.run_dir else '未启动'}", (x0, 931), 12, (175, 175, 180))
        if self.measurement_buffer is not None:
            text.add(f"测量 f,l,deg: {self.measurement_buffer}_", (x0, 954), 14, (100, 240, 240))
        text.add(self.message, (x0, 986), 12, (0, 215, 255))

    def render_logical(self) -> np.ndarray:
        canvas = np.zeros((LOGICAL_HEIGHT, LOGICAL_WIDTH, 3), dtype=np.uint8)
        text = TextPainter(font_scale=1.0 / self.render_scale)
        self.hitboxes.clear()
        self.draw_field(canvas, text)
        self.draw_trajectories(canvas)
        self.draw_robot(canvas)
        self.draw_panel(canvas, text)
        return text.paint(canvas)

    def render_screen(self) -> np.ndarray:
        logical = self.render_logical()
        resized = cv2.resize(logical, (self.content_width, self.content_height),
                             interpolation=cv2.INTER_AREA if self.render_scale < 1.0 else cv2.INTER_LINEAR)
        screen = np.zeros((self.screen_height, self.screen_width, 3), dtype=np.uint8)
        screen[self.content_offset_y:self.content_offset_y + self.content_height,
               self.content_offset_x:self.content_offset_x + self.content_width] = resized
        return screen

    def status_age(self, status: dict | None) -> float:
        if not status:
            return float("inf")
        try:
            return max(0.0, (time.monotonic_ns() - int(status["timestamp_monotonic_ns"])) / 1_000_000.0)
        except (KeyError, TypeError, ValueError):
            return float("inf")

    def append_point(self, points: list[tuple[float, float]], x, y) -> None:
        try:
            point = (float(x), float(y))
        except (TypeError, ValueError):
            return
        if not all(math.isfinite(value) for value in point):
            return
        if not points or math.hypot(point[0] - points[-1][0], point[1] - points[-1][1]) >= 0.002:
            points.append(point)
            if len(points) > 12000:
                del points[:2000]

    def update_data(self) -> None:
        if self.localization_json is not None:
            data = read_json(self.localization_json)
            if data is not None:
                timestamp = int(data.get("timestamp_monotonic_ns", 0))
                if timestamp != self.last_json_timestamp:
                    self.last_json_timestamp = timestamp
                    self.localization = data
                    self.append_point(self.t265_points, data.get("pose", {}).get("x_m"), data.get("pose", {}).get("y_m"))
                    self.append_point(self.odom_points, data.get("wheel_odom", {}).get("x_m"), data.get("wheel_odom", {}).get("y_m"))
                    tracking = data.get("t265", {}).get("tracking_origin", {})
                    self.append_point(self.raw_points, tracking.get("x_m"), tracking.get("y_m"))
        if self.bridge is not None:
            if self.bridge.log_error is not None:
                self.message = f"测试日志保存失败：{self.bridge.log_error}"
            status, _ = self.bridge.status_snapshot()
            if status is not None:
                timestamp = int(status.get("timestamp_monotonic_ns", 0))
                if timestamp != self.last_status_timestamp:
                    self.last_status_timestamp = timestamp
                    self.motion_status = status
                    self.handle_motion_status(status)
        if (self.active_test is not None and self.active_command_sequence is not None and
                time.monotonic_ns() - self.last_matching_status_ns > MOTION_STATUS_TIMEOUT_MS * 1_000_000):
            self.record_event(f"{self.active_test}_end_status_lost")
            self.active_test = None
            self.return_phase = None
            self.active_command_sequence = None
            self.message = "F407状态失联，已取消自动后续动作；当前机械动作状态未知"

    def record_event(self, name: str, details: dict | None = None) -> bool:
        if self.events_path is None:
            return False
        event = {
            "timestamp_monotonic_ns": time.monotonic_ns(),
            "wall_time": datetime.now().isoformat(timespec="milliseconds"),
            "event": name,
            "details": details or {},
            "test_id": self.test_id,
            "test_parameters": dict(self.active_test_parameters),
            "localization": {
                "timestamp_monotonic_ns": self.localization.get("timestamp_monotonic_ns"),
                "pose": self.localization.get("pose"),
                "t265": self.localization.get("t265", {}),
                "wheel_odom": self.localization.get("wheel_odom", {}),
            },
            "motion_status": self.motion_status,
        }
        try:
            with self.events_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(event, ensure_ascii=False) + "\n")
        except OSError:
            self.logging_failed = True
            return False
        return True

    def record_command(self, name: str, sequence: int, packet: bytes, description: str) -> None:
        self.last_sent_command = {
            "name": name,
            "sequence": sequence,
            "description": description,
        }
        if self.commands_path is not None:
            document = {
                "timestamp_monotonic_ns": time.monotonic_ns(),
                "name": name,
                "sequence": sequence,
                "description": description,
                "frame_hex": packet.hex(" ").upper(),
                "test_id": self.test_id,
                "test_parameters": dict(self.active_test_parameters),
                "wire": {
                    "type": packet[2], "opcode": packet[4], "flags": packet[5],
                    "arg1": int.from_bytes(packet[6:8], "big", signed=True),
                    "arg2": int.from_bytes(packet[8:10], "big", signed=True),
                    "speed": int.from_bytes(packet[10:12], "big"),
                },
            }
            try:
                with self.commands_path.open("a", encoding="utf-8") as stream:
                    stream.write(json.dumps(document, ensure_ascii=False) + "\n")
            except OSError:
                self.logging_failed = True

    def send_packet(self, packet: bytes, name: str, description: str) -> int:
        if packet[4] != MOTION_CMD_STOP and not self.motion_identity_confirmed():
            self.message = "未确认实际固件和MotionDebug模式，未发送运动指令"
            return -1
        if self.bridge is None:
            self.message = "F407串口尚未连接"
            return -1
        sequence = self.command_sequence & 0xFF
        self.command_sequence = (self.command_sequence + 1) & 0xFF
        sent_ns = time.monotonic_ns()
        self.bridge.send(packet)
        self.active_command_sequence = sequence
        self.active_command_opcode = packet[4]
        self.active_command_sent_ns = sent_ns
        self.last_matching_status_ns = sent_ns
        self.record_command(name, sequence, packet, description)
        return sequence

    def send_stop(self, reason: str = "手动STOP") -> None:
        sequence = self.send_packet(
            motion_stop_frame(self.command_sequence & 0xFF), "STOP", reason
        )
        if sequence >= 0:
            self.active_command_sequence = None
            self.message = f"已发送 STOP（seq={sequence}）"

    def send_turn(self, angle_deg: float) -> int:
        self.active_test_parameters.update(turn_deg_target=angle_deg)
        sequence = self.command_sequence & 0xFF
        wire_angle_deg = angle_deg * DEBUG_COMMAND_TURN_SIGN
        packet = motion_turn_frame(sequence, wire_angle_deg, self.speed_mm_s)
        self.record_event("turn_requested", {"angle_deg": angle_deg,
                                             "wire_angle_deg": wire_angle_deg,
                                             "turn_command_sign": DEBUG_COMMAND_TURN_SIGN,
                                             "wheel_tangent_speed_mm_s": self.speed_mm_s})
        result = self.send_packet(packet, "TURN_REL", f"相对转角 {angle_deg:+.1f}°，速度 {self.speed_mm_s} mm/s")
        if result >= 0:
            self.message = f"已发送 TURN_REL {angle_deg:+.1f}°，等待F407完成"
            rate_ddeg_s = int.from_bytes(packet[10:12], "big")
            if rate_ddeg_s > 0:
                ideal_s = abs(int.from_bytes(packet[6:8], "big", signed=True)) / rate_ddeg_s
                if ideal_s >= MOTION_TURN_TIMEOUT_S:
                    self.message += f"；理想转动时间{ideal_s:.1f}s，可能超过下位机10s超时"
                    self.record_event("turn_timeout_warning", {"ideal_duration_s": ideal_s,
                                                               "firmware_timeout_s": MOTION_TURN_TIMEOUT_S})
        return result

    def send_move(self, direction_deg: float, distance_mm: int) -> int:
        self.active_test_parameters.update(direction_deg_target=direction_deg, distance_mm_target=distance_mm)
        sequence = self.command_sequence & 0xFF
        # Apply the observed command signs independently; never rotate the
        # T265 reference to compensate a changed firmware/motor convention.
        direction_rad = math.radians(direction_deg)
        wire_direction_deg = math.degrees(math.atan2(
            DEBUG_COMMAND_LEFT_SIGN * math.sin(direction_rad),
            DEBUG_COMMAND_FORWARD_SIGN * math.cos(direction_rad))) % 360.0
        packet = motion_move_frame(sequence, wire_direction_deg, distance_mm,
                                   self.speed_mm_s, field_frame=False)
        self.record_event("move_requested", {"direction_deg": direction_deg,
                                             "wire_direction_deg": wire_direction_deg,
                                             "forward_command_sign": DEBUG_COMMAND_FORWARD_SIGN,
                                             "left_command_sign": DEBUG_COMMAND_LEFT_SIGN,
                                             "distance_mm": distance_mm,
                                             "speed_mm_s": self.speed_mm_s})
        result = self.send_packet(packet, "MOVE_BODY",
                                  f"相对方向 {direction_deg:.1f}°，距离 {distance_mm} mm，速度 {self.speed_mm_s} mm/s")
        if result >= 0:
            self.message = f"已发送 MOVE_DISTANCE {direction_deg:.1f}°/{distance_mm} mm"
        return result

    def toggle_test(self, name: str, action) -> None:
        if not self.motion_identity_confirmed():
            self.message = "请用启动参数记录实际固件提交及MotionDebug模式，再操作"
            return
        if self.active_test == name:
            self.record_event(f"{name}_end_manual")
            self.active_test = None
            self.return_phase = None
            self.send_stop("实验按钮再次点击，中止当前动作")
            return
        if self.active_test is not None:
            self.record_event(f"{self.active_test}_end_aborted")
            self.send_stop("切换实验，中止上一动作")
        self.active_test = name
        self.return_phase = None
        self.test_id += 1
        self.active_test_parameters = {"name": name, "distance_mm": self.distance_mm,
                                       "speed_mm_s": self.speed_mm_s}
        self.record_event(f"{name}_start")
        action()

    def motion_identity_confirmed(self) -> bool:
        return (self.options.actual_lower_mode == "MotionDebug" and
                self.options.actual_lower_firmware.strip().lower() not in {"", "unconfirmed", "unknown"})

    def save_ground_truth(self, text: str) -> None:
        """Signed, independently measured displacement/turn for the current test.
        A full turn remains +/-360, never wrapped or inferred from the command.
        """
        if self.active_test is not None or self.test_id <= 0:
            raise ValueError("先结束动作，停稳后再录入卷尺/地面角度测量")
        parts = text.split(",")
        if len(parts) != 3:
            raise ValueError("格式为前向mm,左向mm,转角deg；未知填?")
        values = [None if p.strip() == "?" else float(p) for p in parts]
        if all(v is None for v in values) or any(v is not None and not math.isfinite(v) for v in values):
            raise ValueError("至少录入一项有限实测值")
        if not self.options.reference_description.strip():
            raise ValueError("启动时用--reference-description记录卷尺零点和地面角度基准")
        document = {"test_id": self.test_id, "timestamp_monotonic_ns": time.monotonic_ns(),
            "source": "operator_tape_and_ground_reference", "reference": self.options.reference_description,
            "forward_mm": values[0], "left_mm": values[1], "turn_deg_unwrapped": values[2],
            "test_parameters": self.active_test_parameters,
            "localization": self.localization, "motion_status": self.motion_status}
        with (self.run_dir / "ground_truth.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(document, ensure_ascii=False) + "\n")
        self.record_event("ground_truth_recorded", document)
        self.message = f"已保存实验{self.test_id}的独立实测值"

    def start_forward(self) -> None:
        self.send_move(0.0, self.distance_mm)

    def start_lateral(self) -> None:
        self.send_move(90.0, self.distance_mm)

    def adjust_distance(self, delta_mm: int) -> None:
        self.distance_mm = max(1, min(10000, self.distance_mm + delta_mm))
        self.record_event("distance_selected", {"distance_mm": self.distance_mm})

    def start_return(self) -> None:
        pose = self.localization.get("pose", {})
        try:
            x, y, current_yaw = float(pose["x_m"]), float(pose["y_m"]), float(pose["yaw_deg"])
        except (KeyError, TypeError, ValueError):
            self.active_test = None
            self.message = "T265位置尚未就绪，不能计算返航方向"
            return
        measured_distance_mm = math.hypot(x, y) * 1000.0
        if measured_distance_mm <= 10.0:
            self.record_event("turn_and_return_end_already_center")
            self.active_test = None
            self.active_command_sequence = None
            self.message = "已在地图中心，无需返航"
            return
        distance = min(10000, max(1, int(round(measured_distance_mm))))
        desired_yaw = math.degrees(math.atan2(-y, -x)) % 360.0
        turn = wrap_degrees(desired_yaw - current_yaw)
        if abs(turn) < 1.0:
            turn = 1.0
        self.return_distance_mm = distance
        self.return_phase = "turn"
        self.record_event("turn_and_return_plan", {
            "turn_angle_deg": turn,
            "distance_mm": distance,
            "desired_field_heading_deg": desired_yaw,
        })
        self.send_turn(turn)

    def handle_motion_status(self, status: dict) -> None:
        if self.active_test is None or self.active_command_sequence is None:
            return
        if int(status.get("command_sequence", -1)) != self.active_command_sequence:
            return
        timestamp_ns = int(status.get("timestamp_monotonic_ns", 0))
        if (int(status.get("command", -1)) != self.active_command_opcode or
                timestamp_ns <= self.active_command_sent_ns or
                self.status_age(status) > MOTION_STATUS_TIMEOUT_MS):
            return
        if time.monotonic_ns() - self.last_matching_status_ns > MOTION_STATUS_TIMEOUT_MS * 1_000_000:
            return  # update_data cancels the chain; a late DONE cannot resume it.
        self.last_matching_status_ns = timestamp_ns
        state = int(status.get("state", -1))
        if state == MOTION_STATE_DONE:
            if self.active_test == "turn_and_return" and self.return_phase == "turn":
                self.record_event("turn_and_return_turn_done")
                self.return_phase = "move"
                self.active_command_sequence = self.send_move(0.0, self.return_distance_mm)
                return
            self.record_event(f"{self.active_test}_end_done")
            self.message = f"实验 {self.active_test} 完成"
            self.active_test = None
            self.return_phase = None
            self.active_command_sequence = None
        elif state in (MOTION_STATE_FAULT, MOTION_STATE_STOPPED):
            self.record_event(f"{self.active_test}_end_{STATE_NAMES.get(state, 'unknown').lower()}")
            self.message = f"实验 {self.active_test} 结束：{STATE_NAMES.get(state, 'UNKNOWN')}"
            self.active_test = None
            self.return_phase = None
            self.active_command_sequence = None

    def select_at(self, x: int, y: int) -> None:
        for name, (x0, y0, x1, y1) in self.hitboxes.items():
            if not (x0 <= x <= x1 and y0 <= y <= y1):
                continue
            if name == "stop":
                if self.active_test is not None:
                    self.record_event(f"{self.active_test}_end_manual_stop")
                    self.active_test = None
                    self.return_phase = None
                self.send_stop()
            elif name.startswith("turn_"):
                angle = int(name.split("_", 1)[1])
                self.toggle_test(f"rotate_{angle}deg", lambda: self.send_turn(angle))
            elif name in {"forward", "backward", "lateral", "right"}:
                direction = {"forward": 0, "backward": 180, "lateral": 90, "right": 270}[name]
                self.toggle_test(name, lambda: self.send_move(direction, self.distance_mm))
            elif name == "returntest":
                self.toggle_test("turn_and_return", self.start_return)
            elif name == "dminus":
                self.adjust_distance(-100)
            elif name == "dplus":
                self.adjust_distance(100)
            elif name == "dfineminus":
                self.adjust_distance(-10)
            elif name == "dfineplus":
                self.adjust_distance(10)
            elif name == "sminus":
                self.speed_mm_s = max(50, self.speed_mm_s - 50)
            elif name == "splus":
                self.speed_mm_s = min(700, self.speed_mm_s + 50)
            break

    def mouse_callback(self, event, x, y, _flags, _parameter) -> None:
        if event == cv2.EVENT_LBUTTONUP:
            logical_x = int(round((x - self.content_offset_x) / self.render_scale))
            logical_y = int(round((y - self.content_offset_y) / self.render_scale))
            self.select_at(logical_x, logical_y)

    def start_session(self) -> None:
        self.options.output_root.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = self.options.output_root / f"{stamp}_interactive"
        suffix = 1
        while self.run_dir.exists():
            self.run_dir = self.options.output_root / f"{stamp}_interactive_{suffix:02d}"
            suffix += 1
        self.run_dir.mkdir(parents=True, exist_ok=False)
        markers = [read_json(p) or {} for p in self.options.output_root.glob("*/session_complete.json")]
        self.session_index = 1 + max((m["session_index"] for m in markers
            if m.get("owner") == "t265_f407_motion_map" and isinstance(m.get("session_index"), int)), default=0)
        config_path = self.run_dir / "localization.conf"
        write_debug_config(self.options.config, config_path, start_zone=self.zone,
                           center_origin=True, start_heading_offset_deg=START_HEADING_OFFSET_DEG,
                           unit_translation_scale=True)
        self.session_config = config_values(config_path)
        self.localization_json = self.run_dir / "localization_result.json"
        self.motion_status_path = self.run_dir / "f407_motion_status.json"
        self.events_path = self.run_dir / "events.jsonl"
        self.commands_path = self.run_dir / "commands.jsonl"
        status_log = self.run_dir / "f407_motion_status.jsonl"
        self.bridge = F407SerialBridge(
            self.options.uart, self.options.baud,
            self.motion_status_path, status_log,
        )
        self.bridge.start()
        localizer_log_path = self.run_dir / "localizer.log"
        self.localizer_log = localizer_log_path.open("w", encoding="utf-8")
        command = [
            str(DEFAULT_LOCALIZER_SCRIPT),
            "--config", str(config_path),
            "--output", str(self.localization_json),
            "--csv", str(self.run_dir / "localization_debug.csv"),
            "--raw-pose-csv", str(self.run_dir / "t265_raw_pose.csv"),
            "--events", str(self.run_dir / "localization_events.jsonl"),
            "--enable-pose-jumping",
            "--t265-only-output",
            "--rate", "20",
            "--tx-rate", "0",
            "--uart", self.bridge.slave_path,
            "--baud", str(self.options.baud),
        ]
        if self.options.serial:
            command += ["--serial", self.options.serial]
        self.localizer = subprocess.Popen(
            command, cwd=str(LOCALIZATION_ROOT),
            stdout=self.localizer_log, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        metadata = {
            "schema_version": 3,
            "created_local": datetime.now().astimezone().isoformat(timespec="seconds"),
            "timestamp_monotonic_ns": time.monotonic_ns(),
            "mode": "interactive_f407_motion_debug",
            "zone": self.zone,
            "initial_pose": {"x_m": 0.0, "y_m": 0.0, "yaw_deg": start_pose(self.zone)[2]},
            "source_config": str(self.options.config),
            "effective_config": self.session_config,
            "distance_controls_mm": {"coarse": 100, "fine": 10, "initial": self.distance_mm},
            "forward_command_sign": DEBUG_COMMAND_FORWARD_SIGN,
            "left_command_sign": DEBUG_COMMAND_LEFT_SIGN,
            "turn_command_sign": DEBUG_COMMAND_TURN_SIGN,
            "disable_pose_jumping_requested": False,
            "enable_pose_jumping_requested": True,
            "pose_source": "t265_direct_with_camera_offset",
            "listen_and_control": True,
            "expected_lower_firmware": LOWER_FIRMWARE,
            "actual_lower_firmware": self.options.actual_lower_firmware,
            "actual_lower_mode": self.options.actual_lower_mode,
            "lower_identity_source": "operator supplied; not reported by UART",
            "mount_status": "candidate_not_physically_validated",
            "reference_description": self.options.reference_description,
            "ground_truth_policy": "command target and DONE are not ground truth; operator measures signed displacement/turn after settling",
            "protocol": MOTION_PROTOCOL,
            "localizer_command": command,
            "files": {
                "localization_csv": "localization_debug.csv",
                "raw_pose_csv": "t265_raw_pose.csv",
                "ground_truth": "ground_truth.jsonl",
                "localization_events": "localization_events.jsonl",
                "localization_json": self.localization_json.name,
                "motion_status": self.motion_status_path.name,
                "motion_status_history": status_log.name,
                "events": self.events_path.name,
                "commands": self.commands_path.name,
                "analysis": "analysis.json",
                "localizer_log": localizer_log_path.name,
                "uart_rx_history": "uart_rx.jsonl",
                "effective_config": "localization.conf",
            },
        }
        try:
            metadata["upper_git_commit"] = subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=PROJECT_ROOT, text=True).strip()
            metadata["upper_tracked_changes"] = subprocess.check_output(
                ["git", "diff", "--name-only", "HEAD"], cwd=PROJECT_ROOT, text=True).splitlines()
        except (OSError, subprocess.SubprocessError):
            metadata["upper_git_commit"] = "unavailable"
        (self.run_dir / "metadata.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        self.message = "已启动：等待F407 ODOM和运动状态"
        self.record_event("session_started", {"initial_pose": metadata["initial_pose"]})
        print(f"测试日志目录：{self.run_dir}", flush=True)

    def stop_session(self) -> None:
        logs_ok = not self.logging_failed and (self.bridge is None or self.bridge.log_error is None)
        localizer_returncode = None
        if self.active_test is not None:
            self.record_event(f"{self.active_test}_end_app_exit")
            self.active_test = None
        if self.bridge is not None and self.bridge.serial is not None and self.last_sent_command is not None:
            try:
                self.send_stop("程序退出安全停车")
                time.sleep(0.12)
            except (RuntimeError, serial.SerialException):
                pass
        if self.localizer is not None and self.localizer.poll() is None:
            self.localizer.send_signal(signal.SIGINT)
            try:
                self.localizer.wait(timeout=4.0)
            except subprocess.TimeoutExpired:
                self.localizer.terminate()
                try:
                    self.localizer.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self.localizer.kill()
                    self.localizer.wait(timeout=2.0)
        if self.localizer is not None:
            localizer_returncode = self.localizer.returncode
        self.localizer = None
        if self.localizer_log is not None:
            self.localizer_log.close()
            self.localizer_log = None
        if self.bridge is not None:
            self.bridge.stop()
            self.bridge = None
        if self.run_dir is not None:
            csv_path = self.run_dir / "localization_debug.csv"
            if csv_path.exists():
                try:
                    result = analyze(csv_path, None, "unknown",
                                     float(self.session_config.get("camera_offset_forward_m", "-0.0296")),
                                     float(self.session_config.get("camera_offset_left_m", "-0.0301")),
                                     fit_wheel=False)
                    result["angle_crossings_not_settled_measurements"] = result.pop("rotation_check")
                    result["protocol"] = MOTION_PROTOCOL
                    result["effective_config"] = self.session_config
                    result["accuracy_verified"] = False
                    result["validation_scope"] = "trajectory diagnostics only; review test_id events and independent ground_truth.jsonl"
                    (self.run_dir / "analysis.json").write_text(
                        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
                except (OSError, RuntimeError, ValueError) as error:
                    self.record_event("analysis_failed", {"error": str(error)})
            stopped_saved = self.record_event("session_stopped", {"localizer_returncode": localizer_returncode})
            # Save completion last. Failed/incomplete sessions never cause pruning.
            required = [self.run_dir / n for n in ("metadata.json", "localization_debug.csv",
                        "t265_raw_pose.csv", "analysis.json", "events.jsonl")]
            if (stopped_saved and logs_ok and not self.logging_failed and localizer_returncode == 0 and
                    all(p.is_file() and p.stat().st_size > 0 for p in required)):
                (self.run_dir / "session_complete.json").write_text(json.dumps({
                    "owner": "t265_f407_motion_map", "complete": True,
                    "session_index": self.session_index,
                    "accuracy_verified": False}) + "\n", encoding="utf-8")
                try:
                    removed = prune_completed_sessions(self.options.output_root, self.run_dir)
                    self.record_event("session_retention", {"keep_complete_sessions": 5, "removed": removed})
                except OSError as error:
                    self.record_event("session_retention_failed", {"error": str(error)})

    def handle_key(self, key: int) -> bool:
        key &= 0xFF
        if self.measurement_buffer is not None:
            if key == 27:
                self.measurement_buffer = None
            elif key in (10, 13):
                try:
                    self.save_ground_truth(self.measurement_buffer)
                    self.measurement_buffer = None
                except (ValueError, OSError) as error:
                    self.message = str(error)
            elif key in (8, 127):
                self.measurement_buffer = self.measurement_buffer[:-1]
            elif key in b"0123456789+-.?,":
                self.measurement_buffer += chr(key)
            return True
        if key in (ord("q"), ord("Q"), 27):
            return False
        if key in (ord("i"), ord("I")):
            self.adjust_distance(-100)
        elif key in (ord("k"), ord("K")):
            self.adjust_distance(100)
        elif key in (ord(","), ord("<")):
            self.adjust_distance(-10)
        elif key in (ord("."), ord(">")):
            self.adjust_distance(10)
        elif key in (ord("u"), ord("U")):
            self.speed_mm_s = min(700, self.speed_mm_s + 50)
        elif key in (ord("j"), ord("J")):
            self.speed_mm_s = max(50, self.speed_mm_s - 50)
        elif key in (ord("s"), ord("S")):
            if self.active_test is not None:
                self.record_event(f"{self.active_test}_end_manual_stop")
            self.active_test = None
            self.return_phase = None
            self.send_stop()
        elif key in (ord("o"), ord("O")):
            self.toggle_test("rotate_90deg", lambda: self.send_turn(90.0))
        elif key in (ord("p"), ord("P")):
            self.toggle_test("rotate_180deg", lambda: self.send_turn(180.0))
        elif key == ord("0"):
            self.toggle_test("rotate_360deg", lambda: self.send_turn(360.0))
        elif key in (ord("v"), ord("V")):
            self.toggle_test("forward", self.start_forward)
        elif key in (ord("l"), ord("L")):
            self.toggle_test("lateral", self.start_lateral)
        elif key in (ord("b"), ord("B")):
            self.toggle_test("backward", lambda: self.send_move(180.0, self.distance_mm))
        elif key in (ord("h"), ord("H")):
            self.toggle_test("right", lambda: self.send_move(270.0, self.distance_mm))
        elif key in (ord("1"), ord("2"), ord("3")):
            angle = {ord("1"): -90.0, ord("2"): -180.0, ord("3"): -360.0}[key]
            self.toggle_test(f"rotate_{int(angle)}deg", lambda: self.send_turn(angle))
        elif key in (ord("m"), ord("M")):
            self.record_event("manual_settled_reference_mark")
            self.message = f"已记录实验{self.test_id}人工停稳基准点"
        elif key in (ord("g"), ord("G")):
            if self.active_test is None and self.test_id > 0:
                self.measurement_buffer = ""
                self.message = "输入前向mm,左向mm,转角deg；未知?；回车保存/Esc取消"
            else:
                self.message = "先结束动作，再人工确认停稳并测量"
        elif key in (ord("y"), ord("Y")):
            self.toggle_test("turn_and_return", self.start_return)
        elif key in (ord("f"), ord("F")):
            self.options.fullscreen = not self.options.fullscreen
            cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN,
                                  cv2.WINDOW_FULLSCREEN if self.options.fullscreen else cv2.WINDOW_NORMAL)
        return True

    def run(self) -> int:
        try:
            self.start_session()
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
            cv2.resizeWindow(WINDOW_NAME, self.screen_width, self.screen_height)
            cv2.imshow(WINDOW_NAME, self.render_screen())
            cv2.waitKey(1)
            cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)
            if self.options.fullscreen:
                cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            while True:
                self.update_data()
                cv2.imshow(WINDOW_NAME, self.render_screen())
                if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                    break
                if not self.handle_key(cv2.waitKey(20)):
                    break
        finally:
            self.stop_session()
            try:
                cv2.destroyWindow(WINDOW_NAME)
            except cv2.error:
                pass
        return 0


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="F407运动调试分支/T265简洁实验地图")
    parser.add_argument("--uart", default="/dev/ttyS1")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--serial")
    parser.add_argument("--reference-description", default="",
                        help="本轮卷尺零点、车中心标记及地面角度基准；G键录入实测前必填")
    parser.add_argument("--actual-lower-firmware", default="unconfirmed")
    parser.add_argument("--actual-lower-mode", default="unconfirmed",
                        choices=("unconfirmed", "NormalRun", "Gamepad", "MotionDebug"))
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG,
                        help="本轮测试配置；复制到独立日志目录，保留安装轴和偏置")
    parser.add_argument("--zone", type=int, choices=range(1, 5), default=4)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_RUN_ROOT)
    parser.add_argument("--fullscreen", dest="fullscreen", action="store_true", default=True)
    parser.add_argument("--windowed", dest="fullscreen", action="store_false")
    return parser.parse_args()


def main() -> int:
    try:
        return MotionMapApp(arguments()).run()
    except (OSError, RuntimeError, ValueError, serial.SerialException) as error:
        print(f"错误：{error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
