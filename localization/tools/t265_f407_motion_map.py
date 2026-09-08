#!/usr/bin/env python3
"""Simple interactive map for F407 ``feat/uart-motion-debug``.

The F407 motion-debug branch has a different protocol from the competition
task firmware: TYPE=0x17 is a one-shot motion command and TYPE=0x18 is the
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
DEFAULT_CONFIG = LOCALIZATION_ROOT / "config" / "localization.example.conf"
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


def read_json(path: Path | None) -> dict | None:
    if path is None:
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return None


def start_pose(zone: int) -> tuple[float, float, float]:
    signs = {1: (-1.0, 1.0), 2: (1.0, 1.0), 3: (-1.0, -1.0), 4: (1.0, -1.0)}
    headings = {1: 135.0, 2: 45.0, 3: 225.0, 4: 315.0}
    return signs[zone][0] * 1.35, signs[zone][1] * 1.35, headings[zone]


def wrap_degrees(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


class TextPainter:
    def __init__(self, font_scale: float = 1.0) -> None:
        self.font_scale = max(1.0, float(font_scale))
        self.items: list[tuple[str, tuple[int, int], int, tuple[int, int, int], bool, str]] = []
        self.fonts: dict[tuple[int, bool], ImageFont.FreeTypeFont] = {}

    def add(self, value, xy, size=18, color=(230, 230, 230), bold=False, anchor="la") -> None:
        self.items.append((str(value), tuple(map(int, xy)),
                           max(12, int(round(size * self.font_scale))),
                           color, bold, anchor))

    def paint(self, canvas: np.ndarray) -> np.ndarray:
        image = Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(image)
        for value, xy, size, color, bold, anchor in self.items:
            key = (size, bold)
            if key not in self.fonts:
                path = FONT_MEDIUM if bold else FONT_REGULAR
                try:
                    self.fonts[key] = ImageFont.truetype(path, size=size)
                except OSError:
                    self.fonts[key] = ImageFont.load_default()
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
        except OSError:
            pass

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
            self.parser.feed(data)
            self._forward_to_pty(data)

    def start(self) -> None:
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
        if self.serial is not None:
            self.serial.close()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        self.thread = None
        self.serial = None
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
        self.return_phase: str | None = None
        self.distance_mm = 1000
        self.speed_mm_s = 300

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
            self.world_rect(canvas, *rect, (0, 245, 255) if zone == self.zone else (80, 30, 80),
                            4 if zone == self.zone else 2)
            text.add(zone, self.world_to_pixel((rect[0] + rect[2]) / 2,
                                               (rect[1] + rect[3]) / 2), 32, (20, 20, 20), True, "mm")
        cv2.rectangle(canvas, (ml, mt), (ml + size, mt + size), (35, 35, 35), 4)
        origin = self.world_to_pixel(0.0, 0.0)
        cv2.line(canvas, (origin[0] - 10, origin[1]), (origin[0] + 10, origin[1]), (70, 70, 70), 2)
        cv2.line(canvas, (origin[0], origin[1] - 10), (origin[0], origin[1] + 10), (70, 70, 70), 2)
        text.add("场地中心", (origin[0] + 12, origin[1] - 10), 15, (70, 70, 70))
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
            return
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
        text.add(label, ((x0 + x1) // 2, (y0 + y1) // 2), 16, (245, 245, 245), selected, "mm")

    def draw_panel(self, canvas: np.ndarray, text: TextPainter) -> None:
        x0 = 920
        cv2.rectangle(canvas, (900, 0), (LOGICAL_WIDTH, LOGICAL_HEIGHT), (26, 27, 30), -1)
        text.add("T265/F407 运动调试", (x0, 35), 27, (245, 245, 245), True)
        running = self.localizer is not None and self.localizer.poll() is None
        text.add("定位进程运行中" if running else "定位进程未运行", (x0, 68), 16,
                 (50, 220, 65) if running else (50, 80, 230), True)
        text.add(f"地图锚点：{self.zone}号（只影响T265显示）", (x0, 96), 15, (220, 220, 220))
        for index in range(4):
            self.button(canvas, text, f"zone{index + 1}", str(index + 1),
                        (x0 + index * 42, 110, x0 + 34 + index * 42, 140), index + 1 == self.zone)
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
        text.add(f"T265-轮式：{float(comparison.get('t265_vs_wheel_odom_distance_m', -1.0)):.3f} m",
                 (x0, 266), 16, (0, 205, 220))
        t265 = self.localization.get("t265", {})
        text.add(f"raw偏置：{float(t265.get('camera_offset_forward_m', 0.0)):+.4f} / {float(t265.get('camera_offset_left_m', 0.0)):+.4f} m",
                 (x0, 292), 15, (50, 170, 80))
        text.add(f"raw增量：{float(t265.get('tracking_origin_delta_forward_m', 0.0)):+.3f} / {float(t265.get('tracking_origin_delta_left_m', 0.0)):+.3f} m",
                 (x0, 316), 15, (50, 170, 80))

        status = self.motion_status
        state = int(status.get("state", 0)) if status else 0
        command = int(status.get("command", 0)) if status else 0
        health = int(status.get("health", 0)) if status else 0
        status_age = self.status_age(status)
        text.add(f"F407：{STATE_NAMES.get(state, 'UNKNOWN')} / {COMMAND_NAMES.get(command, 'UNKNOWN')} age={status_age:.0f}ms",
                 (x0, 350), 15, (245, 215, 150), True)
        text.add(f"进度={int(status.get('progress', 0))}  剩余={int(status.get('remaining', 0))}  健康=0x{health:02X}",
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

        self.button(canvas, text, "stop", "STOP", (x0, 535, x0 + 330, 580), False, (100, 55, 55))
        self.button(canvas, text, "rotate90", "原地90°", (x0, 595, x0 + 103, 630), self.active_test == "rotate_90deg", (75, 75, 105))
        self.button(canvas, text, "rotate180", "原地180°", (x0 + 113, 595, x0 + 226, 630), self.active_test == "rotate_180deg", (75, 75, 105))
        self.button(canvas, text, "rotate360", "原地360°", (x0 + 236, 595, x0 + 330, 630), self.active_test == "rotate_360deg", (75, 75, 105))
        self.button(canvas, text, "forward1m", "直行1m", (x0, 642, x0 + 103, 677), self.active_test == "forward_1m", (55, 110, 80))
        self.button(canvas, text, "lateral1m", "横移1m", (x0 + 113, 642, x0 + 226, 677), self.active_test == "lateral_1m", (55, 110, 80))
        self.button(canvas, text, "returntest", "转向返航", (x0 + 236, 642, x0 + 330, 677), self.active_test == "turn_and_return", (55, 100, 135))
        command = self.last_sent_command
        if command:
            text.add(f"最近发送：{command['name']} seq={command['sequence']}", (x0, 715), 15, (245, 215, 150), True)
            text.add(command["description"], (x0, 739), 14, (220, 220, 220))
        else:
            text.add("最近发送：无", (x0, 715), 15, (180, 180, 185))
        active = self.active_test or "无"
        phase = "" if not self.return_phase else f" / {self.return_phase}"
        text.add(f"实验：{active}{phase}", (x0, 770), 16, (0, 215, 255), True)
        text.add("点击实验按钮：第一次开始，完成后自动结束；再次点击可中止", (x0, 800), 13, (175, 175, 180))
        text.add("旋转/移动均由F407本地IMU和三轮里程计闭环完成", (x0, 824), 13, (175, 175, 180))
        text.add("1-4改地图锚点  I/K距离  U/J速度  Q退出", (x0, 860), 14, (175, 175, 180))
        text.add(f"日志：{self.run_dir.name if self.run_dir else '未启动'}", (x0, 905), 14, (175, 175, 180))
        text.add(self.message, (x0, 970), 15, (0, 215, 255))

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
            status, _ = self.bridge.status_snapshot()
            if status is not None:
                timestamp = int(status.get("timestamp_monotonic_ns", 0))
                if timestamp != self.last_status_timestamp:
                    self.last_status_timestamp = timestamp
                    self.motion_status = status
                    self.handle_motion_status(status)

    def record_event(self, name: str, details: dict | None = None) -> None:
        if self.events_path is None:
            return
        event = {
            "timestamp_monotonic_ns": time.monotonic_ns(),
            "wall_time": datetime.now().isoformat(timespec="milliseconds"),
            "event": name,
            "details": details or {},
            "localization": {
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
            pass

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
            }
            try:
                with self.commands_path.open("a", encoding="utf-8") as stream:
                    stream.write(json.dumps(document, ensure_ascii=False) + "\n")
            except OSError:
                pass

    def send_packet(self, packet: bytes, name: str, description: str) -> int:
        if self.bridge is None:
            self.message = "F407串口尚未连接"
            return -1
        sequence = self.command_sequence & 0xFF
        self.command_sequence = (self.command_sequence + 1) & 0xFF
        self.bridge.send(packet)
        self.active_command_sequence = sequence
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
        sequence = self.command_sequence & 0xFF
        packet = motion_turn_frame(sequence, angle_deg, self.speed_mm_s)
        result = self.send_packet(packet, "TURN_REL", f"相对转角 {angle_deg:+.1f}°，速度 {self.speed_mm_s} mm/s")
        if result >= 0:
            self.message = f"已发送 TURN_REL {angle_deg:+.1f}°，等待F407完成"
        return result

    def send_move(self, direction_deg: float, distance_mm: int) -> int:
        sequence = self.command_sequence & 0xFF
        packet = motion_move_frame(sequence, direction_deg, distance_mm,
                                   self.speed_mm_s, field_frame=False)
        result = self.send_packet(packet, "MOVE_DISTANCE",
                                  f"相对方向 {direction_deg:.1f}°，距离 {distance_mm} mm，速度 {self.speed_mm_s} mm/s")
        if result >= 0:
            self.message = f"已发送 MOVE_DISTANCE {direction_deg:.1f}°/{distance_mm} mm"
        return result

    def toggle_test(self, name: str, action) -> None:
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
        self.record_event(f"{name}_start")
        action()

    def start_forward(self) -> None:
        self.distance_mm = 1000
        self.send_move(0.0, 1000)

    def start_lateral(self) -> None:
        self.distance_mm = 1000
        self.send_move(90.0, 1000)

    def start_return(self) -> None:
        pose = self.localization.get("pose", {})
        try:
            x, y, current_yaw = float(pose["x_m"]), float(pose["y_m"]), float(pose["yaw_deg"])
        except (KeyError, TypeError, ValueError):
            self.active_test = None
            self.message = "T265位置尚未就绪，不能计算返航方向"
            return
        distance = min(10000, max(1, int(round(math.hypot(x, y) * 1000.0))))
        desired_yaw = math.degrees(math.atan2(-y, -x)) % 360.0
        turn = wrap_degrees(desired_yaw - current_yaw)
        if abs(turn) < 1.0:
            turn = 1.0
        self.distance_mm = distance
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
        state = int(status.get("state", -1))
        if state == MOTION_STATE_DONE:
            if self.active_test == "turn_and_return" and self.return_phase == "turn":
                self.record_event("turn_and_return_turn_done")
                self.return_phase = "move"
                self.active_command_sequence = self.send_move(0.0, self.distance_mm)
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
            if name.startswith("zone"):
                self.zone = int(name[-1])
            elif name == "stop":
                if self.active_test is not None:
                    self.record_event(f"{self.active_test}_end_manual_stop")
                    self.active_test = None
                    self.return_phase = None
                self.send_stop()
            elif name == "rotate90":
                self.toggle_test("rotate_90deg", lambda: self.send_turn(90.0))
            elif name == "rotate180":
                self.toggle_test("rotate_180deg", lambda: self.send_turn(180.0))
            elif name == "rotate360":
                self.toggle_test("rotate_360deg", lambda: self.send_turn(360.0))
            elif name == "forward1m":
                self.toggle_test("forward_1m", self.start_forward)
            elif name == "lateral1m":
                self.toggle_test("lateral_1m", self.start_lateral)
            elif name == "returntest":
                self.toggle_test("turn_and_return", self.start_return)
            elif name == "dminus":
                self.distance_mm = max(1, self.distance_mm - 100)
            elif name == "dplus":
                self.distance_mm = min(10000, self.distance_mm + 100)
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
        config_path = self.run_dir / "localization.conf"
        write_debug_config(DEFAULT_CONFIG, config_path, start_zone=self.zone)
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
            "schema_version": 1,
            "timestamp_monotonic_ns": time.monotonic_ns(),
            "mode": "interactive_f407_motion_debug",
            "zone": self.zone,
            "listen_and_control": True,
            "lower_repository": "gandizm/F407-Rescue-Robot@9774dac",
            "protocol": PROTOCOL,
            "localizer_command": command,
            "files": {
                "localization_csv": "localization_debug.csv",
                "localization_json": self.localization_json.name,
                "motion_status": self.motion_status_path.name,
                "motion_status_history": status_log.name,
                "events": self.events_path.name,
                "commands": self.commands_path.name,
                "analysis": "analysis.json",
                "localizer_log": localizer_log_path.name,
            },
        }
        (self.run_dir / "metadata.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        self.message = "已启动：等待F407 ODOM和运动状态"

    def stop_session(self) -> None:
        if self.active_test is not None:
            self.record_event(f"{self.active_test}_end_app_exit")
            self.active_test = None
        if self.bridge is not None and self.bridge.serial is not None:
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
                    analyze(csv_path, self.run_dir / "analysis.json", "unknown",
                            -0.0296, -0.0301)
                except (OSError, RuntimeError, ValueError):
                    pass

    def handle_key(self, key: int) -> bool:
        key &= 0xFF
        if key in (ord("q"), ord("Q"), 27):
            return False
        if ord("1") <= key <= ord("4"):
            self.zone = key - ord("0")
        elif key in (ord("i"), ord("I")):
            self.distance_mm = max(1, self.distance_mm - 100)
        elif key in (ord("k"), ord("K")):
            self.distance_mm = min(10000, self.distance_mm + 100)
        elif key in (ord("u"), ord("U")):
            self.speed_mm_s = min(700, self.speed_mm_s + 50)
        elif key in (ord("j"), ord("J")):
            self.speed_mm_s = max(50, self.speed_mm_s - 50)
        elif key in (ord("s"), ord("S")):
            self.select_at(920, 550)
        elif key in (ord("o"), ord("O")):
            self.toggle_test("rotate_90deg", lambda: self.send_turn(90.0))
        elif key in (ord("p"), ord("P")):
            self.toggle_test("rotate_180deg", lambda: self.send_turn(180.0))
        elif key == ord("0"):
            self.toggle_test("rotate_360deg", lambda: self.send_turn(360.0))
        elif key in (ord("v"), ord("V")):
            self.toggle_test("forward_1m", self.start_forward)
        elif key in (ord("l"), ord("L")):
            self.toggle_test("lateral_1m", self.start_lateral)
        elif key in (ord("y"), ord("Y")):
            self.toggle_test("turn_and_return", self.start_return)
        elif key in (ord("f"), ord("F")):
            self.options.fullscreen = not self.options.fullscreen
            cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN,
                                  cv2.WINDOW_FULLSCREEN if self.options.fullscreen else cv2.WINDOW_NORMAL)
        return True

    def run(self) -> int:
        self.start_session()
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow(WINDOW_NAME, self.screen_width, self.screen_height)
        cv2.imshow(WINDOW_NAME, self.render_screen())
        cv2.waitKey(1)
        cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)
        if self.options.fullscreen:
            cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        try:
            while True:
                self.update_data()
                cv2.imshow(WINDOW_NAME, self.render_screen())
                if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                    break
                if not self.handle_key(cv2.waitKey(20)):
                    break
        finally:
            self.stop_session()
            cv2.destroyWindow(WINDOW_NAME)
        return 0


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="F407运动调试分支/T265简洁实验地图")
    parser.add_argument("--uart", default="/dev/ttyS1")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--serial")
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
