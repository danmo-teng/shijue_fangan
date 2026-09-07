#!/usr/bin/env python3
"""Interactive T265/F407 diagnostic map and mission-command console.

This is intentionally separate from the competition rescue map.  It starts
the upper-computer localizer with zero encoder fusion, displays T265 corrected
robot-center / raw tracking-origin / wheel-odometry trajectories, and relays
operator-selected legal TYPE=0x18 commands through the localizer command-file
bridge.  It never edits or runs the F407 repository.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

TOOLS_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = TOOLS_ROOT.parents[1]
LOCALIZATION_ROOT = PROJECT_ROOT / "localization"
RUNTIME_ROOT = PROJECT_ROOT / "rescue_map" / "runtime"
DEFAULT_RUN_ROOT = RUNTIME_ROOT / "history" / "t265_f407_debug_map"
VISION_ROOT = PROJECT_ROOT / "vision"
if str(VISION_ROOT) not in sys.path:
    sys.path.insert(0, str(VISION_ROOT))

from rescue_vision.mission_protocol import (  # noqa: E402
    CMD_ABORT,
    CMD_DISTANCE_VALID,
    CMD_DRIVE_STRAIGHT,
    CMD_ENTER_SAFE_ZONE,
    CMD_GRAB_CONFIRMED,
    CMD_NAVIGATE_WAYPOINT,
    CMD_RED_SIDE,
    CMD_RETURN_CENTER,
    CMD_STOP,
    CMD_TASK_COMPLETE,
    CMD_USE_FINAL_HEADING,
    CMD_VALID,
    MissionCommand,
    write_command_frame,
)
from rescue_vision.vision_protocol import config_frame  # noqa: E402
from t265_f407_debug import PROTOCOL, write_debug_config  # noqa: E402


WINDOW_NAME = "t265_f407_debug_map"
FONT_REGULAR = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
FONT_MEDIUM = "/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc"
FIELD_HALF_M = 1.5


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="T265/F407调试地图和下位机命令控制台")
    parser.add_argument("--uart", default="/dev/ttyS1")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--serial")
    parser.add_argument("--zone", type=int, choices=range(1, 5), default=4)
    parser.add_argument("--side", choices=("red", "blue"), default="red")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_RUN_ROOT)
    parser.add_argument("--fullscreen", action="store_true")
    parser.add_argument("--no-start", action="store_true", help="只显示界面，不启动定位进程")
    return parser.parse_args()


def read_json(path: Path) -> dict | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError):
        return None


def start_pose(zone: int) -> tuple[float, float, float]:
    signs = {1: (-1.0, 1.0), 2: (1.0, 1.0), 3: (-1.0, -1.0), 4: (1.0, -1.0)}
    headings = {1: 135.0, 2: 45.0, 3: 225.0, 4: 315.0}
    x_sign, y_sign = signs[zone]
    return x_sign * 1.35, y_sign * 1.35, headings[zone]


class TextPainter:
    def __init__(self) -> None:
        self.items: list[tuple[str, tuple[int, int], int, tuple[int, int, int], bool, str]] = []
        self.fonts: dict[tuple[int, bool], ImageFont.FreeTypeFont] = {}

    def add(self, value, xy, size=18, color=(230, 230, 230), bold=False, anchor="la") -> None:
        self.items.append((str(value), tuple(map(int, xy)), size, color, bold, anchor))

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


class DebugMapApp:
    def __init__(self, options: argparse.Namespace) -> None:
        self.options = options
        self.width, self.height = 1280, 1024
        self.map_left, self.map_top, self.map_size = 30, 60, 850
        self.zone = options.zone
        self.side = options.side
        self.fullscreen = options.fullscreen
        self.hitboxes: dict[str, tuple[int, int, int, int]] = {}
        self.message = "准备启动只监听定位进程"
        self.process: subprocess.Popen | None = None
        self.log_handle = None
        self.run_dir: Path | None = None
        self.localization_json: Path | None = None
        self.status_json: Path | None = None
        self.command_path: Path | None = None
        self.events_path: Path | None = None
        self.active_command: MissionCommand | None = None
        self.active_experiment: str | None = None
        self.command_sequence = 0
        self.config_sequence = 0
        self.last_command_write = 0.0
        self.last_json_timestamp = 0
        self.localization: dict = {}
        self.status: dict = {}
        self.t265_points: list[tuple[float, float]] = []
        self.odom_points: list[tuple[float, float]] = []
        self.raw_points: list[tuple[float, float]] = []
        self.distance_mm = 1000
        self.heading_deg = 0.0
        self.current_command_text = "无持续命令"
        self.last_status_read = 0.0
        self.message_time = time.monotonic()

    @property
    def config_color(self) -> int:
        return 0x11 if self.side == "red" else 0x12

    def world_to_pixel(self, x_m: float, y_m: float) -> tuple[int, int]:
        px = self.map_left + (x_m + FIELD_HALF_M) / (2.0 * FIELD_HALF_M) * self.map_size
        py = self.map_top + (FIELD_HALF_M - y_m) / (2.0 * FIELD_HALF_M) * self.map_size
        return int(round(px)), int(round(py))

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
        for side, y0, y1, color in (
            ("red", 1.14, 1.50, (70, 70, 185)),
            ("blue", -1.50, -1.14, (190, 125, 25)),
        ):
            self.world_rect(canvas, -0.33, y0, 0.33, y1, (120, 45, 145), -1)
            inner_y0, inner_y1 = (1.20, 1.50) if side == "red" else (-1.50, -1.20)
            self.world_rect(canvas, -0.30, inner_y0, 0.30, inner_y1, color, -1)
            if side == self.side:
                self.world_rect(canvas, -0.34, y0 - 0.01, 0.34, y1 + 0.01, (0, 245, 255), 4)
            label = ("红方" if side == "red" else "蓝方") + ("·本方" if side == self.side else "")
            text.add(label, self.world_to_pixel(0.0, 1.34 if side == "red" else -1.34),
                     18, (245, 245, 245), True, "mm")

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
                                               (rect[1] + rect[3]) / 2),
                     32, (20, 20, 20), True, "mm")
        cv2.rectangle(canvas, (ml, mt), (ml + size, mt + size), (35, 35, 35), 4)
        origin = self.world_to_pixel(0.0, 0.0)
        cv2.line(canvas, (origin[0] - 10, origin[1]), (origin[0] + 10, origin[1]), (70, 70, 70), 2)
        cv2.line(canvas, (origin[0], origin[1] - 10), (origin[0], origin[1] + 10), (70, 70, 70), 2)
        text.add("场地中心", (origin[0] + 12, origin[1] - 10), 15, (70, 70, 70))
        text.add("T265修正中心", (ml + 20, mt + size - 28), 15, (20, 150, 245), True)
        text.add("轮式中心", (ml + 150, mt + size - 28), 15, (220, 80, 220), True)
        text.add("raw tracking origin", (ml + 265, mt + size - 28), 15, (50, 170, 80), True)

    @staticmethod
    def draw_path(canvas: np.ndarray, points: list[tuple[float, float]], color, width: int) -> None:
        if len(points) >= 2:
            pixels = np.asarray([
                DebugMapApp.world_to_pixel_static(x, y) for x, y in points
            ], dtype=np.int32)
            cv2.polylines(canvas, [pixels], False, color, width, cv2.LINE_AA)

    @staticmethod
    def world_to_pixel_static(x_m: float, y_m: float) -> tuple[int, int]:
        return (
            int(round(30 + (x_m + 1.5) / 3.0 * 850)),
            int(round(60 + (1.5 - y_m) / 3.0 * 850)),
        )

    def draw_trajectories(self, canvas: np.ndarray) -> None:
        self.draw_path(canvas, self.t265_points, (20, 150, 245), 3)
        self.draw_path(canvas, self.odom_points, (220, 80, 220), 2)
        self.draw_path(canvas, self.raw_points, (50, 170, 80), 2)

    def current_pose(self) -> tuple[float, float, float] | None:
        pose = self.localization.get("pose", {})
        try:
            return float(pose["x_m"]), float(pose["y_m"]), float(pose["yaw_deg"])
        except (KeyError, TypeError, ValueError):
            return None

    def draw_robots(self, canvas: np.ndarray) -> None:
        pose = self.current_pose()
        if pose is None:
            return
        x_m, y_m, yaw_deg = pose
        center = self.world_to_pixel(x_m, y_m)
        tip = self.world_to_pixel(x_m + 0.24 * math.cos(math.radians(yaw_deg)),
                                  y_m + 0.24 * math.sin(math.radians(yaw_deg)))
        cv2.circle(canvas, center, 22, (25, 25, 25), -1, cv2.LINE_AA)
        cv2.circle(canvas, center, 19, (20, 150, 245), 3, cv2.LINE_AA)
        cv2.arrowedLine(canvas, center, tip, (20, 150, 245), 5, cv2.LINE_AA, tipLength=0.30)

        odom = self.localization.get("wheel_odom", {})
        try:
            ox, oy, oyaw = float(odom["x_m"]), float(odom["y_m"]), float(odom["yaw_deg"])
            odom_center = self.world_to_pixel(ox, oy)
            odom_tip = self.world_to_pixel(ox + 0.20 * math.cos(math.radians(oyaw)),
                                           oy + 0.20 * math.sin(math.radians(oyaw)))
            cv2.line(canvas, center, odom_center, (115, 115, 115), 1, cv2.LINE_AA)
            cv2.circle(canvas, odom_center, 9, (220, 80, 220), 2, cv2.LINE_AA)
            cv2.arrowedLine(canvas, odom_center, odom_tip, (220, 80, 220), 3,
                            cv2.LINE_AA, tipLength=0.30)
        except (KeyError, TypeError, ValueError):
            pass

        tracking = self.localization.get("t265", {}).get("tracking_origin", {})
        try:
            rx, ry = float(tracking["x_m"]), float(tracking["y_m"])
            raw_center = self.world_to_pixel(rx, ry)
            cv2.circle(canvas, raw_center, 7, (50, 170, 80), 2, cv2.LINE_AA)
            cv2.line(canvas, center, raw_center, (50, 170, 80), 1, cv2.LINE_AA)
        except (KeyError, TypeError, ValueError):
            pass

    def button(self, canvas, text: TextPainter, name: str, label: str,
               rect: tuple[int, int, int, int], selected=False, color=(65, 65, 70)) -> None:
        x0, y0, x1, y1 = rect
        self.hitboxes[name] = rect
        cv2.rectangle(canvas, (x0, y0), (x1, y1), color if selected else (45, 45, 48), -1)
        cv2.rectangle(canvas, (x0, y0), (x1, y1), (0, 225, 255) if selected else (105, 105, 110), 2)
        text.add(label, ((x0 + x1) // 2, (y0 + y1) // 2), 16, (245, 245, 245), selected, "mm")

    def draw_panel(self, canvas: np.ndarray, text: TextPainter) -> None:
        x0 = 920
        cv2.rectangle(canvas, (900, 0), (self.width, self.height), (26, 27, 30), -1)
        text.add("T265 / F407 调试地图", (x0, 35), 27, (245, 245, 245), True)
        process_text = "定位进程运行中" if self.process is not None and self.process.poll() is None else "定位进程未运行"
        text.add(process_text, (x0, 68), 16, (50, 220, 65) if "运行" in process_text else (50, 80, 230), True)
        text.add(f"配置：{self.zone}号 / {'红方' if self.side == 'red' else '蓝方'}", (x0, 94), 16, (220, 220, 220))
        for index in range(4):
            self.button(canvas, text, f"zone{index + 1}", str(index + 1),
                        (x0 + index * 42, 108, x0 + 34 + index * 42, 138), index + 1 == self.zone)
        self.button(canvas, text, "red", "红", (x0, 145, x0 + 75, 178), self.side == "red", (70, 60, 170))
        self.button(canvas, text, "blue", "蓝", (x0 + 85, 145, x0 + 160, 178), self.side == "blue", (170, 100, 35))

        pose = self.current_pose()
        if pose is None:
            text.add("T265：等待数据", (x0, 215), 18, (170, 170, 175))
        else:
            text.add(f"T265中心 X/Y：{pose[0]:+.3f} / {pose[1]:+.3f} m", (x0, 215), 17, (20, 170, 245), True)
            text.add(f"T265方向：{pose[2]:06.2f}°", (x0, 242), 17, (20, 170, 245), True)
        odom = self.localization.get("wheel_odom", {})
        text.add(f"轮式中心：{float(odom.get('x_m', 0.0)):+.3f} / {float(odom.get('y_m', 0.0)):+.3f} m",
                 (x0, 275), 16, (220, 80, 220), True)
        comparison = self.localization.get("comparison", {})
        text.add(f"T265-轮式：{float(comparison.get('t265_vs_wheel_odom_distance_m', -1.0)):.3f} m",
                 (x0, 301), 16, (0, 205, 220))
        t265 = self.localization.get("t265", {})
        text.add(f"raw中心偏置：{float(t265.get('camera_offset_forward_m', -0.0)):+.4f} / {float(t265.get('camera_offset_left_m', -0.0)):+.4f} m",
                 (x0, 327), 15, (50, 170, 80))
        text.add(f"raw-修正：{float(t265.get('tracking_origin_delta_forward_m', 0.0)):+.3f} / {float(t265.get('tracking_origin_delta_left_m', 0.0)):+.3f} m",
                 (x0, 351), 15, (50, 170, 80))

        status = self.status
        status_age = self.age_ms(status)
        text.add(f"F407：mode={status.get('mode', '--')} flags=0x{int(status.get('flags', 0)):02X} age={status_age:.0f}ms",
                 (x0, 385), 15, (245, 215, 150))
        text.add(f"ACK={status.get('acknowledged_sequence', '--')} fault={status.get('fault_code', '--')} motors={'ON' if int(status.get('flags', 0)) & 4 else 'OFF'}",
                 (x0, 409), 15, (245, 215, 150))
        uart = self.localization.get("uart", {})
        text.add(f"UART RX={uart.get('frames', 0)} CRC={uart.get('crc_errors', 0)} gap={uart.get('sequence_gaps', 0)}",
                 (x0, 435), 15, (190, 190, 200))
        relay = status.get("relay", {})
        text.add(f"TX={relay.get('tx_frames', 0)} err={relay.get('tx_errors', 0)}",
                 (x0, 459), 15, (190, 190, 200))

        text.add(f"命令参数：{self.distance_mm} mm / {self.heading_deg:.1f}°", (x0, 492), 16, (245, 245, 245), True)
        self.button(canvas, text, "dminus", "距离-100", (x0, 510, x0 + 78, 544))
        self.button(canvas, text, "dplus", "距离+100", (x0 + 84, 510, x0 + 162, 544))
        self.button(canvas, text, "hminus", "方向-10", (x0 + 168, 510, x0 + 246, 544))
        self.button(canvas, text, "hplus", "方向+10", (x0 + 252, 510, x0 + 330, 544))

        self.button(canvas, text, "config", "发送配置", (x0, 565, x0 + 155, 607), False, (95, 75, 35))
        self.button(canvas, text, "clear", "释放命令", (x0 + 165, 565, x0 + 330, 607))
        self.button(canvas, text, "stop", "STOP", (x0, 620, x0 + 75, 660), False, (75, 75, 75))
        self.button(canvas, text, "abort", "ABORT", (x0 + 85, 620, x0 + 160, 660), False, (80, 45, 150))
        self.button(canvas, text, "grab", "GRAB", (x0 + 170, 620, x0 + 245, 660), False, (50, 110, 70))
        self.button(canvas, text, "enter", "ENTER", (x0 + 255, 620, x0 + 330, 660), False, (50, 110, 70))
        self.button(canvas, text, "nav", "NAV", (x0, 675, x0 + 155, 715), False, (50, 120, 80))
        self.button(canvas, text, "return", "RETURN", (x0 + 165, 675, x0 + 330, 715), False, (50, 100, 130))
        self.button(canvas, text, "complete", "TASK_COMPLETE", (x0, 730, x0 + 330, 770), False, (120, 70, 50))
        self.button(canvas, text, "rotate90", "原地90°", (x0, 785, x0 + 103, 818), False, (75, 75, 105))
        self.button(canvas, text, "rotate180", "原地180°", (x0 + 113, 785, x0 + 226, 818), False, (75, 75, 105))
        self.button(canvas, text, "rotate360", "原地360°", (x0 + 236, 785, x0 + 330, 818), False, (75, 75, 105))
        self.button(canvas, text, "forward1m", "直行1m", (x0, 825, x0 + 103, 858), False, (55, 110, 80))
        self.button(canvas, text, "lateral1m", "横移1m", (x0 + 113, 825, x0 + 226, 858), False, (55, 110, 80))
        self.button(canvas, text, "returntest", "转向返航", (x0 + 236, 825, x0 + 330, 858), False, (55, 100, 135))
        command_name = "无持续命令" if self.active_command is None else f"0x{self.active_command.command:02X} seq={self.command_sequence:03d}"
        text.add(f"当前命令：{command_name}", (x0, 884), 15, (245, 215, 150), True)
        text.add(f"实验：{self.active_experiment or '未标记'}（再次点击结束并写入events.jsonl）", (x0, 906), 13, (0, 215, 255))
        text.add("旋转实验需人工推动/转动车体；横移命令会先转向再直行", (x0, 928), 12, (175, 175, 180))
        text.add("C配置 S停止 A终止 G抓取 N导航 Y返中 E入区 T完成 X释放", (x0, 948), 12, (175, 175, 180))
        text.add(f"日志：{self.run_dir.name if self.run_dir else '未启动'}", (x0, 970), 13, (175, 175, 180))
        text.add(self.message, (x0, 997), 14, (0, 215, 255))

    def render(self) -> np.ndarray:
        canvas = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        text = TextPainter()
        self.draw_field(canvas, text)
        self.draw_trajectories(canvas)
        self.draw_robots(canvas)
        self.draw_panel(canvas, text)
        return text.paint(canvas)

    def age_ms(self, document: dict | None) -> float:
        if not document:
            return float("inf")
        try:
            return max(0.0, (time.monotonic_ns() - int(document["timestamp_monotonic_ns"])) / 1_000_000.0)
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
        if self.localization_json is None:
            return
        data = read_json(self.localization_json)
        if data is None:
            return
        timestamp = int(data.get("timestamp_monotonic_ns", 0))
        if timestamp == self.last_json_timestamp:
            return
        self.last_json_timestamp = timestamp
        self.localization = data
        pose = data.get("pose", {})
        self.append_point(self.t265_points, pose.get("x_m"), pose.get("y_m"))
        odom = data.get("wheel_odom", {})
        self.append_point(self.odom_points, odom.get("x_m"), odom.get("y_m"))
        tracking = data.get("t265", {}).get("tracking_origin", {})
        self.append_point(self.raw_points, tracking.get("x_m"), tracking.get("y_m"))
        status = read_json(self.status_json) if self.status_json else None
        if status is not None:
            self.status = status

    def write_command(self, command: MissionCommand) -> None:
        if self.command_path is None:
            self.message = "定位进程尚未启动"
            return
        packet = command.to_frame(self.command_sequence & 0xFF)
        write_command_frame(self.command_path, packet)
        self.command_sequence = (self.command_sequence + 1) & 0xFF
        self.active_command = command
        self.last_command_write = time.monotonic()
        self.current_command_text = f"0x{command.command:02X}"

    def refresh_command(self) -> None:
        if self.active_command is None or self.command_path is None:
            return
        if time.monotonic() - self.last_command_write >= 0.10:
            self.write_command(self.active_command)

    def command_flags(self, *, straight=False, distance=False) -> int:
        flags = CMD_VALID | CMD_USE_FINAL_HEADING
        if self.side == "red":
            flags |= CMD_RED_SIDE
        if straight:
            flags |= CMD_DRIVE_STRAIGHT
        if distance:
            flags |= CMD_DISTANCE_VALID
        return flags

    def send_simple(self, code: int) -> None:
        self.write_command(MissionCommand(code, self.command_flags()))
        self.message = f"已发送命令 0x{code:02X}，持续心跳"

    def send_navigation(self, code: int) -> None:
        distance = max(0, min(5000, int(self.distance_mm)))
        heading = int(round(self.heading_deg * 100.0)) % 36000
        command = MissionCommand(
            code,
            self.command_flags(straight=True, distance=True),
            target_x_mm=distance,
            target_y_mm=0,
            heading_cdeg=heading,
        )
        self.write_command(command)
        self.message = f"已发送{('NAV' if code == CMD_NAVIGATE_WAYPOINT else 'RETURN')}：{distance} mm / {self.heading_deg:.1f}°"

    def send_config(self) -> None:
        if self.command_path is None:
            self.message = "定位进程尚未启动"
            return
        for _ in range(3):
            packet = config_frame(self.config_sequence & 0xFF, self.config_color, self.zone)
            self.config_sequence = (self.config_sequence + 1) & 0xFF
            temporary = self.command_path.with_suffix(self.command_path.suffix + ".tmp")
            temporary.write_bytes(packet)
            os.replace(temporary, self.command_path)
            time.sleep(0.08)
        self.message = f"已发送3帧配置：{self.side}/{self.zone}；F407可能开始任务流程"

    def clear_command(self) -> None:
        self.active_command = None
        if self.command_path is not None:
            self.command_path.unlink(missing_ok=True)
        self.message = "已释放命令文件，等待F407命令超时停车"

    def record_event(self, name: str) -> None:
        if self.events_path is None:
            return
        pose = self.localization.get("pose", {})
        t265 = self.localization.get("t265", {})
        odom = self.localization.get("wheel_odom", {})
        event = {
            "timestamp_monotonic_ns": time.monotonic_ns(),
            "wall_time": datetime.now().isoformat(timespec="milliseconds"),
            "event": name,
            "pose": pose,
            "t265": {
                "tracking_origin": t265.get("tracking_origin"),
                "robot_center_delta_forward_m": t265.get("robot_center_delta_forward_m"),
                "robot_center_delta_left_m": t265.get("robot_center_delta_left_m"),
            },
            "wheel_odom": {
                "x_m": odom.get("x_m"),
                "y_m": odom.get("y_m"),
                "yaw_deg": odom.get("yaw_deg"),
                "travel_m": odom.get("travel_m"),
            },
            "status": self.status,
            "active_command": None if self.active_command is None else {
                "command": self.active_command.command,
                "flags": self.active_command.flags,
                "target_x_mm": self.active_command.target_x_mm,
                "heading_cdeg": self.active_command.heading_cdeg,
            },
        }
        with self.events_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(event, ensure_ascii=False) + "\n")

    def toggle_experiment(self, name: str, action=None) -> None:
        if self.active_experiment == name:
            self.record_event(f"{name}_end")
            self.active_experiment = None
            if action is not None:
                self.send_simple(CMD_STOP)
            self.message = f"实验{name}已结束，结果保留在events.jsonl"
            return
        if self.active_experiment is not None:
            self.record_event(f"{self.active_experiment}_end_aborted")
        self.active_experiment = name
        self.record_event(f"{name}_start")
        if action is not None:
            action()
        else:
            # Rotation markers are manual experiments; hold the lower machine
            # instead of allowing a previous navigation command to continue.
            self.send_simple(CMD_STOP)
        self.message = f"实验{name}开始；完成后再次点击同一按钮结束"

    def start_forward_experiment(self) -> None:
        pose = self.current_pose()
        self.distance_mm = 1000
        self.heading_deg = pose[2] if pose is not None else start_pose(self.zone)[2]
        self.send_navigation(CMD_NAVIGATE_WAYPOINT)

    def start_lateral_experiment(self) -> None:
        pose = self.current_pose()
        self.distance_mm = 1000
        base_heading = pose[2] if pose is not None else start_pose(self.zone)[2]
        self.heading_deg = (base_heading + 90.0) % 360.0
        self.send_navigation(CMD_NAVIGATE_WAYPOINT)

    def start_return_experiment(self) -> None:
        pose = self.current_pose()
        if pose is None:
            self.distance_mm = 1000
            self.heading_deg = 0.0
        else:
            self.distance_mm = min(5000, max(0, int(round(math.hypot(pose[0], pose[1]) * 1000.0))))
            self.heading_deg = math.degrees(math.atan2(-pose[1], -pose[0])) % 360.0
        self.send_navigation(CMD_RETURN_CENTER)

    def unique_run_dir(self) -> Path:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = self.options.output_root / f"{stamp}_interactive"
        candidate = base
        suffix = 1
        while candidate.exists():
            candidate = self.options.output_root / f"{base.name}_{suffix:02d}"
            suffix += 1
        return candidate

    def start_session(self) -> None:
        self.stop_session()
        self.options.output_root.mkdir(parents=True, exist_ok=True)
        self.run_dir = self.unique_run_dir()
        self.run_dir.mkdir(parents=True, exist_ok=False)
        config_path = self.run_dir / "localization.conf"
        write_debug_config(LOCALIZATION_ROOT / "config" / "localization.example.conf", config_path)
        self.localization_json = self.run_dir / "localization_result.json"
        self.status_json = self.run_dir / "stm32_status.json"
        self.command_path = self.run_dir / "uart_command.bin"
        self.events_path = self.run_dir / "events.jsonl"
        log_path = self.run_dir / "localizer.log"
        metadata = {
            "schema_version": 1,
            "timestamp_monotonic_ns": time.monotonic_ns(),
            "mode": "interactive_debug_map",
            "zone": self.zone,
            "side": self.side,
            "listen_and_control_via_command_file": True,
            "f407_interface": PROTOCOL,
            "files": {
                "localization_config": config_path.name,
                "localization_json": self.localization_json.name,
                "localization_csv": "localization_debug.csv",
                "stm_status": self.status_json.name,
                "command_file": self.command_path.name,
                "events": self.events_path.name,
                "localizer_log": log_path.name,
            },
        }
        (self.run_dir / "metadata.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        command = [
            str(LOCALIZATION_ROOT / "run_localization.sh"),
            "--config", str(config_path),
            "--output", str(self.localization_json),
            "--csv", str(self.run_dir / "localization_debug.csv"),
            "--rate", "20",
            "--tx-rate", "0",
            "--command-file", str(self.command_path),
            "--stm-status", str(self.status_json),
        ]
        if self.options.uart.lower() not in {"", "none", "off"}:
            command += ["--uart", self.options.uart, "--baud", str(self.options.baud)]
        if self.options.serial:
            command += ["--serial", self.options.serial]
        self.log_handle = log_path.open("w", encoding="utf-8")
        if not self.options.no_start:
            self.process = subprocess.Popen(
                command, cwd=str(LOCALIZATION_ROOT),
                stdout=self.log_handle, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            self.message = "定位进程已启动；等待T265/F407数据"
        else:
            self.message = "未启动定位进程（--no-start）"

    def stop_process(self, process: subprocess.Popen | None) -> None:
        if process is None or process.poll() is not None:
            return
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=4.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()

    def stop_session(self) -> None:
        if self.active_experiment is not None:
            self.record_event(f"{self.active_experiment}_end_aborted")
            self.active_experiment = None
        self.clear_command()
        self.stop_process(self.process)
        self.process = None
        if self.log_handle is not None:
            self.log_handle.close()
            self.log_handle = None

    def select_at(self, x: int, y: int) -> None:
        for name, (x0, y0, x1, y1) in self.hitboxes.items():
            if x0 <= x <= x1 and y0 <= y <= y1:
                if name.startswith("zone"):
                    self.zone = int(name[-1])
                elif name in {"red", "blue"}:
                    self.side = name
                elif name == "config":
                    self.send_config()
                elif name == "clear":
                    self.clear_command()
                elif name == "stop":
                    self.send_simple(CMD_STOP)
                elif name == "abort":
                    self.send_simple(CMD_ABORT)
                elif name == "grab":
                    self.send_simple(CMD_GRAB_CONFIRMED)
                elif name == "enter":
                    self.write_command(MissionCommand(CMD_ENTER_SAFE_ZONE,
                                                       self.command_flags(straight=True)))
                    self.message = "已发送ENTER_SAFE_ZONE"
                elif name == "nav":
                    self.send_navigation(CMD_NAVIGATE_WAYPOINT)
                elif name == "return":
                    self.send_navigation(CMD_RETURN_CENTER)
                elif name == "complete":
                    self.send_simple(CMD_TASK_COMPLETE)
                elif name == "rotate90":
                    self.toggle_experiment("rotate_90deg")
                elif name == "rotate180":
                    self.toggle_experiment("rotate_180deg")
                elif name == "rotate360":
                    self.toggle_experiment("rotate_360deg")
                elif name == "forward1m":
                    self.toggle_experiment("forward_1m", self.start_forward_experiment)
                elif name == "lateral1m":
                    self.toggle_experiment("lateral_1m_turn_then_drive", self.start_lateral_experiment)
                elif name == "returntest":
                    self.toggle_experiment("turn_and_return", self.start_return_experiment)
                elif name == "dminus":
                    self.distance_mm = max(0, self.distance_mm - 100)
                elif name == "dplus":
                    self.distance_mm = min(5000, self.distance_mm + 100)
                elif name == "hminus":
                    self.heading_deg = (self.heading_deg - 10.0) % 360.0
                elif name == "hplus":
                    self.heading_deg = (self.heading_deg + 10.0) % 360.0
                break

    def mouse_callback(self, event, x, y, _flags, _parameter) -> None:
        if event == cv2.EVENT_LBUTTONUP:
            self.select_at(x, y)

    def handle_key(self, key: int) -> bool:
        key &= 0xFF
        if key in (ord("q"), ord("Q"), 27):
            return False
        if ord("1") <= key <= ord("4"):
            self.zone = key - ord("0")
        elif key in (ord("r"), ord("R")):
            self.side = "red"
        elif key in (ord("b"), ord("B")):
            self.side = "blue"
        elif key in (ord("c"), ord("C")):
            self.send_config()
        elif key in (ord("s"), ord("S")):
            self.send_simple(CMD_STOP)
        elif key in (ord("a"), ord("A")):
            self.send_simple(CMD_ABORT)
        elif key in (ord("g"), ord("G")):
            self.send_simple(CMD_GRAB_CONFIRMED)
        elif key in (ord("e"), ord("E")):
            self.write_command(MissionCommand(CMD_ENTER_SAFE_ZONE,
                                               self.command_flags(straight=True)))
            self.message = "已发送ENTER_SAFE_ZONE"
        elif key in (ord("n"), ord("N")):
            self.send_navigation(CMD_NAVIGATE_WAYPOINT)
        elif key in (ord("y"), ord("Y")):
            self.send_navigation(CMD_RETURN_CENTER)
        elif key in (ord("t"), ord("T")):
            self.send_simple(CMD_TASK_COMPLETE)
        elif key in (ord("o"), ord("O")):
            self.toggle_experiment("rotate_90deg")
        elif key in (ord("p"), ord("P")):
            self.toggle_experiment("rotate_180deg")
        elif key == ord("0"):
            self.toggle_experiment("rotate_360deg")
        elif key in (ord("v"), ord("V")):
            self.toggle_experiment("forward_1m", self.start_forward_experiment)
        elif key in (ord("l"), ord("L")):
            self.toggle_experiment("lateral_1m_turn_then_drive", self.start_lateral_experiment)
        elif key in (ord("x"), ord("X")):
            self.clear_command()
        elif key in (ord("u"), ord("U")):
            self.distance_mm = min(5000, self.distance_mm + 100)
        elif key in (ord("i"), ord("I")):
            self.distance_mm = max(0, self.distance_mm - 100)
        elif key in (ord("h"), ord("H")):
            self.heading_deg = (self.heading_deg + 10.0) % 360.0
        elif key in (ord("j"), ord("J")):
            self.heading_deg = (self.heading_deg - 10.0) % 360.0
        elif key in (ord("z"), ord("Z")):
            self.t265_points.clear()
            self.odom_points.clear()
            self.raw_points.clear()
            self.message = "已清除显示轨迹，日志仍保留"
        elif key in (ord("f"), ord("F")):
            self.fullscreen = not self.fullscreen
            cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN,
                                  cv2.WINDOW_FULLSCREEN if self.fullscreen else cv2.WINDOW_NORMAL)
        return True

    def run(self) -> int:
        self.start_session()
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow(WINDOW_NAME, self.width, self.height)
        cv2.imshow(WINDOW_NAME, self.render())
        cv2.waitKey(1)
        cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)
        if self.fullscreen:
            cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        try:
            while True:
                self.update_data()
                self.refresh_command()
                cv2.imshow(WINDOW_NAME, self.render())
                if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                    break
                if not self.handle_key(cv2.waitKey(20)):
                    break
        finally:
            self.stop_session()
            cv2.destroyWindow(WINDOW_NAME)
        return 0


def main() -> int:
    try:
        return DebugMapApp(parse_args()).run()
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
