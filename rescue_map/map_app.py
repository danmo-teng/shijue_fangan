#!/usr/bin/env python3
"""Interactive start selector and live rescue-field trajectory display."""

from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from field_model import (
    FIELD_HALF_M,
    Pose,
    Trajectory,
    initial_pose,
    load_localization_pose,
    write_localization_config,
    write_session,
)


ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = ROOT.parent
RUNTIME = ROOT / "runtime"
# OpenCV's Qt backend can create a window for a Unicode title but then fail to
# retrieve the native handle in cvSetMouseCallback.  Keep the internal key ASCII;
# all user-facing Chinese labels are drawn inside the canvas.
WINDOW_NAME = "rescue_field_map"
FONT_REGULAR = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
FONT_MEDIUM = "/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RDK X5 rescue-field selector and pose map")
    parser.add_argument("--zone", type=int, choices=range(1, 5), help="skip zone selection")
    parser.add_argument("--side", choices=("red", "blue"), help="skip side selection")
    parser.add_argument("--corner-offset-mm", type=float, default=300.0)
    parser.add_argument(
        "--localization-mode",
        choices=("fusion", "t265"),
        default="fusion",
        help="fusion uses T265 plus F407 encoders; t265 does not open UART",
    )
    parser.add_argument("--localization-json", type=Path)
    parser.add_argument("--launch-localization", action="store_true")
    parser.add_argument("--launch-vision", action="store_true")
    parser.add_argument("--uart", default="/dev/ttyS1")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--tx-rate",
        type=float,
        default=50.0,
        help="fused-pose UART transmit rate in Hz (default: 50)",
    )
    parser.add_argument("--fullscreen", action="store_true")
    parser.add_argument("--demo", action="store_true", help="animate a hardware-free pose")
    parser.add_argument("--screenshot", type=Path, help="render one frame and exit")
    return parser.parse_args()


class TextPainter:
    def __init__(self) -> None:
        self.items: list[tuple[str, tuple[int, int], int, tuple[int, int, int], bool, str]] = []
        self.fonts: dict[tuple[int, bool], ImageFont.FreeTypeFont] = {}

    def add(self, text, xy, size=22, color=(230, 230, 230), bold=False, anchor="la") -> None:
        self.items.append((str(text), tuple(map(int, xy)), size, color, bold, anchor))

    def paint(self, bgr: np.ndarray) -> np.ndarray:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(rgb)
        draw = ImageDraw.Draw(image)
        for text, xy, size, bgr_color, bold, anchor in self.items:
            key = (size, bold)
            if key not in self.fonts:
                font_path = FONT_MEDIUM if bold else FONT_REGULAR
                try:
                    self.fonts[key] = ImageFont.truetype(font_path, size=size)
                except OSError:
                    self.fonts[key] = ImageFont.load_default()
            color = (bgr_color[2], bgr_color[1], bgr_color[0])
            draw.text(xy, text, font=self.fonts[key], fill=color, anchor=anchor)
        self.items.clear()
        return cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)


class RescueMapApp:
    def __init__(self, options: argparse.Namespace) -> None:
        self.options = options
        # Match the RDK X5 desktop panel exactly; the square field remains
        # undistorted and the status panel occupies the remaining width.
        self.width, self.height = 1280, 1024
        self.map_left, self.map_top, self.map_size = 30, 60, 850
        self.zone = options.zone or 1
        self.side = options.side or "red"
        self.selecting = not (options.zone and options.side)
        self.corner_offset_m = options.corner_offset_mm / 1000.0
        self.localization_mode = options.localization_mode
        initial_pose(self.zone, self.corner_offset_m)  # validates parameters
        self.localization_json = options.localization_json or (RUNTIME / "localization_result.json")
        self.trajectory = Trajectory()
        self.pose = initial_pose(self.zone, self.corner_offset_m)
        self.localization_process: subprocess.Popen | None = None
        self.vision_process: subprocess.Popen | None = None
        self.fullscreen = options.fullscreen
        self.hitboxes: dict[str, tuple[int, int, int, int]] = {}
        self.message = "请选择出发区和红蓝方，然后开始"
        self.started_monotonic = time.monotonic()
        self.last_live_read_monotonic: float | None = None

    def world_to_pixel(self, x_m: float, y_m: float) -> tuple[int, int]:
        x = self.map_left + (x_m + FIELD_HALF_M) / (2 * FIELD_HALF_M) * self.map_size
        y = self.map_top + (FIELD_HALF_M - y_m) / (2 * FIELD_HALF_M) * self.map_size
        return int(round(x)), int(round(y))

    def world_rect(self, canvas, x0, y0, x1, y1, color, thickness=-1) -> None:
        left, bottom = self.world_to_pixel(min(x0, x1), min(y0, y1))
        right, top = self.world_to_pixel(max(x0, x1), max(y0, y1))
        cv2.rectangle(canvas, (left, top), (right, bottom), color, thickness, cv2.LINE_AA)

    def draw_field(self, canvas: np.ndarray, text: TextPainter) -> None:
        ml, mt, size = self.map_left, self.map_top, self.map_size
        cv2.rectangle(canvas, (ml, mt), (ml + size, mt + size), (218, 218, 218), -1)

        for coordinate in np.arange(-1.0, 1.01, 0.5):
            x0, y0 = self.world_to_pixel(coordinate, -1.5)
            x1, y1 = self.world_to_pixel(coordinate, 1.5)
            cv2.line(canvas, (x0, y0), (x1, y1), (190, 190, 190), 1, cv2.LINE_AA)
            x0, y0 = self.world_to_pixel(-1.5, coordinate)
            x1, y1 = self.world_to_pixel(1.5, coordinate)
            cv2.line(canvas, (x0, y0), (x1, y1), (190, 190, 190), 1, cv2.LINE_AA)

        # Safety zones: outer 660x360 mm, inner usable area 600x300 mm.
        for side, y0, y1, color in (
            ("red", 1.14, 1.50, (70, 70, 185)),
            ("blue", -1.50, -1.14, (190, 125, 25)),
        ):
            self.world_rect(canvas, -0.33, y0, 0.33, y1, (120, 45, 145), -1)
            inner_y0, inner_y1 = (1.20, 1.50) if side == "red" else (-1.50, -1.20)
            self.world_rect(canvas, -0.30, inner_y0, 0.30, inner_y1, color, -1)
            self.world_rect(canvas, -0.01, inner_y0, 0.01, inner_y1, (25, 25, 25), -1)
            if self.side == side:
                self.world_rect(canvas, -0.34, y0 - 0.01, 0.34, y1 + 0.01, (0, 245, 255), 4)
            label_y = 1.34 if side == "red" else -1.34
            px, py = self.world_to_pixel(0.0, label_y)
            text.add(("红方" if side == "red" else "蓝方") + ("·本方" if self.side == side else ""),
                     (px, py), 19, (245, 245, 245), True, "mm")
            section_y = 1.23 if side == "red" else -1.23
            left_label, right_label = (("物资", "伤员") if side == "red" else ("伤员", "物资"))
            left_x, left_y = self.world_to_pixel(-0.16, section_y)
            right_x, right_y = self.world_to_pixel(0.16, section_y)
            text.add(left_label, (left_x, left_y), 12, (245, 245, 245), False, "mm")
            text.add(right_label, (right_x, right_y), 12, (245, 245, 245), False, "mm")

        # Four 300x300 mm magenta start zones.
        zones = {
            1: (-1.50, 1.20, -1.20, 1.50),
            2: (1.20, 1.20, 1.50, 1.50),
            3: (-1.50, -1.50, -1.20, -1.20),
            4: (1.20, -1.50, 1.50, -1.20),
        }
        for zone, rect in zones.items():
            self.world_rect(canvas, *rect, (215, 35, 220), -1)
            self.world_rect(canvas, *rect, (0, 245, 255) if zone == self.zone else (80, 30, 80), 4 if zone == self.zone else 2)
            center_x = (rect[0] + rect[2]) / 2
            center_y = (rect[1] + rect[3]) / 2
            px, py = self.world_to_pixel(center_x, center_y)
            text.add(str(zone), (px, py), 34, (20, 20, 20), True, "mm")

        self.draw_speed_bumps(canvas)
        cv2.rectangle(canvas, (ml, mt), (ml + size, mt + size), (35, 35, 35), 4, cv2.LINE_AA)
        ox, oy = self.world_to_pixel(0.0, 0.0)
        cv2.line(canvas, (ox - 10, oy), (ox + 10, oy), (70, 70, 70), 2)
        cv2.line(canvas, (ox, oy - 10), (ox, oy + 10), (70, 70, 70), 2)
        text.add("场地中心 (0,0)", (ox + 12, oy - 10), 16, (70, 70, 70))
        text.add("3.00 m", (ml + size // 2, mt + size + 25), 18, (220, 220, 220), False, "mm")
        text.add("+Y", (ml + 8, mt + 8), 16, (70, 70, 70))
        text.add("+X", (ml + size - 8, mt + size - 8), 16, (70, 70, 70), False, "ra")

    def draw_speed_bumps(self, canvas: np.ndarray) -> None:
        color = (235, 235, 235)
        outline = (105, 105, 105)
        for sx in (-1, 1):
            for sy in (-1, 1):
                # Three vertical strips along the top/bottom boundary beside the zone.
                for index in range(3):
                    near_x = sx * (1.14 - index * 0.11)
                    x0, x1 = sorted((near_x, near_x + sx * 0.06))
                    y0, y1 = (1.20, 1.50) if sy > 0 else (-1.50, -1.20)
                    self.world_rect(canvas, x0, y0, x1, y1, color, -1)
                    self.world_rect(canvas, x0, y0, x1, y1, outline, 1)
                # Three horizontal strips along the left/right boundary.
                for index in range(3):
                    near_y = sy * (1.14 - index * 0.11)
                    y0, y1 = sorted((near_y, near_y + sy * 0.06))
                    x0, x1 = (-1.50, -1.20) if sx < 0 else (1.20, 1.50)
                    self.world_rect(canvas, x0, y0, x1, y1, color, -1)
                    self.world_rect(canvas, x0, y0, x1, y1, outline, 1)

    def draw_trajectory(self, canvas: np.ndarray) -> None:
        points = [self.world_to_pixel(x, y) for x, y in self.trajectory.points]
        if len(points) >= 2:
            cv2.polylines(canvas, [np.asarray(points, dtype=np.int32)], False, (20, 150, 245), 3, cv2.LINE_AA)

    def draw_robot(self, canvas: np.ndarray) -> None:
        pose = self.pose
        center = self.world_to_pixel(pose.x_m, pose.y_m)
        angle = math.radians(pose.yaw_deg)
        tip = self.world_to_pixel(pose.x_m + 0.24 * math.cos(angle), pose.y_m + 0.24 * math.sin(angle))
        color = {
            "GOOD": (50, 220, 65),
            "DEGRADED": (0, 185, 255),
            "LOST": (40, 40, 230),
            "STALE": (80, 80, 180),
            "NO_DATA": (80, 80, 180),
        }.get(pose.quality, (150, 150, 150))
        if abs(pose.x_m) > FIELD_HALF_M or abs(pose.y_m) > FIELD_HALF_M:
            color = (40, 40, 230)
        radius = max(10, int(round(0.12 / 3.0 * self.map_size)))
        cv2.circle(canvas, center, radius, (25, 25, 25), -1, cv2.LINE_AA)
        cv2.circle(canvas, center, radius, color, 3, cv2.LINE_AA)
        cv2.arrowedLine(canvas, center, tip, color, 5, cv2.LINE_AA, tipLength=0.30)

    def button(self, canvas, text: TextPainter, name, label, rect, selected=False, color=(80, 80, 80)) -> None:
        x0, y0, x1, y1 = rect
        fill = color if selected else (45, 45, 48)
        cv2.rectangle(canvas, (x0, y0), (x1, y1), fill, -1, cv2.LINE_AA)
        cv2.rectangle(canvas, (x0, y0), (x1, y1), (0, 225, 255) if selected else (105, 105, 110), 2, cv2.LINE_AA)
        text.add(label, ((x0 + x1) // 2, (y0 + y1) // 2), 21, (245, 245, 245), selected, "mm")
        self.hitboxes[name] = rect

    def draw_panel(self, canvas: np.ndarray, text: TextPainter) -> None:
        x0 = 920
        cv2.rectangle(canvas, (900, 0), (self.width, self.height), (26, 27, 30), -1)
        text.add("救援场地定位", (x0, 45), 30, (245, 245, 245), True)
        text.add("赛题图6 · 3000×3000 mm", (x0, 80), 17, (165, 165, 170))
        if self.selecting:
            text.add("选择出发区", (x0, 130), 22, (230, 230, 230), True)
            for idx in range(4):
                bx = x0 + (idx % 2) * 175
                by = 155 + (idx // 2) * 64
                zone = idx + 1
                self.button(canvas, text, f"zone{zone}", f"{zone}号", (bx, by, bx + 150, by + 48), zone == self.zone)
            text.add("选择本方颜色", (x0, 300), 22, (230, 230, 230), True)
            self.button(canvas, text, "red", "红方", (x0, 330, x0 + 150, 382), self.side == "red", (55, 55, 185))
            self.button(canvas, text, "blue", "蓝方", (x0 + 175, 330, x0 + 325, 382), self.side == "blue", (185, 110, 20))
            text.add("定位方式", (x0, 420), 22, (230, 230, 230), True)
            self.button(canvas, text, "fusion", "T265+编码器", (x0, 450, x0 + 150, 502), self.localization_mode == "fusion", (55, 125, 80))
            self.button(canvas, text, "t265", "仅T265", (x0 + 175, 450, x0 + 325, 502), self.localization_mode == "t265", (75, 100, 165))
            text.add("距角落顶点（直线距离）", (x0, 545), 20, (230, 230, 230), True)
            text.add(f"{self.corner_offset_m * 1000:.0f} mm", (x0 + 162, 578), 25, (245, 245, 245), True, "mm")
            self.button(canvas, text, "offset_minus_50", "−50", (x0, 600, x0 + 75, 648))
            self.button(canvas, text, "offset_minus_10", "−10", (x0 + 83, 600, x0 + 158, 648))
            self.button(canvas, text, "offset_plus_10", "+10", (x0 + 167, 600, x0 + 242, 648))
            self.button(canvas, text, "offset_plus_50", "+50", (x0 + 250, 600, x0 + 325, 648))
            self.button(canvas, text, "start", "确认并开始", (x0, 685, x0 + 325, 747), True, (35, 135, 70))
            text.add("快捷键：1–4、R/B、E切模式、Enter开始", (x0, 780), 14, (170, 170, 175))
            text.add("−/+调10 mm，[/]调50 mm", (x0, 803), 14, (170, 170, 175))
            pose = initial_pose(self.zone, self.corner_offset_m)
            text.add(f"初始 X={pose.x_m:+.3f} m  Y={pose.y_m:+.3f} m", (x0, 835), 17, (220, 220, 220))
            text.add(f"车头={pose.yaw_deg:.0f}°（倒车驶入场内）", (x0, 862), 17, (220, 220, 220))
            text.add(self.message, (x0, 910), 16, (0, 215, 255))
        else:
            pose = self.pose
            quality_color = {
                "GOOD": (50, 220, 65),
                "DEGRADED": (0, 185, 255),
                "WAITING": (160, 160, 165),
            }.get(pose.quality, (50, 80, 235))
            text.add(f"{self.zone}号出发区 / {'红方' if self.side == 'red' else '蓝方'}", (x0, 135), 23, (235, 235, 235), True)
            text.add(f"定位状态：{pose.quality}", (x0, 185), 21, quality_color, True)
            text.add(f"X：{pose.x_m:+.3f} m", (x0, 235), 25, (245, 245, 245), True)
            text.add(f"Y：{pose.y_m:+.3f} m", (x0, 275), 25, (245, 245, 245), True)
            text.add(f"方向：{pose.yaw_deg:06.2f}°", (x0, 315), 25, (245, 245, 245), True)
            text.add(f"融合轨迹：{self.trajectory.distance_m:.3f} m", (x0, 385), 23, (20, 170, 245), True)
            text.add(f"T265起点位移：{pose.t265_travel_m:.3f} m", (x0, 425), 18, (200, 200, 205))
            text.add(f"T265置信度：{pose.tracker_confidence}/{pose.mapper_confidence}", (x0, 475), 18, (200, 200, 205))
            text.add(f"定位方式：{'T265+编码器融合' if self.localization_mode == 'fusion' else '仅T265'}", (x0, 510), 18, (200, 200, 205))
            if self.localization_mode == "fusion":
                text.add(f"编码器UART：{'正常' if pose.uart_fresh else '超时'}", (x0, 545), 18, (200, 200, 205))
                text.add(f"轮速门控：{pose.wheel_gate}", (x0, 580), 17, (175, 175, 180))
            else:
                text.add("编码器融合：已关闭（UART未打开）", (x0, 545), 18, (200, 200, 205))
            age_text = "--" if math.isinf(pose.age_ms) else f"{pose.age_ms:.0f} ms"
            text.add(f"数据年龄：{age_text}", (x0, 615), 17, (175, 175, 180))
            if abs(pose.x_m) > FIELD_HALF_M or abs(pose.y_m) > FIELD_HALF_M:
                text.add("警告：融合坐标已越出场地边界", (x0, 650), 17, (40, 70, 235), True)
            text.add("S 重选  R 清轨迹  F 全屏  Q 退出", (x0, 700), 17, (180, 180, 185))
            if self.localization_process and self.localization_process.poll() is not None:
                text.add(f"定位进程已退出：{self.localization_process.returncode}", (x0, 735), 17, (50, 80, 235), True)
            elif self.message:
                text.add(self.message, (x0, 735), 16, (0, 190, 255))
            if self.vision_process and self.vision_process.poll() is not None:
                text.add(f"识别进程已退出：{self.vision_process.returncode}", (x0, 770), 17, (50, 80, 235), True)
            elif self.vision_process:
                text.add("YOLO识别：运行中", (x0, 770), 17, (50, 210, 80), True)

    def render(self) -> np.ndarray:
        canvas = np.full((self.height, self.width, 3), (18, 19, 21), dtype=np.uint8)
        text = TextPainter()
        self.hitboxes.clear()
        self.draw_field(canvas, text)
        self.draw_trajectory(canvas)
        self.draw_robot(canvas)
        self.draw_panel(canvas, text)
        return text.paint(canvas)

    def select_at(self, x: int, y: int) -> None:
        if not self.selecting:
            return
        for name, (x0, y0, x1, y1) in self.hitboxes.items():
            if x0 <= x <= x1 and y0 <= y <= y1:
                if name.startswith("zone"):
                    self.zone = int(name[-1])
                    self.pose = initial_pose(self.zone, self.corner_offset_m)
                elif name in {"red", "blue"}:
                    self.side = name
                elif name in {"fusion", "t265"}:
                    self.localization_mode = name
                elif name.startswith("offset_"):
                    offsets_mm = {
                        "offset_minus_50": -50.0,
                        "offset_minus_10": -10.0,
                        "offset_plus_10": 10.0,
                        "offset_plus_50": 50.0,
                    }
                    self.adjust_corner_offset(offsets_mm[name])
                elif name == "start":
                    self.start_session()
                break

    def adjust_corner_offset(self, delta_mm: float) -> None:
        candidate = self.corner_offset_m + delta_mm / 1000.0
        try:
            initial_pose(self.zone, candidate)
        except ValueError:
            self.message = "距角落顶点必须大于0且小于1500 mm"
            return
        self.corner_offset_m = candidate
        self.pose = initial_pose(self.zone, self.corner_offset_m)

    def mouse_callback(self, event, x, y, _flags, _parameter) -> None:
        if event == cv2.EVENT_LBUTTONUP:
            self.select_at(x, y)

    def start_session(self) -> None:
        self.stop_session_processes()
        self.trajectory.reset()
        self.pose = initial_pose(self.zone, self.corner_offset_m)
        self.trajectory.seed(self.pose.x_m, self.pose.y_m)
        self.last_live_read_monotonic = None
        RUNTIME.mkdir(parents=True, exist_ok=True)
        write_session(
            RUNTIME / "session.json",
            self.zone,
            self.side,
            self.corner_offset_m,
            self.localization_mode,
        )
        write_localization_config(
            PROJECT_ROOT / "localization/config/localization.example.conf",
            RUNTIME / "localization.conf",
            self.zone,
            self.corner_offset_m,
        )
        (RUNTIME / "delivery_contact_pose.json").unlink(missing_ok=True)
        self.selecting = False
        self.started_monotonic = time.monotonic()
        self.message = "等待T265和编码器融合数据" if self.localization_mode == "fusion" else "等待T265定位数据"
        if self.options.launch_localization and not self.options.screenshot:
            command = self.localization_command()
            try:
                self.localization_json.unlink(missing_ok=True)
                (RUNTIME / "uart_command.bin").unlink(missing_ok=True)
                (RUNTIME / "stm32_status.json").unlink(missing_ok=True)
                self.localization_process = subprocess.Popen(command, cwd=PROJECT_ROOT / "localization")
                self.message = "融合定位进程已启动" if self.localization_mode == "fusion" else "T265定位进程已启动"
            except OSError as exc:
                self.message = f"定位启动失败：{exc}"
        if self.options.launch_vision and not self.options.screenshot:
            try:
                command = self.vision_command()
                self.vision_process = subprocess.Popen(
                    command,
                    cwd=(PROJECT_ROOT / "mission_test"
                         if self.localization_mode == "fusion"
                         else PROJECT_ROOT / "vision"),
                )
            except OSError as exc:
                self.message = f"识别启动失败：{exc}"

    def localization_command(self) -> list[str]:
        """Build the localizer invocation for the selected hardware mode."""
        command = [
            str(PROJECT_ROOT / "localization/run_localization.sh"),
            "--config", str(RUNTIME / "localization.conf"),
            "--output", str(self.localization_json),
            "--rate", "20",
            "--tx-rate", str(self.options.tx_rate),
        ]
        if self.localization_mode != "fusion":
            return command
        command += [
            "--command-file", str(RUNTIME / "uart_command.bin"),
            "--stm-status", str(RUNTIME / "stm32_status.json"),
        ]
        if self.options.uart.lower() not in {"", "none", "off"}:
            command += ["--uart", self.options.uart, "--baud", str(self.options.baud)]
        return command

    def vision_command(self) -> list[str]:
        """Launch mission vision with UART fusion, or detection-only with T265."""
        if self.localization_mode == "fusion":
            return [
                str(PROJECT_ROOT / "mission_test/run_mission_test.sh"),
                "--window-mode", "normal",
                "--display-fps", "15",
            ]
        return [
            sys.executable,
            str(PROJECT_ROOT / "vision/run_detector.py"),
            "--device", "auto",
            "--window-mode", "normal",
            "--display-fps", "15",
        ]

    @staticmethod
    def stop_process(process: subprocess.Popen | None) -> None:
        if process is None or process.poll() is not None:
            return
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()

    def stop_vision(self) -> None:
        self.stop_process(self.vision_process)
        self.vision_process = None

    def stop_localization(self) -> None:
        self.stop_process(self.localization_process)
        self.localization_process = None

    def stop_session_processes(self) -> None:
        # Stop vision first so it releases the camera and stops producing UART
        # command files before the localization relay closes the serial port.
        self.stop_vision()
        self.stop_localization()

    def update_pose(self) -> None:
        if self.selecting:
            return
        if self.options.demo:
            elapsed = time.monotonic() - self.started_monotonic
            start = initial_pose(self.zone, self.corner_offset_m)
            distance = min(1.1, elapsed * 0.18)
            reverse_heading = math.radians(start.yaw_deg + 180.0)
            self.pose = Pose(
                start.x_m + distance * math.cos(reverse_heading),
                start.y_m + distance * math.sin(reverse_heading),
                start.yaw_deg,
                quality="GOOD",
                age_ms=5.0,
                tracker_confidence=3,
                mapper_confidence=3,
                t265_travel_m=distance,
                uart_fresh=True,
                wheel_gate="accepted" if distance > 0.7 else "startup_obstacle",
            )
            self.message = "演示模式（未读取真实硬件）"
        else:
            live = load_localization_pose(self.localization_json)
            if live is not None:
                self.pose = live
                self.last_live_read_monotonic = time.monotonic()
                self.message = ""
            else:
                now = time.monotonic()
                reference = self.last_live_read_monotonic or self.started_monotonic
                missing_ms = (now - reference) * 1000.0
                quality = "WAITING" if missing_ms <= 250.0 else "NO_DATA"
                self.pose = replace(self.pose, quality=quality, age_ms=missing_ms, uart_fresh=False)
        self.trajectory.update(self.pose)

    def handle_key(self, key: int) -> bool:
        key &= 0xFF
        if key in (ord("q"), ord("Q"), 27):
            return False
        if self.selecting:
            if ord("1") <= key <= ord("4"):
                self.zone = key - ord("0")
                self.pose = initial_pose(self.zone, self.corner_offset_m)
            elif key in (ord("r"), ord("R")):
                self.side = "red"
            elif key in (ord("b"), ord("B")):
                self.side = "blue"
            elif key in (ord("e"), ord("E")):
                self.localization_mode = "t265" if self.localization_mode == "fusion" else "fusion"
            elif key in (ord("-"), ord("_")):
                self.adjust_corner_offset(-10.0)
            elif key in (ord("+"), ord("=")):
                self.adjust_corner_offset(10.0)
            elif key == ord("["):
                self.adjust_corner_offset(-50.0)
            elif key == ord("]"):
                self.adjust_corner_offset(50.0)
            elif key in (10, 13):
                self.start_session()
        else:
            if key in (ord("s"), ord("S")):
                self.stop_session_processes()
                self.selecting = True
                self.message = "重新选择后需再次确认开始"
            elif key in (ord("r"), ord("R")):
                self.trajectory.seed(self.pose.x_m, self.pose.y_m)
            elif key in (ord("f"), ord("F")):
                self.fullscreen = not self.fullscreen
                cv2.setWindowProperty(
                    WINDOW_NAME,
                    cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN if self.fullscreen else cv2.WINDOW_NORMAL,
                )
                if not self.fullscreen:
                    cv2.resizeWindow(WINDOW_NAME, self.width, self.height)
        return True

    def run(self) -> int:
        if not self.selecting:
            self.start_session()
        if self.options.screenshot:
            if self.options.demo:
                now = time.monotonic()
                for step in range(31):
                    self.started_monotonic = now - 3.0 * step / 30.0
                    self.update_pose()
            self.options.screenshot.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(self.options.screenshot), self.render()):
                raise RuntimeError(f"cannot write screenshot: {self.options.screenshot}")
            self.stop_session_processes()
            return 0

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow(WINDOW_NAME, self.width, self.height)
        # Qt creates the native window lazily.  Present one frame and process an
        # event before registering callbacks, otherwise the handler may be NULL.
        cv2.imshow(WINDOW_NAME, self.render())
        cv2.waitKey(1)
        cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)
        if self.fullscreen:
            cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        try:
            while True:
                self.update_pose()
                cv2.imshow(WINDOW_NAME, self.render())
                if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                    break
                if not self.handle_key(cv2.waitKey(20)):
                    break
        finally:
            self.stop_session_processes()
            cv2.destroyWindow(WINDOW_NAME)
        return 0


def main() -> int:
    try:
        return RescueMapApp(parse_args()).run()
    except (ValueError, RuntimeError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
