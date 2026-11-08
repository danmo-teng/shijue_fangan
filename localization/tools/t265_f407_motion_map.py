#!/usr/bin/env python3
"""Existing desktop map adapted to the audited F405 NormalDebug contract.

ChassisTestClient is the only UART owner. The T265-only child receives no UART;
raw counts and F405 host cross-checks stay in this test path. Optional official
wheel input uses IPC to that SAME SDK child and requires qualified capture times.
"""
from __future__ import annotations

import argparse
import functools
import copy
import json
import math
import re
import signal
import subprocess
import sys
import time
import uuid
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
    MOTION_CMD_TURN_REL,
    MOTION_STATE_DONE,
    MOTION_STATE_FAULT,
    MOTION_STATE_STOPPED,
    STATE_NAMES,
    imu_is_fresh,
)
from f405_test_client import ChassisTestClient, IMPLEMENTATION_COMMIT, DOCUMENT_COMMIT, firmware_compatible
from t265_f407_debug import PROTOCOL, analyze, write_debug_config  # noqa: E402
from motion_log_storage import StoragePanel, list_sessions, delete_sessions, DYNAMIC_SESSION_FILES, WHEEL_SESSION_FILES
from t265_wheel_input import WheelInputBridge, SyntheticWheelInputBridge, write_sdk_config, DEPENDENCIES, TIME_POLICY


WINDOW_NAME = "t265_f407_motion_map"
FONT_REGULAR = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
FONT_MEDIUM = "/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc"
LOGICAL_WIDTH, LOGICAL_HEIGHT = 1280, 1024
FIELD_HALF_M = 1.5
START_HEADING_OFFSET_DEG = 45.0
DEBUG_COMMAND_FORWARD_SIGN = 1
DEBUG_COMMAND_LEFT_SIGN = 1
DEBUG_COMMAND_TURN_SIGN = -1
LOWER_FIRMWARE = IMPLEMENTATION_COMMIT
MOTION_STATUS_TIMEOUT_MS = 1000
MOTION_PROTOCOL = dict(PROTOCOL, uart="RDK /dev/ttyS1 to F405 UART4/JP506", board="F405_ROBO", motion_command_type="0x19", motion_status_type="0x1A",
                       turn_angle_unit="signed 0.1 degree", move_arguments="physical forward/left mm",
                       wheel_body_axes="F=(u2-u1)/sqrt(3), L=(2*u3-u1-u2)/3; ui=-K*raw_delta_i/dt; receive interval estimate only",
                       t265_heading="CCW positive, selected field start reference, wrapped -180..180 degrees",
                       motion_status_heading="F405 Location heading CCW/local, 0 also means unavailable; no health bits on 0x1A")


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
    "commands.jsonl", "analysis.json", "ground_truth.jsonl", "session_complete.json",
    "imu_telemetry.jsonl", "chassis_telemetry.jsonl", "chassis_actions.jsonl",
    "wheel_input.jsonl", "wheel_sdk.jsonl", "wheel_sdk_status.json", "wheel_sdk_config.json",
    "chassis_link.jsonl", "chassis_extended.jsonl", "chassis_identity.json",
    "wheel_measurements.jsonl","logging_events.jsonl","fisheye_request.json","fisheye_frames.csv","wheel_fixture.jsonl",
    "t265_usb_recovery.json"}) | DYNAMIC_SESSION_FILES


def prune_completed_sessions(root: Path, current: Path, keep: int = 5) -> list[str]:
    """Retain current plus the newest completed owned sessions; same UI boundary."""
    marker = read_json(current / "session_complete.json") or {}
    if marker.get("owner") != "t265_f407_motion_map" or marker.get("complete") is not True:
        return []
    rows = [r for r in list_sessions(root, current, SESSION_FILES) if not r["reason"]]
    rows.sort(key=lambda r: (r["order"], r["name"]), reverse=True)
    removed, _ = delete_sessions(root, rows[max(0, keep-1):], current, SESSION_FILES)
    return removed


class TextPainter:
    _font_cache={}
    def __init__(self, font_scale: float = 1.0) -> None:
        self.font_scale = max(1.0, float(font_scale))
        self.items: list[tuple] = []
        self.fonts = self._font_cache

    def add(self, value, xy, size=18, color=(230, 230, 230), bold=False, anchor="la", max_width=None) -> None:
        self.items.append((str(value), tuple(map(int, xy)),
                           max(12, int(round(size * self.font_scale))),
                           color, bold, anchor, max_width))

    def paint(self, canvas: np.ndarray) -> np.ndarray:
        image = Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
        for value, xy, size, color, bold, anchor, max_width in self.items:
            if max_width is None and anchor.startswith("l"):
                max_width = image.width - xy[0] - 12
            tile,origin=self._raster(value,size,tuple(color),bold,anchor,max_width)
            image.paste(tile,(xy[0]+origin[0],xy[1]+origin[1]),tile)
        self.items.clear()
        return cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)

    @staticmethod
    @functools.lru_cache(maxsize=512)
    def _raster(value,size,color,bold,anchor,max_width):
        # Repeated Chinese labels used to re-run FreeType under the GIL on
        # every render, delaying the same-process UART/wheel threads. Cache
        # only bounded transparent glyph images; positions/background vary.
        measure=ImageDraw.Draw(Image.new('L',(1,1)))
        while True:
            key=(size,bold)
            if key not in TextPainter._font_cache:
                try:font=ImageFont.truetype(FONT_MEDIUM if bold else FONT_REGULAR,size=size)
                except OSError:font=ImageFont.load_default()
                TextPainter._font_cache[key]=font
            font=TextPainter._font_cache[key]
            if max_width is None or measure.textlength(value,font=font)<=max_width or size<=9:break
            size-=1
        box=measure.textbbox((0,0),value,font=font,anchor=anchor)
        tile=Image.new('RGBA',(max(1,box[2]-box[0]),max(1,box[3]-box[1])))
        ImageDraw.Draw(tile).text((-box[0],-box[1]),value,font=font,
            fill=(color[2],color[1],color[0],255),anchor=anchor)
        return tile,(box[0],box[1])


# Historical class name retained for existing imports; there is one UART owner.
F407SerialBridge = ChassisTestClient


class MotionMapApp:
    def __init__(self, options: argparse.Namespace) -> None:
        cv2.setNumThreads(1)  # Small UI resizes must not spawn competing CPU workers.
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
        competition_logs=getattr(options,"log_module","test")=="competition"
        self.storage_panel = StoragePanel(options.output_root,
            WHEEL_SESSION_FILES if competition_logs else SESSION_FILES,
            owner="competition_wheel_input" if competition_logs else "t265_f407_motion_map")
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
        self.imu_telemetry: dict | None = None
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
        self.turn_rate_deg_s = 60
        self.active_host_action_id = None
        self.latest_wheel_reference = None
        self.wheel_reference_pose = None
        self.wheel_reference_valid = False
        self.test_id = 0
        self.active_test_parameters: dict = {}
        self.session_config: dict[str, str] = {}
        self.measurement_buffer: str | None = None
        self.settled_reference_mark: dict | None = None
        self.session_index = 0
        self.logging_failed = False
        self.last_chassis_identity_ns = 0
        self.wheel_input = None
        self.wheel_unsubscribe = None
        self.wheel_sdk_status = None
        self.fisheye_requests=0
        self.visual_disturbance_active = False

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
        text.add("SDK→车中心", (ml + 20, mt + size - 28), 14, (20, 150, 245), True)
        text.add("轮反馈参考(借用T265航向)", (ml + 170, mt + size - 28), 14, (220, 80, 220), True)
        text.add("SDK相机中心投影", (ml + 490, mt + size - 28), 14, (50, 170, 80), True)

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
        if not odom.get("available",False):
            odom = {}
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
        text.add("T265/F405 运动调试", (x0, 12), 22, (245, 245, 245), True)
        mode = getattr(self.options, "wheel_input", "off")
        wheel_state=self.wheel_input.snapshot(self.wheel_sdk_status) if self.wheel_input else {}
        labels={"NOT_REQUESTED":"未请求注入","REQUESTED_BLOCKED":"已请求/尚未加载","SDK_CONFIG_LOADED":"配置已加载/等待实测输入","SDK_API_ACCEPTED":"有SDK API成功样本"}
        text.add(f"{labels.get(wheel_state.get('state'),'轮速状态待取')} / 成功{wheel_state.get('sdk_api_success_samples',0)}", (x0, 43), 12,
                 (100, 190, 230))
        age=wheel_state.get("sdk_success_age_ms")
        text.add(f"SDK API实际接受 {wheel_state.get('sdk_api_rate_hz',0)} Hz(近1s) / 最近成功距今 {age:.1f}ms" if age is not None else "SDK API实际接受 0Hz / 尚无成功样本",(30,918),15,(220,220,220))
        sample_age=wheel_state.get("last_sample_age_ms")
        stream=wheel_state.get("stream") or {}
        stream_label="07/08已订阅" if stream.get("status")==0 and stream.get("enabled")==1 else "07/08未订阅或无回执"
        text.add((f"输入事件年龄(仅诊断)：{sample_age:.1f}ms" if sample_age is not None else "输入主机采样时间未知；按源区间采用")+" / "+stream_label,(30,938),13,(220,220,220))
        reason_labels={"unqualified_capture_time_or_epoch":"采样时标/epoch未合格",
            "mcu_clock_drift_bound_unknown":"MCU漂移界未知",
            "wheel_capture_time_bound_unknown_including_rear_EXTI":"轮反馈采集延迟界未知",
            "IMU_acquisition_filter_link_to_ISR_delay_unbounded":"IMU采集延迟界未知",
            "no_matched_clock_exchange":"尚无匹配对时", "boot_epoch_invalid":"boot epoch无效",
            "waiting_for_bracketing_gyro":"等待该轮区间的gyro样本"}
        reason=wheel_state.get("last_input_rejection")
        missing=wheel_state.get("qualification_missing") or wheel_state.get("clock",{}).get("missing",[])
        text.add("输入状态/时间诊断："+reason_labels.get(reason,str(reason))+" / "+','.join(reason_labels.get(r,r) for r in missing),(30,956),12,(100,190,230),max_width=840)
        text.add("轮反馈诊断参考不是独立真值；实际API成功也不等于实车精度通过",(30,972),12,(175,175,180))
        running = self.localizer is not None and self.localizer.poll() is None
        service_text = "身份已确认" if self.motion_identity_confirmed() else "仅监听"
        text.add(("定位运行" if running else "定位未运行") + " / " + service_text, (x0, 68), 16,
                 (50, 220, 65) if running else (50, 80, 230), True)
        if self.bridge is not None:
            link=self.bridge.connection_snapshot()
            fw=link["actual_lower_firmware"]
            text.add(f"底盘 {link['baud']} / 实际固件 {fw[:12]}", (28, 25), 15, (220,220,220))
        text.add(f"测试起点：(0,0)  初始航向 {start_pose(self.zone)[2]:.0f}°", (x0, 96), 15, (220, 220, 220))
        forward = self.session_config.get("camera_robot_forward_axis", "?")
        up = self.session_config.get("camera_robot_up_axis", "?")
        text.add(f"前向候选：{forward}", (x0, 123), 15, (220, 220, 220))
        up_world = self.localization.get("t265", {}).get("up_world")
        up_text = "待采集" if up_world is None else f"世界上向Y={up_world[1]:+.3f}"
        text.add(f"上向：{up} / {up_text}", (x0, 149), 13, (175, 175, 180))
        pose = self.localization.get("pose", {})
        try:
            text.add(f"车中心(C→B)：{float(pose['x_m']):+.3f} / {float(pose['y_m']):+.3f} m",
                     (x0, 180), 17, (20, 170, 245), True)
            text.add(f"T265方向：{float(pose['yaw_deg']):06.2f}°",
                     (x0, 207), 17, (20, 170, 245), True)
        except (KeyError, TypeError, ValueError):
            text.add("T265：等待数据", (x0, 190), 17, (170, 170, 175))
        odom = self.localization.get("wheel_odom", {})
        if not odom.get("available",False):
            odom = {}
        try:
            text.add(f"复算参考：{float(odom['x_m']):+.3f} / {float(odom['y_m']):+.3f} m",
                     (x0, 240), 16, (220, 80, 220), True)
        except (KeyError, TypeError, ValueError):
            text.add("复算参考：无有效数据", (x0, 240), 16, (180, 130, 180), True)
        measured=wheel_state.get("latest_measured")
        if measured and measured.get("v_body_mps") is not None:
            v=measured["v_body_mps"]
            quality="时间合格" if measured.get("capture_time_qualified") else "时间未合格"
            comparison_text=f"实测F/L {v[0]:+.3f}/{v[1]:+.3f}m/s {quality}"
            if measured["receive_age_ms"]>50:comparison_text+=" 接收过期"
        else:comparison_text = "下位机实测前/左车速：未提供"
        text.add(comparison_text,
                 (x0, 266), 16, (0, 205, 220))
        wheel = self.latest_wheel_reference
        text.add(f"raw计数：{wheel['raw_counts']}" if wheel else "raw计数：待收0x15", (x0,292),14,(50,170,80))
        if measured and measured.get("host_recomputed_v_body_mps") is not None:
            v=measured["host_recomputed_v_body_mps"]
            estimate=f"同区间计数复算F/L {v[0]:+.3f}/{v[1]:+.3f}m/s"
            if measured["receive_age_ms"]>50:estimate+=" 接收过期"
        else:
            estimate = (f"接收区间复算 F/L={wheel['forward_mps']:+.3f}/{wheel['left_mps']:+.3f}m/s" if
                        wheel and wheel["valid"] and time.monotonic_ns()-wheel["receive_monotonic_ns"] <= 250_000_000 else "上位机复算 F/L：不可用/过期")
        text.add(estimate,(x0,316),14,(50,170,80))

        status = self.motion_status
        state = int(status.get("state", 0)) if status else 0
        command = int(status.get("command", 0)) if status else 0
        fault = int(status.get("fault", 0)) if status else 0
        status_age = self.status_age(status)
        unit = "ddeg" if command == MOTION_CMD_TURN_REL else "mm"
        status_text = (f"F405 {STATE_NAMES.get(state, 'UNKNOWN')} {COMMAND_NAMES.get(command, 'UNKNOWN')} p={status.get('progress',0)}{unit} f={fault}"
                       if status else "F405：等待0x1A运动状态")
        if status is not None and status_age > MOTION_STATUS_TIMEOUT_MS:
            status_text += " 过期"
        text.add(status_text,
                 (x0, 350), 15, (245, 215, 150), True)
        unit = "0.1°" if command == MOTION_CMD_TURN_REL else "mm"
        heading = float(status.get("heading_cdeg", 0)) * 0.01
        progress_text = (f"进度={int(status.get('progress', 0))}{unit}  故障={fault}  IMU={heading:.1f}°"
                         if status else "运动进度及IMU航向：等待数据")
        imu = self.imu_telemetry
        if imu is not None:
            progress_text = f"1F yaw={imu['yaw_deg']:+.2f}° rate={imu['yaw_rate_deg_s']:+.1f}°/s"
        text.add(progress_text,
                 (x0, 374), 15, (245, 215, 150))
        stats = self.bridge.status_snapshot()[1] if self.bridge else {}
        text.add(f"帧 ODOM={stats.get('odom_frames', 0)} IMU={stats.get('imu_frames', 0)} STATUS={stats.get('motion_status_frames', 0)}",
                 (x0, 400), 14, (190, 190, 200))
        health = (f" 1F={imu['status']:02X} age={imu['age_ms']}ms " +
                  ("新鲜" if imu_is_fresh(imu, time.monotonic_ns()) else "无效/过期")) if imu else " 1F待收"
        text.add(f"CRC={stats.get('crc_errors', 0)}" + health,
                 (x0, 423), 14, (190, 190, 200))

        text.add(f"距离{self.distance_mm}mm / {self.speed_mm_s}mm/s / 转速{self.turn_rate_deg_s}°/s", (x0, 448), 16, (245, 245, 245), True)
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
        self.button(canvas, text, "returntest", "转向返航", (x0, 711, x0 + 152, 741),
                    self.active_test == "turn_and_return", (55, 100, 135))
        self.button(canvas,text,"resetreference","已停稳：重设参考",(x0+160,711,x0+330,741))
        command = self.last_sent_command
        text.add(f"最近发送：{command['name']} seq={command['sequence']}" if command else "最近发送：无",
                 (x0, 765), 15, (245, 215, 150))
        if command:
            text.add(command["description"], (x0, 788), 13, (220, 220, 220))
        text.add(f"实验{self.test_id}：{self.active_test or '无'} / {self.return_phase or ''}",
                 (x0, 814), 16, (0, 215, 255), True)
        text.add("V/B前后 L/H左右 O/P/0正转 1/2/3反转", (x0, 843), 13, (175, 175, 180))
        text.add("I/K距离±100 ,/.±10 U/J速度 [/]转速", (x0, 865), 13, (175, 175, 180))
        text.add("M记录停稳点；G录入独立实测；Y转向返航", (x0, 887), 13, (175, 175, 180))
        text.add("组合未提供；C短录鱼眼(需显式启用)", (x0, 909), 13, (175, 175, 180))
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
        if getattr(self.options, "logs_only", False):
            logical = np.zeros((LOGICAL_HEIGHT, LOGICAL_WIDTH, 3), dtype=np.uint8)
            self.hitboxes.clear()
            text = TextPainter(font_scale=1. / self.render_scale)
            text.add("测试日志管理（未启动传感器或底盘）", (30,20), 20)
            text.add("ESC / Q 或关闭窗口退出", (920,80), 16, max_width=340)
            logical = text.paint(logical)
        else:
            logical = self.render_logical()
        self.storage_panel.render(logical, self, TextPainter)
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
        if getattr(self, "wheel_input", None) is not None:
            self.wheel_input.set_context(dict(test_id=self.test_id, host_action_id=self.active_host_action_id,
                                       identity_basis="GUI receive-time context; not MCU capture identity"))
            self.wheel_sdk_status = read_json(self.run_dir / "wheel_sdk_status.json")
        if self.localization_json is not None:
            data = read_json(self.localization_json)
            if data is not None:
                timestamp = int(data.get("timestamp_monotonic_ns", 0))
                if timestamp != self.last_json_timestamp:
                    self.last_json_timestamp = timestamp
                    self.localization = data
                    self.append_point(self.t265_points,data.get("pose",{}).get("x_m"),data.get("pose",{}).get("y_m"))
                    tracking = data.get("t265",{}).get("tracking_origin",{})
                    self.append_point(self.raw_points,tracking.get("x_m"),tracking.get("y_m"))
        if self.bridge is not None:
            self.bridge.poll()
            link=self.bridge.connection_snapshot()
            if not link["motion_identity_ready"]:
                self.message=link["reason"] or "底盘身份未确认"
            if getattr(self,"last_chassis_identity_ns",0)!=self.bridge.identity_received_ns:
                self.last_chassis_identity_ns=self.bridge.identity_received_ns
                self.record_event("chassis_identity_observed",link)
            self.imu_telemetry = self.bridge.imu_snapshot()
            if self.bridge.log_error is not None:
                self.message = f"日志不完整（不阻断动作/注入）：{self.bridge.log_error}"
            status, _ = self.bridge.status_snapshot()
            if status is not None:
                self.motion_status = status
            for result in self.bridge.drain_action_results():
                if result["host_action_id"] != self.active_host_action_id:
                    continue
                self.record_event("chassis_action_result", result)
                if result.get("status") is not None:
                    self.handle_motion_status(result["status"])
                else:
                    self.active_test = None
                    self.return_phase = None
                    self.active_command_sequence = None
                    self.message = "动作状态未知，未续发；人工STOP并确认回执后再操作"
            for sample in self.bridge.drain_wheel_increments():
                self.latest_wheel_reference = sample
                if not sample["valid"]:
                    self.wheel_reference_valid = False
                    self.wheel_reference_pose = None
                    continue
                pose = self.localization.get("pose",{})
                if (not all(k in pose for k in ("x_m","y_m","yaw_deg")) or
                        self.localization.get("quality") == "LOST"):
                    self.wheel_reference_valid = False
                    self.wheel_reference_pose = None
                    continue
                if self.wheel_reference_pose is None:
                    self.wheel_reference_pose = dict(pose)
                # Diagnostic drawing only: latest T265 heading, not a qualified
                # simultaneous yaw sample, and never an input to T265/output pose.
                yaw = math.radians(pose["yaw_deg"])
                f,l = sample["forward_delta_m"],sample["left_delta_m"]
                self.wheel_reference_pose["x_m"] += math.cos(yaw)*f-math.sin(yaw)*l
                self.wheel_reference_pose["y_m"] += math.sin(yaw)*f+math.cos(yaw)*l
                self.wheel_reference_pose["yaw_deg"] = pose["yaw_deg"]
                self.wheel_reference_valid = True
                self.append_point(self.odom_points,self.wheel_reference_pose["x_m"],self.wheel_reference_pose["y_m"])
        self.localization["wheel_odom"] = dict(self.wheel_reference_pose or {},available=self.wheel_reference_valid,
            source="host_f405_counts_latest_t265_heading_diagnostic_only")

    def record_event(self, name: str, details: dict | None = None) -> bool:
        if self.events_path is None:
            return False
        event = {
            "timestamp_monotonic_ns": time.monotonic_ns(),
            "wall_time": datetime.now().isoformat(timespec="milliseconds"),
            "event": name,
            "details": details or {},
            "test_id": self.test_id,
            "host_action_id": self.active_host_action_id,
            "test_parameters": dict(self.active_test_parameters),
            "localization": {
                "timestamp_monotonic_ns": self.localization.get("timestamp_monotonic_ns"),
                "pose": self.localization.get("pose"),
                "t265": self.localization.get("t265", {}),
                "wheel_odom": self.localization.get("wheel_odom", {}),
            },
            "motion_status": self.motion_status,
            "imu_native_unaligned": self.imu_telemetry,
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
                "host_action_id": self.active_host_action_id,
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

    def _submit_action(self, operation, description: str) -> int:
        if self.bridge is None:
            self.message = "底盘测试客户端尚未连接"
            return -1
        try:
            action = operation()
        except (RuntimeError, ValueError, serial.SerialException) as error:
            self.message = str(error)
            self.record_event("chassis_request_not_sent", {"reason": str(error)})
            self.active_test = None
            self.return_phase = None
            return -1
        self.active_host_action_id = action["host_action_id"]
        self.active_command_sequence = action["wire_sequence"]
        self.active_command_opcode = action["opcode"]
        self.active_command_sent_ns = action["sent_ns"]
        self.last_matching_status_ns = action["sent_ns"]
        self.record_command(action["name"], action["wire_sequence"], bytes.fromhex(action["packet_hex"]), description)
        self.record_event("chassis_action_submitted", action)
        self.message = f"已请求 {action['name']} seq={action['wire_sequence']}，等待匹配回执"
        return action["wire_sequence"]

    def send_stop(self, reason: str = "手动STOP") -> None:
        if self.bridge is None:
            return
        sequence = self._submit_action(self.bridge.stop, reason)
        if sequence >= 0:
            self.message = f"STOP已请求 seq={sequence}，等待匹配STOPPED；不代表已停稳"

    def send_turn(self, angle_deg: float) -> int:
        self.active_test_parameters.update(turn_deg_target=angle_deg, rate_deg_s=self.turn_rate_deg_s)
        self.record_event("turn_requested", {"application_ccw_deg":angle_deg,
                                             "rate_deg_s":self.turn_rate_deg_s})
        return self._submit_action(lambda:self.bridge.turn_ccw(angle_deg,self.turn_rate_deg_s),
                                   f"逆时针 {angle_deg:+.1f}°，角速度上限 {self.turn_rate_deg_s}°/s")

    def send_move(self, direction_deg: float, distance_mm: int) -> int:
        self.active_test_parameters.update(direction_deg_target=direction_deg,distance_mm_target=distance_mm)
        a = math.radians(direction_deg)
        forward, left = round(distance_mm*math.cos(a)), round(distance_mm*math.sin(a))
        self.record_event("move_requested", {"forward_mm":forward,"left_mm":left,"speed_mm_s":self.speed_mm_s})
        return self._submit_action(lambda:self.bridge.move_body(forward,left,self.speed_mm_s),
                                   f"前{forward:+d}/左{left:+d}mm，速度{self.speed_mm_s}mm/s")

    def reset_reference(self) -> None:
        if not self.motion_identity_confirmed():
            self.record_blocked_request("reset_reference",self.motion_identity_problem())
            return
        self._submit_action(lambda:self.bridge.reset_reference(stationary_confirmed=True),
                            "操作者确认停稳后重建下位机Location参考；不重置T265或原始gyro")

    def toggle_test(self, name: str, action) -> None:
        if not self.motion_identity_confirmed():
            self.record_blocked_request(name,self.motion_identity_problem())
            return
        if self.active_test == name:
            self.record_event(f"{name}_end_manual")
            self.active_test = None
            self.return_phase = None
            self.send_stop("实验按钮再次点击，中止当前动作")
            return
        if self.bridge is not None and (self.bridge.pending is not None or self.bridge.needs_stop_ack):
            self.record_blocked_request(name,"请先结束当前动作；需要取消时STOP并等待匹配回执")
            return
        self.active_test = name
        self.return_phase = None
        self.test_id += 1
        self.settled_reference_mark = None
        self.active_test_parameters = {"name":name,"distance_mm":self.distance_mm,"speed_mm_s":self.speed_mm_s}
        self.record_event(f"{name}_start")
        action()

    def record_blocked_request(self, name: str, reason: str) -> None:
        self.message="未发送："+reason
        self.record_event("chassis_request_not_sent",dict(requested_test=name,reason=reason,
            distance_mm=self.distance_mm,speed_mm_s=self.speed_mm_s,rate_deg_s=self.turn_rate_deg_s,
            uart_motion_write_attempted=False))

    def motion_identity_problem(self) -> str:
        if self.bridge is not None and self.bridge.serial is not None:
            return self.bridge.control_identity_problem() or "身份检查未通过"
        return "需确认实际F405 NormalDebug/MOTIONDEBUG服务及已审查固件"

    def motion_identity_confirmed(self) -> bool:
        if self.bridge is not None and self.bridge.serial is not None:
            return self.bridge.control_identity_problem() is None
        value = self.options.actual_lower_firmware.strip().lower()
        # UART4 cannot attest a firmware commit. Requiring an unattestable
        # hash blocked the desktop even after the operator confirmed the test
        # service. Keep unknown firmware explicitly unknown; reject a known
        # incompatible commit instead of silently treating it as this contract.
        return (self.options.actual_lower_mode == "NormalDebug" and
                self.options.actual_lower_service == "MOTIONDEBUG" and
                firmware_compatible(value))

    def save_ground_truth(self, text: str) -> None:
        """Signed, independently measured displacement/turn for the current test.
        A full turn remains +/-360, never wrapped or inferred from the command.
        """
        if self.active_test is not None or self.test_id <= 0:
            raise ValueError("先结束动作，停稳后再录入卷尺/地面角度测量")
        marker = self.settled_reference_mark
        if marker is None or marker["test_id"] != self.test_id:
            raise ValueError("请先人工确认停稳并按M冻结终点，再用G录入实测值")
        parts = text.split(",")
        if len(parts) != 3:
            raise ValueError("格式为前向mm,左向mm,转角deg；未知填?")
        values = [None if p.strip() == "?" else float(p) for p in parts]
        if all(v is None for v in values) or any(v is not None and not math.isfinite(v) for v in values):
            raise ValueError("至少录入一项有限实测值")
        if not self.options.reference_description.strip():
            raise ValueError("启动时用--reference-description记录卷尺零点和地面角度基准")
        document = {"test_id": self.test_id, "timestamp_monotonic_ns": time.monotonic_ns(),
            "measurement_end_monotonic_ns": marker["timestamp_monotonic_ns"],
            "pose_sample_monotonic_ns":marker["localization"].get("timestamp_monotonic_ns"),
            "pose_age_at_mark_ms":marker.get("pose_age_at_mark_ms"),
            "endpoint_reference": "operator_settled_M_mark",
            "source": "operator_tape_and_ground_reference", "reference": self.options.reference_description,
            "forward_mm": values[0], "left_mm": values[1], "turn_deg_unwrapped": values[2],
            "test_parameters": self.active_test_parameters,
            "localization": copy.deepcopy(marker["localization"]),
            "motion_status": copy.deepcopy(marker["motion_status"])}
        with (self.run_dir / "ground_truth.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(document, ensure_ascii=False) + "\n")
        self.record_event("ground_truth_recorded", document)
        self.message = f"已保存实验{self.test_id}的独立实测值"

    def mark_settled_reference(self) -> None:
        """Freeze the existing manual M marker before measuring/typing.
        The operator declares settling; no motion ACK/gyro value proves it.
        """
        if (self.test_id <= 0 or self.active_test is not None or
                (self.bridge is not None and self.bridge.pending is not None)):
            raise ValueError("先结束本段动作，再人工确认停稳并标记")
        now = time.monotonic_ns()
        stamp = self.localization.get("timestamp_monotonic_ns", 0)
        if (not stamp or
                self.localization.get("quality") == "LOST" or
                self.localization.get("t265",{}).get("tracker_confidence",0) <= 0):
            raise ValueError("T265尚无可记录位姿，未冻结终点")
        self.settled_reference_mark = dict(test_id=self.test_id,timestamp_monotonic_ns=now,
            pose_age_at_mark_ms=(now-stamp)/1_000_000,
            source="operator_declared_settled_not_controller_or_gyro_truth",
            localization=copy.deepcopy(self.localization), motion_status=copy.deepcopy(self.motion_status))
        self.record_event("manual_settled_reference_mark", self.settled_reference_mark)
        self.message = f"已冻结实验{self.test_id}末帧（年龄{(now-stamp)/1_000_000:.0f}ms，仅记录）；测量后G录入"

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
        if (self.active_command_sequence is None or not status.get("correlated") or
                status.get("host_action_id") != self.active_host_action_id or
                status.get("command_sequence") != self.active_command_sequence or
                status.get("command") != self.active_command_opcode or
                status.get("timestamp_monotonic_ns",0) <= self.active_command_sent_ns):
            return
        state = status["state"]
        if state == MOTION_STATE_DONE and not status.get("fault"):
            if self.active_test == "turn_and_return" and self.return_phase == "turn":
                if status.get("running_seen"):
                    self.record_event("turn_and_return_turn_done")
                    self.return_phase = "move"
                    self.send_move(0.0,self.return_distance_mm)
                    return
                self.message = "仅收到终态，缺RUNNING证据，已取消自动返航后段"
            else:
                self.message = "收到匹配DONE；指令完成不等于地面精度或已停稳"
        elif state == MOTION_STATE_STOPPED and not status.get("fault"):
            self.message = "收到匹配STOPPED；仍需人工确认车身停稳"
        elif state == MOTION_STATE_FAULT or status.get("fault"):
            self.message = f"下位机报告FAULT={status.get('fault')}，本动作结束"
        else:
            return
        self.record_event(f"{self.active_test or 'manual'}_end_status", {"status":status})
        self.active_test = None
        self.return_phase = None
        self.active_command_sequence = None

    def select_at(self, x: int, y: int) -> None:
        if getattr(self, "storage_panel", None) and self.storage_panel.click(x, y, self):
            return
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
            elif name == "resetreference":
                self.reset_reference()
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
            if getattr(self.options, "logs_only", False):
                self.storage_panel.click(logical_x, logical_y, self)
            else:
                self.select_at(logical_x, logical_y)

    def extra_localizer_args(self):
        return []

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
                           center_origin=getattr(self.options,"center_origin",True),
                           start_heading_offset_deg=getattr(self.options,"heading_offset_deg",START_HEADING_OFFSET_DEG),
                           unit_translation_scale=True)
        self.session_config = config_values(config_path)
        self.localization_json = self.run_dir / "localization_result.json"
        self.motion_status_path = self.run_dir / "f407_motion_status.json"
        self.events_path = self.run_dir / "events.jsonl"
        self.commands_path = self.run_dir / "commands.jsonl"
        status_log = self.run_dir / "f407_motion_status.jsonl"
        usb_recovery=None
        if getattr(self.options,"t265_usb_recovery",False):
            from t265_usb_recovery import recover_t265
            # Before UART subscriptions and the sole SDK process are started.
            usb_recovery=recover_t265(self.run_dir/"t265_usb_recovery.json")
        synthetic=getattr(self.options,"wheel_fixture","off")!="off"
        if not synthetic:
            self.bridge = F407SerialBridge(
                self.options.uart, self.options.baud,
                self.motion_status_path, status_log,
                identity_policy=getattr(self.options,"chassis_profile","auto"),
                asserted_firmware=self.options.actual_lower_firmware,
            )
            self.bridge.identity_confirmed = self.motion_identity_confirmed()
        sdk_session_id = uuid.uuid4().hex
        wheel_enabled = getattr(self.options, "wheel_input", "off") == "on"
        wheel_noise = getattr(self.options, "wheel_noise_std_mps", .10)
        wheel_config = write_sdk_config(self.run_dir / "wheel_sdk_config.json", wheel_noise)
        bridge_type=SyntheticWheelInputBridge if synthetic else WheelInputBridge
        self.wheel_input = bridge_type(self.run_dir, self.session_config, sdk_session_id, wheel_enabled,
            **({"plan":self.options.wheel_fixture} if synthetic else {}))
        if self.bridge:self.wheel_unsubscribe = self.bridge.telemetry(self.wheel_input.telemetry,readonly=True)
        self.wheel_input.start()
        if self.bridge:
            self.bridge.start()
            if wheel_enabled or getattr(self.options,"timed_measurements",False):self.bridge.start_timed_measurements()
            link=self.bridge.connection_snapshot()
        else:
            link=dict(identity=None,motion_identity_ready=False,reason="synthetic SDK experiment; chassis UART not opened",
                mcu_boot_epoch=None,expected_lower_firmware=None,contract_document_commit=None,firmware_identity_reported_by_uart=False)
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
            "--ignore-encoders",  # UART belongs solely to ChassisTestClient.
            "--debug-fresh-session",
            "--debug-test-session",sdk_session_id,

        ]
        if self.options.serial:
            command += ["--serial", self.options.serial]
        if getattr(self.options,"fisheye_evidence",False):
            command += ["--debug-fisheye-output",str(self.run_dir),"--debug-fisheye-request",str(self.run_dir/"fisheye_request.json")]
        if wheel_enabled:
            command += ["--debug-wheel-socket", "@t265-wheel-"+sdk_session_id,
                        "--debug-wheel-config", str(self.run_dir / "wheel_sdk_config.json"),
                        "--debug-wheel-log", str(self.run_dir / "wheel_sdk.jsonl"),
                        "--debug-wheel-session", sdk_session_id]
        if synthetic:command += ["--debug-wheel-synthetic"]
        command += self.extra_localizer_args()
        self.localizer = subprocess.Popen(
            command, cwd=str(LOCALIZATION_ROOT),
            stdout=self.localizer_log, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        metadata = {
            "schema_version": 4,
            "created_local": datetime.now().astimezone().isoformat(timespec="seconds"),
            "timestamp_monotonic_ns": time.monotonic_ns(),
            "mode": "interactive_f405_normaldebug",
            "session_purpose":"sensor_lifecycle_check_no_motion" if getattr(self.options,"sensor_check_seconds",0) else "interactive_motion_test",
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
            "pose_source": "t265_sdk_pose_with_camera_offset" if wheel_enabled else "t265_direct_with_camera_offset",
            "sdk_session_id": sdk_session_id,
            "sdk_owner_pid": self.localizer.pid,
            "device_boot_epoch": None,
            "device_restart_note":getattr(self.options,"device_restart_note","not_confirmed"),
            "clean_device_restart_verified":False,
            "fisheye_evidence_enabled":getattr(self.options,"fisheye_evidence",False),
            "wheel_input_requested": wheel_enabled,
            "timed_measurements_requested": wheel_enabled or getattr(self.options,"timed_measurements",False),
            "wheel_processing": "bounded background worker; no GUI poll dependency; asynchronous logging",
            "wheel_input_time_policy": TIME_POLICY,
            "wheel_input_definition": "v_C^C=R_CB*(v_B^B+omega_B^B cross r_BC^B)",
            "wheel_sdk_config": wheel_config,
            "wheel_noise_basis": f"engineering initial sigma={wheel_noise}m/s, variance={wheel_noise**2}(m/s)^2; not measured calibration; no per-sample covariance",
            "trajectory_dependency_sources": [],
            "potential_input_dependency_sources": DEPENDENCIES if wheel_enabled else [],
            "wheel_dependency_state_at_start": "requested_not_proven" if wheel_enabled else "not_requested",
            "wheel_and_primary_gyro_independent_of_sdk_input": not wheel_enabled,
            "wheel_reference_independent_pose": False,
            "ab_policy": "separate SDK sessions, no live toggle; stopping sends does not remove dependency; clean device restart for strict AB",
            "listen_and_control": self.motion_identity_confirmed(),
            "host_chassis_session_id": self.bridge.host_session_id if self.bridge else None,
            "mcu_boot_epoch": link["mcu_boot_epoch"],
            "expected_lower_firmware": link["expected_lower_firmware"],
            "expected_lower_mode": "NormalDebug",
            "expected_lower_service": "MOTIONDEBUG",
            "contract_document_commit": link["contract_document_commit"],
            "chassis_link": link,
            "capabilities": self.bridge.capabilities() if self.bridge else {},
            "historical_flash_evidence_is_current_identity": False,
            "lower_measured_body_velocity": "new_stream_available_but_not_subscribed" if link["identity"] else "not_provided_on_legacy_UART4",
            "wheel_reference": "host_count_recompute_receive_time_and_latest_T265_heading; diagnostic only",
            "actual_lower_service": link["actual_lower_service"] if link["identity"] else self.options.actual_lower_service,
            "actual_lower_firmware": link["actual_lower_firmware"] if link["identity"] else self.options.actual_lower_firmware,
            "actual_lower_mode": link["actual_lower_mode"] if link["identity"] else self.options.actual_lower_mode,
            "operator_lower_profile": dict(mode=self.options.actual_lower_mode,service=self.options.actual_lower_service,
                                           firmware=self.options.actual_lower_firmware),
            "lower_identity_source": "nonce-correlated device reply" if link["identity"] else "operator profile only; no device reply",
            "firmware_commit_reported_by_uart": link["firmware_identity_reported_by_uart"],
            "firmware_commit_attested_by_uart": False,
            "motion_enable_policy": "921600/230400 timed uses actual compatible board/mode/service and existing motion status; no data-age gate; explicit 115200 legacy retains operator profile",
            "mount_status": "candidate_not_physically_validated",
            "reference_description": self.options.reference_description,
            "ground_truth_policy": "command target and DONE are not ground truth; operator measures signed displacement/turn after settling",
            "protocol": dict(MOTION_PROTOCOL,baud=self.options.baud,extended_identity_namespace="D4 4D v1"),
            "localizer_command": command,
            "files": {
                "localization_csv": "localization_debug.csv",
                "raw_pose_csv": "t265_raw_pose.csv",
                "ground_truth": "ground_truth.jsonl",
                "imu_telemetry": "imu_telemetry.jsonl",
                "chassis_telemetry": "chassis_telemetry.jsonl",
                "chassis_actions": "chassis_actions.jsonl",
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
                "wheel_input": "wheel_input.jsonl",
                "wheel_sdk": "wheel_sdk.jsonl",
                "wheel_sdk_status": "wheel_sdk_status.json",
                "wheel_sdk_config": "wheel_sdk_config.json",
                "chassis_identity": "chassis_identity.json",
                "chassis_link": "chassis_link.jsonl",
                "chassis_extended": "chassis_extended.jsonl",
                "wheel_measurements":"wheel_measurements.jsonl",
            },
        }
        if synthetic:
            metadata.update(session_purpose="synthetic_sdk_response_no_motion",wheel_input_source="synthetic_test_fixture",
                actual_lower_firmware="not_opened",actual_lower_mode="not_opened",actual_lower_service="not_opened",
                lower_identity_source="chassis_not_opened",listen_and_control=False,timed_measurements_requested=False,
                potential_input_dependency_sources=["synthetic_test_fixture"],chassis_uart_opened=False,
                fixture_plan=self.wheel_input.PHASES,physical_accuracy_test=False,
                wheel_and_primary_gyro_independent_of_sdk_input=None,wheel_reference=None,
                lower_measured_body_velocity="chassis_not_opened",motion_enable_policy="disabled_no_chassis_uart",
                ground_truth_policy="synthetic SDK response only; no independent physical measurement",
                ab_policy="synthetic response experiment; prior SDK/device state unverified; user restart before physical baseline")
        try:
            metadata["upper_git_commit"] = subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=PROJECT_ROOT, text=True).strip()
            metadata["upper_tracked_changes"] = subprocess.check_output(
                ["git", "diff", "--name-only", "HEAD"], cwd=PROJECT_ROOT, text=True).splitlines()
        except (OSError, subprocess.SubprocessError):
            metadata["upper_git_commit"] = "unavailable"
        (self.run_dir / "metadata.json").write_text(
            json.dumps(dict(metadata,t265_usb_recovery=usb_recovery), ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        self.message = f"底盘{self.options.baud}："+("身份已读回，动作须人工触发；轮速尚未注入" if link["motion_identity_ready"] else str(link["reason"]))
        self.record_event("session_started", {"initial_pose": metadata["initial_pose"]})
        print(f"测试日志目录：{self.run_dir}", flush=True)

    def stop_session(self) -> None:
        wheel_summary=None
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
            self.bridge.close()
            logs_ok = logs_ok and self.bridge.log_error is None
            self.bridge = None
        if self.wheel_unsubscribe is not None:
            self.wheel_unsubscribe()
            self.wheel_unsubscribe = None
        if self.wheel_input is not None:
            wheel_summary=self.wheel_input.snapshot(read_json(self.run_dir / "wheel_sdk_status.json"))
            self.record_event("wheel_input_session_summary",dict(wheel_summary,accuracy_verified=False))
            self.wheel_input.close()
            logs_ok=(logs_ok and self.wheel_input.log.healthy()
                     and wheel_summary.get("sdk_logging_healthy") is not False)
            self.wheel_input = None
        if getattr(self, "storage_panel", None):
            self.storage_panel.close()
        if self.run_dir is not None:
            csv_path = self.run_dir / "localization_debug.csv"
            if csv_path.exists():
                try:
                    result = analyze(csv_path, None, "unknown",
                                     float(self.session_config.get("camera_offset_forward_m", "-0.0296")),
                                     float(self.session_config.get("camera_offset_left_m", "-0.0301")),
                                     fit_wheel=False)
                    result["angle_crossings_not_settled_measurements"] = result.pop("rotation_check")
                    result["protocol"] = dict(MOTION_PROTOCOL,baud=self.options.baud)
                    result["child_wheel_reference_available"] = False
                    result["wheel_reference_source"] = "chassis_telemetry.jsonl: host recomputation; not lower-published velocity"
                    for key in list(result.get("trajectory",{})):
                        if "wheel" in key:
                            result["trajectory"][key] = None
                    result["effective_config"] = self.session_config
                    result["accuracy_verified"] = False
                    result["wheel_input_requested"] = getattr(self.options, "wheel_input", "off") == "on"
                    sdk=read_json(self.run_dir / "wheel_sdk_status.json") or {}
                    dependencies=["synthetic_test_fixture"] if getattr(self.options,"wheel_fixture","off")!="off" else DEPENDENCIES
                    result["trajectory_dependency_sources"] = dependencies if (sdk.get("sdk_api_success_samples",0)>0 or sdk.get("wheel_dependency_suspect")) else []
                    result["potential_input_dependency_sources"] = dependencies if result["wheel_input_requested"] else []
                    result["wheel_reference_independent_pose"] = False  # Plot uses T265 heading in both modes.
                    result["wheel_sdk_status"] = read_json(self.run_dir / "wheel_sdk_status.json")
                    result["wheel_input_pipeline"]=wheel_summary
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
                self.record_event("session_retention", {"policy":"capacity_warning_manual_review",
                    "automatic_deletion":False,"removed":[]})

    def handle_key(self, key: int) -> bool:
        if getattr(self.options, "logs_only", False):
            return key & 255 not in (27, ord("q"), ord("Q"))
        if getattr(self, "storage_panel", None) and self.storage_panel.key(key, self):
            return True
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
            try:
                self.mark_settled_reference()
            except ValueError as error:
                self.message = str(error)
        elif key in (ord("g"), ord("G")):
            if self.active_test is None and self.test_id > 0:
                self.measurement_buffer = ""
                self.message = "输入前向mm,左向mm,转角deg；未知?；回车保存/Esc取消"
            else:
                self.message = "先结束动作，再人工确认停稳并测量"
        elif key in (ord("r"), ord("R")):
            self.reset_reference()
        elif key in (ord("d"), ord("D")):
            self.visual_disturbance_active = not self.visual_disturbance_active
            self.record_event("visual_disturbance_start" if self.visual_disturbance_active else "visual_disturbance_end",
                              {"source": "operator_marker", "independent_ground_truth": False})
            self.message = "视觉受扰开始标记" if self.visual_disturbance_active else "视觉受扰结束标记"
        elif key == ord("["):
            self.turn_rate_deg_s = max(10,self.turn_rate_deg_s-10)
        elif key == ord("]"):
            self.turn_rate_deg_s = min(360,self.turn_rate_deg_s+10)
        elif key in (ord("y"), ord("Y")):
            self.toggle_test("turn_and_return", self.start_return)
        elif key in (ord("c"),ord("C")):
            if not getattr(self.options,"fisheye_evidence",False):
                self.message="本会话未启用鱼眼证据；用 --fisheye-evidence 新开同一调试入口"
            elif self.fisheye_requests>=2:self.message="本会话2次短录额度已用完"
            else:
                self.fisheye_requests+=1
                request=dict(request_id=self.fisheye_requests,test_id=self.test_id,host_action_id=self.active_host_action_id)
                path=self.run_dir/"fisheye_request.json";tmp=path.with_suffix(".json.tmp")
                tmp.write_text(json.dumps(request)+"\n");tmp.replace(path)
                self.record_event("fisheye_evidence_requested",request)
                self.message="请求同一SDK进程短录2秒/4Hz鱼眼画面；不是视觉原因已确诊"
        elif key in (ord("f"), ord("F")):
            self.options.fullscreen = not self.options.fullscreen
            cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN,
                                  cv2.WINDOW_FULLSCREEN if self.options.fullscreen else cv2.WINDOW_NORMAL)
        return True

    def validate_sensor_check(self):
        """Evidence for the explicit no-motion check, not a GUI motion gate."""
        if not self.localization.get("pose"):
            raise RuntimeError("传感器检查未取得T265 Pose；查看本会话localizer.log，不能算启动成功")
        if (getattr(self.options,"wheel_input","off")=="on" and
                (self.wheel_sdk_status or {}).get("sdk_api_success_samples",0)<=0):
            raise RuntimeError("传感器检查没有真实SDK轮速接受结果；不能以配置或IPC成功代替")

    def run(self) -> int:
        try:
            if self.options.logs_only:
                self.storage_panel.open = self.storage_panel.scan_requested = True
            else:
                self.start_session()
            if getattr(self.options,"sensor_check_seconds",0):
                synthetic=getattr(self.options,"wheel_fixture","off")!="off"
                began=time.monotonic();deadline=began+self.options.sensor_check_seconds
                render_mode=getattr(self.options,'sensor_check_render_mode','on');previous_phase=None
                frames=0
                while time.monotonic()<deadline:
                    self.update_data()
                    # Exercise the existing map renderer without opening a
                    # clickable window or accepting any automatic actions.
                    if synthetic:
                        if self.localization.get("pose") and (self.wheel_sdk_status or {}).get("config_loaded"):
                            self.wheel_input.activate_fixture()
                        if self.wheel_input.done.is_set():break
                    else:
                        phase=min(3,int(4*(time.monotonic()-began)/self.options.sensor_check_seconds)) if render_mode=='compare' else 0
                        render=(phase in (1,2)) if render_mode=='compare' else render_mode=='on'
                        if phase!=previous_phase:
                            self.record_event('sensor_render_phase',dict(phase_index=phase,render_enabled=render,
                                comparison='off_on_on_off' if render_mode=='compare' else render_mode,
                                same_sdk_session=True,motion_commands_sent=0))
                            previous_phase=phase
                        if render:self.render_screen();frames+=1
                    if self.localizer.poll() is not None:raise RuntimeError("SDK owner exited during sensor lifecycle check")
                    time.sleep(.02)
                if synthetic and not self.wheel_input.done.is_set():raise RuntimeError("synthetic sequence did not complete before check deadline")
                self.validate_sensor_check()
                self.record_event("synthetic_check_complete" if synthetic else "sensor_render_check_complete",dict(rendered_frames=frames,
                    duration_s=self.options.sensor_check_seconds,motion_commands_sent=0,
                    chassis_link=self.bridge.connection_snapshot() if self.bridge else None,
                    measurement_stream=self.bridge.measurement_snapshot() if self.bridge else None))
                return 0
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
            cv2.resizeWindow(WINDOW_NAME, self.screen_width, self.screen_height)
            cv2.imshow(WINDOW_NAME, self.render_screen())
            cv2.waitKey(1)
            cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)
            if self.options.fullscreen:
                cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            while True:
                if not self.options.logs_only:
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
    parser = argparse.ArgumentParser(description="F405 NormalDebug / T265实验地图（保留原桌面入口）")
    parser.add_argument("--uart", default="/dev/ttyS1")
    parser.add_argument("--baud", type=int, choices=(115200,230400,921600), default=921600)
    parser.add_argument("--chassis-profile", choices=("auto","legacy","timed"), default="auto")
    parser.add_argument("--check-chassis", action="store_true",help="read-only UART/identity/status check; no T265, GUI or motion")
    parser.add_argument("--timed-measurements",action="store_true",help="explicitly sync and subscribe measured 07/08 for diagnostics; not injection eligibility")
    parser.add_argument("--no-log-prune",action="store_true",help="compatibility flag; automatic pruning is disabled by default")
    parser.add_argument("--t265-usb-recovery",action="store_true",
                        help="fast startup firmware preparation; skip reset in runtime state; no fixed hub-cycle delay")
    parser.add_argument("--fisheye-evidence",action="store_true",help="same SDK owner enables both fisheyes; C records max two 2s/4Hz bursts")
    parser.add_argument("--device-restart-note",default="not_confirmed",help="operator record of an ACTUAL device restart; never automatically inferred from pipeline start")
    parser.add_argument("--sensor-check-seconds",type=float,default=0,help="existing entry's bounded headless sensor/map-render check (0..60s); no motion commands")
    parser.add_argument('--sensor-check-render-mode',choices=('on','off','compare'),default='on',
                        help='existing headless check only: same-SDK no-render/render comparison; never sends motion')
    parser.add_argument("--serial")
    parser.add_argument("--wheel-input", choices=("off", "on"), default="off",
                        help="SDK-session choice; default pure T265, on uses measured 07/08 with timing diagnostics only")
    parser.add_argument("--wheel-fixture",choices=("off","axes","pause-resume"),default="off",
                        help="explicit synthetic SDK-response sequence; headless only, NO chassis UART, not measured feedback")
    parser.add_argument("--wheel-noise-std-mps", type=float, default=.10,
                        help="initial engineering velocity sigma, not calibrated; SDK uses its square")
    parser.add_argument("--reference-description", default="",
                        help="本轮卷尺零点、车中心标记及地面角度基准；G键录入实测前必填")
    parser.add_argument("--actual-lower-firmware", default="unconfirmed")
    parser.add_argument("--actual-lower-service", default="unconfirmed",
                        choices=("unconfirmed", "MOTIONDEBUG", "RESCUE_TASK"))
    parser.add_argument("--actual-lower-mode", default="NormalDebug",
                        choices=("unconfirmed", "NormalRun", "Gamepad", "MotionDebug", "NormalDebug"))
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG,
                        help="本轮测试配置；复制到独立日志目录，保留安装轴和偏置")
    parser.add_argument("--zone", type=int, choices=range(1, 5), default=4)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_RUN_ROOT)
    parser.add_argument("--logs-only", action="store_true", help="只管理已有日志，不启动USB/SDK/UART，不生成测试会话")
    parser.add_argument("--log-module",choices=("test","competition"),default="test",
                        help="日志管理选择测试或正式比赛定位会话；competition仅用于--logs-only")
    parser.add_argument("--fullscreen", dest="fullscreen", action="store_true", default=True)
    parser.add_argument("--windowed", dest="fullscreen", action="store_false")
    options=parser.parse_args()
    if options.sensor_check_render_mode!='on' and not options.sensor_check_seconds:
        parser.error('--sensor-check-render-mode requires --sensor-check-seconds')
    if options.log_module=="competition":
        if not options.logs_only:parser.error("competition日志管理必须使用--logs-only，不启动传感器")
        if options.output_root==DEFAULT_RUN_ROOT:
            options.output_root=DEFAULT_RUN_ROOT.parent/"competition_wheel_input"
    if options.logs_only and (options.check_chassis or options.sensor_check_seconds or options.wheel_fixture != "off"):
        parser.error("--logs-only不能与传感器/底盘/人工注入检查组合")
    if options.check_chassis and options.t265_usb_recovery:
        parser.error("USB恢复属于T265启动路径，不能与仅底盘--check-chassis组合")
    if options.wheel_fixture!="off":
        if options.wheel_input!="on" or options.check_chassis or options.timed_measurements:
            parser.error("synthetic fixture requires --wheel-input on and forbids chassis check/subscription")
        if not options.sensor_check_seconds:options.sensor_check_seconds=30
    if not 0<=options.sensor_check_seconds<=60:parser.error("sensor check must be 0..60 seconds")
    return options


def main() -> int:
    try:
        options=arguments()
        if options.check_chassis:
            from f405_connection_check import check_connection
            return check_connection(options,DEFAULT_RUN_ROOT.parent/"f405_connection_check")
        return MotionMapApp(options).run()
    except (OSError, RuntimeError, ValueError, serial.SerialException) as error:
        print(f"错误：{error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
