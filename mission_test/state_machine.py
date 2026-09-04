#!/usr/bin/env python3
"""Pure rescue mission state machine; hardware and vision are adapters."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable

from rescue_vision.mission_protocol import (
    CMD_ALIGN_SAFE_ZONE,
    CMD_DRIVE_STRAIGHT,
    CMD_ENTER_SAFE_ZONE,
    CMD_GRAB_CONFIRMED,
    CMD_NAVIGATE_WAYPOINT,
    CMD_RED_SIDE,
    CMD_TASK_COMPLETE,
    CMD_USE_FINAL_HEADING,
    CMD_VALID,
    MissionCommand,
    Stm32Status,
)
from rescue_vision.vision_protocol import NormalSupplyReport


class MissionState(str, Enum):
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    GRAB_CHECK = "GRAB_CHECK"
    GRABBING = "GRABBING"
    NAVIGATE = "NAVIGATE"
    ALIGN = "ALIGN"
    ENTER_SAFE_ZONE = "ENTER_SAFE_ZONE"
    COMPLETE = "COMPLETE"
    FAULT = "FAULT"


@dataclass(frozen=True)
class PoseInput:
    valid: bool = False
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_deg: float = 0.0


@dataclass(frozen=True)
class VisionInput:
    target_found: bool = False
    target_x: int = 0
    target_y: int = 0
    target_bbox: tuple[int, int, int, int] | None = None
    safe_found: bool = False
    safe_bbox: tuple[int, int, int, int] | None = None


@dataclass(frozen=True)
class MissionOutput:
    state: MissionState
    report: NormalSupplyReport | None
    command: MissionCommand | None
    message: str


@dataclass
class MissionSettings:
    side: str
    confirmation_frames: int = 3
    grab_timeout_s: float = 2.0
    approach_x_m: float = 0.0
    approach_y_abs_m: float = 0.95
    safe_center_y_abs_m: float = 1.32
    approach_radius_m: float = 0.25
    align_tolerance_deg: float = 8.0

    def __post_init__(self) -> None:
        if self.side not in {"red", "blue"}:
            raise ValueError("side must be red or blue")
        if self.confirmation_frames <= 0 or self.grab_timeout_s <= 0:
            raise ValueError("confirmation_frames and grab_timeout_s must be positive")


def angle_error_deg(target: float, current: float) -> float:
    return (target - current + 180.0) % 360.0 - 180.0


def target_inside_safe_zone(vision: VisionInput) -> bool:
    if not vision.target_found or not vision.safe_found:
        return False
    if vision.target_bbox is None or vision.safe_bbox is None:
        return False
    tx, ty, tw, th = vision.target_bbox
    sx, sy, sw, sh = vision.safe_bbox
    center_x, center_y = tx + tw * 0.5, ty + th * 0.5
    margin_x = max(4.0, sw * 0.03)
    margin_y = max(4.0, sh * 0.03)
    return (
        sx + margin_x <= center_x <= sx + sw - margin_x
        and sy + margin_y <= center_y <= sy + sh - margin_y
    )


class RescueMission:
    def __init__(self, settings: MissionSettings,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.settings = settings
        self.clock = clock
        self.state = MissionState.SEARCH
        self.grab_hits = 0
        self.safe_hits = 0
        self.grab_started_s: float | None = None

    @property
    def desired_heading_deg(self) -> float:
        return 90.0 if self.settings.side == "red" else 270.0

    @property
    def approach_point(self) -> tuple[float, float]:
        sign = 1.0 if self.settings.side == "red" else -1.0
        return self.settings.approach_x_m, sign * self.settings.approach_y_abs_m

    @property
    def safe_center(self) -> tuple[float, float]:
        sign = 1.0 if self.settings.side == "red" else -1.0
        return 0.0, sign * self.settings.safe_center_y_abs_m

    def _flags(self, *, straight=False, heading=False) -> int:
        result = CMD_VALID | (CMD_RED_SIDE if self.settings.side == "red" else 0)
        if straight:
            result |= CMD_DRIVE_STRAIGHT
        if heading:
            result |= CMD_USE_FINAL_HEADING
        return result

    def _waypoint_command(self, command: int, target: tuple[float, float], *,
                          straight=False, heading=False,
                          heading_deg: float | None = None) -> MissionCommand:
        selected_heading = self.desired_heading_deg if heading_deg is None else heading_deg % 360.0
        return MissionCommand(
            command=command,
            flags=self._flags(straight=straight, heading=heading),
            target_x_mm=round(target[0] * 1000),
            target_y_mm=round(target[1] * 1000),
            heading_cdeg=round(selected_heading * 100) % 36000,
        )

    def _navigate(self, pose: PoseInput) -> MissionOutput:
        target = self.approach_point
        if not pose.valid:
            return MissionOutput(
                self.state, None, MissionCommand(0),
                "融合位姿无效，停止并等待导航航向",
            )
        delta_x = target[0] - pose.x_m
        delta_y = target[1] - pose.y_m
        distance = math.hypot(delta_x, delta_y)
        travel_heading_deg = math.degrees(math.atan2(delta_y, delta_x)) % 360.0
        if distance <= self.settings.approach_radius_m:
            self.state = MissionState.ALIGN
            command = self._waypoint_command(
                CMD_ALIGN_SAFE_ZONE, target, heading=True
            )
            return MissionOutput(
                self.state, None, command,
                f"已到安全区前置点，开始对正{self.desired_heading_deg:.1f}°",
            )
        command = self._waypoint_command(
            CMD_NAVIGATE_WAYPOINT,
            target,
            straight=True,
            heading=True,
            heading_deg=travel_heading_deg,
        )
        return MissionOutput(
            self.state, None, command,
            f"直线驶向安全区前置点，航向{travel_heading_deg:.1f}°，剩余{distance:.2f}m",
        )

    def step(self, vision: VisionInput, pose: PoseInput, stm: Stm32Status) -> MissionOutput:
        if stm.fault:
            self.state = MissionState.FAULT
            return MissionOutput(self.state, None, MissionCommand(0), "STM32报告故障，停止任务")

        report = NormalSupplyReport(
            x_px=vision.target_x if vision.target_found else 0,
            y_px=vision.target_y if vision.target_found else 0,
            found=vision.target_found,
        )

        if self.state == MissionState.SEARCH:
            if vision.target_found:
                self.state = MissionState.APPROACH
            return MissionOutput(self.state, report, None, "搜索普通物资并发送1280×1024坐标")

        if self.state == MissionState.APPROACH:
            if stm.claw_visible and stm.age_ms <= 250.0:
                self.state = MissionState.GRAB_CHECK
                self.grab_hits = 0
            return MissionOutput(self.state, report, None, "STM32保持目标居中并靠近")

        if self.state == MissionState.GRAB_CHECK:
            camera_ready = stm.claw_visible and stm.age_ms <= 250.0
            target_visible = camera_ready and vision.target_found
            self.grab_hits = self.grab_hits + 1 if target_visible else 0
            if self.grab_hits >= self.settings.confirmation_frames:
                self.state = MissionState.GRABBING
                self.grab_started_s = self.clock()
                return MissionOutput(
                    self.state,
                    None,
                    MissionCommand(CMD_GRAB_CONFIRMED, self._flags()),
                    "摄像头下压后连续检测到物资，确认抓取",
                )
            return MissionOutput(self.state, report, None, "等待摄像头下压后在画面中确认物资")

        if self.state == MissionState.GRABBING:
            status_fresh = stm.age_ms <= 250.0
            if status_fresh and stm.gripper_closed:
                self.state = MissionState.NAVIGATE
                return self._navigate(pose)
            started = self.grab_started_s if self.grab_started_s is not None else self.clock()
            elapsed = self.clock() - started
            if elapsed >= self.settings.grab_timeout_s:
                self.state = MissionState.FAULT
                return MissionOutput(
                    self.state, None, MissionCommand(0),
                    f"夹爪{self.settings.grab_timeout_s:.1f}秒内未确认闭合，停止任务",
                )
            return MissionOutput(
                self.state, None,
                MissionCommand(CMD_GRAB_CONFIRMED, self._flags()),
                f"重复发送抓取命令并等待夹爪闭合（{elapsed:.2f}s）",
            )

        if self.state == MissionState.NAVIGATE:
            return self._navigate(pose)

        if self.state == MissionState.ALIGN:
            target = self.approach_point
            yaw_error = abs(angle_error_deg(self.desired_heading_deg, pose.yaw_deg)) if pose.valid else math.inf
            command = self._waypoint_command(CMD_ALIGN_SAFE_ZONE, target, heading=True)
            if pose.valid and yaw_error <= self.settings.align_tolerance_deg:
                self.state = MissionState.ENTER_SAFE_ZONE
                command = self._waypoint_command(
                    CMD_ENTER_SAFE_ZONE, self.safe_center, straight=True, heading=True
                )
            return MissionOutput(self.state, None, command, f"对正安全区入口，角度误差{yaw_error:.1f}°")

        if self.state == MissionState.ENTER_SAFE_ZONE:
            self.safe_hits = self.safe_hits + 1 if target_inside_safe_zone(vision) else 0
            if self.safe_hits >= self.settings.confirmation_frames:
                self.state = MissionState.COMPLETE
                return MissionOutput(
                    self.state,
                    None,
                    MissionCommand(CMD_TASK_COMPLETE, self._flags()),
                    "普通物资已连续出现在本方安全区内，任务完成",
                )
            return MissionOutput(
                self.state,
                report,
                self._waypoint_command(CMD_ENTER_SAFE_ZONE, self.safe_center, straight=True, heading=True),
                "识别本方安全区并确认物资进入",
            )

        if self.state == MissionState.COMPLETE:
            return MissionOutput(
                self.state, None, MissionCommand(CMD_TASK_COMPLETE, self._flags()), "任务完成，等待STM32停车"
            )

        return MissionOutput(self.state, None, MissionCommand(0), "故障停车")
