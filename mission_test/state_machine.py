#!/usr/bin/env python3
"""Pure rescue mission state machine; hardware and vision are adapters."""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

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
    grab_y_min: int = 760
    grab_x_min: int = 384
    grab_x_max: int = 896
    confirmation_frames: int = 3
    approach_x_m: float = 0.0
    approach_y_abs_m: float = 0.95
    safe_center_y_abs_m: float = 1.32
    approach_radius_m: float = 0.25
    align_tolerance_deg: float = 8.0

    def __post_init__(self) -> None:
        if self.side not in {"red", "blue"}:
            raise ValueError("side must be red or blue")


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
    def __init__(self, settings: MissionSettings) -> None:
        self.settings = settings
        self.state = MissionState.SEARCH
        self.grab_hits = 0
        self.safe_hits = 0

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

    def _waypoint_command(self, command: int, target: tuple[float, float], *, straight=False, heading=False) -> MissionCommand:
        return MissionCommand(
            command=command,
            flags=self._flags(straight=straight, heading=heading),
            target_x_mm=round(target[0] * 1000),
            target_y_mm=round(target[1] * 1000),
            heading_cdeg=round(self.desired_heading_deg * 100),
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
            in_grab_window = (
                vision.target_found
                and self.settings.grab_x_min <= vision.target_x <= self.settings.grab_x_max
                and vision.target_y >= self.settings.grab_y_min
            )
            self.grab_hits = self.grab_hits + 1 if in_grab_window else 0
            if self.grab_hits >= self.settings.confirmation_frames:
                self.state = MissionState.NAVIGATE
                return MissionOutput(
                    self.state,
                    None,
                    MissionCommand(CMD_GRAB_CONFIRMED, self._flags()),
                    "物资连续位于画面底部抓取窗，确认抓取",
                )
            return MissionOutput(self.state, report, None, "等待物资进入底部抓取确认区域")

        if self.state == MissionState.NAVIGATE:
            target = self.approach_point
            distance = math.hypot(pose.x_m - target[0], pose.y_m - target[1]) if pose.valid else math.inf
            if pose.valid and distance <= self.settings.approach_radius_m:
                self.state = MissionState.ALIGN
            command = self._waypoint_command(CMD_NAVIGATE_WAYPOINT, target, straight=True)
            return MissionOutput(self.state, None, command, f"直线驶向安全区前置点，剩余{distance:.2f}m")

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
