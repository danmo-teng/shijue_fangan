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


@dataclass(frozen=True)
class MissionOutput:
    state: MissionState
    report: NormalSupplyReport | None
    command: MissionCommand | None
    message: str
    contact_pose: tuple[float, float, float] | None = None


@dataclass
class MissionSettings:
    side: str
    confirmation_frames: int = 3
    grab_timeout_s: float = 3.0
    material_target_x_abs_m: float = 0.15
    approach_y_abs_m: float = 1.20
    safe_center_y_abs_m: float = 1.32
    safe_zone_half_width_m: float = 0.30
    safe_zone_inner_edge_abs_m: float = 1.20
    safe_zone_outer_edge_abs_m: float = 1.50
    robot_radius_m: float = 0.12
    delivery_stationary_s: float = 0.8
    delivery_stationary_tolerance_m: float = 0.015
    align_tolerance_deg: float = 8.0

    def __post_init__(self) -> None:
        if self.side not in {"red", "blue"}:
            raise ValueError("side must be red or blue")
        if self.confirmation_frames <= 0 or self.grab_timeout_s <= 0:
            raise ValueError("confirmation_frames and grab_timeout_s must be positive")
        if not (0.0 < self.safe_zone_inner_edge_abs_m < self.safe_zone_outer_edge_abs_m):
            raise ValueError("safe-zone edges must be positive and ordered")
        if self.safe_zone_half_width_m <= 0 or self.robot_radius_m <= 0:
            raise ValueError("safe-zone width and robot radius must be positive")
        if not 0 < self.material_target_x_abs_m <= self.safe_zone_half_width_m - self.robot_radius_m:
            raise ValueError("material target must keep the robot circle inside its half-zone")
        if self.delivery_stationary_s <= 0 or self.delivery_stationary_tolerance_m <= 0:
            raise ValueError("material target and stationary thresholds must be positive")


def angle_error_deg(target: float, current: float) -> float:
    return (target - current + 180.0) % 360.0 - 180.0


def robot_intersects_safe_zone(pose: PoseInput, settings: MissionSettings) -> bool:
    """Match the map: a 120mm robot circle must touch the own safe rectangle."""
    if not pose.valid:
        return False
    x_min = -settings.safe_zone_half_width_m
    x_max = settings.safe_zone_half_width_m
    if settings.side == "red":
        y_min = settings.safe_zone_inner_edge_abs_m
        y_max = settings.safe_zone_outer_edge_abs_m
    else:
        y_min = -settings.safe_zone_outer_edge_abs_m
        y_max = -settings.safe_zone_inner_edge_abs_m
    nearest_x = min(max(pose.x_m, x_min), x_max)
    nearest_y = min(max(pose.y_m, y_min), y_max)
    dx = pose.x_m - nearest_x
    dy = pose.y_m - nearest_y
    return dx * dx + dy * dy <= settings.robot_radius_m * settings.robot_radius_m + 1e-12


class RescueMission:
    def __init__(self, settings: MissionSettings,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.settings = settings
        self.clock = clock
        self.state = MissionState.SEARCH
        self.grab_hits = 0
        self.grab_started_s: float | None = None
        self.delivery_stationary_started_s: float | None = None
        self.delivery_stationary_anchor: tuple[float, float] | None = None

    @property
    def desired_heading_deg(self) -> float:
        return 90.0 if self.settings.side == "red" else 270.0

    @property
    def approach_point(self) -> tuple[float, float]:
        # The map labels the red-side material compartment on field-left and
        # the blue-side material compartment on field-right. Aim at the centre
        # of that 300mm-wide half instead of the x=0 divider.
        target_x = (-self.settings.material_target_x_abs_m
                    if self.settings.side == "red"
                    else self.settings.material_target_x_abs_m)
        sign = 1.0 if self.settings.side == "red" else -1.0
        return target_x, sign * self.settings.approach_y_abs_m

    @property
    def safe_center(self) -> tuple[float, float]:
        target_x = (-self.settings.material_target_x_abs_m
                    if self.settings.side == "red"
                    else self.settings.material_target_x_abs_m)
        sign = 1.0 if self.settings.side == "red" else -1.0
        return target_x, sign * self.settings.safe_center_y_abs_m

    def contact_pose(self, observed: PoseInput) -> tuple[float, float, float]:
        """Apply only the boundary-normal constraint justified by fence contact."""
        sign = 1.0 if self.settings.side == "red" else -1.0
        tangent_y = sign * (
            self.settings.safe_zone_inner_edge_abs_m - self.settings.robot_radius_m
        )
        return observed.x_m, tangent_y, observed.yaw_deg

    def _stationary_at_safe_zone(self, pose: PoseInput) -> tuple[bool, float]:
        if not robot_intersects_safe_zone(pose, self.settings):
            self.delivery_stationary_started_s = None
            self.delivery_stationary_anchor = None
            return False, 0.0
        now = self.clock()
        if self.delivery_stationary_anchor is None:
            self.delivery_stationary_anchor = (pose.x_m, pose.y_m)
            self.delivery_stationary_started_s = now
            return False, 0.0
        moved = math.hypot(
            pose.x_m - self.delivery_stationary_anchor[0],
            pose.y_m - self.delivery_stationary_anchor[1],
        )
        if moved > self.settings.delivery_stationary_tolerance_m:
            self.delivery_stationary_anchor = (pose.x_m, pose.y_m)
            self.delivery_stationary_started_s = now
            return False, 0.0
        started = (
            self.delivery_stationary_started_s
            if self.delivery_stationary_started_s is not None
            else now
        )
        elapsed = max(0.0, now - started)
        return elapsed >= self.settings.delivery_stationary_s, elapsed

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
        if robot_intersects_safe_zone(pose, self.settings):
            self.state = MissionState.ALIGN
            command = self._waypoint_command(
                CMD_ALIGN_SAFE_ZONE, target, heading=True
            )
            return MissionOutput(
                self.state, None, command,
                f"地图车体圆已接触安全区，开始对正{self.desired_heading_deg:.1f}°",
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
                self.delivery_stationary_started_s = None
                self.delivery_stationary_anchor = None
                command = self._waypoint_command(
                    CMD_ENTER_SAFE_ZONE, self.safe_center, straight=True, heading=True
                )
            return MissionOutput(self.state, None, command, f"对正安全区入口，角度误差{yaw_error:.1f}°")

        if self.state == MissionState.ENTER_SAFE_ZONE:
            stationary, stationary_s = self._stationary_at_safe_zone(pose)
            if stationary:
                self.state = MissionState.COMPLETE
                return MissionOutput(
                    self.state,
                    None,
                    MissionCommand(CMD_TASK_COMPLETE, self._flags()),
                    "地图车体圆接触安全区且定位稳定不动，任务完成",
                    self.contact_pose(pose),
                )
            return MissionOutput(
                self.state,
                report,
                self._waypoint_command(CMD_ENTER_SAFE_ZONE, self.safe_center, straight=True, heading=True),
                f"等待车体圆接触安全区并稳定{self.settings.delivery_stationary_s:.1f}s（当前{stationary_s:.2f}s）",
            )

        if self.state == MissionState.COMPLETE:
            return MissionOutput(
                self.state, None, MissionCommand(CMD_TASK_COMPLETE, self._flags()), "任务完成，等待STM32停车"
            )

        return MissionOutput(self.state, None, MissionCommand(0), "故障停车")
