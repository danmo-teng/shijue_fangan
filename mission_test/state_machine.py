#!/usr/bin/env python3
"""Pure rescue mission state machine; hardware and vision are adapters."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable

from rescue_vision.mission_protocol import (
    CMD_DISTANCE_VALID,
    CMD_DRIVE_STRAIGHT,
    CMD_ENTER_SAFE_ZONE,
    CMD_GRAB_CONFIRMED,
    CMD_NAVIGATE_WAYPOINT,
    CMD_RED_SIDE,
    CMD_RETURN_CENTER,
    CMD_TASK_COMPLETE,
    CMD_USE_FINAL_HEADING,
    CMD_VALID,
    MissionCommand,
    Stm32Status,
)
from rescue_vision.safe_zone import bbox_center_in_safe_zone
from rescue_vision.vision_protocol import NormalSupplyReport


class MissionState(str, Enum):
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    GRAB_CHECK = "GRAB_CHECK"
    GRABBING = "GRABBING"
    NAVIGATE = "NAVIGATE"
    ENTER_SAFE_ZONE = "ENTER_SAFE_ZONE"
    COMPLETE = "COMPLETE"
    RETURN_CENTER = "RETURN_CENTER"
    FAULT = "FAULT"


CARGO_CLASSES = ("green_supply", "core_black", "danger_cyan", "injured_orange")
MATERIAL_CLASSES = frozenset(("green_supply", "core_black", "danger_cyan"))
STM_MODE_SEARCH = 3
STM_MODE_APPROACH = 4
STM_MODE_NAVIGATE = 10
STM_MODE_RAM_VERIFY = 15
STM_MODE_EXIT_SAFE_ZONE = 16
STM_MODE_FACE_FIELD_CENTER = 17


@dataclass(frozen=True)
class PoseInput:
    valid: bool = False
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_deg: float = 0.0
    age_ms: float = float("inf")
    navigation_distance_m: float = 0.0
    navigation_distance_valid: bool = False
    navigation_wheel_primary: bool = False
    encoder_fusion_weight: float = 0.0


@dataclass(frozen=True)
class VisionInput:
    target_found: bool = False
    target_x: int = 0
    target_y: int = 0
    target_bbox: tuple[int, int, int, int] | None = None
    class_name: str = ""
    safe_found: bool = False
    safe_bbox: tuple[int, int, int, int] | None = None
    # These fields are independent of the search/report target selection. The
    # mission uses them to follow the currently grabbed cargo through an
    # outside -> inside-safe-zone transition.
    frame_sequence: int = 0
    delivery_target_found: bool = False
    delivery_target_inside_safe_zone: bool = False


@dataclass(frozen=True)
class DeliveryConfirmation:
    cargo_class: str
    outside_seen: bool
    outside_frame_sequence: int
    outside_target_bbox: tuple[int, int, int, int] | None
    outside_safe_bbox: tuple[int, int, int, int] | None
    inside_hits: int
    frame_sequence: int
    target_bbox: tuple[int, int, int, int] | None
    safe_bbox: tuple[int, int, int, int] | None


@dataclass(frozen=True)
class MissionOutput:
    state: MissionState
    report: NormalSupplyReport | None
    command: MissionCommand | None
    message: str
    contact_pose: tuple[float, float, float] | None = None
    delivery_confirmation: DeliveryConfirmation | None = None


@dataclass
class MissionSettings:
    side: str
    confirmation_frames: int = 3
    zone_center_x_abs_m: float = 0.15
    approach_y_abs_m: float = 1.20
    safe_center_y_abs_m: float = 1.32
    robot_body_radius_m: float = 0.130
    push_plate_offset_m: float = 0.105
    front_pusher_offset_m: float = 0.150
    safe_zone_outer_width_m: float = 0.660
    safe_zone_outer_depth_m: float = 0.360
    safe_zone_inner_width_m: float = 0.600
    safe_zone_inner_depth_m: float = 0.300
    safe_zone_half_width_m: float = 0.30
    safe_zone_inner_edge_abs_m: float = 1.20
    safe_zone_outer_edge_abs_m: float = 1.50
    safe_fence_field_face_abs_m: float = 1.140
    fence_stop_safety_margin_m: float = 0.0075
    t265_delivery_extra_m: float = 0.0
    nav_axial_tolerance_m: float = 0.030
    nav_lateral_tolerance_m: float = 0.050
    nav_fence_heading_tolerance_deg: float = 12.0
    nav_near_fence_distance_m: float = 0.30
    nav_near_fence_heading_limit_deg: float = 10.0
    delivery_stationary_s: float = 0.8
    delivery_stationary_tolerance_m: float = 0.025
    delivery_visual_confirmation_frames: int = 5
    center_stop_radius_m: float = 0.60

    def __post_init__(self) -> None:
        if self.side not in {"red", "blue"}:
            raise ValueError("side must be red or blue")
        if self.confirmation_frames <= 0:
            raise ValueError("confirmation_frames must be positive")
        if not (0.0 < self.safe_zone_inner_edge_abs_m < self.safe_zone_outer_edge_abs_m):
            raise ValueError("safe-zone edges must be positive and ordered")
        if self.safe_zone_half_width_m <= 0 or self.robot_body_radius_m <= 0:
            raise ValueError("safe-zone width and robot radius must be positive")
        if not 0 < self.zone_center_x_abs_m <= self.safe_zone_half_width_m:
            raise ValueError("zone center must keep the robot circle inside its half-zone")
        if (self.delivery_stationary_s <= 0 or
                self.delivery_stationary_tolerance_m <= 0 or
                self.delivery_visual_confirmation_frames <= 0):
            raise ValueError("material target and stationary thresholds must be positive")
        if self.center_stop_radius_m <= 0:
            raise ValueError("center stop radius must be positive")
        if not 0 < self.push_plate_offset_m < self.safe_fence_field_face_abs_m:
            raise ValueError("push plate offset must be a positive body-frame distance")
        if (self.front_pusher_offset_m <= 0 or self.t265_delivery_extra_m < 0 or
                self.nav_fence_heading_tolerance_deg <= 0 or
                self.nav_fence_heading_tolerance_deg > 90 or
                self.nav_near_fence_distance_m <= 0 or
                self.nav_near_fence_heading_limit_deg <= 0):
            raise ValueError("mechanism and safety parameters are invalid")


def angle_error_deg(target: float, current: float) -> float:
    return (target - current + 180.0) % 360.0 - 180.0


def target_inside_safe_zone(vision: VisionInput) -> bool:
    """Return whether the selected visual target is already in the safe zone."""
    if not vision.target_found or not vision.safe_found:
        return False
    return bbox_center_in_safe_zone(vision.target_bbox, vision.safe_bbox)


def robot_intersects_safe_zone(pose: PoseInput, settings: MissionSettings) -> bool:
    """Map-only geometry for the 130 mm body collision circle."""
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
    return dx * dx + dy * dy <= settings.robot_body_radius_m * settings.robot_body_radius_m + 1e-12


class RescueMission:
    def __init__(self, settings: MissionSettings,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.settings = settings
        self.clock = clock
        self.state = MissionState.SEARCH
        self.grab_hits = 0
        self.delivery_stationary_started_s: float | None = None
        self.delivery_stationary_anchor: tuple[float, float] | None = None
        self.delivery_outside_seen = False
        self.delivery_inside_hits = 0
        self.delivery_visual_confirmed = False
        self.delivery_last_frame_sequence: int | None = None
        self.delivery_last_outside_frame_sequence = 0
        self.delivery_last_outside_target_bbox: tuple[int, int, int, int] | None = None
        self.delivery_last_outside_safe_bbox: tuple[int, int, int, int] | None = None
        self.delivery_last_inside_frame_sequence = 0
        self.delivery_last_inside_target_bbox: tuple[int, int, int, int] | None = None
        self.delivery_last_inside_safe_bbox: tuple[int, int, int, int] | None = None
        self.selected_class: str | None = None
        self.delivered_common = False
        self.delivery_count = 0
        self.approach_acknowledged = False
        self.delivery_arrival_confirmed = False
        self.navigation_start_distance_m: float | None = None
        self.navigation_encoder_anchor_m: float | None = None
        self.navigation_encoder_last_progress_m: float | None = None

    @property
    def allowed_classes(self) -> tuple[str, ...]:
        if self.selected_class is not None:
            return (self.selected_class,)
        return CARGO_CLASSES if self.delivered_common else ("green_supply",)

    @property
    def desired_heading_deg(self) -> float:
        return 90.0 if self.settings.side == "red" else 270.0

    @property
    def approach_point(self) -> tuple[float, float]:
        cargo_class = self.selected_class or "green_supply"
        material_half = cargo_class in MATERIAL_CLASSES
        red_material_x = -self.settings.zone_center_x_abs_m
        target_x = red_material_x if material_half else -red_material_x
        if self.settings.side == "blue":
            target_x = -target_x
        sign = 1.0 if self.settings.side == "red" else -1.0
        return target_x, sign * self.settings.approach_y_abs_m

    @property
    def safe_center(self) -> tuple[float, float]:
        target_x = self.approach_point[0]
        sign = 1.0 if self.settings.side == "red" else -1.0
        return target_x, sign * self.settings.safe_center_y_abs_m

    @property
    def fence_contact_center(self) -> tuple[float, float]:
        sign = 1.0 if self.settings.side == "red" else -1.0
        return (
            self.approach_point[0],
            sign * (
                self.settings.safe_fence_field_face_abs_m
                - self.settings.push_plate_offset_m
            ),
        )

    @property
    def fence_stop_point(self) -> tuple[float, float]:
        x_m, contact_y_m = self.fence_contact_center
        sign = 1.0 if self.settings.side == "red" else -1.0
        return (
            x_m,
            contact_y_m
            - sign * self.settings.fence_stop_safety_margin_m
            + sign * self.settings.t265_delivery_extra_m,
        )

    def contact_pose(self, observed: PoseInput) -> tuple[float, float, float]:
        """Apply only the boundary-normal constraint justified by fence contact."""
        sign = 1.0 if self.settings.side == "red" else -1.0
        tangent_y = sign * (
            self.settings.safe_fence_field_face_abs_m
            - self.settings.push_plate_offset_m
        )
        return observed.x_m, tangent_y, observed.yaw_deg

    def _facing_fence(self, yaw_deg: float) -> bool:
        return abs(angle_error_deg(self.desired_heading_deg, yaw_deg)) <= (
            self.settings.nav_fence_heading_tolerance_deg
        )

    def _at_fence_stop(self, pose: PoseInput) -> bool:
        if not pose.valid or not self._facing_fence(pose.yaw_deg):
            return False
        target_x, target_y = self.fence_stop_point
        return (
            abs(pose.x_m - target_x) <= self.settings.nav_lateral_tolerance_m
            and abs(pose.y_m - target_y) <= self.settings.nav_axial_tolerance_m
        )

    def _stationary_pose(self, pose: PoseInput) -> tuple[bool, float]:
        if not pose.valid:
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

    def _reset_delivery_observation(self) -> None:
        self.delivery_outside_seen = False
        self.delivery_inside_hits = 0
        self.delivery_visual_confirmed = False
        self.delivery_last_frame_sequence = None
        self.delivery_last_outside_frame_sequence = 0
        self.delivery_last_outside_target_bbox = None
        self.delivery_last_outside_safe_bbox = None
        self.delivery_last_inside_frame_sequence = 0
        self.delivery_last_inside_target_bbox = None
        self.delivery_last_inside_safe_bbox = None

    def _is_new_delivery_frame(self, vision: VisionInput) -> bool:
        # A zero sequence is used by direct unit-test inputs that have no
        # camera packet identity; live camera observations always carry the
        # packet frame id.
        if vision.frame_sequence <= 0:
            return True
        if vision.frame_sequence == self.delivery_last_frame_sequence:
            return False
        self.delivery_last_frame_sequence = vision.frame_sequence
        return True

    def _update_delivery_observation(self, vision: VisionInput) -> None:
        if self.selected_class is None or self.state not in {
            MissionState.GRABBING,
            MissionState.NAVIGATE,
            MissionState.ENTER_SAFE_ZONE,
        }:
            return
        if not self._is_new_delivery_frame(vision) or self.delivery_visual_confirmed:
            return
        if not vision.delivery_target_found:
            self.delivery_inside_hits = 0
            return
        if vision.delivery_target_inside_safe_zone:
            if not self.delivery_outside_seen:
                return
            self.delivery_inside_hits += 1
            self.delivery_last_inside_frame_sequence = vision.frame_sequence
            self.delivery_last_inside_target_bbox = vision.target_bbox
            self.delivery_last_inside_safe_bbox = vision.safe_bbox
            if self.delivery_inside_hits >= self.settings.delivery_visual_confirmation_frames:
                self.delivery_visual_confirmed = True
            return
        # Before the safe-zone stage, the camera remains on the grabbed cargo;
        # a visible cargo that is not inside is the required outside baseline.
        # After arrival, a missing safe-zone detection is unknown rather than
        # a new outside observation, so detector dropouts cannot manufacture a
        # second transition.
        if vision.safe_found or not self.delivery_arrival_confirmed:
            self.delivery_outside_seen = True
            self.delivery_last_outside_frame_sequence = vision.frame_sequence
            self.delivery_last_outside_target_bbox = vision.target_bbox
            self.delivery_last_outside_safe_bbox = vision.safe_bbox
        self.delivery_inside_hits = 0

    def _delivery_confirmation(self) -> DeliveryConfirmation:
        return DeliveryConfirmation(
            cargo_class=self.selected_class or "",
            outside_seen=self.delivery_outside_seen,
            outside_frame_sequence=self.delivery_last_outside_frame_sequence,
            outside_target_bbox=self.delivery_last_outside_target_bbox,
            outside_safe_bbox=self.delivery_last_outside_safe_bbox,
            inside_hits=self.delivery_inside_hits,
            frame_sequence=self.delivery_last_inside_frame_sequence,
            target_bbox=self.delivery_last_inside_target_bbox,
            safe_bbox=self.delivery_last_inside_safe_bbox,
        )

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

    def _distance_command(self, command: int, heading_deg: float,
                          distance_m: float) -> MissionCommand:
        return MissionCommand(
            command=command,
            flags=self._flags(straight=True, heading=True) | CMD_DISTANCE_VALID,
            target_x_mm=max(0, round(distance_m * 1000.0)),
            target_y_mm=0,
            heading_cdeg=round(heading_deg % 360.0 * 100.0) % 36000,
        )

    @staticmethod
    def _route_to(pose: PoseInput, target: tuple[float, float]) -> tuple[float, float]:
        dx = target[0] - pose.x_m
        dy = target[1] - pose.y_m
        return math.degrees(math.atan2(dy, dx)) % 360.0, math.hypot(dx, dy)

    def _fence_navigation_heading(self, route_heading_deg: float,
                                   distance_m: float) -> float:
        """Avoid an unstable bearing when only a few centimetres remain.

        The single heading field is used by F407 for both straight travel and
        final heading. Near the fence, the exact target bearing becomes very
        sensitive to a few millimetres of pose noise, so keep it close to the
        known fence-facing heading until the arrival gate is satisfied.
        """
        if distance_m > self.settings.nav_near_fence_distance_m:
            return route_heading_deg
        error = angle_error_deg(route_heading_deg, self.desired_heading_deg)
        limit = min(
            self.settings.nav_near_fence_heading_limit_deg,
            self.settings.nav_fence_heading_tolerance_deg,
        )
        limited_error = max(-limit, min(limit, error))
        return (self.desired_heading_deg + limited_error) % 360.0

    def _navigate(self, pose: PoseInput, stm: Stm32Status) -> MissionOutput:
        fence_stop_reached = self._at_fence_stop(pose)
        stm_distance_done = (
            pose.valid
            and stm.age_ms <= 250.0
            and stm.mode == STM_MODE_NAVIGATE
            and stm.gripper_closed
            and stm.distance_done
            and abs(pose.x_m - self.fence_stop_point[0]) <= 0.080
            and abs(pose.y_m - self.fence_stop_point[1]) <=
                self.settings.nav_axial_tolerance_m
            and self._facing_fence(pose.yaw_deg)
        )
        if fence_stop_reached or stm_distance_done or self.delivery_visual_confirmed:
            self.delivery_arrival_confirmed = True
            self.state = MissionState.ENTER_SAFE_ZONE
            self.delivery_stationary_started_s = None
            self.delivery_stationary_anchor = None
            command = self._waypoint_command(
                CMD_ENTER_SAFE_ZONE, self.safe_center, straight=True, heading=True
            )
            if self.delivery_visual_confirmed:
                arrival_reason = "视觉确认物资已从区外进入安全区"
            elif fence_stop_reached:
                arrival_reason = "高围栏前停车点"
            else:
                arrival_reason = "STM32定距完成"
            return MissionOutput(
                self.state, None, command,
                arrival_reason + "，锁存到达并进入安全区视觉确认",
            )
        if not pose.valid:
            return MissionOutput(
                self.state, None, None,
                "融合位姿暂时无效，暂停更新返航航向和剩余距离",
            )
        route_heading_deg, t265_distance = self._route_to(
            pose, self.fence_stop_point
        )
        if self.navigation_start_distance_m is None:
            self.navigation_start_distance_m = t265_distance
        if (pose.navigation_distance_valid and
                self.navigation_encoder_last_progress_m is not None and
                pose.navigation_distance_m + 0.05 <
                self.navigation_encoder_last_progress_m):
            # The localization relay restarted the NAV segment. Re-anchor the
            # scalar distance to the current T265 position instead of replaying
            # the old segment length.
            self.navigation_start_distance_m = t265_distance
            self.navigation_encoder_anchor_m = pose.navigation_distance_m
        if (self.navigation_encoder_anchor_m is None and
                pose.navigation_distance_valid):
            self.navigation_encoder_anchor_m = pose.navigation_distance_m
        if pose.navigation_distance_valid:
            self.navigation_encoder_last_progress_m = pose.navigation_distance_m
        distance = t265_distance
        if (pose.navigation_distance_valid and
                self.navigation_encoder_anchor_m is not None):
            encoder_progress = max(
                0.0,
                pose.navigation_distance_m - self.navigation_encoder_anchor_m,
            )
            distance = max(0.0, self.navigation_start_distance_m - encoder_progress)
        travel_heading_deg = self._fence_navigation_heading(
            route_heading_deg, distance
        )
        command = self._distance_command(
            CMD_NAVIGATE_WAYPOINT, travel_heading_deg, distance
        )
        return MissionOutput(
            self.state, None, command,
            f"持续修正返安全区航向{travel_heading_deg:.1f}°，剩余{distance:.2f}m",
        )

    def step(self, vision: VisionInput, pose: PoseInput, stm: Stm32Status) -> MissionOutput:
        if stm.fault:
            self.state = MissionState.FAULT
            return MissionOutput(self.state, None, MissionCommand(0), "STM32报告故障，停止任务")

        target_in_safe_zone = target_inside_safe_zone(vision)
        target_allowed = (
            vision.target_found
            and vision.class_name in self.allowed_classes
            and (self.selected_class is None or vision.class_name == self.selected_class)
            and not target_in_safe_zone
        )
        report = NormalSupplyReport(
            x_px=vision.target_x if target_allowed else 0,
            y_px=vision.target_y if target_allowed else 0,
            found=target_allowed,
            cargo_class=vision.class_name if target_allowed else "green_supply",
        )
        self._update_delivery_observation(vision)

        # If a target was selected before the zone detector became visible,
        # abandon that approach immediately and overwrite the old UART report
        # with an empty report. Never let an already delivered target continue
        # into APPROACH or GRAB_CHECK just because its previous frame was valid.
        if target_in_safe_zone and self.state in {
            MissionState.SEARCH, MissionState.APPROACH, MissionState.GRAB_CHECK,
        }:
            self.state = MissionState.SEARCH
            self.selected_class = None
            self.grab_hits = 0
            self.approach_acknowledged = False
            return MissionOutput(
                self.state,
                report,
                None,
                "检测到物资位于本方安全区，忽略目标并停止发送抓取坐标",
            )

        status_fresh = stm.age_ms <= 250.0
        if (self.state in {MissionState.APPROACH, MissionState.GRAB_CHECK}
                and status_fresh):
            if STM_MODE_APPROACH <= stm.mode <= 9:
                self.approach_acknowledged = True
            elif stm.mode == STM_MODE_SEARCH and self.approach_acknowledged:
                self.state = MissionState.SEARCH
                self.selected_class = None
                self.grab_hits = 0
                self.approach_acknowledged = False
                return MissionOutput(
                    self.state, NormalSupplyReport(), None,
                    "STM32已放弃旧目标，重新选择搜索目标",
                )

        if self.state == MissionState.SEARCH:
            if target_allowed:
                self.selected_class = vision.class_name
                self.approach_acknowledged = False
                self.state = MissionState.APPROACH
            search_name = "普通物资" if not self.delivered_common else "下一件物资或伤员"
            return MissionOutput(self.state, report, None, f"搜索{search_name}并发送1280×1024坐标")

        if self.state == MissionState.APPROACH:
            if stm.claw_visible and stm.age_ms <= 250.0:
                self.state = MissionState.GRAB_CHECK
                self.grab_hits = 0
                self.approach_acknowledged = True
            return MissionOutput(self.state, report, None, "STM32保持目标居中并靠近")

        if self.state == MissionState.GRAB_CHECK:
            camera_ready = stm.claw_visible and stm.age_ms <= 250.0
            target_visible = camera_ready and target_allowed
            self.grab_hits = self.grab_hits + 1 if target_visible else 0
            if self.grab_hits >= self.settings.confirmation_frames:
                self._reset_delivery_observation()
                self.state = MissionState.GRABBING
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
                self.delivery_arrival_confirmed = False
                self.navigation_start_distance_m = None
                self.navigation_encoder_anchor_m = None
                self.navigation_encoder_last_progress_m = None
                return self._navigate(pose, stm)
            return MissionOutput(
                self.state, None,
                MissionCommand(CMD_GRAB_CONFIRMED, self._flags()),
                "持续发送抓取命令并等待夹爪闭合",
            )

        if self.state == MissionState.NAVIGATE:
            return self._navigate(pose, stm)

        if self.state == MissionState.ENTER_SAFE_ZONE:
            delivery_verify_active = (
                stm.age_ms <= 250.0
                and stm.mode == STM_MODE_RAM_VERIFY
            )
            if (delivery_verify_active and self.delivery_arrival_confirmed and
                    self.delivery_visual_confirmed):
                self.state = MissionState.COMPLETE
                self.delivery_count += 1
                if self.selected_class == "green_supply":
                    self.delivered_common = True
                return MissionOutput(
                    self.state,
                    None,
                    MissionCommand(CMD_TASK_COMPLETE, self._flags()),
                    f"第{self.delivery_count}件经视觉确认已进入安全区，通知STM32张爪退出",
                    delivery_confirmation=self._delivery_confirmation(),
                )
            return MissionOutput(
                self.state,
                report,
                self._waypoint_command(CMD_ENTER_SAFE_ZONE, self.safe_center, straight=True, heading=True),
                "等待摄像头确认抓取物资已从区外进入安全区",
            )

        if self.state == MissionState.COMPLETE:
            status_fresh = stm.age_ms <= 250.0
            if status_fresh and stm.mode in {
                STM_MODE_EXIT_SAFE_ZONE,
                STM_MODE_FACE_FIELD_CENTER,
                STM_MODE_SEARCH,
            }:
                self.state = MissionState.RETURN_CENTER
                return MissionOutput(
                    self.state, None, None,
                    "STM32已张爪并退出安全区，返回中心区域",
                )
            return MissionOutput(
                self.state, None, MissionCommand(CMD_TASK_COMPLETE, self._flags()),
                "重复发送完成命令，等待STM32张爪并退出",
            )

        if self.state == MissionState.RETURN_CENTER:
            if stm.age_ms <= 250.0 and stm.mode == STM_MODE_SEARCH:
                self.state = MissionState.SEARCH
                self.selected_class = None
                self.grab_hits = 0
                self.approach_acknowledged = False
                self.delivery_stationary_started_s = None
                self.delivery_stationary_anchor = None
                self.delivery_arrival_confirmed = False
                self._reset_delivery_observation()
                self.navigation_start_distance_m = None
                self.navigation_encoder_anchor_m = None
                self.navigation_encoder_last_progress_m = None
                return MissionOutput(
                    self.state, NormalSupplyReport(), None,
                    f"已回到中心搜索流程，累计投送{self.delivery_count}件",
                )
            if stm.age_ms <= 250.0 and stm.mode == STM_MODE_FACE_FIELD_CENTER:
                if not pose.valid:
                    return MissionOutput(
                        self.state, None, None,
                        "融合位姿暂时无效，暂停更新返中航向和剩余距离",
                    )
                center_distance = math.hypot(pose.x_m, pose.y_m)
                distance = max(
                    0.0, center_distance - self.settings.center_stop_radius_m
                )
                heading = math.degrees(
                    math.atan2(-pose.y_m, -pose.x_m)
                ) % 360.0
                return MissionOutput(
                    self.state, None,
                    self._distance_command(CMD_RETURN_CENTER, heading, distance),
                    f"持续修正返中航向{heading:.1f}°，剩余{distance:.2f}m，"
                    f"距中心{self.settings.center_stop_radius_m:.2f}m停车",
                )
            return MissionOutput(
                self.state, None, None,
                "等待STM32完成张爪和退出安全区",
            )

        return MissionOutput(self.state, None, MissionCommand(0), "故障停车")
