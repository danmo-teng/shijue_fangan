#!/usr/bin/env python3
"""Complete rescue competition planner.

This module deliberately does not change ``mission_test/state_machine.py``.
It keeps the semantic decisions on the X5 and emits compact motion/audit
commands for the matching F407 firmware.
"""
from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass, replace
from enum import Enum
import math
from statistics import median
from typing import Iterable

from protocol import (
    AUDIT_DESTINATION_INJURY,
    CargoAuditPayload,
    CMD_ABORT,
    CMD_ALIGN_SAFE_ZONE,
    CMD_APPROACH_TARGET,
    CMD_CARGO_AUDIT,
    CMD_CLEAR_SAFE_ZONE,
    CMD_CLUSTER_TARGET,
    CMD_DISPERSE_PILE,
    CMD_DISTANCE_VALID,
    CMD_DRIVE_STRAIGHT,
    CMD_ESCAPE_MANEUVER,
    CMD_FIRST_GREEN_BUMP,
    CMD_GRAB_CONFIRMED,
    CMD_HOLD,
    CMD_NAVIGATE_WAYPOINT,
    CMD_PAUSE,
    CMD_RELEASE_BOTH,
    CMD_RELEASE_LEFT,
    CMD_RELEASE_RIGHT,
    CMD_RED_SIDE,
    CMD_RETURN_CENTER,
    CMD_SIDE_VALID,
    CMD_TASK_COMPLETE,
    CMD_TARGET_RIGHT,
    CMD_ENTER_SAFE_ZONE,
    CMD_YIELD_BACKOFF,
    CMD_STAGE_ONLY,
    CMD_USE_FINAL_HEADING,
    CMD_VALID,
    CMD_VISUAL_CORRECTION_VALID,
    mission_frame,
)


FIELD_HALF_M = 1.50
CAMERA_IMAGE_CENTER_X_PX = 640
CARGO_CLASSES = frozenset(
    {"green_supply", "core_black", "injured_orange", "danger_cyan", "unknown"}
)
MATERIAL_CLASSES = frozenset({"green_supply", "core_black"})

STM_MODE_SEARCH = 3
STM_MODE_NAVIGATE = 10
STM_MODE_ALIGN_SAFE_ZONE = 11
STM_MODE_RAM_VERIFY = 15
STM_MODE_EXIT_SAFE_ZONE = 16
STM_MODE_FACE_FIELD_CENTER = 17
STM_MODE_APPROACH_TARGET = 20
STM_MODE_CAPTURE_AUDIT = 21
STM_MODE_CAPTURE_DONE = 22
STM_MODE_POST_GRAB_AUDIT = 23
STM_MODE_APPROACH_RECOVER = 24
STM_MODE_REMOTE_ACTION = 25
STM_MODE_YIELD_DONE = 30
STM_MODE_ESCAPE_DONE = 31
STM_MODE_RELEASE_LEFT_DONE = 32
STM_MODE_RELEASE_RIGHT_DONE = 33
STM_MODE_RELEASE_BOTH_DONE = 34
STM_MODE_DISPERSE_DONE = 35
STM_MODE_LANE_DONE = 36
STM_MODE_CLUSTER_READY = 37
STM_MODE_CLUSTER_CAPTURE_AUDIT = 38
STM_MODE_SAFE_SWEEP = 39
STM_MODE_SAFE_SWEEP_DONE = 40
STM_MODE_BOUNDARY_RECOVER = 41


def angle_error_deg(target: float, current: float) -> float:
    return (target - current + 180.0) % 360.0 - 180.0


def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


@dataclass(frozen=True)
class PoseSnapshot:
    valid: bool = False
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_deg: float = 0.0
    age_ms: float = math.inf
    wheel_progress_m: float | None = None


@dataclass(frozen=True)
class StmSnapshot:
    mode: int = 0
    flags: int = 0
    age_ms: float = math.inf
    fault_code: int = 0
    acknowledged_sequence: int = 0
    relay_mission_tx_frames: int = 0
    relay_last_mission_command: int | None = None
    relay_last_mission_sequence: int | None = None
    relay_last_mission_payload: tuple[int, ...] = ()
    relay_last_mission_tx_monotonic_ns: int = 0
    relay_last_mission_tx_age_ms: float = math.inf
    relay_tx_frames: int = 0
    relay_tx_errors: int = 0
    relay_last_sequence: int | None = None
    relay_last_tx_age_ms: float = math.inf
    camera_pitch_cdeg: int = 0

    @property
    def claw_visible(self) -> bool:
        return bool(self.flags & (1 << 0))

    @property
    def gripper_closed(self) -> bool:
        return bool(self.flags & (1 << 1))

    @property
    def audit_valid(self) -> bool:
        return bool(self.flags & (1 << 4))

    @property
    def motors_active(self) -> bool:
        return bool(self.flags & (1 << 2))

    @property
    def distance_done(self) -> bool:
        return bool(self.flags & (1 << 5))

    @property
    def fresh(self) -> bool:
        return self.age_ms <= 250.0

    @property
    def fault(self) -> bool:
        return self.fault_code != 0 or bool(self.flags & (1 << 7))


@dataclass(frozen=True)
class TrackedCargo:
    track_id: int
    class_name: str
    confidence: float
    bbox: tuple[int, int, int, int]
    relative_xy_m: tuple[float, float] | None = None
    hits: int = 0
    misses: int = 0
    visible: bool = True
    inside_safe_zone: bool = False

    @property
    def center_px(self) -> tuple[int, int]:
        x, y, width, height = self.bbox
        return x + width // 2, y + height // 2

    @property
    def area_px(self) -> int:
        return max(0, self.bbox[2]) * max(0, self.bbox[3])

    @property
    def distance_m(self) -> float:
        if self.relative_xy_m is None:
            return math.inf
        return math.hypot(*self.relative_xy_m)


@dataclass(frozen=True)
class CargoAudit:
    left_class: str = ""
    right_class: str = ""
    left_count: int = 0
    right_count: int = 0
    left_green_count: int = 0
    right_green_count: int = 0
    total_count: int = 0
    danger_present: bool = False
    unknown_present: bool = False
    injury_mixed: bool = False
    stable: bool = False
    left_invalid: bool = False
    right_invalid: bool = False
    left_selected_count: int = 0
    right_selected_count: int = 0
    left_track_stability: int = 0
    right_track_stability: int = 0
    left_nearest_distance_m: float | None = None
    right_nearest_distance_m: float | None = None
    left_stable_track_id: int | None = None
    right_stable_track_id: int | None = None

    @property
    def signature(self) -> tuple:
        classes = Counter()
        for class_name, count, green_count in (
            (self.left_class, self.left_count, self.left_green_count),
            (self.right_class, self.right_count, self.right_green_count),
        ):
            if not class_name or count <= 0:
                continue
            if class_name == "mixed_material":
                bounded_green = max(0, min(count, green_count))
                classes["green_supply"] += bounded_green
                classes["core_black"] += count - bounded_green
            else:
                classes[class_name] += count
        return (
            self.total_count,
            tuple(sorted(classes.items())),
            self.danger_present,
            self.unknown_present,
            self.injury_mixed,
        )

    @property
    def release_side_hint(self) -> str | None:
        """Return the clearly invalid side, when the audit identifies one."""
        if self.left_invalid and not self.right_invalid:
            return "left"
        if self.right_invalid and not self.left_invalid:
            return "right"
        if self.left_invalid and self.right_invalid:
            return "both"
        return None

    @property
    def valid_side(self) -> str | None:
        """Backward-compatible alias; callers must treat it as discard-side."""
        return self.release_side_hint

    def to_protocol(self, *, initial_stash: bool, destination: str, audit_id: int) -> CargoAuditPayload:
        # The wire format reserves two bits per claw (0..3).  A vision frame
        # can still report a larger raw count; encode that side as UNKNOWN and
        # clamp only the wire count so F407 receives a safe, explicitly
        # invalid audit instead of the planner raising while serializing it.
        left_overflow = self.left_count > 3
        right_overflow = self.right_count > 3
        return CargoAuditPayload(
            left_class="unknown" if left_overflow else self.left_class,
            right_class="unknown" if right_overflow else self.right_class,
            left_count=max(0, min(3, self.left_count)),
            right_count=max(0, min(3, self.right_count)),
            total_count=self.total_count,
            danger_present=self.danger_present,
            unknown_present=self.unknown_present or left_overflow or right_overflow,
            injury_mixed=self.injury_mixed,
            stable=self.stable,
            initial_stash=initial_stash,
            destination_injury=destination == "injury",
            audit_id=audit_id,
        )


@dataclass(frozen=True)
class VisionSnapshot:
    frame_sequence: int = 0
    observed_monotonic_s: float | None = None
    cargo: tuple[TrackedCargo, ...] = ()
    capture_cargo: tuple[TrackedCargo, ...] = ()
    safe_bbox: tuple[int, int, int, int] | None = None
    danger_ahead: bool = False
    danger_side: str = "unknown"
    capture_audit: CargoAudit | None = None
    delivery_target_found: bool = False
    delivery_target_inside_safe_zone: bool = False
    delivery_target_outside_safe_zone: bool = False
    delivery_target_inside_track_ids: tuple[int, ...] = ()
    delivery_target_outside_track_ids: tuple[int, ...] = ()
    safe_zone_filter_blocked: bool = False
    low_conf_green_seen: bool = False
    safe_corridor_polygon: tuple[tuple[int, int], ...] = ()
    safe_corridor_obstacle_class: str | None = None
    safe_corridor_obstacle_distance_mm: int | None = None
    safe_corridor_obstacle_track_id: int | None = None
    carried_reference_bboxes: tuple[tuple[int, int, int, int], ...] = ()


@dataclass(frozen=True)
class CargoBatch:
    track_ids: tuple[int, ...]
    classes: tuple[str, ...]
    destination: str
    initial_stash: bool = False
    confirmed_total_count: int | None = None

    @property
    def total_count(self) -> int:
        return (
            self.confirmed_total_count
            if self.confirmed_total_count is not None else
            len(self.track_ids)
        )

    @property
    def counts(self) -> Counter:
        return Counter(self.classes)


@dataclass(frozen=True)
class CommandRequest:
    opcode: int
    flags: int = CMD_VALID
    arg_a: int = 0
    arg_b: int = 0
    aux: int = 0
    audit: CargoAuditPayload | None = None

    def to_frame(self, sequence: int) -> bytes:
        if self.audit is not None:
            return self.audit.frame(sequence)
        return mission_frame(
            sequence,
            self.opcode,
            self.flags,
            self.arg_a,
            self.arg_b,
            self.aux,
        )


@dataclass(frozen=True)
class CompetitionOutput:
    state: "CompetitionState"
    command: CommandRequest | None
    message: str
    batch: CargoBatch | None = None
    audit: CargoAudit | None = None
    event: str = ""
    motion_expected: bool = False
    stuck_phase: str = ""
    suppress_command_tx: bool = False
    suppression_reason: str = ""
    tx_policy: str = ""
    reason: str = ""
    expected_stm_modes: tuple[int, ...] = ()


class CompetitionState(str, Enum):
    WAIT_START = "WAIT_START"
    WAIT_SEARCH_RECOVERY = "WAIT_SEARCH_RECOVERY"
    INITIAL_OBSERVE = "INITIAL_OBSERVE"
    INITIAL_APPROACH = "INITIAL_APPROACH"
    CAPTURE_AUDIT = "CAPTURE_AUDIT"
    AUDIT_CONFIRM = "AUDIT_CONFIRM"
    GRAB = "GRAB"
    POST_GRAB_AUDIT = "POST_GRAB_AUDIT"
    INITIAL_STASH_NAV = "INITIAL_STASH_NAV"
    INITIAL_RELEASE = "INITIAL_RELEASE"
    SEARCH = "SEARCH"
    CLUSTER_APPROACH = "CLUSTER_APPROACH"
    DISPERSE = "DISPERSE"
    APPROACH = "APPROACH"
    NAVIGATE = "NAVIGATE"
    ALIGN_SAFE_ZONE_BY_POSE = "ALIGN_SAFE_ZONE_BY_POSE"
    ACQUIRE_SAFE_ZONE = "ACQUIRE_SAFE_ZONE"
    ALIGN_SAFE_ZONE_BY_LOCKED_BOX = "ALIGN_SAFE_ZONE_BY_LOCKED_BOX"
    SAFE_ZONE_CORRIDOR_CHECK = "SAFE_ZONE_CORRIDOR_CHECK"
    CLEAR_SAFE_ZONE = "CLEAR_SAFE_ZONE"
    WAIT_SAFE_ZONE_CLEAR = "WAIT_SAFE_ZONE_CLEAR"
    ENTER_SAFE_ZONE = "ENTER_SAFE_ZONE"
    DELIVERY_VERIFY = "DELIVERY_VERIFY"
    TASK_COMPLETE = "TASK_COMPLETE"
    RETURN_CENTER = "RETURN_CENTER"
    RETURN_STASH = "RETURN_STASH"
    WAIT_STASH_SEARCH_HANDOFF = "WAIT_STASH_SEARCH_HANDOFF"
    BOUNDARY_RECOVERY = "BOUNDARY_RECOVERY"
    DETOUR = "DETOUR"
    INVALID_RELEASE = "INVALID_RELEASE"
    INVALID_BACKOFF = "INVALID_BACKOFF"
    FIELD_STUCK_ESCAPE = "FIELD_STUCK_ESCAPE"
    SAFE_ZONE_ESCAPE = "SAFE_ZONE_ESCAPE"
    FINISHED = "FINISHED"
    FAULT = "FAULT"


@dataclass
class CompetitionSettings:
    side: str
    start_zone: int = 1
    initial_stash_enabled: bool = True
    audit_stable_frames: int = 3
    normal_grab_audit_frames: int = 3
    cluster_grab_audit_frames: int = 3
    post_grab_audit_frames: int = 3
    delivery_visual_frames: int = 5
    batch_radius_m: float = 0.55
    near_material_max_distance_m: float = 0.85
    capture_clearance_m: float = 0.16
    max_batch_count: int = 3
    center_stop_radius_m: float = 0.0
    return_zero_tolerance_m: float = 0.025
    vision_stale_s: float = 0.30
    search_min_turn_deg: float = 720.0
    search_target_confirm_frames: int = 1
    disperse_limit: int = 2
    approach_missing_hold_min_s: float = 0.30
    approach_missing_frame_multiplier: float = 2.5
    yield_distance_m: float = 0.25
    detour_lateral_m: float = 0.25
    boundary_margin_m: float = 0.08
    fence_axial_tolerance_m: float = 0.035
    fence_lateral_tolerance_m: float = 0.060
    safe_zone_center_x_m: float = 0.15
    safe_zone_inner_edge_m: float = 1.20
    safe_fence_face_m: float = 1.14
    safe_zone_staging_distance_m: float = 0.60
    safe_zone_freeze_frames: int = 3
    safe_zone_acquire_timeout_s: float = 5.0
    safe_sweep_capture_offset_mm: float = 150.0
    push_plate_offset_m: float = 0.105
    fence_stop_margin_m: float = 0.0075
    delivery_observation_timeout_s: float = 5.0
    delivery_window_s: float = 1.0
    delivery_window_frames: int = 7
    delivery_inside_required: int = 5
    delivery_max_misses: int = 2
    delivery_max_observations: int = 2
    stuck_observation_s: float = 3.0
    stuck_translation_m: float = 0.03
    stuck_yaw_deg: float = 3.0
    stuck_wheel_progress_m: float = 0.02
    escape_spin_deg: int = 90
    escape_lateral_m: float = 0.16

    def __post_init__(self) -> None:
        if self.side not in {"red", "blue"}:
            raise ValueError("side must be red or blue")
        if self.start_zone not in {1, 2, 3, 4}:
            raise ValueError("start zone must be 1..4")
        if self.max_batch_count != 3:
            raise ValueError("competition batch maximum must remain 3")
        if (
            self.audit_stable_frames <= 0 or
            self.normal_grab_audit_frames <= 0 or
            self.cluster_grab_audit_frames <= 0 or
            self.post_grab_audit_frames <= 0 or
            self.delivery_visual_frames <= 0
        ):
            raise ValueError("audit frame counts must be positive")
        if self.center_stop_radius_m < 0 or self.return_zero_tolerance_m <= 0:
            raise ValueError("return distance thresholds are invalid")
        if self.vision_stale_s <= 0:
            raise ValueError("vision stale threshold must be positive")
        if self.search_min_turn_deg <= 0 or self.search_target_confirm_frames <= 0:
            raise ValueError("search scan thresholds must be positive")
        if (
            self.approach_missing_hold_min_s <= 0 or
            self.approach_missing_frame_multiplier <= 0
        ):
            raise ValueError("cluster and approach timing settings must be positive")
        if self.near_material_max_distance_m <= 0:
            raise ValueError("near material distance must be positive")
        if self.boundary_margin_m < 0:
            raise ValueError("boundary margin cannot be negative")
        if (
            self.safe_zone_staging_distance_m <= 0 or
            self.safe_zone_freeze_frames <= 0 or
            self.safe_zone_acquire_timeout_s <= 0 or
            self.safe_sweep_capture_offset_mm < 0
        ):
            raise ValueError("safe-zone staging parameters must be positive")
        if self.delivery_observation_timeout_s <= 0 or self.delivery_window_s <= 0:
            raise ValueError("delivery observation thresholds must be positive")
        if self.delivery_window_frames <= 0 or self.delivery_inside_required <= 0:
            raise ValueError("delivery window frame thresholds must be positive")
        if not 0 <= self.delivery_max_misses < self.delivery_window_frames:
            raise ValueError("delivery miss threshold is invalid")
        if not 1 <= self.delivery_max_observations <= 2:
            raise ValueError("delivery observation attempts must be 1 or 2")
        if (
            self.stuck_observation_s <= 0 or
            self.stuck_translation_m <= 0 or
            self.stuck_yaw_deg <= 0 or
            self.stuck_wheel_progress_m <= 0
        ):
            raise ValueError("stuck observation thresholds must be positive")

    @property
    def side_sign(self) -> float:
        return 1.0 if self.side == "red" else -1.0

    @property
    def material_target_x_m(self) -> float:
        return -self.safe_zone_center_x_m if self.side == "red" else self.safe_zone_center_x_m

    @property
    def injury_target_x_m(self) -> float:
        return -self.material_target_x_m

    @property
    def safe_center_y_m(self) -> float:
        return self.side_sign * 1.32

    @property
    def safe_heading_deg(self) -> float:
        return 90.0 if self.side == "red" else 270.0

    @property
    def fence_stop_y_m(self) -> float:
        return self.side_sign * (
            self.safe_fence_face_m
            - self.push_plate_offset_m
            - self.fence_stop_margin_m
        )

    @property
    def safe_staging_y_m(self) -> float:
        return self.side_sign * (
            self.safe_fence_face_m - self.safe_zone_staging_distance_m
        )

    @property
    def initial_start_pose(self) -> tuple[float, float]:
        right = self.start_zone in {2, 4}
        top = self.start_zone in {1, 2}
        return (1.35 if right else -1.35, 1.35 if top else -1.35)

    @property
    def stash_point(self) -> tuple[float, float]:
        # Keep the temporary pile away from the centre-to-safe-zone transport
        # corridor and away from either safe zone.  The point is a strategy
        # parameter, not a field-coordinate correction.
        stash_x = 0.85 if self.start_zone in {2, 4} else -0.85
        stash_y = 0.55 if self.side == "red" else -0.55
        return stash_x, stash_y


class CompetitionMission:
    """Upper-level competition strategy with lower-level safety handshakes."""

    def __init__(self, settings: CompetitionSettings) -> None:
        self.settings = settings
        self.state = CompetitionState.WAIT_START
        self.state_started_s = 0.0
        self.initial_stash_done = not settings.initial_stash_enabled
        self.stash_has_cargo = False
        self.stash_zero_tx_baseline: int | None = None
        self.stash_handoff_hold_tx_baseline: int | None = None
        self.first_common_delivered = False
        self.first_green_bump_used = False
        self.delivery_count = 0
        self.selected_batch: CargoBatch | None = None
        self.first_fault_code: int | None = None
        self.audit_id = 0
        self.audit_last_signature: tuple | None = None
        self.audit_last_frame_sequence: int | None = None
        self.audit_hits = 0
        self.audit_recheck_frame_floor: int | None = None
        self.audit_recheck_started_s: float | None = None
        self.audit_legal_candidate_pending = False
        self.audit_invalid_after_legal_hits = 0
        self.pending_audit: CargoAudit | None = None
        self.pending_audit_payload: CargoAuditPayload | None = None
        self.pending_audit_valid = False
        self.pending_audit_release_side = "both"
        self.pending_audit_final = False
        self.pending_audit_release_context = "none"
        self.pending_audit_signature: tuple | None = None
        self.pending_audit_hits = 0
        self.pending_audit_frame_sequence: int | None = None
        self.pending_audit_observed_s: float | None = None
        self.pending_audit_tx_baseline: int | None = None
        self.pending_audit_initial_ack: int | None = None
        self.pending_audit_event_emitted = False
        self.invalid_release_side = "both"
        self.invalid_release_final = False
        self.invalid_release_context = "none"
        self.initial_release_initial_ack: int | None = None
        self.initial_release_tx_baseline: int | None = None
        self.initial_release_command_accepted = False
        self.invalid_release_initial_ack: int | None = None
        self.invalid_release_tx_baseline: int | None = None
        self.invalid_release_command_accepted = False
        self.grab_initial_ack: int | None = None
        self.grab_complete_confirmed = False
        self.post_grab_audit_active = False
        self.post_grab_camera_ready: bool | None = None
        self.navigation_initial_ack: int | None = None
        self.enter_initial_ack: int | None = None
        self.task_complete_initial_ack: int | None = None
        self.return_initial_ack: int | None = None
        self.return_relay_tx_baseline: int | None = None
        self.return_command_accepted = False
        self.return_search_frame_floor: int | None = None
        self.return_search_mode_seen = False
        self.stash_zero_initial_ack: int | None = None
        self.stash_handoff_initial_ack: int | None = None
        self.cargo_recheck_pending = False
        self.cargo_recheck_context = "none"
        self.safe_zone_align_initial_ack: int | None = None
        self.safe_zone_align_tx_baseline: int | None = None
        self.safe_zone_align_command_accepted = False
        self.safe_zone_visual_align_initial_ack: int | None = None
        self.safe_zone_visual_align_tx_baseline: int | None = None
        self.safe_zone_visual_align_command_accepted = False
        self.staging_zero_initial_ack: int | None = None
        self.staging_zero_tx_baseline: int | None = None
        self.staging_zero_accepted = False
        self.staging_camera_mismatch_reported = False
        self.safe_zone_acquire_started_s: float | None = None
        self.safe_zone_freeze_frame_floor: int | None = None
        self.safe_zone_freeze_last_frame_sequence: int | None = None
        self.safe_zone_freeze_hits = 0
        self.safe_zone_freeze_boxes: deque[tuple[int, int, int, int]] = deque(
            maxlen=settings.safe_zone_freeze_frames
        )
        self.locked_safe_bbox: tuple[int, int, int, int] | None = None
        self.locked_safe_target_x_px: int | None = None
        self.safe_zone_visual_pixel_error = 0
        self.safe_zone_visual_locked = False
        self.safe_zone_fallback = False
        self.safe_corridor_frame_floor: int | None = None
        self.safe_corridor_last_frame_sequence: int | None = None
        self.safe_sweep_attempts = 0
        self.safe_sweep_command: CommandRequest | None = None
        self.safe_sweep_initial_ack: int | None = None
        self.safe_sweep_tx_baseline: int | None = None
        self.safe_sweep_command_accepted = False
        self.safe_sweep_execution_seen = False
        self.safe_sweep_reaudit_active = False
        self.delivery_outside_seen = False
        self.delivery_inside_hits = 0
        self.delivery_visual_confirmed = False
        self.delivery_last_frame_sequence: int | None = None
        self.search_yaw_accum_deg = 0.0
        self.search_last_yaw_deg: float | None = None
        self.search_epoch_frame_floor: int | None = None
        self.search_candidate_key: tuple | None = None
        self.search_candidate_hits = 0
        self.stash_checked = False
        self.disperse_attempts = 0
        self.disperse_command: CommandRequest | None = None
        self.disperse_initial_ack: int | None = None
        self.disperse_relay_tx_baseline: int | None = None
        self.disperse_expected_done_mode = STM_MODE_DISPERSE_DONE
        self.disperse_context = "observe"
        self.disperse_observe_attempts = 0
        self.disperse_command_accepted = False
        self.cluster_command: CommandRequest | None = None
        self.cluster_initial_ack: int | None = None
        self.cluster_relay_tx_baseline: int | None = None
        self.cluster_command_accepted = False
        self.cluster_execution_seen = False
        self.cluster_id = 0
        self.cluster_target_class: str | None = None
        self.cluster_target_track_id: int | None = None
        self.cluster_target_bbox: tuple[int, int, int, int] | None = None
        self.cluster_bbox: tuple[int, int, int, int] | None = None
        self.cluster_signature: tuple | None = None
        self.cluster_keep_side: str | None = None
        self.cluster_keep_side_count = 0
        self.cluster_audit_active = False
        self.disperse_side_votes: deque[tuple[int, str | None, int]] = deque(
            maxlen=1
        )
        self.disperse_side_vote_last_frame_sequence: int | None = None
        self.target_last_seen_s: float | None = None
        self.target_last_center_px: tuple[int, int] | None = None
        self.target_last_area_px: int | None = None
        self.target_last_class: str | None = None
        self.target_missing_frames = 0
        self.target_missing_frame_sequence: int | None = None
        self.vision_frame_period_s: float | None = None
        self.vision_period_last_sequence: int | None = None
        self.vision_period_last_observed_s: float | None = None
        self.locked_target_track_id: int | None = None
        self.approach_f407_active_seen = False
        self.approach_initial_ack: int | None = None
        self.approach_command_accepted = False
        self.last_approach_command: CommandRequest | None = None
        self.last_selected_track_ids: tuple[int, ...] = ()
        self.search_recovery_resume_state: CompetitionState | None = None
        self.resume_state: CompetitionState | None = None
        self.detour_lateral_m = settings.detour_lateral_m
        self.detour_initial_ack: int | None = None
        self.detour_tx_baseline: int | None = None
        self.detour_command_accepted = False
        self.detour_execution_seen = False
        self.danger_event_latched = False
        self.danger_clear_frames = 0
        self.invalid_backoff_initial_ack: int | None = None
        self.invalid_backoff_tx_baseline: int | None = None
        self.invalid_backoff_command_accepted = False
        self.stm_fault_waiting = False
        self.motion_watch_state: CompetitionState | None = None
        self.motion_watch_started_s: float | None = None
        self.motion_watch_pose: tuple[float, float, float] | None = None
        self.motion_watch_wheel_m: float | None = None
        self.stuck_resume_state: CompetitionState | None = None
        self.stuck_initial_ack: int | None = None
        self.stuck_tx_baseline: int | None = None
        self.stuck_command_accepted = False
        self.stuck_recovery_level = 0
        self.safe_zone_exit_pending = False
        self.delivery_window: deque[bool] = deque(maxlen=settings.delivery_window_frames)
        self.delivery_window_started_s: float | None = None
        self.delivery_last_new_frame_s: float | None = None
        self.delivery_observation_started_s: float | None = None
        self.delivery_observation_attempt = 0
        self.delivery_timeout_reason = ""
        self.delivery_conflict_frames = 0
        self.delivery_last_frame_age_ms: float | None = None
        self.carried_manifest: tuple[str, ...] = ()
        self.carried_total_count = 0
        self.carried_has_green = False
        self.carried_has_core = False
        self.carried_green_core_mixed = False
        self.confirmed_delivery_destination: str | None = None
        self.delivery_completion_basis = ""
        self.enter_tx_baseline: int | None = None
        self.enter_command_accepted = False

    def _set_state(self, state: CompetitionState, now: float) -> None:
        if self.state != state:
            self.state = state
            self.state_started_s = now
            self.audit_last_signature = None
            self.audit_last_frame_sequence = None
            self.audit_hits = 0
            if state != CompetitionState.WAIT_SEARCH_RECOVERY:
                self.search_recovery_resume_state = None
            if state != CompetitionState.INVALID_BACKOFF:
                self.invalid_backoff_initial_ack = None
                self.invalid_backoff_tx_baseline = None
                self.invalid_backoff_command_accepted = False
            if state != CompetitionState.DISPERSE:
                self.disperse_command = None
                self.disperse_initial_ack = None
                self.disperse_relay_tx_baseline = None
                self.disperse_command_accepted = False
            if state != CompetitionState.CLUSTER_APPROACH:
                self.cluster_command = None
                self.cluster_initial_ack = None
                self.cluster_relay_tx_baseline = None
                self.cluster_command_accepted = False
                self.cluster_execution_seen = False
            if state != CompetitionState.DETOUR:
                self.detour_initial_ack = None
                self.detour_tx_baseline = None
                self.detour_command_accepted = False
                self.detour_execution_seen = False
            if state != CompetitionState.INITIAL_RELEASE:
                self.initial_release_initial_ack = None
                self.initial_release_tx_baseline = None
                self.initial_release_command_accepted = False
            if state != CompetitionState.INVALID_RELEASE:
                self.invalid_release_initial_ack = None
                self.invalid_release_tx_baseline = None
                self.invalid_release_command_accepted = False
            if state != CompetitionState.GRAB:
                self.grab_complete_confirmed = False
            if state != CompetitionState.RETURN_CENTER:
                self.return_initial_ack = None
                self.return_relay_tx_baseline = None
                self.return_command_accepted = False
            if state != CompetitionState.ALIGN_SAFE_ZONE_BY_POSE:
                self.safe_zone_align_initial_ack = None
                self.safe_zone_align_tx_baseline = None
                self.safe_zone_align_command_accepted = False
            if state != CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX:
                self.safe_zone_visual_align_initial_ack = None
                self.safe_zone_visual_align_tx_baseline = None
                self.safe_zone_visual_align_command_accepted = False
            if state != CompetitionState.CLEAR_SAFE_ZONE:
                self.safe_sweep_command = None
                self.safe_sweep_initial_ack = None
                self.safe_sweep_tx_baseline = None
                self.safe_sweep_command_accepted = False
                self.safe_sweep_execution_seen = False
            if state != CompetitionState.NAVIGATE:
                self.staging_zero_initial_ack = None
                self.staging_zero_tx_baseline = None
                self.staging_zero_accepted = False
                self.staging_camera_mismatch_reported = False
            if state != CompetitionState.WAIT_STASH_SEARCH_HANDOFF:
                self.stash_handoff_hold_tx_baseline = None
            if state not in {
                CompetitionState.FIELD_STUCK_ESCAPE,
                CompetitionState.SAFE_ZONE_ESCAPE,
            }:
                self.stuck_initial_ack = None
                self.stuck_tx_baseline = None
                self.stuck_command_accepted = False
            if state not in {
                CompetitionState.ENTER_SAFE_ZONE,
                CompetitionState.DELIVERY_VERIFY,
            }:
                self.enter_tx_baseline = None
                self.enter_command_accepted = False
            if state == CompetitionState.SEARCH:
                self.search_yaw_accum_deg = 0.0
                self.search_last_yaw_deg = None
                self.search_epoch_frame_floor = None
                self.search_candidate_key = None
                self.search_candidate_hits = 0

    def _hold(self) -> CommandRequest:
        return CommandRequest(CMD_HOLD)

    def _pause(self) -> CommandRequest:
        return CommandRequest(CMD_PAUSE)

    def _side_flags(self) -> int:
        return CMD_VALID | (1 << 3 if self.settings.side == "red" else 0)

    def _start_ready(self, pose: PoseSnapshot, stm: StmSnapshot) -> bool:
        if not pose.valid or not stm.fresh:
            return False
        if stm.mode != STM_MODE_SEARCH:
            return False
        return _distance(
            (pose.x_m, pose.y_m), self.settings.initial_start_pose
        ) >= 0.20

    @staticmethod
    def _command_payload(command: CommandRequest) -> tuple[int, ...]:
        return tuple(command.to_frame(0)[4:12])

    @classmethod
    def _relay_sent_since(
        cls,
        stm: StmSnapshot,
        command: CommandRequest,
        baseline: int | None,
    ) -> bool:
        return (
            stm.fresh and
            baseline is not None and
            stm.relay_mission_tx_frames > baseline and
            stm.relay_last_mission_command == command.opcode and
            stm.relay_last_mission_payload == cls._command_payload(command)
        )

    @staticmethod
    def _relay_sent_stage_zero_since(
        stm: StmSnapshot, baseline: int | None
    ) -> bool:
        payload = stm.relay_last_mission_payload
        expected_flags = CMD_VALID | CMD_DISTANCE_VALID | CMD_STAGE_ONLY
        if payload and payload[1] & CMD_RED_SIDE:
            expected_flags |= CMD_RED_SIDE
        return (
            stm.fresh and
            baseline is not None and
            stm.relay_mission_tx_frames > baseline and
            stm.relay_last_mission_command == CMD_NAVIGATE_WAYPOINT and
            len(payload) == 8 and
            payload[1] == expected_flags and
            payload[2:6] == (0, 0, 0, 0)
        )

    @staticmethod
    def _relay_sent_opcode_since(
        stm: StmSnapshot, opcode: int, baseline: int | None
    ) -> bool:
        return (
            stm.fresh and
            baseline is not None and
            stm.relay_mission_tx_frames > baseline and
            stm.relay_last_mission_command == opcode
        )

    @classmethod
    def _command_acceptance_seen(
        cls,
        stm: StmSnapshot,
        command: CommandRequest,
        baseline: int | None,
        initial_ack: int | None,
        accepted: bool,
    ) -> bool:
        return accepted or (
            initial_ack is not None and
            cls._relay_sent_since(stm, command, baseline) and
            stm.acknowledged_sequence != initial_ack
        )

    @classmethod
    def _opcode_acceptance_seen(
        cls,
        stm: StmSnapshot,
        opcode: int,
        baseline: int | None,
        initial_ack: int | None,
        accepted: bool,
    ) -> bool:
        return accepted or (
            initial_ack is not None and
            cls._relay_sent_opcode_since(stm, opcode, baseline) and
            stm.acknowledged_sequence != initial_ack
        )

    @staticmethod
    def _pose_fresh(pose: PoseSnapshot) -> bool:
        return pose.valid and pose.age_ms <= 250.0

    def expected_stm_modes(self) -> tuple[int, ...]:
        if self.state == CompetitionState.WAIT_SEARCH_RECOVERY:
            return (STM_MODE_APPROACH_TARGET, STM_MODE_APPROACH_RECOVER, STM_MODE_SEARCH)
        if self.state == CompetitionState.INITIAL_OBSERVE:
            return (STM_MODE_SEARCH,)
        if self.state in {CompetitionState.INITIAL_APPROACH, CompetitionState.APPROACH}:
            return (
                STM_MODE_APPROACH_TARGET,
                STM_MODE_CAPTURE_AUDIT,
                STM_MODE_APPROACH_RECOVER,
            )
        if self.state == CompetitionState.CLUSTER_APPROACH:
            return (
                STM_MODE_APPROACH_TARGET,
                STM_MODE_CLUSTER_CAPTURE_AUDIT,
                STM_MODE_APPROACH_RECOVER,
                STM_MODE_SEARCH,
            )
        if self.state == CompetitionState.CAPTURE_AUDIT:
            return (
                STM_MODE_CAPTURE_AUDIT,
                STM_MODE_CLUSTER_CAPTURE_AUDIT,
                STM_MODE_CLUSTER_READY,
                STM_MODE_APPROACH_RECOVER,
                STM_MODE_SEARCH,
            )
        if self.state == CompetitionState.AUDIT_CONFIRM:
            if self.pending_audit_release_context == "recheck_empty_to_search":
                return (STM_MODE_CAPTURE_AUDIT, STM_MODE_SEARCH)
            if self.pending_audit_release_context.startswith("post_grab_"):
                return (
                    STM_MODE_POST_GRAB_AUDIT,
                    STM_MODE_CAPTURE_DONE,
                    STM_MODE_SEARCH,
                )
            return (
                STM_MODE_CAPTURE_AUDIT,
                STM_MODE_CAPTURE_DONE,
                STM_MODE_CLUSTER_CAPTURE_AUDIT,
                STM_MODE_CLUSTER_READY,
                STM_MODE_APPROACH_RECOVER,
                STM_MODE_SEARCH,
            )
        if self.state == CompetitionState.GRAB:
            return (STM_MODE_CAPTURE_AUDIT, STM_MODE_CLUSTER_READY, STM_MODE_POST_GRAB_AUDIT)
        if self.state == CompetitionState.POST_GRAB_AUDIT:
            return (
                STM_MODE_POST_GRAB_AUDIT,
                STM_MODE_CAPTURE_DONE,
                STM_MODE_SAFE_SWEEP_DONE,
                STM_MODE_SEARCH,
            )
        if self.state in {
            CompetitionState.INITIAL_STASH_NAV,
            CompetitionState.NAVIGATE,
        }:
            return (STM_MODE_NAVIGATE,)
        if self.state in {
            CompetitionState.ALIGN_SAFE_ZONE_BY_POSE,
            CompetitionState.ACQUIRE_SAFE_ZONE,
            CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX,
            CompetitionState.SAFE_ZONE_CORRIDOR_CHECK,
            CompetitionState.WAIT_SAFE_ZONE_CLEAR,
        }:
            return (STM_MODE_NAVIGATE, STM_MODE_ALIGN_SAFE_ZONE)
        if self.state == CompetitionState.CLEAR_SAFE_ZONE:
            return (
                STM_MODE_ALIGN_SAFE_ZONE,
                STM_MODE_SAFE_SWEEP,
                STM_MODE_POST_GRAB_AUDIT,
            )
        if self.state in {
            CompetitionState.ENTER_SAFE_ZONE,
            CompetitionState.DELIVERY_VERIFY,
        }:
            return (STM_MODE_RAM_VERIFY,)
        if self.state == CompetitionState.TASK_COMPLETE:
            return (STM_MODE_EXIT_SAFE_ZONE, STM_MODE_FACE_FIELD_CENTER)
        if self.state == CompetitionState.RETURN_CENTER:
            return (STM_MODE_FACE_FIELD_CENTER, STM_MODE_SEARCH)
        if self.state == CompetitionState.DISPERSE:
            if self.disperse_context == "first_green_bump":
                return (STM_MODE_REMOTE_ACTION, STM_MODE_SEARCH)
            return (self.disperse_expected_done_mode,)
        if self.state == CompetitionState.INITIAL_RELEASE:
            return (STM_MODE_RELEASE_BOTH_DONE,)
        if self.state == CompetitionState.INVALID_RELEASE:
            if (
                self.invalid_release_context == "separate_then_search" and
                not self.invalid_release_final
            ):
                return (STM_MODE_DISPERSE_DONE,)
            return ({
                "left": STM_MODE_RELEASE_LEFT_DONE,
                "right": STM_MODE_RELEASE_RIGHT_DONE,
                "both": STM_MODE_RELEASE_BOTH_DONE,
            }[self.invalid_release_side],)
        if self.state == CompetitionState.INVALID_BACKOFF:
            return (STM_MODE_YIELD_DONE,)
        if self.state in {
            CompetitionState.FIELD_STUCK_ESCAPE,
            CompetitionState.SAFE_ZONE_ESCAPE,
        }:
            return (STM_MODE_REMOTE_ACTION, STM_MODE_ESCAPE_DONE)
        if self.state == CompetitionState.RETURN_STASH:
            return (STM_MODE_NAVIGATE,)
        if self.state == CompetitionState.WAIT_STASH_SEARCH_HANDOFF:
            return (STM_MODE_SEARCH,)
        return ()

    def diagnostic_selected_track_ids(self) -> tuple[int, ...]:
        if self.selected_batch is not None:
            return self.selected_batch.track_ids
        if self.state == CompetitionState.WAIT_SEARCH_RECOVERY:
            return self.last_selected_track_ids
        return ()

    def target_last_seen_age_ms(self, now: float) -> float | None:
        if self.target_last_seen_s is None:
            return None
        return max(0.0, (now - self.target_last_seen_s) * 1000.0)

    def _update_vision_frame_period(self, vision: VisionSnapshot) -> None:
        sequence = vision.frame_sequence
        observed_s = vision.observed_monotonic_s
        if sequence <= 0 or observed_s is None:
            return
        if (
            self.vision_period_last_sequence is not None and
            self.vision_period_last_observed_s is not None and
            sequence > self.vision_period_last_sequence and
            observed_s > self.vision_period_last_observed_s
        ):
            sample_s = observed_s - self.vision_period_last_observed_s
            if self.vision_frame_period_s is None:
                self.vision_frame_period_s = sample_s
            else:
                self.vision_frame_period_s = (
                    0.8 * self.vision_frame_period_s + 0.2 * sample_s
                )
        if (
            self.vision_period_last_sequence is None or
            sequence != self.vision_period_last_sequence
        ):
            self.vision_period_last_sequence = sequence
            self.vision_period_last_observed_s = observed_s

    def _approach_missing_hold_window_s(self) -> float:
        if self.vision_frame_period_s is None:
            return self.settings.approach_missing_hold_min_s
        return max(
            self.settings.approach_missing_hold_min_s,
            self.settings.approach_missing_frame_multiplier *
            self.vision_frame_period_s,
        )

    def _mark_target_seen(self, candidate: TrackedCargo, now: float) -> None:
        self.target_last_seen_s = now
        self.target_last_center_px = candidate.center_px
        self.target_last_area_px = candidate.area_px
        self.target_last_class = candidate.class_name
        self.target_missing_frames = 0
        self.target_missing_frame_sequence = None
        previous_track_id = self.locked_target_track_id
        self.locked_target_track_id = candidate.track_id
        if (
            self.selected_batch is not None and
            previous_track_id is not None and
            previous_track_id != candidate.track_id and
            previous_track_id in self.selected_batch.track_ids
        ):
            track_ids = list(self.selected_batch.track_ids)
            track_ids[track_ids.index(previous_track_id)] = candidate.track_id
            self.selected_batch = replace(self.selected_batch, track_ids=tuple(track_ids))
        if self.selected_batch is not None:
            self.last_selected_track_ids = self.selected_batch.track_ids
        else:
            self.last_selected_track_ids = (candidate.track_id,)

    def _select_batch(self, batch: CargoBatch) -> None:
        self.selected_batch = batch
        self._clear_disperse_side_votes()
        self.invalid_release_context = "none"
        self.invalid_release_final = False
        self._clear_carried_manifest()
        self.confirmed_delivery_destination = None
        self.delivery_completion_basis = ""
        self.locked_target_track_id = None
        self.target_last_center_px = None
        self.target_last_area_px = None
        self.target_last_class = None
        self.target_missing_frames = 0
        self.target_missing_frame_sequence = None
        self.approach_f407_active_seen = False
        self.approach_initial_ack = None
        self.approach_command_accepted = False
        self.last_approach_command = None

    def _clear_selected_batch(self) -> None:
        self.selected_batch = None
        self.confirmed_delivery_destination = None
        self.post_grab_audit_active = False
        self.post_grab_camera_ready = None
        self.safe_sweep_reaudit_active = False
        self._clear_disperse_side_votes()
        self.locked_target_track_id = None
        self.target_missing_frames = 0
        self.target_missing_frame_sequence = None
        self.approach_f407_active_seen = False
        self.approach_initial_ack = None
        self.approach_command_accepted = False
        self.last_approach_command = None

    def _clear_carried_manifest(self) -> None:
        self.carried_manifest = ()
        self.carried_total_count = 0
        self.carried_has_green = False
        self.carried_has_core = False
        self.carried_green_core_mixed = False

    def _clear_cluster_context(self, *, reset_attempts: bool) -> None:
        self._clear_disperse_side_votes()
        if reset_attempts:
            self.disperse_attempts = 0
            self.disperse_observe_attempts = 0
        self.cluster_target_class = None
        self.cluster_target_track_id = None
        self.cluster_target_bbox = None
        self.cluster_bbox = None
        self.cluster_signature = None
        self.cluster_keep_side = None
        self.cluster_keep_side_count = 0
        self.cluster_audit_active = False

    def _arm_approach(self, stm: StmSnapshot) -> None:
        self.approach_initial_ack = (
            stm.acknowledged_sequence if stm.fresh else None
        )
        self.approach_command_accepted = False
        self.last_approach_command = None

    def _update_search_scan_progress(self, pose: PoseSnapshot) -> None:
        if not self._pose_fresh(pose):
            return
        if self.search_last_yaw_deg is not None:
            self.search_yaw_accum_deg += abs(
                angle_error_deg(pose.yaw_deg, self.search_last_yaw_deg)
            )
        self.search_last_yaw_deg = pose.yaw_deg

    def _observe_search_candidate(
        self, key: tuple | None, vision: VisionSnapshot
    ) -> bool:
        if vision.frame_sequence <= 0:
            self.search_candidate_key = None
            self.search_candidate_hits = 0
            return False
        if (
            self.search_epoch_frame_floor is not None and
            vision.frame_sequence <= self.search_epoch_frame_floor
        ):
            return False
        if self.search_candidate_key != key:
            self.search_candidate_key = key
            self.search_candidate_hits = 1 if key is not None else 0
        elif key is not None:
            self.search_candidate_hits += 1
        return (
            key is not None and
            self.search_candidate_hits >= self.settings.search_target_confirm_frames
        )

    @staticmethod
    def _manifest_for_audit(audit: CargoAudit) -> tuple[str, ...]:
        classes: list[str] = []
        for class_name, count, green_count in (
            (audit.left_class, audit.left_count, audit.left_green_count),
            (audit.right_class, audit.right_count, audit.right_green_count),
        ):
            if count <= 0:
                continue
            if class_name == "mixed_material":
                bounded_green = max(0, min(count, green_count))
                classes.extend(["green_supply"] * bounded_green)
                classes.extend(["core_black"] * (count - bounded_green))
            elif class_name:
                classes.extend([class_name] * count)
        if not classes and audit.total_count > 0:
            classes = ["unknown"] * audit.total_count
        return tuple(classes)

    def _audit_destination(self, audit: CargoAudit) -> str:
        assert self.selected_batch is not None
        if self.selected_batch.initial_stash:
            return self.selected_batch.destination
        manifest = self._manifest_for_audit(audit)
        if audit.total_count == 1 and manifest == ("injured_orange",):
            return "injury"
        if manifest and set(manifest).issubset(MATERIAL_CLASSES):
            return "material"
        return self.selected_batch.destination

    def _audit_payload(self, audit: CargoAudit) -> CargoAuditPayload:
        assert self.selected_batch is not None
        return audit.to_protocol(
            initial_stash=self.selected_batch.initial_stash,
            destination=self._audit_destination(audit),
            audit_id=self.audit_id,
        )

    def _audit_signature_for_state(
        self, audit: CargoAudit, audit_valid: bool
    ) -> tuple:
        if (
            self.selected_batch is not None and
            self.selected_batch.initial_stash and
            audit.total_count > 0
        ):
            return ("STASH_NONEMPTY",)
        if audit_valid:
            if not self.first_common_delivered:
                return ("FIRST_GREEN",)
            manifest = self._manifest_for_audit(audit)
            if audit.total_count == 1 and manifest == ("injured_orange",):
                return ("INJURY_SINGLE",)
            if manifest and set(manifest).issubset(MATERIAL_CLASSES):
                return ("MATERIAL_LEGAL", audit.total_count)
        # Match F407's task-semantic normalization.  An invalid observation
        # remains the same decision when green/core/mixed classification or
        # claw side jitters, as long as its count, blocking flags and encoded
        # destination stay unchanged.
        return (
            "INVALID",
            audit.total_count,
            audit.danger_present,
            audit.unknown_present,
            audit.injury_mixed,
            self._audit_destination(audit),
        )

    def _latch_carried_manifest(self, audit: CargoAudit) -> None:
        self.carried_manifest = self._manifest_for_audit(audit)
        self.carried_total_count = audit.total_count
        self.carried_has_green = (
            "green_supply" in self.carried_manifest
        )
        self.carried_has_core = "core_black" in self.carried_manifest
        self.carried_green_core_mixed = (
            self.carried_has_green and self.carried_has_core
        )
        if (
            self.selected_batch is not None and
            not self.selected_batch.initial_stash
        ):
            confirmed_destination: str | None = None
            if (
                audit.total_count == 1 and
                self.carried_manifest == ("injured_orange",)
            ):
                confirmed_destination = "injury"
            elif (
                self.carried_manifest and
                set(self.carried_manifest).issubset(MATERIAL_CLASSES)
            ):
                confirmed_destination = "material"
            if confirmed_destination is not None:
                self.confirmed_delivery_destination = confirmed_destination
                self.selected_batch = replace(
                    self.selected_batch,
                    classes=self.carried_manifest,
                    destination=confirmed_destination,
                    confirmed_total_count=audit.total_count,
                )
        self._clear_cluster_context(reset_attempts=True)

    def carried_delivery_classes(self) -> frozenset[str]:
        classes = set(self.carried_manifest)
        if self.carried_green_core_mixed:
            classes.update({"green_supply", "core_black"})
        return frozenset(classes)

    def carried_delivery_count(self) -> int:
        return max(1, self.carried_total_count, len(self.carried_manifest))

    def _clear_search_recovery_context(
        self,
        vision: VisionSnapshot,
        *,
        clear_carried: bool,
    ) -> None:
        self._clear_pending_audit()
        self._clear_selected_batch()
        if clear_carried:
            self._clear_carried_manifest()
        self._clear_cluster_context(reset_attempts=True)
        self.cargo_recheck_pending = False
        self.cargo_recheck_context = "none"
        self.audit_hits = 0
        self.audit_last_signature = None
        self.audit_last_frame_sequence = None
        self.audit_recheck_frame_floor = None
        self.audit_recheck_started_s = None
        self.audit_legal_candidate_pending = False
        self.audit_invalid_after_legal_hits = 0
        self.invalid_release_side = "both"
        self.invalid_release_final = False
        self.invalid_release_context = "none"
        self.last_selected_track_ids = ()
        self.target_last_seen_s = None
        self.target_last_center_px = None
        self.target_last_area_px = None
        self.target_last_class = None
        self.target_missing_frames = 0
        self.target_missing_frame_sequence = None
        self.search_epoch_frame_floor = (
            vision.frame_sequence if vision.frame_sequence > 0 else None
        )

    def _begin_search_recovery(
        self, vision: VisionSnapshot, now: float
    ) -> None:
        self._clear_search_recovery_context(vision, clear_carried=False)
        self._set_state(CompetitionState.WAIT_SEARCH_RECOVERY, now)
        self.search_recovery_resume_state = CompetitionState.SEARCH

    def _search_recovery_output(
        self,
        vision: VisionSnapshot,
        stm: StmSnapshot,
        now: float,
        *,
        event: str = "",
    ) -> CompetitionOutput:
        expected = (
            STM_MODE_APPROACH_TARGET,
            STM_MODE_APPROACH_RECOVER,
            STM_MODE_SEARCH,
        )
        if stm.fresh and stm.mode == STM_MODE_SEARCH:
            self._set_state(CompetitionState.SEARCH, now)
            self.search_epoch_frame_floor = (
                vision.frame_sequence if vision.frame_sequence > 0 else None
            )
            return CompetitionOutput(
                self.state,
                self._hold(),
                "F407已完成目标丢失恢复，等待重新搜索",
                event="search_recovery_complete",
                tx_policy="recovery_hold",
                reason="f407_search_recovery_complete",
                expected_stm_modes=expected,
            )
        if stm.fresh and stm.mode in {
            STM_MODE_APPROACH_TARGET, STM_MODE_APPROACH_RECOVER
        }:
            return CompetitionOutput(
                self.state,
                self._hold(),
                "持续发送HOLD，等待F407静止完成目标丢失恢复并进入mode3",
                event=event,
                tx_policy="hold",
                reason="f407_approach_recovery",
                expected_stm_modes=expected,
            )
        return CompetitionOutput(
            self.state,
            self._hold(),
            f"等待F407目标丢失恢复状态，当前mode={stm.mode}",
            event=event,
            tx_policy="hold",
            reason="f407_search_recovery_wait",
            expected_stm_modes=expected,
        )

    def _approach_output(
        self,
        vision: VisionSnapshot,
        stm: StmSnapshot,
        now: float,
        *,
        message: str,
    ) -> CompetitionOutput:
        vision_fresh = self._vision_fresh(vision, now)
        candidate = self._target_for_batch(vision) if vision_fresh else None
        if (
            stm.fresh and
            stm.mode in {
                STM_MODE_APPROACH_TARGET,
                STM_MODE_CAPTURE_AUDIT,
                STM_MODE_APPROACH_RECOVER,
            } and
            self.approach_initial_ack is not None and
            stm.acknowledged_sequence != self.approach_initial_ack
        ):
            self.approach_command_accepted = True
        if self.approach_command_accepted and stm.mode in {
            STM_MODE_APPROACH_TARGET, STM_MODE_APPROACH_RECOVER
        }:
            self.approach_f407_active_seen = True
        if (
            stm.fresh and
            stm.mode == STM_MODE_CAPTURE_AUDIT and
            stm.claw_visible and
            stm.camera_pitch_cdeg == 14000
        ):
            self.approach_command_accepted = True
            self._begin_capture_audit(
                vision,
                now,
                cluster_active=False,
                recheck_pending=False,
                recheck_context="none",
            )
            return self._audit_output(vision, stm, now)
        if stm.mode == STM_MODE_SEARCH and (
            self.approach_f407_active_seen or self.target_missing_frames > 2
        ):
            self._clear_search_recovery_context(vision, clear_carried=False)
            self._set_state(CompetitionState.SEARCH, now)
            self.search_epoch_frame_floor = (
                vision.frame_sequence if vision.frame_sequence > 0 else None
            )
            return CompetitionOutput(
                self.state,
                self._hold(),
                "F407已回到SEARCH，清除旧目标并等待恢复后的新视觉帧",
                event="search_recovery_complete",
                tx_policy="recovery_hold",
                reason="f407_search_recovery_complete",
                expected_stm_modes=(STM_MODE_SEARCH,),
            )
        if stm.mode == STM_MODE_APPROACH_RECOVER:
            self._begin_search_recovery(vision, now)
            return self._search_recovery_output(
                vision, stm, now, event="search_recovery_start"
            )
        if candidate is not None:
            self._mark_target_seen(candidate, now)
            if stm.mode in {
                STM_MODE_SEARCH,
                STM_MODE_APPROACH_TARGET,
                STM_MODE_CAPTURE_AUDIT,
                STM_MODE_APPROACH_RECOVER,
            }:
                approach_command = self._approach_command(candidate)
                self.last_approach_command = approach_command
                return CompetitionOutput(
                    self.state,
                    approach_command,
                    message,
                    self.selected_batch,
                    motion_expected=True,
                    tx_policy="normal_command",
                    reason="approach_target_fresh",
                    expected_stm_modes=(
                        STM_MODE_APPROACH_TARGET,
                        STM_MODE_CAPTURE_AUDIT,
                        STM_MODE_APPROACH_RECOVER,
                    ),
                )
        elif vision_fresh:
            self._note_target_missing_frame(vision)

        missing_hold_window_s = self._approach_missing_hold_window_s()
        if (
            self.target_last_seen_s is not None and
            now - self.target_last_seen_s <= missing_hold_window_s and
            self.last_approach_command is not None
        ):
            return CompetitionOutput(
                self.state,
                self.last_approach_command,
                (
                    "目标短暂缺帧，在动态窗口"
                    f"{missing_hold_window_s * 1000.0:.0f} ms内保留最后APPROACH坐标"
                ),
                self.selected_batch,
                motion_expected=True,
                tx_policy="normal_command",
                reason="approach_target_briefly_missing",
                expected_stm_modes=(
                    STM_MODE_APPROACH_TARGET,
                    STM_MODE_CAPTURE_AUDIT,
                    STM_MODE_APPROACH_RECOVER,
                    STM_MODE_SEARCH,
                ),
            )
        return CompetitionOutput(
            self.state,
            self._hold(),
            (
                "目标缺帧超过动态窗口"
                f"{missing_hold_window_s * 1000.0:.0f} ms，明确发送HOLD停车等待目标重新出现"
            ),
            self.selected_batch,
            tx_policy="hold",
            reason="approach_target_missing_hold",
            expected_stm_modes=(
                STM_MODE_APPROACH_TARGET,
                STM_MODE_APPROACH_RECOVER,
                STM_MODE_SEARCH,
            ),
        )

    def _arm_disperse(
        self,
        stm: StmSnapshot,
        keep_side: str | None,
        *,
        first_green_bump: bool = False,
    ) -> None:
        if first_green_bump:
            self.disperse_context = "first_green_bump"
            self.disperse_command = CommandRequest(
                CMD_DISPERSE_PILE,
                self._side_flags() | CMD_FIRST_GREEN_BUMP,
            )
            self.disperse_initial_ack = (
                stm.acknowledged_sequence if stm.fresh else None
            )
            self.disperse_relay_tx_baseline = stm.relay_mission_tx_frames
            self.disperse_command_accepted = False
            self.disperse_expected_done_mode = STM_MODE_SEARCH
            return
        if keep_side is not None and self.cluster_keep_side_count <= 0:
            keep_side = None
            self.cluster_keep_side = None
        flags = self._side_flags()
        if keep_side is not None:
            flags |= CMD_SIDE_VALID
        if keep_side == "right":
            flags |= CMD_TARGET_RIGHT
        self.disperse_context = (
            "selective" if keep_side is not None else "observe"
        )
        self.disperse_command = CommandRequest(CMD_DISPERSE_PILE, flags)
        self.disperse_initial_ack = stm.acknowledged_sequence if stm.fresh else None
        self.disperse_relay_tx_baseline = stm.relay_mission_tx_frames
        self.disperse_command_accepted = False
        self.disperse_expected_done_mode = STM_MODE_DISPERSE_DONE

    def _arm_detour(self, stm: StmSnapshot, now: float) -> None:
        self.detour_initial_ack = stm.acknowledged_sequence if stm.fresh else None
        self.detour_tx_baseline = stm.relay_mission_tx_frames
        self.detour_command_accepted = False
        self.detour_execution_seen = False

    def _arm_initial_release(self, stm: StmSnapshot, now: float) -> None:
        self.initial_release_initial_ack = stm.acknowledged_sequence if stm.fresh else None
        self.initial_release_tx_baseline = stm.relay_mission_tx_frames
        self.initial_release_command_accepted = False

    def _arm_invalid_release(self, stm: StmSnapshot, now: float) -> None:
        if self.invalid_release_initial_ack is None:
            self.invalid_release_initial_ack = (
                stm.acknowledged_sequence if stm.fresh else None
            )
        if self.invalid_release_tx_baseline is None:
            self.invalid_release_tx_baseline = stm.relay_mission_tx_frames
            self.invalid_release_command_accepted = False

    def _clear_pending_audit(self) -> None:
        self.pending_audit = None
        self.pending_audit_payload = None
        self.pending_audit_valid = False
        self.pending_audit_release_side = "both"
        self.pending_audit_final = False
        self.pending_audit_release_context = "none"
        self.pending_audit_signature = None
        self.pending_audit_hits = 0
        self.pending_audit_frame_sequence = None
        self.pending_audit_observed_s = None
        self.pending_audit_tx_baseline = None
        self.pending_audit_initial_ack = None
        self.pending_audit_event_emitted = False

    def _begin_capture_audit(
        self,
        vision: VisionSnapshot,
        now: float,
        *,
        cluster_active: bool,
        recheck_pending: bool,
        recheck_context: str,
    ) -> None:
        self.post_grab_audit_active = False
        self.post_grab_camera_ready = None
        self.cluster_audit_active = cluster_active
        self.cargo_recheck_pending = recheck_pending
        self.cargo_recheck_context = recheck_context
        self.audit_hits = 0
        self.audit_last_signature = None
        self.audit_last_frame_sequence = None
        self.audit_recheck_frame_floor = (
            vision.frame_sequence if vision.frame_sequence > 0 else None
        )
        self.audit_recheck_started_s = now
        self.audit_legal_candidate_pending = False
        self.audit_invalid_after_legal_hits = 0
        self._clear_disperse_side_votes()
        self._clear_pending_audit()
        self.cluster_keep_side = None
        self.cluster_keep_side_count = 0
        self._set_state(CompetitionState.CAPTURE_AUDIT, now)

    def _begin_post_grab_audit(
        self, vision: VisionSnapshot, now: float
    ) -> None:
        self.post_grab_audit_active = True
        self.post_grab_camera_ready = None
        self.cluster_audit_active = False
        self.audit_hits = 0
        self.audit_last_signature = None
        self.audit_last_frame_sequence = None
        self.audit_recheck_frame_floor = (
            vision.frame_sequence if vision.frame_sequence > 0 else None
        )
        self.audit_recheck_started_s = now
        self.audit_legal_candidate_pending = False
        self.audit_invalid_after_legal_hits = 0
        self._clear_disperse_side_votes()
        self.cluster_keep_side = None
        self.cluster_keep_side_count = 0
        self._clear_pending_audit()
        self._set_state(CompetitionState.POST_GRAB_AUDIT, now)

    def _observe_post_grab_camera_edge(
        self,
        vision: VisionSnapshot,
        stm: StmSnapshot,
        now: float,
    ) -> bool:
        if not (
            self.post_grab_audit_active and
            stm.fresh and
            stm.mode == STM_MODE_POST_GRAB_AUDIT
        ):
            return False
        ready = stm.camera_pitch_cdeg == 14000
        previous = self.post_grab_camera_ready
        self.post_grab_camera_ready = ready
        if previous is None:
            if ready:
                return False
        elif previous == ready:
            return False

        self.audit_hits = 0
        self.audit_last_signature = None
        self.audit_last_frame_sequence = None
        self.audit_recheck_frame_floor = (
            vision.frame_sequence if vision.frame_sequence > 0 else None
        )
        self.audit_recheck_started_s = now
        self.audit_legal_candidate_pending = False
        self.audit_invalid_after_legal_hits = 0
        self._clear_pending_audit()
        self._clear_disperse_side_votes()
        self.cluster_keep_side = None
        self.cluster_keep_side_count = 0
        if self.state == CompetitionState.AUDIT_CONFIRM:
            self._set_state(CompetitionState.POST_GRAB_AUDIT, now)
        return True

    def _audit_reacquire_output(
        self, vision: VisionSnapshot, stm: StmSnapshot, now: float
    ) -> CompetitionOutput:
        post_grab_recovery = (
            self.post_grab_audit_active or
            self.pending_audit_release_context.startswith("post_grab_")
        )
        self._clear_search_recovery_context(
            vision,
            clear_carried=post_grab_recovery,
        )
        self.grab_initial_ack = None
        if stm.mode == STM_MODE_SEARCH:
            self._set_state(CompetitionState.SEARCH, now)
            self.search_epoch_frame_floor = (
                vision.frame_sequence if vision.frame_sequence > 0 else None
            )
            return CompetitionOutput(
                self.state,
                self._hold(),
                (
                    "mode23有限视觉恢复仍未形成3帧审核，F407已双开回SEARCH"
                    if post_grab_recovery else
                    "近距离观察未确认夹内物资，F407已回到SEARCH"
                ),
                event=(
                    "post_grab_recovery_search"
                    if post_grab_recovery else
                    "capture_audit_search_recovered"
                ),
                tx_policy="recovery_hold",
                reason="capture_audit_search_recovered",
                expected_stm_modes=(STM_MODE_SEARCH,),
            )
        self._set_state(CompetitionState.WAIT_SEARCH_RECOVERY, now)
        self.search_recovery_resume_state = CompetitionState.SEARCH
        return CompetitionOutput(
            self.state,
            self._hold(),
            "F407进入mode24恢复，清除旧目标和审核并持续HOLD等待mode3",
            event="capture_audit_reacquire_start",
            tx_policy="hold",
            reason="f407_capture_reacquire",
            expected_stm_modes=(STM_MODE_APPROACH_RECOVER, STM_MODE_SEARCH),
        )

    def _begin_audit_confirmation(
        self,
        audit: CargoAudit,
        payload: CargoAuditPayload,
        valid: bool,
        release_side: str,
        final_release: bool,
        release_context: str,
        stm: StmSnapshot,
        now: float,
    ) -> None:
        self.pending_audit = audit
        self.pending_audit_payload = payload
        self.pending_audit_valid = valid
        self.pending_audit_release_side = release_side
        self.pending_audit_final = final_release
        self.pending_audit_release_context = release_context
        self.pending_audit_signature = self._audit_signature_for_state(
            audit, valid
        )
        self.pending_audit_hits = self.audit_hits
        self.pending_audit_frame_sequence = self.audit_last_frame_sequence
        self.pending_audit_observed_s = now
        self.pending_audit_tx_baseline = stm.relay_mission_tx_frames
        self.pending_audit_initial_ack = (
            stm.acknowledged_sequence if stm.fresh else None
        )
        self.pending_audit_event_emitted = False
        self._set_state(CompetitionState.AUDIT_CONFIRM, now)

    def _resume_legal_audit_before_disperse(
        self, vision: VisionSnapshot, stm: StmSnapshot, now: float
    ) -> CompetitionOutput | None:
        """Do not start DISPERSE when the latest fresh claw audit is legal."""
        if not (
            self._vision_fresh(vision, now) and
            stm.fresh and
            stm.claw_visible and
            stm.camera_pitch_cdeg == 14000 and
            vision.capture_audit is not None
        ):
            return None
        audit = vision.capture_audit
        cluster_screening = (
            self.cluster_audit_active or
            self.cargo_recheck_context in {
                "disperse_observe",
                "disperse_selective",
            }
        )
        audit_valid = (
            self._cluster_audit_valid(audit)
            if cluster_screening else self._audit_valid(audit)
        )
        if not audit_valid:
            return None

        self._clear_pending_audit()
        self._set_state(CompetitionState.CAPTURE_AUDIT, now)
        self.audit_recheck_frame_floor = None
        self.audit_recheck_started_s = None
        self.audit_legal_candidate_pending = False
        self.audit_invalid_after_legal_hits = 0
        self._clear_disperse_side_votes()
        self.cluster_keep_side = None
        self.cluster_keep_side_count = 0
        return self._audit_output(vision, stm, now)

    def _audit_confirmation_output(
        self, vision: VisionSnapshot, stm: StmSnapshot, now: float
    ) -> CompetitionOutput:
        assert self.pending_audit is not None
        assert self.pending_audit_payload is not None
        stable_command = CommandRequest(
            CMD_CARGO_AUDIT,
            CMD_VALID,
            audit=self.pending_audit_payload,
        )
        if (
            stm.fresh and
            stm.mode in {STM_MODE_APPROACH_RECOVER, STM_MODE_SEARCH} and
            not (
                stm.mode == STM_MODE_SEARCH and
                self.pending_audit_release_context == "recheck_empty_to_search"
            )
        ):
            return self._audit_reacquire_output(vision, stm, now)
        if (
            self.pending_audit_release_context.startswith("post_grab_") and
            stm.fresh and
            stm.mode == STM_MODE_POST_GRAB_AUDIT
        ):
            camera_edge = self._observe_post_grab_camera_edge(
                vision, stm, now
            )
            if camera_edge or stm.camera_pitch_cdeg != 14000:
                return self._audit_output(vision, stm, now)
        if (
            self.pending_audit_valid and
            not stm.audit_valid and
            self._vision_fresh(vision, now) and
            vision.frame_sequence > 0 and
            (
                self.pending_audit_frame_sequence is None or
                vision.frame_sequence > self.pending_audit_frame_sequence
            ) and
            vision.capture_audit is not None and
            stm.claw_visible and
            stm.camera_pitch_cdeg == 14000 and
            stm.mode in {
                STM_MODE_CAPTURE_AUDIT,
                STM_MODE_CLUSTER_READY,
                STM_MODE_POST_GRAB_AUDIT,
            }
        ):
            post_grab = self.pending_audit_release_context == "post_grab_valid"
            continued_signature = self.pending_audit_signature
            continued_hits = self.pending_audit_hits
            continued_frame_sequence = self.pending_audit_frame_sequence
            self._clear_pending_audit()
            self._set_state(
                CompetitionState.POST_GRAB_AUDIT
                if post_grab else CompetitionState.CAPTURE_AUDIT,
                now,
            )
            self.audit_last_signature = continued_signature
            self.audit_hits = continued_hits
            self.audit_last_frame_sequence = continued_frame_sequence
            return self._audit_output(vision, stm, now)
        if self.pending_audit_release_context == "cluster_capture":
            if (
                self._relay_sent_since(
                    stm, stable_command, self.pending_audit_tx_baseline
                ) and
                self._fresh_mode_after(
                    stm, STM_MODE_CLUSTER_READY, self.pending_audit_initial_ack
                ) and
                (not self.pending_audit_valid or stm.audit_valid)
            ):
                audit = self.pending_audit
                valid = self.pending_audit_valid
                self._clear_pending_audit()
                if valid:
                    self.cluster_audit_active = False
                    self.post_grab_audit_active = False
                    self.cargo_recheck_pending = False
                    self.cargo_recheck_context = "none"
                    self._set_state(CompetitionState.GRAB, now)
                    self.grab_initial_ack = stm.acknowledged_sequence
                    return CompetitionOutput(
                        self.state,
                        CommandRequest(CMD_GRAB_CONFIRMED, self._side_flags()),
                        "聚集目标夹内审核合法，持续发送GRAB_CONFIRMED",
                        self.selected_batch,
                        audit,
                        event="cluster_audit_grab_confirmed",
                        tx_policy="normal_command",
                        reason="cluster_audit_valid",
                        expected_stm_modes=(STM_MODE_CAPTURE_DONE,),
                    )
                legal_output = self._resume_legal_audit_before_disperse(
                    vision, stm, now
                )
                if legal_output is not None:
                    return legal_output
                self.cluster_audit_active = False
                if self._should_first_green_bump(audit):
                    return self._start_first_green_bump(stm, now)
                self.disperse_attempts += 1
                return self._start_disperse(stm, now)
            event = (
                "" if self.pending_audit_event_emitted
                else "cluster_audit_stable_publish"
            )
            self.pending_audit_event_emitted = True
            return CompetitionOutput(
                self.state,
                stable_command,
                "持续发送聚集目标稳定夹内审核，等待ACK和mode=37",
                self.selected_batch,
                self.pending_audit,
                event=event,
                tx_policy="audit_stable_publish",
                reason="cluster_audit_pending_ready",
                expected_stm_modes=(
                    STM_MODE_CLUSTER_CAPTURE_AUDIT,
                    STM_MODE_CLUSTER_READY,
                    STM_MODE_APPROACH_RECOVER,
                    STM_MODE_SEARCH,
                ),
            )
        if self.pending_audit_release_context == "recheck_empty_to_search":
            if (
                self._relay_sent_since(
                    stm, stable_command, self.pending_audit_tx_baseline
                ) and
                self._fresh_mode_after(
                    stm, STM_MODE_SEARCH, self.pending_audit_initial_ack
                )
            ):
                empty_audit = self.pending_audit
                post_grab_empty = self.post_grab_audit_active
                self._clear_selected_batch()
                self._clear_cluster_context(reset_attempts=True)
                self.post_grab_audit_active = False
                self.post_grab_camera_ready = None
                if post_grab_empty:
                    self._clear_carried_manifest()
                self.cargo_recheck_pending = False
                self.cargo_recheck_context = "none"
                self.audit_hits = 0
                self.audit_last_signature = None
                self.audit_last_frame_sequence = None
                self.audit_recheck_frame_floor = None
                self.audit_recheck_started_s = None
                self.last_selected_track_ids = ()
                self._clear_pending_audit()
                self._set_state(CompetitionState.SEARCH, now)
                return CompetitionOutput(
                    self.state,
                    self._hold(),
                    "稳定空爪审核已由F407确认并进入mode=3，重新搜索目标",
                    audit=empty_audit,
                    event="recheck_empty_search_confirmed",
                    tx_policy="hold",
                    reason="recheck_empty_acknowledged",
                    expected_stm_modes=(STM_MODE_SEARCH,),
                )
            event = (
                "" if self.pending_audit_event_emitted
                else "recheck_empty_stable_publish"
            )
            self.pending_audit_event_emitted = True
            return CompetitionOutput(
                self.state,
                stable_command,
                "持续发送稳定空爪审核，等待relay、ACK和F407 mode=3确认",
                self.selected_batch,
                self.pending_audit,
                event=event,
                tx_policy="audit_stable_publish",
                reason="recheck_empty_pending_search",
                expected_stm_modes=(
                    STM_MODE_CAPTURE_AUDIT,
                    STM_MODE_POST_GRAB_AUDIT,
                    STM_MODE_SEARCH,
                ),
            )
        if self.pending_audit_release_context == "post_grab_valid":
            expected_done_mode = (
                STM_MODE_SAFE_SWEEP_DONE
                if self.safe_sweep_reaudit_active else
                STM_MODE_CAPTURE_DONE
            )
            if (
                self._relay_sent_since(
                    stm, stable_command, self.pending_audit_tx_baseline
                ) and
                self._fresh_mode_after(
                    stm, expected_done_mode, self.pending_audit_initial_ack
                ) and
                stm.gripper_closed and
                stm.audit_valid
            ):
                audit = self.pending_audit
                self._clear_pending_audit()
                self._latch_carried_manifest(audit)
                self.post_grab_audit_active = False
                self.post_grab_camera_ready = None
                if self.safe_sweep_reaudit_active:
                    self.safe_sweep_reaudit_active = False
                    self._begin_safe_zone_pose_align(stm, now)
                    return CompetitionOutput(
                        self.state,
                        self._safe_zone_pose_align_command(),
                        "扫障后3帧审核合法且F407进入mode40，重新执行定位ALIGN",
                        self.selected_batch,
                        audit,
                        event="safe_zone_clear_realign_start",
                        tx_policy="normal_command",
                        reason="safe_zone_clear_realign_start",
                        expected_stm_modes=(STM_MODE_ALIGN_SAFE_ZONE,),
                    )
                self.grab_complete_confirmed = True
                self._set_state(CompetitionState.GRAB, now)
                return CompetitionOutput(
                    self.state,
                    None,
                    "合爪后3帧审核已确认，下一周期启动对应目的地NAV",
                    self.selected_batch,
                    audit,
                    event="post_grab_audit_confirmed",
                    tx_policy="post_grab_handoff",
                    reason="post_grab_audit_confirmed",
                    expected_stm_modes=(STM_MODE_CAPTURE_DONE,),
                )
            event = (
                "post_grab_audit_stable_publish"
                if not self.pending_audit_event_emitted else ""
            )
            self.pending_audit_event_emitted = True
            return CompetitionOutput(
                self.state,
                stable_command,
                "持续发送合爪后稳定审核，等待AUDIT_VALID和mode=22",
                self.selected_batch,
                self.pending_audit,
                event=event,
                tx_policy="audit_stable_publish",
                reason="post_grab_audit_pending",
                expected_stm_modes=(
                    STM_MODE_POST_GRAB_AUDIT,
                    STM_MODE_CAPTURE_DONE,
                    STM_MODE_SAFE_SWEEP_DONE,
                ),
            )
        if self.pending_audit_release_context == "post_grab_invalid":
            if (
                self._relay_sent_since(
                    stm, stable_command, self.pending_audit_tx_baseline
                ) and
                self._fresh_mode_after(
                    stm, STM_MODE_POST_GRAB_AUDIT,
                    self.pending_audit_initial_ack,
                )
            ):
                audit = self.pending_audit
                self._clear_pending_audit()
                release_side = self._choose_release_side(audit)
                if (
                    (release_side == "left" and audit.left_count <= 0) or
                    (release_side == "right" and audit.right_count <= 0)
                ):
                    release_side = "both"
                self.invalid_release_side = release_side
                self.invalid_release_final = release_side == "both"
                self.invalid_release_context = (
                    "final_release"
                    if release_side == "both" else
                    "single_side_then_reaudit"
                )
                self.post_grab_audit_active = False
                self.post_grab_camera_ready = None
                self._set_state(CompetitionState.INVALID_RELEASE, now)
                self._arm_invalid_release(stm, now)
                return self._invalid_release_output(audit, stm, now)
            event = (
                "post_grab_invalid_stable_publish"
                if not self.pending_audit_event_emitted else ""
            )
            self.pending_audit_event_emitted = True
            return CompetitionOutput(
                self.state,
                stable_command,
                "持续发送合爪后非法审核，等待F407确认后执行释放分离",
                self.selected_batch,
                self.pending_audit,
                event=event,
                tx_policy="audit_stable_publish",
                reason="post_grab_invalid_pending",
                expected_stm_modes=(STM_MODE_POST_GRAB_AUDIT,),
            )
        if not self._vision_fresh(vision, now):
            self._clear_pending_audit()
            self._set_state(CompetitionState.CAPTURE_AUDIT, now)
            return CompetitionOutput(
                self.state,
                self._pause(),
                "稳定审核视觉帧已过期，取消待推进审核",
                event="cargo_audit_pending_expired",
                tx_policy="pause",
                reason="audit_visual_stale",
            )
        audit_confirm_mode = (
            STM_MODE_SAFE_SWEEP_DONE
            if self.pending_audit_valid and self.safe_sweep_reaudit_active else
            STM_MODE_CAPTURE_AUDIT
        )
        if (
            self._relay_sent_since(
                stm, stable_command, self.pending_audit_tx_baseline
            ) and
            self._fresh_mode_after(
                stm, audit_confirm_mode, self.pending_audit_initial_ack
            ) and
            (not self.pending_audit_valid or stm.audit_valid)
        ):
            audit = self.pending_audit
            valid = self.pending_audit_valid
            release_side = self.pending_audit_release_side
            final_release = self.pending_audit_final
            release_context = self.pending_audit_release_context
            self._clear_pending_audit()
            if valid:
                if self.safe_sweep_reaudit_active:
                    self._latch_carried_manifest(audit)
                    self.safe_sweep_reaudit_active = False
                    self.post_grab_audit_active = False
                    self.cargo_recheck_pending = False
                    self.cargo_recheck_context = "none"
                    self._begin_safe_zone_pose_align(stm, now)
                    return CompetitionOutput(
                        self.state,
                        self._safe_zone_pose_align_command(),
                        "扫障分离复审合法且F407进入mode40，重新执行定位ALIGN",
                        self.selected_batch,
                        audit,
                        event="safe_zone_clear_realign_start",
                        tx_policy="normal_command",
                        reason="safe_zone_clear_realign_start",
                        expected_stm_modes=(STM_MODE_ALIGN_SAFE_ZONE,),
                    )
                self.post_grab_audit_active = False
                self.cargo_recheck_pending = False
                self.cargo_recheck_context = "none"
                self._set_state(CompetitionState.GRAB, now)
                self.grab_initial_ack = stm.acknowledged_sequence
                return CompetitionOutput(
                    self.state,
                    CommandRequest(CMD_GRAB_CONFIRMED, self._side_flags()),
                    f"稳定审核已转发确认：{audit.total_count}件，等待合爪",
                    self.selected_batch,
                    audit,
                    event="cargo_audit_relay_confirmed",
                    tx_policy="normal_command",
                    reason="stable_audit_relay_confirmed",
                )
            if release_context == "disperse_reaudit":
                legal_output = self._resume_legal_audit_before_disperse(
                    vision, stm, now
                )
                if legal_output is not None:
                    return legal_output
                if self._should_first_green_bump(audit):
                    return self._start_first_green_bump(stm, now)
                if self.cluster_keep_side is not None:
                    self.disperse_attempts += 1
                    return self._start_disperse(stm, now)
                if self.disperse_observe_attempts < 2:
                    self.cluster_keep_side_count = 0
                    self.disperse_attempts += 1
                    return self._start_disperse(stm, now)
                self.invalid_release_side = "both"
                self.invalid_release_final = True
                self.invalid_release_context = "disperse_final_release"
                self._set_state(CompetitionState.INVALID_RELEASE, now)
                self._arm_invalid_release(stm, now)
                return self._invalid_release_output(audit, stm, now)
            self.invalid_release_side = release_side
            self.invalid_release_final = final_release
            self.invalid_release_context = release_context
            self._set_state(CompetitionState.INVALID_RELEASE, now)
            self._arm_invalid_release(stm, now)
            return self._invalid_release_output(audit, stm, now)
        event = "" if self.pending_audit_event_emitted else "cargo_audit_stable_publish"
        self.pending_audit_event_emitted = True
        return CompetitionOutput(
            self.state,
            stable_command,
            "持续转发稳定夹内审核，等待relay成功证据",
            self.selected_batch,
            self.pending_audit,
            event=event,
            tx_policy="audit_stable_publish",
            reason="stable_audit_pending_relay",
            expected_stm_modes=(STM_MODE_CAPTURE_AUDIT, STM_MODE_CAPTURE_DONE),
        )

    @staticmethod
    def _visible_cargo(vision: VisionSnapshot) -> list[TrackedCargo]:
        return [
            item for item in vision.cargo
            if item.visible and not item.inside_safe_zone and item.class_name in CARGO_CLASSES
        ]

    def _vision_fresh(self, vision: VisionSnapshot, now: float) -> bool:
        # Direct unit-test snapshots may omit a timestamp. Live runner frames
        # always carry a monotonic timestamp so stale detections cannot start
        # a new physical action.
        if vision.observed_monotonic_s is None:
            return True
        age = now - vision.observed_monotonic_s
        # ``step`` can begin just before the vision thread publishes a frame;
        # allow a small same-clock scheduling skew without accepting genuinely
        # old data.
        return -0.05 <= age <= self.settings.vision_stale_s

    def _target_for_batch(self, vision: VisionSnapshot) -> TrackedCargo | None:
        if self.selected_batch is None:
            return None
        visible = self._visible_cargo(vision)
        if self.locked_target_track_id is not None:
            locked = next(
                (item for item in visible if item.track_id == self.locked_target_track_id),
                None,
            )
            if locked is not None:
                return locked
        selected = [
            item for item in visible if item.track_id in self.selected_batch.track_ids
        ]
        if selected:
            if self.selected_batch.initial_stash:
                return max(
                    selected,
                    key=lambda item: (
                        item.bbox[1] + item.bbox[3],
                        item.area_px,
                        -item.track_id,
                    ),
                )
            return max(selected, key=lambda item: item.area_px)
        target_class = self.target_last_class
        fallback = [
            item for item in visible
            if item.class_name == target_class or (
                target_class is None and item.class_name in self.selected_batch.classes
            )
        ]
        if len(fallback) == 1:
            return fallback[0]
        if not fallback or self.target_last_center_px is None:
            return None

        if self.selected_batch.initial_stash:
            return max(
                fallback,
                key=lambda item: (
                    item.bbox[1] + item.bbox[3],
                    item.area_px,
                    -item.track_id,
                ),
            )

        last_area = max(1, self.target_last_area_px or 1)
        last_x, last_y = self.target_last_center_px

        def continuity_score(item: TrackedCargo) -> float:
            cx, cy = item.center_px
            centre_distance = math.hypot(cx - last_x, cy - last_y)
            size_scale = max(16.0, math.sqrt(last_area), math.sqrt(max(1, item.area_px)))
            area_ratio = max(1, item.area_px) / last_area
            return centre_distance / size_scale + abs(math.log(area_ratio))

        ranked = sorted(fallback, key=lambda item: (continuity_score(item), -item.area_px))
        if continuity_score(ranked[1]) - continuity_score(ranked[0]) < 0.35:
            return None
        return ranked[0]

    def _note_target_missing_frame(self, vision: VisionSnapshot) -> None:
        if vision.frame_sequence <= 0:
            return
        if vision.frame_sequence == self.target_missing_frame_sequence:
            return
        self.target_missing_frame_sequence = vision.frame_sequence
        self.target_missing_frames += 1

    def _is_isolated(self, target: TrackedCargo, cargo: Iterable[TrackedCargo]) -> bool:
        neighbours = [item for item in cargo if item.track_id != target.track_id]
        if target.relative_xy_m is not None:
            return all(
                item.relative_xy_m is None or
                _distance(target.relative_xy_m, item.relative_xy_m) >= self.settings.capture_clearance_m
                for item in neighbours
            )
        x, y, width, height = target.bbox
        target_cx, target_cy = x + width * 0.5, y + height * 0.5
        for item in neighbours:
            ox, oy, ow, oh = item.bbox
            other_cx, other_cy = ox + ow * 0.5, oy + oh * 0.5
            pixel_gap = math.hypot(target_cx - other_cx, target_cy - other_cy)
            if pixel_gap < max(width, height) * 1.35:
                return False
        return True

    def _choose_initial_green(self, vision: VisionSnapshot) -> TrackedCargo | None:
        candidates = [
            item for item in self._visible_cargo(vision)
            if item.class_name == "green_supply"
        ]
        isolated = [item for item in candidates if self._is_isolated(item, self._visible_cargo(vision))]
        return min(
            isolated,
            key=lambda item: (item.distance_m, -item.area_px, item.track_id),
            default=None,
        )

    def _choose_material_batch(self, vision: VisionSnapshot) -> CargoBatch | None:
        candidates = [
            item for item in self._visible_cargo(vision)
            if item.class_name in MATERIAL_CLASSES and item.hits >= 2
        ]
        if not candidates:
            return None
        best: tuple[float, list[TrackedCargo]] | None = None
        for seed in sorted(candidates, key=lambda item: (item.distance_m, -item.area_px)):
            if seed.relative_xy_m is None:
                group = candidates[: self.settings.max_batch_count]
            else:
                group = [
                    item for item in candidates
                    if item.relative_xy_m is not None and
                    _distance(seed.relative_xy_m, item.relative_xy_m) <= self.settings.batch_radius_m
                ]
                group.sort(key=lambda item: (item.distance_m, -item.area_px))
                group = group[: self.settings.max_batch_count]
            group = [seed] + [item for item in group if item.track_id != seed.track_id]
            group = group[: self.settings.max_batch_count]
            cost = max((item.distance_m for item in group), default=math.inf)
            if not math.isfinite(cost):
                cost = -sum(item.area_px for item in group) / 1_000_000.0
            if best is None or cost < best[0]:
                best = cost, group
        if best is None:
            return None
        group = best[1]
        return CargoBatch(
            tuple(item.track_id for item in group),
            tuple(item.class_name for item in group),
            "material",
        )

    def _choose_next_batch(self, vision: VisionSnapshot) -> CargoBatch | None:
        material = self._choose_material_batch(vision)
        casualty = min(
            [
                item for item in self._visible_cargo(vision)
                if item.class_name == "injured_orange" and item.hits >= 2
            ],
            key=lambda item: (item.distance_m, -item.area_px, item.track_id),
            default=None,
        )
        if material is not None:
            material_items = [
                item for item in self._visible_cargo(vision)
                if item.track_id in material.track_ids
            ]
            material_distance = min(
                (item.distance_m for item in material_items),
                default=math.inf,
            )
            if casualty is None or material_distance <= self.settings.near_material_max_distance_m:
                return material
        if casualty is not None:
            return CargoBatch((casualty.track_id,), (casualty.class_name,), "injury")
        return material

    def _pile_batch(self, vision: VisionSnapshot) -> CargoBatch | None:
        cargo = [
            item for item in self._visible_cargo(vision)
            if item.hits >= 2
        ]
        if not cargo:
            return None
        cargo.sort(key=lambda item: (-item.area_px, item.distance_m, item.track_id))
        return CargoBatch(
            tuple(item.track_id for item in cargo),
            tuple(item.class_name for item in cargo),
            "stash",
            initial_stash=True,
        )

    def _mixed_pile_requires_disperse(self, vision: VisionSnapshot) -> bool:
        cargo = [item for item in self._visible_cargo(vision) if item.hits >= 2]
        for item in cargo:
            neighbours = [
                other for other in cargo
                if other.track_id != item.track_id and
                not self._is_isolated(item, (item, other))
            ]
            if not neighbours:
                continue
            classes = {item.class_name, *(other.class_name for other in neighbours)}
            if "danger_cyan" in classes:
                return True
            if "injured_orange" in classes and classes & MATERIAL_CLASSES:
                return True
        return False

    def _audit_valid(self, audit: CargoAudit) -> bool:
        counts = Counter()
        if audit.left_class and audit.left_class != "mixed_material":
            counts[audit.left_class] += audit.left_count
        if audit.right_class and audit.right_class != "mixed_material":
            counts[audit.right_class] += audit.right_count
        if self.selected_batch is None:
            return False
        if self.selected_batch.initial_stash:
            return audit.total_count > 0
        if (
            audit.total_count <= 0 or
            audit.total_count > self.settings.max_batch_count or
            audit.left_count + audit.right_count != audit.total_count
        ):
            return False
        if (
            audit.danger_present or
            audit.unknown_present or
            audit.left_class in {"danger_cyan", "unknown"} or
            audit.right_class in {"danger_cyan", "unknown"} or
            audit.injury_mixed
        ):
            return False
        if not self.first_common_delivered:
            return (
                audit.total_count == 1 and
                counts == Counter({"green_supply": 1})
            )
        if (
            audit.total_count == 1 and
            counts == Counter({"injured_orange": 1})
        ):
            return True
        if "injured_orange" in counts:
            return False
        side_classes = {audit.left_class, audit.right_class} - {"", "mixed_material"}
        return (
            1 <= audit.total_count <= self.settings.max_batch_count and
            side_classes.issubset(MATERIAL_CLASSES) and
            audit.left_class not in {"injured_orange", "danger_cyan", "unknown"} and
            audit.right_class not in {"injured_orange", "danger_cyan", "unknown"} and
            not audit.injury_mixed
        )

    def _cluster_audit_valid(self, audit: CargoAudit) -> bool:
        if self.selected_batch is None:
            return False
        if not self.first_common_delivered:
            return self._audit_valid(audit)
        return self._audit_valid(audit)

    def _side_is_legal_for_task(self, side_class: str, side_count: int) -> bool:
        if side_count <= 0:
            return False
        if self.selected_batch is not None and self.selected_batch.initial_stash:
            return (
                (1 <= side_count <= self.settings.max_batch_count and
                 side_class in {"green_supply", "core_black", "mixed_material"}) or
                (side_count == 1 and side_class == "injured_orange")
            )
        if not self.first_common_delivered:
            return side_count == 1 and side_class == "green_supply"
        if self.selected_batch is not None and self.selected_batch.destination == "injury":
            return side_count == 1 and side_class == "injured_orange"
        return (
            1 <= side_count <= self.settings.max_batch_count and
            side_class in {"green_supply", "core_black", "mixed_material"}
        )

    def _preferred_keep_side(self, audit: CargoAudit) -> str | None:
        left_nonempty = audit.left_count > 0
        right_nonempty = audit.right_count > 0
        left_green = audit.left_green_count
        right_green = audit.right_green_count
        if left_green > 0 and right_green == 0:
            return "left"
        if right_green > 0 and left_green == 0:
            return "right"
        if left_green > 0 and right_green > 0:
            if left_green != right_green:
                return "left" if left_green < right_green else "right"
            return "left"
        if left_nonempty:
            return "left"
        if right_nonempty:
            return "right"
        return None

    def _should_first_green_bump(self, audit: CargoAudit) -> bool:
        if self.first_common_delivered or self.first_green_bump_used:
            return False
        green_total = audit.left_green_count + audit.right_green_count
        if audit.total_count < 2 or green_total <= 0:
            return False
        left_isolated_green = (
            audit.left_count == 1 and audit.left_green_count == 1
        )
        right_isolated_green = (
            audit.right_count == 1 and audit.right_green_count == 1
        )
        return not (left_isolated_green or right_isolated_green)

    def _clear_disperse_side_votes(self) -> None:
        self.disperse_side_votes.clear()
        self.disperse_side_vote_last_frame_sequence = None

    def _observe_disperse_side_vote(
        self, frame_sequence: int, audit: CargoAudit
    ) -> None:
        if (
            frame_sequence <= 0 or
            frame_sequence == self.disperse_side_vote_last_frame_sequence
        ):
            return
        self.disperse_side_vote_last_frame_sequence = frame_sequence
        keep_side = self._preferred_keep_side(audit)
        keep_count = {
            "left": audit.left_count,
            "right": audit.right_count,
        }.get(keep_side, 0)
        if keep_count <= 0:
            keep_side = None
            keep_count = 0
        self.disperse_side_votes.append(
            (frame_sequence, keep_side, keep_count)
        )

    def _disperse_side_vote_result(self) -> tuple[str | None, int, bool]:
        if not self.disperse_side_votes:
            return None, 0, False
        _, side, keep_count = self.disperse_side_votes[-1]
        if side is not None and keep_count > 0:
            return side, keep_count, True
        return None, 0, True

    def _choose_release_side(self, audit: CargoAudit) -> str:
        """Return the side to open; RELEASE_LEFT means discard left cargo."""
        if (
            self.cargo_recheck_pending and
            self.cargo_recheck_context != "disperse_selective"
        ):
            return "both"
        # An unknown category or an audit-level unknown flag cannot identify a
        # safe side.  Do not guess by count in that case.
        if (
            audit.unknown_present or
            audit.left_class == "unknown" or
            audit.right_class == "unknown"
        ):
            return "both"
        left_danger = audit.left_class == "danger_cyan"
        right_danger = audit.right_class == "danger_cyan"
        if audit.danger_present and not (left_danger or right_danger):
            return "both"
        if (
            audit.injury_mixed and
            audit.left_class != "injured_orange" and
            audit.right_class != "injured_orange"
        ):
            return "both"
        left_legal = self._side_is_legal_for_task(audit.left_class, audit.left_count)
        right_legal = self._side_is_legal_for_task(audit.right_class, audit.right_count)

        if left_danger and right_legal:
            return "left"
        if right_danger and left_legal:
            return "right"
        if left_legal != right_legal:
            return "right" if left_legal else "left"

        # A mixed injury/material audit can be salvaged when the two sides are
        # individually classifiable and the current destination identifies the
        # side to retain.
        if audit.left_class == "injured_orange" and audit.right_class in {
            "green_supply", "core_black", "mixed_material"
        }:
            return "left" if self.selected_batch and self.selected_batch.destination != "injury" else "right"
        if audit.right_class == "injured_orange" and audit.left_class in {
            "green_supply", "core_black", "mixed_material"
        }:
            return "right" if self.selected_batch and self.selected_batch.destination != "injury" else "left"

        if left_legal and right_legal:
            keep_side = self._preferred_keep_side(audit)
            if keep_side == "left":
                return "right"
            if keep_side == "right":
                return "left"
        return "both"

    def _audit_output(
        self, vision: VisionSnapshot, stm: StmSnapshot, now: float
    ) -> CompetitionOutput:
        if stm.fresh and stm.mode in {
            STM_MODE_APPROACH_RECOVER,
            STM_MODE_SEARCH,
        }:
            return self._audit_reacquire_output(vision, stm, now)
        self._observe_post_grab_camera_edge(vision, stm, now)
        if not (
            stm.fresh and
            stm.claw_visible and
            stm.camera_pitch_cdeg == 14000
        ):
            assert self.selected_batch is not None
            empty_payload = self._audit_payload(CargoAudit())
            return CompetitionOutput(
                self.state,
                CommandRequest(CMD_CARGO_AUDIT, CMD_VALID, audit=empty_payload),
                (
                    "mode23相机有限恢复中，持续发送当前audit_id的非STABLE空审核"
                    if (
                        self.post_grab_audit_active and
                        stm.fresh and
                        stm.mode == STM_MODE_POST_GRAB_AUDIT and
                        stm.camera_pitch_cdeg in {13800, 14200}
                    ) else
                    "等待新鲜CLAW_VISIBLE和相机140°，不使用全局检测建立夹内审核"
                ),
                self.selected_batch,
                event="capture_audit_gate_wait",
                tx_policy="audit_unstable_publish",
                reason="capture_audit_140_gate_wait",
                expected_stm_modes=(
                    STM_MODE_CAPTURE_AUDIT,
                    STM_MODE_CLUSTER_CAPTURE_AUDIT,
                    STM_MODE_DISPERSE_DONE,
                    STM_MODE_APPROACH_RECOVER,
                    STM_MODE_SEARCH,
                ),
            )
        if not self._vision_fresh(vision, now):
            assert self.selected_batch is not None
            empty_payload = self._audit_payload(CargoAudit())
            return CompetitionOutput(
                self.state,
                CommandRequest(
                    CMD_CARGO_AUDIT,
                    CMD_VALID,
                    audit=empty_payload,
                ),
                "夹爪ROI暂未取得新鲜确认，发送非STABLE全零审核等待新帧",
                self.selected_batch,
                event="cargo_audit_no_observation",
                tx_policy="audit_unstable_publish",
                reason="audit_no_observation",
                expected_stm_modes=(
                    STM_MODE_CAPTURE_AUDIT,
                    STM_MODE_APPROACH_RECOVER,
                    STM_MODE_SEARCH,
                ),
            )
        if (
            self.cargo_recheck_pending or
            self.cluster_audit_active or
            self.audit_recheck_frame_floor is not None or
            self.audit_recheck_started_s is not None
        ):
            new_sequence = (
                vision.frame_sequence > 0 and
                (
                    self.audit_recheck_frame_floor is None or
                    vision.frame_sequence > self.audit_recheck_frame_floor
                )
            )
            if (
                self.audit_recheck_frame_floor is not None or
                self.audit_recheck_started_s is not None
            ) and not new_sequence:
                assert self.selected_batch is not None
                empty_payload = self._audit_payload(CargoAudit())
                return CompetitionOutput(
                    self.state,
                    CommandRequest(
                        CMD_CARGO_AUDIT,
                        CMD_VALID,
                        audit=empty_payload,
                    ),
                    "等待动作完成后的新夹内视觉帧，发送非STABLE全零审核",
                    self.selected_batch,
                    event="audit_recheck_wait_frame",
                    tx_policy="audit_unstable_publish",
                    reason="audit_recheck_wait_frame",
                )
            self.audit_recheck_frame_floor = None
            self.audit_recheck_started_s = None
        if vision.capture_audit is None:
            assert self.selected_batch is not None
            if self.cargo_recheck_pending or self.post_grab_audit_active:
                empty_audit = CargoAudit(
                    left_class="none",
                    right_class="none",
                    left_count=0,
                    right_count=0,
                    total_count=0,
                    stable=False,
                )
                empty_new_frame = (
                    vision.frame_sequence > 0 and
                    vision.frame_sequence != self.audit_last_frame_sequence
                )
                if empty_new_frame:
                    self.audit_last_frame_sequence = vision.frame_sequence
                    self.audit_id = (self.audit_id + 1) & 0xFF
                    empty_signature = self._audit_signature_for_state(
                        empty_audit, False
                    )
                    if empty_signature == self.audit_last_signature:
                        self.audit_hits += 1
                    else:
                        self.audit_last_signature = empty_signature
                        self.audit_hits = 1
                stable_empty = replace(
                    empty_audit,
                    stable=(
                        empty_new_frame and
                        self.audit_hits >= self.settings.audit_stable_frames
                    ),
                )
                if stable_empty.stable:
                    stable_payload = self._audit_payload(stable_empty)
                    self._begin_audit_confirmation(
                        stable_empty,
                        stable_payload,
                        False,
                        "both",
                        False,
                        "recheck_empty_to_search",
                        stm,
                        now,
                    )
                    return self._audit_confirmation_output(vision, stm, now)
            else:
                empty_new_frame = (
                    vision.frame_sequence > 0 and
                    vision.frame_sequence != self.audit_last_frame_sequence
                )
                if empty_new_frame:
                    self.audit_last_frame_sequence = vision.frame_sequence
                    self.audit_id = (self.audit_id + 1) & 0xFF
                self.audit_last_signature = None
                self.audit_hits = 0
            empty_payload = self._audit_payload(CargoAudit())
            return CompetitionOutput(
                self.state,
                CommandRequest(CMD_CARGO_AUDIT, CMD_VALID, audit=empty_payload),
                (
                        "合爪后暂未识别到物资，等待3帧稳定空爪审核"
                    if self.post_grab_audit_active else
                    "单侧分离后暂未识别到物资，原地等待稳定复审"
                    if self.cargo_recheck_pending else
                    "夹爪ROI暂未确认物资，发送非STABLE全零审核继续慢速观察"
                ),
                self.selected_batch,
                None,
                "cargo_audit_no_observation",
                tx_policy="audit_unstable_publish",
                reason=(
                    "audit_recheck_empty_stabilizing"
                    if self.cargo_recheck_pending else "audit_no_observation"
                ),
                expected_stm_modes=(
                    STM_MODE_CAPTURE_AUDIT,
                    STM_MODE_APPROACH_RECOVER,
                    STM_MODE_SEARCH,
                ),
            )
        audit = vision.capture_audit
        new_frame = (
            vision.frame_sequence > 0 and
            vision.frame_sequence != self.audit_last_frame_sequence
        )
        cluster_screening = (
            self.cluster_audit_active or
            self.cargo_recheck_context in {
                "disperse_observe",
                "disperse_selective",
            }
        )
        normal_grab_audit = (
            not cluster_screening and
            not self.cargo_recheck_pending
        )
        audit_valid = (
            self._cluster_audit_valid(audit)
            if cluster_screening else
            self._audit_valid(audit)
        )
        if new_frame:
            self.audit_last_frame_sequence = vision.frame_sequence
            audit_signature = self._audit_signature_for_state(
                audit, audit_valid
            )
            if audit_signature == self.audit_last_signature:
                self.audit_hits += 1
            else:
                self.audit_last_signature = audit_signature
                self.audit_hits = 1
            self.audit_id = (self.audit_id + 1) & 0xFF
        cluster_grab_audit = cluster_screening and audit_valid
        required_audit_frames = (
            self.settings.post_grab_audit_frames
            if self.post_grab_audit_active else
            self.settings.normal_grab_audit_frames
            if normal_grab_audit else
            (
                self.settings.cluster_grab_audit_frames
                if cluster_grab_audit else
                self.settings.audit_stable_frames
            )
        )
        stable = replace(
            audit,
            stable=(
                new_frame and
                self.audit_hits >= required_audit_frames
            ),
        )
        assert self.selected_batch is not None
        if cluster_screening and new_frame:
            if audit_valid:
                self.audit_legal_candidate_pending = True
                self.audit_invalid_after_legal_hits = 0
            else:
                self.audit_legal_candidate_pending = False
                self.audit_invalid_after_legal_hits = 0
        disperse_vote_context = (
            not audit_valid and
            (
                self.cluster_audit_active or
                self.cargo_recheck_context in {
                    "disperse_observe",
                    "disperse_selective",
                }
            )
        )
        if disperse_vote_context:
            if new_frame:
                self._observe_disperse_side_vote(
                    vision.frame_sequence, audit
                )
            keep_side, keep_count, vote_ready = (
                self._disperse_side_vote_result()
            )
            if not vote_ready:
                unstable = replace(audit, stable=False)
                unstable_payload = self._audit_payload(unstable)
                vote_summary = [
                    side or "unknown"
                    for _, side, _ in self.disperse_side_votes
                ]
                return CompetitionOutput(
                    self.state,
                    CommandRequest(
                        CMD_CARGO_AUDIT,
                        CMD_VALID,
                        audit=unstable_payload,
                    ),
                    f"等待frame floor后的新鲜非空夹内帧：{vote_summary}",
                    self.selected_batch,
                    unstable,
                    "disperse_side_vote_wait",
                    tx_policy="audit_unstable_publish",
                    reason="disperse_side_vote_pending",
                )
            self.cluster_keep_side = keep_side
            self.cluster_keep_side_count = keep_count
        payload = self._audit_payload(stable)
        if stable.stable and audit_valid:
            self._begin_audit_confirmation(
                stable,
                payload,
                True,
                "both",
                False,
                (
                    "post_grab_valid"
                    if self.post_grab_audit_active else
                    "cluster_capture"
                    if self.cluster_audit_active else "valid"
                ),
                stm,
                now,
            )
            return self._audit_confirmation_output(vision, stm, now)
        if stable.stable and not audit_valid:
            if self.post_grab_audit_active:
                self._begin_audit_confirmation(
                    stable,
                    payload,
                    False,
                    "both",
                    False,
                    "post_grab_invalid",
                    stm,
                    now,
                )
                return self._audit_confirmation_output(vision, stm, now)
            if self.cluster_audit_active:
                self._begin_audit_confirmation(
                    stable,
                    payload,
                    False,
                    "both",
                    False,
                    "cluster_capture",
                    stm,
                    now,
                )
                return self._audit_confirmation_output(vision, stm, now)
            if self.cargo_recheck_context in {
                "disperse_observe",
                "disperse_selective",
            }:
                self._begin_audit_confirmation(
                    stable,
                    payload,
                    False,
                    "both",
                    False,
                    "disperse_reaudit",
                    stm,
                    now,
                )
                return self._audit_confirmation_output(vision, stm, now)
            release_side = self._choose_release_side(stable)
            if (
                (release_side == "left" and stable.left_count <= 0) or
                (release_side == "right" and stable.right_count <= 0)
            ):
                release_side = "both"
            if (
                self.cargo_recheck_pending and
                self.cargo_recheck_context != "disperse_selective"
            ):
                release_side = "both"
                release_context = "final_release"
                final_release = True
            elif release_side == "both":
                release_context = "separate_then_search"
                final_release = False
            else:
                release_context = "single_side_then_reaudit"
                final_release = False
            self._begin_audit_confirmation(
                stable,
                payload,
                False,
                release_side,
                final_release,
                release_context,
                stm,
                now,
            )
            return self._audit_confirmation_output(vision, stm, now)
        return CompetitionOutput(
            self.state,
            CommandRequest(CMD_CARGO_AUDIT, CMD_VALID, audit=payload),
            f"等待夹内审核稳定：{stable.total_count}件，{self.audit_hits}/{required_audit_frames}",
            self.selected_batch,
            stable,
            "cargo_audit_wait",
            tx_policy="audit_unstable_publish",
            reason="audit_wait_stable",
        )

    def _invalid_release_output(
        self, audit: CargoAudit, stm: StmSnapshot, now: float
    ) -> CompetitionOutput:
        self._arm_invalid_release(stm, now)
        side = self.invalid_release_side
        release_command = self._invalid_release_command()
        if self.invalid_release_context == "separate_then_search":
            message = "首次无法判断左右归属，发送无侧DISPERSE执行20°观察转向"
        elif self.invalid_release_context in {
            "final_release",
            "disperse_final_release",
        }:
            message = "复审仍非法，执行最终双开"
        else:
            message = f"夹内组合非法，释放{side}侧后退复审"
        expected_mode = (
            STM_MODE_DISPERSE_DONE
            if (
                self.invalid_release_context == "separate_then_search" and
                not self.invalid_release_final
            )
            else {
                "left": STM_MODE_RELEASE_LEFT_DONE,
                "right": STM_MODE_RELEASE_RIGHT_DONE,
                "both": STM_MODE_RELEASE_BOTH_DONE,
            }[side]
        )
        return CompetitionOutput(
            self.state,
            release_command,
            message,
            self.selected_batch,
            audit,
            "invalid_cargo_release",
            tx_policy="normal_command",
            reason="invalid_audit_release",
            expected_stm_modes=(expected_mode,),
        )

    def _invalid_release_command(self) -> CommandRequest:
        if (
            self.invalid_release_context == "separate_then_search" and
            not self.invalid_release_final
        ):
            return CommandRequest(CMD_DISPERSE_PILE, self._side_flags())
        opcode = {
            "left": CMD_RELEASE_LEFT,
            "right": CMD_RELEASE_RIGHT,
            "both": CMD_RELEASE_BOTH,
        }[self.invalid_release_side]
        return CommandRequest(opcode, self._side_flags())

    def _update_delivery_observation(self, vision: VisionSnapshot, now: float) -> None:
        if self.state not in {
            CompetitionState.NAVIGATE,
            CompetitionState.ENTER_SAFE_ZONE,
            CompetitionState.DELIVERY_VERIFY,
        }:
            return
        if self.delivery_window_started_s is not None:
            last_sample = self.delivery_last_new_frame_s
            if (
                last_sample is None or
                now - last_sample > self.settings.delivery_window_s or
                now - self.delivery_window_started_s > self.settings.delivery_window_s
            ):
                self.delivery_window.clear()
                self.delivery_window_started_s = None
                self.delivery_last_new_frame_s = None
                self.delivery_inside_hits = 0
                self.delivery_visual_confirmed = False
        self.delivery_last_frame_age_ms = (
            None
            if vision.observed_monotonic_s is None
            else max(0.0, (now - vision.observed_monotonic_s) * 1000.0)
        )
        if not self._vision_fresh(vision, now):
            return
        if (
            vision.frame_sequence <= 0 or
            vision.frame_sequence == self.delivery_last_frame_sequence
        ):
            return
        self.delivery_last_frame_sequence = vision.frame_sequence
        if vision.delivery_target_outside_safe_zone:
            self.delivery_outside_seen = True
        if self.state != CompetitionState.DELIVERY_VERIFY:
            return
        if self.delivery_window_started_s is None:
            self.delivery_window_started_s = now
        self.delivery_last_new_frame_s = now
        inside_ids = set(vision.delivery_target_inside_track_ids)
        outside_ids = set(vision.delivery_target_outside_track_ids)
        conflict = False
        if inside_ids or outside_ids:
            conflict = bool(inside_ids and outside_ids)
        elif vision.delivery_target_inside_safe_zone and vision.delivery_target_outside_safe_zone:
            conflict = True
        inside = (
            self.delivery_outside_seen and
            vision.delivery_target_found and
            vision.delivery_target_inside_safe_zone and
            not conflict
        )
        if conflict:
            self.delivery_conflict_frames += 1
        self.delivery_window.append(inside)
        self.delivery_inside_hits = sum(self.delivery_window)
        misses = len(self.delivery_window) - self.delivery_inside_hits
        if (
            self.delivery_inside_hits >= self.settings.delivery_inside_required and
            misses <= self.settings.delivery_max_misses
        ):
            self.delivery_visual_confirmed = True

    def _start_delivery_observation(self, now: float, attempt: int) -> None:
        self.delivery_observation_attempt = attempt
        self.delivery_observation_started_s = now
        self.delivery_timeout_reason = ""
        self.delivery_window.clear()
        self.delivery_window_started_s = None
        self.delivery_last_new_frame_s = None
        self.delivery_inside_hits = 0
        self.delivery_visual_confirmed = False

    def _reset_delivery_evidence(self) -> None:
        self.delivery_outside_seen = False
        self.delivery_inside_hits = 0
        self.delivery_visual_confirmed = False
        self.delivery_last_frame_sequence = None
        self.delivery_window.clear()
        self.delivery_window_started_s = None
        self.delivery_last_new_frame_s = None
        self.delivery_observation_started_s = None
        self.delivery_observation_attempt = 0
        self.delivery_timeout_reason = ""
        self.delivery_conflict_frames = 0
        self.delivery_last_frame_age_ms = None

    def _delivery_window_misses(self) -> int:
        return len(self.delivery_window) - self.delivery_inside_hits

    def _delivery_observation_elapsed(self, now: float) -> float | None:
        if self.delivery_observation_started_s is None:
            return None
        return max(0.0, now - self.delivery_observation_started_s)

    def _reset_safe_zone_alignment(self) -> None:
        self.safe_zone_align_initial_ack = None
        self.safe_zone_align_tx_baseline = None
        self.safe_zone_align_command_accepted = False
        self.safe_zone_visual_align_initial_ack = None
        self.safe_zone_visual_align_tx_baseline = None
        self.safe_zone_visual_align_command_accepted = False
        self.staging_zero_initial_ack = None
        self.staging_zero_tx_baseline = None
        self.staging_zero_accepted = False
        self.staging_camera_mismatch_reported = False
        self.safe_zone_acquire_started_s = None
        self.safe_zone_freeze_frame_floor = None
        self.safe_zone_freeze_last_frame_sequence = None
        self.safe_zone_freeze_hits = 0
        self.safe_zone_freeze_boxes.clear()
        self.locked_safe_bbox = None
        self.locked_safe_target_x_px = None
        self.safe_zone_visual_pixel_error = 0
        self.safe_zone_visual_locked = False
        self.safe_zone_fallback = False
        self.safe_corridor_frame_floor = None
        self.safe_corridor_last_frame_sequence = None
        self.safe_sweep_attempts = 0
        self.safe_sweep_command = None
        self.safe_sweep_initial_ack = None
        self.safe_sweep_tx_baseline = None
        self.safe_sweep_command_accepted = False
        self.safe_sweep_execution_seen = False
        self.safe_sweep_reaudit_active = False

    def _safe_zone_pose_align_command(self) -> CommandRequest:
        return CommandRequest(
            CMD_ALIGN_SAFE_ZONE,
            self._side_flags() | CMD_USE_FINAL_HEADING,
            aux=round(self.settings.safe_heading_deg * 100.0) % 36000,
        )

    def _safe_zone_visual_align_command(self) -> CommandRequest:
        return CommandRequest(
            CMD_ALIGN_SAFE_ZONE,
            self._side_flags() | CMD_VISUAL_CORRECTION_VALID,
            max(-32768, min(32767, self.safe_zone_visual_pixel_error)),
            0,
        )

    def _begin_safe_corridor_check(
        self, vision: VisionSnapshot, now: float
    ) -> None:
        self.safe_corridor_frame_floor = (
            vision.frame_sequence if vision.frame_sequence > 0 else None
        )
        self.safe_corridor_last_frame_sequence = None
        self._set_state(CompetitionState.SAFE_ZONE_CORRIDOR_CHECK, now)

    def _start_safe_zone_clear(
        self, vision: VisionSnapshot, stm: StmSnapshot, now: float
    ) -> CompetitionOutput:
        distance_mm = vision.safe_corridor_obstacle_distance_mm
        assert distance_mm is not None
        destination = (
            self.confirmed_delivery_destination or
            (self.selected_batch.destination if self.selected_batch else None)
        )
        lateral_mm = -150 if destination == "injury" else 150
        self.safe_sweep_command = CommandRequest(
            CMD_CLEAR_SAFE_ZONE,
            self._side_flags(),
            max(80, min(600, int(distance_mm))),
            lateral_mm,
            0,
        )
        self.safe_sweep_initial_ack = stm.acknowledged_sequence
        self.safe_sweep_tx_baseline = stm.relay_mission_tx_frames
        self.safe_sweep_command_accepted = False
        self.safe_sweep_execution_seen = False
        self.safe_sweep_attempts += 1
        self._set_state(CompetitionState.CLEAR_SAFE_ZONE, now)
        return CompetitionOutput(
            self.state,
            self.safe_sweep_command,
            f"安全区推进走廊发现{vision.safe_corridor_obstacle_class}，执行第{self.safe_sweep_attempts}次扫障",
            self.selected_batch,
            event="safe_zone_clear_start",
            motion_expected=True,
            tx_policy="normal_command",
            reason="safe_zone_corridor_blocked",
            expected_stm_modes=(STM_MODE_SAFE_SWEEP,),
        )

    def _clear_for_boundary_recovery(
        self, vision: VisionSnapshot, now: float
    ) -> None:
        self._clear_pending_audit()
        self._clear_selected_batch()
        self._clear_carried_manifest()
        self._clear_cluster_context(reset_attempts=True)
        self.cargo_recheck_pending = False
        self.cargo_recheck_context = "none"
        self.audit_hits = 0
        self.audit_last_signature = None
        self.audit_last_frame_sequence = None
        self.audit_recheck_frame_floor = None
        self.audit_recheck_started_s = None
        self.audit_legal_candidate_pending = False
        self.audit_invalid_after_legal_hits = 0
        self.invalid_release_side = "both"
        self.invalid_release_final = False
        self.invalid_release_context = "none"
        self.last_selected_track_ids = ()
        self.target_last_seen_s = None
        self.target_last_center_px = None
        self.target_last_area_px = None
        self.target_last_class = None
        self.navigation_initial_ack = None
        self.grab_initial_ack = None
        self.grab_complete_confirmed = False
        self.enter_initial_ack = None
        self.enter_tx_baseline = None
        self.enter_command_accepted = False
        self.task_complete_initial_ack = None
        self.return_initial_ack = None
        self.return_relay_tx_baseline = None
        self.return_command_accepted = False
        self.return_search_frame_floor = None
        self.return_search_mode_seen = False
        self.safe_zone_exit_pending = False
        self.resume_state = None
        self._reset_motion_watch()
        self._reset_safe_zone_alignment()
        self._reset_delivery_evidence()
        self.search_epoch_frame_floor = (
            vision.frame_sequence if vision.frame_sequence > 0 else None
        )
        self._set_state(CompetitionState.BOUNDARY_RECOVERY, now)

    def _begin_safe_zone_pose_align(self, stm: StmSnapshot, now: float) -> None:
        self.safe_zone_align_initial_ack = (
            stm.acknowledged_sequence if stm.fresh else None
        )
        self.safe_zone_align_tx_baseline = stm.relay_mission_tx_frames
        self.safe_zone_align_command_accepted = False
        self._set_state(CompetitionState.ALIGN_SAFE_ZONE_BY_POSE, now)

    def _begin_safe_zone_acquire(
        self, vision: VisionSnapshot, now: float
    ) -> None:
        self.safe_zone_acquire_started_s = now
        self.safe_zone_freeze_frame_floor = (
            vision.frame_sequence if vision.frame_sequence > 0 else None
        )
        self.safe_zone_freeze_last_frame_sequence = None
        self.safe_zone_freeze_hits = 0
        self.safe_zone_freeze_boxes.clear()
        self.locked_safe_bbox = None
        self.locked_safe_target_x_px = None
        self.safe_zone_visual_pixel_error = 0
        self.safe_zone_visual_locked = False
        self.safe_zone_fallback = False
        self._set_state(CompetitionState.ACQUIRE_SAFE_ZONE, now)

    def _observe_safe_zone_freeze(
        self, vision: VisionSnapshot, now: float
    ) -> bool:
        if not self._vision_fresh(vision, now):
            return False
        if (
            self.safe_zone_freeze_frame_floor is not None and
            vision.frame_sequence <= self.safe_zone_freeze_frame_floor
        ):
            return False
        if vision.frame_sequence == self.safe_zone_freeze_last_frame_sequence:
            return False
        self.safe_zone_freeze_last_frame_sequence = vision.frame_sequence
        if vision.safe_bbox is None:
            self.safe_zone_freeze_hits = 0
            self.safe_zone_freeze_boxes.clear()
            return False
        self.safe_zone_freeze_boxes.append(vision.safe_bbox)
        self.safe_zone_freeze_hits = len(self.safe_zone_freeze_boxes)
        if self.safe_zone_freeze_hits < self.settings.safe_zone_freeze_frames:
            return False
        boxes = tuple(self.safe_zone_freeze_boxes)
        locked = tuple(
            round(median(box[index] for box in boxes))
            for index in range(4)
        )
        self.locked_safe_bbox = locked
        x, _, width, _ = locked
        fraction = (
            2.0 / 3.0
            if self.selected_batch is not None and
            self.selected_batch.destination == "injury"
            else 1.0 / 3.0
        )
        self.locked_safe_target_x_px = round(x + width * fraction)
        self.safe_zone_visual_pixel_error = (
            self.locked_safe_target_x_px - CAMERA_IMAGE_CENTER_X_PX
        )
        return True

    def _enter_safe_zone_command(self) -> CommandRequest:
        flags = self._side_flags() | CMD_DRIVE_STRAIGHT
        if self.safe_zone_visual_locked:
            flags |= CMD_VISUAL_CORRECTION_VALID
        else:
            flags |= CMD_USE_FINAL_HEADING
        return CommandRequest(
            CMD_ENTER_SAFE_ZONE,
            flags,
            0,
            0,
            (
                0
                if self.safe_zone_visual_locked else
                round(self.settings.safe_heading_deg * 100.0) % 36000
            ),
        )

    def _finish_delivery(
        self, stm: StmSnapshot, now: float, *, basis: str, message: str, event: str
    ) -> CompetitionOutput:
        batch = self.selected_batch
        self.delivery_completion_basis = basis
        self._clear_cluster_context(reset_attempts=True)
        self._set_state(CompetitionState.TASK_COMPLETE, now)
        self.task_complete_initial_ack = stm.acknowledged_sequence
        self.delivery_count += 1
        if (
            self.carried_has_green or
            (batch is not None and "green_supply" in batch.classes)
        ):
            self.first_common_delivered = True
        return CompetitionOutput(
            self.state,
            CommandRequest(CMD_TASK_COMPLETE, self._side_flags()),
            message,
            batch,
            event=event,
            tx_policy="normal_command",
            reason=basis,
        )

    def _delivery_timeout_output(
        self,
        stm: StmSnapshot,
        now: float,
    ) -> CompetitionOutput:
        if self.delivery_observation_attempt < self.settings.delivery_max_observations:
            self._start_delivery_observation(now, self.delivery_observation_attempt + 1)
            self.delivery_timeout_reason = "first_observation_timeout"
            return CompetitionOutput(
                self.state,
                self._enter_safe_zone_command(),
                "首次投送视觉观察超时，原地重新观察",
                self.selected_batch,
                event="delivery_reobserve_start",
                tx_policy="normal_command",
                reason="delivery_reobserve",
            )
        if stm.fresh and stm.mode in {
            STM_MODE_RAM_VERIFY,
            STM_MODE_EXIT_SAFE_ZONE,
            STM_MODE_FACE_FIELD_CENTER,
            STM_MODE_SEARCH,
        }:
            return self._finish_delivery(
                stm,
                now,
                basis="delivery_timeout_after_reobserve",
                message="投送视觉在有限观察窗口内未确认，按F407已完成投送进入下一阶段",
                event="delivery_timeout_complete",
            )
        first_warning = self.delivery_timeout_reason != "observation_wait_extended"
        self.delivery_timeout_reason = "observation_wait_extended"
        return CompetitionOutput(
            self.state,
            self._enter_safe_zone_command(),
            "等待F407进入投送完成状态，视觉未确认时不再重复推进",
            self.selected_batch,
            event="delivery_verify_wait_extended" if first_warning else "",
            tx_policy="normal_command",
            reason=self.delivery_timeout_reason,
        )

    def _target_point(self) -> tuple[float, float]:
        assert self.selected_batch is not None
        if self.selected_batch.destination == "stash":
            return self.settings.stash_point
        if self.confirmed_delivery_destination == "injury":
            return self.settings.injury_target_x_m, self.settings.safe_staging_y_m
        if self.confirmed_delivery_destination == "material":
            return self.settings.material_target_x_m, self.settings.safe_staging_y_m
        raise RuntimeError("formal delivery destination is not audit-confirmed")

    def _at_target(self, pose: PoseSnapshot, target: tuple[float, float]) -> bool:
        return (
            pose.valid and
            abs(pose.x_m - target[0]) <= self.settings.fence_lateral_tolerance_m and
            abs(pose.y_m - target[1]) <= self.settings.fence_axial_tolerance_m
        )

    def _delivery_lateral_aligned(self, pose: PoseSnapshot) -> bool:
        if (
            self.selected_batch is None or
            self.confirmed_delivery_destination is None or
            not pose.valid
        ):
            return False
        target_x, _ = self._target_point()
        return abs(pose.x_m - target_x) <= self.settings.fence_lateral_tolerance_m

    def _navigation_command(
        self,
        pose: PoseSnapshot,
        target: tuple[float, float],
        *,
        use_safe_zone_heading: bool = True,
        staging_only: bool = False,
    ) -> CommandRequest:
        if not pose.valid:
            return self._pause()
        dx, dy = target[0] - pose.x_m, target[1] - pose.y_m
        distance = math.hypot(dx, dy)
        heading = math.degrees(math.atan2(dy, dx)) % 360.0
        # Keep the geometric target bearing all the way to the staging point.
        # F407 uses it as the field-frame translation direction and keeps the
        # chassis heading separately, so near-zone lateral correction remains
        # possible for the omni base.
        flags = (
            self._side_flags() | CMD_DISTANCE_VALID | CMD_STAGE_ONLY
            if staging_only else
            self._side_flags() | CMD_DRIVE_STRAIGHT |
            CMD_USE_FINAL_HEADING | CMD_DISTANCE_VALID
        )
        return CommandRequest(
            CMD_NAVIGATE_WAYPOINT,
            flags,
            max(0, min(32767, round(distance * 1000.0))),
            0,
            round(heading * 100.0) % 36000,
        )

    def _stash_navigation_command(
        self, pose: PoseSnapshot, target: tuple[float, float]
    ) -> CommandRequest:
        return self._navigation_command(
            pose, target, use_safe_zone_heading=False
        )

    def _return_stash_output(
        self, pose: PoseSnapshot, stm: StmSnapshot, now: float
    ) -> CompetitionOutput:
        target = self.settings.stash_point
        if not self.stash_has_cargo:
            self._set_state(CompetitionState.SEARCH, now)
            return CompetitionOutput(
                self.state,
                self._hold(),
                "没有已确认的藏点物资，继续搜索",
                tx_policy="hold",
                reason="stash_empty_not_confirmed",
            )
        if not self._pose_fresh(pose):
            return CompetitionOutput(
                self.state,
                self._pause(),
                "藏点复查路线等待新鲜定位",
                tx_policy="pause",
                reason="stash_return_pose_stale",
            )
        distance = _distance((pose.x_m, pose.y_m), target)
        if distance > self.settings.return_zero_tolerance_m:
            self.stash_zero_tx_baseline = None
            self.stash_zero_initial_ack = None
            return CompetitionOutput(
                self.state,
                self._stash_navigation_command(pose, target),
                f"前往藏点复查，剩余{distance * 1000.0:.0f} mm",
                motion_expected=True,
                tx_policy="normal_command",
                reason="stash_return_navigation",
                expected_stm_modes=(STM_MODE_NAVIGATE,),
            )
        zero_command = replace(
            self._stash_navigation_command(pose, target),
            arg_a=0,
        )
        if self.stash_zero_tx_baseline is None:
            self.stash_zero_tx_baseline = stm.relay_mission_tx_frames
            self.stash_zero_initial_ack = stm.acknowledged_sequence
        if (
            self._fresh_mode_after(
                stm, STM_MODE_NAVIGATE, self.stash_zero_initial_ack
            ) and
            stm.distance_done and
            self._relay_sent_since(
                stm, zero_command, self.stash_zero_tx_baseline
            )
        ):
            self._set_state(CompetitionState.WAIT_STASH_SEARCH_HANDOFF, now)
            self.stash_handoff_hold_tx_baseline = stm.relay_mission_tx_frames
            self.stash_handoff_initial_ack = stm.acknowledged_sequence
            return self._stash_search_handoff_output(stm, now)
        return CompetitionOutput(
            self.state,
            zero_command,
            "藏点复查已到零距离，持续发送D=0等待本次mode=10完成",
            motion_expected=True,
            tx_policy="stash_return_zero",
            reason="stash_return_zero_pending",
            expected_stm_modes=(STM_MODE_NAVIGATE,),
        )

    def _stash_search_handoff_output(
        self, stm: StmSnapshot, now: float
    ) -> CompetitionOutput:
        hold_command = CommandRequest(CMD_HOLD, self._side_flags())
        if (
            self.stash_handoff_hold_tx_baseline is not None and
            self._relay_sent_since(
                stm, hold_command, self.stash_handoff_hold_tx_baseline
            ) and
            self._fresh_mode_after(
                stm, STM_MODE_SEARCH, self.stash_handoff_initial_ack
            )
        ):
            self.stash_checked = True
            self.stash_has_cargo = False
            self._set_state(CompetitionState.SEARCH, now)
            return CompetitionOutput(
                self.state,
                hold_command,
                "藏点复查已确认为空并完成SEARCH交接，继续搜索",
                event="stash_search_handoff_complete",
                tx_policy="hold",
                reason="stash_search_handoff_confirmed",
                expected_stm_modes=(STM_MODE_SEARCH,),
            )
        return CompetitionOutput(
            self.state,
            hold_command,
            "已确认藏点D=0完成，等待HOLD转发后新鲜mode=3",
            tx_policy="stash_search_handoff_hold",
            reason="stash_search_handoff_pending",
            expected_stm_modes=(STM_MODE_SEARCH,),
        )

    def _return_center_output(self, pose: PoseSnapshot, now: float, message: str) -> CompetitionOutput:
        if not pose.valid:
            return CompetitionOutput(
                self.state,
                self._pause(),
                "返中定位暂时无效，冻结当前阶段等待定位恢复",
                motion_expected=False,
                tx_policy="pause",
                reason="return_pose_stale",
            )
        distance = math.hypot(pose.x_m, pose.y_m)
        heading = math.degrees(math.atan2(-pose.y_m, -pose.x_m)) % 360.0
        remaining = 0.0 if distance <= self.settings.return_zero_tolerance_m else distance
        return CompetitionOutput(
            self.state,
            CommandRequest(
                CMD_RETURN_CENTER,
                self._side_flags() | (1 << 1) | (1 << 2) | (1 << 4),
                round(remaining * 1000.0),
                0,
                round(heading * 100.0) % 36000,
            ),
            f"{message}，剩余{remaining * 1000.0:.0f} mm，等待F407进入SEARCH",
            motion_expected=True,
        )

    def _begin_return(self, stm: StmSnapshot, now: float) -> None:
        self.return_initial_ack = (
            stm.acknowledged_sequence if stm.fresh else None
        )
        self.return_relay_tx_baseline = stm.relay_mission_tx_frames
        self.return_command_accepted = False
        self.return_search_frame_floor = None
        self.return_search_mode_seen = False
        self._set_state(CompetitionState.RETURN_CENTER, now)

    def _latch_return_command_acceptance(self, stm: StmSnapshot) -> None:
        if (
            not self.return_command_accepted and
            stm.fresh and
            self.return_initial_ack is not None and
            self.return_relay_tx_baseline is not None and
            stm.relay_mission_tx_frames > self.return_relay_tx_baseline and
            stm.relay_last_mission_command == CMD_RETURN_CENTER and
            stm.acknowledged_sequence != self.return_initial_ack
        ):
            self.return_command_accepted = True

    def _update_danger_event(self, vision: VisionSnapshot, now: float) -> None:
        if self.state == CompetitionState.DETOUR or not self._vision_fresh(vision, now):
            return
        if vision.danger_ahead:
            self.danger_clear_frames = 0
            return
        self.danger_clear_frames += 1
        if self.danger_clear_frames >= 2:
            self.danger_event_latched = False

    def _cluster_items(self, vision: VisionSnapshot) -> list[TrackedCargo]:
        cargo = [
            item for item in self._visible_cargo(vision)
            if not self.first_common_delivered or item.hits >= 2
        ]
        return [
            item for item in cargo
            if not self._is_isolated(item, cargo)
        ]

    @staticmethod
    def _items_bbox(
        items: Iterable[TrackedCargo],
    ) -> tuple[int, int, int, int] | None:
        values = tuple(items)
        if not values:
            return None
        left = min(item.bbox[0] for item in values)
        top = min(item.bbox[1] for item in values)
        right = max(item.bbox[0] + item.bbox[2] for item in values)
        bottom = max(item.bbox[1] + item.bbox[3] for item in values)
        return left, top, right - left, bottom - top

    def _cluster_members_for_target(
        self, vision: VisionSnapshot, target: TrackedCargo
    ) -> list[TrackedCargo]:
        cargo = [
            item for item in self._visible_cargo(vision)
            if not self.first_common_delivered or item.hits >= 2
        ]
        return [
            item for item in cargo
            if item.track_id == target.track_id or
            not self._is_isolated(target, (target, item))
        ]

    def _choose_cluster_target(
        self, vision: VisionSnapshot
    ) -> TrackedCargo | None:
        items = self._cluster_items(vision)
        candidates = [
            item for item in items
            if item.class_name in {"green_supply", "core_black", "injured_orange"}
        ]
        if not candidates:
            return None
        if not self.first_common_delivered:
            candidates = [
                item for item in candidates if item.class_name == "green_supply"
            ]
        else:
            material = [
                item for item in candidates if item.class_name in MATERIAL_CLASSES
            ]
            casualty = [
                item for item in candidates
                if item.class_name == "injured_orange"
            ]
            nearest_material = min(
                material,
                key=lambda item: (item.distance_m, -item.area_px, item.track_id),
                default=None,
            )
            nearest_casualty = min(
                casualty,
                key=lambda item: (item.distance_m, -item.area_px, item.track_id),
                default=None,
            )
            if (
                nearest_material is not None and
                (
                    nearest_casualty is None or
                    nearest_material.distance_m <=
                    self.settings.near_material_max_distance_m
                )
            ):
                return nearest_material
            return nearest_casualty
        return min(
            candidates,
            key=lambda item: (item.distance_m, -item.area_px, item.track_id),
            default=None,
        )

    def _cluster_target_for_vision(
        self, vision: VisionSnapshot
    ) -> TrackedCargo | None:
        visible = [
            item for item in self._visible_cargo(vision)
            if item.class_name == self.cluster_target_class
        ]
        exact = next(
            (
                item for item in visible
                if item.track_id == self.cluster_target_track_id
            ),
            None,
        )
        if exact is not None:
            self.cluster_target_bbox = exact.bbox
            return exact
        if not visible or self.cluster_target_bbox is None:
            return None
        old_x, old_y, old_width, old_height = self.cluster_target_bbox
        old_center = (old_x + old_width * 0.5, old_y + old_height * 0.5)
        old_area = max(1, old_width * old_height)

        def score(item: TrackedCargo) -> float:
            center_x, center_y = item.center_px
            center_distance = math.hypot(
                center_x - old_center[0], center_y - old_center[1]
            )
            scale = max(16.0, math.sqrt(old_area), math.sqrt(max(1, item.area_px)))
            return (
                center_distance / scale
                + abs(math.log(max(1, item.area_px) / old_area))
            )

        ranked = sorted(visible, key=lambda item: (score(item), -item.area_px))
        if len(ranked) > 1 and score(ranked[1]) - score(ranked[0]) < 0.35:
            return None
        target = ranked[0]
        self.cluster_target_track_id = target.track_id
        self.cluster_target_bbox = target.bbox
        if self.selected_batch is not None:
            self.selected_batch = replace(
                self.selected_batch,
                track_ids=(target.track_id,),
                classes=(target.class_name,),
            )
        return target

    def _same_cluster_target(self, target: TrackedCargo) -> bool:
        if self.cluster_target_class != target.class_name:
            return False
        if self.cluster_target_track_id == target.track_id:
            return True
        if self.cluster_target_bbox is None:
            return False
        old_x, old_y, old_width, old_height = self.cluster_target_bbox
        old_center_x = old_x + old_width * 0.5
        old_center_y = old_y + old_height * 0.5
        center_x, center_y = target.center_px
        center_distance = math.hypot(
            center_x - old_center_x, center_y - old_center_y
        )
        continuity_limit = max(
            40.0,
            float(old_width),
            float(old_height),
            float(target.bbox[2]),
            float(target.bbox[3]),
        )
        area_ratio = max(1, target.area_px) / max(1, old_width * old_height)
        return (
            center_distance <= continuity_limit and
            0.25 <= area_ratio <= 4.0
        )

    def _cluster_approach_command(
        self, vision: VisionSnapshot
    ) -> CommandRequest | None:
        target = self._cluster_target_for_vision(vision)
        if target is None:
            return None
        items = self._cluster_members_for_target(vision, target)
        if len(items) < 2:
            return None
        cluster_bbox = self._items_bbox(items)
        assert cluster_bbox is not None
        self.cluster_bbox = cluster_bbox
        left, top, width, height = cluster_bbox
        return CommandRequest(
            CMD_APPROACH_TARGET,
            CMD_VALID | CMD_CLUSTER_TARGET,
            left + width // 2,
            top + height // 2,
        )

    @staticmethod
    def _relay_sent_cluster_since(
        stm: StmSnapshot, baseline: int | None
    ) -> bool:
        payload = stm.relay_last_mission_payload
        return (
            stm.fresh and
            baseline is not None and
            stm.relay_mission_tx_frames > baseline and
            stm.relay_last_mission_command == CMD_APPROACH_TARGET and
            len(payload) >= 2 and
            bool(payload[1] & CMD_CLUSTER_TARGET)
        )

    def _start_cluster_approach(
        self,
        vision: VisionSnapshot,
        stm: StmSnapshot,
        now: float,
        *,
        preserve_cluster: bool = False,
    ) -> CompetitionOutput:
        if not preserve_cluster:
            target = self._choose_cluster_target(vision)
            if target is None:
                return CompetitionOutput(
                    self.state,
                    self._hold(),
                    "聚集区域内没有可取得的合法目标，保持SEARCH",
                    tx_policy="hold",
                    reason="cluster_target_unavailable",
                )
            same_cluster = self._same_cluster_target(target)
            previous_attempts = self.disperse_attempts if same_cluster else 0
            previous_cluster_id = self.cluster_id
            if same_cluster and previous_attempts >= self.settings.disperse_limit:
                return CompetitionOutput(
                    self.state,
                    self._hold(),
                    "当前聚集目标已完成2次撞分，不再重复启动",
                    tx_policy="hold",
                    reason="cluster_disperse_limit_reached",
                )
            self._clear_cluster_context(reset_attempts=True)
            self.disperse_attempts = previous_attempts
            self.cluster_id = (
                previous_cluster_id
                if same_cluster else previous_cluster_id + 1
            )
            self.cluster_target_class = target.class_name
            self.cluster_target_track_id = target.track_id
            self.cluster_target_bbox = target.bbox
            destination = (
                "injury" if target.class_name == "injured_orange" else "material"
            )
            self._select_batch(
                CargoBatch(
                    (target.track_id,),
                    (target.class_name,),
                    destination,
                )
            )
            self._mark_target_seen(target, now)
            self.cluster_signature = (
                self.cluster_id,
                self.cluster_target_class,
            )
        command = self._cluster_approach_command(vision)
        if command is None:
            return CompetitionOutput(
                self.state,
                self._hold(),
                "锁定目标当前无法确认所属聚集区域，保持当前状态重新观察",
                self.selected_batch,
                tx_policy="hold",
                reason="cluster_signature_unavailable",
            )
        self._set_state(CompetitionState.CLUSTER_APPROACH, now)
        self.cluster_command = command
        self.cluster_initial_ack = stm.acknowledged_sequence
        self.cluster_relay_tx_baseline = stm.relay_mission_tx_frames
        self.cluster_command_accepted = False
        self.cluster_execution_seen = False
        self.cluster_keep_side = None
        return CompetitionOutput(
            self.state,
            command,
            "聚集目标已锁定，持续靠近并等待F407上报mode=38夹内观察",
            event="cluster_approach_start",
            motion_expected=True,
            tx_policy="cluster_approach",
            reason="cluster_target_locked",
            expected_stm_modes=(
                STM_MODE_APPROACH_TARGET,
                STM_MODE_CLUSTER_CAPTURE_AUDIT,
            ),
        )

    def _cluster_approach_output(
        self, vision: VisionSnapshot, stm: StmSnapshot, now: float
    ) -> CompetitionOutput:
        if stm.fresh and stm.mode in {
            STM_MODE_APPROACH_TARGET,
            STM_MODE_CLUSTER_CAPTURE_AUDIT,
        }:
            self.cluster_execution_seen = True
        if (
            not self.cluster_command_accepted and
            stm.fresh and
            self.cluster_initial_ack is not None and
            self._relay_sent_cluster_since(stm, self.cluster_relay_tx_baseline) and
            stm.acknowledged_sequence != self.cluster_initial_ack
        ):
            self.cluster_command_accepted = True
        if (
            stm.fresh and
            stm.mode in {
            STM_MODE_APPROACH_RECOVER,
            STM_MODE_SEARCH,
            } and
            (self.cluster_command_accepted or self.cluster_execution_seen)
        ):
            return self._audit_reacquire_output(vision, stm, now)
        if (
            stm.fresh and
            stm.mode == STM_MODE_CLUSTER_CAPTURE_AUDIT and
            stm.claw_visible and
            stm.camera_pitch_cdeg == 14000
        ):
            self.cluster_command_accepted = True
            self._begin_capture_audit(
                vision,
                now,
                cluster_active=True,
                recheck_pending=False,
                recheck_context="none",
            )
            return self._audit_output(vision, stm, now)
        if self._vision_fresh(vision, now):
            command = self._cluster_approach_command(vision)
            if command is not None:
                self.cluster_command = command
        assert self.cluster_command is not None
        return CompetitionOutput(
            self.state,
            self.cluster_command,
            "持续发送CLUSTER_TARGET靠近聚集区域，等待F407 mode=38夹内观察",
            motion_expected=True,
            tx_policy="cluster_approach",
            reason="cluster_capture_audit_pending",
            expected_stm_modes=(
                STM_MODE_APPROACH_TARGET,
                STM_MODE_CLUSTER_CAPTURE_AUDIT,
                STM_MODE_APPROACH_RECOVER,
                STM_MODE_SEARCH,
            ),
        )

    def _start_disperse(self, stm: StmSnapshot, now: float) -> CompetitionOutput:
        self._set_state(CompetitionState.DISPERSE, now)
        self._arm_disperse(stm, self.cluster_keep_side)
        return CompetitionOutput(
            self.state,
            self.disperse_command,
            (
                f"锁定保留{self.cluster_keep_side}侧目标，启动选择性分离"
                if self.cluster_keep_side is not None else
                "目标左右归属不明确，启动20°观察转向"
            ),
            event="disperse_start",
            motion_expected=True,
            tx_policy="disperse_command",
            reason="disperse_permission_locked",
            expected_stm_modes=(self.disperse_expected_done_mode,),
        )

    def _start_first_green_bump(
        self, stm: StmSnapshot, now: float
    ) -> CompetitionOutput:
        self.first_green_bump_used = True
        self._set_state(CompetitionState.DISPERSE, now)
        self._arm_disperse(stm, None, first_green_bump=True)
        return CompetitionOutput(
            self.state,
            self.disperse_command,
            "首件绿色位于中心混堆，启动一次轻撞后重新SEARCH",
            self.selected_batch,
            event="first_green_bump_start",
            motion_expected=True,
            tx_policy="disperse_command",
            reason="first_green_bump_locked",
            expected_stm_modes=(STM_MODE_REMOTE_ACTION, STM_MODE_SEARCH),
        )

    def _disperse_output(
        self, vision: VisionSnapshot, stm: StmSnapshot, now: float
    ) -> CompetitionOutput:
        assert self.disperse_command is not None
        self.disperse_command_accepted = self._command_acceptance_seen(
            stm,
            self.disperse_command,
            self.disperse_relay_tx_baseline,
            self.disperse_initial_ack,
            self.disperse_command_accepted,
        )
        if not self.disperse_command_accepted:
            legal_output = self._resume_legal_audit_before_disperse(
                vision, stm, now
            )
            if legal_output is not None:
                return legal_output
        if (
            self.disperse_context == "first_green_bump" and
            stm.fresh and
            stm.mode == STM_MODE_SEARCH and
            self.disperse_command_accepted
        ):
            self._clear_pending_audit()
            self._clear_selected_batch()
            self._clear_cluster_context(reset_attempts=True)
            self.cargo_recheck_pending = False
            self.cargo_recheck_context = "none"
            self.audit_hits = 0
            self.audit_last_signature = None
            self.audit_last_frame_sequence = None
            self.audit_recheck_frame_floor = None
            self.audit_recheck_started_s = None
            self.last_selected_track_ids = ()
            self.target_last_seen_s = None
            self.target_last_center_px = None
            self.target_last_area_px = None
            self.target_last_class = None
            self.search_epoch_frame_floor = (
                vision.frame_sequence if vision.frame_sequence > 0 else None
            )
            self._set_state(CompetitionState.SEARCH, now)
            self.search_epoch_frame_floor = (
                vision.frame_sequence if vision.frame_sequence > 0 else None
            )
            return CompetitionOutput(
                self.state,
                self._hold(),
                "首件绿色轻撞完成并回mode3，等待动作后的新帧重新找绿色",
                event="first_green_bump_done",
                tx_policy="hold",
                reason="first_green_bump_search_reset",
                expected_stm_modes=(STM_MODE_SEARCH,),
            )
        if (
            stm.fresh and
            stm.mode == STM_MODE_DISPERSE_DONE and
            stm.claw_visible and
            stm.camera_pitch_cdeg == 14000 and
            self.disperse_command_accepted
        ):
            if self.disperse_context == "observe":
                self.disperse_observe_attempts += 1
            self._begin_capture_audit(
                vision,
                now,
                cluster_active=False,
                recheck_pending=True,
                recheck_context=(
                    "disperse_observe"
                    if self.disperse_context == "observe" else
                    "disperse_selective"
                ),
            )
            return self._audit_output(vision, stm, now)
        return CompetitionOutput(
            self.state,
            self.disperse_command,
            (
                "首件绿色轻撞执行中，持续发送固定命令等待新鲜mode3"
                if self.disperse_context == "first_green_bump" else
                "打散执行中；视觉暂失或画面模糊不改变本次固定动作"
            ),
            motion_expected=True,
            tx_policy="disperse_command",
            reason="disperse_visual_not_required_during_action",
            expected_stm_modes=self.expected_stm_modes(),
        )

    @staticmethod
    def _fresh_mode_after(
        stm: StmSnapshot, expected_mode: int, initial_ack: int | None
    ) -> bool:
        return (
            stm.fresh and
            stm.mode == expected_mode and
            initial_ack is not None and
            stm.acknowledged_sequence != initial_ack
        )

    def _yield_command(self) -> CommandRequest:
        return CommandRequest(
            CMD_YIELD_BACKOFF,
            CMD_VALID,
            -round(self.settings.yield_distance_m * 1000.0),
            0,
        )

    def _escape_command(self) -> CommandRequest:
        return CommandRequest(
            CMD_ESCAPE_MANEUVER,
            CMD_VALID,
            self.settings.escape_spin_deg,
            round(self.settings.escape_lateral_m * 1000.0),
        )

    def _reset_motion_watch(self) -> None:
        self.motion_watch_state = None
        self.motion_watch_started_s = None
        self.motion_watch_pose = None
        self.motion_watch_wheel_m = None

    def _motion_command_for_state(self) -> int | None:
        if self.state in {CompetitionState.INITIAL_APPROACH, CompetitionState.APPROACH}:
            return CMD_APPROACH_TARGET
        if self.state in {
            CompetitionState.INITIAL_STASH_NAV,
            CompetitionState.NAVIGATE,
            CompetitionState.RETURN_STASH,
        }:
            return CMD_NAVIGATE_WAYPOINT
        if self.state == CompetitionState.RETURN_CENTER:
            return CMD_RETURN_CENTER
        return None

    def _stuck_recovery_output(
        self, pose: PoseSnapshot, stm: StmSnapshot, now: float
    ) -> CompetitionOutput | None:
        if self.state in {
            CompetitionState.FIELD_STUCK_ESCAPE,
            CompetitionState.SAFE_ZONE_ESCAPE,
        }:
            escape_command = self._escape_command()
            self.stuck_command_accepted = self._command_acceptance_seen(
                stm,
                escape_command,
                self.stuck_tx_baseline,
                self.stuck_initial_ack,
                self.stuck_command_accepted,
            )
            if (
                stm.fresh and
                stm.mode == STM_MODE_ESCAPE_DONE and
                self.stuck_command_accepted
            ):
                resume = self.stuck_resume_state or CompetitionState.RETURN_CENTER
                if resume == CompetitionState.RETURN_CENTER:
                    self._begin_return(stm, now)
                else:
                    self._set_state(resume, now)
                self._reset_motion_watch()
                return None
            return CompetitionOutput(
                self.state,
                escape_command,
                (
                    "安全区退出长期无进展，等待F407抬起机构并完成脱困"
                    if self.state == CompetitionState.SAFE_ZONE_ESCAPE else
                    "场内退让后仍无进展，等待F407完成脱困"
                ),
                self.selected_batch,
                motion_expected=True,
                stuck_phase="safe_zone_escape" if self.state == CompetitionState.SAFE_ZONE_ESCAPE else "escape",
                expected_stm_modes=(STM_MODE_ESCAPE_DONE,),
            )

        if (
            self.state in {
                CompetitionState.INITIAL_STASH_NAV,
                CompetitionState.NAVIGATE,
            } and
            self.selected_batch is not None and
            self._pose_fresh(pose) and
            _distance((pose.x_m, pose.y_m), self._target_point()) <= 0.30
        ):
            self._reset_motion_watch()
            return None
        if (
            self.state == CompetitionState.RETURN_STASH and
            self._pose_fresh(pose) and
            _distance((pose.x_m, pose.y_m), self.settings.stash_point) <= 0.30
        ):
            self._reset_motion_watch()
            return None

        command = self._motion_command_for_state()
        expected_mode = {
            CMD_APPROACH_TARGET: STM_MODE_APPROACH_TARGET,
            CMD_NAVIGATE_WAYPOINT: STM_MODE_NAVIGATE,
            CMD_RETURN_CENTER: STM_MODE_FACE_FIELD_CENTER,
        }.get(command)
        active = (
            command is not None and
            expected_mode is not None and
            self._pose_fresh(pose) and
            stm.fresh and
            not stm.fault and
            stm.mode == expected_mode and
            stm.motors_active and
            stm.relay_last_mission_command == command
        )
        if not active:
            self._reset_motion_watch()
            return None
        current_pose = (pose.x_m, pose.y_m, pose.yaw_deg)
        if self.motion_watch_state != self.state or self.motion_watch_pose is None:
            self.motion_watch_state = self.state
            self.motion_watch_started_s = now
            self.motion_watch_pose = current_pose
            self.motion_watch_wheel_m = pose.wheel_progress_m
            return None
        anchor_x, anchor_y, anchor_yaw = self.motion_watch_pose
        translated = math.hypot(pose.x_m - anchor_x, pose.y_m - anchor_y)
        rotated = abs(angle_error_deg(pose.yaw_deg, anchor_yaw))
        wheel_progress = 0.0
        if pose.wheel_progress_m is not None and self.motion_watch_wheel_m is not None:
            wheel_progress = abs(pose.wheel_progress_m - self.motion_watch_wheel_m)
        progressed = (
            translated >= self.settings.stuck_translation_m or
            rotated >= self.settings.stuck_yaw_deg or
            wheel_progress >= self.settings.stuck_wheel_progress_m
        )
        if progressed:
            self.stuck_recovery_level = 0
            if self.safe_zone_exit_pending and translated >= 0.20:
                self.safe_zone_exit_pending = False
            self.motion_watch_started_s = now
            self.motion_watch_pose = current_pose
            self.motion_watch_wheel_m = pose.wheel_progress_m
            return None
        if (
            self.motion_watch_started_s is None or
            now - self.motion_watch_started_s < self.settings.stuck_observation_s
        ):
            return None

        self.stuck_resume_state = self.state
        self.stuck_initial_ack = stm.acknowledged_sequence
        self.stuck_tx_baseline = stm.relay_mission_tx_frames
        self.stuck_command_accepted = False
        self._reset_motion_watch()
        if self.state == CompetitionState.RETURN_CENTER and self.safe_zone_exit_pending:
            self._set_state(CompetitionState.SAFE_ZONE_ESCAPE, now)
            return CompetitionOutput(
                self.state,
                self._escape_command(),
                "确认安全区退出长期无进展，申请抬起机构后脱困",
                self.selected_batch,
                event="safe_zone_exit_stuck",
                motion_expected=True,
                stuck_phase="safe_zone_escape",
                expected_stm_modes=(STM_MODE_ESCAPE_DONE,),
            )
        self.stuck_recovery_level = 0
        self._set_state(CompetitionState.FIELD_STUCK_ESCAPE, now)
        return CompetitionOutput(
            self.state,
            self._escape_command(),
            "确认场内长期无进展，申请旋转横移脱困",
            self.selected_batch,
            event="field_stuck_escape",
            motion_expected=True,
            stuck_phase="escape",
            expected_stm_modes=(STM_MODE_ESCAPE_DONE,),
        )

    def _navigate(
        self,
        vision: VisionSnapshot,
        pose: PoseSnapshot,
        stm: StmSnapshot,
        now: float,
    ) -> CompetitionOutput:
        if self.selected_batch is None:
            self._set_state(CompetitionState.SEARCH, now)
            return CompetitionOutput(self.state, self._hold(), "没有锁定批次，返回搜索")
        target = self._target_point()
        navigation_accepted = self._fresh_mode_after(
            stm, STM_MODE_NAVIGATE, self.navigation_initial_ack
        )
        if self.selected_batch.destination != "stash":
            staging_command = self._navigation_command(
                pose, target, staging_only=True
            )
            if (
                self._at_target(pose, target) or
                self.staging_zero_tx_baseline is not None
            ):
                if self.staging_zero_tx_baseline is None:
                    self.staging_zero_tx_baseline = stm.relay_mission_tx_frames
                    self.staging_zero_initial_ack = stm.acknowledged_sequence
                zero_command = replace(staging_command, arg_a=0)
                if (
                    not self.staging_zero_accepted and
                    stm.fresh and
                    stm.mode == STM_MODE_NAVIGATE and
                    stm.distance_done and
                    stm.gripper_closed
                ):
                    self.staging_zero_accepted = True
                staging_done = (
                    stm.fresh and
                    stm.mode == STM_MODE_NAVIGATE and
                    stm.distance_done and
                    stm.gripper_closed and
                    self.staging_zero_accepted
                )
                camera_mismatch = (
                    stm.fresh and
                    stm.mode == STM_MODE_NAVIGATE and
                    stm.distance_done and
                    stm.camera_pitch_cdeg == 14000
                )
                if not staging_done:
                    event = ""
                    if camera_mismatch and not self.staging_camera_mismatch_reported:
                        self.staging_camera_mismatch_reported = True
                        event = "staging_camera_firmware_mismatch"
                    return CompetitionOutput(
                        self.state,
                        zero_command,
                        (
                            "mode10和DISTANCE_DONE已成立但摄像头仍为140°，"
                            "下位机固件版本与最新STAGE流程不匹配"
                            if camera_mismatch else
                            "已进入60 cm预备点容差，持续发送STAGE NAV D=0等待F407确认"
                        ),
                        self.selected_batch,
                        event=event,
                        motion_expected=True,
                        tx_policy="staging_zero",
                        reason="safe_zone_staging_zero_pending",
                        expected_stm_modes=(STM_MODE_NAVIGATE,),
                    )
                mismatch_event = ""
                if camera_mismatch and not self.staging_camera_mismatch_reported:
                    self.staging_camera_mismatch_reported = True
                    mismatch_event = "staging_camera_firmware_mismatch"
                self._begin_safe_zone_pose_align(stm, now)
                return CompetitionOutput(
                    self.state,
                    self._safe_zone_pose_align_command(),
                    (
                        "STAGE完成但摄像头仍为140°，下位机固件版本不匹配；"
                        "继续发送定位ALIGN"
                        if camera_mismatch else
                        "到达安全区半区前60 cm预备点，按定位正方向对准安全区"
                    ),
                    self.selected_batch,
                    event=mismatch_event or "safe_zone_staging_arrived",
                    tx_policy="normal_command",
                    reason="safe_zone_pose_align_start",
                    expected_stm_modes=(
                        STM_MODE_NAVIGATE,
                        STM_MODE_ALIGN_SAFE_ZONE,
                    ),
                )
            return CompetitionOutput(
                self.state,
                staging_command,
                "前往对应安全区半区围栏前600 mm预备点",
                self.selected_batch,
                motion_expected=True,
                tx_policy="normal_command",
                reason="safe_zone_staging_navigation",
                expected_stm_modes=(STM_MODE_NAVIGATE,),
            )
        else:
            reached = navigation_accepted and (
                self._at_target(pose, target) or
                (
                    stm.mode == STM_MODE_NAVIGATE and
                    stm.distance_done
                )
            )
            if reached:
                if not self._vision_fresh(vision, now):
                    return CompetitionOutput(
                        self.state,
                        self._pause(),
                        "到达藏点但视觉暂时过期，等待新鲜帧后再释放",
                        self.selected_batch,
                        tx_policy="pause",
                        reason="initial_stash_release_vision_stale",
                        expected_stm_modes=(STM_MODE_NAVIGATE,),
                    )
                self._set_state(CompetitionState.INITIAL_RELEASE, now)
                self._arm_initial_release(stm, now)
                return CompetitionOutput(self.state, CommandRequest(CMD_RELEASE_BOTH, self._side_flags()), "到达临时藏物资点，释放整批物资", self.selected_batch, event="stash_arrived")
        return CompetitionOutput(
            self.state,
            self._stash_navigation_command(pose, target),
            "前往临时物资点，持续更新剩余距离",
            self.selected_batch,
            motion_expected=True,
        )

    def _handle_search(
        self,
        vision: VisionSnapshot,
        pose: PoseSnapshot,
        stm: StmSnapshot,
        now: float,
    ) -> CompetitionOutput:
        if self.return_search_mode_seen:
            ready = (
                stm.fresh and
                stm.mode == STM_MODE_SEARCH and
                stm.camera_pitch_cdeg == 12000 and
                self._vision_fresh(vision, now) and
                self.return_search_frame_floor is not None and
                vision.frame_sequence > self.return_search_frame_floor
            )
            if not ready:
                return CompetitionOutput(
                    self.state,
                    self._hold(),
                    "返中完成后等待相机120°和新的SEARCH视觉帧",
                    tx_policy="hold",
                    reason="return_search_frame_gate",
                    expected_stm_modes=(STM_MODE_SEARCH,),
                )
            self.return_search_mode_seen = False
            self.return_search_frame_floor = None
        self._update_search_scan_progress(pose)
        if not self._vision_fresh(vision, now):
            return CompetitionOutput(
                self.state,
                self._hold(),
                "视觉结果暂时过期，暂停新目标决策并维持SEARCH心跳",
                event="search_candidate_rejected",
                tx_policy="hold",
                reason="STALE",
            )
        if not self.first_common_delivered:
            visible = self._visible_cargo(vision)
            visible_green = [
                item for item in visible
                if item.class_name == "green_supply"
            ]
            candidate = self._choose_initial_green(vision)
            if candidate is not None:
                if not self._observe_search_candidate(
                    ("green", candidate.class_name), vision
                ):
                    return CompetitionOutput(
                        self.state,
                        self._hold(),
                        (
                            "首件绿色物资已出现，等待连续"
                            f"{self.settings.search_target_confirm_frames}帧确认"
                        ),
                        tx_policy="hold",
                        reason="search_target_stabilizing",
                    )
                self._select_batch(CargoBatch((candidate.track_id,), ("green_supply",), "material"))
                self._reset_delivery_evidence()
                self._mark_target_seen(candidate, now)
                self._set_state(CompetitionState.APPROACH, now)
                self._arm_approach(stm)
                return CompetitionOutput(self.state, self._approach_command(candidate), "锁定首件单独普通物资", self.selected_batch, event="first_green_locked")
            if visible_green:
                if not self._observe_search_candidate(("green_cluster",), vision):
                    return CompetitionOutput(
                        self.state,
                        self._hold(),
                        (
                            "绿色物资堆已出现，等待连续"
                            f"{self.settings.search_target_confirm_frames}帧确认"
                        ),
                        tx_policy="hold",
                        reason="search_target_stabilizing",
                    )
                if (
                    stm.fresh and
                    stm.mode == STM_MODE_SEARCH and
                    not stm.gripper_closed and
                    not self.cargo_recheck_pending
                ):
                    return self._start_cluster_approach(vision, stm, now)
                return CompetitionOutput(
                    self.state,
                    self._hold(),
                    "绿色已识别为聚集目标，等待F407空爪SEARCH接受CLUSTER_APPROACH",
                    event="search_candidate_rejected",
                    tx_policy="hold",
                    reason="CLUSTER",
                )
            self._observe_search_candidate(None, vision)
        else:
            if self._mixed_pile_requires_disperse(vision):
                if not self._observe_search_candidate(("mixed_pile",), vision):
                    return CompetitionOutput(
                        self.state,
                        self._hold(),
                        (
                            "混合物资堆已出现，等待连续"
                            f"{self.settings.search_target_confirm_frames}帧确认"
                        ),
                        tx_policy="hold",
                        reason="search_target_stabilizing",
                    )
                if (
                    stm.mode == STM_MODE_SEARCH and
                    not stm.gripper_closed and
                    not self.cargo_recheck_pending
                ):
                    return self._start_cluster_approach(vision, stm, now)
                return CompetitionOutput(
                    self.state,
                    self._hold(),
                    "检测到不同投送区或危险物资混堆，等待空爪打散",
                    tx_policy="hold",
                    reason="mixed_pile_wait_disperse",
                )
            batch = self._choose_next_batch(vision)
            if batch is not None:
                if not self._observe_search_candidate(
                    (
                        "batch",
                        batch.destination,
                        tuple(sorted(batch.classes)),
                    ),
                    vision,
                ):
                    return CompetitionOutput(
                        self.state,
                        self._hold(),
                        (
                            "候选批次已出现，等待连续"
                            f"{self.settings.search_target_confirm_frames}帧确认"
                        ),
                        tx_policy="hold",
                        reason="search_target_stabilizing",
                    )
                self.stash_checked = False
                self._select_batch(batch)
                self._reset_delivery_evidence()
                self.last_selected_track_ids = batch.track_ids
                self._set_state(CompetitionState.APPROACH, now)
                self._arm_approach(stm)
                candidate = self._target_for_batch(vision)
                if candidate is not None:
                    self._mark_target_seen(candidate, now)
                return CompetitionOutput(self.state, self._approach_command(candidate), f"锁定{batch.total_count}件{batch.destination}批次", batch, event="batch_locked")
            self._observe_search_candidate(None, vision)
        if self.search_yaw_accum_deg >= self.settings.search_min_turn_deg:
            if not (
                stm.fresh and
                stm.mode == STM_MODE_SEARCH and
                not stm.gripper_closed
            ):
                return CompetitionOutput(
                    self.state,
                    self._hold(),
                    "两层搜索已完成，等待新鲜mode3和空爪后返场地中心",
                    tx_policy="hold",
                    reason="search_return_center_gate",
                    expected_stm_modes=(STM_MODE_SEARCH,),
                )
            self._begin_return(stm, now)
            return self._return_center_output(
                pose,
                now,
                "120°和90°两圈未找到合法目标，返回场地中心重启搜索",
            )
        reject_reasons: list[str] = []
        if vision.low_conf_green_seen:
            reject_reasons.append("LOW_CONF")
        if any(item.visible and item.inside_safe_zone for item in vision.cargo):
            reject_reasons.append("INSIDE_SAFE")
        if any(item.visible and item.hits < 2 for item in vision.cargo):
            reject_reasons.append("TRACK")
        if reject_reasons:
            return CompetitionOutput(
                self.state,
                self._hold(),
                f"候选未建立：{'|'.join(reject_reasons)}",
                event="search_candidate_rejected",
                tx_policy="hold",
                reason="|".join(reject_reasons),
            )
        return CompetitionOutput(
            self.state,
            self._hold(),
            "搜索并锁定合法目标",
            reason="NO_CANDIDATE",
        )

    def _approach_command(self, candidate: TrackedCargo | None) -> CommandRequest:
        if candidate is None:
            return self._pause()
        x, y = candidate.center_px
        return CommandRequest(CMD_APPROACH_TARGET, self._side_flags(), x, y)

    def step(
        self,
        vision: VisionSnapshot,
        pose: PoseSnapshot,
        stm: StmSnapshot,
        now: float,
    ) -> CompetitionOutput:
        if self.state_started_s == 0.0:
            self.state_started_s = now
        self._update_vision_frame_period(vision)
        self._update_delivery_observation(vision, now)
        self._update_danger_event(vision, now)
        if self.state == CompetitionState.FAULT:
            return CompetitionOutput(
                self.state,
                CommandRequest(CMD_ABORT, self._side_flags()),
                "任务故障，发送ABORT安全停车",
                tx_policy="fault_abort",
                reason="mission_fault_abort",
            )
        if self.state == CompetitionState.FINISHED:
            return CompetitionOutput(
                self.state,
                self._pause(),
                "任务完成，冻结当前阶段等待用户结束",
                tx_policy="pause",
                reason="mission_finished_pause",
            )
        if not stm.fresh:
            if self.state == CompetitionState.WAIT_START:
                return CompetitionOutput(
                    self.state,
                    None,
                    "启动自主阶段等待STM状态恢复，不覆盖F407自主出发",
                    suppress_command_tx=True,
                    suppression_reason="f407_autonomous_start",
                    tx_policy="autonomous_start",
                    reason="f407_autonomous_start",
                    expected_stm_modes=(STM_MODE_SEARCH,),
                )
            return CompetitionOutput(
                self.state,
                self._pause(),
                "STM状态失联，持续发送PAUSE冻结当前阶段；状态恢复后重发当前阶段合法命令",
                self.selected_batch,
                tx_policy="pause",
                reason="stm_status_stale_pause",
                expected_stm_modes=self.expected_stm_modes(),
            )
        if stm.fault:
            if self.first_fault_code is None:
                self.first_fault_code = stm.fault_code
            event = "stm_fault_wait" if not self.stm_fault_waiting else ""
            self.stm_fault_waiting = True
            return CompetitionOutput(
                self.state,
                None,
                f"F407报告故障码{stm.fault_code}，保留任务状态等待下位机恢复",
                self.selected_batch,
                event=event,
                suppress_command_tx=True,
                suppression_reason="f407_fault_wait",
                tx_policy="f407_fault_wait",
                reason="f407_fault_wait",
                expected_stm_modes=self.expected_stm_modes(),
            )
        self.stm_fault_waiting = False

        if stm.mode == STM_MODE_BOUNDARY_RECOVER:
            if self.state != CompetitionState.BOUNDARY_RECOVERY:
                self._clear_for_boundary_recovery(vision, now)
            return CompetitionOutput(
                self.state,
                self._hold(),
                "F407已触发边界恢复，清除旧任务并等待张爪、转向中心、驶入距边至少400 mm后回SEARCH",
                event="f407_boundary_recovery",
                tx_policy="hold",
                reason="f407_boundary_recovery",
                expected_stm_modes=(STM_MODE_BOUNDARY_RECOVER, STM_MODE_SEARCH),
            )
        if self.state == CompetitionState.BOUNDARY_RECOVERY:
            if stm.mode == STM_MODE_SEARCH:
                self._set_state(CompetitionState.SEARCH, now)
                self.search_epoch_frame_floor = (
                    vision.frame_sequence if vision.frame_sequence > 0 else None
                )
                return CompetitionOutput(
                    self.state,
                    self._hold(),
                    "F407边界转向完成，从动作后的新视觉帧重新SEARCH",
                    event="f407_boundary_recovery_done",
                    tx_policy="hold",
                    reason="f407_boundary_recovery_done",
                    expected_stm_modes=(STM_MODE_SEARCH,),
                )
            return CompetitionOutput(
                self.state,
                self._hold(),
                "等待F407完成边界恢复并驶入距边至少400 mm后回mode3",
                tx_policy="hold",
                reason="f407_boundary_recovery_wait",
                expected_stm_modes=(STM_MODE_BOUNDARY_RECOVER, STM_MODE_SEARCH),
            )

        stuck_output = self._stuck_recovery_output(pose, stm, now)
        if stuck_output is not None:
            return stuck_output

        if self.state == CompetitionState.WAIT_START:
            if self._start_ready(pose, stm):
                self._set_state(CompetitionState.INITIAL_OBSERVE if not self.initial_stash_done else CompetitionState.SEARCH, now)
                return CompetitionOutput(self.state, self._hold(), "已离开出发区，开始中心观察", event="start_clear")
            return CompetitionOutput(
                self.state,
                None,
                "等待下位机完成出发并离开出发区",
                suppress_command_tx=True,
                suppression_reason="f407_autonomous_start",
                tx_policy="autonomous_start",
                reason="f407_autonomous_start",
                expected_stm_modes=(STM_MODE_SEARCH,),
            )

        if self.state == CompetitionState.INITIAL_OBSERVE:
            pile = self._pile_batch(vision) if self._vision_fresh(vision, now) else None
            if pile is not None:
                self._select_batch(pile)
                self._reset_delivery_evidence()
                self.last_selected_track_ids = pile.track_ids
                self._set_state(CompetitionState.INITIAL_APPROACH, now)
                self._arm_approach(stm)
                candidate = self._target_for_batch(vision)
                if candidate is not None:
                    self._mark_target_seen(candidate, now)
                return CompetitionOutput(self.state, self._approach_command(candidate), "锁定开局物资，统一执行临时藏物资", pile, event="initial_stash_locked")
            return CompetitionOutput(
                self.state,
                self._hold(),
                "等待稳定开局物资并保持F407本地SEARCH扫描",
                tx_policy="hold",
                reason="initial_stash_target_wait",
                expected_stm_modes=(STM_MODE_SEARCH,),
            )

        if self.state == CompetitionState.INITIAL_APPROACH:
            return self._approach_output(
                vision,
                stm,
                now,
                message="靠近中心物资堆",
            )

        if self.state == CompetitionState.SEARCH:
            return self._handle_search(vision, pose, stm, now)

        if self.state == CompetitionState.WAIT_SEARCH_RECOVERY:
            return self._search_recovery_output(vision, stm, now)

        if self.state == CompetitionState.CLUSTER_APPROACH:
            return self._cluster_approach_output(vision, stm, now)

        if self.state == CompetitionState.DISPERSE:
            return self._disperse_output(vision, stm, now)

        if self.state == CompetitionState.APPROACH:
            return self._approach_output(
                vision,
                stm,
                now,
                message="靠近锁定批次",
            )

        if self.state == CompetitionState.CAPTURE_AUDIT:
            return self._audit_output(vision, stm, now)

        if self.state == CompetitionState.AUDIT_CONFIRM:
            return self._audit_confirmation_output(vision, stm, now)

        if self.state == CompetitionState.GRAB:
            if (
                stm.fresh and
                stm.mode == STM_MODE_POST_GRAB_AUDIT and
                stm.gripper_closed and
                stm.claw_visible and
                self.selected_batch is not None
            ):
                self._begin_post_grab_audit(vision, now)
                return self._audit_output(vision, stm, now)
            if self.grab_complete_confirmed:
                if not pose.valid:
                    return CompetitionOutput(
                        self.state,
                        self._pause(),
                        "抓取已完成，等待定位恢复后发送NAV",
                        self.selected_batch,
                        tx_policy="pause",
                        reason="grab_complete_pose_wait",
                        expected_stm_modes=(STM_MODE_CAPTURE_DONE,),
                    )
                self._reset_safe_zone_alignment()
                self._set_state(CompetitionState.INITIAL_STASH_NAV if self.selected_batch and self.selected_batch.initial_stash else CompetitionState.NAVIGATE, now)
                self.navigation_initial_ack = stm.acknowledged_sequence
                self._reset_delivery_evidence()
                navigation = self._navigate(vision, pose, stm, now)
                if (
                    navigation.command is not None and
                    navigation.command.opcode == CMD_NAVIGATE_WAYPOINT
                ):
                    navigation = replace(
                        navigation,
                        event="grab_complete_navigation_start",
                        reason="grab_complete_navigation_start",
                    )
                return navigation
            return CompetitionOutput(
                self.state,
                CommandRequest(CMD_GRAB_CONFIRMED, self._side_flags()),
                "等待F407完成核心前探（如需要）和合爪，并进入mode=23复审",
                self.selected_batch,
                expected_stm_modes=(STM_MODE_POST_GRAB_AUDIT,),
            )

        if self.state == CompetitionState.POST_GRAB_AUDIT:
            return self._audit_output(vision, stm, now)

        if self.state == CompetitionState.ALIGN_SAFE_ZONE_BY_POSE:
            pose_align_command = self._safe_zone_pose_align_command()
            self.safe_zone_align_command_accepted = self._command_acceptance_seen(
                stm,
                pose_align_command,
                self.safe_zone_align_tx_baseline,
                self.safe_zone_align_initial_ack,
                self.safe_zone_align_command_accepted,
            )
            if (
                stm.fresh and
                stm.mode == STM_MODE_ALIGN_SAFE_ZONE and
                self.safe_zone_align_command_accepted
            ):
                self._begin_safe_zone_acquire(vision, now)
                return CompetitionOutput(
                    self.state,
                    self._safe_zone_pose_align_command(),
                    "定位正方向对齐完成，开始连续3帧识别本方安全区",
                    self.selected_batch,
                    event="safe_zone_acquire_start",
                    tx_policy="normal_command",
                    reason="safe_zone_acquire_wait",
                    expected_stm_modes=(STM_MODE_ALIGN_SAFE_ZONE,),
                )
            return CompetitionOutput(
                self.state,
                pose_align_command,
                "在60 cm预备点按定位航向正对安全区",
                self.selected_batch,
                tx_policy="normal_command",
                reason="safe_zone_pose_align_wait",
                expected_stm_modes=(
                    STM_MODE_NAVIGATE,
                    STM_MODE_ALIGN_SAFE_ZONE,
                ),
            )

        if self.state == CompetitionState.ACQUIRE_SAFE_ZONE:
            if self._observe_safe_zone_freeze(vision, now):
                self.safe_zone_visual_align_initial_ack = stm.acknowledged_sequence
                self.safe_zone_visual_align_tx_baseline = stm.relay_mission_tx_frames
                self.safe_zone_visual_align_command_accepted = False
                self._set_state(CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX, now)
                return CompetitionOutput(
                    self.state,
                    self._safe_zone_visual_align_command(),
                    (
                        "连续3帧安全区已冻结，按"
                        f"{self.locked_safe_target_x_px}px目标执行一次视觉转向"
                    ),
                    self.selected_batch,
                    event="safe_zone_bbox_locked",
                    tx_policy="normal_command",
                    reason="safe_zone_visual_align_start",
                    expected_stm_modes=(
                        STM_MODE_NAVIGATE,
                        STM_MODE_ALIGN_SAFE_ZONE,
                    ),
                )
            if (
                self.safe_zone_acquire_started_s is not None and
                now - self.safe_zone_acquire_started_s >=
                self.settings.safe_zone_acquire_timeout_s
            ):
                self.safe_zone_fallback = True
                self.safe_zone_visual_locked = False
                self.enter_initial_ack = stm.acknowledged_sequence
                self.enter_tx_baseline = stm.relay_mission_tx_frames
                self.enter_command_accepted = False
                self._set_state(CompetitionState.ENTER_SAFE_ZONE, now)
                return CompetitionOutput(
                    self.state,
                    self._enter_safe_zone_command(),
                    "5秒内未连续识别3帧安全区，使用定位正方向降级推进",
                    self.selected_batch,
                    event="safe_zone_acquire_fallback",
                    tx_policy="normal_command",
                    reason="safe_zone_position_fallback",
                    expected_stm_modes=(STM_MODE_RAM_VERIFY,),
                )
            return CompetitionOutput(
                self.state,
                self._safe_zone_pose_align_command(),
                (
                    "保持定位正方向，等待连续3帧本方安全区："
                    f"{self.safe_zone_freeze_hits}/{self.settings.safe_zone_freeze_frames}"
                ),
                self.selected_batch,
                tx_policy="normal_command",
                reason="safe_zone_acquire_wait",
                expected_stm_modes=(STM_MODE_ALIGN_SAFE_ZONE,),
            )

        if self.state == CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX:
            visual_align_command = self._safe_zone_visual_align_command()
            self.safe_zone_visual_align_command_accepted = (
                self._command_acceptance_seen(
                    stm,
                    visual_align_command,
                    self.safe_zone_visual_align_tx_baseline,
                    self.safe_zone_visual_align_initial_ack,
                    self.safe_zone_visual_align_command_accepted,
                )
            )
            if (
                stm.fresh and
                stm.mode == STM_MODE_ALIGN_SAFE_ZONE and
                self.safe_zone_visual_align_command_accepted
            ):
                self.safe_zone_visual_locked = True
                self._begin_safe_corridor_check(vision, now)
                return CompetitionOutput(
                    self.state,
                    visual_align_command,
                    "第二次视觉ALIGN完成，建立安全半区入口推进走廊并检查障碍",
                    self.selected_batch,
                    event="safe_zone_corridor_check_start",
                    tx_policy="normal_command",
                    reason="safe_zone_corridor_check_start",
                    expected_stm_modes=(STM_MODE_ALIGN_SAFE_ZONE,),
                )
            return CompetitionOutput(
                self.state,
                visual_align_command,
                "按冻结安全区分区点执行一次视觉修正转向",
                self.selected_batch,
                tx_policy="normal_command",
                reason="safe_zone_visual_align_wait",
                expected_stm_modes=(
                    STM_MODE_NAVIGATE,
                    STM_MODE_ALIGN_SAFE_ZONE,
                ),
            )

        if self.state == CompetitionState.SAFE_ZONE_CORRIDOR_CHECK:
            new_corridor_frame = (
                self._vision_fresh(vision, now) and
                vision.frame_sequence > 0 and
                (
                    self.safe_corridor_frame_floor is None or
                    vision.frame_sequence > self.safe_corridor_frame_floor
                ) and
                vision.frame_sequence != self.safe_corridor_last_frame_sequence and
                bool(vision.safe_corridor_polygon)
            )
            if not new_corridor_frame:
                return CompetitionOutput(
                    self.state,
                    self._safe_zone_visual_align_command(),
                    "保持第二次ALIGN结果，等待新的安全区推进走廊画面",
                    self.selected_batch,
                    tx_policy="normal_command",
                    reason="safe_zone_corridor_frame_wait",
                    expected_stm_modes=(STM_MODE_ALIGN_SAFE_ZONE,),
                )
            self.safe_corridor_last_frame_sequence = vision.frame_sequence
            if vision.safe_corridor_obstacle_class is not None:
                if self.safe_sweep_attempts < 2:
                    return self._start_safe_zone_clear(vision, stm, now)
                self._set_state(CompetitionState.WAIT_SAFE_ZONE_CLEAR, now)
                return CompetitionOutput(
                    self.state,
                    self._pause(),
                    "本趟已完成2次扫障但推进走廊仍有阻挡，保持停车等待处理",
                    self.selected_batch,
                    event="safe_zone_clear_limit_reached",
                    tx_policy="pause",
                    reason="safe_zone_clear_limit_reached",
                    expected_stm_modes=(STM_MODE_ALIGN_SAFE_ZONE,),
                )
            self.enter_initial_ack = stm.acknowledged_sequence
            self.enter_tx_baseline = stm.relay_mission_tx_frames
            self.enter_command_accepted = False
            self._set_state(CompetitionState.ENTER_SAFE_ZONE, now)
            return CompetitionOutput(
                self.state,
                self._enter_safe_zone_command(),
                "推进走廊已确认无须扫障，按锁存航向推进安全区",
                self.selected_batch,
                event="safe_zone_entry",
                tx_policy="normal_command",
                reason="safe_zone_visual_push_start",
                expected_stm_modes=(STM_MODE_RAM_VERIFY,),
            )

        if self.state == CompetitionState.WAIT_SAFE_ZONE_CLEAR:
            new_corridor_frame = (
                self._vision_fresh(vision, now) and
                vision.frame_sequence > 0 and
                vision.frame_sequence != self.safe_corridor_last_frame_sequence and
                bool(vision.safe_corridor_polygon)
            )
            if new_corridor_frame:
                self.safe_corridor_last_frame_sequence = vision.frame_sequence
                if vision.safe_corridor_obstacle_class is None:
                    self.enter_initial_ack = stm.acknowledged_sequence
                    self.enter_tx_baseline = stm.relay_mission_tx_frames
                    self.enter_command_accepted = False
                    self._set_state(CompetitionState.ENTER_SAFE_ZONE, now)
                    return CompetitionOutput(
                        self.state,
                        self._enter_safe_zone_command(),
                        "人工处理后推进走廊已清空，解除PAUSE并进入安全区",
                        self.selected_batch,
                        event="safe_zone_clear_wait_resolved",
                        tx_policy="normal_command",
                        reason="safe_zone_clear_wait_resolved",
                        expected_stm_modes=(STM_MODE_RAM_VERIFY,),
                    )
            return CompetitionOutput(
                self.state,
                self._pause(),
                "扫障次数已达2次且走廊仍被阻挡，保持当前阶段停车",
                self.selected_batch,
                tx_policy="pause",
                reason="safe_zone_clear_limit_wait",
                expected_stm_modes=(STM_MODE_ALIGN_SAFE_ZONE,),
            )

        if self.state == CompetitionState.CLEAR_SAFE_ZONE:
            assert self.safe_sweep_command is not None
            self.safe_sweep_command_accepted = self._command_acceptance_seen(
                stm,
                self.safe_sweep_command,
                self.safe_sweep_tx_baseline,
                self.safe_sweep_initial_ack,
                self.safe_sweep_command_accepted,
            )
            if stm.fresh and stm.mode == STM_MODE_SAFE_SWEEP:
                self.safe_sweep_execution_seen = True
            if (
                self.safe_sweep_command_accepted and
                stm.fresh and
                stm.mode == STM_MODE_POST_GRAB_AUDIT and
                stm.gripper_closed and
                stm.claw_visible
            ):
                self.safe_sweep_reaudit_active = True
                self._begin_post_grab_audit(vision, now)
                return self._audit_output(vision, stm, now)
            return CompetitionOutput(
                self.state,
                self.safe_sweep_command,
                (
                    "F407正在mode39扫障并重新夹回原货物，mode39仅作诊断"
                    if stm.mode == STM_MODE_SAFE_SWEEP else
                    "持续发送CLEAR_SAFE_ZONE，等待本次ACK或新鲜mode23夹爪状态"
                ),
                self.selected_batch,
                motion_expected=True,
                tx_policy="normal_command",
                reason=(
                    "safe_zone_clear_running"
                    if stm.mode == STM_MODE_SAFE_SWEEP else
                    "safe_zone_clear_accept_wait"
                ),
                expected_stm_modes=(
                    STM_MODE_SAFE_SWEEP,
                    STM_MODE_POST_GRAB_AUDIT,
                ),
            )

        if self.state == CompetitionState.INVALID_RELEASE:
            audit = vision.capture_audit or CargoAudit()
            observe_release = (
                self.invalid_release_context == "separate_then_search" and
                not self.invalid_release_final
            )
            expected_mode = (
                STM_MODE_DISPERSE_DONE
                if observe_release else
                {
                    "left": STM_MODE_RELEASE_LEFT_DONE,
                    "right": STM_MODE_RELEASE_RIGHT_DONE,
                    "both": STM_MODE_RELEASE_BOTH_DONE,
                }[self.invalid_release_side]
            )
            release_command = self._invalid_release_command()
            self.invalid_release_command_accepted = (
                self._command_acceptance_seen(
                    stm,
                    release_command,
                    self.invalid_release_tx_baseline,
                    self.invalid_release_initial_ack,
                    self.invalid_release_command_accepted,
                )
            )
            release_complete = (
                stm.fresh and
                stm.mode == expected_mode and
                self.invalid_release_command_accepted
            )
            if release_complete:
                if observe_release:
                    self.disperse_observe_attempts += 1
                    self._begin_capture_audit(
                        vision,
                        now,
                        cluster_active=False,
                        recheck_pending=True,
                        recheck_context="disperse_observe",
                    )
                    self.invalid_release_context = "none"
                    self.invalid_release_final = False
                    return self._audit_output(vision, stm, now)
                if self.invalid_release_context in {
                    "disperse_final_release",
                    "final_release",
                }:
                    self._clear_selected_batch()
                    self._clear_cluster_context(reset_attempts=True)
                    self.cargo_recheck_pending = False
                    self.cargo_recheck_context = "none"
                    self.audit_hits = 0
                    self.audit_last_signature = None
                    self.audit_last_frame_sequence = None
                    self.audit_recheck_frame_floor = None
                    self.audit_recheck_started_s = None
                    self._clear_pending_audit()
                    self.invalid_release_context = "none"
                    self.invalid_release_final = False
                    self.search_epoch_frame_floor = (
                        vision.frame_sequence if vision.frame_sequence > 0 else None
                    )
                    self.search_candidate_key = None
                    self.search_candidate_hits = 0
                    self._set_state(CompetitionState.SEARCH, now)
                    return CompetitionOutput(
                        self.state,
                        self._hold(),
                        "最终RELEASE_BOTH已由新鲜mode=34和本次命令接受证据确认，清空批次并回SEARCH",
                        audit=audit,
                        event="final_release_done",
                        tx_policy="hold",
                        reason="final_release_acknowledged",
                        expected_stm_modes=(STM_MODE_RELEASE_BOTH_DONE, STM_MODE_SEARCH),
                    )
                self._set_state(CompetitionState.INVALID_BACKOFF, now)
                self.invalid_backoff_initial_ack = stm.acknowledged_sequence
                self.invalid_backoff_tx_baseline = stm.relay_mission_tx_frames
                self.invalid_backoff_command_accepted = False
                return CompetitionOutput(
                    self.state,
                    self._yield_command(),
                    "异常侧已释放，后退脱离后复审",
                    audit=audit,
                    event="invalid_release_done",
                    motion_expected=True,
                    tx_policy="normal_command",
                    reason="invalid_release_done_acknowledged",
                    expected_stm_modes=(STM_MODE_YIELD_DONE,),
                )
            return self._invalid_release_output(audit, stm, now)

        if self.state == CompetitionState.INVALID_BACKOFF:
            yield_command = self._yield_command()
            self.invalid_backoff_command_accepted = self._command_acceptance_seen(
                stm,
                yield_command,
                self.invalid_backoff_tx_baseline,
                self.invalid_backoff_initial_ack,
                self.invalid_backoff_command_accepted,
            )
            if (
                stm.fresh and
                stm.mode == STM_MODE_YIELD_DONE and
                self.invalid_backoff_command_accepted
            ):
                self._begin_capture_audit(
                    vision,
                    now,
                    cluster_active=False,
                    recheck_pending=True,
                    recheck_context="single_side",
                )
                return self._audit_output(vision, stm, now)
            return CompetitionOutput(
                self.state,
                yield_command,
                "后退脱离非法物资",
                motion_expected=True,
            )

        if self.state == CompetitionState.RETURN_STASH:
            return self._return_stash_output(pose, stm, now)

        if self.state == CompetitionState.WAIT_STASH_SEARCH_HANDOFF:
            return self._stash_search_handoff_output(stm, now)

        if self.state in {CompetitionState.INITIAL_STASH_NAV, CompetitionState.NAVIGATE}:
            return self._navigate(vision, pose, stm, now)

        if self.state == CompetitionState.INITIAL_RELEASE:
            initial_release_command = CommandRequest(
                CMD_RELEASE_BOTH, self._side_flags()
            )
            self.initial_release_command_accepted = (
                self._command_acceptance_seen(
                    stm,
                    initial_release_command,
                    self.initial_release_tx_baseline,
                    self.initial_release_initial_ack,
                    self.initial_release_command_accepted,
                )
            )
            if (
                stm.fresh and
                stm.mode == STM_MODE_RELEASE_BOTH_DONE and
                self.initial_release_command_accepted
            ):
                self.initial_stash_done = True
                self.stash_has_cargo = True
                self._clear_selected_batch()
                self.safe_zone_exit_pending = False
                self._begin_return(stm, now)
                return self._return_center_output(pose, now, "临时物资已放下，返回中心寻找首件绿色")
            return CompetitionOutput(
                self.state,
                initial_release_command,
                "等待本次藏点释放完成；未收到新鲜且已确认的mode=34",
                self.selected_batch,
                tx_policy="normal_command",
                reason="initial_stash_release_pending",
                expected_stm_modes=(STM_MODE_RELEASE_BOTH_DONE,),
            )

        if self.state == CompetitionState.ENTER_SAFE_ZONE:
            self.enter_command_accepted = self._opcode_acceptance_seen(
                stm,
                CMD_ENTER_SAFE_ZONE,
                self.enter_tx_baseline,
                self.enter_initial_ack,
                self.enter_command_accepted,
            )
            if (
                stm.fresh and
                stm.mode == STM_MODE_RAM_VERIFY and
                self.enter_command_accepted
            ):
                if self.delivery_observation_attempt == 0:
                    self._start_delivery_observation(now, 1)
                self._set_state(CompetitionState.DELIVERY_VERIFY, now)
            return CompetitionOutput(
                self.state,
                self._enter_safe_zone_command(),
                "进入安全区并等待视觉确认",
                self.selected_batch,
                tx_policy="normal_command",
                reason="delivery_enter_safe_zone",
            )

        if self.state == CompetitionState.DELIVERY_VERIFY:
            if self.delivery_visual_confirmed and stm.fresh and stm.mode == STM_MODE_RAM_VERIFY:
                return self._finish_delivery(
                    stm,
                    now,
                    basis="delivery_visual_confirmed",
                    message="视觉确认物资已由区外进入安全区，通知下位机完成",
                    event="delivery_confirmed",
                )
            if (
                self.delivery_observation_started_s is None or
                self._delivery_observation_elapsed(now) is not None and
                self._delivery_observation_elapsed(now) >= self.settings.delivery_observation_timeout_s
            ):
                if self.delivery_observation_started_s is None:
                    self._start_delivery_observation(now, 1)
                else:
                    return self._delivery_timeout_output(stm, now)
            return CompetitionOutput(
                self.state,
                self._enter_safe_zone_command(),
                f"等待安全区视觉确认 {self.delivery_inside_hits}/{self.settings.delivery_inside_required}"
                f"（窗口{len(self.delivery_window)}/{self.settings.delivery_window_frames}，漏检{self._delivery_window_misses()}）",
                self.selected_batch,
                tx_policy="normal_command",
                reason="delivery_visual_pending",
            )

        if self.state == CompetitionState.TASK_COMPLETE:
            if (
                stm.fresh and
                stm.mode in {
                    STM_MODE_EXIT_SAFE_ZONE,
                    STM_MODE_FACE_FIELD_CENTER,
                }
            ):
                self._clear_selected_batch()
                self.safe_zone_exit_pending = True
                self._begin_return(stm, now)
                return self._return_center_output(
                    pose,
                    now,
                    "F407已进入本地退出/转向阶段，持续发送RETURN_CENTER",
                )
            if self._fresh_mode_after(
                stm, STM_MODE_SEARCH, self.task_complete_initial_ack
            ):
                self._clear_selected_batch()
                self._set_state(CompetitionState.SEARCH, now)
                return CompetitionOutput(self.state, self._hold(), "下位机已完成投送并进入SEARCH")
            return CompetitionOutput(self.state, CommandRequest(CMD_TASK_COMPLETE, self._side_flags()), "等待下位机张爪并退出安全区")

        if self.state == CompetitionState.RETURN_CENTER:
            self._latch_return_command_acceptance(stm)
            if (
                stm.fresh and
                stm.mode == STM_MODE_SEARCH and
                self.return_command_accepted
            ):
                self._set_state(CompetitionState.SEARCH, now)
                self.return_search_mode_seen = True
                self.return_search_frame_floor = (
                    vision.frame_sequence if vision.frame_sequence > 0 else 0
                )
                return CompetitionOutput(
                    self.state,
                    self._hold(),
                    f"回到中心搜索区，等待相机90°和新视觉帧；已完成{self.delivery_count}件",
                    event="return_search_gate_start",
                    tx_policy="hold",
                    reason="return_search_frame_gate",
                    expected_stm_modes=(STM_MODE_SEARCH,),
                )
            return self._return_center_output(pose, now, "持续返回中心")

        return CompetitionOutput(
            self.state,
            self._pause(),
            "未处理状态，冻结当前阶段",
            tx_policy="pause",
            reason="unhandled_state_pause",
        )
