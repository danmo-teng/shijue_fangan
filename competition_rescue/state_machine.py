#!/usr/bin/env python3
"""Complete rescue competition planner.

This module deliberately does not change ``mission_test/state_machine.py``.
It keeps the semantic decisions on the X5 and emits compact motion/audit
commands for the matching F407 firmware.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, replace
from enum import Enum
import math
from typing import Iterable

from protocol import (
    AUDIT_DESTINATION_INJURY,
    CargoAuditPayload,
    CMD_ABORT,
    CMD_APPROACH_TARGET,
    CMD_CARGO_AUDIT,
    CMD_CHANGE_LANE,
    CMD_DISPERSE_PILE,
    CMD_ESCAPE_MANEUVER,
    CMD_GRAB_CONFIRMED,
    CMD_HOLD,
    CMD_NAVIGATE_WAYPOINT,
    CMD_RELEASE_BOTH,
    CMD_RELEASE_LEFT,
    CMD_RELEASE_RIGHT,
    CMD_RETURN_CENTER,
    CMD_TASK_COMPLETE,
    CMD_ENTER_SAFE_ZONE,
    CMD_YIELD_BACKOFF,
    CMD_VALID,
    escape_maneuver_frame,
    mission_frame,
)


FIELD_HALF_M = 1.50
CARGO_CLASSES = frozenset(
    {"green_supply", "core_black", "injured_orange", "danger_cyan", "unknown"}
)
MATERIAL_CLASSES = frozenset({"green_supply", "core_black"})

STM_MODE_SEARCH = 3
STM_MODE_NAVIGATE = 10
STM_MODE_RAM_VERIFY = 15
STM_MODE_EXIT_SAFE_ZONE = 16
STM_MODE_FACE_FIELD_CENTER = 17
STM_MODE_APPROACH_TARGET = 20
STM_MODE_CAPTURE_AUDIT = 21
STM_MODE_CAPTURE_DONE = 22
STM_MODE_YIELD_DONE = 30
STM_MODE_ESCAPE_DONE = 31
STM_MODE_RELEASE_LEFT_DONE = 32
STM_MODE_RELEASE_RIGHT_DONE = 33
STM_MODE_RELEASE_BOTH_DONE = 34
STM_MODE_DISPERSE_DONE = 35
STM_MODE_LANE_DONE = 36


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


@dataclass(frozen=True)
class StmSnapshot:
    mode: int = 0
    flags: int = 0
    age_ms: float = math.inf
    fault_code: int = 0
    acknowledged_sequence: int = 0

    @property
    def claw_visible(self) -> bool:
        return bool(self.flags & (1 << 0))

    @property
    def gripper_closed(self) -> bool:
        return bool(self.flags & (1 << 1))

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
    total_count: int = 0
    danger_present: bool = False
    unknown_present: bool = False
    injury_mixed: bool = False
    stable: bool = False
    left_invalid: bool = False
    right_invalid: bool = False
    left_selected_count: int = 0
    right_selected_count: int = 0

    @property
    def signature(self) -> tuple:
        return (
            self.left_class,
            self.right_class,
            self.left_count,
            self.right_count,
            self.total_count,
            self.danger_present,
            self.unknown_present,
            self.injury_mixed,
            self.left_selected_count,
            self.right_selected_count,
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


@dataclass(frozen=True)
class CargoBatch:
    track_ids: tuple[int, ...]
    classes: tuple[str, ...]
    destination: str
    initial_stash: bool = False

    @property
    def total_count(self) -> int:
        return len(self.track_ids)

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


class CompetitionState(str, Enum):
    WAIT_START = "WAIT_START"
    INITIAL_OBSERVE = "INITIAL_OBSERVE"
    INITIAL_APPROACH = "INITIAL_APPROACH"
    CAPTURE_AUDIT = "CAPTURE_AUDIT"
    GRAB = "GRAB"
    INITIAL_STASH_NAV = "INITIAL_STASH_NAV"
    INITIAL_RELEASE = "INITIAL_RELEASE"
    SEARCH = "SEARCH"
    DISPERSE = "DISPERSE"
    APPROACH = "APPROACH"
    NAVIGATE = "NAVIGATE"
    ENTER_SAFE_ZONE = "ENTER_SAFE_ZONE"
    DELIVERY_VERIFY = "DELIVERY_VERIFY"
    TASK_COMPLETE = "TASK_COMPLETE"
    RETURN_CENTER = "RETURN_CENTER"
    RETURN_STASH = "RETURN_STASH"
    BOUNDARY_RECOVERY = "BOUNDARY_RECOVERY"
    DETOUR = "DETOUR"
    INVALID_RELEASE = "INVALID_RELEASE"
    INVALID_BACKOFF = "INVALID_BACKOFF"
    STUCK_YIELD = "STUCK_YIELD"
    STUCK_ESCAPE = "STUCK_ESCAPE"
    FINISHED = "FINISHED"
    FAULT = "FAULT"


@dataclass
class CompetitionSettings:
    side: str
    start_zone: int = 1
    initial_stash_enabled: bool = True
    initial_observe_s: float = 0.8
    target_loss_s: float = 1.2
    audit_stable_frames: int = 3
    delivery_visual_frames: int = 5
    batch_radius_m: float = 0.55
    near_material_max_distance_m: float = 0.85
    capture_clearance_m: float = 0.16
    max_batch_count: int = 3
    center_stop_radius_m: float = 0.0
    return_zero_tolerance_m: float = 0.025
    vision_stale_s: float = 0.30
    search_empty_hold_s: float = 2.0
    disperse_limit: int = 2
    stuck_timeout_s: float = 1.5
    stuck_progress_m: float = 0.04
    yield_distance_m: float = 0.25
    yield_timeout_s: float = 1.8
    escape_spin_deg: int = 90
    escape_lateral_m: float = 0.16
    escape_timeout_s: float = 4.0
    max_escape_attempts: int = 2
    detour_lateral_m: float = 0.25
    detour_timeout_s: float = 3.0
    boundary_margin_m: float = 0.08
    fence_axial_tolerance_m: float = 0.035
    fence_lateral_tolerance_m: float = 0.060
    safe_zone_center_x_m: float = 0.15
    safe_zone_inner_edge_m: float = 1.20
    safe_fence_face_m: float = 1.14
    push_plate_offset_m: float = 0.105
    fence_stop_margin_m: float = 0.0075

    def __post_init__(self) -> None:
        if self.side not in {"red", "blue"}:
            raise ValueError("side must be red or blue")
        if self.start_zone not in {1, 2, 3, 4}:
            raise ValueError("start zone must be 1..4")
        if self.max_batch_count != 3:
            raise ValueError("competition batch maximum must remain 3")
        if self.audit_stable_frames <= 0 or self.delivery_visual_frames <= 0:
            raise ValueError("audit frame counts must be positive")
        if self.stuck_timeout_s <= 0 or self.stuck_progress_m <= 0:
            raise ValueError("stuck thresholds must be positive")
        if self.center_stop_radius_m < 0 or self.return_zero_tolerance_m <= 0:
            raise ValueError("return distance thresholds are invalid")
        if self.vision_stale_s <= 0:
            raise ValueError("vision stale threshold must be positive")
        if self.near_material_max_distance_m <= 0:
            raise ValueError("near material distance must be positive")
        if self.boundary_margin_m < 0:
            raise ValueError("boundary margin cannot be negative")

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

    MOTION_STATES = frozenset(
        {
            CompetitionState.INITIAL_APPROACH,
            CompetitionState.INITIAL_STASH_NAV,
            CompetitionState.APPROACH,
            CompetitionState.NAVIGATE,
            CompetitionState.RETURN_CENTER,
            CompetitionState.RETURN_STASH,
            CompetitionState.BOUNDARY_RECOVERY,
            CompetitionState.DETOUR,
        }
    )

    def __init__(self, settings: CompetitionSettings) -> None:
        self.settings = settings
        self.state = CompetitionState.WAIT_START
        self.state_started_s = 0.0
        self.initial_stash_done = not settings.initial_stash_enabled
        self.first_common_delivered = False
        self.delivery_count = 0
        self.selected_batch: CargoBatch | None = None
        self.audit_id = 0
        self.audit_last_signature: tuple | None = None
        self.audit_last_frame_sequence: int | None = None
        self.audit_hits = 0
        self.audit_recheck_frame_floor: int | None = None
        self.audit_recheck_started_s: float | None = None
        self.invalid_release_side = "both"
        self.invalid_release_final = False
        self.cargo_recheck_pending = False
        self.delivery_outside_seen = False
        self.delivery_inside_hits = 0
        self.delivery_visual_confirmed = False
        self.delivery_last_frame_sequence: int | None = None
        self.search_empty_started_s: float | None = None
        self.stash_checked = False
        self.disperse_attempts = 0
        self.target_lost_started_s: float | None = None
        self.resume_state: CompetitionState | None = None
        self.detour_started_s = 0.0
        self.detour_attempts = 0
        self.detour_lateral_m = settings.detour_lateral_m
        self.stuck_attempts = 0
        self.motion_anchor: tuple[float, float] | None = None
        self.motion_anchor_started_s = 0.0
        self.stuck_started_s = 0.0

    def _set_state(self, state: CompetitionState, now: float) -> None:
        if self.state != state:
            self.state = state
            self.state_started_s = now
            self.motion_anchor = None
            self.motion_anchor_started_s = now
            self.target_lost_started_s = None
            self.audit_last_signature = None
            self.audit_last_frame_sequence = None
            self.audit_hits = 0

    def _hold(self) -> CommandRequest:
        return CommandRequest(CMD_HOLD)

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
        candidates = [
            item for item in self._visible_cargo(vision)
            if item.track_id in self.selected_batch.track_ids
        ]
        if not candidates:
            fallback = [
                item for item in self._visible_cargo(vision)
                if item.class_name in self.selected_batch.classes
            ]
            # Never switch to an arbitrary same-class object while approaching
            # a locked batch. Reacquire only when exactly one fallback exists;
            # otherwise hold and let the loss timeout return to SEARCH.
            candidates = fallback if len(fallback) == 1 else []
        return max(candidates, key=lambda item: item.area_px, default=None)

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
            if item.class_name == "green_supply" and item.hits >= 2
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
        cargo = self._visible_cargo(vision)
        if not cargo:
            return None
        cargo.sort(key=lambda item: (-item.area_px, item.distance_m, item.track_id))
        cargo = cargo[: self.settings.max_batch_count]
        return CargoBatch(
            tuple(item.track_id for item in cargo),
            tuple(item.class_name for item in cargo),
            "stash",
            initial_stash=True,
        )

    def _audit_valid(self, audit: CargoAudit) -> bool:
        counts = Counter()
        if audit.left_class and audit.left_class != "mixed_material":
            counts[audit.left_class] += audit.left_count
        if audit.right_class and audit.right_class != "mixed_material":
            counts[audit.right_class] += audit.right_count
        if self.selected_batch is None:
            return False
        if (
            audit.total_count <= 0 or
            audit.total_count > self.settings.max_batch_count or
            audit.left_count + audit.right_count != audit.total_count or
            audit.danger_present or
            audit.unknown_present or
            audit.left_class in {"danger_cyan", "unknown"} or
            audit.right_class in {"danger_cyan", "unknown"} or
            audit.injury_mixed
        ):
            return False
        if self.selected_batch.initial_stash:
            # The first pile is deliberately only a temporary relocation. It
            # may contain mixed ordinary/core classes; dangerous, unknown or
            # injury-mixed cargo must still be separated first.
            return True
        if not self.first_common_delivered:
            return audit.total_count == 1 and counts == Counter({"green_supply": 1})
        if self.selected_batch.destination == "injury":
            return audit.total_count == 1 and counts == Counter({"injured_orange": 1})
        side_classes = {audit.left_class, audit.right_class} - {"", "mixed_material"}
        return (
            1 <= audit.total_count <= self.settings.max_batch_count and
            side_classes.issubset(MATERIAL_CLASSES) and
            audit.left_class not in {"injured_orange", "danger_cyan", "unknown"} and
            audit.right_class not in {"injured_orange", "danger_cyan", "unknown"} and
            not audit.injury_mixed
        )

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

    def _choose_release_side(self, audit: CargoAudit) -> str:
        """Return the side to open; RELEASE_LEFT means discard left cargo."""
        if self.cargo_recheck_pending:
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

        # For count overflow, retain the side with more legal cargo. If both
        # sides are equally plausible, use the side containing locked tracks;
        # without a reliable preference, discard both rather than guess.
        if left_legal and right_legal:
            if audit.left_count != audit.right_count:
                return "right" if audit.left_count > audit.right_count else "left"
            if audit.left_selected_count != audit.right_selected_count:
                return "right" if audit.left_selected_count > audit.right_selected_count else "left"
        return "both"

    def _audit_output(self, vision: VisionSnapshot, now: float) -> CompetitionOutput:
        if not self._vision_fresh(vision, now):
            return CompetitionOutput(
                self.state,
                CommandRequest(CMD_HOLD, self._side_flags()),
                "夹内审核视觉帧已超时，保持停车等待新帧",
                self.selected_batch,
                event="audit_visual_timeout",
            )
        if self.cargo_recheck_pending:
            new_sequence = (
                self.audit_recheck_frame_floor is not None and
                vision.frame_sequence > self.audit_recheck_frame_floor
            )
            new_timestamp = (
                self.audit_recheck_started_s is not None and
                vision.observed_monotonic_s is not None and
                vision.observed_monotonic_s > self.audit_recheck_started_s
            )
            if (
                self.audit_recheck_frame_floor is not None or
                self.audit_recheck_started_s is not None
            ) and not (new_sequence or new_timestamp):
                return CompetitionOutput(
                    self.state,
                    CommandRequest(CMD_HOLD, self._side_flags()),
                    "等待mode=30之后的新夹内视觉帧，再开始复审",
                    self.selected_batch,
                    event="audit_recheck_wait_frame",
                )
            self.audit_recheck_frame_floor = None
            self.audit_recheck_started_s = None
        if vision.capture_audit is None:
            self.audit_last_signature = None
            self.audit_last_frame_sequence = None
            self.audit_hits = 0
            assert self.selected_batch is not None
            empty_payload = CargoAudit().to_protocol(
                initial_stash=self.selected_batch.initial_stash,
                destination=self.selected_batch.destination,
                audit_id=self.audit_id,
            )
            return CompetitionOutput(
                self.state,
                CommandRequest(CMD_CARGO_AUDIT, CMD_VALID, audit=empty_payload),
                "夹内暂未识别到稳定物资，保持停车等待",
                self.selected_batch,
                None,
                "cargo_audit_no_observation",
            )
        audit = vision.capture_audit
        new_frame = (
            vision.frame_sequence <= 0 or
            vision.frame_sequence != self.audit_last_frame_sequence
        )
        if new_frame:
            self.audit_last_frame_sequence = vision.frame_sequence
            if audit.signature == self.audit_last_signature:
                self.audit_hits += 1
            else:
                self.audit_last_signature = audit.signature
                self.audit_hits = 1
        stable = replace(audit, stable=self.audit_hits >= self.settings.audit_stable_frames)
        self.audit_id = (self.audit_id + 1) & 0xFF
        assert self.selected_batch is not None
        payload = stable.to_protocol(
            initial_stash=self.selected_batch.initial_stash,
            destination=self.selected_batch.destination,
            audit_id=self.audit_id,
        )
        if stable.stable and self._audit_valid(stable):
            self.audit_recheck_frame_floor = None
            self._set_state(CompetitionState.GRAB, now)
            return CompetitionOutput(
                self.state,
                CommandRequest(CMD_GRAB_CONFIRMED, self._side_flags()),
                f"夹内审核通过：{stable.total_count}件，{self.selected_batch.destination}",
                self.selected_batch,
                stable,
                "cargo_audit_ok",
            )
        if stable.stable and not self._audit_valid(stable):
            self.invalid_release_side = self._choose_release_side(stable)
            self.invalid_release_final = (
                self.cargo_recheck_pending or
                self.invalid_release_side == "both"
            )
            self._set_state(CompetitionState.INVALID_RELEASE, now)
            return self._invalid_release_output(stable, now)
        return CompetitionOutput(
            self.state,
            CommandRequest(CMD_CARGO_AUDIT, CMD_VALID, audit=payload),
            f"等待夹内审核稳定：{stable.total_count}件，{self.audit_hits}/{self.settings.audit_stable_frames}",
            self.selected_batch,
            stable,
            "cargo_audit_wait",
        )

    def _invalid_release_output(self, audit: CargoAudit, now: float) -> CompetitionOutput:
        side = self.invalid_release_side
        opcode = {
            "left": CMD_RELEASE_LEFT,
            "right": CMD_RELEASE_RIGHT,
            "both": CMD_RELEASE_BOTH,
        }[side]
        return CompetitionOutput(
            self.state,
            CommandRequest(opcode, self._side_flags()),
            (
                f"夹内组合非法，释放{side}侧并后退复审"
                if side != "both" and not self.invalid_release_final
                else f"夹内组合非法，释放{side}侧并后退处理"
            ),
            self.selected_batch,
            audit,
            "invalid_cargo_release",
        )

    def _update_delivery_observation(self, vision: VisionSnapshot, now: float) -> None:
        if self.state not in {
            CompetitionState.NAVIGATE,
            CompetitionState.ENTER_SAFE_ZONE,
            CompetitionState.DELIVERY_VERIFY,
        }:
            return
        if not self._vision_fresh(vision, now):
            return
        if vision.frame_sequence > 0 and vision.frame_sequence == self.delivery_last_frame_sequence:
            return
        self.delivery_last_frame_sequence = vision.frame_sequence
        if vision.delivery_target_outside_safe_zone:
            self.delivery_outside_seen = True
        if (
            self.delivery_outside_seen and
            vision.delivery_target_found and
            vision.delivery_target_inside_safe_zone
        ):
            self.delivery_inside_hits += 1
            if self.delivery_inside_hits >= self.settings.delivery_visual_frames:
                self.delivery_visual_confirmed = True
        else:
            self.delivery_inside_hits = 0

    def _target_point(self) -> tuple[float, float]:
        assert self.selected_batch is not None
        if self.selected_batch.destination == "stash":
            return self.settings.stash_point
        if self.selected_batch.destination == "injury":
            return self.settings.injury_target_x_m, self.settings.fence_stop_y_m
        return self.settings.material_target_x_m, self.settings.fence_stop_y_m

    def _at_target(self, pose: PoseSnapshot, target: tuple[float, float]) -> bool:
        return (
            pose.valid and
            abs(pose.x_m - target[0]) <= self.settings.fence_lateral_tolerance_m and
            abs(pose.y_m - target[1]) <= self.settings.fence_axial_tolerance_m
        )

    def _navigation_command(self, pose: PoseSnapshot, target: tuple[float, float]) -> CommandRequest:
        if not pose.valid:
            return self._hold()
        dx, dy = target[0] - pose.x_m, target[1] - pose.y_m
        distance = math.hypot(dx, dy)
        heading = math.degrees(math.atan2(dy, dx)) % 360.0
        # Once close to the fence, use its known normal heading instead of a
        # noisy target bearing. This also prevents a near-target yaw flip.
        if distance <= 0.30:
            error = angle_error_deg(heading, self.settings.safe_heading_deg)
            error = max(-10.0, min(10.0, error))
            heading = (self.settings.safe_heading_deg + error) % 360.0
        return CommandRequest(
            CMD_NAVIGATE_WAYPOINT,
            self._side_flags() | (1 << 1) | (1 << 2) | (1 << 4),
            max(0, min(32767, round(distance * 1000.0))),
            0,
            round(heading * 100.0) % 36000,
        )

    def _return_center_output(self, pose: PoseSnapshot, now: float, message: str) -> CompetitionOutput:
        if not pose.valid:
            return CompetitionOutput(
                self.state,
                self._hold(),
                "等待有效定位后返回中心",
                motion_expected=False,
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

    def _outside_field(self, pose: PoseSnapshot) -> bool:
        if not pose.valid:
            return False
        limit = FIELD_HALF_M - self.settings.boundary_margin_m
        return abs(pose.x_m) > limit or abs(pose.y_m) > limit

    def _boundary_risk(self, pose: PoseSnapshot, target: tuple[float, float]) -> bool:
        if not pose.valid:
            return False
        # Keep the chassis centre inside the field by its 130 mm radius plus
        # the configured stopping margin. Only trigger when the requested
        # target continues toward the boundary; a route from the start corner
        # inward to the stash point remains allowed.
        limit = FIELD_HALF_M - 0.130 - self.settings.boundary_margin_m
        for coordinate, destination in ((pose.x_m, target[0]), (pose.y_m, target[1])):
            if abs(coordinate) <= limit:
                continue
            if coordinate > 0.0 and destination > coordinate:
                return True
            if coordinate < 0.0 and destination < coordinate:
                return True
        return False

    def _boundary_recovery_output(self, pose: PoseSnapshot, now: float) -> CompetitionOutput:
        if not pose.valid:
            return CompetitionOutput(
                self.state,
                self._hold(),
                "接近场地边缘但定位无效，保持停车",
                self.selected_batch,
                event="boundary_hold",
            )
        distance = min(0.35, math.hypot(pose.x_m, pose.y_m))
        heading = math.degrees(math.atan2(-pose.y_m, -pose.x_m)) % 360.0
        return CompetitionOutput(
            self.state,
            CommandRequest(
                CMD_NAVIGATE_WAYPOINT,
                self._side_flags() | (1 << 1) | (1 << 2) | (1 << 4),
                round(distance * 1000.0),
                0,
                round(heading * 100.0) % 36000,
            ),
            "接近场地边缘，转向场地内部",
            self.selected_batch,
            event="boundary_recovery",
            motion_expected=True,
        )

    def _start_detour(self, vision: VisionSnapshot, now: float) -> CompetitionOutput:
        self.resume_state = self.state
        self._set_state(CompetitionState.DETOUR, now)
        self.detour_started_s = now
        self.detour_attempts += 1
        lateral = self.settings.detour_lateral_m
        if vision.danger_side == "right":
            lateral = abs(lateral)
        elif vision.danger_side == "left":
            lateral = -abs(lateral)
        self.detour_lateral_m = lateral
        return CompetitionOutput(
            self.state,
            CommandRequest(CMD_CHANGE_LANE, CMD_VALID, round(lateral * 1000.0), 0),
            f"前方危险目标，向{('左' if lateral > 0 else '右')}侧换道",
            self.selected_batch,
            event="danger_detour",
            motion_expected=True,
        )

    def _motion_progress(self, pose: PoseSnapshot, now: float) -> tuple[float, float]:
        if self.motion_anchor is None:
            self.motion_anchor = (pose.x_m, pose.y_m)
            self.motion_anchor_started_s = now
            return 0.0, 0.0
        return _distance(self.motion_anchor, (pose.x_m, pose.y_m)), max(
            0.0, now - self.motion_anchor_started_s
        )

    def _stuck_override(
        self, vision: VisionSnapshot, pose: PoseSnapshot, stm: StmSnapshot, now: float
    ) -> CompetitionOutput | None:
        if self.state == CompetitionState.STUCK_YIELD:
            moved, elapsed = self._motion_progress(pose, now) if pose.valid else (0.0, now - self.stuck_started_s)
            if moved >= self.settings.stuck_progress_m:
                resume = self.resume_state or CompetitionState.SEARCH
                self._set_state(resume, now)
                return None
            if elapsed >= self.settings.yield_timeout_s or stm.mode == STM_MODE_YIELD_DONE:
                self._set_state(CompetitionState.STUCK_ESCAPE, now)
                self.stuck_started_s = now
                self.motion_anchor = (pose.x_m, pose.y_m) if pose.valid else None
                return CompetitionOutput(
                    self.state,
                    CommandRequest(
                        CMD_ESCAPE_MANEUVER,
                        CMD_VALID,
                        self.settings.escape_spin_deg * (-1 if self.stuck_attempts % 2 else 1),
                        round(self.settings.escape_lateral_m * 1000.0),
                    ),
                    "退让后仍无位移，执行旋转+横移脱困",
                    self.selected_batch,
                    event="stuck_escape",
                    motion_expected=True,
                    stuck_phase="escape",
                )
            return CompetitionOutput(
                self.state,
                CommandRequest(
                    CMD_YIELD_BACKOFF,
                    CMD_VALID,
                    -round(self.settings.yield_distance_m * 1000.0),
                    0,
                ),
                "疑似碰撞或物资卡住，先退让",
                self.selected_batch,
                event="stuck_yield",
                motion_expected=True,
                stuck_phase="yield",
            )
        if self.state == CompetitionState.STUCK_ESCAPE:
            moved, elapsed = self._motion_progress(pose, now) if pose.valid else (0.0, now - self.stuck_started_s)
            if moved >= self.settings.stuck_progress_m:
                resume = self.resume_state or CompetitionState.SEARCH
                self._set_state(resume, now)
                return None
            if elapsed >= self.settings.escape_timeout_s or stm.mode == STM_MODE_ESCAPE_DONE:
                self.stuck_attempts += 1
                if self.stuck_attempts >= self.settings.max_escape_attempts:
                    self._set_state(CompetitionState.FAULT, now)
                    return CompetitionOutput(
                        self.state,
                        CommandRequest(CMD_ABORT, self._side_flags()),
                        "脱困尝试后仍无位移，进入安全停车",
                        self.selected_batch,
                        event="stuck_fault",
                        stuck_phase="fault",
                    )
                self._set_state(CompetitionState.STUCK_YIELD, now)
                self.stuck_started_s = now
                self.motion_anchor = (pose.x_m, pose.y_m) if pose.valid else None
                return CompetitionOutput(
                    self.state,
                    CommandRequest(
                        CMD_YIELD_BACKOFF,
                        CMD_VALID,
                        -round(self.settings.yield_distance_m * 1000.0),
                        0,
                    ),
                    "第一次脱困后仍未脱离，重新退让",
                    self.selected_batch,
                    event="stuck_retry_yield",
                    motion_expected=True,
                    stuck_phase="yield",
                )
            return CompetitionOutput(
                self.state,
                CommandRequest(
                    CMD_ESCAPE_MANEUVER,
                    CMD_VALID,
                    self.settings.escape_spin_deg * (-1 if self.stuck_attempts % 2 else 1),
                    round(self.settings.escape_lateral_m * 1000.0),
                ),
                "执行旋转移动脱困",
                self.selected_batch,
                event="stuck_escape_running",
                motion_expected=True,
                stuck_phase="escape",
            )
        if self.state not in self.MOTION_STATES or not pose.valid:
            return None
        moved, elapsed = self._motion_progress(pose, now)
        command_active = stm.motors_active or stm.mode in {
            STM_MODE_APPROACH_TARGET,
            STM_MODE_NAVIGATE,
        }
        if command_active and elapsed >= self.settings.stuck_timeout_s and moved < self.settings.stuck_progress_m:
            self.resume_state = self.state
            self._set_state(CompetitionState.STUCK_YIELD, now)
            self.stuck_started_s = now
            self.motion_anchor = (pose.x_m, pose.y_m)
            return CompetitionOutput(
                self.state,
                CommandRequest(
                    CMD_YIELD_BACKOFF,
                    CMD_VALID,
                    -round(self.settings.yield_distance_m * 1000.0),
                    0,
                ),
                "超过设定时间定位不动，疑似碰撞或物资卡住，先退让",
                self.selected_batch,
                event="stuck_detected",
                motion_expected=True,
                stuck_phase="yield",
            )
        return None

    def _navigate(self, pose: PoseSnapshot, stm: StmSnapshot, now: float) -> CompetitionOutput:
        if self.selected_batch is None:
            self._set_state(CompetitionState.SEARCH, now)
            return CompetitionOutput(self.state, self._hold(), "没有锁定批次，返回搜索")
        target = self._target_point()
        if self._outside_field(pose):
            self._set_state(CompetitionState.FAULT, now)
            return CompetitionOutput(self.state, CommandRequest(CMD_ABORT, self._side_flags()), "超出场地边界，安全停车", self.selected_batch, event="boundary_fault")
        if self._boundary_risk(pose, target):
            self.resume_state = self.state
            self._set_state(CompetitionState.BOUNDARY_RECOVERY, now)
            return self._boundary_recovery_output(pose, now)
        if self.selected_batch.destination != "stash":
            if self._at_target(pose, target) or (stm.fresh and stm.distance_done and stm.mode == STM_MODE_NAVIGATE):
                self._set_state(CompetitionState.ENTER_SAFE_ZONE, now)
                return CompetitionOutput(
                    self.state,
                    CommandRequest(CMD_ENTER_SAFE_ZONE, self._side_flags() | (1 << 1) | (1 << 2), aux=round(self.settings.safe_heading_deg * 100.0) % 36000),
                    "到达安全区入口，进入投送复核",
                    self.selected_batch,
                    event="safe_zone_entry",
                )
        else:
            if self._at_target(pose, target) or (stm.fresh and stm.distance_done):
                self._set_state(CompetitionState.INITIAL_RELEASE, now)
                return CompetitionOutput(self.state, CommandRequest(CMD_RELEASE_BOTH, self._side_flags()), "到达临时藏物资点，释放整批物资", self.selected_batch, event="stash_arrived")
        return CompetitionOutput(
            self.state,
            self._navigation_command(pose, target),
            f"前往{('临时物资点' if self.selected_batch.destination == 'stash' else '本方安全区')}，持续更新剩余距离",
            self.selected_batch,
            motion_expected=True,
        )

    def _handle_search(
        self, vision: VisionSnapshot, stm: StmSnapshot, now: float
    ) -> CompetitionOutput:
        if not self._vision_fresh(vision, now):
            return CompetitionOutput(self.state, self._hold(), "视觉结果已超时，保持停车观察")
        if not self.first_common_delivered:
            candidate = self._choose_initial_green(vision)
            if candidate is not None:
                self.selected_batch = CargoBatch((candidate.track_id,), ("green_supply",), "material")
                self._set_state(CompetitionState.APPROACH, now)
                return CompetitionOutput(self.state, self._approach_command(candidate), "锁定首件单独普通物资", self.selected_batch, event="first_green_locked")
            visible = self._visible_cargo(vision)
            green_seen = any(
                item.class_name == "green_supply" and item.hits >= 2
                for item in visible
            )
            if (
                green_seen and
                stm.fresh and
                stm.mode == STM_MODE_SEARCH and
                not stm.gripper_closed and
                not self.cargo_recheck_pending
            ):
                if self.disperse_attempts < self.settings.disperse_limit:
                    self.disperse_attempts += 1
                    self._set_state(CompetitionState.DISPERSE, now)
                    return CompetitionOutput(self.state, CommandRequest(CMD_DISPERSE_PILE, self._side_flags()), "没有可单独取得的绿色物资，申请受控打散", event="disperse_start")
                return CompetitionOutput(self.state, self._hold(), "首件绿色物资暂不可安全单独取得，停车等待重新观察")
        else:
            batch = self._choose_next_batch(vision)
            if batch is not None:
                self.stash_checked = False
                self.selected_batch = batch
                self._set_state(CompetitionState.APPROACH, now)
                candidate = self._target_for_batch(vision)
                return CompetitionOutput(self.state, self._approach_command(candidate), f"锁定{batch.total_count}件{batch.destination}批次", batch, event="batch_locked")
            if self.search_empty_started_s is None:
                self.search_empty_started_s = now
            if self.stash_checked:
                self._set_state(CompetitionState.FINISHED, now)
                return CompetitionOutput(self.state, self._hold(), "临时物资点复查完成，没有剩余可搬运目标", event="all_targets_exhausted")
            if now - self.search_empty_started_s >= self.settings.search_empty_hold_s:
                self._set_state(CompetitionState.RETURN_STASH, now)
                return CompetitionOutput(self.state, self._hold(), "中心区域完成扫描，前往临时物资点复查", event="return_stash_start")
        return CompetitionOutput(self.state, self._hold(), "搜索并锁定合法目标")

    def _approach_command(self, candidate: TrackedCargo | None) -> CommandRequest:
        if candidate is None:
            return self._hold()
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
        self._update_delivery_observation(vision, now)
        if stm.fault and self.state != CompetitionState.FAULT:
            self._set_state(CompetitionState.FAULT, now)
            return CompetitionOutput(self.state, CommandRequest(CMD_ABORT, self._side_flags()), "F407报告故障，安全停车", event="stm_fault")

        stuck = self._stuck_override(vision, pose, stm, now)
        if stuck is not None:
            return stuck

        if self.state == CompetitionState.WAIT_START:
            if self._start_ready(pose, stm):
                self._set_state(CompetitionState.INITIAL_OBSERVE if not self.initial_stash_done else CompetitionState.SEARCH, now)
                return CompetitionOutput(self.state, self._hold(), "已离开出发区，开始中心观察", event="start_clear")
            return CompetitionOutput(self.state, self._hold(), "等待下位机完成出发并离开出发区")

        if self.state == CompetitionState.INITIAL_OBSERVE:
            pile = self._pile_batch(vision)
            if pile is not None:
                self.selected_batch = pile
                self._set_state(CompetitionState.INITIAL_APPROACH, now)
                candidate = self._target_for_batch(vision)
                return CompetitionOutput(self.state, self._approach_command(candidate), "锁定中心物资堆，准备临时转移", pile, event="initial_pile_locked")
            if now - self.state_started_s >= self.settings.initial_observe_s:
                self.initial_stash_done = True
                self._set_state(CompetitionState.SEARCH, now)
            return CompetitionOutput(self.state, self._hold(), "观察中心物资堆")

        if self.state == CompetitionState.INITIAL_APPROACH:
            candidate = self._target_for_batch(vision)
            if candidate is None:
                if self.target_lost_started_s is None:
                    self.target_lost_started_s = now
                if now - self.target_lost_started_s > self.settings.target_loss_s:
                    self.selected_batch = None
                    self._set_state(CompetitionState.INITIAL_OBSERVE, now)
                    return CompetitionOutput(self.state, self._hold(), "物资堆暂时丢失，重新观察")
            elif stm.claw_visible:
                self._set_state(CompetitionState.CAPTURE_AUDIT, now)
                return self._audit_output(vision, now)
            return CompetitionOutput(self.state, self._approach_command(candidate), "靠近中心物资堆", self.selected_batch, motion_expected=True)

        if self.state == CompetitionState.SEARCH:
            self.search_empty_started_s = None if self._visible_cargo(vision) else self.search_empty_started_s
            return self._handle_search(vision, stm, now)

        if self.state == CompetitionState.DISPERSE:
            if stm.fresh and stm.mode == STM_MODE_DISPERSE_DONE:
                self._set_state(CompetitionState.SEARCH, now)
                return CompetitionOutput(self.state, self._hold(), "受控打散完成，重新搜索")
            if stm.fresh and stm.gripper_closed:
                return CompetitionOutput(
                    self.state,
                    CommandRequest(CMD_HOLD, self._side_flags()),
                    "夹爪状态异常，停止发送打散命令",
                    event="disperse_gripper_guard",
                )
            if not self._vision_fresh(vision, now):
                return CompetitionOutput(
                    self.state,
                    CommandRequest(CMD_HOLD, self._side_flags()),
                    "打散期间视觉帧超时，暂停打散等待恢复",
                    event="disperse_visual_timeout",
                    motion_expected=True,
                )
            green_seen = any(
                item.class_name == "green_supply" and item.hits >= 2
                for item in self._visible_cargo(vision)
            )
            if not green_seen:
                return CompetitionOutput(
                    self.state,
                    CommandRequest(CMD_HOLD, self._side_flags()),
                    "打散许可条件暂未满足，等待重新看到首件绿色物资",
                    event="disperse_wait_green",
                    motion_expected=True,
                )
            return CompetitionOutput(self.state, CommandRequest(CMD_DISPERSE_PILE, self._side_flags()), "下位机执行小范围打散", motion_expected=True)

        if self.state == CompetitionState.APPROACH:
            if vision.danger_ahead:
                return self._start_detour(vision, now)
            candidate = self._target_for_batch(vision)
            if candidate is None:
                if self.target_lost_started_s is None:
                    self.target_lost_started_s = now
                if now - self.target_lost_started_s > self.settings.target_loss_s:
                    self.selected_batch = None
                    self._set_state(CompetitionState.SEARCH, now)
                    return CompetitionOutput(self.state, self._hold(), "锁定目标丢失，退回搜索")
            elif stm.claw_visible:
                self._set_state(CompetitionState.CAPTURE_AUDIT, now)
                return self._audit_output(vision, now)
            return CompetitionOutput(self.state, self._approach_command(candidate), "靠近锁定批次", self.selected_batch, motion_expected=True)

        if self.state == CompetitionState.CAPTURE_AUDIT:
            return self._audit_output(vision, now)

        if self.state == CompetitionState.GRAB:
            if stm.fresh and stm.gripper_closed:
                self.cargo_recheck_pending = False
                self._set_state(CompetitionState.INITIAL_STASH_NAV if self.selected_batch and self.selected_batch.initial_stash else CompetitionState.NAVIGATE, now)
                self.delivery_outside_seen = False
                self.delivery_inside_hits = 0
                self.delivery_visual_confirmed = False
                return self._navigate(pose, stm, now)
            return CompetitionOutput(self.state, CommandRequest(CMD_GRAB_CONFIRMED, self._side_flags()), "等待下位机完成合爪", self.selected_batch)

        if self.state == CompetitionState.INVALID_RELEASE:
            audit = vision.capture_audit or CargoAudit()
            expected_mode = {
                "left": STM_MODE_RELEASE_LEFT_DONE,
                "right": STM_MODE_RELEASE_RIGHT_DONE,
                "both": STM_MODE_RELEASE_BOTH_DONE,
            }[self.invalid_release_side]
            if stm.fresh and stm.mode == expected_mode:
                self._set_state(CompetitionState.INVALID_BACKOFF, now)
                return CompetitionOutput(
                    self.state,
                    CommandRequest(CMD_YIELD_BACKOFF, CMD_VALID, -round(self.settings.yield_distance_m * 1000.0), 0),
                    (
                        "异常侧已释放，后退脱离后复审"
                        if not self.invalid_release_final
                        else "非法组合已双侧释放，后退脱离"
                    ),
                    audit=audit,
                    event="invalid_release_done",
                    motion_expected=True,
                )
            return self._invalid_release_output(audit, now)

        if self.state == CompetitionState.INVALID_BACKOFF:
            if stm.fresh and stm.mode == STM_MODE_YIELD_DONE:
                if self.invalid_release_final:
                    self.selected_batch = None
                    self.cargo_recheck_pending = False
                    self._set_state(CompetitionState.SEARCH, now)
                    return CompetitionOutput(self.state, self._hold(), "复审仍非法，双爪物资已释放并后退，退回搜索")
                self.cargo_recheck_pending = True
                self._set_state(CompetitionState.CAPTURE_AUDIT, now)
                self.audit_recheck_frame_floor = vision.frame_sequence if vision.frame_sequence > 0 else None
                self.audit_recheck_started_s = now
                return self._audit_output(vision, now)
            return CompetitionOutput(
                self.state,
                CommandRequest(CMD_YIELD_BACKOFF, CMD_VALID, -round(self.settings.yield_distance_m * 1000.0), 0),
                "后退脱离非法物资",
                motion_expected=True,
            )

        if self.state == CompetitionState.BOUNDARY_RECOVERY:
            limit = FIELD_HALF_M - 0.130 - self.settings.boundary_margin_m
            if pose.valid and abs(pose.x_m) <= limit and abs(pose.y_m) <= limit:
                self._set_state(self.resume_state or CompetitionState.SEARCH, now)
                return CompetitionOutput(self.state, self._hold(), "已回到场地安全区域，恢复原任务")
            if now - self.state_started_s >= 3.0:
                self._set_state(CompetitionState.FAULT, now)
                return CompetitionOutput(self.state, CommandRequest(CMD_ABORT, self._side_flags()), "场地边缘恢复失败，安全停车", self.selected_batch, event="boundary_recovery_fault")
            return self._boundary_recovery_output(pose, now)

        if self.state in {CompetitionState.INITIAL_STASH_NAV, CompetitionState.NAVIGATE, CompetitionState.RETURN_STASH}:
            if vision.danger_ahead and not (self.selected_batch and self.selected_batch.initial_stash):
                return self._start_detour(vision, now)
            if self.state == CompetitionState.RETURN_STASH and self.selected_batch is None:
                target = self.settings.stash_point
                if self._at_target(pose, target) or (stm.fresh and stm.distance_done):
                    self.stash_checked = True
                    self._set_state(CompetitionState.SEARCH, now)
                    return CompetitionOutput(self.state, self._hold(), "已到临时物资点，开始复查")
                return CompetitionOutput(self.state, self._navigation_command(pose, target), "前往临时物资点复查", motion_expected=True)
            return self._navigate(pose, stm, now)

        if self.state == CompetitionState.INITIAL_RELEASE:
            if stm.fresh and stm.mode == STM_MODE_RELEASE_BOTH_DONE:
                self.initial_stash_done = True
                self.selected_batch = None
                self._set_state(CompetitionState.RETURN_CENTER, now)
                return self._return_center_output(pose, now, "临时物资已放下，返回中心寻找首件绿色")
            return CompetitionOutput(self.state, CommandRequest(CMD_RELEASE_BOTH, self._side_flags()), "释放临时物资堆", self.selected_batch)

        if self.state == CompetitionState.DETOUR:
            if stm.fresh and stm.mode == STM_MODE_LANE_DONE:
                self._set_state(self.resume_state or CompetitionState.NAVIGATE, now)
                return CompetitionOutput(self.state, self._hold(), "换道完成，恢复原任务路线", self.selected_batch, event="detour_done")
            return CompetitionOutput(self.state, CommandRequest(CMD_CHANGE_LANE, CMD_VALID, round(self.detour_lateral_m * 1000.0), 0), "执行危险目标换道", self.selected_batch, motion_expected=True)

        if self.state == CompetitionState.ENTER_SAFE_ZONE:
            if stm.fresh and stm.mode == STM_MODE_RAM_VERIFY:
                self._set_state(CompetitionState.DELIVERY_VERIFY, now)
            return CompetitionOutput(self.state, CommandRequest(CMD_ENTER_SAFE_ZONE, self._side_flags() | (1 << 1) | (1 << 2), aux=round(self.settings.safe_heading_deg * 100.0) % 36000), "进入安全区并等待视觉确认", self.selected_batch)

        if self.state == CompetitionState.DELIVERY_VERIFY:
            if self.delivery_visual_confirmed and stm.fresh and stm.mode == STM_MODE_RAM_VERIFY:
                self._set_state(CompetitionState.TASK_COMPLETE, now)
                self.delivery_count += 1
                if self.selected_batch and "green_supply" in self.selected_batch.classes:
                    self.first_common_delivered = True
                return CompetitionOutput(self.state, CommandRequest(CMD_TASK_COMPLETE, self._side_flags()), "视觉确认物资已由区外进入安全区，通知下位机完成", self.selected_batch, event="delivery_confirmed")
            return CompetitionOutput(self.state, CommandRequest(CMD_ENTER_SAFE_ZONE, self._side_flags() | (1 << 1) | (1 << 2), aux=round(self.settings.safe_heading_deg * 100.0) % 36000), f"等待安全区视觉确认 {self.delivery_inside_hits}/{self.settings.delivery_visual_frames}", self.selected_batch)

        if self.state == CompetitionState.TASK_COMPLETE:
            if stm.fresh and stm.mode in {STM_MODE_EXIT_SAFE_ZONE, STM_MODE_FACE_FIELD_CENTER}:
                self.selected_batch = None
                self._set_state(CompetitionState.RETURN_CENTER, now)
                return self._return_center_output(pose, now, "下位机已完成投送，返回中心区域")
            if stm.fresh and stm.mode == STM_MODE_SEARCH:
                self.selected_batch = None
                self._set_state(CompetitionState.SEARCH, now)
                return CompetitionOutput(self.state, self._hold(), "下位机已完成投送并进入SEARCH")
            return CompetitionOutput(self.state, CommandRequest(CMD_TASK_COMPLETE, self._side_flags()), "等待下位机张爪并退出安全区")

        if self.state == CompetitionState.RETURN_CENTER:
            if stm.fresh and stm.mode == STM_MODE_SEARCH:
                self._set_state(CompetitionState.SEARCH, now)
                return CompetitionOutput(self.state, self._hold(), f"回到中心搜索区，已完成{self.delivery_count}件")
            return self._return_center_output(pose, now, "持续返回中心")

        if self.state == CompetitionState.FINISHED:
            return CompetitionOutput(self.state, self._hold(), "任务完成，保持安全停车")

        if self.state == CompetitionState.FAULT:
            return CompetitionOutput(self.state, CommandRequest(CMD_ABORT, self._side_flags()), "任务故障，安全停车")

        return CompetitionOutput(self.state, self._hold(), "未处理状态，保持停车")
