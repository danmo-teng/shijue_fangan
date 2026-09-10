#!/usr/bin/env python3
"""Run the independent complete rescue competition flow.

Localization is still owned by ``localization`` and camera/YOLO processing is
reused from the existing RDK X5 program.  This runner owns only the new
competition strategy and writes its command frames through the existing
atomic command-file relay.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
import re
import signal
import subprocess
import sys
import threading
import time
import traceback
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = ROOT.parent
VISION_ROOT = PROJECT_ROOT / "vision"
sys.path.insert(0, str(VISION_ROOT))
sys.path.insert(0, str(ROOT))

from rescue_vision.camera import LatestFrameCamera, resolve_camera_device  # noqa: E402
from rescue_vision.config import load_config, require_native_resolution  # noqa: E402
from rescue_vision.localizer import GroundLocalizer  # noqa: E402
from rescue_vision.models import Detection, TrackState  # noqa: E402
from rescue_vision.mission_protocol import write_command_frame  # noqa: E402
from rescue_vision.tracker import MultiFrameTracker  # noqa: E402
from rescue_vision.vision_protocol import IMAGE_HEIGHT, IMAGE_WIDTH, config_frame  # noqa: E402
from rescue_vision.vse import VseScaler  # noqa: E402
from run_yolo_x5 import (  # noqa: E402
    DEFAULT_LABELS,
    DEFAULT_MODEL,
    DEFAULT_YOLO_SCORE_THRESHOLD,
    X5YoloV8,
    load_labels,
)

from protocol import CMD_ABORT, CMD_HOLD  # noqa: E402
from state_machine import (  # noqa: E402
    CARGO_CLASSES,
    CargoAudit,
    CommandRequest,
    CompetitionMission,
    CompetitionOutput,
    CompetitionSettings,
    CompetitionState,
    PoseSnapshot,
    StmSnapshot,
    TrackedCargo,
    VisionSnapshot,
)


TERMINATION_ABORT_TIMEOUT_S = 1.0


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RDK X5完整智能救援比赛流程")
    parser.add_argument("--detector", choices=("yolo",), default="yolo")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--decoder", choices=("jpu", "software"), default="jpu")
    parser.add_argument("--camera-fps", type=int, default=180)
    parser.add_argument("--decode-fps", type=float, default=60.0)
    parser.add_argument("--vision-fps", type=float, default=50.0)
    parser.add_argument("--planner-fps", type=float, default=50.0)
    parser.add_argument("--preprocess", choices=("auto", "vse", "cpu"), default="auto")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--labels", type=Path, default=DEFAULT_LABELS)
    parser.add_argument("--score-thres", type=float, default=DEFAULT_YOLO_SCORE_THRESHOLD)
    parser.add_argument("--nms-thres", type=float, default=0.45)
    parser.add_argument("--priority", type=int, default=0)
    parser.add_argument("--bpu-cores", type=int, nargs="+", default=[0, 1])
    parser.add_argument("--startup-timeout", type=float, default=60.0)
    parser.add_argument("--session", type=Path, default=PROJECT_ROOT / "rescue_map/runtime/session.json")
    parser.add_argument("--pose", type=Path, default=PROJECT_ROOT / "rescue_map/runtime/localization_result.json")
    parser.add_argument("--stm-status", type=Path, default=PROJECT_ROOT / "rescue_map/runtime/stm32_status.json")
    parser.add_argument("--command-file", type=Path, default=PROJECT_ROOT / "rescue_map/runtime/uart_command.bin")
    parser.add_argument("--config", type=Path, default=VISION_ROOT / "config/rescue_vision.json")
    parser.add_argument("--homography", type=Path, default=VISION_ROOT / "config/homography.txt")
    parser.add_argument("--diagnostics", type=Path, default=PROJECT_ROOT / "rescue_map/runtime/competition_diagnostics.json")
    parser.add_argument("--detections-log", type=Path, default=PROJECT_ROOT / "rescue_map/runtime/competition_detections.jsonl")
    parser.add_argument("--events-log", type=Path, default=PROJECT_ROOT / "rescue_map/runtime/competition_events.jsonl")
    parser.add_argument("--window-mode", choices=("fullscreen", "normal"), default="normal")
    parser.add_argument("--display-fps", type=float, default=10.0)
    parser.add_argument(
        "--detection-log-fps",
        type=float,
        default=10.0,
        help="识别日志采样频率；识别仍按vision-fps运行",
    )
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; 0 runs until stopped")
    parser.add_argument(
        "--disable-initial-stash",
        action="store_true",
        help="debug only: skip the configured opening temporary-relocation strategy",
    )
    parser.add_argument("--yield-distance-mm", type=float, default=250.0)
    parser.add_argument("--camera-retries", type=int, default=1)
    return parser.parse_args()


def load_json(path: Path) -> dict | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError):
        return None


def load_pose(path: Path) -> PoseSnapshot:
    data = load_json(path)
    if not data:
        return PoseSnapshot()
    try:
        timestamp = int(data["timestamp_monotonic_ns"])
        age_ms = max(0.0, (time.monotonic_ns() - timestamp) / 1_000_000.0)
        pose = data["pose"]
        if "yaw_deg" in pose:
            yaw = float(pose["yaw_deg"])
        else:
            yaw = math.degrees(float(pose["yaw_rad"]))
        navigation = data.get("navigation", {})
        wheel_progress = navigation.get("wheel_progress_m")
        return PoseSnapshot(
            valid=data.get("quality") in {"GOOD", "DEGRADED"} and age_ms <= 250.0,
            x_m=float(pose["x_m"]),
            y_m=float(pose["y_m"]),
            yaw_deg=yaw % 360.0,
            age_ms=age_ms,
            wheel_progress_m=(
                None if wheel_progress is None else float(wheel_progress)
            ),
        )
    except (KeyError, TypeError, ValueError):
        return PoseSnapshot()


def load_stm(path: Path) -> StmSnapshot:
    data = load_json(path)
    if not data:
        return StmSnapshot()
    try:
        relay = data.get("relay", {})
        if not isinstance(relay, dict):
            relay = {}
        timestamp = int(data["timestamp_monotonic_ns"])
        age_ms = max(0.0, (time.monotonic_ns() - timestamp) / 1_000_000.0)
        return StmSnapshot(
            mode=int(data.get("mode", 0)),
            flags=int(data.get("flags", 0)),
            age_ms=age_ms,
            fault_code=int(data.get("fault_code", 0)),
            acknowledged_sequence=int(data.get("acknowledged_sequence", 0)),
            relay_mission_tx_frames=int(
                relay.get("mission_tx_frames", relay.get("tx_frames", 0))
            ),
            relay_last_mission_command=(
                None
                if relay.get("last_mission_command") is None
                else int(relay["last_mission_command"])
            ),
            relay_last_mission_sequence=(
                None
                if relay.get("last_mission_sequence") is None
                else int(relay["last_mission_sequence"])
            ),
            relay_last_mission_payload=tuple(
                int(value) for value in relay.get("last_mission_payload", [])
            ),
            relay_last_mission_tx_monotonic_ns=int(
                relay.get("last_mission_tx_monotonic_ns", 0)
            ),
            relay_last_mission_tx_age_ms=float(
                relay.get("last_mission_tx_age_ms", float("inf"))
            ),
            relay_tx_frames=int(relay.get("tx_frames", 0)),
            relay_tx_errors=int(relay.get("tx_errors", 0)),
            relay_last_sequence=(
                None
                if relay.get("last_sequence") is None
                else int(relay["last_sequence"])
            ),
            relay_last_tx_age_ms=float(
                relay.get("last_tx_age_ms", float("inf"))
            ),
        )
    except (KeyError, TypeError, ValueError):
        return StmSnapshot()


def make_detection(item, localizer: GroundLocalizer) -> Detection:
    x, y, width, height = item.bbox
    bottom = (x + width * 0.5, y + height)
    ground = localizer.image_to_ground(bottom)
    return Detection(
        class_name=item.class_name,
        confidence=item.confidence,
        bbox=item.bbox,
        bottom_point=bottom,
        ground_xy_mm=ground,
        size_mm=None,
        contour=np.empty((0, 1, 2), dtype=np.int32),
    )


def bbox_center_px(bbox: tuple[int, int, int, int]) -> tuple[int, int]:
    """Return the image-space centre without depending on detector wrappers."""
    x, y, width, height = bbox
    return x + width // 2, y + height // 2


def safe_bbox_from_detections(detections, safe_class: str):
    safe = max(
        (item for item in detections if item.class_name == safe_class),
        key=lambda item: item.bbox[2] * item.bbox[3],
        default=None,
    )
    return None if safe is None else safe.bbox


def tracked_cargo(tracks, safe_bbox) -> tuple[TrackedCargo, ...]:
    result: list[TrackedCargo] = []
    for track in tracks:
        if track.class_name not in CARGO_CLASSES:
            continue
        detection = track.last_detection
        relative = None
        if detection.ground_xy_mm is not None:
            relative = (
                float(detection.ground_xy_mm[0]) / 1000.0,
                float(detection.ground_xy_mm[1]) / 1000.0,
            )
        inside = False
        if safe_bbox is not None:
            tx, ty, tw, th = detection.bbox
            sx, sy, sw, sh = safe_bbox
            center_inside = (
                sx <= tx + tw * 0.5 <= sx + sw and
                sy <= ty + th * 0.5 <= sy + sh
            )
            overlap_width = max(0, min(tx + tw, sx + sw) - max(tx, sx))
            overlap_height = max(0, min(ty + th, sy + sh) - max(ty, sy))
            overlap_ratio = (
                overlap_width * overlap_height / max(1, tw * th)
            )
            overlaps = overlap_ratio >= 0.25
            inside = center_inside or overlaps
        result.append(
            TrackedCargo(
                track_id=track.track_id,
                class_name=track.class_name,
                confidence=float(track.confidence),
                bbox=detection.bbox,
                relative_xy_m=relative,
                hits=track.hits,
                misses=track.misses,
                visible=track.misses == 0 and track.state != TrackState.LOST,
                inside_safe_zone=inside,
            )
        )
    return tuple(result)


def audit_from_cargo(
    items: tuple[TrackedCargo, ...], selected_ids: set[int] | None = None
) -> CargoAudit | None:
    visible = [item for item in items if item.visible]
    if not visible:
        return None
    left = [item for item in visible if item.center_px[0] < IMAGE_WIDTH // 2]
    right = [item for item in visible if item.center_px[0] >= IMAGE_WIDTH // 2]

    def side_class(values: list[TrackedCargo]) -> str:
        names = {item.class_name for item in values}
        if not names:
            return ""
        if names.issubset({"green_supply", "core_black"}) and len(names) > 1:
            return "mixed_material"
        return next(iter(names)) if len(names) == 1 else "unknown"

    all_names = {item.class_name for item in visible}
    has_danger = "danger_cyan" in all_names
    has_injury = "injured_orange" in all_names
    # Match F407's hard rule: any injury count other than exactly one is not
    # a legal single-cargo audit, even during the temporary initial stash.
    injury_mixed = has_injury and len(visible) != 1
    left_name, right_name = side_class(left), side_class(right)
    left_invalid = left_name in {"danger_cyan", "unknown"} or (
        any(item.class_name == "injured_orange" for item in left) and
        any(item.class_name in {"green_supply", "core_black"} for item in left)
    )
    right_invalid = right_name in {"danger_cyan", "unknown"} or (
        any(item.class_name == "injured_orange" for item in right) and
        any(item.class_name in {"green_supply", "core_black"} for item in right)
    )
    return CargoAudit(
        left_class=left_name,
        right_class=right_name,
        left_count=len(left),
        right_count=len(right),
        total_count=len(visible),
        danger_present=has_danger,
        unknown_present="unknown" in {left_name, right_name},
        injury_mixed=injury_mixed,
        left_invalid=left_invalid,
        right_invalid=right_invalid,
        left_selected_count=sum(
            item.track_id in (selected_ids or set()) for item in left
        ),
        right_selected_count=sum(
            item.track_id in (selected_ids or set()) for item in right
        ),
    )


def danger_ahead(
    cargo: tuple[TrackedCargo, ...],
    target: TrackedCargo | None,
) -> tuple[bool, str]:
    candidates = [
        item for item in cargo
        if item.class_name == "danger_cyan" and item.visible and item.hits >= 2
    ]
    if not candidates:
        return False, "unknown"
    ahead = []
    target_cx = IMAGE_WIDTH * 0.5 if target is None else target.center_px[0]
    target_width = 0 if target is None else target.bbox[2]
    for item in candidates:
        x, y, width, height = item.bbox
        cx = x + width * 0.5
        cy = y + height * 0.5
        corridor_padding = max(width, target_width) * 0.5
        corridor_min = min(IMAGE_WIDTH * 0.5, target_cx) - corridor_padding
        corridor_max = max(IMAGE_WIDTH * 0.5, target_cx) + corridor_padding
        if cy >= IMAGE_HEIGHT * 0.42 and corridor_min <= cx <= corridor_max:
            ahead.append((cx, cy))
    if not ahead:
        return False, "unknown"
    cx = sum(point[0] for point in ahead) / len(ahead)
    return True, "left" if cx < IMAGE_WIDTH * 0.5 else "right"


def make_vision_snapshot(
    detections,
    tracks,
    localizer: GroundLocalizer,
    safe_bbox,
    stm: StmSnapshot,
    mission: CompetitionMission,
    frame_sequence: int,
    safe_zone_filter_blocked: bool = False,
) -> VisionSnapshot:
    cargo = tracked_cargo(tracks, safe_bbox)
    capture = tuple(item for item in cargo if item.visible and item.center_px[1] >= IMAGE_HEIGHT * 0.25)
    selected = mission.selected_batch
    selected_ids = set(selected.track_ids) if selected is not None else set()
    selected_classes = set(selected.classes) if selected is not None else set()
    audit = audit_from_cargo(capture, selected_ids) if stm.claw_visible else None
    target_candidates = [
        item for item in cargo
        if item.visible and (
            item.track_id == mission.locked_target_track_id or
            item.track_id in selected_ids
        )
    ]
    approach_target = max(target_candidates, key=lambda item: item.area_px, default=None)
    exact_delivery_items = [
        item for item in cargo
        if item.visible and item.track_id in selected_ids
    ]
    same_class_items = [
        item for item in cargo
        if item.visible and item.class_name in selected_classes
    ]
    delivery_items = (
        exact_delivery_items
        if exact_delivery_items else
        same_class_items if len(same_class_items) == 1 else []
    )
    delivery_inside_ids = tuple(
        item.track_id for item in delivery_items if item.inside_safe_zone
    )
    delivery_outside_ids = tuple(
        item.track_id for item in delivery_items if not item.inside_safe_zone
    )
    delivery_inside = bool(delivery_inside_ids)
    delivery_outside = bool(delivery_outside_ids)
    danger, side = danger_ahead(cargo, approach_target)
    return VisionSnapshot(
        frame_sequence=frame_sequence,
        observed_monotonic_s=time.monotonic(),
        cargo=cargo,
        capture_cargo=capture,
        safe_bbox=safe_bbox,
        danger_ahead=danger,
        danger_side=side,
        capture_audit=audit,
        delivery_target_found=bool(delivery_items),
        delivery_target_inside_safe_zone=delivery_inside,
        delivery_target_outside_safe_zone=delivery_outside,
        delivery_target_inside_track_ids=delivery_inside_ids,
        delivery_target_outside_track_ids=delivery_outside_ids,
        safe_zone_filter_blocked=safe_zone_filter_blocked,
    )


def detection_log_record(
    frame_sequence: int,
    timing: dict[str, float],
    detections: list[Detection],
    tracks: tuple[TrackedCargo, ...],
) -> dict:
    """Build a JSON-safe sampled frame record for the offline analysis logs."""
    return {
        "frame_sequence": frame_sequence,
        "timing_ms": timing,
        "detections": [
            {
                "class": item.class_name,
                "confidence": round(float(item.confidence), 4),
                "bbox": list(item.bbox),
                "center": list(bbox_center_px(item.bbox)),
                "ground_xy_mm": (
                    None if item.ground_xy_mm is None else list(item.ground_xy_mm)
                ),
            }
            for item in detections
        ],
        "tracks": [
            {
                "id": item.track_id,
                "class": item.class_name,
                "confidence": item.confidence,
                "bbox": list(item.bbox),
                "relative_xy_m": item.relative_xy_m,
                "hits": item.hits,
                "misses": item.misses,
                "visible": item.visible,
            }
            for item in tracks
        ],
    }


class JsonlLog:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.lock = threading.Lock()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text("", encoding="utf-8")

    def write(self, event: str, data: dict) -> None:
        record = {
            "timestamp_monotonic_ns": time.monotonic_ns(),
            "event": event,
            **data,
        }
        with self.lock:
            with self.path.open("a", encoding="utf-8") as output:
                output.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")


def write_atomic_json(path: Path, document: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def pose_dict(pose: PoseSnapshot) -> dict:
    return {
        "valid": pose.valid,
        "x_m": pose.x_m,
        "y_m": pose.y_m,
        "yaw_deg": pose.yaw_deg,
        "age_ms": pose.age_ms if math.isfinite(pose.age_ms) else None,
        "wheel_progress_m": pose.wheel_progress_m,
    }


def stm_dict(stm: StmSnapshot) -> dict:
    return {
        "mode": stm.mode,
        "flags": stm.flags,
        "age_ms": stm.age_ms if math.isfinite(stm.age_ms) else None,
        "fault_code": stm.fault_code,
        "acknowledged_sequence": stm.acknowledged_sequence,
        "claw_visible": stm.claw_visible,
        "gripper_closed": stm.gripper_closed,
        "motors_active": stm.motors_active,
        "distance_done": stm.distance_done,
        "relay_tx_frames": stm.relay_tx_frames,
        "relay_tx_errors": stm.relay_tx_errors,
        "relay_last_sequence": stm.relay_last_sequence,
        "relay_last_tx_age_ms": (
            stm.relay_last_tx_age_ms
            if math.isfinite(stm.relay_last_tx_age_ms) else None
        ),
        "relay_mission_tx_frames": stm.relay_mission_tx_frames,
        "relay_last_mission_command": stm.relay_last_mission_command,
        "relay_last_mission_sequence": stm.relay_last_mission_sequence,
        "relay_last_mission_payload": list(stm.relay_last_mission_payload),
        "relay_last_mission_tx_monotonic_ns": stm.relay_last_mission_tx_monotonic_ns,
        "relay_last_mission_tx_age_ms": (
            stm.relay_last_mission_tx_age_ms
            if math.isfinite(stm.relay_last_mission_tx_age_ms) else None
        ),
    }


class CompetitionPlanner:
    def __init__(
        self,
        mission: CompetitionMission,
        pose_path: Path,
        stm_path: Path,
        command_path: Path,
        diagnostics_path: Path,
        events_log: JsonlLog,
        rate_hz: float,
    ) -> None:
        if rate_hz <= 0.0:
            raise ValueError("planner rate must be positive")
        self.mission = mission
        self.pose_path = pose_path
        self.stm_path = stm_path
        self.command_path = command_path
        self.diagnostics_path = diagnostics_path
        self.events_log = events_log
        self.period_s = 1.0 / rate_hz
        self.lock = threading.Lock()
        self.latest_vision = VisionSnapshot()
        self.latest_output = CompetitionOutput(mission.state, None, "等待第一帧识别")
        self.latest_pose = PoseSnapshot()
        self.latest_stm = StmSnapshot()
        self.sequence = 0
        self.running = False
        self.error: Exception | None = None
        self.last_state = mission.state
        self.paused = False
        self.diagnostics_period_s = 0.1
        self.last_diagnostics_s = -math.inf
        self.command_tx_suppression_reasons_logged: set[str] = set()
        self.termination_attempted = False
        self.termination_result: dict = {
            "attempted": False,
            "reason": "",
            "command_file_written": False,
            "transmitted": False,
            "confirmed": False,
            "confirmation_status": "unconfirmed",
            "timeout": False,
            "sequence": None,
        }
        self.thread = threading.Thread(target=self._run, name="competition-planner", daemon=True)

    def set_vision(self, vision: VisionSnapshot) -> None:
        with self.lock:
            self.latest_vision = vision

    def set_paused(self, paused: bool) -> None:
        with self.lock:
            self.paused = paused

    def snapshot(self):
        with self.lock:
            return self.latest_output, self.latest_pose, self.latest_stm

    def start(self) -> None:
        self.running = True
        self.thread.start()

    def stop(self, reason: str = "runner_exit") -> None:
        if self.termination_attempted:
            return
        self.termination_attempted = True
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)
        abort_command = CommandRequest(CMD_ABORT, self.mission._side_flags())
        abort_sequence = self.sequence
        abort_frame = abort_command.to_frame(abort_sequence)
        before = load_stm(self.stm_path)
        result = {
            "attempted": True,
            "reason": reason,
            "command_file_written": False,
            "transmitted": False,
            "confirmed": False,
            "timeout": False,
            "sequence": abort_sequence,
            "relay_baseline_mission_tx_frames": before.relay_mission_tx_frames,
            "last_mission_command": None,
            "last_mission_sequence": None,
            "last_mission_tx_monotonic_ns": 0,
            "stm_mode": before.mode,
            "stm_age_ms": before.age_ms if math.isfinite(before.age_ms) else None,
        }
        try:
            write_command_frame(self.command_path, abort_frame)
            self.sequence = (self.sequence + 1) & 0xFF
            result["command_file_written"] = True
        except OSError as error:
            result["write_error"] = str(error)
        if result["command_file_written"]:
            deadline = time.monotonic() + TERMINATION_ABORT_TIMEOUT_S
            while time.monotonic() < deadline:
                current = load_stm(self.stm_path)
                if (
                    current.relay_mission_tx_frames > before.relay_mission_tx_frames and
                    current.relay_last_mission_command == CMD_ABORT and
                    current.relay_last_mission_payload == tuple(abort_frame[4:12])
                ):
                    result["transmitted"] = True
                    result["last_mission_command"] = current.relay_last_mission_command
                    result["last_mission_sequence"] = current.relay_last_mission_sequence
                    result["last_mission_tx_monotonic_ns"] = current.relay_last_mission_tx_monotonic_ns
                    result["stm_mode"] = current.mode
                    result["stm_age_ms"] = current.age_ms if math.isfinite(current.age_ms) else None
                    if (
                        current.fresh and
                        current.relay_last_mission_sequence is not None and
                        current.acknowledged_sequence == current.relay_last_mission_sequence
                    ):
                        result["confirmed"] = True
                        break
                time.sleep(0.02)
            if not result["confirmed"]:
                result["timeout"] = True
        if result["confirmed"]:
            result["confirmation_status"] = "confirmed"
        self.termination_result = result
        self.events_log.write("termination_abort_result", result)

    def _hold_frame(self) -> bytes:
        from protocol import mission_frame
        return mission_frame(self.sequence, CMD_HOLD)

    def _publish(self, output: CompetitionOutput) -> None:
        if output.suppress_command_tx:
            return
        command = output.command
        if command is None:
            write_command_frame(self.command_path, self._hold_frame())
        else:
            write_command_frame(self.command_path, command.to_frame(self.sequence))
        self.sequence = (self.sequence + 1) & 0xFF

    def _output_for_cycle(
        self,
        vision: VisionSnapshot,
        pose: PoseSnapshot,
        stm: StmSnapshot,
        paused: bool,
        now: float,
    ) -> CompetitionOutput:
        if paused and self.mission.state in {
            CompetitionState.DISPERSE,
            CompetitionState.FAULT,
            CompetitionState.FINISHED,
            CompetitionState.DETOUR,
            CompetitionState.INITIAL_RELEASE,
            CompetitionState.INVALID_RELEASE,
            CompetitionState.INVALID_BACKOFF,
            CompetitionState.FIELD_STUCK_YIELD,
            CompetitionState.FIELD_STUCK_ESCAPE,
            CompetitionState.SAFE_ZONE_ESCAPE,
        }:
            return self.mission.step(vision, pose, stm, now)
        if paused:
            return CompetitionOutput(
                self.mission.state,
                CommandRequest(CMD_HOLD),
                "摄像头恢复中，保持车辆停车",
                tx_policy="camera_pause_hold",
                reason="camera_recovery",
            )
        return self.mission.step(vision, pose, stm, now)

    def _log_command_tx_suppression(self, output: CompetitionOutput) -> None:
        if not output.suppress_command_tx:
            return
        reason = output.suppression_reason or "unspecified"
        if reason in self.command_tx_suppression_reasons_logged:
            return
        self.events_log.write("command_tx_suppressed", {
            "state": output.state.value,
            "reason": reason,
            "tx_policy": output.tx_policy or "autonomous_recovery",
            "message": output.message,
        })
        self.command_tx_suppression_reasons_logged.add(reason)

    @staticmethod
    def _tx_policy(output: CompetitionOutput) -> str:
        if output.tx_policy:
            return output.tx_policy
        if output.suppress_command_tx:
            return "autonomous_recovery"
        if output.command is None:
            return "no_command"
        if output.command.opcode == CMD_HOLD:
            return "hold"
        if output.command.opcode == CMD_ABORT:
            return "fault_abort"
        return "normal_command"

    def _diagnostics(
        self,
        output: CompetitionOutput,
        pose: PoseSnapshot,
        stm: StmSnapshot,
        vision: VisionSnapshot,
        *,
        now: float | None = None,
    ) -> None:
        diagnostic_now = time.monotonic() if now is None else now
        batch = None
        if output.batch is not None:
            batch = {
                "track_ids": list(output.batch.track_ids),
                "classes": list(output.batch.classes),
                "destination": output.batch.destination,
                "initial_stash": output.batch.initial_stash,
            }
        audit = None if output.audit is None else asdict(output.audit)
        command = None if output.command is None else {
            "opcode": output.command.opcode,
            "flags": output.command.flags,
            "arg_a": output.command.arg_a,
            "arg_b": output.command.arg_b,
            "aux": output.command.aux,
            "audit": None if output.command.audit is None else asdict(output.command.audit),
        }
        write_atomic_json(self.diagnostics_path, {
            "schema_version": 1,
            "timestamp_monotonic_ns": time.monotonic_ns(),
            "state": output.state.value,
            "message": output.message,
            "event": output.event,
            "motion_expected": output.motion_expected,
            "stuck_phase": output.stuck_phase,
            "command_tx_suppressed": output.suppress_command_tx,
            "suppression_reason": output.suppression_reason,
            "upper_state": output.state.value,
            "stm_mode": stm.mode,
            "stm_age_ms": stm.age_ms if math.isfinite(stm.age_ms) else None,
            "command_opcode": None if output.command is None else output.command.opcode,
            "tx_policy": self._tx_policy(output),
            "reason": output.reason or output.suppression_reason or output.message,
            "selected_track_ids": list(self.mission.diagnostic_selected_track_ids()),
            "target_last_seen_age_ms": self.mission.target_last_seen_age_ms(diagnostic_now),
            "expected_stm_mode": list(
                output.expected_stm_modes or self.mission.expected_stm_modes()
            ),
            "stash_has_cargo": self.mission.stash_has_cargo,
            "stash_checked": self.mission.stash_checked,
            "danger_event_latched": self.mission.danger_event_latched,
            "detour_execution_seen": self.mission.detour_execution_seen,
            "safe_zone_exit_pending": self.mission.safe_zone_exit_pending,
            "stm_fault_waiting": self.mission.stm_fault_waiting,
            "delivery_outside_seen": self.mission.delivery_outside_seen,
            "delivery_inside_hits": self.mission.delivery_inside_hits,
            "delivery_window_frames": len(self.mission.delivery_window),
            "delivery_window_misses": self.mission._delivery_window_misses(),
            "delivery_window_started_s": self.mission.delivery_window_started_s,
            "delivery_observation_attempt": self.mission.delivery_observation_attempt,
            "delivery_observation_elapsed_s": self.mission._delivery_observation_elapsed(diagnostic_now),
            "delivery_visual_frame_age_ms": self.mission.delivery_last_frame_age_ms,
            "delivery_visual_confirmed": self.mission.delivery_visual_confirmed,
            "delivery_timeout_reason": self.mission.delivery_timeout_reason,
            "delivery_conflict_frames": self.mission.delivery_conflict_frames,
            "pending_audit_stable": (
                None if self.mission.pending_audit is None else self.mission.pending_audit.stable
            ),
            "pending_audit_tx_baseline": self.mission.pending_audit_tx_baseline,
            "pose": pose_dict(pose),
            "stm": stm_dict(stm),
            "vision": {
                "frame_sequence": vision.frame_sequence,
                "observed_monotonic_s": vision.observed_monotonic_s,
                "age_ms": (
                    None
                    if vision.observed_monotonic_s is None
                    else max(0.0, (diagnostic_now - vision.observed_monotonic_s) * 1000.0)
                ),
                "cargo_count": len(vision.cargo),
                "danger_ahead": vision.danger_ahead,
                "danger_side": vision.danger_side,
                "delivery_target_found": vision.delivery_target_found,
                "delivery_target_inside_safe_zone": vision.delivery_target_inside_safe_zone,
                "delivery_target_outside_safe_zone": vision.delivery_target_outside_safe_zone,
                "safe_zone_filter_blocked": vision.safe_zone_filter_blocked,
            },
            "initial_stash_done": self.mission.initial_stash_done,
            "first_common_delivered": self.mission.first_common_delivered,
            "first_fault_code": self.mission.first_fault_code,
            "delivery_count": self.mission.delivery_count,
            "disperse_attempts": self.mission.disperse_attempts,
            "cargo_recheck_pending": self.mission.cargo_recheck_pending,
            "invalid_release_side": self.mission.invalid_release_side,
            "invalid_release_final": self.mission.invalid_release_final,
            "batch": batch,
            "audit": audit,
            "command": command,
        })

    def _run(self) -> None:
        try:
            while self.running:
                started = time.monotonic()
                pose = load_pose(self.pose_path)
                stm = load_stm(self.stm_path)
                with self.lock:
                    vision = self.latest_vision
                    paused = self.paused
                output = self._output_for_cycle(vision, pose, stm, paused, started)
                self._log_command_tx_suppression(output)
                self._publish(output)
                state_changed = output.state != self.last_state
                if state_changed:
                    self.events_log.write("state_changed", {
                        "from": self.last_state.value,
                        "to": output.state.value,
                        "message": output.message,
                    })
                    self.last_state = output.state
                if output.event:
                    self.events_log.write(output.event, {
                        "state": output.state.value,
                        "message": output.message,
                    })
                diagnostics_now = time.monotonic()
                if (
                    state_changed or
                    output.event or
                    diagnostics_now - self.last_diagnostics_s >= self.diagnostics_period_s
                ):
                    self._diagnostics(output, pose, stm, vision, now=started)
                    self.last_diagnostics_s = diagnostics_now
                with self.lock:
                    self.latest_output = output
                    self.latest_pose = pose
                    self.latest_stm = stm
                elapsed = time.monotonic() - started
                time.sleep(max(0.001, self.period_s - elapsed))
        except Exception as error:  # pragma: no cover - surfaced by main loop
            self.error = error
            self.events_log.write("planner_error", {"error": str(error)})


WINDOW_NAME = "complete_rescue_competition"
NORMAL_WINDOW_SIZE = (420, 336)
WINDOW_MARGIN_PX = 12


def display_size() -> tuple[int, int]:
    try:
        output = subprocess.check_output(
            ["xrandr", "--current"],
            text=True,
            stderr=subprocess.DEVNULL,
        )
        match = re.search(r"current\s+(\d+)\s+x\s+(\d+)", output)
        if match:
            return int(match.group(1)), int(match.group(2))
    except (OSError, subprocess.SubprocessError, ValueError):
        pass
    return 1280, 1024


def configure_display_window(window_mode: str, *, create: bool = True) -> tuple[bool, tuple[int, int]]:
    """Create the task window without taking over the localization map."""
    if create:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    screen_width, screen_height = display_size()
    if window_mode == "fullscreen":
        cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        return True, (screen_width, screen_height)
    width, height = NORMAL_WINDOW_SIZE
    cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, width, height)
    cv2.moveWindow(
        WINDOW_NAME,
        max(0, screen_width - width - WINDOW_MARGIN_PX),
        max(0, screen_height - height - WINDOW_MARGIN_PX),
    )
    return True, NORMAL_WINDOW_SIZE


def draw_overlay(
    image: np.ndarray,
    vision: VisionSnapshot,
    output: CompetitionOutput,
    pose: PoseSnapshot,
    stm: StmSnapshot,
    fps: float,
    output_size: tuple[int, int],
) -> np.ndarray:
    """Render a low-cost preview; task/debug details stay in JSONL logs."""
    output_width, output_height = output_size
    view = cv2.resize(image, (output_width, output_height), interpolation=cv2.INTER_AREA)
    scale_x = output_width / float(IMAGE_WIDTH)
    scale_y = output_height / float(IMAGE_HEIGHT)
    selected_ids = set(output.batch.track_ids) if output.batch is not None else set()
    for item in vision.cargo:
        x, y, box_width, box_height = item.bbox
        if item.class_name == "danger_cyan":
            color = (0, 0, 255)
        elif item.track_id in selected_ids:
            color = (0, 255, 255)
        else:
            color = (0, 210, 0)
        x0 = round(x * scale_x)
        y0 = round(y * scale_y)
        x1 = round((x + box_width) * scale_x)
        y1 = round((y + box_height) * scale_y)
        cv2.rectangle(view, (x0, y0), (x1, y1), color, 2)
    if vision.safe_bbox is not None:
        x, y, box_width, box_height = vision.safe_bbox
        cv2.rectangle(
            view,
            (round(x * scale_x), round(y * scale_y)),
            (round((x + box_width) * scale_x), round((y + box_height) * scale_y)),
            (255, 0, 255),
            2,
        )
    return view


def validate_args(args: argparse.Namespace) -> None:
    require_native_resolution(IMAGE_WIDTH, IMAGE_HEIGHT)
    if not 0.0 < args.score_thres < 1.0 or not 0.0 < args.nms_thres < 1.0:
        raise ValueError("score-thres和nms-thres必须在0..1之间")
    if (
        args.vision_fps <= 0 or
        args.planner_fps <= 0 or
        args.display_fps <= 0 or
        args.detection_log_fps <= 0
    ):
        raise ValueError("FPS必须为正数")
    if args.startup_timeout <= 0:
        raise ValueError("startup timeout必须为正数")
    if args.yield_distance_mm <= 0:
        raise ValueError("退让距离必须为正数")
    if args.camera_retries < 0:
        raise ValueError("camera-retries不能为负数")


def main() -> int:
    args = arguments()
    validate_args(args)
    session = load_json(args.session)
    if not session or session.get("side") not in {"red", "blue"}:
        raise RuntimeError(f"请先在完整比赛地图窗口选择出发区和红蓝方：{args.session}")
    side = str(session["side"])
    start_zone = int(session.get("start_zone", 1))
    events = JsonlLog(args.events_log)
    detection_log = JsonlLog(args.detections_log)
    events.write("runner_starting", {
        "side": side,
        "start_zone": start_zone,
        "window_mode": args.window_mode,
        "display_fps": args.display_fps,
        "detection_log_fps": args.detection_log_fps,
        "vision_fps": args.vision_fps,
    })
    try:
        pose_deadline = time.monotonic() + args.startup_timeout
        startup_pose = load_pose(args.pose)
        while not startup_pose.valid:
            if time.monotonic() >= pose_deadline:
                raise RuntimeError("定位尚未就绪，完整比赛流程不会发送运动命令")
            time.sleep(0.05)
            startup_pose = load_pose(args.pose)
    except Exception as error:
        events.write("fatal_exception", {
            "error_type": type(error).__name__,
            "error": str(error),
            "traceback": traceback.format_exc(),
        })
        raise

    config = load_config(args.config)
    localizer = GroundLocalizer.load(args.homography, (IMAGE_WIDTH, IMAGE_HEIGHT))
    scaler: VseScaler | None = None
    try:
        detector = X5YoloV8(
            args.model,
            load_labels(args.labels),
            args.score_thres,
            args.nms_thres,
            args.priority,
            args.bpu_cores,
        )
        use_vse = args.preprocess in {"auto", "vse"} and args.decoder == "jpu"
        if args.preprocess == "vse" and args.decoder != "jpu":
            raise ValueError("VSE NV12路径要求--decoder jpu")
        if use_vse:
            scale = min(detector.input_width / IMAGE_WIDTH, detector.input_height / IMAGE_HEIGHT)
            content_width = max(2, int(round(IMAGE_WIDTH * scale)) // 2 * 2)
            content_height = max(2, int(round(IMAGE_HEIGHT * scale)) // 2 * 2)
            try:
                scaler = VseScaler(IMAGE_WIDTH, IMAGE_HEIGHT, content_width, content_height)
            except Exception as error:
                if args.preprocess == "vse":
                    raise
                print(f"警告：VSE初始化失败，回退CPU预处理：{error}")
    except Exception as error:
        events.write("fatal_exception", {
            "error_type": type(error).__name__,
            "error": str(error),
            "traceback": traceback.format_exc(),
        })
        if scaler is not None:
            scaler.close()
        raise

    mission = CompetitionMission(CompetitionSettings(
        side=side,
        start_zone=start_zone,
        initial_stash_enabled=not args.disable_initial_stash,
        yield_distance_m=args.yield_distance_mm / 1000.0,
    ))
    planner = CompetitionPlanner(
        mission,
        args.pose,
        args.stm_status,
        args.command_file,
        args.diagnostics,
        events,
        args.planner_fps,
    )

    # Configuration still goes through the existing atomic relay. The new
    # task runner never opens /dev/ttyS1 itself.
    red_side = side == "red"
    try:
        for sequence in range(3):
            write_command_frame(args.command_file, config_frame(sequence, 0x11 if red_side else 0x12, start_zone))
            time.sleep(0.04)
        events.write("competition_started", {
            "side": side,
            "start_zone": start_zone,
            "initial_stash_enabled": not args.disable_initial_stash,
            "score_threshold": args.score_thres,
            "display_fps": args.display_fps,
            "detection_log_fps": args.detection_log_fps,
            "vision_fps": args.vision_fps,
            "startup_pose": pose_dict(startup_pose),
        })
        camera_device = resolve_camera_device(args.device)
    except Exception as error:
        events.write("fatal_exception", {
            "error_type": type(error).__name__,
            "error": str(error),
            "traceback": traceback.format_exc(),
        })
        planner.stop("setup_exception")
        raise
    camera_decoder = args.decoder
    camera_output_format = "nv12" if scaler is not None else "bgr"
    camera: LatestFrameCamera | None = None
    running = True
    exit_reason = "setup_complete"
    window_created = False
    display_enabled = not args.no_display
    active_window_mode = args.window_mode
    display_output_size = NORMAL_WINDOW_SIZE
    started = time.monotonic()
    camera_recovery_pending = False
    next_camera_retry = started + 1.0
    next_detection_log = started

    def stop(received_signal, _frame):
        nonlocal running, exit_reason
        running = False
        exit_reason = f"signal_{received_signal}"

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    tracker = MultiFrameTracker(config)
    last_packet_id = 0
    last_inference_id = 0
    latest_image: np.ndarray | None = None
    latest_pixel_format = "bgr"
    latest_vision = VisionSnapshot()
    fps_started = started
    inference_frames = 0
    inference_total = 0
    display_next = started
    next_vision = started
    camera_restarts = 0
    recent_safe_bbox: tuple[int, int, int, int] | None = None
    safe_zone_missing_frames = 0

    def create_camera() -> LatestFrameCamera:
        return LatestFrameCamera(
            camera_device,
            IMAGE_WIDTH,
            IMAGE_HEIGHT,
            args.camera_fps,
            decoder=camera_decoder,
            decode_fps=args.decode_fps,
            output_format=camera_output_format,
        )

    try:
        # Start the planner in a paused/hold state first. Camera startup and
        # later recovery must never leave a previous motion command active.
        planner.set_paused(True)
        planner.start()
        try:
            camera = create_camera()
            camera.start()
            planner.set_paused(False)
        except Exception as error:
            if camera is not None:
                camera.stop()
            camera = None
            camera_recovery_pending = True
            next_camera_retry = started + 1.0
            if camera_decoder == "jpu":
                if scaler is not None:
                    scaler.close()
                    scaler = None
                camera_decoder = "software"
                camera_output_format = "bgr"
            events.write("camera_start_error", {
                "error": str(error),
                "decoder": camera_decoder,
                "recovery": "hold_and_retry",
            })
        if display_enabled:
            try:
                _, display_output_size = configure_display_window(active_window_mode)
                window_created = True
                events.write("display_started", {
                    "window_mode": active_window_mode,
                    "width": display_output_size[0],
                    "height": display_output_size[1],
                    "screen": list(display_size()),
                })
            except cv2.error as error:
                display_enabled = False
                events.write("display_disabled", {
                    "reason": "window_init_failed",
                    "error": str(error),
                })

        while running:
            if planner.error is not None:
                raise RuntimeError(f"比赛规划线程退出：{planner.error}")
            now = time.monotonic()
            if args.duration > 0.0 and now - started >= args.duration:
                exit_reason = "duration"
                break
            if camera is None:
                if now >= next_camera_retry:
                    events.write("camera_recovery_retry", {
                        "decoder": camera_decoder,
                    })
                    try:
                        camera = create_camera()
                        camera.start()
                    except Exception as retry_error:
                        if camera is not None:
                            camera.stop()
                        camera = None
                        next_camera_retry = now + 2.0
                        events.write("camera_recovery_retry_failed", {
                            "error": str(retry_error),
                            "decoder": camera_decoder,
                        })
                    else:
                        camera_recovery_pending = False
                        camera_restarts = 0
                        planner.set_paused(False)
                        last_packet_id = 0
                        last_inference_id = 0
                        next_vision = now
                        events.write("camera_recovery_restored", {
                            "decoder": camera_decoder,
                            "output_format": camera_output_format,
                        })
                time.sleep(0.01)
                continue
            assert camera is not None
            error = camera.check_error()
            if error:
                events.write("camera_error", {
                    "error": error,
                    "decoder": camera_decoder,
                    "retry": camera_restarts,
                })
                if camera_restarts < args.camera_retries:
                    planner.set_paused(True)
                    camera.stop()
                    camera_restarts += 1
                    if camera_decoder == "jpu":
                        # The default recovery path drops the optional VSE
                        # object and switches to software JPEG decoding so a
                        # transient JPU timeout does not terminate the task.
                        if scaler is not None:
                            scaler.close()
                            scaler = None
                        camera_decoder = "software"
                        camera_output_format = "bgr"
                    try:
                        camera = LatestFrameCamera(
                            camera_device,
                            IMAGE_WIDTH,
                            IMAGE_HEIGHT,
                            args.camera_fps,
                            decoder=camera_decoder,
                            decode_fps=args.decode_fps,
                            output_format=camera_output_format,
                        )
                        camera.start()
                    except Exception as restart_error:
                        events.write("camera_restart_failed", {
                            "error": str(restart_error),
                            "decoder": camera_decoder,
                        })
                        raise
                    last_packet_id = 0
                    last_inference_id = 0
                    next_vision = now
                    events.write("camera_restarted", {
                        "decoder": camera_decoder,
                        "output_format": camera_output_format,
                        "retry": camera_restarts,
                    })
                    planner.set_paused(False)
                    continue
                planner.set_paused(True)
                camera.stop()
                camera = None
                camera_recovery_pending = True
                next_camera_retry = now + 1.0
                events.write("camera_recovery_wait", {
                    "error": error,
                    "decoder": camera_decoder,
                    "action": "keep_task_alive_and_hold",
                })
                continue
            packet = camera.latest()
            if packet is not None:
                latest_image = packet.image
                latest_pixel_format = packet.pixel_format
                if packet.frame_id != last_packet_id:
                    last_packet_id = packet.frame_id
                if packet.frame_id != last_inference_id and now >= next_vision:
                    next_vision = now + 1.0 / args.vision_fps
                    last_inference_id = packet.frame_id
                    if packet.pixel_format == "nv12":
                        detections, timing = detector.infer_nv12(
                            packet.image, IMAGE_WIDTH, IMAGE_HEIGHT, scaler
                        )
                    else:
                        detections, timing = detector.infer(packet.image)
                    detection_objects = [make_detection(item, localizer) for item in detections]
                    tracks = tracker.update(detection_objects)
                    stm = load_stm(args.stm_status)
                    safe_class = "safe_red" if side == "red" else "safe_blue"
                    detected_safe_bbox = safe_bbox_from_detections(detections, safe_class)
                    if detected_safe_bbox is not None:
                        recent_safe_bbox = detected_safe_bbox
                        safe_zone_missing_frames = 0
                        safe_bbox = detected_safe_bbox
                        safe_zone_filter_blocked = False
                    elif recent_safe_bbox is not None and safe_zone_missing_frames < 2:
                        safe_zone_missing_frames += 1
                        safe_bbox = None
                        safe_zone_filter_blocked = True
                    else:
                        recent_safe_bbox = None
                        safe_zone_missing_frames = 3
                        safe_bbox = None
                        safe_zone_filter_blocked = False
                    latest_vision = make_vision_snapshot(
                        detections,
                        tracks,
                        localizer,
                        safe_bbox,
                        stm,
                        mission,
                        packet.frame_id,
                        safe_zone_filter_blocked,
                    )
                    planner.set_vision(latest_vision)
                    log_now = time.monotonic()
                    if log_now >= next_detection_log:
                        detection_log.write(
                            "frame",
                            detection_log_record(
                                packet.frame_id,
                                timing,
                                detection_objects,
                                latest_vision.cargo,
                            ),
                        )
                        next_detection_log = log_now + 1.0 / args.detection_log_fps
                    inference_frames += 1
                    inference_total += 1
            if now - fps_started >= 1.0:
                current_fps = inference_frames / max(now - fps_started, 1e-6)
                inference_frames = 0
                fps_started = now
            else:
                current_fps = inference_frames / max(now - fps_started, 1e-6)
            if display_enabled and now >= display_next:
                display_next = now + 1.0 / args.display_fps
                output, pose, current_stm = planner.snapshot()
                if latest_image is None:
                    latest_image = np.zeros(
                        (IMAGE_HEIGHT, IMAGE_WIDTH, 3), dtype=np.uint8
                    )
                shown_image = (
                    cv2.cvtColor(latest_image, cv2.COLOR_YUV2BGR_NV12)
                    if latest_pixel_format == "nv12" else latest_image
                )
                try:
                    shown = draw_overlay(
                        shown_image,
                        latest_vision,
                        output,
                        pose,
                        current_stm,
                        current_fps,
                        display_output_size,
                    )
                    cv2.imshow(WINDOW_NAME, shown)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), ord("Q")):
                        exit_reason = "user_q"
                        running = False
                    elif key == 27:
                        events.write("display_escape_ignored", {
                            "window_mode": active_window_mode,
                        })
                        if active_window_mode == "fullscreen":
                            active_window_mode = "normal"
                            _, display_output_size = configure_display_window(
                                active_window_mode, create=False
                            )
                    elif key in (ord("f"), ord("F")):
                        active_window_mode = (
                            "fullscreen" if active_window_mode == "normal" else "normal"
                        )
                        _, display_output_size = configure_display_window(
                            active_window_mode, create=False
                        )
                        events.write("display_mode_changed", {
                            "window_mode": active_window_mode,
                            "width": display_output_size[0],
                            "height": display_output_size[1],
                        })
                    visible = cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE)
                    if visible < 1:
                        display_enabled = False
                        events.write("display_closed", {
                            "action": "continue_task_without_window",
                        })
                except cv2.error as error:
                    display_enabled = False
                    events.write("display_disabled", {
                        "reason": "display_runtime_error",
                        "error": str(error),
                    })
            time.sleep(0.0005)
        if exit_reason == "setup_complete":
            exit_reason = "loop_stopped"
    except KeyboardInterrupt:
        exit_reason = "keyboard_interrupt"
        running = False
    except Exception as error:
        exit_reason = "fatal_exception"
        events.write("fatal_exception", {
            "error_type": type(error).__name__,
            "error": str(error),
            "traceback": traceback.format_exc(),
        })
        raise
    finally:
        planner.stop(exit_reason)
        if camera is not None:
            camera.stop()
        if scaler is not None:
            scaler.close()
        if window_created:
            try:
                cv2.destroyWindow(WINDOW_NAME)
            except cv2.error:
                pass
        events.write("competition_stopped", {
            "state": mission.state.value,
            "delivery_count": mission.delivery_count,
            "first_common_delivered": mission.first_common_delivered,
            "first_fault_code": mission.first_fault_code,
            "exit_reason": exit_reason,
            "tx_policy": "termination_abort",
            "termination_abort": planner.termination_result,
            "inference_frames": inference_frames,
            "inference_total": inference_total,
            "camera_decoder": camera_decoder,
            "camera_restarts": camera_restarts,
            "camera_recovery_pending": camera_recovery_pending,
            "display_enabled": display_enabled,
        })
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        raise SystemExit(1)
