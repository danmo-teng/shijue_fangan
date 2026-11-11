#!/usr/bin/env python3
"""Run the independent complete rescue competition flow.

Localization is still owned by ``localization`` and camera/YOLO processing is
reused from the existing RDK X5 program.  This runner owns only the new
competition strategy and writes its command frames through the existing
atomic command-file relay.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, replace
import json
import hashlib
import queue
from functools import lru_cache
import math
import re
import secrets
import signal
import socket
import subprocess
import sys
import threading
import time
import traceback
from pathlib import Path

import cv2
import numpy as np

from vision_replay import VisionReplay
from async_diagnostics import AsyncDiagnosticWriter

ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = ROOT.parent
VISION_ROOT = PROJECT_ROOT / "vision"
sys.path.insert(0, str(VISION_ROOT))
sys.path.insert(0, str(ROOT))

from rescue_vision.camera import LatestFrameCamera, resolve_camera_device  # noqa: E402
from rescue_vision.config import load_config, require_native_resolution  # noqa: E402
from rescue_vision.localizer import GroundLocalizer, ApproachRangeCalibration  # noqa: E402
from rescue_vision.models import Detection, TrackState  # noqa: E402
from rescue_vision.mission_protocol import write_command_frame  # noqa: E402
from rescue_vision.tracker import MultiFrameTracker  # noqa: E402
from rescue_vision.vision_protocol import IMAGE_HEIGHT, IMAGE_WIDTH, config_frame  # noqa: E402
from rescue_vision.yolo26 import DEFAULT_MODEL
from rescue_vision.yolo26_pipeline import Yolo26Pipeline
from rescue_vision.vse import VseScaler  # noqa: E402
from run_yolo_x5 import (  # noqa: E402
    DEFAULT_LABELS,
    DEFAULT_YOLO_SCORE_THRESHOLD,
    load_labels,
)

from protocol import (  # noqa: E402
    CMD_ABORT,
    CMD_GRAB_CONFIRMED,
    CMD_HOLD,
    CMD_NAVIGATE_WAYPOINT,
    CMD_PAUSE,
    CMD_APPROACH_TARGET,
    CMD_APPROACH_RANGE,
    CMD_CARGO_AUDIT,
    CMD_DISPERSE_PILE,
    CMD_RELEASE_LEFT,
    CMD_RELEASE_RIGHT,
    CMD_RELEASE_BOTH,
    CMD_RETURN_CENTER,
    CMD_TARGET_RIGHT,
    command_context_frame,
)
from capture_roi import (  # noqa: E402
    bbox_capture_roi_overlap_ratio,
    CaptureRois,
    bbox_in_capture_roi,
    capture_side_for_bbox,
    load_capture_rois,
)
from state_machine import (  # noqa: E402
    CARGO_CLASSES,
    SAFE_CORRIDOR_BLOCKED,
    SAFE_CORRIDOR_CLEAR,
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
    STM_MODE_TRANSPORT_AUDIT,
    STM_MODE_RECOVERY_WAIT,
    STM_MODE_NAVIGATE,
    STM_MODE_ALIGN_SAFE_ZONE,
    STM_MODE_FACE_FIELD_CENTER,
    STM_MODE_SEARCH,
    cargo_in_front_region,
)


TERMINATION_ABORT_TIMEOUT_S = 1.0
GREEN_SUPPLY_SCORE_THRESHOLD = DEFAULT_YOLO_SCORE_THRESHOLD


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
    parser.add_argument("--bpu-cores", type=int, nargs="+", default=[0])
    parser.add_argument("--startup-timeout", type=float, default=60.0)
    parser.add_argument("--session", type=Path, default=PROJECT_ROOT / "rescue_map/runtime/session.json")
    parser.add_argument("--pose", type=Path, default=PROJECT_ROOT / "rescue_map/runtime/localization_result.json")
    parser.add_argument("--stm-status", type=Path, default=PROJECT_ROOT / "rescue_map/runtime/stm32_status.json")
    parser.add_argument("--command-file", type=Path, default=PROJECT_ROOT / "rescue_map/runtime/uart_command.bin")
    parser.add_argument("--config", type=Path, default=VISION_ROOT / "config/rescue_vision.json")
    parser.add_argument("--homography", type=Path, default=VISION_ROOT / "config/homography.txt")
    parser.add_argument(
        "--capture-roi",
        type=Path,
        default=VISION_ROOT / "config/capture_roi.json",
    )
    parser.add_argument("--diagnostics", type=Path, default=PROJECT_ROOT / "rescue_map/runtime/competition_diagnostics.json")
    parser.add_argument("--detections-log", type=Path, default=PROJECT_ROOT / "rescue_map/runtime/competition_detections.jsonl")
    parser.add_argument("--events-log", type=Path, default=PROJECT_ROOT / "rescue_map/runtime/competition_events.jsonl")
    parser.add_argument("--reset-request", type=Path, help="启动窗口的整轮复位请求文件")
    parser.add_argument("--reset-session-token", default="", help="本轮启动窗口复位令牌")
    parser.add_argument("--operator-control", type=Path,
                        help="实体按键会话控制文件；与reset-session-token匹配的stop结束本轮运动并保留观察")
    parser.add_argument("--wait-for-operator", action="store_true", help="预热模型/相机，等匹配的本地开赛请求才发CONFIG")
    parser.add_argument("--warm-status", type=Path, help="一次实际推理完成及开赛状态")
    parser.add_argument("--localization-wheel-policy-socket", default="",
                        help="现有定位所有者的本地安全区轮式策略接口，@表示abstract socket")
    parser.add_argument("--localization-wheel-policy-session", default="")
    parser.add_argument("--window-mode", choices=("fullscreen", "normal"), default="normal")
    parser.add_argument("--display-fps", type=float, default=10.0)
    parser.add_argument(
        "--detection-log-fps",
        type=float,
        default=10.0,
        help="识别日志采样频率；识别仍按vision-fps运行",
    )
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--replay-native-frames", type=int, choices=range(4), default=0,
                        help="事件录像每片最多保存0–3张原生相机array（默认0关闭；不是模型输入张量）")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; 0 runs until stopped")
    parser.add_argument(
        "--disable-initial-stash",
        action="store_true",
        help="debug only: skip the configured opening temporary-relocation strategy",
    )
    parser.add_argument("--yield-distance-mm", type=float, default=250.0)
    parser.add_argument(
        "--safe-sweep-capture-offset-mm",
        type=float,
        default=150.0,
        help="兼容旧启动参数；视觉扫障不再使用此距离偏移",
    )
    parser.add_argument("--camera-retries", type=int, default=1)
    args = parser.parse_args()
    if bool(args.localization_wheel_policy_socket) != bool(args.localization_wheel_policy_session):
        parser.error("localization-wheel-policy-socket/session需要同时提供")
    return args


def load_json(path: Path) -> dict | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError):
        return None


def operator_stop_requested(path: Path | None, session_token: str) -> bool:
    """A STOP belongs to this explicit start only; it stays present until exit."""
    if path is None or not session_token:
        return False
    request = load_json(path)
    return bool(isinstance(request, dict) and
                request.get("session_token") == session_token and
                request.get("action") == "stop")



def operator_start_request(control_path: Path | None, pose_path: Path, session_token: str) -> dict | None:
    """Match the one local coordinate commit to this human start, no age check."""
    request = load_json(control_path) if control_path else None
    if (not session_token or not isinstance(request, dict) or request.get('session_token') != session_token
            or request.get('action') != 'start' or request.get('reference_token') != session_token
            or request.get('side') not in ('red', 'blue') or request.get('start_zone') not in (1, 2, 3, 4)):
        return None
    pose = load_json(pose_path) or {}
    if pose.get('reference', {}).get('round_token') != session_token:
        return None
    return dict(request, reference=pose['reference'])



def publish_start_config(command_path, side, start_zone, continue_start, on_sent):
    """Existing CONFIG sequence; remember the first write before any interruption."""
    for sequence in range(3):
        if not continue_start():
            return False
        write_command_frame(command_path, config_frame(sequence, 0x11 if side == 'red' else 0x12, start_zone))
        on_sent()
        time.sleep(.04)
    return continue_start()


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
            valid=data.get("quality") in {"GOOD", "DEGRADED"},
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
            camera_pitch_cdeg=int(data.get("camera_pitch_cdeg", 0)),
            age_ms=age_ms,
            fault_code=int(data.get("fault_code", 0)),
            acknowledged_sequence=int(data.get("acknowledged_sequence", 0)),
            context_valid=bool(data.get("context_valid", False)),
            task_id=int(data.get("task_id", 0)),
            action_id=int(data.get("action_id", 0)),
            accepted_command=int(data.get("accepted_command", 0)),
            action_status=int(data.get("action_status", 0)),
            rejected_action_id=int(data.get("rejected_action_id", 0)),
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


def bbox_overlap_ratio(
    target_bbox: tuple[int, int, int, int],
    region_bbox: tuple[int, int, int, int],
) -> float:
    tx, ty, tw, th = target_bbox
    sx, sy, sw, sh = region_bbox
    if tw <= 0 or th <= 0 or sw <= 0 or sh <= 0:
        return 0.0
    overlap_width = max(0, min(tx + tw, sx + sw) - max(tx, sx))
    overlap_height = max(0, min(ty + th, sy + sh) - max(ty, sy))
    return float(overlap_width * overlap_height) / float(tw * th)


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
            overlap_ratio = bbox_overlap_ratio(detection.bbox, safe_bbox)
            x, y, width, height = detection.bbox
            sx, sy, sw, sh = safe_bbox
            foot_inside = (sx <= x + width * 0.5 <= sx + sw and
                           sy <= y + height <= sy + sh)
            inside = (
                overlap_ratio >= 1.0
                if track.class_name == "core_black"
                else overlap_ratio > 0.80
            ) and foot_inside
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
    items: tuple[TrackedCargo, ...],
    selected_ids: set[int] | None = None,
    side_by_track: dict[int, str] | None = None,
    *,
    isolated_track_ids: set[int] | None = None,
) -> CargoAudit | None:
    visible = [item for item in items if item.visible]
    if not visible:
        return CargoAudit()
    side_by_track = side_by_track or {}
    left = [
        item for item in visible
        if side_by_track.get(item.track_id) == "left"
    ]
    right = [
        item for item in visible
        if side_by_track.get(item.track_id) == "right"
    ]
    unassigned = [
        item for item in visible
        if side_by_track.get(item.track_id) not in {"left", "right"}
    ]

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
    # Keep injury mixing in the audit for formal-flow validation. The
    # initial-stash path intentionally ignores category/count legality.
    injury_mixed = has_injury and len(visible) != 1
    left_name, right_name = side_class(left), side_class(right)

    def track_stability(values: list[TrackedCargo]) -> int:
        return max((item.hits for item in values), default=0)

    def nearest_distance(values: list[TrackedCargo]) -> float | None:
        distances = [item.distance_m for item in values if math.isfinite(item.distance_m)]
        return min(distances, default=None)

    def stable_track_id(values: list[TrackedCargo]) -> int | None:
        if not values:
            return None
        return min(
            values,
            key=lambda item: (-item.hits, item.distance_m, item.track_id),
        ).track_id

    left_invalid = left_name in {"danger_cyan", "unknown"} or (
        any(item.class_name == "injured_orange" for item in left) and
        any(item.class_name in {"green_supply", "core_black"} for item in left)
    )
    right_invalid = right_name in {"danger_cyan", "unknown"} or (
        any(item.class_name == "injured_orange" for item in right) and
        any(item.class_name in {"green_supply", "core_black"} for item in right)
    )
    isolated_track_ids = isolated_track_ids or set()
    return CargoAudit(
        class_counts=tuple(sorted(Counter(item.class_name for item in visible).items())),
        left_class_counts=tuple(sorted(Counter(item.class_name for item in left).items())),
        right_class_counts=tuple(sorted(Counter(item.class_name for item in right).items())),
        left_class=left_name,
        right_class=right_name,
        left_count=len(left),
        right_count=len(right),
        left_green_count=sum(
            item.class_name == "green_supply" for item in left
        ),
        right_green_count=sum(
            item.class_name == "green_supply" for item in right
        ),
        total_count=len(visible),
        danger_present=has_danger,
        unknown_present=bool(all_names - {"green_supply", "core_black", "injured_orange", "danger_cyan"}) or bool(unassigned),
        injury_mixed=injury_mixed,
        left_invalid=left_invalid,
        right_invalid=right_invalid,
        left_selected_count=sum(
            item.track_id in (selected_ids or set()) for item in left
        ),
        right_selected_count=sum(
            item.track_id in (selected_ids or set()) for item in right
        ),
        left_track_stability=track_stability(left),
        right_track_stability=track_stability(right),
        left_nearest_distance_m=nearest_distance(left),
        right_nearest_distance_m=nearest_distance(right),
        left_stable_track_id=stable_track_id(left),
        right_stable_track_id=stable_track_id(right),
        left_isolated_green=(len(left) == 1 and left_name == "green_supply" and
                             left[0].track_id in isolated_track_ids),
        right_isolated_green=(len(right) == 1 and right_name == "green_supply" and
                              right[0].track_id in isolated_track_ids),
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


def safe_zone_corridor_polygon(safe_bbox, destination):
    if safe_bbox is None or destination not in {"material", "injury"}:
        return ()
    x, y, width, height = safe_bbox
    midpoint = x + width // 2
    left, right = (midpoint, x + width) if destination == "injury" else (x, midpoint)
    top = max(0, min(IMAGE_HEIGHT - 1, y + height))
    half = max(80, min(180, width // 4))
    return ((IMAGE_WIDTH // 2 - half, IMAGE_HEIGHT - 1),
            (IMAGE_WIDTH // 2 + half, IMAGE_HEIGHT - 1),
            (max(0, min(IMAGE_WIDTH - 1, right)), top),
            (max(0, min(IMAGE_WIDTH - 1, left)), top))


def carried_view_foot_polygon(camera_pitch_cdeg: int):
    # Front carrying-platform foot bands in the actual 120/115-degree replays
    # (native 1280x1024 pixels, excluding the replay's 96-pixel header). They are
    # not the much larger 140-degree capture-audit ROI or forward push corridor.
    if camera_pitch_cdeg == 12000:
        return ((390, 735), (910, 735), (960, 895), (310, 895))
    if camera_pitch_cdeg == 11500:
        # Clip1/frame1732: held green136 has foot(487,863), on the platform;
        # the separate orange147 at right has foot(1077,959), outside it.
        # The earlier 120-degree image's foot(491,786) is outside this band,
        # so an early angle report cannot overwrite its old image anchor.
        return ((390, 805), (910, 805), (960, 965), (310, 965))
    return ()


def cargo_on_carrying_platform(item: TrackedCargo, camera_pitch_cdeg: int) -> bool:
    polygon = carried_view_foot_polygon(camera_pitch_cdeg)
    if not polygon:
        return False
    x, y, width, height = item.bbox
    return cv2.pointPolygonTest(
        np.asarray(polygon, dtype=np.float32),
        (float(x + width * 0.5), float(y + height)), False,
    ) >= 0


def carried_boxes_shape_compatible(old_boxes, new_boxes, *, same_view=False) -> bool:
    # In the same camera view require actual spatial overlap. Only a verified
    # pitch change permits translating box centres together for shape matching.
    # Both paths retain the existing 0.60 bidirectional overlap rule.
    if len(old_boxes) != len(new_boxes) or not old_boxes:
        return False

    def compatible(first, second):
        if same_view:
            return (bbox_overlap_ratio(first, second) >= 0.60 and
                    bbox_overlap_ratio(second, first) >= 0.60)
        _, _, width_a, height_a = first
        _, _, width_b, height_b = second
        a = (-width_a / 2.0, -height_a / 2.0, width_a, height_a)
        b = (-width_b / 2.0, -height_b / 2.0, width_b, height_b)
        return (bbox_overlap_ratio(a, b) >= 0.60 and
                bbox_overlap_ratio(b, a) >= 0.60)

    matched_old = {}

    def match(index, visited):
        for old_index, old_box in enumerate(old_boxes):
            if old_index in visited or not compatible(old_box, new_boxes[index]):
                continue
            visited.add(old_index)
            if old_index not in matched_old or match(matched_old[old_index], visited):
                matched_old[old_index] = index
                return True
        return False

    return all(match(index, set()) for index in range(len(new_boxes)))


def carried_ids_after_pitch_change(cargo, stm, mission, known_ids,
                                   recent_carried_bboxes, recent_carried_pitch_cdeg):
    # Rebind only the still-held, already audited load while preparing delivery.
    # A sweep/release starts a different ownership lifecycle; matching its floor
    # targets to the original manifest would hide real obstacles.
    if (mission.state not in {
            CompetitionState.ALIGN_SAFE_ZONE_BY_POSE,
            CompetitionState.ACQUIRE_SAFE_ZONE,
            CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX,
            CompetitionState.SAFE_ZONE_CORRIDOR_CHECK,
            CompetitionState.ENTER_SAFE_ZONE,
        } or not stm.gripper_closed or
            recent_carried_pitch_cdeg not in {11500, 12000, 14000} or
            stm.camera_pitch_cdeg not in {11500, 12000} or
            (mission.state == CompetitionState.ENTER_SAFE_ZONE and
             stm.mode not in {STM_MODE_NAVIGATE, STM_MODE_ALIGN_SAFE_ZONE}) or
            not known_ids or not mission.carried_manifest or
            mission.carried_total_count != len(mission.carried_manifest) or
            len(recent_carried_bboxes) != mission.carried_total_count):
        return known_ids
    expected = Counter(mission.carried_manifest)
    known_visible = [item for item in cargo if item.visible and item.track_id in known_ids]
    if Counter(item.class_name for item in known_visible) == expected:
        return known_ids
    candidates = [item for item in cargo
                  if item.visible and not item.inside_safe_zone and
                  item.class_name in expected and
                  cargo_on_carrying_platform(item, stm.camera_pitch_cdeg)]
    # Every visible candidate of an expected class must be explained. With two
    # possible greens for a one-green manifest, retain the old IDs, not a guess.
    if (Counter(item.class_name for item in candidates) != expected or
            not carried_boxes_shape_compatible(
                recent_carried_bboxes, tuple(item.bbox for item in candidates),
                same_view=recent_carried_pitch_cdeg == stm.camera_pitch_cdeg)):
        return known_ids
    return {item.track_id for item in candidates}


def safe_zone_push_corridor(
    cargo: tuple[TrackedCargo, ...],
    safe_bbox: tuple[int, int, int, int] | None,
    destination: str | None,
    first_common_delivered: bool,
    excluded_track_ids: frozenset[int] = frozenset(),
    safe_sweep_capture_offset_mm: float = 150.0,
    carried_manifest: tuple[str, ...] = (),
    carried_total_count: int = 0,
    recent_carried_bboxes: tuple[tuple[int, int, int, int], ...] = (),
) -> tuple[
    tuple[tuple[int, int], ...],
    TrackedCargo | None,
    int | None,
    str,
]:
    if safe_bbox is None or destination not in {"material", "injury"}:
        return (), None, None, SAFE_CORRIDOR_CLEAR
    width = safe_bbox[2]
    front_half_width = max(80, min(180, width // 4))
    polygon = safe_zone_corridor_polygon(safe_bbox, destination)
    claw_near_field_top = round(IMAGE_HEIGHT * 0.80)
    claw_near_field = np.asarray(
        (
            (IMAGE_WIDTH // 2 - front_half_width, IMAGE_HEIGHT - 1),
            (IMAGE_WIDTH // 2 + front_half_width, IMAGE_HEIGHT - 1),
            (IMAGE_WIDTH // 2 + front_half_width, claw_near_field_top),
            (IMAGE_WIDTH // 2 - front_half_width, claw_near_field_top),
        ),
        dtype=np.int32,
    )
    blocking_classes = (
        CARGO_CLASSES
        if not first_common_delivered else
        frozenset({"danger_cyan", "injured_orange"})
    )
    corridor_items: list[tuple[TrackedCargo, tuple[float, float]]] = []
    for item in cargo:
        if item.inside_safe_zone:
            continue
        # Existing tracker survivors retain a recently seen obstacle through
        # a missed detection; a missing box does not prove a clear corridor.
        box_x, box_y, box_width, box_height = item.bbox
        bottom_center = (
            float(box_x + box_width * 0.5),
            float(box_y + box_height),
        )
        if not cargo_in_front_region(item, polygon, safe_bbox):
            continue
        corridor_items.append((item, bottom_center))

    carried_counts = Counter(carried_manifest)

    candidates: list[TrackedCargo] = []
    for item, bottom_center in corridor_items:
        if (
            item.track_id in excluded_track_ids or
            item.class_name not in blocking_classes
        ):
            continue
        in_claw_near_field = (
            cv2.pointPolygonTest(claw_near_field, bottom_center, False) >= 0
        )
        different_critical_class = (
            item.class_name in {"danger_cyan", "injured_orange"} and
            carried_counts[item.class_name] == 0
        )
        if (
            in_claw_near_field and
            not different_critical_class and
            carried_counts[item.class_name] > 0 and
            any(bbox_overlap_ratio(item.bbox, box) >= 0.60 and
                bbox_overlap_ratio(box, item.bbox) >= 0.60
                for box in recent_carried_bboxes)
        ):
            continue
        candidates.append(item)
    if not candidates:
        return polygon, None, None, SAFE_CORRIDOR_CLEAR
    # Pixel ordering chooses a target; it is not a ground-distance estimate.
    obstacle = max(candidates, key=lambda item: (
        item.bbox[1] + item.bbox[3], item.area_px, -item.track_id,
    ))
    return polygon, obstacle, None, SAFE_CORRIDOR_BLOCKED


@lru_cache(maxsize=8)
def capture_quality_mask(polygon):
    mask = np.zeros((IMAGE_HEIGHT // 8, IMAGE_WIDTH // 8), np.uint8)
    cv2.fillPoly(mask, [np.asarray(polygon, np.int32) // 8], 1)
    return mask.astype(bool)


def capture_image_quality(image, pixel_format: str, rois: CaptureRois,
                          calibrated: bool) -> str:
    """Reject measurable incomplete/covered views; valid empty is a real image."""
    if not calibrated:
        return "capture_roi_uncalibrated"
    expected_shape = ((IMAGE_HEIGHT * 3 // 2, IMAGE_WIDTH) if pixel_format == "nv12"
                      else (IMAGE_HEIGHT, IMAGE_WIDTH, 3))
    if image is None or image.shape != expected_shape:
        return "capture_image_incomplete"
    if (len(rois.overall) < 3 or any(
            not (0 <= x < IMAGE_WIDTH and 0 <= y < IMAGE_HEIGHT)
            for x, y in rois.overall)):
        return "capture_roi_incomplete"
    gray = (image[:IMAGE_HEIGHT] if pixel_format == "nv12"
            else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
    pixels = gray[::8, ::8][capture_quality_mask(rois.overall)]
    if pixels.size == 0:
        return "capture_roi_incomplete"
    if (float(pixels.std()) < 2.0 or
            float(np.mean((pixels < 5) | (pixels > 250))) > 0.95):
        return "capture_image_obscured"
    return ""


def make_vision_snapshot(
    detections,
    tracks,
    localizer: GroundLocalizer,
    safe_bbox,
    stm: StmSnapshot,
    mission: CompetitionMission,
    frame_sequence: int,
    safe_zone_filter_blocked: bool = False,
    capture_rois: CaptureRois | None = None,
    low_conf_green_seen: bool = False,
    recent_carried_bboxes: tuple[tuple[int, int, int, int], ...] = (),
    capture_quality_reason: str = "",
    observed_s: float | None = None,
    range_calibration: ApproachRangeCalibration | None = None,
    recent_carried_track_ids: tuple[int, ...] = (),
    recent_corridor_obstacle: TrackedCargo | None = None,
    recent_carried_pitch_cdeg: int = 0,
) -> VisionSnapshot:
    # Cached colour rectangles can belong to the prior camera angle. They
    # cannot establish a current object's support inside the zone. Current
    # rectangle area plus its ground-contact proxy remains image evidence,
    # not an independent semantic/ground-truth inside classification.
    cargo = tracked_cargo(tracks, None if safe_zone_filter_blocked else safe_bbox)
    rois = capture_rois or load_capture_rois(
        VISION_ROOT / "config/capture_roi.json"
    )
    roi_started = time.monotonic()
    capture_items: list[TrackedCargo] = []
    capture_side_by_track: dict[int, str] = {}
    capture_enabled = (
        stm.audit_fresh and
        stm.claw_visible and
        stm.camera_pitch_cdeg == 14000
    )
    capture_observation_valid = (capture_enabled and frame_sequence > 0 and
                                 not capture_quality_reason)
    image_observation_valid = capture_observation_valid
    image_quality_reason = capture_quality_reason
    target_quality: dict[int, str] = {}
    roi_decisions = []
    if capture_enabled:
        for item in tracked_cargo(tracks, None):
            if not item.visible:
                continue
            x, y, width, height = item.bbox
            overlap = bbox_capture_roi_overlap_ratio(item.bbox, rois.overall)
            included = bbox_in_capture_roi(item.bbox, rois.overall, minimum_overlap_ratio=0.60, overlap_ratio=overlap)
            if (not included and item.class_name == "injured_orange" and overlap >= 0.80):
                # A held pyramid can project its foot outside the polygon.
                # Keep the user's ROI: require its centre and >=80% box area.
                included = cv2.pointPolygonTest(
                    np.asarray(rois.overall, dtype=np.float32),
                    tuple(map(float, item.center_px)), False,
                ) >= 0
            truncated = (x <= 0 or y <= 0 or x + width >= IMAGE_WIDTH - 1 or
                         y + height >= IMAGE_HEIGHT - 1)
            reason = "included" if included else (
                "claw_target_truncated_outside_roi" if truncated else "outside_capture_roi")
            roi_decisions.append({"track_id": item.track_id, "class": item.class_name,
                                  "overlap": overlap, "included": included, "reason": reason})
            if not included:
                continue
            # Only a qualifying in-claw truncated object affects load quality.
            # Outside/partial ROI detections cannot veto other clear objects.
            if truncated:
                capture_observation_valid = False
                capture_quality_reason = "claw_target_truncated"
                target_quality[item.track_id] = capture_quality_reason
                roi_decisions[-1]["reason"] = capture_quality_reason
            capture_overlap = 0.60
            capture_items.append(item)
            capture_side = capture_side_for_bbox(
                item.bbox,
                rois,
                minimum_overlap_ratio=capture_overlap,
                overall_included=included,
            )
            capture_side_by_track[item.track_id] = capture_side or (
                "left" if item.center_px[0] <= IMAGE_WIDTH // 2 else "right"
            )
    capture = tuple(capture_items)
    for index, item in enumerate(capture):
        for other in capture[index + 1:]:
            if (bbox_overlap_ratio(item.bbox, other.bbox) >= 0.85 or
                    bbox_overlap_ratio(other.bbox, item.bbox) >= 0.85):
                known_pair = (
                    stm.mode in {21, 35, 37, 38} and
                    item.track_id != other.track_id and item.bbox != other.bbox and
                    item.hits >= 2 and other.hits >= 2 and
                    item.class_name in {"green_supply", "core_black", "injured_orange"} and
                    other.class_name in {"green_supply", "core_black", "injured_orange"} and
                    not target_quality.get(item.track_id) and not target_quality.get(other.track_id)
                )
                if known_pair:
                    continue  # Real multi-object evidence; the two-frame audit still gates actions.
                capture_observation_valid = False
                if capture_quality_reason != "claw_target_truncated":
                    capture_quality_reason = "claw_targets_occluded"
                target_quality.setdefault(item.track_id, "claw_targets_occluded")
                target_quality.setdefault(other.track_id, "claw_targets_occluded")
    roi_elapsed_ms = (time.monotonic() - roi_started) * 1000.0
    selected = mission.selected_batch
    if selected is not None and selected.initial_stash and capture and image_observation_valid:
        # A detected nonempty pile is enough for opening relocation. Object
        # overlap/mixture is expected; image/ROI/camera validity still applies.
        capture_observation_valid = True
        capture_quality_reason = ""
    selected_ids = set(selected.track_ids) if selected is not None else set()
    audit = (
        audit_from_cargo(
            capture,
            selected_ids,
            capture_side_by_track,
            isolated_track_ids={
                item.track_id for item in capture
                if item.class_name == "green_supply" and
                not target_quality.get(item.track_id) and
                mission._is_isolated(item, capture) and
                all(other.track_id == item.track_id or
                    not mission._boxes_overlap(item.bbox, other.bbox)
                    for other in capture)
            },
        )
        if capture_observation_valid else None
    )
    # Separate obstacle audit from the original delivery manifest. Only the
    # locked/reassociated sweep target actually inside the claw ROI counts.
    sweep_search_polygon = ()
    if mission.safe_sweep_pickup is not None:
        # Camera-relative front channel during pickup/retrieval. A live
        # 120-degree safe-box corridor takes priority for obstacle approach;
        # never reuse a frozen 120-degree box after the camera tilts to 140.
        top = 240 if stm.camera_pitch_cdeg == 14000 else 0
        sweep_search_polygon = ((460, top), (820, top), (820, 1023), (460, 1023))
        if (not mission.safe_sweep_pickup.recovering_original and
                stm.camera_pitch_cdeg == 12000 and not safe_zone_filter_blocked):
            sweep_search_polygon = safe_zone_corridor_polygon(
                safe_bbox, mission.confirmed_delivery_destination
            ) or sweep_search_polygon
    sweep_target = mission.safe_sweep_target(
        cargo, sweep_search_polygon, None if safe_zone_filter_blocked else safe_bbox
    )
    sweep_capture = tuple(
        item for item in capture
        if sweep_target is not None and item.track_id == sweep_target.track_id
    )
    recovering_original = bool(mission.safe_sweep_pickup is not None and
                               (mission.safe_sweep_pickup.recovering_original or stm.mode in {44, 45}))
    sweep_observation_valid = capture_observation_valid
    sweep_quality_reason = capture_quality_reason
    if recovering_original:
        # Recovering the load is formal delivery again. Count the WHOLE claw
        # ROI, including unexpected cargo; never truncate it to the target
        # or the two-item limit and accidentally hide overload/danger.
        sweep_capture = capture
    elif mission.safe_sweep_pickup is not None:
        # Only defects involving this obstacle invalidate a mode43 view.
        # Unrelated overlapping cargo elsewhere in the ROI is not its load.
        sweep_quality_reason = (image_quality_reason or
                                (target_quality.get(sweep_target.track_id, "") if sweep_target else ""))
        sweep_observation_valid = image_observation_valid and not sweep_quality_reason
    sweep_audit = (
        audit_from_cargo(sweep_capture, set(), capture_side_by_track)
        if sweep_observation_valid and mission.safe_sweep_pickup is not None else None
    )
    target_candidates = [
        item for item in cargo
        if item.visible and (
            item.track_id == mission.locked_target_track_id or
            item.track_id in selected_ids
        )
    ]
    approach_target = max(target_candidates, key=lambda item: item.area_px, default=None)
    known_carried_ids = (
        set(recent_carried_track_ids)
        if mission.carried_manifest or mission.post_grab_audit_active else set()
    )
    expected_manifest = mission.carried_manifest
    if (capture_enabled and capture_observation_valid and audit is not None and
            mission._audit_valid(audit) and
            (not expected_manifest or
             Counter(item.class_name for item in capture) == Counter(expected_manifest))):
        known_carried_ids = {item.track_id for item in capture}
    known_carried_ids = carried_ids_after_pitch_change(
        cargo, stm, mission, known_carried_ids,
        recent_carried_bboxes, recent_carried_pitch_cdeg,
    )
    exact_delivery_items = [
        item for item in cargo
        if item.visible and item.track_id in known_carried_ids
    ]
    # Do not turn an arbitrary, nearer same-class floor object into the load
    # when its original tracker disappears.
    delivery_items = exact_delivery_items
    delivery_inside_ids = tuple(
        item.track_id for item in delivery_items if item.inside_safe_zone
    )
    delivery_outside_ids = tuple(
        item.track_id for item in delivery_items if not item.inside_safe_zone
    )
    delivery_inside = bool(delivery_inside_ids)
    delivery_outside = bool(delivery_outside_ids)
    danger, side = danger_ahead(cargo, approach_target)
    current_carried_bboxes = ()
    if (
        mission.state != CompetitionState.CLEAR_SAFE_ZONE and
        mission.carried_total_count > 0 and
        len(delivery_items) == mission.carried_total_count and
        Counter(item.class_name for item in delivery_items) ==
        Counter(mission.carried_manifest) and
        (stm.camera_pitch_cdeg not in {11500, 12000} or all(
             cargo_on_carrying_platform(item, stm.camera_pitch_cdeg)
             for item in delivery_items))
    ):
        current_carried_bboxes = tuple(item.bbox for item in delivery_items)
    carried_bbox_reference = current_carried_bboxes or (
        recent_carried_bboxes if recent_carried_pitch_cdeg == stm.camera_pitch_cdeg else ()
    )
    if (mission.state in {
            CompetitionState.CLEAR_SAFE_ZONE,
            CompetitionState.DELIVERY_VERIFY,
            CompetitionState.INVALID_RELEASE,
            CompetitionState.INITIAL_RELEASE,
        } or not stm.gripper_closed or
            (mission.state == CompetitionState.ENTER_SAFE_ZONE and
             stm.mode not in {STM_MODE_NAVIGATE, STM_MODE_ALIGN_SAFE_ZONE})):
        # A sweep may have placed the original load on the ground. Keep its
        # explicit IDs for retrieve/delivery observation, but do not identify
        # an unknown floor track from a box retained before release/ram.
        carried_bbox_reference = ()
    corridor_bbox = None if safe_zone_filter_blocked else safe_bbox
    corridor_destination = (
        mission.confirmed_delivery_destination or
        (selected.destination if selected is not None else None)
    )
    corridor_excluded_track_ids = known_carried_ids
    corridor_cargo = cargo
    if (recent_corridor_obstacle is not None and
            not any(item.track_id == recent_corridor_obstacle.track_id for item in cargo)):
        corridor_cargo = cargo + (recent_corridor_obstacle,)
    (
        corridor,
        corridor_obstacle,
        corridor_distance_mm,
        corridor_status,
    ) = safe_zone_push_corridor(
        corridor_cargo,
        corridor_bbox,
        corridor_destination,
        mission.first_common_delivered,
        frozenset(corridor_excluded_track_ids),
        mission.settings.safe_sweep_capture_offset_mm,
        mission.carried_manifest,
        mission.carried_total_count,
        carried_bbox_reference,
    )
    range_distances = []
    range_reason = ("range_calibration_missing" if range_calibration is None else
                    "range_status_unavailable" if not stm.fresh else
                    "range_pitch_not_applicable" if stm.camera_pitch_cdeg not in {10500, 12000} else
                    image_quality_reason or "range_no_visible_target")
    if range_calibration is not None and stm.fresh and stm.camera_pitch_cdeg in {10500, 12000} and not image_quality_reason:
        for item in cargo:
            if item.visible:
                distance, reason = range_calibration.distance(stm.camera_pitch_cdeg, item.center_px,
                                                             (IMAGE_WIDTH, IMAGE_HEIGHT))
                range_reason = reason
                if distance is not None:
                    range_distances.append((item.track_id, distance))
    return VisionSnapshot(
        range_pitch_cdeg=stm.camera_pitch_cdeg, range_distances=tuple(range_distances),
        range_reason=range_reason,
        roi_decisions=tuple(roi_decisions),
        roi_elapsed_ms=roi_elapsed_ms,
        frame_sequence=frame_sequence,
        observed_monotonic_s=time.monotonic() if observed_s is None else observed_s,
        cargo=cargo,
        capture_cargo=capture,
        safe_bbox=safe_bbox,
        danger_ahead=danger,
        danger_side=side,
        capture_audit=audit,
        capture_observation_valid=capture_observation_valid,
        capture_observation_reason=capture_quality_reason,
        safe_sweep_observation_valid=sweep_observation_valid,
        safe_sweep_observation_reason=sweep_quality_reason,
        delivery_target_found=bool(delivery_items),
        delivery_target_inside_safe_zone=delivery_inside,
        delivery_target_outside_safe_zone=delivery_outside,
        delivery_target_inside_track_ids=delivery_inside_ids,
        delivery_target_outside_track_ids=delivery_outside_ids,
        safe_zone_filter_blocked=safe_zone_filter_blocked,
        low_conf_green_seen=low_conf_green_seen,
        safe_corridor_polygon=corridor,
        safe_corridor_obstacle_class=(
            None if corridor_obstacle is None else corridor_obstacle.class_name
        ),
        safe_corridor_obstacle_distance_mm=corridor_distance_mm,
        safe_corridor_obstacle_track_id=(
            None if corridor_obstacle is None else corridor_obstacle.track_id
        ),
        safe_corridor_obstacle_bbox=(
            None if corridor_obstacle is None else corridor_obstacle.bbox
        ),
        safe_corridor_status=corridor_status,
        safe_corridor_obstacle_remembered=bool(corridor_obstacle is not None and not corridor_obstacle.visible),
        ground_localizer_calibrated=localizer.calibrated,
        carried_reference_bboxes=current_carried_bboxes,
        carried_reference_track_ids=tuple(sorted(known_carried_ids)),
        safe_sweep_capture_audit=sweep_audit,
        safe_sweep_search_polygon=sweep_search_polygon,
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
    def __init__(self, path: Path, writer: AsyncDiagnosticWriter | None = None) -> None:
        self.path = path
        self.writer = writer
        self.lock = threading.Lock()
        if writer is not None:
            writer.reset(path)
        else:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.path.write_text("", encoding="utf-8")

    def write(self, event: str, data: dict) -> None:
        record = {
            "timestamp_monotonic_ns": time.monotonic_ns(),
            "event": event,
            **data,
        }
        if self.writer is not None:
            self.writer.append(self.path, record)
            return
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
        "context_valid": stm.context_valid,
        "task_id": stm.task_id,
        "action_id": stm.action_id,
        "accepted_command": stm.accepted_command,
        "action_status": stm.action_status,
        "rejected_action_id": stm.rejected_action_id,
        "audit_ready": stm.audit_ready,
        "flags": stm.flags,
        "camera_pitch_cdeg": stm.camera_pitch_cdeg,
        "age_ms": stm.age_ms if math.isfinite(stm.age_ms) else None,
        "fault_code": stm.fault_code,
        "acknowledged_sequence": stm.acknowledged_sequence,
        "claw_visible": stm.claw_visible,
        "gripper_closed": stm.gripper_closed,
        "audit_valid": stm.audit_valid,
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


class LocalizationWheelPolicy:
    """One-way business-stage policy; owns neither chassis UART nor SDK."""
    SAFE_ZONE_STATES = frozenset({
        CompetitionState.ALIGN_SAFE_ZONE_BY_POSE,
        CompetitionState.ACQUIRE_SAFE_ZONE,
        CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX,
        CompetitionState.SAFE_ZONE_CORRIDOR_CHECK,
        CompetitionState.CLEAR_SAFE_ZONE,
        CompetitionState.WAIT_SAFE_ZONE_CLEAR,
        CompetitionState.ENTER_SAFE_ZONE,
        CompetitionState.DELIVERY_VERIFY,
        CompetitionState.TASK_COMPLETE,
    })

    def __init__(self, address: str, session_id: str, events_log=None):
        self.address = '\0' + address[1:] if address.startswith('@') else address
        if (not session_id or not self.address.startswith('\0') or
                not 1 < len(self.address.encode()) <= 108):
            raise ValueError('轮式阶段策略需要本轮session和abstract socket地址')
        self.session_id, self.events_log = session_id, events_log
        self.socket = None
        self.closed = False
        self.suppressed = False
        self.last_sent = None
        self.sequence = 0
        self.contact_task_ids = set()
        self.safe_cycle_departed = False
        self.last_state = None
        self.error = None

    def _event(self, name, **data):
        if self.events_log is not None:
            try:
                self.events_log.write(name, data)
            except (OSError, ValueError):
                pass  # A diagnostic write must not block command publication.

    def update(self, output, stm, task_id, action_id):
        if self.closed:
            return
        entering = (output.state in self.SAFE_ZONE_STATES and
                    (not self.safe_cycle_departed or
                     (output.state == CompetitionState.ALIGN_SAFE_ZONE_BY_POSE and
                      self.last_state != CompetitionState.ALIGN_SAFE_ZONE_BY_POSE)))
        if entering:
            self.suppressed = True
            self.safe_cycle_departed = False
        if self.suppressed and output.state in self.SAFE_ZONE_STATES:
            self.contact_task_ids.add(task_id)
        command = output.command
        overlay = (output.suppress_command_tx or command is None or
                   command.opcode in {CMD_HOLD, CMD_PAUSE, CMD_ABORT})
        belongs = stm.context_valid and stm.task_id in self.contact_task_ids
        face_center = (output.state in {CompetitionState.TASK_COMPLETE, CompetitionState.RETURN_CENTER}
                       and stm.mode == STM_MODE_FACE_FIELD_CENTER and belongs)
        later_navigation = (
            output.state in {CompetitionState.RETURN_CENTER, CompetitionState.NAVIGATE, CompetitionState.SEARCH}
            and stm.mode in {STM_MODE_NAVIGATE, STM_MODE_SEARCH}
            and stm.context_valid and stm.context_matches
            and stm.accepted_command in {CMD_RETURN_CENTER, CMD_NAVIGATE_WAYPOINT}
            and (belongs or stm.task_id == task_id))
        if self.suppressed and not overlay and (face_center or later_navigation):
            self.suppressed = False
            self.safe_cycle_departed = True
            self.contact_task_ids.clear()
        self.last_state = output.state
        # mode16 only announces EXIT_SAFE_ZONE, not departure from the ramp.
        # HOLD/PAUSE/recovery/FAULT do not imply that contact risk has ended.
        if self.last_sent == self.suppressed:
            return
        packet = dict(schema=1, kind='competition_wheel_policy', session_id=self.session_id,
                      sequence=self.sequence, suppressed=self.suppressed,
                      reason='safe_zone_low_speed' if self.suppressed else 'normal_motion',
                      state=output.state.value, task_id=task_id, action_id=action_id)
        try:
            if self.socket is None:
                self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
                self.socket.setblocking(False)
            self.socket.sendto(json.dumps(packet, separators=(',', ':')).encode(), self.address)
        except OSError as error:
            if self.error != str(error):
                self._event('localization_wheel_policy_send_failed', error=str(error), policy=packet)
            self.error = str(error)
            return  # Retry the current phase next planner cycle; never wait for an ACK.
        self.error = None
        self.last_sent = self.suppressed
        self.sequence += 1
        self._event('localization_wheel_policy_sent', policy=packet)

    def close(self):
        self.closed = True
        if self.socket is not None:
            self.socket.close()
            self.socket = None
        # STOP/error/exit never resumes wheel input in the remaining SDK session.


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
        self.replay_event_sink = None
        self.localization_wheel_policy = None
        self.period_s = 1.0 / rate_hz
        self.lock = threading.Lock()
        self.latest_vision = VisionSnapshot()
        self.latest_output = CompetitionOutput(mission.state, None, "等待第一帧识别")
        self.latest_pose = PoseSnapshot()
        self.latest_stm = StmSnapshot()
        self.sequence = 0
        self.protocol_task = secrets.randbelow(65535) + 1
        self.protocol_action = secrets.randbelow(65535) + 1
        self.protocol_task_cursor = self.protocol_task
        self.protocol_action_cursor = self.protocol_action
        self.protocol_accepted: StmSnapshot | None = None
        self.protocol_task_generation = mission.task_generation
        self.protocol_action_key = None
        self.protocol_source_stamp = None
        self.protocol_vision_frame = 0
        self.protocol_vision = VisionSnapshot()
        self.protocol_last_output: CompetitionOutput | None = None
        self.published_audit_handoff = None
        self.running = False
        self.error: Exception | None = None
        self.last_state = mission.state
        self.paused = False
        self.pause_reason = "manual"
        self.diagnostics_period_s = 0.1
        self.last_diagnostics_s = -math.inf
        self.command_tx_suppression_reasons_logged: set[str] = set()
        self.grab_waitnav_started_s: float | None = None
        self.grab_waitnav_event_emitted = False
        self.nav_waitnav_last_ack: int | None = None
        self.sweep_observation_key: tuple[int, int, bool] | None = None
        self.sweep_observation_started_s: float | None = None
        self.sweep_observation_start_pose: dict | None = None
        self.sweep_rejection_identity: tuple[int, int, int] | None = None
        self.sweep_rejection_modes: set[int] = set()
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

    def set_paused(self, paused: bool, *, reason: str = "manual") -> None:
        with self.lock:
            self.paused = paused
            self.pause_reason = reason

    def snapshot(self):
        with self.lock:
            return self.latest_output, self.latest_pose, self.latest_stm

    def observe_without_commands(self, now: float) -> None:
        """Keep preview/diagnostics live before start and after operator STOP."""
        if self.running or now - self.last_diagnostics_s < self.diagnostics_period_s:
            return
        pose, stm = load_pose(self.pose_path), load_stm(self.stm_path)
        with self.lock:
            output, vision = self.latest_output, self.latest_vision
            self.latest_pose, self.latest_stm = pose, stm
        try:
            self._diagnostics(output, pose, stm, vision, now=now)
        except Exception as error:
            self._record_diagnostic_error(error, now)
        self.last_diagnostics_s = now

    def _update_grab_waitnav_diagnostic(
        self,
        output: CompetitionOutput,
        pose: PoseSnapshot,
        stm: StmSnapshot,
        now: float,
    ) -> None:
        grab_wait = output.state == CompetitionState.GRAB and stm.mode == 22
        nav_wait = (
            output.state == CompetitionState.NAVIGATE and
            stm.mode == 22 and
            stm.relay_last_mission_command == CMD_NAVIGATE_WAYPOINT
        )
        if not (grab_wait or nav_wait):
            self.grab_waitnav_started_s = None
            self.grab_waitnav_event_emitted = False
            self.nav_waitnav_last_ack = None
            return
        if nav_wait and self.nav_waitnav_last_ack != stm.acknowledged_sequence:
            self.nav_waitnav_last_ack = stm.acknowledged_sequence
            self.grab_waitnav_started_s = now
            self.grab_waitnav_event_emitted = False
            return
        if self.grab_waitnav_started_s is None:
            self.grab_waitnav_started_s = now
            self.nav_waitnav_last_ack = stm.acknowledged_sequence
            return
        if (
            self.grab_waitnav_event_emitted or
            now - self.grab_waitnav_started_s < 0.5
        ):
            return
        if nav_wait:
            block = "NAV_REJECTED_OR_NOT_ACKED"
        elif not stm.fresh:
            block = "STM_STALE"
        elif not stm.gripper_closed:
            block = "GRIPPER_FLAG_MISSING"
        elif not pose.valid:
            block = "POSE_INVALID"
        else:
            block = "NAV_NOT_PUBLISHED"
        self.grab_waitnav_event_emitted = True
        self.events_log.write("grab_waitnav_detected", {
            "upper_state": output.state.value,
            "upper_reason": output.reason or output.message,
            "protocol_context": {
                "task_id": self.protocol_task,
                "action_id": self.protocol_action,
                "vision_frame": self.protocol_vision_frame,
                "status_matches": stm.context_valid and stm.task_id == self.protocol_task and stm.action_id == self.protocol_action,
            },
            "waitnav_block": f"WAITNAV_BLOCK: {block}",
            "stm_mode": stm.mode,
            "stm_fresh": stm.fresh,
            "stm_age_ms": stm.age_ms if math.isfinite(stm.age_ms) else None,
            "stm_gripper_closed": stm.gripper_closed,
            "stm_audit_valid": stm.audit_valid,
            "stm_ack": stm.acknowledged_sequence,
            "grab_initial_ack": self.mission.grab_initial_ack,
            "relay_last_mission_command": stm.relay_last_mission_command,
            "relay_last_mission_sequence": stm.relay_last_mission_sequence,
            "relay_last_mission_payload": list(
                stm.relay_last_mission_payload
            ),
            "pose_valid": pose.valid,
            "pose_age_ms": pose.age_ms if math.isfinite(pose.age_ms) else None,
        })

    def _update_safe_sweep_diagnostic(
        self,
        output: CompetitionOutput,
        vision: VisionSnapshot,
        pose: PoseSnapshot,
        stm: StmSnapshot,
        now: float,
    ) -> None:
        """Observe wire mode/claw edges only; never infer a mechanical subphase."""
        active = (
            output.state == CompetitionState.CLEAR_SAFE_ZONE or
            self.mission.safe_sweep_reaudit_active
        )
        if not active:
            if self.sweep_observation_key is not None:
                self.events_log.write("safe_sweep_observation_end", {
                    "previous_observation": self.sweep_observation_key,
                    "upper_state": output.state.value,
                    "stm": stm_dict(stm),
                    "pose": pose_dict(pose),
                })
            self.sweep_observation_key = None
            self.sweep_observation_started_s = None
            self.sweep_observation_start_pose = None
            return
        if not stm.fresh:
            return
        key = (self.mission.safe_sweep_attempts, stm.mode, stm.gripper_closed)
        if key == self.sweep_observation_key:
            return
        previous = self.sweep_observation_key
        previous_elapsed = (
            None if self.sweep_observation_started_s is None else
            max(0.0, now - self.sweep_observation_started_s)
        )
        self.sweep_observation_key = key
        self.sweep_observation_started_s = now
        self.sweep_observation_start_pose = pose_dict(pose)
        pickup = self.mission.safe_sweep_pickup
        self.events_log.write("safe_sweep_mode_observed", {
            "previous_observation": previous,
            "previous_observation_elapsed_s": previous_elapsed,
            "attempt": key[0],
            "upper_state": output.state.value,
            "stm": stm_dict(stm),
            "pose": pose_dict(pose),
            "vision_frame_sequence": vision.frame_sequence,
            "command_opcode": None if output.command is None else output.command.opcode,
            "reason": output.reason,
            "recovering_original": None if pickup is None else pickup.recovering_original,
            "pickup_grab_started": None if pickup is None else pickup.grab_started,
            "mechanical_subphase_available": False,
        })

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
        if self.localization_wheel_policy is not None:
            self.localization_wheel_policy.close()
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

    def _publish(self, output: CompetitionOutput, *, stm: StmSnapshot | None = None) -> None:
        if self.localization_wheel_policy is not None and stm is not None:
            matched = (stm.context_valid and stm.task_id == self.protocol_task and
                       stm.action_id == self.protocol_action)
            self.localization_wheel_policy.update(output, replace(stm, context_matches=matched),
                self.protocol_task, self.protocol_action)
        if output.suppress_command_tx:
            return
        command = output.command or CommandRequest(CMD_HOLD)
        overlay = command.opcode in {CMD_HOLD, CMD_PAUSE}
        if not overlay and self.protocol_task_generation != self.mission.task_generation:
            self.protocol_task_generation = self.mission.task_generation
            if self.mission.disperse_attempts == 0 and not self.mission.recovery_task_locked:
                self.protocol_task_cursor = (self.protocol_task_cursor + 1) & 0xFFFF
                self.protocol_task = self.protocol_task_cursor
            self.protocol_action_key = None
        # HOLD/PAUSE suspend the current action; they do not allocate a new
        # mechanical epoch. Dynamic coordinates and audit IDs are not keys.
        pickup = self.mission.safe_sweep_pickup
        audit_role = (
            self.mission.audit_epoch,
            None if pickup is None else pickup.mode,
        ) if command.opcode == CMD_CARGO_AUDIT else None
        separation_epoch = (
            self.mission.separation_request_epoch
            if command.opcode in {CMD_DISPERSE_PILE, CMD_RELEASE_LEFT, CMD_RELEASE_RIGHT, CMD_RELEASE_BOTH}
            else None
        )
        range_epoch = self.mission.range_epoch if command.opcode == CMD_APPROACH_TARGET and command.flags & CMD_APPROACH_RANGE else None
        key = (command.opcode, command.flags, audit_role, separation_epoch, range_epoch)
        if not overlay or (self.protocol_accepted is None and self.protocol_action_key is None):
            if key != self.protocol_action_key:
                self.protocol_action_cursor = (self.protocol_action_cursor + 1) & 0xFFFF
                self.protocol_action = self.protocol_action_cursor
                if (self.protocol_action == 0 and self.protocol_accepted is not None and
                        self.protocol_accepted.task_id == self.protocol_task and
                        self.protocol_accepted.action_id == 0xFFFF):
                    self.protocol_task_cursor = (self.protocol_task_cursor + 1) & 0xFFFF
                    self.protocol_task = self.protocol_task_cursor
                self.protocol_action_key = key
                self.published_audit_handoff = None
                self.protocol_source_stamp = None
                self.protocol_vision_frame = 0
        if command.opcode == CMD_APPROACH_TARGET and command.flags & CMD_APPROACH_RANGE:
            self.protocol_vision_frame = command.source_vision_frame or 0
        elif command.opcode == CMD_APPROACH_TARGET:
            stamp = (
                pickup.last_seen_s if pickup is not None else
                self.mission.cluster_command_observed_s
                if output.state == CompetitionState.CLUSTER_APPROACH else
                self.mission.target_last_seen_s
            )
            if stamp is not None and stamp != self.protocol_source_stamp:
                self.protocol_source_stamp = stamp
                self.protocol_vision_frame = self.protocol_vision.frame_sequence & 0xFFFFFFFF
        elif command.opcode == CMD_CARGO_AUDIT:
            frame = (pickup.last_audit_frame if pickup is not None else
                     self.mission.pending_audit_frame_sequence or self.mission.audit_last_frame_sequence or 0)
            if command.audit_vision_frame is not None:
                frame = command.audit_vision_frame
            self.protocol_vision_frame = frame & 0xFFFFFFFF
        owner = self.protocol_accepted if overlay else None
        context = command_context_frame(
            self.sequence,
            owner.task_id if owner is not None else self.protocol_task,
            owner.action_id if owner is not None else self.protocol_action,
            self.protocol_vision_frame if command.opcode in {CMD_APPROACH_TARGET, CMD_CARGO_AUDIT} else 0,
        )
        write_command_frame(self.command_path, context + command.to_frame(self.sequence))
        if (command.opcode == CMD_CARGO_AUDIT and command.audit is not None and
                command.audit.stable and command.audit == self.mission.pending_audit_payload and
                command.audit_vision_frame == self.mission.pending_audit_frame_sequence and
                self.mission.pending_audit_valid):
            # Candidate observations can still change and supply real new IDs
            # to the MCU's two-frame audit. Keep the published legal snapshot
            # separately in case the already accepted handoff arrives later.
            self.published_audit_handoff = (
                self.protocol_task, self.protocol_action, self.mission.audit_epoch,
                {k: v for k, v in vars(self.mission).items() if k.startswith("pending_audit")})
        if not overlay or self.protocol_last_output is None:
            self.protocol_last_output = output
        self.sequence = (self.sequence + 1) & 0xFF

    def _adopt_recovery_context(self) -> None:
        """Cancel an unaccepted request without reusing its allocated IDs."""
        owner = self.protocol_accepted
        if owner is not None:
            same_task = self.protocol_task == owner.task_id
            self.protocol_task = owner.task_id
            self.protocol_action = owner.action_id
            self.protocol_task_cursor = owner.task_id
            delta = (self.protocol_action_cursor - owner.action_id) & 0xFFFF
            if not same_task or delta >= 0x8000:
                self.protocol_action_cursor = owner.action_id
        self.protocol_task_generation = self.mission.task_generation
        self.protocol_action_key = None
        self.protocol_source_stamp = None
        self.protocol_vision_frame = 0
        self.protocol_last_output = None
        self.published_audit_handoff = None

    def _restore_audit_handoff(self, stm: StmSnapshot) -> None:
        saved = self.published_audit_handoff
        if (saved is None or stm.mode not in {22, 40, 50} or not stm.context_valid or
                stm.fault or not stm.action_status & 1 or stm.accepted_command != CMD_CARGO_AUDIT):
            return
        task, action, epoch, evidence = saved
        if ((stm.task_id, stm.action_id) != (task, action) or
                (self.protocol_task, self.protocol_action) != (task, action) or
                self.mission.audit_epoch != epoch):
            return
        if stm.mode in {22, 40} and not (stm.gripper_closed and stm.audit_ready and stm.audit_valid):
            return
        # Only the matching MCU handoff commits this snapshot. It is never
        # reused as a fresh camera observation or evidence for a new action.
        if self.mission.committed_audit_handoff_mode is None:
            for key, value in evidence.items():
                setattr(self.mission, key, value)
            self.mission.pending_audit_handoff_accepted = True

    def _is_recovery_observation(self, stm: StmSnapshot) -> bool:
        if not stm.fresh:
            return False
        state = self.mission.state
        if stm.mode in {24, 3} and self.mission.committed_audit_handoff_mode is not None:
            return True
        if stm.mode == 41:
            return True
        if stm.mode == 47:
            return (state != CompetitionState.RETURN_CENTER or
                    (stm.context_valid and stm.task_id != self.protocol_task))
        if stm.mode == 46:
            return (state == CompetitionState.CLEAR_SAFE_ZONE or
                    (state == CompetitionState.RETURN_CENTER and stm.context_valid and
                     stm.task_id != self.protocol_task))
        capture_states = {
            CompetitionState.CAPTURE_AUDIT, CompetitionState.AUDIT_CONFIRM,
            CompetitionState.TRANSPORT_AUDIT,
            CompetitionState.POST_GRAB_AUDIT, CompetitionState.GRAB,
            CompetitionState.DISPERSE, CompetitionState.INVALID_RELEASE,
            CompetitionState.INVALID_BACKOFF, CompetitionState.WAIT_SEARCH_RECOVERY,
        }
        if stm.mode == 24:
            return state in capture_states | {
                CompetitionState.APPROACH, CompetitionState.INITIAL_APPROACH,
                CompetitionState.CLUSTER_APPROACH,
            }
        if stm.mode == 49:
            return (stm.context_valid and stm.task_id == self.protocol_task and
                    not self.mission.post_grab_half_close_active and state in capture_states | {
                CompetitionState.GRAB,
                CompetitionState.POST_GRAB_AUDIT,
            })
        if stm.mode in {42, 44} and state == CompetitionState.CLEAR_SAFE_ZONE:
            pickup = self.mission.safe_sweep_pickup
            return bool(stm.context_valid and stm.task_id == self.protocol_task and
                        pickup is not None and pickup.mode in {43, 45})
        if stm.mode == 3:
            if state in capture_states | {CompetitionState.BOUNDARY_RECOVERY}:
                return True
            if state == CompetitionState.CLUSTER_APPROACH:
                return self.mission.cluster_execution_seen
            if state in {CompetitionState.APPROACH, CompetitionState.INITIAL_APPROACH}:
                return self.mission.approach_f407_active_seen
            return (state == CompetitionState.RETURN_CENTER and stm.context_valid and
                    stm.accepted_command == CMD_RETURN_CENTER and bool(stm.action_status & 1))
        return False

    def _reconcile_transport_rejection(
        self, vision: VisionSnapshot, pose: PoseSnapshot, stm: StmSnapshot, now: float
    ) -> CompetitionOutput | None:
        # An older accepted status alone is only a delayed ACK. Require the
        # MCU's explicit same-task rejection of the actual pending action.
        if not (stm.fresh and stm.context_valid and stm.action_status & 4 and
                stm.task_id == self.protocol_task and
                stm.rejected_action_id == self.protocol_action and
                self.protocol_last_output is not None and
                self.mission.selected_batch is not None and
                not self.mission.selected_batch.initial_stash):
            return None
        if not (self.mission.transport_audit_handoff_active or
                self.mission.state == CompetitionState.TRANSPORT_AUDIT):
            return None
        if stm.mode not in {10, STM_MODE_TRANSPORT_AUDIT}:
            return None
        self._adopt_recovery_context()
        current = replace(stm, context_matches=True)
        if stm.mode == 10 and (not stm.distance_done or not stm.gripper_closed):
            return self.mission._resume_stage_navigation(pose, current, now)
        # Restart an actual full-ROI observation, not the rejected stale
        # release/ALIGN/NAV. READY/VALID determines its normal exit afterward.
        return replace(
            self.mission._begin_transport_audit(vision, pose, current, now),
            event="transport_audit_request_rejected",
            reason="transport_audit_reobserve_after_reject",
            message="下位机明确拒绝当前预备点请求，回到完整夹爪复审并按最新READY/VALID重新决策",
        )

    def _reconcile_sweep_rejection(
        self, vision: VisionSnapshot, pose: PoseSnapshot, stm: StmSnapshot, now: float
    ) -> CompetitionOutput | None:
        if not (stm.fresh and stm.context_valid and stm.action_status & 4 and
                stm.task_id == self.protocol_task and
                stm.rejected_action_id == self.protocol_action and
                self.protocol_last_output is not None and
                (self.mission.state == CompetitionState.CLEAR_SAFE_ZONE or
                 self.mission.safe_sweep_reaudit_active) and
                stm.mode in {23, 39, 42, 43, 44, 45}):
            return None
        pending = self.protocol_last_output.command
        if pending is None or pending.opcode in {CMD_HOLD, CMD_PAUSE}:
            return None  # An overlay is not a rejected mechanical request.
        if (pending is not None and stm.action_id == self.protocol_action and
                stm.accepted_command == pending.opcode and stm.action_status & 1):
            # The MCU may reject a duplicate update while the accepted action
            # is already executing. Do not reset its pickup evidence in a loop.
            return None
        identity = (stm.task_id, stm.rejected_action_id, stm.action_id)
        if identity != self.sweep_rejection_identity:
            self.sweep_rejection_identity = identity
            self.sweep_rejection_modes.clear()
        if stm.mode in self.sweep_rejection_modes:
            return None
        self.sweep_rejection_modes.add(stm.mode)
        rejected_output = self.protocol_last_output
        self._adopt_recovery_context()
        # HOLD keeps the accepted context while the original mode selects its
        # next business command. It must not replace the rejected request's
        # identity and make the same persistent rejection look new next cycle.
        self.protocol_last_output = rejected_output
        current = replace(stm, context_matches=True)
        if stm.mode == 23:
            self.mission.safe_sweep_reaudit_active = True
            self.mission._begin_post_grab_audit(vision, now)
        else:
            # Real mode/camera transitions already establish their own frame
            # floor. Repeated status snapshots must not erase that evidence.
            pickup = self.mission.safe_sweep_pickup
            if (pickup is not None and pending.opcode == CMD_GRAB_CONFIRMED and
                    stm.mode in {43, 45}):
                # This GRAB never became an accepted action. Re-publish the
                # existing audit without discarding its real visual evidence.
                pickup.grab_started = False
                pickup.audit_command = None
            self.mission.safe_sweep_command_accepted = True
        return replace(
            self.mission.step(vision, pose, current, now),
            event="safe_sweep_request_rejected", reason="safe_sweep_reobserve_after_reject",
            message="下位机明确拒绝当前扫障请求，按实际模式恢复目标查找或新帧审核",
        )

    def _reconcile_separation_rejection(
        self, vision: VisionSnapshot, stm: StmSnapshot, now: float
    ) -> CompetitionOutput | None:
        pending = self.protocol_last_output.command if self.protocol_last_output is not None else None
        if not (pending is not None and pending.opcode in {
                CMD_DISPERSE_PILE, CMD_RELEASE_LEFT, CMD_RELEASE_RIGHT, CMD_RELEASE_BOTH} and
                stm.fresh and stm.context_valid and stm.action_status & 4 and
                stm.task_id == self.protocol_task and stm.rejected_action_id == self.protocol_action):
            return None
        accepted = (stm.action_id == self.protocol_action and stm.accepted_command == pending.opcode and
                    bool(stm.action_status & 1))
        if (accepted or self.mission.disperse_command_accepted or
                self.mission.invalid_release_command_accepted):
            return None
        self._adopt_recovery_context()
        current = replace(stm, context_matches=True)
        if stm.mode == 48:
            self.mission.transport_separation_active = False
            return self.mission._begin_transport_audit(vision, load_pose(self.pose_path), current, now)
        if stm.mode == 23:
            self.mission._begin_post_grab_audit(vision, now)
        else:
            self.mission._begin_capture_audit(
                vision, now, cluster_active=stm.mode in {37, 38}, recheck_pending=True,
                recheck_context="disperse_selective")
        return replace(self.mission._audit_output(vision, current, now),
                       event="separation_request_rejected", reason="separation_reobserve_after_reject",
                       message="分离/释放请求明确拒绝，不计次数，撤销等待并用新帧重新审核")

    def _reconcile_approach_rejection(
        self, vision: VisionSnapshot, pose: PoseSnapshot, stm: StmSnapshot, now: float
    ) -> CompetitionOutput | None:
        pending = self.protocol_last_output
        if not (pending is not None and pending.command is not None and
                pending.command.opcode == CMD_APPROACH_TARGET and
                self.mission.state in {CompetitionState.INITIAL_APPROACH,
                    CompetitionState.APPROACH, CompetitionState.CLUSTER_APPROACH} and
                stm.fresh and stm.context_valid and stm.mode == 3 and
                stm.task_id == self.protocol_task and stm.action_status & 4 and
                stm.rejected_action_id == self.protocol_action):
            return None
        if (stm.action_id == self.protocol_action and stm.action_status & 1 and
                stm.accepted_command == CMD_APPROACH_TARGET):
            return None  # A late duplicate rejection cannot cancel an accepted action.
        self._adopt_recovery_context()
        current = replace(stm, context_matches=True)
        if self.mission.disperse_attempts or self.mission.recovery_task_locked:
            # The existing MCU keeps the pile budget until RETURN_CENTER.
            # Finish that route rather than retrying a cancelled same-task target.
            self.mission._clear_search_recovery_context(vision, clear_carried=True)
            self.mission._begin_return(current, now)
            result = self.mission._return_center_output(pose, now, "分离后接近请求被拒绝，按原流程返中结束当前批次")
        else:
            result = self.mission._audit_reacquire_output(vision, current, now)
        return replace(result, event="approach_request_rejected",
                       reason="approach_recover_after_reject")

    def _output_for_cycle(
        self,
        vision: VisionSnapshot,
        pose: PoseSnapshot,
        stm: StmSnapshot,
        paused: bool,
        now: float,
    ) -> CompetitionOutput:
        self.protocol_vision = vision
        self._restore_audit_handoff(stm)
        if (stm.fresh and stm.mode == STM_MODE_RECOVERY_WAIT and not stm.fault and
                self.mission.state not in {CompetitionState.FAULT, CompetitionState.FINISHED}):
            if stm.context_valid and stm.action_status & 1:
                self.protocol_accepted = stm
            if not self.mission.recovery_wait_active:
                # An accepted separation may have faulted before its normal ACK was observed.
                if (stm.context_valid and stm.task_id == self.protocol_task and
                        stm.action_id == self.protocol_action and stm.action_status & 1):
                    if (self.mission.state == CompetitionState.DISPERSE and
                            self.mission.disperse_context == "selective" and
                            self.mission.disperse_command is not None and
                            stm.accepted_command == CMD_DISPERSE_PILE):
                        self.mission._count_separation_acceptance(
                            "right" if self.mission.disperse_command.flags & CMD_TARGET_RIGHT else "left")
                    elif (self.mission.state == CompetitionState.INVALID_RELEASE and
                          self.mission.invalid_release_context == "single_side_then_reaudit" and
                          stm.accepted_command in {CMD_RELEASE_LEFT, CMD_RELEASE_RIGHT}):
                        self.mission._count_separation_acceptance(
                            "right" if stm.accepted_command == CMD_RELEASE_LEFT else "left")
                self._adopt_recovery_context()
                self.command_path.unlink(missing_ok=True)
                self.mission._enter_recovery_wait(vision, now)
            if (stm.context_valid and stm.task_id == self.protocol_task and stm.action_status & 4 and
                    stm.rejected_action_id == self.protocol_action and
                    not (stm.action_id == self.protocol_action and stm.action_status & 1 and
                         stm.accepted_command != CMD_CARGO_AUDIT)):
                self._adopt_recovery_context()
                self.mission._restart_recovery_audit(vision, now)
            matches = (stm.context_valid and stm.task_id == self.protocol_task and
                       stm.action_id == self.protocol_action)
            recovery_vision = (replace(vision, capture_audit=None, capture_observation_valid=False)
                               if paused and self.pause_reason != "manual" else vision)
            return self.mission._recovery_wait_output(
                recovery_vision, pose, replace(stm, context_matches=matches), now,
                manual_paused=paused and self.pause_reason == "manual")
        if self.mission.stm_fault_waiting or (stm.fresh and stm.fault):
            # A cleared bit or a pending ACK must not resume the retained task after a fault.
            return self.mission.step(vision, pose, stm, now)
        if stm.fresh and stm.context_valid and stm.action_status & 1:
            self.protocol_accepted = stm
        if paused and self.pause_reason == "manual" and self.mission.state != CompetitionState.FAULT:
            self.mission._audit_wire_timeout_output(stm, now, manual_paused=True)
            return CompetitionOutput(
                self.mission.state, CommandRequest(CMD_PAUSE),
                "人工PAUSE，冻结审核握手期限直到恢复", tx_policy="pause", reason="manual_pause",
            )
        if self.mission.recovery_wait_active:
            matches = (stm.context_valid and stm.task_id == self.protocol_task and
                       stm.action_id == self.protocol_action)
            return self.mission.step(vision, pose, replace(stm, context_matches=matches), now)
        if (stm.fresh and not stm.fault and stm.mode in {52, 53} and
                self.mission.state in {CompetitionState.INITIAL_APPROACH, CompetitionState.APPROACH,
                                       CompetitionState.CLUSTER_APPROACH}):
            if (stm.context_valid and stm.task_id == self.protocol_task and stm.action_status & 4 and
                    stm.rejected_action_id == self.protocol_action and stm.mode == 52 and
                    not (stm.action_id == self.protocol_action and stm.action_status & 1 and
                         stm.accepted_command == CMD_APPROACH_TARGET)):
                self._adopt_recovery_context()
                self.mission._clear_range_approach()
            ranged_vision = replace(vision, range_distances=()) if paused else vision
            return self.mission._range_approach_output(ranged_vision, stm, now)
        handoff = self.mission._range_handoff_output(stm)
        if handoff is not None:
            return handoff
        # Recovery is an observation from the MCU, not an ACK of the last
        # requested action. Handle it before context_pending/camera pause.
        if self._is_recovery_observation(stm):
            self._adopt_recovery_context()
            result = self.mission.step(vision, pose, replace(stm, context_matches=True), now)
            return result
        if stm.fresh and not stm.fault:
            timeout = self.mission._audit_wire_timeout_output(stm, now)
            if timeout is not None:
                return timeout
        if self.mission.state == CompetitionState.FAULT:
            return self.mission.step(vision, pose, stm, now)
        if paused and self.mission.state == CompetitionState.WAIT_START:
            return CompetitionOutput(
                self.mission.state,
                None,
                "启动阶段等待摄像头恢复，不干预F407自主出发",
                suppress_command_tx=True,
                suppression_reason="camera_recovery_wait_start",
                tx_policy="autonomous_start",
                reason="camera_recovery_wait_start",
            )
        if paused:
            return CompetitionOutput(
                self.mission.state,
                CommandRequest(CMD_PAUSE),
                "摄像头恢复中，冻结当前阶段",
                tx_policy="pause",
                reason="camera_recovery",
            )
        matches = (stm.context_valid and stm.task_id == self.protocol_task and
                   stm.action_id == self.protocol_action)
        reconciled = self._reconcile_transport_rejection(vision, pose, stm, now)
        if reconciled is not None:
            return reconciled
        reconciled = self._reconcile_sweep_rejection(vision, pose, stm, now)
        if reconciled is not None:
            return reconciled
        reconciled = self._reconcile_separation_rejection(vision, stm, now)
        if reconciled is not None:
            return reconciled
        reconciled = self._reconcile_approach_rejection(vision, pose, stm, now)
        if reconciled is not None:
            return reconciled
        if (stm.fresh and stm.mode in {22, 40, 50} and
                (not self.mission._has_committed_audit(stm.mode) or self.mission.selected_batch is None)):
            return self.mission.step(vision, pose, replace(stm, context_matches=matches), now)
        if (stm.fresh and stm.context_valid and stm.task_id == self.protocol_task and
                stm.mode in {21, 22, 23, 37, 38, 39, 40, 42, 43, 44, 45, 48, 49, 50} and
                self.mission.state in {CompetitionState.CAPTURE_AUDIT,
                    CompetitionState.POST_GRAB_AUDIT, CompetitionState.AUDIT_CONFIRM,
                    CompetitionState.TRANSPORT_AUDIT, CompetitionState.CLEAR_SAFE_ZONE}):
            # Re-evaluate evidence even while an audit ACK is in flight.
            # Otherwise the context gate can keep transmitting expired STABLE.
            return self.mission.step(vision, pose, replace(stm, context_matches=matches), now)
        if (self.protocol_last_output is not None and stm.fresh and
                not matches and stm.mode != 41 and not stm.fault):
            pending = self.protocol_last_output.command
            if (pending is not None and pending.opcode == CMD_APPROACH_TARGET and
                    not pending.flags & CMD_APPROACH_RANGE and stm.mode in {3, 20} and
                    self.mission.state in {CompetitionState.INITIAL_APPROACH,
                        CompetitionState.APPROACH, CompetitionState.CLUSTER_APPROACH}):
                # Pixel tracking is dynamic. Waiting for ACCEPT must not freeze
                # the source frame/target while the camera continues to move.
                return self.mission.step(vision, pose, replace(stm, context_matches=False), now)
            # Re-send this request while waiting for its paired status. Do
            # not replace it with PAUSE and prevent acceptance of the action.
            return replace(self.protocol_last_output, event="", reason="command_context_pending",
                           message="等待本次任务/动作上下文回执，重发原请求")
        return self.mission.step(vision, pose, replace(stm, context_matches=matches), now)

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
        if output.command.opcode == CMD_PAUSE:
            return "pause"
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
                "confirmed_total_count": output.batch.confirmed_total_count,
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
        diagnostics_writer = getattr(self.events_log, "writer", None)
        write_snapshot = diagnostics_writer.snapshot if diagnostics_writer is not None else write_atomic_json
        write_snapshot(self.diagnostics_path, {
            "diagnostic_logging": diagnostics_writer.statistics() if diagnostics_writer is not None else None,
            "diagnostic_error_count": getattr(self, "diagnostic_error_count", 0),
            "approach_range": {"pitch_cdeg": self.mission.range_pitch_cdeg,
                               "epoch": self.mission.range_epoch, "reason": vision.range_reason,
                               "source_frame": (self.mission.range_command.source_vision_frame if self.mission.range_command else None),
                               "distance_mm": (self.mission.range_command.arg_a if self.mission.range_command else None)},
            "schema_version": 1,
            "timestamp_monotonic_ns": time.monotonic_ns(),
            "protocol_context": {
                "task_id": self.protocol_task,
                "action_id": self.protocol_action,
                "vision_frame": self.protocol_vision_frame,
                "accepted_task_id": None if self.protocol_accepted is None else self.protocol_accepted.task_id,
                "accepted_action_id": None if self.protocol_accepted is None else self.protocol_accepted.action_id,
                "accepted_command": None if self.protocol_accepted is None else self.protocol_accepted.accepted_command,
            },
            "state": output.state.value,
            "message": output.message,
            "event": output.event,
            "motion_expected": output.motion_expected,
            "command_tx_suppressed": output.suppress_command_tx,
            "suppression_reason": output.suppression_reason,
            "upper_state": output.state.value,
            "upper_reason": output.reason or output.suppression_reason or output.message,
            "stm_mode": stm.mode,
            "stm_fresh": stm.fresh,
            "stm_age_ms": stm.age_ms if math.isfinite(stm.age_ms) else None,
            "stm_gripper_closed": stm.gripper_closed,
            "stm_ack": stm.acknowledged_sequence,
            "relay_last_mission_command": stm.relay_last_mission_command,
            "relay_last_mission_sequence": stm.relay_last_mission_sequence,
            "relay_last_mission_payload": list(stm.relay_last_mission_payload),
            "pose_valid": pose.valid,
            "pose_age_ms": pose.age_ms if math.isfinite(pose.age_ms) else None,
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
            "detour_tx_baseline": self.mission.detour_tx_baseline,
            "detour_command_accepted": self.mission.detour_command_accepted,
            "stm_fault_waiting": self.mission.stm_fault_waiting,
            "recovery_wait_active": self.mission.recovery_wait_active,
            "recovery_origin_state": (self.mission.recovery_origin_state.value
                                      if self.mission.recovery_origin_state else None),
            "recovery_reference_ready": self.mission.recovery_reference_ready,
            "recovery_task_locked": self.mission.recovery_task_locked,
            "recovery_abandon_requested": self.mission.recovery_abandon_requested,
            "recovery_reason": "F407 mode51; detailed cause is not carried by 0x17" if self.mission.recovery_wait_active else "",
            "recovery_interrupted_pickup": (
                {"target_id": self.mission.recovery_interrupted_pickup.target_id,
                 "target_class": self.mission.recovery_interrupted_pickup.target_class,
                 "target_bbox": self.mission.recovery_interrupted_pickup.target_bbox,
                 "original_ids": sorted(self.mission.recovery_interrupted_pickup.original_ids)}
                if self.mission.recovery_interrupted_pickup is not None else None),
            "delivery_outside_seen": self.mission.delivery_outside_seen,
            "delivery_inside_hits": self.mission.delivery_inside_hits,
            "delivery_window_frames": len(self.mission.delivery_window),
            "delivery_window_misses": self.mission._delivery_window_misses(),
            "delivery_window_started_s": self.mission.delivery_window_started_s,
            "delivery_observation_attempt": self.mission.delivery_observation_attempt,
            "delivery_observation_elapsed_s": self.mission._delivery_observation_elapsed(diagnostic_now),
            "delivery_visual_frame_age_ms": self.mission.delivery_last_frame_age_ms,
            "delivery_visual_confirmed": self.mission.delivery_visual_confirmed,
            "delivery_lateral_aligned": self.mission._delivery_lateral_aligned(pose),
            "delivery_target_point": (
                None
                if (
                    self.mission.selected_batch is None or
                    (
                        self.mission.selected_batch.destination != "stash" and
                        self.mission.confirmed_delivery_destination is None
                    )
                )
                else list(self.mission._target_point())
            ),
            "confirmed_delivery_destination": (
                self.mission.confirmed_delivery_destination
            ),
            "delivery_completion_basis": self.mission.delivery_completion_basis,
            "delivery_timeout_reason": self.mission.delivery_timeout_reason,
            "delivery_conflict_frames": self.mission.delivery_conflict_frames,
            "safe_zone_freeze_hits": self.mission.safe_zone_freeze_hits,
            "safe_zone_freeze_frame_floor": self.mission.safe_zone_freeze_frame_floor,
            "locked_safe_bbox": (
                None
                if self.mission.locked_safe_bbox is None
                else list(self.mission.locked_safe_bbox)
            ),
            "locked_safe_target_x_px": self.mission.locked_safe_target_x_px,
            "safe_zone_visual_pixel_error": self.mission.safe_zone_visual_pixel_error,
            "safe_zone_visual_locked": self.mission.safe_zone_visual_locked,
            "safe_zone_fallback": self.mission.safe_zone_fallback,
            "safe_corridor_obstacle_remembered": vision.safe_corridor_obstacle_remembered,
            "safe_zone_view_pitch_cdeg": self.mission.safe_zone_view_pitch_cdeg,
            "empty_load_recheck_pending": self.mission.empty_load_recheck_pending,
            "empty_load_view_moved": self.mission.empty_load_view_moved,
            "empty_load_recheck_done": self.mission.empty_load_recheck_done,
            "carried_reference_track_ids": list(vision.carried_reference_track_ids),
            "safe_sweep_attempts": self.mission.safe_sweep_attempts,
            "safe_sweep_command_accepted": self.mission.safe_sweep_command_accepted,
            "safe_sweep_execution_seen": self.mission.safe_sweep_execution_seen,
            "safe_sweep_reaudit_active": self.mission.safe_sweep_reaudit_active,
            "safe_sweep_control": "pixel_approach",
            "safe_sweep_observation": {
                "key": self.sweep_observation_key,
                "elapsed_s": (
                    None if self.sweep_observation_started_s is None else
                    max(0.0, diagnostic_now - self.sweep_observation_started_s)
                ),
                "start_pose": self.sweep_observation_start_pose,
                "mechanical_subphase_available": False,
            },
            "safe_sweep_pickup": (
                None if self.mission.safe_sweep_pickup is None else {
                    "target_id": self.mission.safe_sweep_pickup.target_id,
                    "target_class": self.mission.safe_sweep_pickup.target_class,
                    "recovering_original": self.mission.safe_sweep_pickup.recovering_original,
                    "original_classes": sorted(self.mission.safe_sweep_pickup.original_classes),
                    "target_bbox": self.mission.safe_sweep_pickup.target_bbox,
                    "mode": self.mission.safe_sweep_pickup.mode,
                    "frame_floor": self.mission.safe_sweep_pickup.frame_floor,
                    "audit_id": self.mission.safe_sweep_pickup.audit_id,
                    "audit_hits": self.mission.safe_sweep_pickup.audit_hits,
                    "grab_started": self.mission.safe_sweep_pickup.grab_started,
                }
            ),
            "enter_distance_source": "f407_encoder",
            "carried_manifest": list(self.mission.carried_manifest),
            "carried_total_count": self.mission.carried_total_count,
            "carried_has_green": self.mission.carried_has_green,
            "carried_has_core": self.mission.carried_has_core,
            "carried_green_core_mixed": self.mission.carried_green_core_mixed,
            "search_yaw_accum_deg": self.mission.search_yaw_accum_deg,
            "search_candidate_hits": self.mission.search_candidate_hits,
            "search_min_turn_deg": self.mission.settings.search_min_turn_deg,
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
                "safe_corridor_polygon": [list(point) for point in vision.safe_corridor_polygon],
                "safe_sweep_search_polygon": [list(point) for point in vision.safe_sweep_search_polygon],
                "safe_corridor_obstacle_class": vision.safe_corridor_obstacle_class,
                "safe_corridor_obstacle_distance_mm": vision.safe_corridor_obstacle_distance_mm,
                "safe_corridor_obstacle_track_id": vision.safe_corridor_obstacle_track_id,
                "safe_corridor_obstacle_bbox": vision.safe_corridor_obstacle_bbox,
                "safe_corridor_status": vision.safe_corridor_status,
                "ground_localizer_calibrated": vision.ground_localizer_calibrated,
                "safe_sweep_capture_count": (
                    vision.safe_sweep_capture_audit.total_count
                    if vision.safe_sweep_capture_audit is not None else 0
                ),
                "low_conf_green_seen": vision.low_conf_green_seen,
            },
            "initial_stash_enabled": self.mission.settings.initial_stash_enabled,
            "opening_strategy": self.mission.settings.opening_strategy,
            "stash_point_m": list(self.mission.settings.stash_point),
            "max_material_batch_count": self.mission.settings.max_batch_count,
            "initial_stash_done": self.mission.initial_stash_done,
            "first_common_delivered": self.mission.first_common_delivered,
            "first_green_visually_confirmed": self.mission.first_green_visually_confirmed,
            "first_green_bump_used": self.mission.first_green_bump_used,
            "first_fault_code": self.mission.first_fault_code,
            "delivery_count": self.mission.delivery_count,
            "disperse_attempts": self.mission.disperse_attempts,
            "separation_keep_side": self.mission.separation_keep_side,
            "separation_side_attempts": self.mission.separation_side_attempts,
            "separation_request_epoch": self.mission.separation_request_epoch,
            "disperse_expected_done_mode": self.mission.disperse_expected_done_mode,
            "disperse_relay_tx_baseline": self.mission.disperse_relay_tx_baseline,
            "disperse_context": self.mission.disperse_context,
            "disperse_observe_attempts": self.mission.disperse_observe_attempts,
            "disperse_command_accepted": self.mission.disperse_command_accepted,
            "cluster_id": self.mission.cluster_id,
            "cluster_signature": self.mission.cluster_signature,
            "cluster_target_class": self.mission.cluster_target_class,
            "cluster_target_track_id": self.mission.cluster_target_track_id,
            "cluster_target_bbox": (
                None
                if self.mission.cluster_target_bbox is None
                else list(self.mission.cluster_target_bbox)
            ),
            "cluster_bbox": (
                None
                if self.mission.cluster_bbox is None
                else list(self.mission.cluster_bbox)
            ),
            "cluster_keep_side": self.mission.cluster_keep_side,
            "cluster_keep_side_count": self.mission.cluster_keep_side_count,
            "cluster_audit_active": self.mission.cluster_audit_active,
            "cluster_initial_ack": self.mission.cluster_initial_ack,
            "cluster_relay_tx_baseline": self.mission.cluster_relay_tx_baseline,
            "cluster_command_accepted": self.mission.cluster_command_accepted,
            "cluster_execution_seen": self.mission.cluster_execution_seen,
            "cluster_align_turn_deg": self.mission.cluster_align_turn_deg,
            "cluster_command_frame": self.mission.cluster_command_frame,
            "cluster_command_age_ms": (
                None if self.mission.cluster_command_observed_s is None else
                max(0.0, diagnostic_now - self.mission.cluster_command_observed_s) * 1000.0
            ),
            "deferred_cluster_ids": sorted(self.mission.deferred_cluster_ids),
            "approach_initial_ack": self.mission.approach_initial_ack,
            "approach_command_accepted": self.mission.approach_command_accepted,
            "pending_audit_initial_ack": self.mission.pending_audit_initial_ack,
            "grab_initial_ack": self.mission.grab_initial_ack,
            "navigation_initial_ack": self.mission.navigation_initial_ack,
            "staging_zero_initial_ack": self.mission.staging_zero_initial_ack,
            "staging_zero_tx_baseline": self.mission.staging_zero_tx_baseline,
            "staging_zero_accepted": self.mission.staging_zero_accepted,
            "last_tx_opcode": stm.relay_last_mission_command,
            "last_tx_seq": stm.relay_last_mission_sequence,
            "relay_tx_frames": stm.relay_mission_tx_frames,
            "relay_last_payload": list(stm.relay_last_mission_payload),
            "staging_stm_mode": stm.mode,
            "staging_stm_ack": stm.acknowledged_sequence,
            "staging_distance_done": stm.distance_done,
            "staging_gripper_closed": stm.gripper_closed,
            "staging_camera_pitch_cdeg": stm.camera_pitch_cdeg,
            "enter_initial_ack": self.mission.enter_initial_ack,
            "enter_command_accepted": self.mission.enter_command_accepted,
            "task_complete_initial_ack": self.mission.task_complete_initial_ack,
            "return_initial_ack": self.mission.return_initial_ack,
            "return_relay_tx_baseline": self.mission.return_relay_tx_baseline,
            "return_command_accepted": self.mission.return_command_accepted,
            "return_search_frame_floor": self.mission.return_search_frame_floor,
            "return_search_mode_seen": self.mission.return_search_mode_seen,
            "cargo_recheck_pending": self.mission.cargo_recheck_pending,
            "cargo_recheck_context": self.mission.cargo_recheck_context,
            "post_grab_audit_active": self.mission.post_grab_audit_active,
            "transport_audit_camera_ready": self.mission.transport_audit_camera_ready,
            "transport_audit_frame_floor": self.mission.transport_audit_frame_floor,
            "post_grab_camera_ready": self.mission.post_grab_camera_ready,
            "audit_hits": self.mission.audit_hits,
            "audit_wire_mode": self.mission.audit_wire_mode,
            "audit_wire_mode_started_s": self.mission.audit_wire_mode_started_s,
            "audit_last_evidence_s": self.mission.audit_last_evidence_s,
            "capture_observation_valid": vision.capture_observation_valid,
            "capture_observation_reason": vision.capture_observation_reason,
            "safe_sweep_observation_valid": vision.safe_sweep_observation_valid,
            "safe_sweep_observation_reason": vision.safe_sweep_observation_reason,
            "missing_committed_audit_mode": self.mission.missing_committed_audit_mode,
            "committed_audit_handoff_mode": self.mission.committed_audit_handoff_mode,
            "invalid_release_side": self.mission.invalid_release_side,
            "invalid_release_final": self.mission.invalid_release_final,
            "invalid_release_context": self.mission.invalid_release_context,
            "invalid_release_tx_baseline": self.mission.invalid_release_tx_baseline,
            "invalid_backoff_tx_baseline": self.mission.invalid_backoff_tx_baseline,
            "batch": batch,
            "audit": audit,
            "command": command,
        })

    def _record_diagnostic_error(self, error: Exception, now: float) -> None:
        self.diagnostic_error_count = getattr(self, "diagnostic_error_count", 0) + 1
        if now - getattr(self, "last_diagnostic_error_s", -math.inf) >= 2.0:
            self.last_diagnostic_error_s = now
            try:
                print(f"diagnostic_error count={self.diagnostic_error_count}: {error}", file=sys.stderr)
            except Exception:
                pass  # Failure to report diagnostics must not stop control.

    def _record_replay_event(self, output, pose, stm, vision, now: float) -> None:
        if (output.event not in {"start_clear", "safe_zone_acquire_no_box", "load_empty_reobserve",
                                 "safe_zone_entry", "safe_zone_clear_start"} or
                self.replay_event_sink is None):
            return
        try:
            self.replay_event_sink(dict(event=output.event,
                trigger_monotonic_ns=round(now * 1_000_000_000),
                acquire_started_s=self.mission.safe_zone_acquire_started_s,
                upper_state=output.state.value, reason=output.reason, message=output.message,
                vision_frame_sequence=vision.frame_sequence,
                observed_monotonic_s=vision.observed_monotonic_s,
                pose=asdict(pose), stm=asdict(stm), safe_bbox=vision.safe_bbox,
                safe_zone_filter_blocked=vision.safe_zone_filter_blocked,
                command=None if output.command is None else asdict(output.command)))
        except Exception as error:
            self._record_diagnostic_error(error, now)

    def _log_capture_decision(self, output, vision, stm, now: float, pending_task: int, pending_action: int) -> None:
        try:
            self._write_capture_decision(output, vision, stm, now, pending_task, pending_action)
        except Exception as error:
            self._record_diagnostic_error(error, now)

    def _write_capture_decision(self, output, vision, stm, now: float, pending_task: int, pending_action: int) -> None:
        audit = output.audit or vision.capture_audit or self.mission.pending_audit
        wire = output.command.audit if output.command else None
        signature = wire.semantic_signature if wire else None
        roi_reasons = tuple((item["class"], item["included"], item["reason"])
                            for item in vision.roi_decisions)
        matched_rejection = bool(stm.fresh and stm.context_valid and bool(stm.action_status & 4) and
                                 stm.task_id == pending_task and
                                 stm.rejected_action_id == pending_action)
        key = (output.state, output.reason, signature, roi_reasons,
               self.mission.cluster_keep_side, self.mission.disperse_context,
               self.mission.disperse_attempts, matched_rejection,
               audit.left_class_counts if audit else (), audit.right_class_counts if audit else (),
               bool(wire and wire.stable))
        if (key == getattr(self, "last_capture_log_key", None) or
                now - getattr(self, "last_capture_log_s", -math.inf) < 0.5):
            return
        self.last_capture_log_key, self.last_capture_log_s = key, now
        self.events_log.write("capture_decision", {
            "state": output.state.value, "reason": output.reason,
            "frame": vision.frame_sequence, "observed_monotonic_s": vision.observed_monotonic_s,
            "camera_pitch_cdeg": stm.camera_pitch_cdeg, "stm_age_ms": stm.age_ms,
            "roi": vision.roi_decisions, "observation_valid": vision.capture_observation_valid,
            "observation_reason": vision.capture_observation_reason,
            "left_composition": audit.left_class_counts if audit else (),
            "right_composition": audit.right_class_counts if audit else (),
            "left_green": audit.left_green_count if audit else 0,
            "right_green": audit.right_green_count if audit else 0,
            "raw_audit": asdict(audit) if audit else None,
            "wire_signature": signature, "audit_hits": self.mission.audit_hits,
            "stable": bool(wire and wire.stable), "keep_side": self.mission.cluster_keep_side,
            "keep_count": self.mission.cluster_keep_side_count,
            "separation_branch": self.mission.disperse_context,
            "separation_attempts": self.mission.disperse_attempts,
            "separation_no_progress_attempts": (
                self.mission.separation_reviewed_attempt - self.mission.separation_progress_attempt
            ),
            "task_id": self.protocol_task, "action_id": self.protocol_action,
            "rejected_action_id": stm.rejected_action_id, "matched_rejection": matched_rejection,
            "request_task_id": pending_task, "request_action_id": pending_action,
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
                pending_task, pending_action = self.protocol_task, self.protocol_action
                output = self._output_for_cycle(vision, pose, stm, paused, started)
                self._publish(output, stm=stm)
                self._record_replay_event(output, pose, stm, vision, started)
                try:
                    self._update_grab_waitnav_diagnostic(
                        output, pose, stm, started
                    )
                    self._update_safe_sweep_diagnostic(
                        output, vision, pose, stm, started
                    )
                    self._log_command_tx_suppression(output)
                    self._log_capture_decision(output, vision, stm, started, pending_task, pending_action)
                    state_changed = output.state != self.last_state
                    if state_changed:
                        self.events_log.write("state_changed", {
                            "from": self.last_state.value,
                            "to": output.state.value,
                            "message": output.message,
                        })
                        self.last_state = output.state
                    event_key = (output.state, output.event, output.reason)
                    event_logged = bool(output.event and (state_changed or event_key != getattr(self, "last_output_event_key", None) or
                                         started - getattr(self, "last_output_event_s", -math.inf) >= 2.0))
                    if event_logged:
                        self.last_output_event_key, self.last_output_event_s = event_key, started
                        event_data = {
                            "state": output.state.value,
                            "message": output.message,
                        }
                        if output.event == "initial_stash_state_anomaly":
                            event_data.update({
                                "invalid_state": "SEARCH",
                                "initial_stash_enabled": self.mission.settings.initial_stash_enabled,
                                "initial_stash_done": self.mission.initial_stash_done,
                            })
                        audit_snapshot = (
                            output.audit or
                            self.mission.pending_audit or
                            vision.capture_audit
                        )
                        wire_audit = (
                            output.command.audit
                            if output.command is not None else None
                        )
                        if audit_snapshot is not None or wire_audit is not None:
                            left_count = (
                                audit_snapshot.left_count
                                if audit_snapshot is not None else
                                wire_audit.left_count
                            )
                            right_count = (
                                audit_snapshot.right_count
                                if audit_snapshot is not None else
                                wire_audit.right_count
                            )
                            total_count = (
                                audit_snapshot.total_count
                                if audit_snapshot is not None else
                                wire_audit.total_count
                            )
                            event_data.update({
                                "vision_frame_sequence": vision.frame_sequence,
                                "audit_id": (
                                    None if wire_audit is None else wire_audit.audit_id
                                ),
                                "selected_target_class": (
                                    self.mission.cluster_target_class or
                                    self.mission.target_last_class
                                ),
                                "audit_left_count": left_count,
                                "audit_right_count": right_count,
                                "audit_left_green_count": (
                                    None
                                    if audit_snapshot is None else
                                    audit_snapshot.left_green_count
                                ),
                                "audit_right_green_count": (
                                    None
                                    if audit_snapshot is None else
                                    audit_snapshot.right_green_count
                                ),
                                "audit_unassigned_count": max(
                                    0, total_count - left_count - right_count
                                ),
                                "audit_total_count": total_count,
                                "audit_stable": (
                                    wire_audit.stable
                                    if wire_audit is not None else
                                    audit_snapshot.stable
                                ),
                            })
                        if output.event in {
                            "safe_zone_staging_arrived",
                            "staging_camera_firmware_mismatch",
                        }:
                            event_data.update({
                                "upper_state": output.state.value,
                                "last_tx_opcode": stm.relay_last_mission_command,
                                "last_tx_seq": stm.relay_last_mission_sequence,
                                "relay_tx_frames": stm.relay_mission_tx_frames,
                                "relay_last_payload": list(
                                    stm.relay_last_mission_payload
                                ),
                                "stm_mode": stm.mode,
                                "stm_ack": stm.acknowledged_sequence,
                                "distance_done": stm.distance_done,
                                "gripper_closed": stm.gripper_closed,
                                "camera_pitch_cdeg": stm.camera_pitch_cdeg,
                            })
                        if output.event == "safe_zone_acquire_no_box":
                            event_data.update(dict(vision_frame_sequence=vision.frame_sequence,
                                observed_monotonic_s=vision.observed_monotonic_s,
                                pose=asdict(pose), camera_pitch_cdeg=stm.camera_pitch_cdeg,
                                stm_mode=stm.mode, action_status=stm.action_status,
                                task_id=stm.task_id, action_id=stm.action_id,
                                safe_bbox=vision.safe_bbox, safe_zone_filter_blocked=vision.safe_zone_filter_blocked,
                                safe_zone_freeze_hits=self.mission.safe_zone_freeze_hits,
                                cargo_count=len(vision.cargo),
                                command=None if output.command is None else asdict(output.command)))
                        self.events_log.write(output.event, event_data)
                    diagnostics_now = time.monotonic()
                    if (
                        state_changed or
                        event_logged or
                        diagnostics_now - self.last_diagnostics_s >= self.diagnostics_period_s
                    ):
                        self._diagnostics(output, pose, stm, vision, now=started)
                        self.last_diagnostics_s = diagnostics_now
                except Exception as error:
                    self._record_diagnostic_error(error, started)
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
    capture_rois: CaptureRois,
) -> np.ndarray:
    """Render a low-cost preview; task/debug details stay in JSONL logs."""
    output_width, output_height = output_size
    view = cv2.resize(image, (output_width, output_height), interpolation=cv2.INTER_AREA)
    scale_x = output_width / float(IMAGE_WIDTH)
    scale_y = output_height / float(IMAGE_HEIGHT)
    front_check = output.state in {
        CompetitionState.ALIGN_SAFE_ZONE_BY_POSE, CompetitionState.ACQUIRE_SAFE_ZONE,
        CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX, CompetitionState.SAFE_ZONE_CORRIDOR_CHECK,
        CompetitionState.CLEAR_SAFE_ZONE, CompetitionState.WAIT_SAFE_ZONE_CLEAR,
    }
    for polygon, color in (() if front_check else (
        (capture_rois.overall, (0, 255, 255)),
        (capture_rois.left, (255, 128, 0)),
        (capture_rois.right, (0, 128, 255)),
    )):
        scaled_polygon = np.asarray(
            [
                (round(x * scale_x), round(y * scale_y))
                for x, y in polygon
            ],
            dtype=np.int32,
        )
        cv2.polylines(
            view,
            [scaled_polygon],
            True,
            color,
            2,
            cv2.LINE_AA,
        )
    front_polygon = (
        vision.safe_sweep_search_polygon if output.state == CompetitionState.CLEAR_SAFE_ZONE
        else vision.safe_corridor_polygon
    )
    if front_polygon:
        corridor = np.asarray(
            [
                (round(x * scale_x), round(y * scale_y))
                for x, y in front_polygon
            ],
            dtype=np.int32,
        )
        cv2.polylines(view, [corridor], True, (255, 255, 0), 2, cv2.LINE_AA)
    selected_ids = set(output.batch.track_ids) if output.batch is not None else set()
    for item in vision.cargo:
        x, y, box_width, box_height = item.bbox
        if front_check and not cargo_in_front_region(item, front_polygon, vision.safe_bbox):
            color = (100, 100, 100)
        elif item.class_name == "danger_cyan":
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


def competition_display_worker(frames, messages, stop, render_ms, window_mode, capture_rois):
    """GUI owns no planner or UART; display work cannot stall vision publication."""
    cv2.setNumThreads(1)
    try:
        _, output_size = configure_display_window(window_mode)
        messages.put(("display_started", {"window_mode": window_mode, "width": output_size[0],
                                          "height": output_size[1], "screen": list(display_size())}))
        while not stop.is_set():
            try:
                image, pixel_format, vision, output, pose, stm, fps = frames.get(timeout=.03)
            except queue.Empty:
                image = None
            if image is not None:
                begin = time.monotonic()
                bgr = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_NV12) if pixel_format == "nv12" else image
                shown = draw_overlay(bgr, vision, output, pose, stm, fps, output_size, capture_rois)
                cv2.imshow(WINDOW_NAME, shown)
                render_ms.value = (time.monotonic() - begin) * 1000
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                messages.put(("quit", {}))
                break
            if key in (ord("r"), ord("R")):
                messages.put(("reset", {}))
            elif key == 27:
                messages.put(("display_escape_ignored", {"window_mode": window_mode}))
                if window_mode == "fullscreen":
                    window_mode = "normal"
                    _, output_size = configure_display_window(window_mode, create=False)
            elif key in (ord("f"), ord("F")):
                window_mode = "fullscreen" if window_mode == "normal" else "normal"
                _, output_size = configure_display_window(window_mode, create=False)
                messages.put(("display_mode_changed", {"window_mode": window_mode,
                                                       "width":output_size[0], "height":output_size[1]}))
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                messages.put(("display_closed", {"action":"continue_task_without_window"}))
                break
    except Exception as error:
        messages.put(("display_disabled", {"reason":"display_process_error", "error":str(error)}))
    finally:
        cv2.destroyAllWindows()


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
    cv2.setNumThreads(1)  # Match measured X5 settings; avoid competing OpenCV worker pools.
    session = load_json(args.session)
    if not session or session.get("side") not in {"red", "blue"}:
        raise RuntimeError(f"请先在完整比赛地图窗口选择出发区和红蓝方：{args.session}")
    side = str(session["side"])
    start_zone = int(session.get("start_zone", 1))
    opening_strategy = str(session.get("opening_strategy", "attack"))
    diagnostic_writer = AsyncDiagnosticWriter(args.events_log.with_name("competition_logging_status.json"))
    events = JsonlLog(args.events_log, diagnostic_writer)
    detection_log = JsonlLog(args.detections_log, diagnostic_writer)
    events.write("runner_starting", {
        "side": side,
        "start_zone": start_zone,
        "window_mode": args.window_mode,
        "display_fps": args.display_fps,
        "detection_log_fps": args.detection_log_fps,
        "vision_fps": args.vision_fps,
        "score_threshold": args.score_thres,
        "green_supply_score_threshold": GREEN_SUPPLY_SCORE_THRESHOLD,
        "replay_native_frames_per_clip": args.replay_native_frames,
    })
    try:
        pose_deadline = time.monotonic() + args.startup_timeout
        startup_pose = load_pose(args.pose)
        while not startup_pose.valid and not args.wait_for_operator:
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
    range_calibration = ApproachRangeCalibration.load(VISION_ROOT / "config/approach_range.json")
    capture_rois = load_capture_rois(args.capture_roi)
    args.homography = args.homography.resolve()
    homography_metadata = args.homography.with_suffix(
        args.homography.suffix + ".meta.json"
    )
    try:
        localizer = GroundLocalizer.load(args.homography, (IMAGE_WIDTH, IMAGE_HEIGHT))
    except (OSError, ValueError, TypeError) as error:
        # Ground distance is optional; a stale calibration must not block
        # pixel-guided approach, sweep or encoder-controlled delivery.
        events.write("optional_homography_ignored", {"error": str(error)})
        localizer = GroundLocalizer()
    homography_diagnostics = {
        "homography": str(args.homography),
        "metadata": str(homography_metadata),
        "required_resolution": [IMAGE_WIDTH, IMAGE_HEIGHT],
        "localizer.calibrated": localizer.calibrated,
        "required_for_delivery": False,
        "safe_sweep_control": "pixel_approach",
    }
    print(json.dumps(homography_diagnostics, ensure_ascii=False))
    events.write("optional_homography_status", homography_diagnostics)
    if args.preprocess == "vse" and args.decoder != "jpu":
        raise ValueError("VSE NV12路径要求--decoder jpu")
    try:
        detector = Yolo26Pipeline(args.model, load_labels(args.labels), args.score_thres,
                                  args.nms_thres, args.priority, args.bpu_cores,
                                  args.preprocess if args.decoder == "jpu" else "cpu")
    except Exception as error:
        events.write("fatal_exception", {"error_type":type(error).__name__, "error":str(error),
                                          "traceback":traceback.format_exc()})
        raise
    scaler = None  # VSE is owned by the inference worker, never shared between processes.

    mission = CompetitionMission(CompetitionSettings(
        side=side,
        start_zone=start_zone,
        opening_strategy=opening_strategy,
        initial_stash_enabled=not args.disable_initial_stash,
        yield_distance_m=args.yield_distance_mm / 1000.0,
        safe_sweep_capture_offset_mm=args.safe_sweep_capture_offset_mm,
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
    wheel_policy = (LocalizationWheelPolicy(args.localization_wheel_policy_socket,
        args.localization_wheel_policy_session, events)
        if args.localization_wheel_policy_socket else None)
    planner.localization_wheel_policy = wheel_policy

    # Launching this runner is an explicit start request from the operator.
    # Prepare model/camera first: the first CONFIG starts F405 autonomous motion.
    # This runner keeps using the sole C++ UART owner's atomic command relay.
    try:
        startup_details = {
            "model": str(args.model), "model_family": "YOLO26",
            "model_sha256": hashlib.sha256(args.model.read_bytes()).hexdigest(),
            "model_input_wh": [detector.input_width, detector.input_height],
            "model_output_shapes": detector.shapes,
            "initial_preprocess_backend": detector.preprocess,
            "side": side,
            "start_zone": start_zone,
            "initial_stash_enabled": not args.disable_initial_stash,
            "initial_stash_done": mission.initial_stash_done,
            "opening_strategy": mission.settings.opening_strategy,
            "stash_point_m": list(mission.settings.stash_point),
            "max_material_batch_count": mission.settings.max_batch_count,
            "score_threshold": args.score_thres,
            "green_supply_score_threshold": GREEN_SUPPLY_SCORE_THRESHOLD,
            "replay_native_frames_per_clip": args.replay_native_frames,
            "safe_sweep_capture_offset_mm": args.safe_sweep_capture_offset_mm,
            "safe_sweep_capture_offset_ignored": True,
            **homography_diagnostics,
            "homography_calibrated": localizer.calibrated,
            "capture_roi": str(args.capture_roi),
            "capture_polygon_px": [list(point) for point in capture_rois.overall],
            "capture_left_polygon_px": [list(point) for point in capture_rois.left],
            "capture_right_polygon_px": [list(point) for point in capture_rois.right],
            "display_fps": args.display_fps,
            "detection_log_fps": args.detection_log_fps,
            "vision_fps": args.vision_fps,
            "startup_pose": pose_dict(startup_pose),
        }
        camera_device = resolve_camera_device(args.device)
    except Exception as error:
        events.write("fatal_exception", {
            "error_type": type(error).__name__,
            "error": str(error),
            "traceback": traceback.format_exc(),
        })
        if not args.wait_for_operator:
            planner.stop("setup_exception")
        if wheel_policy is not None:
            wheel_policy.close()
        detector.close()
        raise
    camera_decoder = args.decoder
    camera_output_format = "nv12" if args.decoder == "jpu" else "bgr"
    camera: LatestFrameCamera | None = None
    running = True
    exit_reason = "setup_complete"
    display_process = display_frames = display_messages = display_stop = display_render_ms = None
    replay = None
    replay_root = session.get('localization_log_dir')
    if replay_root and session.get('sdk_session_id'):
        try:
            replay = VisionReplay(Path(replay_root), session['sdk_session_id'],
                                  native_frames=args.replay_native_frames)
            replay.start()
            planner.replay_event_sink = replay.record_event
        except Exception as error:
            events.write('vision_replay_unavailable', dict(error=str(error)))
            replay = None
    display_enabled = not args.no_display
    active_window_mode = args.window_mode
    started = time.monotonic()
    camera_recovery_pending = False
    next_camera_retry = started + 1.0
    display_elapsed_ms = 0.0
    log_write_elapsed_ms = 0.0
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
    vision_sequence = 0  # Session-wide: camera restarts must not rewind this ID.
    latest_image: np.ndarray | None = None
    latest_pixel_format = "bgr"
    latest_vision = VisionSnapshot()
    fps_started = started
    inference_frames = 0
    inference_total = 0
    vision_performance_started = None
    vision_performance_bins = []
    vision_performance_last = None
    display_next = started
    next_vision = started
    camera_restarts = 0
    recent_safe_bbox: tuple[int, int, int, int] | None = None
    safe_zone_missing_frames = 0
    recent_carried_bboxes: tuple[tuple[int, int, int, int], ...] = ()
    recent_carried_track_ids: tuple[int, ...] = ()
    recent_carried_pitch_cdeg = 0
    recent_corridor_obstacle: TrackedCargo | None = None
    corridor_memory_pitch = 0
    competition_started = False
    config_sent = False
    operator_stopped = False
    operator_start_ready = not args.wait_for_operator
    warm_ready_written = False
    warm_status_error_reported = False
    next_operator_poll = 0.0

    def write_warm_status():
        nonlocal warm_status_error_reported
        if args.warm_status is not None:
            # Low-rate control state, separate from diagnostic queues. No
            # per-frame disk writes: only first inference and start transition.
            try:
                write_atomic_json(args.warm_status, dict(session_token=args.reset_session_token,
                    ready=True, competition_started=competition_started))
            except OSError as error:
                if not warm_status_error_reported:
                    print(f'预热控制状态写入失败，尚不能向按键入口报告就绪：{error}',file=sys.stderr)
                    events.write('warm_status_write_failed', dict(error=str(error)))
                    warm_status_error_reported = True
                return False
        return True

    def prepare_operator_start():
        nonlocal operator_start_ready, mission, planner, side, start_zone, opening_strategy
        nonlocal tracker, recent_safe_bbox, safe_zone_missing_frames, recent_carried_bboxes
        nonlocal recent_carried_track_ids, recent_carried_pitch_cdeg
        nonlocal recent_corridor_obstacle, corridor_memory_pitch
        if operator_start_ready or operator_stopped or not args.wait_for_operator:
            return
        request = operator_start_request(args.operator_control,args.pose,args.reset_session_token)
        if request is None:
            return
        side, start_zone = request['side'], request['start_zone']
        opening_strategy = request.get('opening_strategy', opening_strategy)
        mission = CompetitionMission(CompetitionSettings(side=side,start_zone=start_zone,
            opening_strategy=opening_strategy,initial_stash_enabled=not args.disable_initial_stash,
            yield_distance_m=args.yield_distance_mm / 1000.0,
            safe_sweep_capture_offset_mm=args.safe_sweep_capture_offset_mm))
        planner = CompetitionPlanner(mission,args.pose,args.stm_status,args.command_file,
            args.diagnostics,events,args.planner_fps)
        planner.localization_wheel_policy = wheel_policy
        if replay is not None:
            planner.replay_event_sink = replay.record_event
        tracker = MultiFrameTracker(config)
        recent_safe_bbox = None
        safe_zone_missing_frames = 0
        recent_carried_bboxes = ()
        recent_carried_track_ids = ()
        recent_carried_pitch_cdeg = 0
        recent_corridor_obstacle = None
        corridor_memory_pitch = 0
        startup_details.update(side=side,start_zone=start_zone,opening_strategy=opening_strategy,
            startup_pose=pose_dict(load_pose(args.pose)),stash_point_m=list(mission.settings.stash_point))
        operator_start_ready = True
        events.write('warm_reference_bound', dict(session_token=args.reset_session_token,
            reference=request['reference'],side=side,start_zone=start_zone))


    def apply_operator_stop(*, force: bool = False) -> bool:
        nonlocal operator_stopped, next_operator_poll
        if operator_stopped:
            return True
        now = time.monotonic()
        if not force and now < next_operator_poll:
            return False
        next_operator_poll = now + 0.02
        if not operator_stop_requested(args.operator_control, args.reset_session_token):
            return False
        operator_stopped = True
        planner.stop("operator_stop")
        with planner.lock:
            planner.latest_output = CompetitionOutput(
                mission.state, None, "本轮已停止；保留视觉和定位观察，长按退出后可重新开始",
                suppress_command_tx=True, suppression_reason="operator_stop",
                reason="operator_stop")
        events.write("operator_stopped", {
            "competition_started": competition_started,
            "session_token": args.reset_session_token,
            "action": "observe_without_further_commands",
        })
        return True

    def start_after_first_vision() -> None:
        nonlocal competition_started, config_sent, warm_ready_written
        if (competition_started or not operator_start_ready
                or apply_operator_stop(force=True) or not running):
            return
        def remember_config_sent():
            nonlocal config_sent
            config_sent = True
        if not publish_start_config(args.command_file,side,start_zone,
                lambda: running and not apply_operator_stop(force=True),remember_config_sent):
            return
        competition_started = True
        warm_ready_written = write_warm_status()
        events.write("competition_started", {
            **startup_details,
            "first_vision_sequence": vision_sequence,
            "start_basis": "warm_operator_start_and_reference_bound" if args.wait_for_operator else "explicit_session_start_and_first_inference",
        })
        planner.set_paused(False, reason="camera_recovery")
        planner.start()

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
        # No planner or CONFIG until actual camera inference is ready. Existing
        # in-session camera recovery retains its original pause/resume behavior.
        try:
            camera = create_camera()
            camera.start()
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
            import multiprocessing as mp
            context = mp.get_context("spawn")
            display_frames, display_messages = context.Queue(1), context.Queue()
            display_stop = context.Event()
            display_render_ms = context.Value("d", 0.0)
            display_process = context.Process(target=competition_display_worker,
                args=(display_frames, display_messages, display_stop, display_render_ms,
                      active_window_mode, capture_rois), daemon=True)
            try:
                display_process.start()
            except Exception as error:
                display_enabled = False
                display_process = None
                display_frames.close()
                display_messages.close()
                display_frames = display_messages = None
                events.write("display_disabled", {"reason":"window_process_start_failed", "error":str(error)})

        while running:
            if planner.error is not None:
                raise RuntimeError(f"比赛规划线程退出：{planner.error}")
            now = time.monotonic()
            apply_operator_stop()
            prepare_operator_start()
            planner.observe_without_commands(now)
            if display_messages is not None:
                while True:
                    try:
                        event, details = display_messages.get_nowait()
                    except queue.Empty:
                        break
                    if event == "quit":
                        exit_reason, running = "user_q", False
                    elif event == "reset":
                        if args.reset_request is not None and args.reset_session_token:
                            try:
                                write_atomic_json(args.reset_request, {"session_token":args.reset_session_token,
                                                                      "timestamp_monotonic_ns":time.monotonic_ns()})
                                events.write("session_reset_forwarded", {"source":"vision_window"})
                                exit_reason, running = "user_reset", False
                            except OSError as error:
                                events.write("session_reset_forward_failed", {"error":str(error)})
                        else:
                            events.write("session_reset_unavailable", {"reason":"runner_not_owned_by_startup_window"})
                    else:
                        events.write(event, details)
                        if event in {"display_closed", "display_disabled"}:
                            display_enabled = False
                if display_enabled and not display_process.is_alive():
                    display_enabled = False
                    events.write("display_disabled", {"reason":"display_process_exited", "action":"continue_task_without_window"})
            if not running:
                break
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
                        planner.set_paused(False, reason="camera_recovery")
                        last_packet_id = 0
                        last_inference_id = 0
                        detector.reset()
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
                    planner.set_paused(True, reason="camera_recovery")
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
                    detector.reset()
                    next_vision = now
                    events.write("camera_restarted", {
                        "decoder": camera_decoder,
                        "output_format": camera_output_format,
                        "retry": camera_restarts,
                    })
                    planner.set_paused(False, reason="camera_recovery")
                    continue
                planner.set_paused(True, reason="camera_recovery")
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
            fetch_started = time.monotonic()
            packet = camera.latest()
            fetch_ms = (time.monotonic() - fetch_started) * 1000.0
            if packet is not None and packet.frame_id != last_inference_id and now >= next_vision:
                next_vision = now + 1.0 / args.vision_fps
                last_inference_id = packet.frame_id
                detector.submit(packet)
            result = detector.receive()
            if result is not None:
                packet, detections, timing = result
                snapshot_started = time.monotonic()
                # Display exactly the image that produced latest_vision,
                # not a newer camera frame underneath older ROI/results.
                latest_image = packet.image  # Camera publishes an owned immutable copy.
                latest_pixel_format = packet.pixel_format
                vision_sequence += 1
                low_conf_green_seen = any(
                    item.class_name == "green_supply" and
                    float(item.confidence) < GREEN_SUPPLY_SCORE_THRESHOLD
                    for item in detections
                )
                detections = [
                    item for item in detections
                    if item.class_name != "green_supply" or
                    float(item.confidence) >= GREEN_SUPPLY_SCORE_THRESHOLD
                ]
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
                    safe_bbox = recent_safe_bbox
                    safe_zone_filter_blocked = True
                else:
                    recent_safe_bbox = None
                    safe_zone_missing_frames = 3
                    safe_bbox = None
                    safe_zone_filter_blocked = False
                remember_corridor = (
                    mission.state in {CompetitionState.ACQUIRE_SAFE_ZONE,
                        CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX,
                        CompetitionState.SAFE_ZONE_CORRIDOR_CHECK,
                        CompetitionState.ENTER_SAFE_ZONE} and
                    stm.camera_pitch_cdeg in {11500, 12000}
                )
                if not remember_corridor or stm.camera_pitch_cdeg != corridor_memory_pitch:
                    recent_corridor_obstacle = None
                corridor_memory_pitch = stm.camera_pitch_cdeg
                latest_vision = make_vision_snapshot(
                    detections,
                    tracks,
                    localizer,
                    safe_bbox,
                    stm,
                    mission,
                    vision_sequence,
                    safe_zone_filter_blocked,
                    capture_rois,
                    low_conf_green_seen,
                    recent_carried_bboxes,
                    capture_quality_reason=capture_image_quality(
                        packet.image, packet.pixel_format, capture_rois,
                        args.capture_roi.is_file(),
                    ),
                    observed_s=packet.published_ns / 1_000_000_000.0,
                    range_calibration=range_calibration,
                    recent_carried_track_ids=recent_carried_track_ids,
                    recent_corridor_obstacle=recent_corridor_obstacle,
                    recent_carried_pitch_cdeg=recent_carried_pitch_cdeg,
                )
                if (remember_corridor and latest_vision.safe_corridor_obstacle_class is not None):
                    recent_corridor_obstacle = TrackedCargo(
                        latest_vision.safe_corridor_obstacle_track_id,
                        latest_vision.safe_corridor_obstacle_class, 0.0,
                        latest_vision.safe_corridor_obstacle_bbox, visible=False,
                    )
                elif remember_corridor and latest_vision.safe_corridor_polygon:
                    recent_corridor_obstacle = None
                if latest_vision.carried_reference_track_ids:
                    recent_carried_track_ids = latest_vision.carried_reference_track_ids
                elif not mission.carried_manifest:
                    recent_carried_track_ids = ()
                if not mission.carried_manifest:
                    recent_carried_bboxes = ()
                    recent_carried_pitch_cdeg = 0
                elif latest_vision.carried_reference_bboxes:
                    recent_carried_bboxes = (
                        latest_vision.carried_reference_bboxes
                    )
                    # An angle report may precede the actual changed image.
                    # Snapshot emits boxes only after this view's carrying
                    # geometry matches, so the old 140-degree anchor survives.
                    recent_carried_pitch_cdeg = stm.camera_pitch_cdeg
                planner.set_vision(latest_vision)
                if not warm_ready_written:
                    warm_ready_written = write_warm_status()
                    events.write('vision_prewarm_ready', dict(session_token=args.reset_session_token,
                        first_vision_sequence=vision_sequence,wait_for_operator=args.wait_for_operator))
                start_after_first_vision()
                vision_performance_last = time.monotonic()
                if vision_performance_started is None:
                    vision_performance_started = vision_performance_last
                bucket = int(vision_performance_last - vision_performance_started)
                while len(vision_performance_bins) <= bucket:
                    vision_performance_bins.append(0)
                vision_performance_bins[bucket] += 1
                timing.update({"fetch": fetch_ms, "roi": latest_vision.roi_elapsed_ms,
                               "snapshot": (time.monotonic() - snapshot_started) * 1000.0,
                               "capture_to_publish": (time.monotonic() - latest_vision.observed_monotonic_s) * 1000.0,
                               "previous_display": display_elapsed_ms,
                               "previous_log_write": log_write_elapsed_ms})
                log_now = time.monotonic()
                if log_now >= next_detection_log:
                    log_write_started = time.monotonic()
                    detection_log.write(
                        "frame",
                        detection_log_record(
                            vision_sequence,
                            timing,
                            detection_objects,
                            latest_vision.cargo,
                        ),
                    )
                    log_write_elapsed_ms = (time.monotonic() - log_write_started) * 1000.0
                    next_detection_log = log_now + 1.0 / args.detection_log_fps
                if replay is not None and replay.due(log_now):
                    try:
                        replay_output, replay_pose, replay_stm = planner.snapshot()
                        replay_record = detection_log_record(vision_sequence,timing,detection_objects,latest_vision.cargo)
                        replay.submit(packet.image,packet.pixel_format,dict(
                            sdk_session_id=session.get('sdk_session_id'),
                            frame_sequence=vision_sequence, camera_frame_id=packet.frame_id,
                            capture_monotonic_ns=packet.published_ns,
                            snapshot_monotonic_ns=time.monotonic_ns(),
                            source_size=[IMAGE_WIDTH,IMAGE_HEIGHT],
                            detections=replay_record['detections'], tracks=replay_record['tracks'],
                            pose=asdict(replay_pose), stm=asdict(replay_stm),
                            upper_state=replay_output.state.value, reason=replay_output.reason,
                            message=replay_output.message,
                            command=None if replay_output.command is None else asdict(replay_output.command),
                            acquire_started_s=mission.safe_zone_acquire_started_s,
                            safe_bbox=latest_vision.safe_bbox,
                            safe_zone_filter_blocked=latest_vision.safe_zone_filter_blocked,
                            roi_polygons=[capture_rois.overall,latest_vision.safe_corridor_polygon],
                            model=str(args.model), model_family='YOLO26',
                            pose_alignment='latest planner state, not capture synchronized'),log_now)
                    except Exception as error:
                        replay.error = str(error)  # Recording never aborts the mission.
                inference_frames += 1
                inference_total += 1
            if now - fps_started >= 1.0:
                current_fps = inference_frames / max(now - fps_started, 1e-6)
                inference_frames = 0
                fps_started = now
            else:
                current_fps = inference_frames / max(now - fps_started, 1e-6)
            if display_enabled and now >= display_next and latest_image is not None:
                display_next = now + 1.0 / args.display_fps
                output, pose, current_stm = planner.snapshot()
                try:
                    display_frames.put_nowait((latest_image, latest_pixel_format, latest_vision,
                                               output, pose, current_stm, current_fps))
                except queue.Full:
                    pass  # Skip preview frames, never block inference or mission control.
                display_elapsed_ms = display_render_ms.value
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
        if replay is not None: replay.request_close()
        if config_sent or competition_started or operator_stopped or not args.wait_for_operator:
            planner.stop(exit_reason)
        if wheel_policy is not None:
            wheel_policy.close()
        if replay is not None:
            events.write('vision_replay_closed', replay.close())
        detector.close()
        if camera is not None:
            camera.stop()
        if scaler is not None:
            scaler.close()
        if display_process is not None:
            display_stop.set()
            display_process.join(3)
            if display_process.is_alive():
                display_process.terminate()
                display_process.join()
            display_frames.cancel_join_thread()
            display_frames.close()
            display_messages.close()
        completed_windows = vision_performance_bins[:-1]
        measured = completed_windows[5:]
        events.write("vision_performance", {
            "model": str(args.model), "model_family": "YOLO26", "warmup_seconds": 5,
            "metric": "unique frames after tracking/ROI and planner.set_vision",
            "fps_1s_including_warmup": completed_windows,
            "measurement_seconds": len(measured), "frames_after_warmup": sum(measured),
            "mean_fps": sum(measured)/len(measured) if measured else None,
            "min_fps_1s": min(measured) if measured else None,
            "stable_30fps": bool(len(measured) >= 30 and min(measured) >= 30),
        })
        events.write("competition_stopped", {
            "state": mission.state.value,
            "delivery_count": mission.delivery_count,
            "first_common_delivered": mission.first_common_delivered,
            "first_green_visually_confirmed": mission.first_green_visually_confirmed,
            "delivery_completion_basis": mission.delivery_completion_basis,
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
            "competition_started": competition_started,
            "config_sent": config_sent,
            "operator_stopped": operator_stopped,
        })
        logging_status = diagnostic_writer.close()
        if logging_status["incomplete"]:
            print("competition diagnostic logging incomplete: " + json.dumps(logging_status), file=sys.stderr)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        raise SystemExit(1)
