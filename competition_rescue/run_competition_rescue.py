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
import signal
import sys
import threading
import time
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

from protocol import CMD_HOLD  # noqa: E402
from state_machine import (  # noqa: E402
    CARGO_CLASSES,
    CargoAudit,
    CompetitionMission,
    CompetitionOutput,
    CompetitionSettings,
    PoseSnapshot,
    StmSnapshot,
    TrackedCargo,
    VisionSnapshot,
)


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
    parser.add_argument("--window-mode", choices=("fullscreen", "normal"), default="fullscreen")
    parser.add_argument("--display-fps", type=float, default=15.0)
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; 0 runs until stopped")
    parser.add_argument(
        "--disable-initial-stash",
        action="store_true",
        help="debug only: skip the configured opening temporary-relocation strategy",
    )
    parser.add_argument("--stuck-timeout", type=float, default=1.5)
    parser.add_argument("--yield-distance-mm", type=float, default=250.0)
    parser.add_argument("--escape-attempts", type=int, default=2)
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
        return PoseSnapshot(
            valid=data.get("quality") in {"GOOD", "DEGRADED"} and age_ms <= 250.0,
            x_m=float(pose["x_m"]),
            y_m=float(pose["y_m"]),
            yaw_deg=yaw % 360.0,
            age_ms=age_ms,
        )
    except (KeyError, TypeError, ValueError):
        return PoseSnapshot()


def load_stm(path: Path) -> StmSnapshot:
    data = load_json(path)
    if not data:
        return StmSnapshot()
    try:
        timestamp = int(data["timestamp_monotonic_ns"])
        age_ms = max(0.0, (time.monotonic_ns() - timestamp) / 1_000_000.0)
        return StmSnapshot(
            mode=int(data.get("mode", 0)),
            flags=int(data.get("flags", 0)),
            age_ms=age_ms,
            fault_code=int(data.get("fault_code", 0)),
            acknowledged_sequence=int(data.get("acknowledged_sequence", 0)),
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
            inside = (
                sx <= tx + tw * 0.5 <= sx + sw and
                sy <= ty + th * 0.5 <= sy + sh
            )
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


def audit_from_cargo(items: tuple[TrackedCargo, ...]) -> CargoAudit | None:
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
    has_material = bool(all_names & {"green_supply", "core_black"})
    injury_mixed = has_injury and has_material
    left_name, right_name = side_class(left), side_class(right)
    left_invalid = any(item.class_name == "danger_cyan" for item in left) or (
        any(item.class_name == "injured_orange" for item in left) and
        any(item.class_name in {"green_supply", "core_black"} for item in left)
    )
    right_invalid = any(item.class_name == "danger_cyan" for item in right) or (
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
    )


def danger_ahead(detections) -> tuple[bool, str]:
    candidates = [item for item in detections if item.class_name == "danger_cyan"]
    if not candidates:
        return False, "unknown"
    ahead = []
    for item in candidates:
        x, y, width, height = item.bbox
        cx = x + width * 0.5
        cy = y + height * 0.5
        if cy >= IMAGE_HEIGHT * 0.42 and abs(cx - IMAGE_WIDTH * 0.5) <= IMAGE_WIDTH * 0.38:
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
) -> VisionSnapshot:
    cargo = tracked_cargo(tracks, safe_bbox)
    capture = tuple(item for item in cargo if item.visible and item.center_px[1] >= IMAGE_HEIGHT * 0.25)
    audit = audit_from_cargo(capture) if stm.claw_visible else None
    selected = mission.selected_batch
    selected_ids = set(selected.track_ids) if selected is not None else set()
    selected_classes = set(selected.classes) if selected is not None else set()
    delivery_items = [
        item for item in cargo
        if item.visible and (item.track_id in selected_ids or item.class_name in selected_classes)
    ]
    delivery_inside = any(item.inside_safe_zone for item in delivery_items)
    delivery_outside = any(not item.inside_safe_zone for item in delivery_items)
    danger, side = danger_ahead(detections)
    return VisionSnapshot(
        frame_sequence=frame_sequence,
        cargo=cargo,
        capture_cargo=capture,
        safe_bbox=safe_bbox,
        danger_ahead=danger,
        danger_side=side,
        capture_audit=audit,
        delivery_target_found=bool(delivery_items),
        delivery_target_inside_safe_zone=delivery_inside,
        delivery_target_outside_safe_zone=delivery_outside,
    )


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
        self.thread = threading.Thread(target=self._run, name="competition-planner", daemon=True)

    def set_vision(self, vision: VisionSnapshot) -> None:
        with self.lock:
            self.latest_vision = vision

    def snapshot(self):
        with self.lock:
            return self.latest_output, self.latest_pose, self.latest_stm

    def start(self) -> None:
        self.running = True
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)
        try:
            # Never leave a motion or release command as the last relay frame
            # when the vision window exits.
            write_command_frame(self.command_path, self._hold_frame())
        except OSError:
            pass

    def _hold_frame(self) -> bytes:
        from protocol import mission_frame
        return mission_frame(self.sequence, CMD_HOLD)

    def _publish(self, output: CompetitionOutput) -> None:
        command = output.command
        if command is None:
            write_command_frame(self.command_path, self._hold_frame())
        else:
            write_command_frame(self.command_path, command.to_frame(self.sequence))
        self.sequence = (self.sequence + 1) & 0xFF

    def _diagnostics(self, output: CompetitionOutput, pose: PoseSnapshot, stm: StmSnapshot, vision: VisionSnapshot) -> None:
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
            "pose": pose_dict(pose),
            "stm": stm_dict(stm),
            "vision": {
                "frame_sequence": vision.frame_sequence,
                "cargo_count": len(vision.cargo),
                "danger_ahead": vision.danger_ahead,
                "danger_side": vision.danger_side,
                "delivery_target_found": vision.delivery_target_found,
                "delivery_target_inside_safe_zone": vision.delivery_target_inside_safe_zone,
                "delivery_target_outside_safe_zone": vision.delivery_target_outside_safe_zone,
            },
            "initial_stash_done": self.mission.initial_stash_done,
            "first_common_delivered": self.mission.first_common_delivered,
            "delivery_count": self.mission.delivery_count,
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
                output = self.mission.step(vision, pose, stm, started)
                self._publish(output)
                if output.state != self.last_state:
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
                self._diagnostics(output, pose, stm, vision)
                with self.lock:
                    self.latest_output = output
                    self.latest_pose = pose
                    self.latest_stm = stm
                elapsed = time.monotonic() - started
                time.sleep(max(0.001, self.period_s - elapsed))
        except Exception as error:  # pragma: no cover - surfaced by main loop
            self.error = error
            self.events_log.write("planner_error", {"error": str(error)})


def draw_overlay(image: np.ndarray, vision: VisionSnapshot, output: CompetitionOutput, pose: PoseSnapshot, stm: StmSnapshot, fps: float) -> np.ndarray:
    view = image.copy()
    selected_ids = set(output.batch.track_ids) if output.batch is not None else set()
    for item in vision.cargo:
        x, y, width, height = item.bbox
        if item.class_name == "danger_cyan":
            color = (0, 0, 255)
        elif item.track_id in selected_ids:
            color = (0, 255, 255)
        else:
            color = (0, 210, 0)
        cv2.rectangle(view, (x, y), (x + width, y + height), color, 3)
        cv2.putText(
            view,
            f"{item.class_name}#{item.track_id} {item.confidence:.2f}",
            (x, max(24, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
    if vision.safe_bbox is not None:
        x, y, width, height = vision.safe_bbox
        cv2.rectangle(view, (x, y), (x + width, y + height), (255, 0, 255), 3)
    lines = [
        f"COMPETITION {fps:.1f} FPS  state={output.state.value}",
        output.message,
        f"pose=({pose.x_m:+.2f},{pose.y_m:+.2f}) yaw={pose.yaw_deg:.1f} valid={int(pose.valid)} age={pose.age_ms:.0f}ms",
        f"stm mode={stm.mode} flags=0x{stm.flags:02X} motors={int(stm.motors_active)} claw={int(stm.claw_visible)} closed={int(stm.gripper_closed)} fault={stm.fault_code}",
        f"cargo={len(vision.cargo)} danger_ahead={int(vision.danger_ahead)} side={vision.danger_side} selected={sorted(selected_ids)}",
        f"stash={int(output.batch.initial_stash) if output.batch else 0} first_green_done={int(output.batch is not None and not output.batch.initial_stash) if output.batch else 0}",
        "WASD/keyboard is disabled in competition mode; Q/Esc exits display",
    ]
    for index, line in enumerate(lines):
        cv2.putText(view, line, (14, 32 + index * 32), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
    return view


def validate_args(args: argparse.Namespace) -> None:
    require_native_resolution(IMAGE_WIDTH, IMAGE_HEIGHT)
    if not 0.0 < args.score_thres < 1.0 or not 0.0 < args.nms_thres < 1.0:
        raise ValueError("score-thres和nms-thres必须在0..1之间")
    if args.vision_fps <= 0 or args.planner_fps <= 0 or args.display_fps <= 0:
        raise ValueError("FPS必须为正数")
    if args.startup_timeout <= 0 or args.stuck_timeout <= 0:
        raise ValueError("startup/stuck timeout必须为正数")
    if args.yield_distance_mm <= 0 or args.escape_attempts <= 0:
        raise ValueError("退让距离和脱困次数必须为正数")


def main() -> int:
    args = arguments()
    validate_args(args)
    session = load_json(args.session)
    if not session or session.get("side") not in {"red", "blue"}:
        raise RuntimeError(f"请先在完整比赛地图窗口选择出发区和红蓝方：{args.session}")
    side = str(session["side"])
    start_zone = int(session.get("start_zone", 1))
    pose_deadline = time.monotonic() + args.startup_timeout
    startup_pose = load_pose(args.pose)
    while not startup_pose.valid:
        if time.monotonic() >= pose_deadline:
            raise RuntimeError("定位尚未就绪，完整比赛流程不会发送运动命令")
        time.sleep(0.05)
        startup_pose = load_pose(args.pose)

    config = load_config(args.config)
    localizer = GroundLocalizer.load(args.homography, (IMAGE_WIDTH, IMAGE_HEIGHT))
    detector = X5YoloV8(
        args.model,
        load_labels(args.labels),
        args.score_thres,
        args.nms_thres,
        args.priority,
        args.bpu_cores,
    )
    scaler: VseScaler | None = None
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

    mission = CompetitionMission(CompetitionSettings(
        side=side,
        start_zone=start_zone,
        initial_stash_enabled=not args.disable_initial_stash,
        stuck_timeout_s=args.stuck_timeout,
        yield_distance_m=args.yield_distance_mm / 1000.0,
        max_escape_attempts=args.escape_attempts,
    ))
    events = JsonlLog(args.events_log)
    detection_log = JsonlLog(args.detections_log)
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
    for sequence in range(3):
        write_command_frame(args.command_file, config_frame(sequence, 0x11 if red_side else 0x12, start_zone))
        time.sleep(0.04)
    events.write("competition_started", {
        "side": side,
        "start_zone": start_zone,
        "initial_stash_enabled": not args.disable_initial_stash,
        "score_threshold": args.score_thres,
        "startup_pose": pose_dict(startup_pose),
    })

    camera = LatestFrameCamera(
        resolve_camera_device(args.device),
        IMAGE_WIDTH,
        IMAGE_HEIGHT,
        args.camera_fps,
        decoder=args.decoder,
        decode_fps=args.decode_fps,
        output_format="nv12" if scaler is not None else "bgr",
    )
    running = True

    def stop(_signal, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    tracker = MultiFrameTracker(config)
    camera.start()
    planner.start()
    started = time.monotonic()
    last_frame_id = 0
    latest_image: np.ndarray | None = None
    latest_pixel_format = "bgr"
    latest_vision = VisionSnapshot()
    fps_started = started
    fps_frames = 0
    display_next = started
    try:
        while running:
            if planner.error is not None:
                raise RuntimeError(f"比赛规划线程退出：{planner.error}")
            now = time.monotonic()
            if args.duration > 0.0 and now - started >= args.duration:
                break
            error = camera.check_error()
            if error:
                raise RuntimeError(error)
            packet = camera.latest()
            if packet is None or packet.frame_id == last_frame_id:
                time.sleep(0.001)
                continue
            last_frame_id = packet.frame_id
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
            safe_bbox = safe_bbox_from_detections(detections, safe_class)
            latest_vision = make_vision_snapshot(
                detections, tracks, localizer, safe_bbox, stm, mission, packet.frame_id
            )
            planner.set_vision(latest_vision)
            detection_log.write("frame", {
                "frame_sequence": packet.frame_id,
                "timing_ms": timing,
                "detections": [
                    {
                        "class": item.class_name,
                        "confidence": round(float(item.confidence), 4),
                        "bbox": list(item.bbox),
                        "center": list(item.center),
                        "ground_xy_mm": (
                            None if item.ground_xy_mm is None else list(item.ground_xy_mm)
                        ),
                    }
                    for item in detection_objects
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
                    for item in latest_vision.cargo
                ],
            })
            latest_image = packet.image
            latest_pixel_format = packet.pixel_format
            fps_frames += 1
            if now - fps_started >= 1.0:
                current_fps = fps_frames / max(now - fps_started, 1e-6)
                fps_frames = 0
                fps_started = now
            else:
                current_fps = fps_frames / max(now - fps_started, 1e-6)
            if not args.no_display and latest_image is not None and now >= display_next:
                display_next = now + 1.0 / args.display_fps
                output, pose, current_stm = planner.snapshot()
                shown_image = (
                    cv2.cvtColor(latest_image, cv2.COLOR_YUV2BGR_NV12)
                    if latest_pixel_format == "nv12" else latest_image
                )
                shown = draw_overlay(shown_image, latest_vision, output, pose, current_stm, current_fps)
                cv2.imshow("complete_rescue_competition", shown)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q"), ord("Q")):
                    running = False
            time.sleep(0.0005)
    finally:
        planner.stop()
        camera.stop()
        if scaler is not None:
            scaler.close()
        if not args.no_display:
            cv2.destroyAllWindows()
        events.write("competition_stopped", {
            "state": mission.state.value,
            "delivery_count": mission.delivery_count,
            "first_common_delivered": mission.first_common_delivered,
        })
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        raise SystemExit(1)
