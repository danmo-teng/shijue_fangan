#!/usr/bin/env python3
"""Camera/state-machine adapter for continuous rescue cargo delivery."""
from __future__ import annotations

import argparse
import json
import math
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = ROOT.parent
T265_DELIVERY_EXTRA_M = 0.030
sys.path.insert(0, str(PROJECT_ROOT / "vision"))

from state_machine import (
    MissionOutput,
    MissionSettings,
    MissionState,
    PoseInput,
    RescueMission,
    VisionInput,
    target_inside_safe_zone,
)
from rescue_vision.camera import LatestFrameCamera, resolve_camera_device
from rescue_vision.config import load_config
from rescue_vision.detector import TraditionalDetector
from rescue_vision.localizer import GroundLocalizer
from rescue_vision.mission_protocol import Stm32Status, write_command_frame
from rescue_vision.safe_zone import bbox_center_in_safe_zone
from rescue_vision.vision_protocol import IMAGE_HEIGHT, IMAGE_WIDTH, config_frame
from rescue_vision.vse import VseScaler
from run_yolo_x5 import DEFAULT_LABELS, DEFAULT_MODEL, X5YoloV8, load_labels


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="连续物资寻找、抓取与分区投送")
    parser.add_argument("--detector", choices=("yolo", "traditional"), default="yolo")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--decoder", choices=("jpu", "software"), default="jpu")
    parser.add_argument("--camera-fps", type=int, default=180)
    parser.add_argument("--decode-fps", type=float, default=60.0)
    parser.add_argument("--vision-fps", type=float, default=50.0)
    parser.add_argument("--planner-fps", type=float, default=50.0)
    parser.add_argument("--preprocess", choices=("auto", "vse", "cpu"), default="auto")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--labels", type=Path, default=DEFAULT_LABELS)
    parser.add_argument("--score-thres", type=float, default=0.50)
    parser.add_argument("--nms-thres", type=float, default=0.45)
    parser.add_argument("--priority", type=int, default=0)
    parser.add_argument("--bpu-cores", type=int, nargs="+", default=[0, 1])
    parser.add_argument("--confirm-frames", type=int, default=3)
    parser.add_argument("--zone-center-x-mm", type=float, default=150.0)
    parser.add_argument("--delivery-stationary-seconds", type=float, default=0.8)
    parser.add_argument("--delivery-stationary-tolerance-mm", type=float, default=25.0)
    parser.add_argument("--center-stop-radius-mm", type=float, default=600.0)
    parser.add_argument("--nav-fence-heading-tolerance-deg", type=float, default=12.0)
    parser.add_argument("--nav-near-fence-distance-m", type=float, default=0.30)
    parser.add_argument("--nav-near-fence-heading-limit-deg", type=float, default=10.0)
    parser.add_argument("--startup-timeout", type=float, default=30.0)
    parser.add_argument("--start-pose-tolerance-mm", type=float, default=20.0)
    parser.add_argument("--session", type=Path, default=PROJECT_ROOT / "rescue_map/runtime/session.json")
    parser.add_argument("--pose", type=Path, default=PROJECT_ROOT / "rescue_map/runtime/localization_result.json")
    parser.add_argument("--stm-status", type=Path, default=PROJECT_ROOT / "rescue_map/runtime/stm32_status.json")
    parser.add_argument("--command-file", type=Path, default=PROJECT_ROOT / "rescue_map/runtime/uart_command.bin")
    parser.add_argument("--contact-output", type=Path, default=PROJECT_ROOT / "rescue_map/runtime/delivery_contact_pose.json")
    parser.add_argument("--diagnostics", type=Path, default=PROJECT_ROOT / "rescue_map/runtime/mission_diagnostics.json")
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "vision/config/rescue_vision.json")
    parser.add_argument("--homography", type=Path, default=PROJECT_ROOT / "vision/config/homography.txt")
    parser.add_argument("--window-mode", choices=("fullscreen", "normal"), default="fullscreen")
    parser.add_argument("--display-fps", type=float, default=15.0)
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; 0 runs until stopped")
    return parser.parse_args()


def load_json(path: Path) -> dict | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError):
        return None


def load_pose(path: Path) -> PoseInput:
    data = load_json(path)
    if not data:
        return PoseInput()
    try:
        age_ms = (time.monotonic_ns() - int(data["timestamp_monotonic_ns"])) / 1_000_000.0
        pose = data["pose"]
        wheel = data.get("wheel", {})
        navigation = data.get("navigation", {})
        valid = data.get("quality") in {"GOOD", "DEGRADED"} and 0 <= age_ms <= 250
        navigation_distance_m = float(
            navigation.get(
                "distance_compensation_m",
                navigation.get("wheel_progress_m", 0.0),
            )
        )
        navigation_distance_valid = (
            bool(navigation.get(
                "distance_compensation_valid",
                navigation.get("wheel_primary", False),
            ))
            and math.isfinite(navigation_distance_m)
            and float(wheel.get("last_update_age_ms", float("inf"))) <= 150.0
        )
        return PoseInput(
            valid, float(pose["x_m"]), float(pose["y_m"]),
            float(pose["yaw_deg"]), max(0.0, age_ms),
            navigation_distance_m,
            navigation_distance_valid,
            bool(navigation.get("wheel_primary", False)),
            float(wheel.get("fusion_weight", 0.0)),
        )
    except (KeyError, TypeError, ValueError):
        return PoseInput()


def load_stm_status(path: Path) -> Stm32Status:
    data = load_json(path)
    if not data:
        return Stm32Status()
    try:
        age_ms = max(0.0, (time.monotonic_ns() - int(data["timestamp_monotonic_ns"])) / 1_000_000.0)
        data = dict(data)
        data["age_ms"] = age_ms
        return Stm32Status.from_json(data)
    except (KeyError, TypeError, ValueError):
        return Stm32Status()


def validate_start_pose(session: dict, pose: PoseInput,
                        tolerance_mm: float) -> None:
    zone = int(session["start_zone"])
    signs = {
        1: (-1.0, 1.0),
        2: (1.0, 1.0),
        3: (-1.0, -1.0),
        4: (1.0, -1.0),
    }
    if zone not in signs:
        raise RuntimeError(f"START_POSE_MISMATCH invalid start zone: {zone}")
    expected_x = signs[zone][0] * 1.350
    expected_y = signs[zone][1] * 1.350
    error_mm = math.hypot(
        pose.x_m - expected_x, pose.y_m - expected_y
    ) * 1000.0
    if not pose.valid or error_mm > tolerance_mm:
        raise RuntimeError(
            "START_POSE_MISMATCH "
            f"zone={zone} expected=({expected_x:+.3f},{expected_y:+.3f})m "
            f"actual=({pose.x_m:+.3f},{pose.y_m:+.3f})m error={error_mm:.1f}mm "
            f"limit={tolerance_mm:.1f}mm"
        )


def write_contact_pose(path: Path, observed: PoseInput,
                       reference: tuple[float, float, float], side: str,
                       cargo_class: str, delivery_count: int) -> None:
    reference_x, reference_y, reference_yaw = reference
    document = {
        "schema_version": 1,
        "timestamp_monotonic_ns": time.monotonic_ns(),
        "side": side,
        "cargo_class": cargo_class,
        "delivery_count": delivery_count,
        "source": "safe_zone_fence_contact",
        "constraint_axis": "y",
        "observed_pose": {
            "x_m": observed.x_m,
            "y_m": observed.y_m,
            "yaw_deg": observed.yaw_deg,
        },
        "tangent_reference_pose": {
            "x_m": reference_x,
            "y_m": reference_y,
            "yaw_deg": reference_yaw,
        },
        "suggested_position_correction_m": {
            "x": reference_x - observed.x_m,
            "y": reference_y - observed.y_m,
        },
        "applied_to_localization": False,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def observation(
    detections,
    allowed_classes: tuple[str, ...],
    safe_class: str,
) -> VisionInput:
    matches = [item for item in detections if item.class_name in allowed_classes]
    safe_matches = [item for item in detections if item.class_name == safe_class]
    safe = max(
        safe_matches,
        key=lambda item: item.bbox[2] * item.bbox[3],
        default=None,
    )
    safe_bbox = None if safe is None else safe.bbox
    target = max(
        matches,
        key=lambda item: item.bbox[2] * item.bbox[3],
        default=None,
    )
    if target is not None and safe_bbox is not None:
        outside = [
            item for item in matches
            if not bbox_center_in_safe_zone(item.bbox, safe_bbox)
        ]
        # Prefer a real candidate outside the safe zone when both an already
        # delivered object and a new object are visible. If every candidate is
        # inside, retain the largest one for the UI and let the state machine
        # produce an empty report.
        target = max(
            outside,
            key=lambda item: item.bbox[2] * item.bbox[3],
            default=target,
        )
    if target is None:
        return VisionInput(safe_found=safe is not None, safe_bbox=safe_bbox)
    x, y, width, height = target.bbox
    return VisionInput(
        target_found=True,
        target_x=max(0, min(IMAGE_WIDTH - 1, x + width // 2)),
        target_y=max(0, min(IMAGE_HEIGHT - 1, y + height // 2)),
        target_bbox=target.bbox,
        class_name=target.class_name,
        safe_found=safe is not None,
        safe_bbox=safe_bbox,
    )


class MissionPlanner:
    """Run non-visual mission updates independently from camera inference."""

    CONTINUOUS_STATES = {
        MissionState.GRABBING,
        MissionState.NAVIGATE,
        MissionState.ENTER_SAFE_ZONE,
        MissionState.COMPLETE,
        MissionState.RETURN_CENTER,
        MissionState.FAULT,
    }

    def __init__(self, mission: RescueMission, pose_path: Path,
                 stm_path: Path, command_path: Path, diagnostics_path: Path,
                 rate_hz: float) -> None:
        self.mission = mission
        self.pose_path = pose_path
        self.stm_path = stm_path
        self.command_path = command_path
        self.diagnostics_path = diagnostics_path
        self.period_s = 1.0 / rate_hz
        self.lock = threading.Lock()
        self.latest_vision = VisionInput()
        self.vision_generation = 0
        self.processed_vision_generation = 0
        self.latest_output = MissionOutput(
            MissionState.SEARCH, None, None, "等待第一帧识别结果"
        )
        self.latest_pose = PoseInput()
        self.latest_stm = Stm32Status()
        self.command_generated_s: float | None = None
        self.warning = ""
        self.error: Exception | None = None
        self.contact_event = None
        self.command_sequence = 0
        self.report_sequence = 0
        self.last_command_code: int | None = None
        self.last_command_sequence: int | None = None
        self.last_command_heading_deg: float | None = None
        self.last_command_remaining_mm: int | None = None
        self.last_command_kind = None
        self.last_command_log_s = 0.0
        self.last_remaining_mm: int | None = None
        self.remaining_changed_s = time.monotonic()
        self.last_warning_log_s = 0.0
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, name="mission-planner", daemon=True)

    def allowed_classes(self) -> tuple[str, ...]:
        with self.lock:
            return self.mission.allowed_classes

    def set_vision(self, vision: VisionInput) -> None:
        with self.lock:
            self.latest_vision = vision
            self.vision_generation += 1

    def snapshot(self):
        with self.lock:
            command_age_ms = (
                float("inf") if self.command_generated_s is None
                else max(0.0, (time.monotonic() - self.command_generated_s) * 1000.0)
            )
            metrics = {
                "pose_age_ms": self.latest_pose.age_ms,
                "pose_valid": self.latest_pose.valid,
                "command_age_ms": command_age_ms,
                "heading_deg": (
                    None if self.latest_output.command is None
                    else self.latest_output.command.heading_cdeg / 100.0
                ),
                "remaining_mm": (
                    None if self.latest_output.command is None
                    else self.latest_output.command.target_x_mm
                ),
                "last_command": self.last_command_code,
                "last_command_sequence": self.latest_stm.relay_last_sequence,
                "planner_command_sequence": self.last_command_sequence,
            }
            return self.latest_output, self.latest_pose, self.latest_stm, metrics, self.warning

    def consume_contact_event(self):
        with self.lock:
            event = self.contact_event
            self.contact_event = None
            return event

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=2.0)

    def _publish(self, output: MissionOutput) -> None:
        if output.command is not None:
            sequence = self.command_sequence
            write_command_frame(
                self.command_path,
                output.command.to_frame(sequence),
            )
            self.command_sequence = (self.command_sequence + 1) & 0xFF
            self.command_generated_s = time.monotonic()
            self.last_command_code = output.command.command
            self.last_command_sequence = sequence
            self.last_command_heading_deg = output.command.heading_cdeg / 100.0
            self.last_command_remaining_mm = output.command.target_x_mm
        elif output.report is not None:
            write_command_frame(
                self.command_path,
                output.report.to_frame(self.report_sequence),
            )
            self.report_sequence = (self.report_sequence + 1) & 0xFF

    def _write_diagnostics(self, output: MissionOutput, pose: PoseInput,
                           stm: Stm32Status, vision: VisionInput,
                           now: float) -> None:
        command_age_ms = (
            None if self.command_generated_s is None
            else max(0.0, (now - self.command_generated_s) * 1000.0)
        )
        document = {
            "schema_version": 1,
            "timestamp_monotonic_ns": time.monotonic_ns(),
            "state_machine_state": output.state.value,
            "stm_mode": stm.mode,
            "stm_fault_code": stm.fault_code,
            "stm_acknowledged_sequence": stm.acknowledged_sequence,
            "last_command": self.last_command_code,
            "last_command_sequence": stm.relay_last_sequence,
            "planner_command_sequence": self.last_command_sequence,
            "pose_x_mm": round(pose.x_m * 1000.0),
            "pose_y_mm": round(pose.y_m * 1000.0),
            "pose_yaw_deg": pose.yaw_deg,
            "pose_valid": pose.valid,
            "planner_pose_age_ms": pose.age_ms if math.isfinite(pose.age_ms) else None,
            "planner_command_age_ms": command_age_ms,
            "command_heading_deg": self.last_command_heading_deg,
            "command_remaining_mm": self.last_command_remaining_mm,
            "relay_tx_age_ms": (
                stm.relay_last_tx_age_ms
                if math.isfinite(stm.relay_last_tx_age_ms) else None
            ),
            "motors_active": stm.motors_active,
            "gripper_closed": stm.gripper_closed,
            "distance_done": stm.distance_done,
            "vision_target_found": vision.target_found,
            "vision_target_class": vision.class_name or None,
            "vision_target_in_safe_zone": target_inside_safe_zone(vision),
            "vision_safe_zone_found": vision.safe_found,
            "vision_target_bbox": None if vision.target_bbox is None else list(vision.target_bbox),
            "vision_safe_bbox": None if vision.safe_bbox is None else list(vision.safe_bbox),
            "delivery_arrival_confirmed": self.mission.delivery_arrival_confirmed,
            "fence_stop_x_mm": round(self.mission.fence_stop_point[0] * 1000.0),
            "fence_stop_y_mm": round(self.mission.fence_stop_point[1] * 1000.0),
            "nav_fence_heading_tolerance_deg": (
                self.mission.settings.nav_fence_heading_tolerance_deg
            ),
            "nav_near_fence_distance_mm": round(
                self.mission.settings.nav_near_fence_distance_m * 1000.0
            ),
            "nav_near_fence_heading_limit_deg": (
                self.mission.settings.nav_near_fence_heading_limit_deg
            ),
            "t265_delivery_extra_mm": round(
                self.mission.settings.t265_delivery_extra_m * 1000.0
            ),
            "warning": self.warning,
        }
        self.diagnostics_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.diagnostics_path.with_suffix(
            self.diagnostics_path.suffix + ".tmp"
        )
        temporary.write_text(
            json.dumps(document, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.diagnostics_path)

    def _update_progress_warning(self, output: MissionOutput,
                                 stm: Stm32Status, now: float) -> None:
        dynamic = output.state in {MissionState.NAVIGATE, MissionState.RETURN_CENTER}
        remaining = None if output.command is None else output.command.target_x_mm
        if not dynamic or not stm.motors_active or remaining is None:
            self.last_remaining_mm = remaining
            self.remaining_changed_s = now
            self.warning = ""
            return
        if self.last_remaining_mm is None or abs(remaining - self.last_remaining_mm) > 2:
            self.last_remaining_mm = remaining
            self.remaining_changed_s = now
            self.warning = ""
            return
        if now - self.remaining_changed_s >= 0.5:
            self.warning = f"警告：车辆运动但剩余距离已{now - self.remaining_changed_s:.1f}s未变化"
            if now - self.last_warning_log_s >= 1.0:
                print(self.warning)
                self.last_warning_log_s = now

    def tick(self) -> None:
        with self.lock:
            vision = self.latest_vision
            generation = self.vision_generation
            continuous = self.mission.state in self.CONTINUOUS_STATES
        if not continuous and generation == self.processed_vision_generation:
            return
        pose = load_pose(self.pose_path)
        stm = load_stm_status(self.stm_path)
        with self.lock:
            output = self.mission.step(vision, pose, stm)
            self._publish(output)
            now = time.monotonic()
            self._update_progress_warning(output, stm, now)
            self._write_diagnostics(output, pose, stm, vision, now)
            command_kind = (
                None if output.command is None
                else (output.state.value, output.command.command)
            )
            if (command_kind is not None and
                    (command_kind != self.last_command_kind or now - self.last_command_log_s >= 0.5)):
                command = output.command
                print(
                    "任务规划："
                    f"state={output.state.value} cmd={command.command} "
                    f"flags=0x{command.flags:02X} remaining={command.target_x_mm}mm "
                    f"heading={command.heading_cdeg / 100.0:.2f}°"
                )
                self.last_command_kind = command_kind
                self.last_command_log_s = now
            self.latest_output = output
            self.latest_pose = pose
            self.latest_stm = stm
            if generation != self.processed_vision_generation:
                self.processed_vision_generation = generation
            if output.contact_pose is not None:
                self.contact_event = (
                    pose, output.contact_pose,
                    self.mission.selected_class or "unknown",
                    self.mission.delivery_count,
                )

    def _run(self) -> None:
        try:
            next_tick = time.monotonic()
            while not self.stop_event.is_set():
                now = time.monotonic()
                if now >= next_tick:
                    self.tick()
                    next_tick = now + self.period_s
                self.stop_event.wait(max(0.001, min(self.period_s, next_tick - time.monotonic())))
        except Exception as error:
            self.error = error
            self.stop_event.set()


def display_size() -> tuple[int, int]:
    try:
        output = subprocess.check_output(["xrandr", "--current"], text=True, stderr=subprocess.DEVNULL)
        match = re.search(r"current\s+(\d+)\s+x\s+(\d+)", output)
        if match:
            return int(match.group(1)), int(match.group(2))
    except (OSError, subprocess.SubprocessError):
        pass
    return 1280, 1024


def fit_image(image, width: int, height: int):
    scale = min(width / image.shape[1], height / image.shape[0])
    resized = cv2.resize(image, (round(image.shape[1] * scale), round(image.shape[0] * scale)), interpolation=cv2.INTER_AREA)
    top = (height - resized.shape[0]) // 2
    left = (width - resized.shape[1]) // 2
    return cv2.copyMakeBorder(resized, top, height - resized.shape[0] - top, left, width - resized.shape[1] - left, cv2.BORDER_CONSTANT)


def draw(image, vision: VisionInput, output, pose: PoseInput, stm: Stm32Status,
         planner_metrics: dict, planner_warning: str):
    view = image.copy()
    cv2.line(view, (640, 0), (640, 1023), (110, 110, 110), 1)
    cv2.line(view, (0, 512), (1279, 512), (110, 110, 110), 1)
    if vision.target_bbox:
        x, y, w, h = vision.target_bbox
        target_blocked = target_inside_safe_zone(vision)
        target_color = (0, 165, 255) if target_blocked else (0, 255, 0)
        cv2.rectangle(view, (x, y), (x + w, y + h), target_color, 3)
        cv2.circle(view, (vision.target_x, vision.target_y), 8, target_color, 2)
    if vision.safe_bbox:
        x, y, w, h = vision.safe_bbox
        cv2.rectangle(view, (x, y), (x + w, y + h), (255, 0, 255), 3)
    lines = [
        f"state={output.state.value}  {output.message}",
        f"stm mode={stm.mode} fault={stm.fault_code} ack={stm.acknowledged_sequence} motors={int(stm.motors_active)} gripper={int(stm.gripper_closed)} done={int(stm.distance_done)}",
        f"target={vision.class_name or '-'} ({vision.target_x},{vision.target_y}) found={int(vision.target_found)} safe={int(vision.safe_found)} blocked={int(target_inside_safe_zone(vision))} claw={int(stm.claw_visible)} age={stm.age_ms:.0f}ms",
        f"pose=({pose.x_m:+.2f},{pose.y_m:+.2f}) yaw={pose.yaw_deg:.1f} valid={int(pose.valid)}",
        f"planner pose_age={planner_metrics['pose_age_ms']:.0f}ms valid={int(planner_metrics['pose_valid'])} command_age={planner_metrics['command_age_ms']:.0f}ms",
        f"command last={planner_metrics['last_command']} seq={planner_metrics['last_command_sequence']} heading={planner_metrics['heading_deg']}deg remaining={planner_metrics['remaining_mm']}mm",
        f"relay seq={stm.relay_last_sequence} age={stm.relay_last_tx_age_ms:.0f}ms tx={stm.relay_tx_frames} err={stm.relay_tx_errors}",
        planner_warning,
        "camera-down + target anywhere x confirm-frames = grab; F fullscreen; Q/Esc quit",
    ]
    for index, line in enumerate(lines):
        cv2.putText(view, line, (14, 34 + index * 30), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
    return view


def main() -> int:
    args = arguments()
    if args.vision_fps <= 0 or args.planner_fps <= 0 or args.display_fps <= 0:
        raise ValueError("vision-fps、planner-fps和display-fps必须为正数")
    if args.start_pose_tolerance_mm <= 0:
        raise ValueError("start-pose-tolerance-mm必须为正数")
    if not 0.0 < args.score_thres < 1.0 or not 0.0 < args.nms_thres < 1.0:
        raise ValueError("score-thres和nms-thres必须在0..1之间")
    session = load_json(args.session)
    if not session or session.get("side") not in {"red", "blue"}:
        raise RuntimeError(f"请先在rescue_map选择出发区和红蓝方：{args.session}")
    side = str(session["side"])
    safe_class = "safe_red" if side == "red" else "safe_blue"
    settings = MissionSettings(
        side=side,
        confirmation_frames=args.confirm_frames,
        zone_center_x_abs_m=args.zone_center_x_mm / 1000.0,
        delivery_stationary_s=args.delivery_stationary_seconds,
        delivery_stationary_tolerance_m=args.delivery_stationary_tolerance_mm / 1000.0,
        center_stop_radius_m=args.center_stop_radius_mm / 1000.0,
        nav_fence_heading_tolerance_deg=args.nav_fence_heading_tolerance_deg,
        nav_near_fence_distance_m=args.nav_near_fence_distance_m,
        nav_near_fence_heading_limit_deg=args.nav_near_fence_heading_limit_deg,
        t265_delivery_extra_m=(
            T265_DELIVERY_EXTRA_M
            if session.get("localization_mode") == "t265" else 0.0
        ),
    )
    mission = RescueMission(settings)
    config = load_config(args.config)
    localizer = GroundLocalizer.load(args.homography, (IMAGE_WIDTH, IMAGE_HEIGHT))
    traditional_detector = (
        TraditionalDetector(config, localizer)
        if args.detector == "traditional" else None
    )
    yolo_detector = None
    scaler = None
    if args.detector == "yolo":
        yolo_detector = X5YoloV8(
            args.model, load_labels(args.labels), args.score_thres, args.nms_thres,
            args.priority, args.bpu_cores,
        )
        use_vse = args.preprocess in {"auto", "vse"} and args.decoder == "jpu"
        if args.preprocess == "vse" and args.decoder != "jpu":
            raise ValueError("VSE NV12路径要求--decoder jpu")
        if use_vse:
            scale = min(yolo_detector.input_width / IMAGE_WIDTH,
                        yolo_detector.input_height / IMAGE_HEIGHT)
            content_width = max(2, int(round(IMAGE_WIDTH * scale)) // 2 * 2)
            content_height = max(2, int(round(IMAGE_HEIGHT * scale)) // 2 * 2)
            try:
                scaler = VseScaler(
                    IMAGE_WIDTH, IMAGE_HEIGHT, content_width, content_height
                )
            except Exception as error:
                if args.preprocess == "vse":
                    raise
                print(f"警告：VSE初始化失败，回退CPU预处理：{error}")
    camera = LatestFrameCamera(
        resolve_camera_device(args.device), IMAGE_WIDTH, IMAGE_HEIGHT, args.camera_fps,
        decoder=args.decoder, decode_fps=args.decode_fps,
        output_format="nv12" if scaler is not None else "bgr",
    )
    print(
        f"识别器={args.detector}，"
        f"置信度阈值={args.score_thres:.2f}，"
        f"预处理={'JPU NV12 + VSE' if scaler is not None else 'CPU BGR'}"
    )
    print("任务命令由定位串口进程独立以100 Hz刷新；视觉窗口只显示状态切换")
    team_color = 0x11 if side == "red" else 0x12
    deadline = time.monotonic() + args.startup_timeout
    startup_pose = load_pose(args.pose)
    while not startup_pose.valid:
        if time.monotonic() >= deadline:
            raise RuntimeError(
                "融合定位/串口转发器未就绪；请先运行rescue_map并确认T265为GOOD"
            )
        time.sleep(0.05)
        startup_pose = load_pose(args.pose)
    try:
        validate_start_pose(session, startup_pose, args.start_pose_tolerance_mm)
    except RuntimeError as error:
        args.diagnostics.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.diagnostics.with_suffix(args.diagnostics.suffix + ".tmp")
        temporary.write_text(
            json.dumps({
                "schema_version": 1,
                "timestamp_monotonic_ns": time.monotonic_ns(),
                "state_machine_state": "START_POSE_MISMATCH",
                "pose_x_mm": round(startup_pose.x_m * 1000.0),
                "pose_y_mm": round(startup_pose.y_m * 1000.0),
                "pose_yaw_deg": startup_pose.yaw_deg,
                "pose_valid": startup_pose.valid,
                "error": str(error),
            }, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(args.diagnostics)
        raise
    print(
        "启动位姿检查通过："
        f"({startup_pose.x_m:+.3f},{startup_pose.y_m:+.3f})m "
        f"age={startup_pose.age_ms:.1f}ms"
    )
    config_sequence = 0
    for _ in range(3):
        write_command_frame(args.command_file, config_frame(config_sequence, team_color, int(session["start_zone"])))
        config_sequence = (config_sequence + 1) & 0xFF
        time.sleep(0.04)
    contact_pose_written_for = 0
    planner = MissionPlanner(
        mission, args.pose, args.stm_status, args.command_file,
        args.diagnostics,
        args.planner_fps,
    )

    running = True
    def stop(_signal, _frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    screen_width, screen_height = display_size()
    window = "ordinary supply mission test"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    fullscreen = args.window_mode == "fullscreen"
    cv2.setWindowProperty(window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL)
    camera.start()
    started = time.perf_counter()
    next_vision = started
    next_display = next_vision
    last_id = 0
    latest_image = None
    latest_pixel_format = "bgr"
    latest_vision = VisionInput()
    planner.start()
    try:
        while running:
            if planner.error is not None:
                raise RuntimeError(f"任务规划线程退出：{planner.error}")
            contact_event = planner.consume_contact_event()
            if (contact_event is not None
                    and contact_event[3] > contact_pose_written_for):
                contact_pose, reference, cargo_class, delivery_count = contact_event
                write_contact_pose(
                    args.contact_output, contact_pose, reference, side,
                    cargo_class, delivery_count,
                )
                contact_pose_written_for = delivery_count
                print(f"安全区接触校正参考已保存：{args.contact_output}")
            error = camera.check_error()
            if error:
                raise RuntimeError(error)
            packet = camera.latest()
            if packet is None:
                time.sleep(0.002)
                continue
            now = time.perf_counter()
            if args.duration > 0 and now - started >= args.duration:
                break
            if now >= next_vision and packet.frame_id != last_id:
                next_vision = now + 1.0 / args.vision_fps
                allowed_classes = planner.allowed_classes()
                if yolo_detector is not None:
                    if packet.pixel_format == "nv12":
                        detections, _ = yolo_detector.infer_nv12(
                            packet.image, IMAGE_WIDTH, IMAGE_HEIGHT, scaler
                        )
                    else:
                        detections, _ = yolo_detector.infer(packet.image)
                else:
                    assert traditional_detector is not None
                    detection_classes = list(allowed_classes)
                    if safe_class not in detection_classes:
                        detection_classes.append(safe_class)
                    detections, _ = traditional_detector.detect(
                        packet.image, detection_classes
                    )
                latest_vision = observation(detections, allowed_classes, safe_class)
                planner.set_vision(latest_vision)
                latest_image = packet.image
                latest_pixel_format = packet.pixel_format
                last_id = packet.frame_id
            if latest_image is not None and now >= next_display:
                next_display = now + 1.0 / args.display_fps
                latest_output, planner_pose, planner_stm, planner_metrics, planner_warning = planner.snapshot()
                display_image = (
                    cv2.cvtColor(latest_image, cv2.COLOR_YUV2BGR_NV12)
                    if latest_pixel_format == "nv12" else latest_image
                )
                shown = fit_image(
                    draw(
                        display_image, latest_vision, latest_output,
                        planner_pose, planner_stm, planner_metrics, planner_warning,
                    ),
                    screen_width, screen_height,
                )
                cv2.imshow(window, shown)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q"), ord("Q")):
                    break
                if key in (ord("f"), ord("F")):
                    fullscreen = not fullscreen
                    cv2.setWindowProperty(window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL)
            time.sleep(0.0005)
    finally:
        planner.stop()
        camera.stop()
        if scaler is not None:
            scaler.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        raise SystemExit(1)
