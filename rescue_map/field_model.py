#!/usr/bin/env python3
"""Field geometry and runtime data model for the rescue-map display."""

from __future__ import annotations

import json
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path


FIELD_SIZE_M = 3.0
FIELD_HALF_M = FIELD_SIZE_M / 2.0
START_ZONE_SIZE_M = 0.3
START_CENTER_M = 1.35
DEFAULT_CORNER_OFFSET_M = 0.15 * math.sqrt(2.0)
DEFAULT_ENCODER_FUSION_WEIGHT = 0.25
DEFAULT_T265_TRANSLATION_SCALE = 1.04
DEFAULT_T265_MAP_TIMEOUT_S = 30.0

# Heading is counter-clockwise from field +X.  The robot front points toward
# the outside corner so that reverse motion takes it into the field.
START_HEADINGS_DEG = {1: 135.0, 2: 45.0, 3: 225.0, 4: 315.0}
ZONE_SIGNS = {1: (-1.0, 1.0), 2: (1.0, 1.0), 3: (-1.0, -1.0), 4: (1.0, -1.0)}


@dataclass(frozen=True)
class Pose:
    x_m: float
    y_m: float
    yaw_deg: float
    quality: str = "WAITING"
    age_ms: float = math.inf
    tracker_confidence: int = 0
    mapper_confidence: int = 0
    t265_travel_m: float = 0.0
    t265_translation_scale: float = 1.0
    t265_translation_scale_enabled: bool = False
    t265_map_enabled: bool = False
    t265_map_imported: bool = False
    t265_map_relocalized: bool = False
    t265_map_startup_ready: bool = True
    t265_map_event_count: int = 0
    t265_map_wait_ms: float = -1.0
    t265_map_timeout: bool = False
    uart_fresh: bool = False
    wheel_gate: str = "waiting"
    encoder_fusion_weight: float = DEFAULT_ENCODER_FUSION_WEIGHT
    navigation_wheel_primary: bool = False
    odom_available: bool = False
    odom_x_m: float = 0.0
    odom_y_m: float = 0.0
    odom_yaw_deg: float = 0.0
    odom_travel_m: float = 0.0
    odom_forward_velocity_mps: float = 0.0
    odom_left_velocity_mps: float = 0.0
    odom_yaw_rate_degps: float = 0.0
    odom_yaw_source: str = "none"
    odom_updates: int = 0
    odom_update_age_ms: float = math.inf
    fused_odom_delta_m: float = -1.0
    fused_odom_yaw_delta_deg: float = 0.0


def start_center_coordinate(corner_offset_m: float) -> float:
    """Return one field-axis coordinate for a radial corner distance.

    ``corner_offset_m`` is the straight-line distance from the field corner to
    the robot reference point.  Each axis therefore uses its equal diagonal
    component rather than the full radial distance.
    """
    return FIELD_HALF_M - corner_offset_m / math.sqrt(2.0)


def initial_pose(zone: int, corner_offset_m: float = DEFAULT_CORNER_OFFSET_M) -> Pose:
    if zone not in ZONE_SIGNS:
        raise ValueError("zone must be 1..4")
    if not 0.0 < corner_offset_m < FIELD_HALF_M:
        raise ValueError("corner offset must be inside the field")
    sx, sy = ZONE_SIGNS[zone]
    coordinate = start_center_coordinate(corner_offset_m)
    return Pose(sx * coordinate, sy * coordinate, START_HEADINGS_DEG[zone])


def normalize_heading(degrees: float) -> float:
    return degrees % 360.0


def load_localization_pose(path: Path, stale_ms: int = 250) -> Pose | None:
    """Read one atomic localization snapshot; stale/partial input is invalid."""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        timestamp = int(data["timestamp_monotonic_ns"])
        pose = data["pose"]
        t265 = data.get("t265", {})
        wheel = data.get("wheel", {})
        navigation = data.get("navigation", {})
        now_ns = time.monotonic_ns()
        if timestamp > now_ns + 1_000_000_000:
            return None
        age_ms = max(0.0, (now_ns - timestamp) / 1_000_000.0)
        quality = str(data.get("quality", "LOST")).upper()
        if data.get("frame") != "field" or quality not in {"GOOD", "DEGRADED", "LOST"}:
            return None
        if age_ms > stale_ms:
            quality = "STALE"
        x_m = float(pose["x_m"])
        y_m = float(pose["y_m"])
        if "yaw_deg" in pose:
            yaw_deg = float(pose["yaw_deg"])
        else:
            yaw_deg = math.degrees(float(pose["yaw_rad"]))
        if not all(math.isfinite(value) for value in (x_m, y_m, yaw_deg)):
            return None
        t265_translation_scale = float(t265.get("translation_scale", 1.0))
        if not math.isfinite(t265_translation_scale) or t265_translation_scale <= 0.0:
            return None
        encoder_fusion_weight = float(
            wheel.get("fusion_weight", DEFAULT_ENCODER_FUSION_WEIGHT)
        )
        if not math.isfinite(encoder_fusion_weight) or not 0.0 <= encoder_fusion_weight <= 1.0:
            return None
        odom = data.get("wheel_odom", {})
        odom_available = bool(odom.get("available", False))
        odom_x_m = float(odom.get("x_m", 0.0))
        odom_y_m = float(odom.get("y_m", 0.0))
        if "yaw_deg" in odom:
            odom_yaw_deg = float(odom["yaw_deg"])
        else:
            odom_yaw_deg = math.degrees(float(odom.get("yaw_rad", 0.0)))
        odom_travel_m = float(odom.get("travel_m", 0.0))
        odom_forward_velocity_mps = float(odom.get("forward_velocity_mps", 0.0))
        odom_left_velocity_mps = float(odom.get("left_velocity_mps", 0.0))
        odom_yaw_rate_degps = math.degrees(float(odom.get("yaw_rate_radps", 0.0)))
        odom_yaw_source = str(odom.get("yaw_source", "unknown"))
        odom_updates = int(odom.get("updates", 0))
        odom_update_age_ms = float(odom.get("last_update_age_ms", math.inf))
        odom_values = (
            odom_x_m,
            odom_y_m,
            odom_yaw_deg,
            odom_travel_m,
            odom_forward_velocity_mps,
            odom_left_velocity_mps,
            odom_yaw_rate_degps,
            odom_update_age_ms,
        )
        if not all(math.isfinite(value) for value in odom_values):
            odom_available = False
            odom_x_m = odom_y_m = odom_yaw_deg = 0.0
            odom_travel_m = 0.0
            odom_forward_velocity_mps = odom_left_velocity_mps = 0.0
            odom_yaw_rate_degps = 0.0
            odom_yaw_source = "none"
            odom_updates = 0
            odom_update_age_ms = math.inf
        comparison = data.get("comparison", {})
        fused_odom_delta_m = float(comparison.get("fused_vs_wheel_odom_distance_m", -1.0))
        fused_odom_yaw_delta_deg = float(comparison.get("fused_vs_wheel_odom_yaw_deg", 0.0))
        if not all(math.isfinite(value) for value in (fused_odom_delta_m, fused_odom_yaw_delta_deg)):
            fused_odom_delta_m = -1.0
            fused_odom_yaw_delta_deg = 0.0
        t265_map = data.get("t265_map", {})
        if not isinstance(t265_map, dict):
            t265_map = {}
        t265_map_wait_ms = float(t265_map.get("relocalization_wait_ms", -1.0))
        if not math.isfinite(t265_map_wait_ms):
            t265_map_wait_ms = -1.0
        if odom_available and fused_odom_delta_m < 0.0:
            fused_odom_delta_m = math.hypot(x_m - odom_x_m, y_m - odom_y_m)
            fused_odom_yaw_delta_deg = (yaw_deg - odom_yaw_deg + 180.0) % 360.0 - 180.0
        return Pose(
            x_m=x_m,
            y_m=y_m,
            yaw_deg=normalize_heading(yaw_deg),
            quality=quality,
            age_ms=age_ms,
            tracker_confidence=int(t265.get("tracker_confidence", 0)),
            mapper_confidence=int(t265.get("mapper_confidence", 0)),
            t265_travel_m=float(t265.get("travel_from_start_m", 0.0)),
            t265_translation_scale=t265_translation_scale,
            t265_translation_scale_enabled=bool(
                t265.get("translation_scale_enabled", False)
            ),
            t265_map_enabled=bool(t265_map.get("enabled", False)),
            t265_map_imported=bool(t265_map.get("imported", False)),
            t265_map_relocalized=bool(t265_map.get("relocalized", False)),
            t265_map_startup_ready=bool(t265_map.get("startup_ready", True)),
            t265_map_event_count=int(t265_map.get("event_count", 0)),
            t265_map_wait_ms=t265_map_wait_ms,
            t265_map_timeout=bool(t265_map.get("timeout", False)),
            uart_fresh=bool(wheel.get("uart_fresh", False)),
            wheel_gate=str(wheel.get("gate", "unknown")),
            encoder_fusion_weight=encoder_fusion_weight,
            navigation_wheel_primary=bool(navigation.get("wheel_primary", False)),
            odom_available=odom_available,
            odom_x_m=odom_x_m,
            odom_y_m=odom_y_m,
            odom_yaw_deg=normalize_heading(odom_yaw_deg),
            odom_travel_m=odom_travel_m,
            odom_forward_velocity_mps=odom_forward_velocity_mps,
            odom_left_velocity_mps=odom_left_velocity_mps,
            odom_yaw_rate_degps=odom_yaw_rate_degps,
            odom_yaw_source=odom_yaw_source,
            odom_updates=odom_updates,
            odom_update_age_ms=odom_update_age_ms,
            fused_odom_delta_m=fused_odom_delta_m,
            fused_odom_yaw_delta_deg=fused_odom_yaw_delta_deg,
        )
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError):
        return None


class Trajectory:
    def __init__(self, minimum_step_m: float = 0.003, maximum_step_m: float = 0.35) -> None:
        self.minimum_step_m = minimum_step_m
        self.maximum_step_m = maximum_step_m
        self.points: list[tuple[float, float]] = []
        self.distance_m = 0.0
        self.consecutive_jump_rejections = 0

    def reset(self) -> None:
        self.points.clear()
        self.distance_m = 0.0
        self.consecutive_jump_rejections = 0

    def seed(self, x_m: float, y_m: float) -> None:
        """Start distance accumulation at a known field position."""
        if not math.isfinite(x_m) or not math.isfinite(y_m):
            raise ValueError("trajectory seed must be finite")
        self.points = [(x_m, y_m)]
        self.distance_m = 0.0
        self.consecutive_jump_rejections = 0

    def update(self, pose: Pose) -> bool:
        if pose.quality not in {"GOOD", "DEGRADED"}:
            return False
        point = (pose.x_m, pose.y_m)
        if not self.points:
            self.points.append(point)
            return True
        step = math.hypot(point[0] - self.points[-1][0], point[1] - self.points[-1][1])
        if step < self.minimum_step_m:
            return False
        if step > self.maximum_step_m:
            self.consecutive_jump_rejections += 1
            # Do not draw or count a discontinuity.  Re-anchor after repeated
            # valid samples so one early mismatch cannot freeze the trail.
            if self.consecutive_jump_rejections >= 5:
                self.points = [point]
                self.consecutive_jump_rejections = 0
            return False
        self.consecutive_jump_rejections = 0
        self.points.append(point)
        self.distance_m += step
        if len(self.points) > 12000:
            self.points = self.points[-10000:]
        return True


def write_session(
    path: Path,
    zone: int,
    side: str,
    corner_offset_m: float,
    localization_mode: str = "fusion",
    encoder_fusion_weight: float = DEFAULT_ENCODER_FUSION_WEIGHT,
    t265_map_enabled: bool = False,
    t265_map_path: Path | None = None,
    t265_translation_scale_enabled: bool = False,
    t265_translation_scale: float = DEFAULT_T265_TRANSLATION_SCALE,
) -> None:
    if side not in {"red", "blue"}:
        raise ValueError("side must be red or blue")
    if localization_mode not in {"fusion", "t265"}:
        raise ValueError("localization mode must be fusion or t265")
    if not 0.0 <= encoder_fusion_weight <= 1.0:
        raise ValueError("encoder fusion weight must be in 0..1")
    if not math.isfinite(t265_translation_scale) or not 0.0 < t265_translation_scale <= 2.0:
        raise ValueError("T265 translation scale must be in (0,2]")
    pose = initial_pose(zone, corner_offset_m)
    data = {
        "schema_version": 3,
        "start_zone": zone,
        "side": side,
        "corner_offset_m": corner_offset_m,
        "localization_mode": localization_mode,
        "encoder_fusion_weight": encoder_fusion_weight,
        "t265_map_enabled": bool(t265_map_enabled),
        "t265_map_path": str(t265_map_path) if t265_map_path is not None else "",
        "t265_translation_scale_enabled": bool(t265_translation_scale_enabled),
        "t265_translation_scale": t265_translation_scale,
        "initial_pose": {"x_m": pose.x_m, "y_m": pose.y_m, "yaw_deg": pose.yaw_deg},
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def write_localization_config(
    template_path: Path,
    output_path: Path,
    zone: int,
    corner_offset_m: float,
    encoder_fusion_weight: float | None = None,
    t265_translation_scale_enabled: bool | None = None,
    t265_translation_scale: float | None = None,
) -> None:
    """Generate the localization config matching the selected start pose."""
    start_center_m = start_center_coordinate(corner_offset_m)
    replacements = {"start_zone": str(zone), "start_center_m": f"{start_center_m:.6f}"}
    if encoder_fusion_weight is not None:
        if not 0.0 <= encoder_fusion_weight <= 1.0:
            raise ValueError("encoder fusion weight must be in 0..1")
        replacements["encoder_fusion_weight"] = f"{encoder_fusion_weight:.3f}"
    if t265_translation_scale_enabled is not None:
        replacements["t265_translation_scale_enabled"] = (
            "true" if t265_translation_scale_enabled else "false"
        )
    if t265_translation_scale is not None:
        if not math.isfinite(t265_translation_scale) or not 0.0 < t265_translation_scale <= 2.0:
            raise ValueError("T265 translation scale must be in (0,2]")
        replacements["t265_translation_scale"] = f"{t265_translation_scale:.3f}"
    found: set[str] = set()
    lines: list[str] = []
    for original in template_path.read_text(encoding="utf-8").splitlines():
        content = original.split("#", 1)[0]
        if "=" in content:
            key = content.split("=", 1)[0].strip()
            if key in replacements:
                comment = " #" + original.split("#", 1)[1] if "#" in original else ""
                original = f"{key} = {replacements[key]}{comment}"
                found.add(key)
        lines.append(original)
    missing = replacements.keys() - found
    if missing:
        raise ValueError(f"localization template missing keys: {sorted(missing)}")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_suffix(output_path.suffix + ".tmp")
    temporary.write_text("\n".join(lines) + "\n", encoding="utf-8")
    os.replace(temporary, output_path)
