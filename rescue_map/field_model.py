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
    uart_fresh: bool = False
    wheel_gate: str = "waiting"


def start_center_coordinate(corner_offset_m: float) -> float:
    """Return one field-axis coordinate for a radial corner distance.

    ``corner_offset_m`` is the straight-line distance from the field corner to
    the robot reference point.  Each axis therefore uses its equal diagonal
    component rather than the full radial distance.
    """
    return FIELD_HALF_M - corner_offset_m / math.sqrt(2.0)


def initial_pose(zone: int, corner_offset_m: float = 0.30) -> Pose:
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
        return Pose(
            x_m=x_m,
            y_m=y_m,
            yaw_deg=normalize_heading(yaw_deg),
            quality=quality,
            age_ms=age_ms,
            tracker_confidence=int(t265.get("tracker_confidence", 0)),
            mapper_confidence=int(t265.get("mapper_confidence", 0)),
            t265_travel_m=float(t265.get("travel_from_start_m", 0.0)),
            uart_fresh=bool(wheel.get("uart_fresh", False)),
            wheel_gate=str(wheel.get("gate", "unknown")),
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
) -> None:
    if side not in {"red", "blue"}:
        raise ValueError("side must be red or blue")
    if localization_mode not in {"fusion", "t265"}:
        raise ValueError("localization mode must be fusion or t265")
    pose = initial_pose(zone, corner_offset_m)
    data = {
        "schema_version": 1,
        "start_zone": zone,
        "side": side,
        "corner_offset_m": corner_offset_m,
        "localization_mode": localization_mode,
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
) -> None:
    """Generate the localization config matching the selected start pose."""
    start_center_m = start_center_coordinate(corner_offset_m)
    replacements = {"start_zone": str(zone), "start_center_m": f"{start_center_m:.6f}"}
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
