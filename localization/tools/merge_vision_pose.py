#!/usr/bin/env python3
"""Transform robot-relative traditional-vision tracks into field coordinates."""
from __future__ import annotations

import argparse
import json
import math
import os
import pathlib
import signal
import time


def read_json(path: pathlib.Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def atomic_write(path: pathlib.Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(temporary, path)


def merge(pose: dict, vision: dict, now_ns: int,
          pose_max_age_ms: float, vision_max_age_ms: float) -> dict:
    pose_age_ms = (now_ns - int(pose["timestamp_monotonic_ns"])) / 1_000_000.0
    vision_age_ms = (now_ns - int(vision["timestamp_monotonic_ns"])) / 1_000_000.0
    valid = (
        pose_age_ms <= pose_max_age_ms
        and vision_age_ms <= vision_max_age_ms
        and pose.get("quality") != "LOST"
        and vision.get("calibrated", False)
    )
    result = {
        "schema_version": 1,
        "timestamp_monotonic_ns": now_ns,
        "valid": valid,
        "pose_age_ms": round(pose_age_ms, 3),
        "vision_age_ms": round(vision_age_ms, 3),
        "robot_pose": pose.get("pose"),
        "tracks": [],
    }
    if not valid:
        return result

    robot = pose["pose"]
    c = math.cos(float(robot["yaw_rad"]))
    s = math.sin(float(robot["yaw_rad"]))
    for source in vision.get("tracks", []):
        track = dict(source)
        position = track.get("position")
        if track.get("coordinate_system") != "ground_mm" or not position:
            continue
        # Monocular ground coordinates: +X image/right, +Y robot/forward.
        forward_m = float(position[1]) * 0.001
        left_m = -float(position[0]) * 0.001
        field_x = float(robot["x_m"]) + c * forward_m - s * left_m
        field_y = float(robot["y_m"]) + s * forward_m + c * left_m
        track["field_position_m"] = [round(field_x, 6), round(field_y, 6)]
        track["field_frame"] = "field"
        result["tracks"].append(track)
    return result


def main() -> int:
    root = pathlib.Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose", default=str(root / "localization_result.json"))
    parser.add_argument(
        "--vision",
        default="/home/sunrise/RDK_X5/traditional_rescue_vision/runtime_result.json",
    )
    parser.add_argument("--output", default=str(root / "navigation_world.json"))
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--pose-max-age-ms", type=float, default=150.0)
    parser.add_argument("--vision-max-age-ms", type=float, default=150.0)
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()

    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    pose_path = pathlib.Path(args.pose)
    vision_path = pathlib.Path(args.vision)
    output_path = pathlib.Path(args.output)
    period = 1.0 / args.rate
    while running:
        try:
            now_ns = time.monotonic_ns()
            result = merge(
                read_json(pose_path), read_json(vision_path), now_ns,
                args.pose_max_age_ms, args.vision_max_age_ms,
            )
            atomic_write(output_path, result)
        except (FileNotFoundError, json.JSONDecodeError, KeyError, ValueError) as error:
            atomic_write(output_path, {
                "schema_version": 1,
                "timestamp_monotonic_ns": time.monotonic_ns(),
                "valid": False,
                "error": str(error),
                "tracks": [],
            })
        if args.once:
            break
        time.sleep(period)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
