#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from field_model import (
    DEFAULT_CORNER_OFFSET_M,
    Pose,
    Trajectory,
    initial_pose,
    load_localization_pose,
    write_localization_config,
    write_session,
)


def near(a, b, tolerance=1e-9):
    return abs(a - b) <= tolerance


def main():
    expected = {
        1: (-1.35, 1.35, 135.0),
        2: (1.35, 1.35, 45.0),
        3: (-1.35, -1.35, 225.0),
        4: (1.35, -1.35, 315.0),
    }
    for zone, values in expected.items():
        pose = initial_pose(zone)
        assert near(pose.x_m, values[0]) and near(pose.y_m, values[1])
        assert near(pose.yaw_deg, values[2])

    trajectory = Trajectory(minimum_step_m=0.0, maximum_step_m=0.35)
    trajectory.seed(0.0, 0.0)
    assert trajectory.update(Pose(0.3, 0.0, 0.0, quality="DEGRADED"))
    assert near(trajectory.distance_m, 0.3)
    assert not trajectory.update(Pose(1.0, 0.0, 0.0, quality="GOOD"))
    assert not trajectory.update(Pose(0.31, 0.0, 0.0, quality="LOST"))

    # Repeated discontinuities do not add distance, but eventually re-anchor
    # so later normal samples can continue the trail instead of freezing it.
    recovery = Trajectory(minimum_step_m=0.0, maximum_step_m=0.35)
    recovery.seed(0.0, 0.0)
    for index in range(5):
        assert not recovery.update(Pose(1.0 + index * 0.01, 1.0, 0.0, quality="GOOD"))
    assert near(recovery.distance_m, 0.0)
    assert recovery.update(Pose(1.05, 1.0, 0.0, quality="GOOD"))
    assert near(recovery.distance_m, 0.01)

    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        snapshot = root / "pose.json"
        snapshot.write_text(
            json.dumps(
                {
                    "timestamp_monotonic_ns": time.monotonic_ns(),
                    "frame": "field",
                    "quality": "GOOD",
                    "pose": {"x_m": 0.2, "y_m": -0.4, "yaw_rad": math.pi / 2},
                    "t265": {"tracker_confidence": 3, "mapper_confidence": 2},
                    "wheel": {"uart_fresh": True, "gate": "accepted"},
                    "wheel_odom": {
                        "available": True,
                        "x_m": 0.21,
                        "y_m": -0.39,
                        "yaw_deg": 92.0,
                        "travel_m": 0.24,
                        "forward_velocity_mps": 0.12,
                        "left_velocity_mps": -0.03,
                        "yaw_rate_radps": 0.05,
                        "updates": 24,
                        "last_update_age_ms": 8.0,
                    },
                }
            ),
            encoding="utf-8",
        )
        loaded = load_localization_pose(snapshot)
        assert loaded is not None and near(loaded.yaw_deg, 90.0)
        assert loaded.uart_fresh and loaded.wheel_gate == "accepted"
        assert loaded.odom_available
        assert near(loaded.odom_x_m, 0.21) and near(loaded.odom_y_m, -0.39)
        assert near(loaded.odom_yaw_deg, 92.0)
        assert near(loaded.odom_travel_m, 0.24) and loaded.odom_updates == 24

        # yaw_deg must work without yaw_rad; non-finite coordinates are rejected.
        data = json.loads(snapshot.read_text(encoding="utf-8"))
        data["pose"] = {"x_m": 0.1, "y_m": 0.2, "yaw_deg": 315.0}
        snapshot.write_text(json.dumps(data), encoding="utf-8")
        assert near(load_localization_pose(snapshot).yaw_deg, 315.0)
        data["pose"]["x_m"] = float("nan")
        snapshot.write_text(json.dumps(data), encoding="utf-8")
        assert load_localization_pose(snapshot) is None

        session = root / "session.json"
        write_session(session, 3, "blue", DEFAULT_CORNER_OFFSET_M, "t265")
        saved = json.loads(session.read_text(encoding="utf-8"))
        expected_coordinate = -1.35
        assert near(saved["initial_pose"]["x_m"], expected_coordinate)
        assert near(saved["initial_pose"]["y_m"], expected_coordinate)
        assert saved["initial_pose"]["yaw_deg"] == 225.0
        assert saved["localization_mode"] == "t265"
        try:
            write_session(session, 1, "green", 0.30)
            raise AssertionError("invalid side accepted")
        except ValueError:
            pass
        try:
            write_session(session, 1, "red", 0.30, "invalid")
            raise AssertionError("invalid localization mode accepted")
        except ValueError:
            pass

        template = root / "template.conf"
        output = root / "runtime.conf"
        template.write_text("start_zone = 4\nstart_center_m = 1.35\n", encoding="utf-8")
        write_localization_config(template, output, 2, DEFAULT_CORNER_OFFSET_M)
        text = output.read_text(encoding="utf-8")
        expected_center = 1.35
        assert "start_zone = 2" in text and f"start_center_m = {expected_center:.6f}" in text

    print("rescue_map field model PASS")


if __name__ == "__main__":
    main()
