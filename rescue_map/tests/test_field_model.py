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
        1: (-1.2, 1.2, 135.0),
        2: (1.2, 1.2, 45.0),
        3: (-1.2, -1.2, 225.0),
        4: (1.2, -1.2, 315.0),
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
                }
            ),
            encoding="utf-8",
        )
        loaded = load_localization_pose(snapshot)
        assert loaded is not None and near(loaded.yaw_deg, 90.0)
        assert loaded.uart_fresh and loaded.wheel_gate == "accepted"

        # yaw_deg must work without yaw_rad; non-finite coordinates are rejected.
        data = json.loads(snapshot.read_text(encoding="utf-8"))
        data["pose"] = {"x_m": 0.1, "y_m": 0.2, "yaw_deg": 315.0}
        snapshot.write_text(json.dumps(data), encoding="utf-8")
        assert near(load_localization_pose(snapshot).yaw_deg, 315.0)
        data["pose"]["x_m"] = float("nan")
        snapshot.write_text(json.dumps(data), encoding="utf-8")
        assert load_localization_pose(snapshot) is None

        session = root / "session.json"
        write_session(session, 3, "blue", 0.30)
        saved = json.loads(session.read_text(encoding="utf-8"))
        assert saved["initial_pose"] == {"x_m": -1.2, "y_m": -1.2, "yaw_deg": 225.0}
        try:
            write_session(session, 1, "green", 0.30)
            raise AssertionError("invalid side accepted")
        except ValueError:
            pass

        template = root / "template.conf"
        output = root / "runtime.conf"
        template.write_text("start_zone = 4\nstart_center_m = 1.35\n", encoding="utf-8")
        write_localization_config(template, output, 2, 0.30)
        text = output.read_text(encoding="utf-8")
        assert "start_zone = 2" in text and "start_center_m = 1.200000" in text

    print("rescue_map field model PASS")


if __name__ == "__main__":
    main()
