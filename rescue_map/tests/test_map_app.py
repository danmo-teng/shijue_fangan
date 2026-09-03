#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import map_app


def options(snapshot: Path) -> argparse.Namespace:
    return argparse.Namespace(
        zone=2,
        side="blue",
        corner_offset_mm=300.0,
        localization_mode="fusion",
        localization_json=snapshot,
        launch_localization=False,
        uart="/dev/ttyS1",
        baud=115200,
        tx_rate=0.0,
        fullscreen=False,
        demo=False,
        screenshot=None,
    )


def main() -> None:
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        map_app.RUNTIME = root / "runtime"
        snapshot = root / "pose.json"
        app = map_app.RescueMapApp(options(snapshot))
        app.start_session()
        expected_coordinate = 1.5 - 0.30 / math.sqrt(2.0)
        assert app.trajectory.points == [(expected_coordinate, expected_coordinate)]
        assert "--uart" in app.localization_command()
        app.localization_mode = "t265"
        assert "--uart" not in app.localization_command()
        assert "--command-file" not in app.localization_command()
        app.adjust_corner_offset(50.0)
        assert math.isclose(app.corner_offset_m, 0.35)

        launched: list[list[str]] = []

        class FakeProcess:
            returncode = None

            def poll(self):
                return self.returncode

            def send_signal(self, _signal):
                self.returncode = 0

            def wait(self, timeout=None):
                return self.returncode

        original_popen = map_app.subprocess.Popen
        map_app.subprocess.Popen = lambda command, cwd: (launched.append(command), FakeProcess())[1]
        try:
            app.options.launch_localization = True
            app.start_session()
            assert launched and "--uart" not in launched[0]
            assert app.message == "T265定位进程已启动"
        finally:
            app.stop_localization()
            map_app.subprocess.Popen = original_popen
        app.started_monotonic -= 0.4
        app.update_pose()
        assert app.pose.quality == "NO_DATA"

        snapshot.write_text(
            json.dumps(
                {
                    "timestamp_monotonic_ns": time.monotonic_ns(),
                    "frame": "field",
                    "quality": "GOOD",
                    "pose": {"x_m": 1.19, "y_m": 1.19, "yaw_deg": 45.0},
                    "t265": {
                        "tracker_confidence": 3,
                        "mapper_confidence": 3,
                        "travel_from_start_m": 0.014,
                    },
                    "wheel": {"uart_fresh": True, "gate": "startup_obstacle"},
                }
            ),
            encoding="utf-8",
        )
        app.update_pose()
        assert app.pose.quality == "GOOD"
        assert app.pose.uart_fresh
        assert app.trajectory.distance_m > 0.01
        frame = app.render()
        assert frame.shape == (1024, 1280, 3)

    print("rescue_map app integration PASS")


if __name__ == "__main__":
    main()
