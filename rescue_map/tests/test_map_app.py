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
        corner_offset_mm=150.0 * math.sqrt(2.0),
        encoder_weight=0.25,
        localization_mode="fusion",
        localization_json=snapshot,
        localization_log=None,
        launch_localization=False,
        launch_vision=False,
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
        expected_coordinate = 1.35
        assert app.trajectory.points == [(expected_coordinate, expected_coordinate)]
        app.localization_log.write_text("old diagnostic log\n", encoding="utf-8")
        app.archive_previous_runtime()
        archived_logs = list((map_app.RUNTIME / "history").glob("*/localization_debug.csv"))
        assert archived_logs and archived_logs[-1].read_text(encoding="utf-8") == "old diagnostic log\n"
        assert "--uart" in app.localization_command()
        assert "--csv" in app.localization_command()
        csv_index = app.localization_command().index("--csv")
        assert app.localization_command()[csv_index + 1].endswith("localization_debug.csv")
        tx_index = app.localization_command().index("--tx-rate")
        assert app.localization_command()[tx_index + 1] == "0.0"
        assert "run_mission_test.sh" in app.vision_command()[0]
        assert math.isclose(app.encoder_weight, 0.25)
        app.adjust_encoder_weight(0.10)
        assert math.isclose(app.encoder_weight, 0.35)
        app.localization_mode = "t265"
        assert "--uart" in app.localization_command()
        assert "--command-file" in app.localization_command()
        assert "--ignore-encoders" in app.localization_command()
        assert "run_mission_test.sh" in app.vision_command()[0]
        app.adjust_corner_offset(50.0)
        assert math.isclose(app.corner_offset_m, 0.15 * math.sqrt(2.0) + 0.05)

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
            app.options.launch_vision = True
            app.start_session()
            assert len(launched) == 2 and "--uart" in launched[0]
            assert "--ignore-encoders" in launched[0]
            assert "run_mission_test.sh" in launched[1][0]
            assert app.message == "T265定位进程已启动"
        finally:
            app.stop_session_processes()
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
                    "wheel": {
                        "uart_fresh": True,
                        "gate": "startup_obstacle",
                        "fusion_weight": 0.35,
                    },
                    "navigation": {"wheel_primary": False},
                    "wheel_odom": {
                        "available": True,
                        "x_m": 1.18,
                        "y_m": 1.20,
                        "yaw_deg": 44.5,
                        "travel_m": 0.18,
                        "forward_velocity_mps": 0.10,
                        "left_velocity_mps": 0.01,
                        "yaw_rate_radps": 0.0,
                        "yaw_source": "t265_gyro",
                        "updates": 18,
                        "last_update_age_ms": 6.0,
                    },
                }
            ),
            encoding="utf-8",
        )
        app.update_pose()
        assert app.pose.quality == "GOOD"
        assert app.pose.uart_fresh
        assert app.pose.odom_available and math.isclose(app.pose.odom_x_m, 1.18)
        assert app.pose.odom_yaw_source == "t265_gyro"
        assert math.isclose(app.pose.encoder_fusion_weight, 0.35)
        assert not app.pose.navigation_wheel_primary
        assert app.trajectory.distance_m > 0.01
        assert app.odometry_trajectory.points[-1] == (1.18, 1.20)
        frame = app.render()
        assert frame.shape == (1024, 1280, 3)

    print("rescue_map app integration PASS")


if __name__ == "__main__":
    main()
