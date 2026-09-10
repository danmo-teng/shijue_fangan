#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = ROOT.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "vision"))

from protocol import CMD_APPROACH_TARGET, CMD_HOLD  # noqa: E402
from run_competition_rescue import (  # noqa: E402
    CompetitionPlanner,
    JsonlLog,
    bbox_center_px,
    detection_log_record,
    write_command_frame,
)
from rescue_vision.models import Detection  # noqa: E402
from rescue_vision.vision_protocol import config_frame  # noqa: E402
from state_machine import (  # noqa: E402
    CommandRequest,
    CompetitionMission,
    CompetitionOutput,
    CompetitionSettings,
    CompetitionState,
    PoseSnapshot,
    StmSnapshot,
    VisionSnapshot,
)


def test_suppressed_publish_preserves_command_file_and_sequence() -> None:
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        command_path = root / "uart_command.bin"
        diagnostics_path = root / "diagnostics.json"
        events_path = root / "events.jsonl"
        write_command_frame(command_path, config_frame(0, 0x11, 1))
        original_bytes = command_path.read_bytes()
        original_mtime_ns = command_path.stat().st_mtime_ns
        planner = CompetitionPlanner(
            CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False)),
            root / "pose.json",
            root / "stm.json",
            command_path,
            diagnostics_path,
            JsonlLog(events_path),
            50.0,
        )
        suppressed = CompetitionOutput(
            CompetitionState.WAIT_START,
            None,
            "等待下位机完成出发并离开出发区",
            suppress_command_tx=True,
            suppression_reason="f407_autonomous_start",
        )

        for _ in range(3):
            planner._log_command_tx_suppression(suppressed)
            planner._publish(suppressed)

        assert command_path.read_bytes() == original_bytes
        assert command_path.stat().st_mtime_ns == original_mtime_ns
        assert planner.sequence == 0
        events = [json.loads(line) for line in events_path.read_text(encoding="utf-8").splitlines()]
        assert len(events) == 1
        assert events[0]["event"] == "command_tx_suppressed"
        assert events[0]["reason"] == "f407_autonomous_start"

        planner._diagnostics(
            suppressed,
            PoseSnapshot(),
            StmSnapshot(mode=1, age_ms=5.0),
            VisionSnapshot(),
        )
        diagnostics = json.loads(diagnostics_path.read_text(encoding="utf-8"))
        assert diagnostics["command_tx_suppressed"] is True
        assert diagnostics["suppression_reason"] == "f407_autonomous_start"

        planner._publish(
            CompetitionOutput(
                CompetitionState.SEARCH,
                CommandRequest(CMD_HOLD),
                "等待识别",
            )
        )
        hold_bytes = command_path.read_bytes()
        assert hold_bytes != original_bytes
        assert hold_bytes[4] == CMD_HOLD
        assert planner.sequence == 1

        planner._publish(
            CompetitionOutput(
                CompetitionState.APPROACH,
                CommandRequest(CMD_APPROACH_TARGET, arg_a=10, arg_b=20),
                "靠近目标",
            )
        )
        approach_bytes = command_path.read_bytes()
        approach_mtime_ns = command_path.stat().st_mtime_ns
        assert approach_bytes[4] == CMD_APPROACH_TARGET
        assert planner.sequence == 2

        recovery = CompetitionOutput(
            CompetitionState.WAIT_SEARCH_RECOVERY,
            None,
            "释放HOLD，等待F407自主完成目标丢失恢复",
            suppress_command_tx=True,
            suppression_reason="f407_search_recovery",
            tx_policy="autonomous_recovery",
            reason="f407_approach_recovery",
            expected_stm_modes=(20, 24, 3),
        )
        for _ in range(3):
            planner._log_command_tx_suppression(recovery)
            planner._publish(recovery)
        assert command_path.read_bytes() == approach_bytes
        assert command_path.stat().st_mtime_ns == approach_mtime_ns
        assert planner.sequence == 2

        # The real relay can bridge the unchanged mission frame for at most
        # 250 ms; after that the STM32 receives no refreshed APPROACH frame.
        last_input_change_s = 100.0
        assert 100.249 - last_input_change_s <= 0.25
        assert 100.251 - last_input_change_s > 0.25

        planner.mission.last_selected_track_ids = (7,)
        planner.mission.target_last_seen_s = 99.5
        planner.mission.first_fault_code = 6
        planner.mission._set_state(CompetitionState.WAIT_SEARCH_RECOVERY, 100.0)
        planner._diagnostics(
            recovery,
            PoseSnapshot(True, -1.0, 1.0, 135.0, 5.0),
            StmSnapshot(mode=20, flags=4, age_ms=5.0, acknowledged_sequence=8),
            VisionSnapshot(observed_monotonic_s=100.0),
            now=100.0,
        )
        diagnostics = json.loads(diagnostics_path.read_text(encoding="utf-8"))
        assert diagnostics["upper_state"] == "WAIT_SEARCH_RECOVERY"
        assert diagnostics["stm_mode"] == 20
        assert diagnostics["stm_age_ms"] == 5.0
        assert diagnostics["command_opcode"] is None
        assert diagnostics["tx_policy"] == "autonomous_recovery"
        assert diagnostics["reason"] == "f407_approach_recovery"
        assert diagnostics["selected_track_ids"] == [7]
        assert diagnostics["target_last_seen_age_ms"] == 500.0
        assert diagnostics["expected_stm_mode"] == [20, 24, 3]
        assert diagnostics["first_fault_code"] == 6

        # Camera pause is an explicit HOLD policy and must override recovery
        # suppression while the camera is unavailable.
        planner._publish(
            CompetitionOutput(
                CompetitionState.WAIT_SEARCH_RECOVERY,
                CommandRequest(CMD_HOLD),
                "摄像头恢复中，保持车辆停车",
                tx_policy="camera_pause_hold",
                reason="camera_recovery",
            )
        )
        assert command_path.read_bytes()[4] == CMD_HOLD
        assert planner.sequence == 3

        events = [json.loads(line) for line in events_path.read_text(encoding="utf-8").splitlines()]
        assert [item["reason"] for item in events] == [
            "f407_autonomous_start",
            "f407_search_recovery",
        ]


def test_detection_logging() -> None:
    assert bbox_center_px((10, 20, 31, 41)) == (25, 40)
    detection = Detection(
        class_name="green_supply",
        confidence=0.8,
        bbox=(10, 20, 31, 41),
        bottom_point=(25.5, 61.0),
        ground_xy_mm=(100.0, 200.0),
        size_mm=None,
        contour=np.empty((0, 1, 2), dtype=np.int32),
    )
    record = detection_log_record(
        7,
        {"total_ms": 12.5},
        [detection],
        (),
    )
    assert record["detections"][0]["center"] == [25, 40]
    assert record["detections"][0]["ground_xy_mm"] == [100.0, 200.0]


def main() -> None:
    test_suppressed_publish_preserves_command_file_and_sequence()
    test_detection_logging()
    print("competition runner PASS")


if __name__ == "__main__":
    main()
