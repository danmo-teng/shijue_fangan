#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
import tempfile
import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = ROOT.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "vision"))

import run_competition_rescue as runner_module  # noqa: E402
from protocol import CMD_ABORT, CMD_APPROACH_TARGET, CMD_DISPERSE_PILE, CMD_HOLD  # noqa: E402
from run_competition_rescue import (  # noqa: E402
    CompetitionPlanner,
    JsonlLog,
    bbox_center_px,
    detection_log_record,
    load_stm,
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

        planner.mission.state = CompetitionState.DISPERSE
        planner.mission.disperse_command = CommandRequest(CMD_DISPERSE_PILE, 9)
        planner.mission.disperse_started_s = 100.0
        planner.mission.disperse_initial_ack = 8
        paused_output = planner._output_for_cycle(
            VisionSnapshot(observed_monotonic_s=0.0),
            PoseSnapshot(True, -1.0, 1.0, 135.0, 5.0),
            StmSnapshot(mode=3, flags=0, age_ms=5.0, acknowledged_sequence=8),
            True,
            100.1,
        )
        assert paused_output.state == CompetitionState.DISPERSE
        assert paused_output.command and paused_output.command.opcode == CMD_DISPERSE_PILE
        assert paused_output.tx_policy == "disperse_command"
        planner.mission.state = CompetitionState.FAULT
        terminal_output = planner._output_for_cycle(
            VisionSnapshot(),
            PoseSnapshot(),
            StmSnapshot(),
            True,
            100.2,
        )
        assert terminal_output.command and terminal_output.command.opcode == CMD_ABORT


def test_relay_snapshot_parsing_and_abort_evidence() -> None:
    with tempfile.TemporaryDirectory() as directory:
        status_path = Path(directory) / "stm.json"
        status_path.write_text(
            json.dumps({
                "timestamp_monotonic_ns": time.monotonic_ns(),
                "mode": 3,
                "flags": 0,
                "fault_code": 0,
                "acknowledged_sequence": 19,
                "relay": {
                    "tx_frames": 40,
                    "tx_errors": 0,
                    "mission_tx_frames": 21,
                    "last_mission_command": 7,
                    "last_mission_sequence": 19,
                    "last_mission_payload": [7, 9, 0, 0, 0, 0, 0, 0],
                    "last_mission_tx_monotonic_ns": time.monotonic_ns(),
                    "last_mission_tx_age_ms": 2.0,
                },
            }),
            encoding="utf-8",
        )
        status = load_stm(status_path)
        assert status.relay_tx_frames == 40
        assert status.relay_tx_errors == 0
        assert status.relay_last_sequence is None
        assert status.relay_mission_tx_frames == 21
        assert status.relay_last_mission_command == CMD_ABORT
        assert status.relay_last_mission_sequence == 19
        assert status.relay_last_mission_payload == (7, 9, 0, 0, 0, 0, 0, 0)
        assert status.relay_last_mission_tx_age_ms == 2.0


def test_planner_stop_sends_bounded_abort_and_accepts_relay_ack() -> None:
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        command_path = root / "uart_command.bin"
        status_path = root / "stm.json"
        events_path = root / "events.jsonl"
        write_command_frame(command_path, config_frame(0, 0x11, 1))
        status_path.write_text(
            json.dumps({
                "timestamp_monotonic_ns": time.monotonic_ns(),
                "mode": 20,
                "flags": 4,
                "fault_code": 0,
                "acknowledged_sequence": 12,
                "relay": {"mission_tx_frames": 10},
            }),
            encoding="utf-8",
        )
        planner = CompetitionPlanner(
            CompetitionMission(CompetitionSettings(side="red")),
            root / "pose.json",
            status_path,
            command_path,
            root / "diagnostics.json",
            JsonlLog(events_path),
            50.0,
        )

        def simulated_relay_ack() -> None:
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                try:
                    packet = command_path.read_bytes()
                except OSError:
                    packet = b""
                if len(packet) == 15 and packet[4] == CMD_ABORT:
                    status_path.write_text(
                        json.dumps({
                            "timestamp_monotonic_ns": time.monotonic_ns(),
                            "mode": 3,
                            "flags": 0,
                            "fault_code": 0,
                            "acknowledged_sequence": packet[3],
                            "relay": {
                                "mission_tx_frames": 11,
                                "last_mission_command": packet[4],
                                "last_mission_sequence": packet[3],
                                "last_mission_payload": list(packet[4:12]),
                                "last_mission_tx_monotonic_ns": time.monotonic_ns(),
                                "last_mission_tx_age_ms": 0.0,
                            },
                        }),
                        encoding="utf-8",
                    )
                    return
                time.sleep(0.005)

        relay = threading.Thread(target=simulated_relay_ack)
        relay.start()
        planner.stop("simulated_user_exit")
        relay.join(timeout=1.0)
        assert planner.termination_result["command_file_written"] is True
        assert planner.termination_result["transmitted"] is True
        assert planner.termination_result["confirmed"] is True
        assert planner.termination_result["confirmation_status"] == "confirmed"
        assert planner.termination_result["last_mission_command"] == CMD_ABORT
        assert planner.termination_result["timeout"] is False
        assert command_path.read_bytes()[4] == CMD_ABORT
        events = [json.loads(line) for line in events_path.read_text(encoding="utf-8").splitlines()]
        result_events = [item for item in events if item["event"] == "termination_abort_result"]
        assert len(result_events) == 1
        assert result_events[0]["confirmed"] is True


def test_planner_stop_records_unconfirmed_abort_timeout() -> None:
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        command_path = root / "uart_command.bin"
        events_path = root / "events.jsonl"
        write_command_frame(command_path, config_frame(0, 0x11, 1))
        planner = CompetitionPlanner(
            CompetitionMission(CompetitionSettings(side="red")),
            root / "pose.json",
            root / "stm.json",
            command_path,
            root / "diagnostics.json",
            JsonlLog(events_path),
            50.0,
        )
        original_timeout = runner_module.TERMINATION_ABORT_TIMEOUT_S
        runner_module.TERMINATION_ABORT_TIMEOUT_S = 0.02
        try:
            planner.stop("simulated_unconfirmed_exit")
        finally:
            runner_module.TERMINATION_ABORT_TIMEOUT_S = original_timeout
        assert planner.termination_result["command_file_written"] is True
        assert planner.termination_result["transmitted"] is False
        assert planner.termination_result["confirmed"] is False
        assert planner.termination_result["confirmation_status"] == "unconfirmed"
        assert planner.termination_result["timeout"] is True


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
    test_relay_snapshot_parsing_and_abort_evidence()
    test_planner_stop_sends_bounded_abort_and_accepts_relay_ack()
    test_detection_logging()
    print("competition runner PASS")


if __name__ == "__main__":
    main()
