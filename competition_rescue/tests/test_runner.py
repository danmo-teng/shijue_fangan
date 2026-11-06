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


def test_async_diagnostics_keeps_producer_running_and_marks_overflow() -> None:
    from unittest.mock import patch
    from async_diagnostics import AsyncDiagnosticWriter

    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        held, release = threading.Event(), threading.Event()
        original = AsyncDiagnosticWriter._run

        def blocked_disk(writer):
            held.set()
            release.wait(2)
            original(writer)

        with patch.object(AsyncDiagnosticWriter, "_run", blocked_disk):
            writer = AsyncDiagnosticWriter(root / "logging.json", capacity=3)
        assert held.wait(1)
        log = JsonlLog(root / "events.jsonl", writer)
        data = {"nested": {"value": 1}}
        before = time.monotonic_ns()
        log.write("first", data)
        data["nested"]["value"] = 9
        log.write("second", {})
        log.write("overflow", {})
        assert writer.statistics()["dropped"] == 1
        assert not release.is_set()  # Producers completed while the writer was held.
        release.set()
        status = writer.close()
        rows = [json.loads(line) for line in log.path.read_text().splitlines()]
        assert [row["event"] for row in rows] == ["first", "second"]
        assert rows[0]["nested"]["value"] == 1
        assert before <= rows[0]["timestamp_monotonic_ns"] <= time.monotonic_ns()
        assert status["incomplete"] and status["pending_records"] == 0
        assert json.loads((root / "logging.json").read_text())["dropped"] == 1


def test_async_diagnostics_coalesces_snapshots_and_disk_error_is_not_control_error() -> None:
    from unittest.mock import patch
    from async_diagnostics import AsyncDiagnosticWriter

    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        release = threading.Event()
        original = AsyncDiagnosticWriter._run

        def blocked_disk(writer):
            release.wait(2)
            original(writer)

        with patch.object(AsyncDiagnosticWriter, "_run", blocked_disk):
            writer = AsyncDiagnosticWriter(root / "logging.json")
        log = JsonlLog(root / "events.jsonl", writer)
        planner = CompetitionPlanner(CompetitionMission(CompetitionSettings(side="red")),
            root / "pose", root / "stm", root / "command", root / "diagnostics.json", log, 50)
        for number in range(4):
            planner._diagnostics(CompetitionOutput(CompetitionState.SEARCH, None, str(number)),
                                 PoseSnapshot(), StmSnapshot(), VisionSnapshot(frame_sequence=number))
        assert writer.statistics()["pending_snapshots"] == 1
        assert writer.statistics()["snapshots_coalesced"] == 3
        assert not (root / "command").exists()
        release.set()
        assert not writer.close()["incomplete"]
        assert json.loads((root / "diagnostics.json").read_text())["message"] == "3"

        # A path whose parent is a file makes real background I/O fail.
        bad_parent = root / "not_a_directory"
        bad_parent.write_text("occupied")
        failed = AsyncDiagnosticWriter(root / "failure_logging.json")
        assert failed.append(bad_parent / "events.jsonl", {"event": "test"})
        failure = failed.close()
        assert failure["error"] and failure["incomplete"]
        assert not failed.append(root / "unused", {"event": "after_failure"})
        assert not (root / "command").exists()


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


    from types import SimpleNamespace
    from unittest.mock import patch
    from capture_roi import CaptureRois, split_capture_roi
    from state_machine import TrackedCargo, CargoBatch
    polygon = ((100, 100), (1100, 100), (1100, 900), (100, 900))
    rois = CaptureRois(polygon, *split_capture_roi(polygon))
    mission = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    mission._select_batch(CargoBatch((1,), ("green_supply",), "material"))
    items = (
        TrackedCargo(1, "green_supply", .9, (200, 200, 40, 40)),
        TrackedCargo(2, "green_supply", .9, (700, 200, 40, 40)),
        TrackedCargo(3, "core_black", .9, (50, 300, 100, 40)),
        TrackedCargo(4, "injured_orange", .9, (0, 400, 150, 40)),
    )
    with patch.object(runner_module, "tracked_cargo", return_value=items):
        view = runner_module.make_vision_snapshot([], [], SimpleNamespace(calibrated=False), None,
            StmSnapshot(mode=38, flags=1, age_ms=0, camera_pitch_cdeg=14000), mission, 1,
            capture_rois=rois, observed_s=1.0)
        assert view.capture_observation_valid and view.capture_audit.total_count == 2
        assert len([d for d in view.roi_decisions if not d["included"]]) == 2
        dark = runner_module.make_vision_snapshot([], [], SimpleNamespace(calibrated=False), None,
            StmSnapshot(mode=38, flags=1, age_ms=0, camera_pitch_cdeg=14000), mission, 2,
            capture_rois=rois, capture_quality_reason="capture_image_black", observed_s=1.1)
        assert dark.capture_audit is None
    mixed = runner_module.audit_from_cargo((items[0], items[3]), side_by_track={1:"left",4:"left"})
    assert mixed.left_class == "unknown" and mixed.injury_mixed and not mixed.unknown_present
    assert mission._side_is_disperse_candidate(mixed, "left") and not mission._audit_valid(mixed)
    assert not mission._should_first_green_bump(mixed)
    planner = CompetitionPlanner.__new__(CompetitionPlanner)
    planner.mission = mission
    planner.protocol_task, planner.protocol_action = 1, 2
    with tempfile.TemporaryDirectory() as directory:
        planner.events_log = JsonlLog(Path(directory) / "events.jsonl")
        output = CompetitionOutput(CompetitionState.CAPTURE_AUDIT, mission._hold(), "wait")
        status = StmSnapshot(mode=38, age_ms=0, context_valid=True, task_id=1,
                             action_status=4, rejected_action_id=2)
        planner._log_capture_decision(output, view, status, 2.0, 1, 2)
        planner._log_capture_decision(output, view, status, 2.1, 1, 2)
        records = planner.events_log.path.read_text().splitlines()
        assert len(records) == 1 and json.loads(records[0])['matched_rejection']

        mission.selected_batch = None
        planner._log_capture_decision(output, view, status, 3.0, 1, 2)
        assert json.loads(records[0])["wire_signature"] is None
        assert json.loads(records[0])["raw_audit"]["total_count"] == 2
        with patch.object(planner.events_log, "write", side_effect=OSError("test diagnostic write failure")):
            planner.last_capture_log_key = None
            planner._log_capture_decision(output, view, status, 4.0, 1, 2)
        assert planner.diagnostic_error_count == 1
    planner.running, planner.error = True, None
    planner.pose_path, planner.stm_path = Path("unused_pose"), Path("unused_stm")
    planner.lock = threading.Lock()
    planner.latest_vision, planner.paused, planner.period_s = view, False, 0.001
    published = []
    def publish(_output):
        published.append(_output)
        if len(published) == 2:
            planner.running = False
    with patch.object(runner_module, "load_pose", return_value=PoseSnapshot()), \
         patch.object(runner_module, "load_stm", return_value=status), \
         patch.object(planner, "_output_for_cycle", return_value=output), \
         patch.object(planner, "_publish", side_effect=publish), \
         patch.object(planner, "_update_grab_waitnav_diagnostic", side_effect=OSError("test diagnostic failure")):
        planner._run()
    assert len(published) == 2 and planner.error is None
    from dataclasses import replace
    overlap_items = (replace(items[0], bbox=(200, 200, 100, 100), hits=2),
                     replace(items[1], class_name="injured_orange", bbox=(205, 205, 100, 100), hits=2))
    with patch.object(runner_module, "tracked_cargo", return_value=overlap_items):
        for mode in (21, 38, 35, 23, 45, 48):
            result = runner_module.make_vision_snapshot([], [], SimpleNamespace(calibrated=False), None,
                StmSnapshot(mode=mode, flags=1, age_ms=0, camera_pitch_cdeg=14000), mission, 3,
                capture_rois=rois, observed_s=1.2)
            if mode in (21, 38, 35):
                assert result.capture_audit.total_count == 2
            else:
                assert result.capture_audit is None
    edge_rois = CaptureRois(((0, 0), (1279, 0), (1279, 1023), (0, 1023)),
                           rois.left, rois.right)
    edge_items = (replace(overlap_items[0], bbox=(0, 200, 100, 100)),
                  replace(overlap_items[1], bbox=(5, 205, 100, 100)))
    with patch.object(runner_module, "tracked_cargo", return_value=edge_items):
        result = runner_module.make_vision_snapshot([], [], SimpleNamespace(calibrated=False), None,
            StmSnapshot(mode=38, flags=1, age_ms=0, camera_pitch_cdeg=14000), mission, 4,
            capture_rois=edge_rois, observed_s=1.3)
        assert result.capture_audit is None and result.capture_observation_reason == "claw_target_truncated"


def test_range_context_and_calibration() -> None:
    from dataclasses import replace
    from state_machine import CargoBatch, TrackedCargo
    from protocol import CMD_APPROACH_RANGE
    from rescue_vision.localizer import ApproachRangeCalibration
    calibration = ApproachRangeCalibration({"angles": {"10500": {
        "measured": True, "width": 1280, "height": 1024, "pixel_reference": "bbox_center",
        "camera_mount": "test fixture only", "distance_reference": "measured_chassis_travel_to_claw_entry",
        "bands": [{"y_min": 0, "y_max": 600, "advance_mm": 700},
                  {"y_min": 600, "y_max": 1024, "advance_mm": 0}],
    }}})
    assert calibration.distance(10500, (640, 500), (1280,1024))[0] == 700
    assert calibration.distance(10500, (640, 700), (1280,1024))[0] == 0
    assert calibration.distance(12000, (640, 500), (1280,1024))[0] is None
    assert calibration.distance(10500, (640, 500), (640,512))[0] is None
    # Production tables use the saved, different partitions for each angle.
    actual = ApproachRangeCalibration.load(PROJECT_ROOT / "vision/config/approach_range.json")
    for pitch, edges, distances in (
        (10500, (0,150,291,432,603,1024), (400,400,370,160,40)),
        (12000, (0,73,217,381,580,1024), (380,300,150,50,20)),
    ):
        for index, (low, high, mm) in enumerate(zip(edges, edges[1:], distances)):
            reason = "range_user_approved_provisional" if pitch == 10500 and index < 2 else "range_measured"
            for y in (low, (low + high) / 2, high - 0.01):
                assert actual.distance(pitch, (640,y), (1280,1024)) == (mm, reason)
        assert actual.distance(pitch, (640,1024), (1280,1024))[0] is None
    unapproved = json.loads(json.dumps(actual.data))
    unapproved["angles"]["10500"]["user_approved"] = False
    assert ApproachRangeCalibration(unapproved).distance(10500, (640,75), (1280,1024))[0] is None
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        mission = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
        mission._select_batch(CargoBatch((1,), ("green_supply",), "material"))
        mission.state = CompetitionState.APPROACH
        planner = CompetitionPlanner(mission, root / "pose", root / "stm", root / "command",
                                     root / "diagnostic", JsonlLog(root / "events"), 50)
        target = TrackedCargo(1, "green_supply", .9, (600,500,40,40), hits=3)
        view = VisionSnapshot(frame_sequence=10, observed_monotonic_s=1.0, cargo=(target,),
                              range_pitch_cdeg=10500, range_distances=((1,700),))
        planner.protocol_vision = view
        planner._publish(CompetitionOutput(mission.state, CommandRequest(CMD_APPROACH_TARGET, 9, 620,520), "align"))
        pixel_action = planner.protocol_action
        status = StmSnapshot(mode=52, age_ms=0, camera_pitch_cdeg=10500, context_valid=True,
                             task_id=planner.protocol_task, action_id=pixel_action,
                             accepted_command=CMD_APPROACH_TARGET, action_status=1)
        planner._output_for_cycle(view, PoseSnapshot(), status, False, 1.0)
        view = replace(view, frame_sequence=11, observed_monotonic_s=1.1)
        request = planner._output_for_cycle(view, PoseSnapshot(), status, False, 1.1)
        planner._publish(request)
        action = planner.protocol_action
        assert action != pixel_action and request.command.flags & CMD_APPROACH_RANGE
        packet = planner.command_path.read_bytes()
        assert len(packet) == 30 and packet[3] == packet[18]
        assert int.from_bytes(packet[8:12], "big") == 11
        for mode in (52,53):
            later = planner._output_for_cycle(replace(view, frame_sequence=12, range_distances=((1,100),)),
                                              PoseSnapshot(), replace(status, mode=mode), False, 1.2)
            planner._publish(later)
            assert planner.protocol_action == action and later.command == request.command


def test_4071_fault_precedes_context_and_manual_pause() -> None:
    planner = CompetitionPlanner.__new__(CompetitionPlanner)
    planner.mission = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    planner.pause_reason = "manual"
    planner.mission.state = CompetitionState.SEARCH
    output = planner._output_for_cycle(VisionSnapshot(), PoseSnapshot(),
                                      StmSnapshot(age_ms=0, fault_code=5), True, 1.0)
    assert output.suppress_command_tx and output.command is None
    output = planner._output_for_cycle(VisionSnapshot(), PoseSnapshot(),
                                      StmSnapshot(age_ms=0, mode=3), False, 2.0)
    assert output.suppress_command_tx and output.command is None


def test_mode51_context_priority_and_separation_rejection() -> None:
    from dataclasses import replace
    from state_machine import CargoBatch
    from protocol import CMD_CARGO_AUDIT, CMD_SIDE_VALID, CargoAuditPayload
    with tempfile.TemporaryDirectory() as d:
        root = Path(d)
        log = JsonlLog(root / "events.jsonl")
        m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
        m._select_batch(CargoBatch((1,), ("green_supply",), "material"))
        p = CompetitionPlanner(m, root/'pose.json', root/'stm.json', root/'cmd.bin', root/'diag.json', log, 20)
        p.protocol_task = p.protocol_task_cursor = 10
        p.protocol_action = p.protocol_action_cursor = 20
        old = CompetitionOutput(CompetitionState.DISPERSE, CommandRequest(CMD_DISPERSE_PILE, CMD_SIDE_VALID | 1), "pending")
        p.protocol_last_output = old
        m.state = CompetitionState.DISPERSE
        m.disperse_command = old.command
        m.disperse_context = "selective"
        m.disperse_attempts = 1
        m.stm_fault_waiting = True
        s = StmSnapshot(mode=51, age_ms=0, context_valid=True, task_id=10, action_id=20,
                        accepted_command=CMD_DISPERSE_PILE, action_status=1)
        v = VisionSnapshot(frame_sequence=100, observed_monotonic_s=1.0)
        out = p._output_for_cycle(v, PoseSnapshot(), s, False, 1.0)
        assert out.command.opcode == CMD_HOLD and not m.stm_fault_waiting
        assert m.recovery_wait_active and m.disperse_attempts == 2
        assert p.protocol_last_output is None and m.selected_batch is not None
        p._publish(out)
        assert p.protocol_task == 10 and p.protocol_action == 20
        # Even an external generation change cannot allocate a new task while the budget is live.
        m.task_generation += 1
        p._publish(CompetitionOutput(m.state, CommandRequest(CMD_CARGO_AUDIT, audit=CargoAuditPayload()), "audit"))
        assert p.protocol_task == 10 and p.protocol_action == 21
        assert m.disperse_attempts == 2
        retry = replace(s, flags=1, camera_pitch_cdeg=14000, action_status=1 | 4, rejected_action_id=21)
        out = p._output_for_cycle(replace(v, frame_sequence=101, observed_monotonic_s=1.2),
                                  PoseSnapshot(), retry, False, 1.2)
        assert out.command.opcode == CMD_CARGO_AUDIT and out.command.audit_vision_frame == 0
        p._publish(out)
        assert p.protocol_task == 10 and p.protocol_action == 22 and m.disperse_attempts == 2
        # Fresh same-task rejection of an unaccepted ordinary action returns to a new frame floor.
        m.recovery_wait_active = False
        m.recovery_task_locked = False
        m.state = CompetitionState.DISPERSE
        m.disperse_command_accepted = False
        p.protocol_last_output = old
        p.protocol_action = p.protocol_action_cursor = 25
        rejection = StmSnapshot(mode=37, flags=1, age_ms=0, camera_pitch_cdeg=14000,
                                context_valid=True, task_id=10, action_id=24,
                                accepted_command=CMD_CARGO_AUDIT, action_status=1 | 4, rejected_action_id=25)
        assert p._reconcile_separation_rejection(v, replace(rejection, task_id=11), 2.0) is None
        assert p._reconcile_separation_rejection(v, replace(rejection, rejected_action_id=23), 2.0) is None
        m.disperse_command_accepted = True
        assert p._reconcile_separation_rejection(v, rejection, 2.0) is None
        m.disperse_command_accepted = False
        out = p._output_for_cycle(v, PoseSnapshot(), rejection, False, 2.0)
        assert out.command.opcode == CMD_CARGO_AUDIT and m.disperse_attempts == 2
        assert m.audit_recheck_frame_floor == 100 and p.protocol_last_output is None
        p._publish(out)
        assert p.protocol_task == 10 and p.protocol_action == 26
        p.protocol_action_cursor = 0xFFFF
        p.protocol_action_key = None
        p.protocol_accepted = replace(rejection, action_id=0xFFFF)
        p._publish(out)
        assert p.protocol_task == 11 and p.protocol_action == 0 and m.disperse_attempts == 2
        p.protocol_action_cursor = 0xFFFF
        p.protocol_action_key = None
        p.protocol_accepted = replace(rejection, task_id=11, action_id=0xFFF0)
        p._publish(out)
        assert p.protocol_task == 11 and p.protocol_action == 0  # No unconfirmed task rollover.


def test_received_status_age_does_not_pause_navigation_but_claw_audit_keeps_check():
    from dataclasses import replace
    from protocol import CMD_CARGO_AUDIT, CMD_RETURN_CENTER
    m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    m.state = CompetitionState.RETURN_CENTER
    status = StmSnapshot(mode=10, age_ms=2000, camera_pitch_cdeg=14000, flags=1)
    out = m.step(VisionSnapshot(), PoseSnapshot(valid=True, x_m=.5), status, 1.)
    assert out.command.opcode == CMD_RETURN_CENTER
    assert status.fresh and not status.audit_fresh and not StmSnapshot().fresh
    # The old check remains local to authorizing actual claw observations.
    waiting = m._audit_observation_wait(VisionSnapshot(frame_sequence=1), status, 1.)
    assert waiting.command.opcode == CMD_CARGO_AUDIT
    assert waiting.command.audit_vision_frame == 0 and not waiting.command.audit.stable
    assert replace(status, age_ms=0).audit_fresh


def test_rejected_approach_exits_pending_and_finishes_separation_budget():
    from dataclasses import replace
    from protocol import CMD_CARGO_AUDIT, CMD_RETURN_CENTER
    from state_machine import CargoBatch
    with tempfile.TemporaryDirectory() as d:
        root = Path(d)
        for attempts in (0, 1):
            m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
            m._select_batch(CargoBatch((1,), ("injured_orange",), "injury"))
            m.state = CompetitionState.APPROACH
            m.disperse_attempts = attempts
            p = CompetitionPlanner(m, root/"pose", root/"stm", root/"command",
                                   root/"diag", JsonlLog(root/"events"), 50)
            p._publish(CompetitionOutput(m.state, CommandRequest(CMD_APPROACH_TARGET, 9, 299, 95), "request"))
            rejected = StmSnapshot(mode=3, age_ms=1000, context_valid=True,
                task_id=p.protocol_task, action_id=(p.protocol_action-1)&65535,
                accepted_command=CMD_CARGO_AUDIT, action_status=7,
                rejected_action_id=p.protocol_action)
            v = VisionSnapshot(frame_sequence=100, observed_monotonic_s=1.)
            assert p._reconcile_approach_rejection(v, PoseSnapshot(), replace(rejected, rejected_action_id=0), 1.) is None
            out = p._output_for_cycle(v, PoseSnapshot(valid=True, x_m=.2), rejected, False, 1.)
            assert out.reason == "approach_recover_after_reject" and p.protocol_last_output is None
            assert out.state == (CompetitionState.RETURN_CENTER if attempts else CompetitionState.SEARCH)
            if attempts:
                assert out.command.opcode == CMD_RETURN_CENTER and m.disperse_attempts == 1
            p._publish(out)
            assert p.protocol_action != rejected.rejected_action_id or out.command.opcode == CMD_HOLD


def test_pending_pixel_approach_continues_updating_camera_target():
    from state_machine import CargoBatch, TrackedCargo
    with tempfile.TemporaryDirectory() as d:
        root = Path(d)
        m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
        m._select_batch(CargoBatch((1,), ("green_supply",), "material"))
        m.state = CompetitionState.APPROACH
        p = CompetitionPlanner(m, root/"pose", root/"stm", root/"command", root/"diag", JsonlLog(root/"events"), 50)
        p._publish(CompetitionOutput(m.state, CommandRequest(CMD_APPROACH_TARGET, 9, 200, 200), "old target"))
        status = StmSnapshot(mode=3, age_ms=500, context_valid=True, task_id=p.protocol_task,
                             action_id=(p.protocol_action-1)&65535, action_status=1)
        target = TrackedCargo(1, "green_supply", .9, (600,500,40,40), hits=3)
        view = VisionSnapshot(frame_sequence=20, observed_monotonic_s=1., cargo=(target,))
        out = p._output_for_cycle(view, PoseSnapshot(valid=True), status, False, 1.)
        assert out.command.opcode == CMD_APPROACH_TARGET
        assert (out.command.arg_a, out.command.arg_b) != (200, 200)
        p._publish(out)
        assert p.protocol_vision_frame == 20


def test_wheel_bridge_coalesces_ready_velocities_without_age_rejection():
    import socket
    sys.path.insert(0, str(PROJECT_ROOT/'localization/tools'))
    from t265_wheel_input import WheelInputBridge
    with tempfile.TemporaryDirectory() as d:
        config = dict(camera_robot_forward_axis='0.8660254,0,0.5',camera_robot_up_axis='0,-1,0',
                      camera_offset_forward_m='-0.0346105',camera_offset_left_m='-0.0599471445')
        bridge = WheelInputBridge(Path(d),config,'coalesce-fixture',True,clock=lambda:10_000_000_000)
        sink = socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
        sink.bind(bridge.address);sink.settimeout(.1)
        try:
            for i in range(7):
                bridge.gyro.append(dict(mcu_epoch='fixture',alignment_clock='mcu_event',
                    alignment_capture_ns=1_000_000_000+i*10_000_000,yaw_rate_deg_s=0.,
                    raw_mcu_sample=dict(rate_source_id=i,fixture_payload='x'*2000),
                    receive_monotonic_ns=1_000_000_000+i*10_000_000))
            for i in range(3):
                bridge.telemetry(dict(kind='measured_body_velocity',sample=dict(
                    mcu_epoch='fixture',sample_id=i,measurement_valid=True,source='measured_wheel_feedback',
                    alignment_clock='mcu_event',alignment_start_ns=1_000_000_000+i*20_000_000,
                    alignment_end_ns=1_020_000_000+i*20_000_000,receive_monotonic_ns=1_020_000_000+i*20_000_000,
                    v_body_mps=[.1*(i+1),0.,0.])))
            bridge.poll()
            packet=json.loads(sink.recv(16384))
            assert packet['sample_id']==2 and abs(packet['v_camera_mps'][0]-.3*.8660254)<1e-7
            assert bridge.stats['sent_to_sdk_owner']==1 and bridge.stats['sdk_requests_coalesced']==2
            assert bridge.stats['rejected']==0 and packet['capture_time_qualified'] is False
            assert all('raw_mcu_sample' not in g for g in packet['gyro_samples'])
            assert packet['gyro_samples'][0]['rate_source_id'] == 4
        finally:
            bridge.close();sink.close()


def test_published_post_grab_audit_survives_changed_frame_before_paired_done():
    from dataclasses import replace
    from protocol import CMD_CARGO_AUDIT
    from state_machine import CargoAudit, CargoBatch
    with tempfile.TemporaryDirectory() as d:
        root = Path(d)
        m = CompetitionMission(CompetitionSettings(side='red', initial_stash_enabled=False))
        m._select_batch(CargoBatch((1,), ('green_supply',), 'material'))
        p = CompetitionPlanner(m, root/'pose', root/'stm', root/'cmd', root/'diag', JsonlLog(root/'events'), 50)
        m._begin_post_grab_audit(VisionSnapshot(frame_sequence=10), 1.)
        m.audit_camera_key = (23, True, m.audit_epoch)
        m.audit_camera_ready_since = 1.
        m.audit_camera_floor = 10
        m.audit_last_frame_sequence = 20
        audit = CargoAudit(left_class='green_supply', left_count=1, left_green_count=1, total_count=1, stable=True)
        status = StmSnapshot(mode=23, flags=1|2, age_ms=0, camera_pitch_cdeg=14000)
        m._begin_audit_confirmation(audit, m._audit_payload(audit), True, 'both', False,
                                    'post_grab_valid', status, 2.)
        stable = CommandRequest(CMD_CARGO_AUDIT, 1, audit=m.pending_audit_payload, audit_vision_frame=20)
        p._publish(CompetitionOutput(m.state, stable, 'published valid'))
        task, action = p.protocol_task, p.protocol_action
        status = replace(status, context_valid=True, task_id=task, action_id=action,
                         accepted_command=CMD_CARGO_AUDIT, action_status=1)
        empty = VisionSnapshot(frame_sequence=21, observed_monotonic_s=2.05,
                               capture_audit=CargoAudit(), capture_observation_valid=True)
        changed = p._output_for_cycle(empty, PoseSnapshot(valid=True), status, False, 2.05)
        assert m.pending_audit is None  # New candidate still follows real observations.
        p._publish(changed)
        assert (p.protocol_task, p.protocol_action) == (task, action)
        done = replace(status, mode=22, flags=2|16|64, relay_mission_tx_frames=5,
                       relay_last_mission_command=CMD_CARGO_AUDIT,
                       relay_last_mission_payload=tuple(changed.command.to_frame(0)[4:12]))
        p._restore_audit_handoff(replace(done, action_id=action+1))
        assert m.pending_audit is None  # A different action cannot borrow evidence.
        out = p._output_for_cycle(empty, PoseSnapshot(valid=True), done, False, 2.1)
        assert out.reason == 'post_grab_audit_confirmed' and m.grab_complete_confirmed
        assert m.pending_audit == audit and m.carried_total_count == 1
        p._publish(out)
        assert p._output_for_cycle(empty, PoseSnapshot(valid=True), done, False, 2.12).reason != 'missing_committed_audit'


def test_range_move_camera_handoff_never_rearms_pixel_approach():
    from dataclasses import replace
    from protocol import CMD_APPROACH_RANGE, CMD_DISTANCE_VALID
    from state_machine import CargoBatch, TrackedCargo
    with tempfile.TemporaryDirectory() as d:
        root = Path(d)
        for state, audit_mode in ((CompetitionState.APPROACH,21), (CompetitionState.CLUSTER_APPROACH,38)):
            m = CompetitionMission(CompetitionSettings(side='red', initial_stash_enabled=False))
            m._select_batch(CargoBatch((1,), ('green_supply',), 'material'))
            m.state = state
            p = CompetitionPlanner(m, root/'pose', root/'stm', root/'cmd', root/'diag', JsonlLog(root/'events'), 50)
            m.range_command = CommandRequest(CMD_APPROACH_TARGET, 1|CMD_APPROACH_RANGE|CMD_DISTANCE_VALID, 400, 12000,
                                             source_vision_frame=20)
            p._publish(CompetitionOutput(state,m.range_command,'range move'))
            original = (p.protocol_task, p.protocol_action)
            status = StmSnapshot(mode=20, age_ms=0, camera_pitch_cdeg=12100, context_valid=True,
                task_id=original[0],action_id=original[1],accepted_command=CMD_APPROACH_TARGET,action_status=1)
            view = VisionSnapshot(frame_sequence=30, observed_monotonic_s=1.,
                cargo=(TrackedCargo(1,'green_supply',.9,(500,400,40,40),hits=3),))
            out = p._output_for_cycle(view,PoseSnapshot(valid=True),status,False,1.)
            assert out.reason == 'approach_range_handoff' and out.command.opcode == CMD_HOLD
            p._publish(out)
            assert original == (p.protocol_task,p.protocol_action) and m.range_command is not None
            status = replace(status,mode=audit_mode,camera_pitch_cdeg=14000,flags=1)
            out = p._output_for_cycle(view,PoseSnapshot(valid=True),status,False,1.1)
            assert out.command.opcode != CMD_APPROACH_TARGET and m.range_command is None
            assert m.state == CompetitionState.CAPTURE_AUDIT


def test_sweep_duplicate_rejection_does_not_reset_accepted_pickup():
    from dataclasses import replace
    from protocol import CMD_CARGO_AUDIT, CargoAuditPayload
    from state_machine import CargoBatch, SweepPickupContext
    with tempfile.TemporaryDirectory() as d:
        root = Path(d)
        m = CompetitionMission(CompetitionSettings(side='red', initial_stash_enabled=False))
        m._select_batch(CargoBatch((1,), ('green_supply',), 'material'))
        m.state = CompetitionState.CLEAR_SAFE_ZONE
        m.safe_sweep_pickup = SweepPickupContext(target_id=2,target_class='injured_orange',target_bbox=None,
            original_ids=frozenset({1}),original_classes=frozenset({'green_supply'}),excluded_ids=frozenset())
        m.safe_sweep_pickup.mode = 43
        m.safe_sweep_pickup.audit_hits = 2
        p = CompetitionPlanner(m,root/'pose',root/'stm',root/'cmd',root/'diag',JsonlLog(root/'events'),50)
        p._publish(CompetitionOutput(m.state,CommandRequest(CMD_CARGO_AUDIT,audit=CargoAuditPayload()),'audit'))
        status = StmSnapshot(mode=39,age_ms=0,context_valid=True,task_id=p.protocol_task,
            action_id=p.protocol_action,accepted_command=CMD_CARGO_AUDIT,action_status=5,rejected_action_id=p.protocol_action)
        assert p._reconcile_sweep_rejection(VisionSnapshot(),PoseSnapshot(),status,1.) is None
        assert m.safe_sweep_pickup.mode == 43 and m.safe_sweep_pickup.audit_hits == 2
        assert p._reconcile_sweep_rejection(VisionSnapshot(),PoseSnapshot(),replace(status,task_id=status.task_id+1),1.) is None


def test_lidar_receiver_and_motion_continue_while_fit_is_blocked():
    """Real PTY + existing worker; no SDK, chassis port, GUI or real log directory."""
    import multiprocessing as mp
    import os
    import pty
    import queue
    import struct
    from unittest.mock import patch
    sys.path.insert(0,str(PROJECT_ROOT/'localization/tools'))
    from lidar_yaw_debug.dynamic import DynamicFence
    from lidar_yaw_debug.protocol import crc8
    from lidar_yaw_debug.worker import run_worker

    def revolution(index):
        packets=[]
        for k in range(60):
            first=k*600
            data=bytes([0x54,0x2c])+struct.pack('<HH',3600,first)
            data+=struct.pack('<HB',1500,100)*12
            data+=struct.pack('<HH',first+550,(index*60+k)*3%65536)
            packets.append(data+bytes([crc8(data)]))
        return b''.join(packets)

    ctx=mp.get_context('spawn');stop=ctx.Event()
    inbox,display,replies=queue.Queue(),ctx.Queue(maxsize=4),ctx.Queue(maxsize=16)
    master,slave=pty.openpty();entered,release=threading.Event(),threading.Event()
    original=DynamicFence.observe
    def slow_fit(engine,scan,**kwargs):
        entered.set();release.wait(4)
        return original(engine,scan,**kwargs)
    config=json.loads((PROJECT_ROOT/'localization/config/lidar_yaw_debug.json').read_text())
    with tempfile.TemporaryDirectory() as d, patch.object(DynamicFence,'observe',slow_fit):
        worker=threading.Thread(target=run_worker,args=(config,os.ttyname(slave),230400,Path(d),inbox,display,replies,stop))
        worker.start()
        try:
            now=time.monotonic_ns()
            inbox.put(('pose',dict(session_id='fixture',frame_number=1,timestamp_monotonic_ns=now,
                receive_monotonic_ns=now,x_m=0.,y_m=0.,yaw_rad=0.,tracker_confidence=3)))
            inbox.put(('yaw',dict(time_ns=now,yaw_deg=0.,source={})))
            inbox.put(('field_start',dict(description='fixture',center_xy_m=[0.,0.],yaw_deg=0.,first_sample=False)))
            # Wait only for local receiver startup, not a sensor qualification.
            deadline=time.monotonic()+4
            while time.monotonic()<deadline:
                try:state=display.get(timeout=.1)
                except queue.Empty:continue
                if state['lidar_connected']:break
            else:raise AssertionError('PTY receiver did not open')
            os.write(master,revolution(0)+revolution(1))
            assert entered.wait(3)
            for k in range(2,5):os.write(master,revolution(k))
            for i in range(1,3):
                t=time.monotonic_ns()
                inbox.put(('yaw',dict(time_ns=t,yaw_deg=0.,source={})))
                inbox.put(('wheel',dict(source='measured_wheel_feedback',measurement_valid=True,
                    mcu_epoch='fixture',sample_id=i,v_body_mps=[.2,0.,0.],receive_monotonic_ns=t,
                    interval_start_ns=t-20_000_000,interval_end_ns=t,
                    raw_mcu_sample=dict(t0_us=i*20000,t1_us=(i+1)*20000))))
                inbox.put(('pose',dict(session_id='fixture',frame_number=i+1,timestamp_monotonic_ns=t,
                    receive_monotonic_ns=t,x_m=.004*i,y_m=0.,yaw_rad=0.,tracker_confidence=3)))
            deadline=time.monotonic()+2
            while time.monotonic()<deadline:
                try:state=display.get(timeout=.1)
                except queue.Empty:continue
                if state['measured_wheel_ingested']>=2 and state['scans_received']>=3:break
            else:raise AssertionError('motion/receiver stalled behind the blocked fit')
            assert state['last']['ready'] and not release.is_set()
            assert state['reader_stats']['crc_errors']==0
        finally:
            release.set();stop.set();worker.join(timeout=5)
            os.close(master);os.close(slave)
        assert not worker.is_alive()
        final=None
        while True:
            try:message=replies.get(timeout=.2)
            except queue.Empty:break
            if message['kind']=='stopped':final=message['summary']
        assert final is not None
        assert final['receiver_stopped_cleanly'] and final['fit_thread_stopped']
        assert (Path(d)/'lidar_rx.jsonl').stat().st_size>0
    for channel in (display,replies):channel.cancel_join_thread();channel.close()


def main() -> None:
    test_received_status_age_does_not_pause_navigation_but_claw_audit_keeps_check()
    test_rejected_approach_exits_pending_and_finishes_separation_budget()
    test_pending_pixel_approach_continues_updating_camera_target()
    test_mode51_context_priority_and_separation_rejection()
    test_4071_fault_precedes_context_and_manual_pause()
    test_suppressed_publish_preserves_command_file_and_sequence()
    test_relay_snapshot_parsing_and_abort_evidence()
    test_planner_stop_sends_bounded_abort_and_accepts_relay_ack()
    test_detection_logging()
    print("competition runner PASS")


if __name__ == "__main__":
    test_range_context_and_calibration()
    main()


def test_formal_wheel_start_and_pose_adoption():
    """No UART/SDK: formal coordinates, lifecycle and received-pose policy."""
    from unittest.mock import patch
    from types import SimpleNamespace
    sys.path.insert(0, str(PROJECT_ROOT / 'rescue_map'))
    import map_app
    import competition_map_app as formal
    from field_model import load_localization_pose
    with tempfile.TemporaryDirectory() as directory:
        runtime = Path(directory)
        options = SimpleNamespace(zone=1, side='red', corner_offset_mm=150*2**.5,
            encoder_weight=.25, localization_mode='t265', localization_json=runtime/'pose.json',
            localization_log=None, launch_localization=False, launch_vision=False,
            uart='/dev/ttyS1', baud=230400, tx_rate=20., fullscreen=False, demo=False,
            screenshot=None, official_wheel_input=True)
        with patch.object(map_app, 'RUNTIME', runtime), patch.object(formal, 'RUNTIME', runtime):
            app = formal.CompetitionMapApp(options)
            try:
                app.start_session()
                cmd = app.localization_command()
                assert cmd.count('--uart') == 1 and '--ignore-encoders' in cmd
                assert all(x in cmd for x in ('--debug-fresh-session', '--debug-test-session',
                    '--enable-pose-jumping', '--t265-only-output', '--wheel-feedback-socket'))
                conf = Path(cmd[cmd.index('--config')+1]).read_text()
                assert 'start_zone = 1' in conf and 'start_center_m = 1.350000' in conf
                assert 'start_heading_offset_deg = 0.0' in conf
                assert 'camera_offset_forward_m = -0.0346105' in conf
                assert 'camera_robot_forward_axis = 0.8660254,0,0.5' in conf
                assert app.pose.yaw_deg == 135 and app.pose.x_m != 0
                assert app.encoder_weight == 0 and not app.t265_translation_scale_enabled
                assert len(app.wheel_input.requests_path.read_bytes()) == 42
                before = app.wheel_input_status()
                assert before['sdk_api_success_samples'] == 0 and not before['config_loaded']
                session = json.loads((runtime/'session.json').read_text())
                assert session['actual_lower_firmware'] == 'unconfirmed'
                assert 'expected_lower_firmware' not in session
                options.localization_json.write_text(json.dumps(dict(frame='field',quality='GOOD',
                    timestamp_monotonic_ns=time.monotonic_ns()-5_000_000_000,
                    pose=dict(x_m=.1,y_m=.2,yaw_deg=30))))
                assert load_localization_pose(options.localization_json).quality == 'STALE'
                app.update_pose()
                assert app.pose.quality == 'GOOD' and app.pose.age_ms > 250
                pose = runner_module.load_pose(options.localization_json)
                assert pose.valid and CompetitionMission._pose_fresh(pose)
            finally:
                adapter = app.wheel_input
                app.stop_localization()
            assert app.wheel_input is None and not adapter.thread.is_alive()
            assert not (app.wheel_run_dir/'session_complete.json').exists()


def test_formal_dynamic_start_wiring():
    """Launcher wiring only; mock worker start, no UART/SDK and no runtime files."""
    from unittest.mock import patch
    from types import SimpleNamespace
    import competition_map_app as formal
    import map_app
    from competition_dynamic_localization import CompetitionDynamicLocalization
    with tempfile.TemporaryDirectory() as directory:
        runtime = Path(directory)
        options = SimpleNamespace(zone=2, side='blue', corner_offset_mm=150*2**.5,
            encoder_weight=.25, localization_mode='t265', localization_json=runtime/'pose.json',
            localization_log=None, launch_localization=False, launch_vision=False,
            uart='/dev/ttyS1', baud=230400, tx_rate=20., fullscreen=False, demo=False,
            screenshot=None, official_wheel_input=False, dynamic_localization=True,
            dynamic_config=PROJECT_ROOT/'localization/config/lidar_yaw_debug.json',
            lidar='/dev/ttyS6', lidar_baud=230400)
        with patch.object(map_app, 'RUNTIME', runtime), patch.object(formal, 'RUNTIME', runtime), \
                patch.object(CompetitionDynamicLocalization, 'start') as start:
            app = formal.CompetitionMapApp(options)
            try:
                app.start_session(); cmd = app.localization_command()
                assert start.call_count == 1 and app.official_wheel_input
                assert app.dynamic.field_start == dict(x_m=1.35,y_m=1.35,yaw_deg=45.)
                assert cmd.count('--uart') == 1 and cmd[cmd.index('--uart')+1] == '/dev/ttyS1'
                assert cmd[cmd.index('--dynamic-pose-output')+1] == str(runtime/'pose.json')
                assert Path(cmd[cmd.index('--output')+1]).name == 'raw_t265_result.json'
                assert '--debug-pose-socket' in cmd and '--wheel-feedback-socket' in cmd
                assert app.wheel_input.telemetry_sink == app.dynamic.telemetry
                assert app.localization_command() == cmd and start.call_count == 1
                metadata = json.loads((runtime/'session.json').read_text())
                assert metadata['pose_source'] == 'lidar_t265_measured_wheel_fallback'
                assert metadata['qualification']['translation_compensated'] is False
                assert metadata['accuracy_verified'] is False
            finally:
                app.stop_localization()
            assert app.dynamic is None and app.wheel_input is None


def test_incident_video_replay_and_log_only_timeout():
    """Encode/read back a short fixture; no camera, motors, runtime logs or GUI."""
    import cv2
    from dataclasses import replace
    from unittest.mock import patch
    from types import SimpleNamespace
    from vision_replay import VisionReplay
    from state_machine import CargoBatch
    # Logging changes must not alter the existing no-box command/transition.
    mission = CompetitionMission(CompetitionSettings(side='red',initial_stash_enabled=False))
    mission._select_batch(CargoBatch((1,),('green_supply',),'material'))
    mission.confirmed_delivery_destination='material'
    mission._begin_safe_zone_acquire(VisionSnapshot(frame_sequence=1),1.)
    p = PoseSnapshot(valid=True,x_m=-.15,y_m=.54,yaw_deg=90,age_ms=0)
    s = StmSnapshot(mode=11,age_ms=0,flags=2,camera_pitch_cdeg=12000)
    a = mission.step(VisionSnapshot(frame_sequence=2,observed_monotonic_s=6.1),p,s,6.1)
    b = mission.step(VisionSnapshot(frame_sequence=3,observed_monotonic_s=6.2),p,s,6.2)
    assert a.event=='safe_zone_acquire_no_box' and not b.event
    assert a.state==b.state==CompetitionState.ACQUIRE_SAFE_ZONE and a.command==b.command
    with tempfile.TemporaryDirectory() as directory:
        root=Path(directory);r=VisionReplay(root,'replay-fixture',fps=8,pre_s=.3,post_s=.3,size=(192,128))
        planner = CompetitionPlanner(mission,root/'pose.json',root/'stm.json',root/'command.bin',
            root/'diagnostics.json',JsonlLog(root/'events.jsonl'),50.)
        planner.replay_event_sink = r.record_event
        r.start()
        try:
            deadline=time.monotonic()+3
            while not (root/'vision_replay_manifest.json').exists() and time.monotonic()<deadline:time.sleep(.02)
            for i in range(10):
                image=np.full((128,192,3),(i*20,20,40),np.uint8)
                if i in (4,6):
                    planner._record_replay_event(a,p,s,VisionSnapshot(frame_sequence=i),1.+i*.13)
                meta=dict(frame_sequence=i,capture_monotonic_ns=1_000_000_000+i*130_000_000,
                    source_size=[192,128],detections=[dict(bbox=[10,20,20,20],confidence=.8,**{'class':'green_supply'})],
                    pose=dict(x_m=.1,y_m=.2,yaw_deg=90,age_ms=float('inf')),stm=dict(mode=11,camera_pitch_cdeg=12000),
                    # The timeout state has already disappeared by the sampled
                    # image; the planner event must still trigger the clip.
                    upper_state='ALIGN_SAFE_ZONE',acquire_started_s=1.,reason='safe_zone_visible')
                assert r.submit(image,'bgr',meta,now=1.+i*.13)
                time.sleep(.03)
        finally:r.close()
        manifest=json.loads((root/'vision_replay_manifest.json').read_text())
        assert manifest['complete'] and manifest['error'] is None and manifest['dropped_frames']==0,manifest
        assert len(manifest['clips'])==1
        clip=manifest['clips'][0];rows=[json.loads(l) for l in (root/clip['frames']).read_text().splitlines()]
        assert [x['frame_sequence'] for x in rows]==[2,3,4,5,6,7,8]
        assert [event['vision_frame_sequence'] for event in clip['events']]==[4,6]
        assert clip['post_window_end_ns']==2_080_000_000
        assert manifest['pending_events']==[] and manifest['event_overflow_count']==0
        assert rows[0]['pose']['age_ms'] is None and rows[0]['seconds_from_trigger']<0
        assert rows[-1]['seconds_from_trigger']>0 and clip['complete']
        cap=cv2.VideoCapture(str(root/clip['video']));count=0
        while True:
            ok,frame=cap.read()
            if not ok:break
            assert frame.shape==(224,192,3)
            assert abs(float(frame[180,180,0])-rows[count]['frame_sequence']*20)<8
            count+=1
        cap.release();assert count==len(rows)==clip['frames_written']
        # Bounded queue pressure changes only diagnostic counters, no blocking.
        r.next_frame_s=0.;r.error=None;r.started=True
        r.record_event(dict(event='safe_zone_acquire_no_box',trigger_monotonic_ns=10_000_000_000))
        with patch.object(r,'process',SimpleNamespace(is_alive=lambda:True)), \
             patch.object(r.inbox,'put_nowait',side_effect=[__import__('queue').Full,None]) as enqueue:
            assert not r.submit(image,'bgr',meta,now=10.) and r.dropped.value==1
            assert len(r.pending_events)==1
            assert not r.submit(image,'bgr',meta,now=10.01) and len(r.pending_events)==1
            assert r.submit(image,'bgr',meta,now=10.13) and not r.pending_events
            assert enqueue.call_args.args[0][3][0]['trigger_monotonic_ns']==10_000_000_000
        # Retention recognizes only this session's completed, manifested files.
        sys.path.insert(0,str(PROJECT_ROOT/'localization/tools'))
        from motion_log_storage import inspect_session
        import competition_map_app as formal
        parent=root/'sessions';parent.mkdir();run=parent/'20261105_000001_competition_wheel';run.mkdir()
        for file in root.glob('safe_zone_replay_*'): file.rename(run/file.name)
        (root/'vision_replay_manifest.json').rename(run/'vision_replay_manifest.json')
        (run/'metadata.json').write_text(json.dumps(dict(owner=formal.WHEEL_OWNER,mode='competition_t265_wheel_input',sdk_session_id='replay-fixture')))
        (run/'session_complete.json').write_text(json.dumps(dict(owner=formal.WHEEL_OWNER,complete=True,session_index=1)))
        assert not inspect_session(parent,run.name,None,formal.WHEEL_FILES,owner=formal.WHEEL_OWNER)['reason']
        (run/'user.txt').write_text('keep')
        assert inspect_session(parent,run.name,None,formal.WHEEL_FILES,owner=formal.WHEEL_OWNER)['reason']
        # Worker-side disk failure is observable and cannot escape to control.
        broken=VisionReplay(root/'missing_directory','broken-replay')
        broken.start();broken.process.join(3)
        assert not broken.submit(image,'bgr',meta,now=20.)
        assert broken.close()['error']


def test_operator_buttons_and_feedback_encoding():
    """Human input sequences only; no GPIO, UART or sensor access."""
    from button_input import SysfsButtonInput
    from operator_protocol import beep_frame
    from protocol import crc16_modbus
    levels = {1: 1, 2: 1, 3: 0, 4: 1}
    buttons = SysfsButtonInput(read_level=lambda key: levels[key])
    assert buttons.poll(0.) == []
    assert buttons.poll(2.) == []  # Boot with START held never starts.
    levels[3] = 1; buttons.poll(2.1)
    assert buttons.poll(2.14) == []
    levels[1] = 0; buttons.poll(3.); assert buttons.poll(3.04) == []
    assert [e['action'] for e in buttons.poll(4.1)] == ['next_zone']
    assert buttons.poll(5.) == []  # Held key does not repeat selection.
    levels[1] = 1; buttons.poll(5.1); assert buttons.poll(5.14) == []
    levels[3] = 0; buttons.poll(6.); buttons.poll(6.04)
    levels[4] = 0; levels[3] = 1; buttons.poll(6.1)
    assert [e['action'] for e in buttons.poll(6.14)] == ['stop']
    assert [e['action'] for e in buttons.poll(8.2)] == ['exit_session']
    for count in range(1, 5):
        for long in (False, True):
            frame = beep_frame(count, long, 9)
            assert len(frame) == 15 and frame[2:7] == bytes((0x20, 9, 1, count, long))
            assert int.from_bytes(frame[12:14], 'little') == crc16_modbus(frame[2:12])
    assert beep_frame().hex() == 'a3b320000101000000000000a061c3'


def test_operator_panel_stop_and_session_ownership():
    from types import SimpleNamespace
    from unittest.mock import Mock, patch
    import operator_panel as panel_module
    from operator_panel import OperatorPanel
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        (root/'operator_selection.json').write_text('{"zone":3,"side":"blue"}')
        app = SimpleNamespace(options=SimpleNamespace(zone=None, side=None, uart='PTY-only', baud=230400),
            zone=1, side='red', corner_offset_m=.15*2**.5, selecting=True,
            session_started=False, reset_session_token=None, vision_start_pending=True,
            message='', record_reset_event=Mock(), start_session=Mock(), reset_session=Mock())
        with patch.object(panel_module, 'RUNTIME', root), \
                patch.object(panel_module, 'send_abort', return_value=True) as abort, \
                patch.object(panel_module, 'send_beep', return_value=True) as beep:
            panel = OperatorPanel(app)
            assert (app.zone, app.side, app.selecting) == (3, 'blue', True)
            panel._enqueue(dict(action='next_zone',key=1));panel.tick()
            assert app.zone == 4 and beep.call_args.kwargs['count'] == 4
            panel._enqueue(dict(action='start',key=3,_stop_serial=0))
            panel.request_stop();panel.tick()
            app.start_session.assert_not_called()  # STOP cancels queued START.
            standby = Mock();standby.poll.return_value=None;panel._standby=standby
            panel.before_start()
            standby.terminate.assert_called_once();standby.wait.assert_called_once()
            assert panel._standby is None and not panel.stopped
            token = app.reset_session_token
            assert token and not panel.control_path.exists()
            # STOP during preparation reaches the owner immediately, not the UI queue.
            assert panel.request_stop() and abort.call_count == 2
            assert runner_module.operator_stop_requested(panel.control_path, token)
            assert not runner_module.operator_stop_requested(panel.control_path, 'previous-session')
            panel.after_start()
            assert panel.stopped and not app.vision_start_pending
            app.selecting=False
            panel._enqueue(dict(action='next_zone',key=1));panel.tick()
            assert app.zone == 4  # Active round can query, never change reference.
            panel.after_reset()
            assert app.reset_session_token is None and not panel.control_path.exists()
            panel.close()


def test_formal_button_launcher_wiring_without_hardware():
    from unittest.mock import patch
    from types import SimpleNamespace
    import competition_map_app as formal
    import operator_panel as panel_module
    import map_app
    with tempfile.TemporaryDirectory() as directory:
        runtime = Path(directory)
        options = SimpleNamespace(zone=2, side='blue', corner_offset_mm=150*2**.5,
            encoder_weight=0., localization_mode='t265', localization_json=runtime/'pose.json',
            localization_log=None, launch_localization=False, launch_vision=False,
            uart='PTY-only', baud=230400, tx_rate=20., fullscreen=False, demo=False,
            screenshot=None, official_wheel_input=False, dynamic_localization=False,
            gpio_buttons=True, headless=True)
        with patch.object(map_app,'RUNTIME',runtime), patch.object(formal,'RUNTIME',runtime), \
                patch.object(panel_module,'RUNTIME',runtime):
            app = formal.CompetitionMapApp(options)
            assert app.selecting and not app.session_started  # Even with CLI zone/side.
            app.start_session()
            token = app.reset_session_token
            command = app.localization_command(); vision = app.vision_command()
            assert command[command.index('--operator-socket')+1] == app.operator.socket_name
            assert '--no-display' in vision
            assert vision[vision.index('--operator-control')+1] == str(runtime/'operator_control.json')
            assert vision[vision.index('--reset-session-token')+1] == token
            assert app.vision_command()[app.vision_command().index('--reset-session-token')+1] == token
            app.reset_session(source='offline-check')
            assert app.selecting and not app.session_started and app.reset_session_token is None
            app.operator.close()


def test_operator_standby_returns_after_failed_start_only_after_cleanup():
    from types import SimpleNamespace
    from unittest.mock import Mock, patch
    import competition_map_app as formal

    def make_app():
        app = formal.CompetitionMapApp.__new__(formal.CompetitionMapApp)
        app.session_started = False
        app.prewarm_enabled = False
        app.selecting = True
        app.message = ''
        app.record_reset_event = Mock()
        calls = []
        app.stop_session_processes = Mock(side_effect=lambda: calls.append('session_owners_closed'))
        app.operator = SimpleNamespace(transitioning=False)
        def before_start():
            app.operator.transitioning = True
            calls.append('standby_owner_closed')
        def after_start():
            app.operator.transitioning = False
        def after_reset():
            assert calls[-1] == 'session_owners_closed'
            calls.append('standby_owner_restored')
        app.operator.before_start = Mock(side_effect=before_start)
        app.operator.after_start = Mock(side_effect=after_start)
        app.operator.after_reset = Mock(side_effect=after_reset)
        return app, calls

    # A synchronous spawn failure is already reported by the base method.
    app, calls = make_app()
    with patch.object(formal.RescueMapApp, 'start_session', return_value=None):
        app.start_session()
    assert calls == ['standby_owner_closed', 'session_owners_closed', 'standby_owner_restored']
    assert not app.operator.transitioning

    # A failure inside before_start also clears the transition state.
    app, calls = make_app()
    def failed_before_start():
        app.operator.transitioning = True
        raise OSError('mock standby shutdown failure')
    app.operator.before_start.side_effect = failed_before_start
    with patch.object(formal.RescueMapApp, 'start_session') as launch:
        app.start_session()
    launch.assert_not_called()
    assert calls == ['session_owners_closed', 'standby_owner_restored']
    assert not app.operator.transitioning and '准备失败' in app.message

    # An asynchronously failed localizer returns to selection later.
    app, calls = make_app()
    app.session_started, app.selecting = True, False
    def failed_async_start(_app):
        _app.session_started, _app.selecting = False, True
    with patch.object(formal.RescueMapApp, 'maybe_start_vision', failed_async_start):
        app.maybe_start_vision()
    assert calls == ['session_owners_closed', 'standby_owner_restored']

    # Never open a standby owner when release of an earlier owner failed.
    app, _calls = make_app()
    app.stop_session_processes.side_effect = RuntimeError('mock owner still running')
    try:
        app._restore_operator_standby()
    except RuntimeError:
        pass
    else:
        raise AssertionError('cleanup failure must remain visible')
    app.operator.after_reset.assert_not_called()
    assert not app.operator.transitioning


def test_warm_start_binds_latest_pose_gyro_and_keeps_sdk_owner():
    """Warm session -> user start reuses processes; no device or motor calls."""
    from unittest.mock import Mock, patch
    from types import SimpleNamespace
    import competition_map_app as formal
    import operator_panel as panel_module
    import map_app
    from lidar_yaw_debug.dynamic import DynamicFence
    from field_model import initial_pose
    with tempfile.TemporaryDirectory() as directory:
        runtime = Path(directory)
        options = SimpleNamespace(zone=1,side='red',corner_offset_mm=150*2**.5,
            encoder_weight=0.,localization_mode='t265',localization_json=runtime/'pose.json',
            localization_log=None,launch_localization=True,launch_vision=True,
            uart='MOCK',baud=230400,tx_rate=20.,fullscreen=False,demo=False,screenshot=None,
            official_wheel_input=True,dynamic_localization=True,gpio_buttons=True,headless=True)
        with patch.object(map_app,'RUNTIME',runtime), patch.object(formal,'RUNTIME',runtime), \
             patch.object(panel_module,'RUNTIME',runtime), patch.object(panel_module,'send_beep',return_value=True):
            app = formal.CompetitionMapApp(options)
            owner = Mock(); owner.poll.return_value = None
            vision = Mock(); vision.poll.return_value = None
            dynamic = SimpleNamespace(radar=SimpleNamespace(state={}),state={},request_start=Mock())
            def cold_prepare(_):
                app.session_started = True
                app.localization_process = owner
                app.dynamic = dynamic
            with patch.object(formal.RescueMapApp,'start_session',cold_prepare), \
                 patch.object(app,'start_owned_process',return_value=vision) as spawn:
                app.prepare_warm_session()
                assert app.warming and app.selecting and not app.session_started
                assert '--wait-for-operator' in spawn.call_args.args[0]
                token = app.reset_session_token
                app.zone,app.side = 4,'blue'
                app.start_session()  # Press before ready is remembered.
                app._update_warm_start()
                assert app.start_requested and not app.start_released
                dynamic.request_start.assert_not_called()
                assert not app.operator.control_path.exists()
                app.warm_status_path.write_text(json.dumps(dict(session_token=token,ready=True,competition_started=False)))
                dynamic.radar.state.update(have_tracked_pose=True,have_primary_yaw=True)
                app._update_warm_start()
                assert app.start_released and spawn.call_count == 1 and app.localization_process is owner
                request=json.loads(app.operator.control_path.read_text())
                assert request['side']=='blue' and request['start_zone']==4 and request['reference_token']==token
                dynamic.request_start.assert_called_once()
                assert runner_module.operator_start_request(app.operator.control_path,app.localization_json,token) is None
                app.localization_json.write_text(json.dumps(dict(reference=dict(round_token='old'))))
                assert runner_module.operator_start_request(app.operator.control_path,app.localization_json,token) is None
                app.localization_json.write_text(json.dumps(dict(reference=dict(round_token=token))))
                assert runner_module.operator_start_request(app.operator.control_path,app.localization_json,token)['start_zone']==4
                app.warm_status_path.write_text(json.dumps(dict(session_token=token,ready=True,competition_started=True)))
                app._update_warm_start()
                assert app.session_started and not app.warming and not app.selecting
                assert spawn.call_count==1 and app.localization_process is owner
            # Exactly the existing transform, sampled after carrying/turning.
            engine=DynamicFence({})
            for seq,x,y,yaw in ((1,0.,0.,0.),(2,2.,-1.,.7)):
                engine.ingest_pose(dict(session_id='sdk',frame_number=seq,timestamp_monotonic_ns=seq*100,
                    receive_monotonic_ns=seq*100,x_m=x,y_m=y,yaw_rad=yaw,tracker_confidence=3))
                engine.ingest_yaw(seq*100,10.+seq*20,dict(raw={}))
            selected=initial_pose(4,app.corner_offset_m)
            ref=engine.align('button',center_xy=(selected.x_m,selected.y_m),heading_deg=selected.yaw_deg,round_token=token)
            assert ref['pose_origin']==[2.,-1.] and ref['reference_sample_ns']==200
            result=engine.trajectory(np.array(engine.poses),np.array(engine.yaws),ref)([200])[0]
            assert np.allclose(result,[selected.x_m,selected.y_m,np.deg2rad(selected.yaw_deg)])
            assert ref['round_token']==token and ref['source']=='selected_competition_start_at_button'


def test_warm_start_stop_wins_before_release_and_failure_does_not_retry():
    from unittest.mock import Mock
    from types import SimpleNamespace
    import competition_map_app as formal
    app=formal.CompetitionMapApp.__new__(formal.CompetitionMapApp)
    app.localization_process=Mock();app.localization_process.poll.return_value=None
    app.vision_process=Mock();app.vision_process.poll.return_value=None
    app.warming=True;app.session_started=False;app.selecting=False
    app.start_requested=True;app.start_released=False;app.warm_ready_announced=False
    app.warm_status_path=Path('/nonexistent/mock-warm-status')
    app.reset_session_token='mock'
    app.dynamic=SimpleNamespace(radar=SimpleNamespace(state={}),request_start=Mock())
    app.operator=SimpleNamespace(stopped=True,_beep=Mock(),_start_standby=Mock())
    app.record_reset_event=Mock();app.stop_session_processes=Mock()
    app._update_warm_start()
    assert not app.start_requested and not app.start_released
    app.dynamic.request_start.assert_not_called()
    app.operator.stopped=False
    app.localization_process.poll.return_value=1;app.localization_process.returncode=1
    app._update_warm_start()
    assert not app.warming and app.selecting and not app.start_requested
    app.stop_session_processes.assert_called_once()
    app.operator._start_standby.assert_called_once()
    app.operator._beep.assert_called_once_with(3,long=True)



def test_warm_config_partial_start_remembers_abort_obligation():
    from unittest.mock import patch
    sent=[];facts=[]
    with patch.object(runner_module,'write_command_frame',side_effect=lambda p,frame:sent.append(frame)), \
         patch.object(runner_module.time,'sleep',return_value=None):
        assert not runner_module.publish_start_config(Path('/unused'), 'blue', 4,
            lambda:len(sent)==0,lambda:facts.append('config_sent'))
        assert len(sent)==1 and facts==['config_sent']
        assert sent[0][2]==0x11  # Real lower CONFIG, not a prepare/handshake frame.
        sent.clear();facts.clear()
        assert not runner_module.publish_start_config(Path('/unused'),'red',1,lambda:False,lambda:facts.append(1))
        assert not sent and not facts
        assert runner_module.publish_start_config(Path('/unused'),'red',1,lambda:True,lambda:facts.append(1))
        assert len(sent)==3 and len(facts)==3



def test_warm_error_beep_waits_only_for_local_socket_binding():
    from unittest.mock import Mock, patch
    from types import SimpleNamespace
    import operator_panel as panel_module
    app=SimpleNamespace(options=SimpleNamespace(zone=1,side='red'),zone=1,side='red',corner_offset_m=.2,
        record_reset_event=Mock())
    with tempfile.TemporaryDirectory() as directory, patch.object(panel_module,'RUNTIME',Path(directory)), \
         patch.object(panel_module,'send_beep',side_effect=[False,True,True]) as send:
        panel=panel_module.OperatorPanel(app)
        panel._beep(3,long=True)
        assert panel._pending_beep is not None
        panel.tick()
        assert panel._pending_beep is None and send.call_count==2
        panel.tick()
        assert send.call_count==2  # No periodic UART resend/ACK polling.
        panel._beep(1)
        assert send.call_count==3 and panel._pending_beep is None
