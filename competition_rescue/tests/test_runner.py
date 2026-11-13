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


def test_claw_isolated_singleton_uses_existing_geometry_without_new_wire_fields() -> None:
    from dataclasses import replace
    from types import SimpleNamespace
    from unittest.mock import patch
    from capture_roi import CaptureRois, split_capture_roi
    from state_machine import TrackedCargo, CargoBatch
    polygon = ((100, 100), (1200, 100), (1200, 950), (100, 950))
    rois = CaptureRois(polygon, *split_capture_roi(polygon))
    mission = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    mission._select_batch(CargoBatch((1,), ("green_supply",), "material"))
    items = (
        TrackedCargo(1, "green_supply", .9, (400, 500, 100, 100), hits=3),
        TrackedCargo(2, "green_supply", .9, (450, 500, 100, 100), hits=3),
        TrackedCargo(3, "green_supply", .9, (500, 500, 100, 100), hits=3),
        TrackedCargo(4, "green_supply", .9, (900, 500, 80, 100), hits=3),
    )
    status = StmSnapshot(mode=38, flags=1, age_ms=0, camera_pitch_cdeg=14000)
    for single_bbox, expected in (((900, 500, 80, 100), "right"),
                                  ((610, 500, 80, 100), "left")):
        current = items[:-1] + (replace(items[-1], bbox=single_bbox),)
        with patch.object(runner_module, "tracked_cargo", return_value=current):
            view = runner_module.make_vision_snapshot([], [], SimpleNamespace(calibrated=False), None,
                status, mission, 1, capture_rois=rois, observed_s=1.0)
        audit = view.capture_audit
        assert view.capture_observation_valid and audit.left_count == 3 and audit.right_count == 1
        assert audit.right_isolated_green == (expected == "right")
        assert mission._preferred_keep_side(audit) == expected
        without_image_hint = replace(audit, left_isolated_green=False, right_isolated_green=False)
        assert mission._audit_payload(audit) == mission._audit_payload(without_image_hint)
    # A large adjacent detection must not be declared independent from its centre distance alone.
    overlapping = (items[0], replace(items[-1], bbox=(350, 480, 650, 160)))
    with patch.object(runner_module, "tracked_cargo", return_value=overlapping):
        view = runner_module.make_vision_snapshot([], [], SimpleNamespace(calibrated=False), None,
            status, mission, 2, capture_rois=rois, observed_s=1.1)
    assert not view.capture_audit.left_isolated_green and not view.capture_audit.right_isolated_green


def test_corridor_missed_obstacle_and_same_class_floor_object_are_not_load():
    from dataclasses import replace
    from types import SimpleNamespace
    from unittest.mock import patch
    from capture_roi import CaptureRois, split_capture_roi
    from state_machine import CargoBatch, TrackedCargo
    m = CompetitionMission(CompetitionSettings(side='red',initial_stash_enabled=False))
    m._select_batch(CargoBatch((229,), ('green_supply',), 'material'))
    m.confirmed_delivery_destination = 'material'
    m.carried_manifest = ('green_supply',)
    m.carried_total_count = 1
    m.state = CompetitionState.SAFE_ZONE_CORRIDOR_CHECK
    carried = TrackedCargo(229,'green_supply',.84,(458,366,180,199),hits=278)
    missed = TrackedCargo(258,'green_supply',.54,(308,566,258,239),hits=1,misses=5,visible=False)
    polygon=((100,100),(1100,100),(1100,900),(100,900))
    rois=CaptureRois(polygon,*split_capture_roi(polygon))
    def snapshot(items, **kwargs):
        with patch.object(runner_module,'tracked_cargo',return_value=items):
            return runner_module.make_vision_snapshot([],[],SimpleNamespace(calibrated=False),
                (261,0,835,107),StmSnapshot(mode=11,age_ms=0,camera_pitch_cdeg=12000),
                m,2851,capture_rois=rois,recent_carried_track_ids=(229,),**kwargs)
    view=snapshot((carried,missed))
    assert view.safe_corridor_obstacle_track_id==258 and view.safe_corridor_status=='BLOCKED'
    # A dropped tracker item must not be reinserted from an older pixel snapshot.
    view=snapshot((carried,),recent_corridor_obstacle=missed)
    assert view.safe_corridor_obstacle_track_id is None
    floor=replace(missed,track_id=259,visible=True,misses=0)
    view=snapshot((floor,))
    assert not view.delivery_target_found
    assert view.safe_corridor_obstacle_track_id==259
    # Same height alone must never identify cargo: disjoint 2D boxes differ.
    other=replace(floor,bbox=(510,770,90,100))
    poly,obstacle,_,_=runner_module.safe_zone_push_corridor((other,), (261,0,835,107),
        'material',False,frozenset(),150,('green_supply',),1,((690,820,90,100),),
        claw_polygon=rois.overall)
    assert obstacle is other


def test_carried_identity_survives_verified_pitch_change_without_waiting():
    from dataclasses import replace
    from types import SimpleNamespace
    from unittest.mock import patch
    from capture_roi import CaptureRois, split_capture_roi
    from state_machine import CargoBatch, TrackedCargo

    m = CompetitionMission(CompetitionSettings(side='red', initial_stash_enabled=False))
    m._select_batch(CargoBatch((68,), ('green_supply',), 'material'))
    m.confirmed_delivery_destination = 'material'
    m.carried_manifest = ('green_supply',)
    m.carried_total_count = 1
    polygon = ((250, 225), (1000, 225), (1000, 715), (250, 715))
    rois = CaptureRois(polygon, *split_capture_roi(polygon))
    old = TrackedCargo(68, 'green_supply', .60, (372, 268, 247, 236), hits=316)
    new = TrackedCargo(75, 'green_supply', .41, (675, 549, 254, 247), hits=10)
    safe_bbox = (543, 0, 735, 131)

    def snapshot(items, pitch, *, ids=(), boxes=(), anchor_pitch=0, remembered=None, flags=2):
        with patch.object(runner_module, 'tracked_cargo', return_value=items):
            return runner_module.make_vision_snapshot([], [], SimpleNamespace(calibrated=False),
                safe_bbox, StmSnapshot(mode=11, flags=flags, age_ms=0, camera_pitch_cdeg=pitch),
                m, 1631, capture_rois=rois, recent_carried_track_ids=ids,
                recent_carried_bboxes=boxes, recent_carried_pitch_cdeg=anchor_pitch,
                recent_corridor_obstacle=remembered)

    m.state = CompetitionState.TRANSPORT_AUDIT
    audited = snapshot((old,), 14000, flags=3)
    assert audited.carried_reference_track_ids == (68,)
    assert audited.carried_reference_bboxes == (old.bbox,)
    m.state = CompetitionState.ALIGN_SAFE_ZONE_BY_POSE
    # Actual frame1617: the reported angle changes before the image. Keep the
    # original 140-degree anchor; no timer or additional frame-count wait.
    transition = snapshot((old,), 12000, ids=(68,), boxes=(old.bbox,), anchor_pitch=14000)
    assert transition.carried_reference_track_ids == (68,)
    assert transition.carried_reference_bboxes == ()
    m.state = CompetitionState.ACQUIRE_SAFE_ZONE
    rebound = snapshot((new,), 12000, ids=(68,), boxes=(old.bbox,), anchor_pitch=14000,
                       remembered=replace(new, visible=False, misses=1))
    assert rebound.carried_reference_track_ids == (75,)
    assert rebound.carried_reference_bboxes == (new.bbox,)
    assert rebound.delivery_target_found
    assert rebound.safe_corridor_obstacle_track_id is None
    # The original loop's no-obstacle branch now clears this false remembered
    # obstacle; even a later missed detection cannot turn known cargo into one.
    missed = snapshot((), 12000, ids=(75,), boxes=(new.bbox,), anchor_pitch=12000,
                      remembered=replace(new, visible=False, misses=1))
    assert missed.safe_corridor_obstacle_track_id is None

    # Bottom857 also enters the old819 near-field overlap fallback: disabling
    # new cross-pitch association alone must not hide a released/new floor ID.
    replacement = replace(new, track_id=76, bbox=(675, 610, 254, 247))
    m.state = CompetitionState.CLEAR_SAFE_ZONE
    released = snapshot((replacement,), 12000, ids=(75,), boxes=(new.bbox,), anchor_pitch=12000)
    assert released.carried_reference_track_ids == (75,)
    assert released.carried_reference_bboxes == ()
    assert released.safe_corridor_obstacle_track_id is None  # Retired sweep has no delivery corridor.
    m.state = CompetitionState.ACQUIRE_SAFE_ZONE
    opened = snapshot((replacement,), 12000, ids=(75,), boxes=(new.bbox,),
                      anchor_pitch=12000, flags=0)
    assert opened.carried_reference_track_ids == (75,)
    assert opened.safe_corridor_obstacle_track_id is None  # Still before alignment completes.
    # Unsupported views, a spatially disjoint same-angle box and an opened
    # gripper cannot turn a same-class replacement into carried cargo.
    for pitch, anchor_pitch, flags in ((11000, 14000, 2), (12000, 12000, 2), (12000, 14000, 0)):
        unchanged = snapshot((replacement,), pitch, ids=(68,), boxes=(old.bbox,),
                             anchor_pitch=anchor_pitch, flags=flags)
        assert unchanged.carried_reference_track_ids == (68,)


def test_pitch_carried_reassociation_preserves_floor_obstacles_and_ambiguity():
    from types import SimpleNamespace
    from unittest.mock import patch
    from capture_roi import CaptureRois, split_capture_roi
    from state_machine import CargoBatch, TrackedCargo

    m = CompetitionMission(CompetitionSettings(side='red', initial_stash_enabled=False))
    m._select_batch(CargoBatch((68,), ('green_supply',), 'material'))
    m.confirmed_delivery_destination = 'material'
    m.carried_manifest = ('green_supply',)
    m.carried_total_count = 1
    m.state = CompetitionState.SAFE_ZONE_CORRIDOR_CHECK
    polygon = ((250, 225), (1000, 225), (1000, 715), (250, 715))
    rois = CaptureRois(polygon, *split_capture_roi(polygon))
    old_bbox = (372, 268, 247, 236)
    held = TrackedCargo(75, 'green_supply', .41, (675, 549, 254, 247), hits=10)
    floor = TrackedCargo(76, 'green_supply', .8, (580, 250, 100, 130), hits=3)

    def snapshot(items, *, ids=(68,), boxes=(old_bbox,), safe_bbox=(543, 0, 735, 131)):
        with patch.object(runner_module, 'tracked_cargo', return_value=items):
            return runner_module.make_vision_snapshot([], [], SimpleNamespace(calibrated=False),
                safe_bbox, StmSnapshot(mode=11, flags=2, age_ms=0, camera_pitch_cdeg=12000),
                m, 1631, capture_rois=rois, recent_carried_track_ids=ids,
                recent_carried_bboxes=boxes, recent_carried_pitch_cdeg=14000)

    extra = snapshot((held, floor))
    assert extra.carried_reference_track_ids == (75,)
    assert extra.safe_corridor_obstacle_track_id == 76
    possible_second_load = TrackedCargo(77, 'green_supply', .7, (440, 548, 245, 248), hits=3)
    ambiguous = snapshot((held, possible_second_load))
    assert ambiguous.carried_reference_track_ids == (68,)
    assert ambiguous.safe_corridor_obstacle_track_id is None  # Both feet are behind this template end715.
    assert not ambiguous.delivery_target_found
    # Existing ground-object regression: same foot height inside the broad
    # platform extent is insufficient. Its box is incompatible with ID229's
    # actual audited box under the existing bidirectional 0.60 overlap rule.
    ground = TrackedCargo(259, 'green_supply', .54, (308, 566, 258, 239), hits=1)
    unchanged = snapshot((ground,), ids=(229,), boxes=((458, 366, 180, 199),),
                         safe_bbox=(261, 0, 835, 107))
    assert unchanged.carried_reference_track_ids == (229,)
    assert unchanged.safe_corridor_obstacle_track_id is None  # Foot805 is outside the fixed end715.
    missing_anchor = snapshot((held,), boxes=())
    assert missing_anchor.carried_reference_track_ids == (68,)


def test_same_view_carried_identity_survives_initial_entry_tracker_replacement():
    from dataclasses import replace
    from types import SimpleNamespace
    from unittest.mock import patch
    from capture_roi import CaptureRois, split_capture_roi
    from state_machine import CargoBatch, TrackedCargo

    m = CompetitionMission(CompetitionSettings(side='red', initial_stash_enabled=False))
    m.first_common_delivered = True
    m._select_batch(CargoBatch((819,), ('injured_orange',), 'injury'))
    m.confirmed_delivery_destination = 'injury'
    m.carried_manifest = ('injured_orange',)
    m.carried_total_count = 1
    m.state = CompetitionState.ENTER_SAFE_ZONE
    polygon = ((250, 225), (1000, 225), (1000, 715), (250, 715))
    rois = CaptureRois(polygon, *split_capture_roi(polygon))
    # Actual clip10: verified 120-degree ID819/frame9065, then ID834/frame9093.
    old_box = (491, 488, 463, 309)
    held = TrackedCargo(834, 'injured_orange', .34, (466, 470, 442, 330), hits=1)
    inside = TrackedCargo(832, 'injured_orange', .8, (566, 0, 150, 66),
                          hits=20, inside_safe_zone=True)

    def snapshot(items, *, flags=2, mode=11, ids=(819,), boxes=(old_box,)):
        with patch.object(runner_module, 'tracked_cargo', return_value=items):
            return runner_module.make_vision_snapshot([], [], SimpleNamespace(calibrated=False),
                (40, 0, 916, 124), StmSnapshot(mode=mode, flags=flags, age_ms=0,
                    camera_pitch_cdeg=12000), m, 9093, capture_rois=rois,
                recent_carried_track_ids=ids, recent_carried_bboxes=boxes,
                recent_carried_pitch_cdeg=12000,
                recent_corridor_obstacle=replace(held, visible=False, misses=1))

    rebound = snapshot((inside, held))
    assert rebound.carried_reference_track_ids == (834,)
    assert rebound.carried_reference_bboxes == (held.bbox,)
    assert rebound.delivery_target_found
    assert rebound.safe_corridor_obstacle_track_id is None
    # Known visible cargo takes precedence, while a different ground object
    # of the same class must still be found in the actual forward corridor.
    floor = replace(held, track_id=835, bbox=(580, 250, 100, 130))
    external = snapshot((held, floor), ids=(834,), boxes=(held.bbox,))
    assert external.carried_reference_track_ids == (834,)
    assert external.safe_corridor_obstacle_track_id == 835
    possible_second_load = replace(held, track_id=836, bbox=(500, 480, 440, 320))
    ambiguous = snapshot((held, possible_second_load))
    assert ambiguous.carried_reference_track_ids == (819,)
    assert ambiguous.safe_corridor_obstacle_track_id is None  # Feet800 are behind template end715.
    # The old pitch-change shape-only rule would accept this similar box.
    # At the same angle its actual bidirectional overlap is below 0.60.
    disjoint = replace(held, track_id=837, bbox=(300, 470, 442, 330))
    assert snapshot((disjoint,)).carried_reference_track_ids == (819,)
    # Bottom861 also enters the original819 near-field fallback: releasing
    # ownership must disable that fallback as well as explicit ID re-binding.
    released = replace(held, bbox=(466, 531, 442, 330))
    opened = snapshot((released,), flags=0)
    assert opened.carried_reference_track_ids == (819,)
    assert opened.safe_corridor_obstacle_track_id is None  # Foot861 is outside the new rectangle.
    ram = snapshot((released,), mode=15)
    assert ram.carried_reference_track_ids == (819,)
    assert ram.safe_corridor_obstacle_track_id is None
    for state in (CompetitionState.CLEAR_SAFE_ZONE, CompetitionState.DELIVERY_VERIFY,
                  CompetitionState.INVALID_RELEASE, CompetitionState.INITIAL_RELEASE):
        m.state = state
        stopped_ownership = snapshot((released,))
        assert stopped_ownership.carried_reference_track_ids == (819,)
        assert stopped_ownership.safe_corridor_obstacle_track_id is None  # Not an entry-check phase.
    m.state = CompetitionState.DELIVERY_VERIFY
    known = snapshot((released,), ids=(834,), boxes=(released.bbox,))
    assert known.carried_reference_track_ids == (834,)
    assert known.delivery_target_found


def test_115_degree_carried_view_uses_actual_platform_and_keeps_external_obstacle():
    from dataclasses import replace
    from types import SimpleNamespace
    from unittest.mock import patch
    from capture_roi import CaptureRois, split_capture_roi
    from state_machine import CargoBatch, TrackedCargo

    m = CompetitionMission(CompetitionSettings(side='red', initial_stash_enabled=False))
    m._select_batch(CargoBatch((136,), ('green_supply',), 'material'))
    m.confirmed_delivery_destination = 'material'
    m.carried_manifest = ('green_supply',)
    m.carried_total_count = 1
    m.state = CompetitionState.ACQUIRE_SAFE_ZONE
    polygon = ((250, 225), (1000, 225), (1000, 715), (250, 715))
    rois = CaptureRois(polygon, *split_capture_roi(polygon))
    # Actual clip1: frame1729 reports115 while still showing the old view;
    # frame1732 has the raised image, held136 and separate right-floor147.
    old = TrackedCargo(136, 'green_supply', .7, (366, 529, 250, 257), hits=164)
    held = replace(old, bbox=(361, 598, 252, 265))
    floor = TrackedCargo(147, 'injured_orange', .56, (876, 502, 402, 457), hits=3)

    def snapshot(items, *, ids=(136,), boxes=(old.bbox,), anchor_pitch=12000):
        with patch.object(runner_module, 'tracked_cargo', return_value=items):
            return runner_module.make_vision_snapshot([], [], SimpleNamespace(calibrated=False),
                (301, 23, 795, 154), StmSnapshot(mode=11, flags=2, age_ms=0,
                    camera_pitch_cdeg=11500), m, 1732, capture_rois=rois,
                recent_carried_track_ids=ids, recent_carried_bboxes=boxes,
                recent_carried_pitch_cdeg=anchor_pitch)

    transition = snapshot((old,))
    assert transition.carried_reference_track_ids == (136,)
    assert transition.carried_reference_bboxes == ()
    actual = snapshot((held, floor))
    assert actual.carried_reference_track_ids == (136,)
    assert actual.carried_reference_bboxes == (held.bbox,)
    assert not runner_module.cargo_on_carrying_platform(floor, 11500)
    assert floor in actual.cargo
    assert floor.track_id not in actual.carried_reference_track_ids
    # This actual object is at the right side, outside the material corridor;
    # retaining it as ground cargo must not invent a blocked forward path.
    assert actual.safe_corridor_obstacle_track_id is None
    # If tracking changes during the lift, the manifest and compatible old
    # box identify the load; then same-view replacement uses actual overlap.
    changed = replace(held, track_id=148)
    assert snapshot((changed, floor)).carried_reference_track_ids == (148,)
    same_view = snapshot((replace(changed, track_id=149, bbox=(363, 601, 254, 262)),),
                         ids=(148,), boxes=(changed.bbox,), anchor_pitch=11500)
    assert same_view.carried_reference_track_ids == (149,)


def test_held_injury_with_high_roi_overlap_does_not_require_projected_foot_inside():
    from types import SimpleNamespace
    from unittest.mock import patch
    from capture_roi import CaptureRois, split_capture_roi
    from state_machine import CargoBatch, TrackedCargo
    m=CompetitionMission(CompetitionSettings(side='red',initial_stash_enabled=False))
    m.first_common_delivered=True
    m._select_batch(CargoBatch((1,),('injured_orange',),'injury'))
    polygon=((100,100),(600,100),(600,500),(100,500))
    rois=CaptureRois(polygon,*split_capture_roi(polygon))
    held=TrackedCargo(1,'injured_orange',.9,(350,300,100,230),hits=3)
    with patch.object(runner_module,'tracked_cargo',return_value=(held,)):
        view=runner_module.make_vision_snapshot([],[],SimpleNamespace(calibrated=False),None,
            StmSnapshot(mode=23,flags=3,age_ms=0,camera_pitch_cdeg=14000),m,1,capture_rois=rois)
    assert view.capture_observation_valid and view.capture_audit.total_count==1
    assert view.capture_audit.left_class=='injured_orange'


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
        (10500, (0,150,291,432,603,1024), (500,500,470,260,140)),
        (12000, (0,73,217,381,580,1024), (460,380,230,100,20)),
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
        # The real mode52 deadlock had new images but a frozen wire frame.
        # Exercise the actual publisher and binary context, keeping the same
        # pixel action while correcting a target just outside the tolerance.
        off_center = replace(target, bbox=(543, 500, 40, 40))
        for frame, now in ((11, 1.1), (12, 1.2)):
            correction_view = replace(view, frame_sequence=frame,
                observed_monotonic_s=now, cargo=(off_center,))
            correction = planner._output_for_cycle(correction_view, PoseSnapshot(), status, False, now)
            planner._publish(correction)
            assert correction.reason == "approach_range_realign"
            assert not correction.command.flags & CMD_APPROACH_RANGE
            assert planner.protocol_action == pixel_action
            assert int.from_bytes(planner.command_path.read_bytes()[8:12], "big") == frame
        view = replace(view, frame_sequence=13, observed_monotonic_s=1.3)
        request = planner._output_for_cycle(view, PoseSnapshot(), status, False, 1.3)
        planner._publish(request)
        action = planner.protocol_action
        assert action != pixel_action and request.command.flags & CMD_APPROACH_RANGE
        packet = planner.command_path.read_bytes()
        assert len(packet) == 30 and packet[3] == packet[18]
        assert int.from_bytes(packet[8:12], "big") == 13
        for mode in (52,53):
            later = planner._output_for_cycle(replace(view, frame_sequence=14, range_distances=((1,100),)),
                                              PoseSnapshot(), replace(status, mode=mode), False, 1.4)
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


def test_opening_claw_probe_keeps_context_and_consumes_completed_empty_exit():
    from dataclasses import replace
    from protocol import CMD_CARGO_AUDIT
    from state_machine import TrackedCargo
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        m = CompetitionMission(CompetitionSettings(side="blue"))
        p = CompetitionPlanner(m, root/"pose", root/"stm", root/"cmd",
            root/"diag", JsonlLog(root/"events"), 50)
        pose = PoseSnapshot(True, 0., 0., 135., 5.)
        search = StmSnapshot(mode=3, camera_pitch_cdeg=10500, age_ms=5)
        view = VisionSnapshot(frame_sequence=10)
        p._publish(p._output_for_cycle(view, pose, search, False, 1.))
        start_ack = replace(search, context_valid=True, task_id=p.protocol_task,
            action_id=p.protocol_action, accepted_command=CMD_HOLD, action_status=1)
        out = p._output_for_cycle(view, pose, start_ack, False, 1.02)
        p._publish(out)
        task, action = p.protocol_task, p.protocol_action
        wire = (root/"cmd").read_bytes()
        assert wire[15+4] == CMD_CARGO_AUDIT and wire[15+9] == 1
        assert wire[8:12] == b"\0\0\0\0"  # No camera result manufactured for the request.
        # A delayed entry ACK must keep the same request, not send PAUSE or APP.
        settling = StmSnapshot(mode=20, camera_pitch_cdeg=12500, age_ms=5,
            context_valid=True, task_id=task, action_id=(action-1)&65535,
            accepted_command=CMD_HOLD, action_status=1)
        out = p._output_for_cycle(replace(view, frame_sequence=11), pose, settling, False, 1.5)
        p._publish(out)
        assert (p.protocol_task, p.protocol_action) == (task, action)
        assert out.command.opcode == CMD_CARGO_AUDIT and out.command.audit_vision_frame == 0
        # If mode21 was not observed, the existing paired completed status is
        # still enough to consume the MCU's empty-view return to SEARCH.
        completed = replace(search, context_valid=True, task_id=task, action_id=action,
            accepted_command=CMD_CARGO_AUDIT, action_status=3)
        out = p._output_for_cycle(replace(view, frame_sequence=12), pose, completed, False, 2.)
        p._publish(out)
        assert out.state == CompetitionState.INITIAL_OBSERVE
        target = TrackedCargo(9, "green_supply", .9, (600,500,40,40), hits=3)
        out = p._output_for_cycle(VisionSnapshot(frame_sequence=13,
            observed_monotonic_s=2.1, cargo=(target,)), pose, completed, False, 2.1)
        p._publish(out)
        assert out.command.opcode == CMD_APPROACH_TARGET
        assert p.protocol_task != task


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


def test_transport_duplicate_rejection_preserves_audit_and_real_reject_recovers():
    from dataclasses import replace
    from protocol import CMD_CARGO_AUDIT, CMD_NAVIGATE_WAYPOINT
    from state_machine import CargoAudit, CargoBatch

    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
        m._select_batch(CargoBatch((1,), ("green_supply",), "material"))
        audit = CargoAudit(left_class="green_supply", left_count=1,
                           left_green_count=1, total_count=1)
        m._latch_carried_manifest(audit)
        x, y = m._target_point()
        pose = PoseSnapshot(True, x, y, 90.0, 0.0)
        p = CompetitionPlanner(m, root/"pose", root/"stm", root/"cmd",
                               root/"diag", JsonlLog(root/"events"), 50)
        status = StmSnapshot(mode=48, flags=3, age_ms=0, camera_pitch_cdeg=14000)
        m._begin_transport_audit(VisionSnapshot(frame_sequence=1), pose, status, 1.0)
        for frame in (2, 3):
            view = VisionSnapshot(frame_sequence=frame, observed_monotonic_s=1 + frame * .13,
                                  capture_audit=audit)
            p._publish(m._transport_audit_output(view, pose, status, 1 + frame * .13))
        epoch, task, action = m.audit_epoch, p.protocol_task, p.protocol_action
        accepted = replace(status, context_valid=True, task_id=task, action_id=action,
                           accepted_command=CMD_CARGO_AUDIT, action_status=1|4,
                           rejected_action_id=action)
        for frame in (4, 5):
            view = replace(view, frame_sequence=frame, observed_monotonic_s=1 + frame * .13)
            out = p._output_for_cycle(view, pose, accepted, False, 1 + frame * .13)
            assert out.event != "transport_audit_request_rejected"
            assert m.audit_epoch == epoch and m.audit_hits >= 2 and out.command.audit.stable
            p._publish(out)
            assert (p.protocol_task, p.protocol_action) == (task, action)

        # A real, unaccepted request must still re-observe under a new action.
        rejected = replace(accepted, action_id=action - 1, action_status=1|4,
                           accepted_command=CMD_NAVIGATE_WAYPOINT)
        wrong_task = replace(rejected, task_id=task + 1)
        assert p._reconcile_transport_rejection(view, pose, wrong_task, 1.8) is None
        recovered = p._output_for_cycle(view, pose, rejected, False, 1.8)
        assert recovered.event == "transport_audit_request_rejected"
        assert m.audit_epoch == epoch + 1 and not recovered.command.audit.stable
        p._publish(recovered)
        assert p.protocol_action != action
        next_view = replace(view, frame_sequence=6, observed_monotonic_s=1.91)
        resumed = p._output_for_cycle(next_view, pose, rejected, False, 1.91)
        assert resumed.event != "transport_audit_request_rejected"
        assert m.audit_epoch == epoch + 1
        p._publish(resumed)
        assert m.audit_hits == 1  # New visual evidence survives the old rejection.
        original = p.protocol_last_output
        p.protocol_last_output = CompetitionOutput(m.state, CommandRequest(CMD_HOLD), "overlay")
        current_reject = replace(rejected, rejected_action_id=p.protocol_action)
        assert p._reconcile_transport_rejection(next_view, pose, current_reject, 1.92) is None
        p.protocol_last_output = original


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
    from lidar_yaw_debug.latest_snapshot import LatestSnapshot

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
    control=LatestSnapshot(ctx)
    inbox,display,replies=queue.Queue(),ctx.Queue(maxsize=4),ctx.Queue(maxsize=16)
    master,slave=pty.openpty();entered,release=threading.Event(),threading.Event()
    original=DynamicFence.observe
    def slow_fit(engine,scan,**kwargs):
        entered.set();release.wait(4)
        return original(engine,scan,**kwargs)
    config=json.loads((PROJECT_ROOT/'localization/config/lidar_yaw_debug.json').read_text())
    with tempfile.TemporaryDirectory() as d, patch.object(DynamicFence,'observe',slow_fit):
        worker=threading.Thread(target=run_worker,args=(config,os.ttyname(slave),230400,Path(d),inbox,display,replies,stop,control))
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
            # Do not consume any display snapshot until control has advanced:
            # both blocked fitting and an unattended GUI remain off its path.
            deadline=time.monotonic()+2
            while time.monotonic()<deadline:
                fast=control.get(timeout=.1)
                if fast and fast['last'].get('sample_monotonic_ns')==t:break
            else:raise AssertionError('control publication stalled behind fitting/display')
            assert fast['last']['ready'] and not release.is_set()
            assert 'points_field' not in fast['last'] and 'last_fit' not in fast['last']
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


def test_formal_t265_mount_is_independent_of_wheel_input():
    """All T265 modes use one mount; pure Pose never creates injection owners."""
    from unittest.mock import patch
    from types import SimpleNamespace
    import competition_map_app as formal
    import map_app
    from competition_dynamic_localization import CompetitionDynamicLocalization
    mount = dict(camera_offset_forward_m='-0.0346105',
        camera_offset_left_m='-0.0599471445',
        camera_robot_forward_axis='0.8660254,0,0.5', camera_robot_up_axis='0,-1,0',
        t265_translation_scale_enabled='false', t265_translation_scale='1.000')
    starts = {1: (-1.35, 1.35, 135.), 2: (1.35, 1.35, 45.),
              3: (-1.35, -1.35, 225.), 4: (1.35, -1.35, 315.)}
    modes = [(False, False, zone) for zone in starts] + [(True, False, 2), (False, True, 3)]
    for official, dynamic, zone in modes:
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory)
            options = SimpleNamespace(zone=zone, side='blue', corner_offset_mm=150*2**.5,
                encoder_weight=.25, localization_mode='t265', localization_json=runtime/'pose.json',
                localization_log=None, launch_localization=False, launch_vision=False,
                uart='/dev/ttyS1', baud=921600, tx_rate=20., fullscreen=False, demo=False,
                screenshot=None, official_wheel_input=official, dynamic_localization=dynamic,
                dynamic_config=PROJECT_ROOT/'localization/config/lidar_yaw_debug.json',
                lidar='/dev/ttyS6', lidar_baud=230400)
            with patch.object(map_app, 'RUNTIME', runtime), patch.object(formal, 'RUNTIME', runtime), \
                    patch.object(formal, 'CompetitionWheelInput', wraps=formal.CompetitionWheelInput) as wheel, \
                    patch.object(CompetitionDynamicLocalization, 'start') as start:
                app = formal.CompetitionMapApp(options)
                try:
                    app.start_session()
                    cmd = app.localization_command()
                    config = dict(line.split('#', 1)[0].strip().split('=', 1) for line in
                        Path(cmd[cmd.index('--config')+1]).read_text().splitlines()
                        if '=' in line.split('#', 1)[0])
                    config = {key.strip(): value.strip() for key, value in config.items()}
                    assert all(config[key] == value for key, value in mount.items()), config
                    assert config['start_zone'] == str(zone) and config['start_center_m'] == '1.350000'
                    assert config['start_heading_offset_deg'] == '0.0'
                    assert (app.pose.x_m, app.pose.y_m, app.pose.yaw_deg) == starts[zone]
                    assert '--t265-only-output' in cmd and '--enable-pose-jumping' in cmd
                    assert wheel.call_count == int(official or dynamic)
                    assert start.call_count == int(dynamic)
                    if not (official or dynamic):
                        assert app.wheel_input is None and app.dynamic is None
                        assert app.wheel_run_dir is None
                        assert not any(flag in cmd for flag in ('--debug-wheel-socket',
                            '--debug-wheel-config', '--wheel-feedback-socket', '--motion-feedback-socket',
                            '--debug-pose-socket', '--dynamic-pose-socket'))
                        assert not list(runtime.rglob('wheel_sdk_config.json'))
                finally:
                    app.stop_localization()
            # Explicit legacy fusion mode retains its existing template and tuning.
            options.localization_mode = 'fusion'
            options.official_wheel_input = options.dynamic_localization = False
            app = formal.CompetitionMapApp(options)
            assert not hasattr(app, 'localization_config_template')
            assert app.localization_mode == 'fusion' and app.encoder_weight == .25


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
            uart='/dev/ttyS1', baud=921600, tx_rate=20., fullscreen=False, demo=False,
            screenshot=None, official_wheel_input=True)
        with patch.object(map_app, 'RUNTIME', runtime), patch.object(formal, 'RUNTIME', runtime):
            app = formal.CompetitionMapApp(options)
            try:
                app.start_session()
                cmd = app.localization_command()
                assert cmd.count('--uart') == 1 and '--ignore-encoders' in cmd
                assert cmd[cmd.index('--baud')+1] == '921600'
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


def test_safe_zone_wheel_policy_preserves_phase_and_requires_own_departure():
    import socket
    import uuid
    from protocol import CMD_PAUSE, CMD_RETURN_CENTER
    from state_machine import STM_MODE_EXIT_SAFE_ZONE, STM_MODE_FACE_FIELD_CENTER
    from run_competition_rescue import LocalizationWheelPolicy
    address = '\0test-wheel-policy-' + uuid.uuid4().hex
    receiver = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    receiver.bind(address)
    receiver.settimeout(.03)
    policy = LocalizationWheelPolicy('@' + address[1:], 'fixture-session')

    def update(state, *, mode=0, task=11, valid=False, matched=False, accepted=0, opcode=CMD_RETURN_CENTER):
        output = CompetitionOutput(state, CommandRequest(opcode), 'fixture')
        policy.update(output, StmSnapshot(mode=mode, context_valid=valid,
            task_id=task, context_matches=matched, accepted_command=accepted), 11, 22)

    def read():
        return json.loads(receiver.recv(4096))

    def empty():
        try:
            receiver.recv(4096)
        except socket.timeout:
            return
        raise AssertionError('unchanged phase must not resend')

    try:
        update(CompetitionState.NAVIGATE)
        first = read()
        assert first['suppressed'] is False and first['sequence'] == 0
        update(CompetitionState.INITIAL_STASH_NAV)
        empty()  # High-speed staging/opening routes remain unchanged.
        for state in LocalizationWheelPolicy.SAFE_ZONE_STATES:
            update(state)
        second = read()
        assert second['suppressed'] is True and second['reason'] == 'safe_zone_low_speed'
        assert second['task_id'] == 11 and second['action_id'] == 22
        assert second['session_id'] == 'fixture-session' and second['sequence'] == 1
        empty()
        update(CompetitionState.RETURN_CENTER, mode=STM_MODE_EXIT_SAFE_ZONE, valid=True)
        update(CompetitionState.RECOVERY_WAIT, opcode=CMD_PAUSE)
        update(CompetitionState.FAULT, opcode=CMD_HOLD)
        update(CompetitionState.RETURN_CENTER, mode=STM_MODE_FACE_FIELD_CENTER, task=99, valid=True)
        update(CompetitionState.RETURN_CENTER, mode=STM_MODE_FACE_FIELD_CENTER, valid=True, opcode=CMD_PAUSE)
        empty()
        update(CompetitionState.TASK_COMPLETE, mode=STM_MODE_FACE_FIELD_CENTER, valid=True)
        assert read()['suppressed'] is False
        update(CompetitionState.TASK_COMPLETE, mode=runner_module.STM_MODE_SEARCH, valid=True)
        empty()  # The same completed delivery cannot immediately re-enter suppression.
        update(CompetitionState.ALIGN_SAFE_ZONE_BY_POSE)
        assert read()['suppressed'] is True
        # A skipped mode17 can use the existing accepted return action in SEARCH.
        update(CompetitionState.SEARCH, mode=runner_module.STM_MODE_SEARCH, valid=True,
               matched=True, accepted=CMD_RETURN_CENTER)
        assert read()['suppressed'] is False
        update(CompetitionState.ALIGN_SAFE_ZONE_BY_POSE)
        assert read()['suppressed'] is True
        policy.close()
        update(CompetitionState.RETURN_CENTER, mode=STM_MODE_FACE_FIELD_CENTER, valid=True)
        empty()  # Exit/STOP closes only: never send a resume into the old SDK session.
    finally:
        policy.close()
        receiver.close()


def test_safe_zone_wheel_policy_retry_does_not_block_existing_task_publish():
    import socket
    import uuid
    from run_competition_rescue import LocalizationWheelPolicy
    from protocol import CMD_NAVIGATE_WAYPOINT, CMD_RETURN_CENTER
    address = '\0test-wheel-policy-' + uuid.uuid4().hex
    receiver = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    receiver.settimeout(.2)
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        planner = CompetitionPlanner(CompetitionMission(CompetitionSettings(side='red')),
            root/'pose', root/'stm', root/'command', root/'diagnostic', JsonlLog(root/'events'), 50)
        policy = LocalizationWheelPolicy('@' + address[1:], 'fixture-session')
        planner.localization_wheel_policy = policy
        output = CompetitionOutput(CompetitionState.ALIGN_SAFE_ZONE_BY_POSE,
                                   CommandRequest(CMD_NAVIGATE_WAYPOINT), 'fixture')
        try:
            # No listener yet: policy fails, but the unchanged paired mission frame is written.
            planner._publish(output, stm=StmSnapshot())
            assert policy.error and policy.last_sent is None
            assert len((root/'command').read_bytes()) == 30 and planner.sequence == 1
            receiver.bind(address)
            planner._publish(output, stm=StmSnapshot())
            packet = json.loads(receiver.recv(4096))
            assert packet['suppressed'] is True and packet['sequence'] == 0
            assert packet['task_id'] == planner.protocol_task
            assert policy.error is None and policy.last_sent is True
            assert len((root/'command').read_bytes()) == 30 and planner.sequence == 2
            # Raw loaded STM does not precompute context_matches; use the
            # planner's existing task/action ownership before the next publish.
            returning = CompetitionOutput(CompetitionState.RETURN_CENTER,
                                         CommandRequest(CMD_RETURN_CENTER), 'fixture')
            planner._publish(returning, stm=StmSnapshot(mode=runner_module.STM_MODE_NAVIGATE,
                context_valid=True, task_id=planner.protocol_task,
                action_id=planner.protocol_action, accepted_command=CMD_RETURN_CENTER))
            assert json.loads(receiver.recv(4096))['suppressed'] is False
        finally:
            policy.close()
            receiver.close()


def test_formal_dynamic_start_wiring():
    """Launcher wiring only; mock worker start, no UART/SDK and no runtime files."""
    from unittest.mock import patch
    from types import SimpleNamespace
    import competition_map_app as formal
    import map_app
    from competition_dynamic_localization import CompetitionDynamicLocalization
    import competition_dynamic_localization as transport
    real_start=CompetitionDynamicLocalization.start
    with tempfile.TemporaryDirectory() as directory:
        runtime = Path(directory)
        options = SimpleNamespace(zone=2, side='blue', corner_offset_mm=150*2**.5,
            encoder_weight=.25, localization_mode='t265', localization_json=runtime/'pose.json',
            localization_log=None, launch_localization=False, launch_vision=False,
            uart='/dev/ttyS1', baud=921600, tx_rate=20., fullscreen=False, demo=False,
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
                assert cmd[cmd.index('--baud')+1] == '921600'
                assert cmd[cmd.index('--dynamic-pose-output')+1] == str(runtime/'pose.json')
                assert Path(cmd[cmd.index('--output')+1]).name == 'raw_t265_result.json'
                assert '--debug-pose-socket' in cmd and '--wheel-feedback-socket' in cmd
                assert app.dynamic.native_feedback and app.dynamic.radar.feedback_socket.fileno()>=0
                assert cmd.count('--motion-feedback-socket')==1
                assert cmd[cmd.index('--motion-feedback-socket')+1]=='@'+app.dynamic.radar.feedback_address[1:]
                vision = app.vision_command()
                assert vision.count('--enable-bulldozer') == 1
                assert vision[vision.index('--localization-wheel-policy-socket')+1] == '@'+app.dynamic.wheel_policy_address[1:]
                assert vision[vision.index('--localization-wheel-policy-session')+1] == app.dynamic.session_id
                # Parent diagnostics must not resend the direct raw 07/08 stream.
                with patch.object(app.dynamic.radar,'send',return_value=True) as send:
                    for kind in ('timed_wheel_diagnostic','timed_gyro_diagnostic'):
                        app.dynamic.telemetry(dict(kind=kind,sample={}))
                    send.assert_not_called()
                    for kind in ('timed_identity','timed_clock_sync','device_identity_changed','imu'):
                        event=dict(kind=kind,sample={})
                        app.dynamic.telemetry(event)
                        send.assert_called_with('feedback_context',event)
                    assert send.call_count==4 and app.dynamic.chassis_inbox.empty()
                    app.dynamic.radar.state['native_feedback']=dict(timed_yaw_seen=True)
                    app.dynamic.telemetry(dict(kind='imu',sample={}))
                    assert send.call_count==4
                assert app.wheel_input.telemetry_sink == app.dynamic.telemetry
                assert app.localization_command() == cmd and start.call_count == 1
                metadata = json.loads((runtime/'session.json').read_text())
                assert metadata['pose_source'] == 'lidar_t265_measured_wheel_fallback'
                assert metadata['qualification']['translation_compensated'] is False
                assert metadata['accuracy_verified'] is False
                assert metadata['motion_feedback_transport']=='native_cpp_complete_frames_direct_to_motion_worker'
                from lidar_yaw_debug.control_packet import format_control_packet
                selected=dict(mode='WHEEL_FALLBACK',reason='fixture conflict')
                reference=dict(round_token='fixture-reference')
                state=dict(ready=True,center_xy_m=[.1,.2],yaw_deg=45.,sample_monotonic_ns=123,
                    source='LIDAR_ANCHORED_WHEEL_FALLBACK',motion_selection=selected,
                    t265_center_xy_m=[.3,.4],raw_t265_sample_ns=120,current_wall_axes=['x'],
                    lidar_corrections_enabled=True,position_reacquisitions=2,reference=reference)
                raw=dict(tracker_confidence=0,mapper_confidence=2,travel_from_start_m=.75)
                lidar=dict(lidar_connected=True,lidar_error=None)
                expected=dict(schema_version=1,session_id=app.dynamic.session_id,packet_id=3,
                    frame='field',timestamp_monotonic_ns=123,quality='DEGRADED',
                    pose=dict(x_m=.1,y_m=.2,yaw_deg=45.),pose_source='LIDAR_ANCHORED_WHEEL_FALLBACK',
                    motion_source='WHEEL_FALLBACK',t265=dict(tracker_confidence=0,mapper_confidence=2,
                        travel_from_start_m=.75,translation_scale=1.,translation_scale_enabled=False,
                        raw_center_xy_m=[.3,.4],sample_monotonic_ns=120),
                    wheel=dict(fusion_weight=0.,official_sdk_input=True,independent_reference=False),
                    navigation=dict(distance_compensation_enabled=False,wheel_progress_m=None),
                    qualified_pose=False,accuracy_verified=False,current_wall_axes=['x'],
                    lidar_corrections_enabled=True,lidar_connected=True,lidar_error=None,
                    position_reacquisitions=2,reference=reference,motion_selection=selected)
                assert format_control_packet(state,raw,app.dynamic.session_id,3,lidar_state=lidar)==expected
                app.dynamic.latest_pose=raw;app.dynamic.control_state=lidar;app.dynamic.packet_id=3
                assert app.dynamic.control_packet(state)==expected
                missing=dict(ready=False,reason='fixture reference reset')
                assert format_control_packet(missing,raw,app.dynamic.session_id,4) is None
                assert format_control_packet(missing,raw,app.dynamic.session_id,4,expected)==dict(
                    expected,packet_id=4,quality='LOST',reference_invalid_reason='fixture reference reset')
                assert expected['quality']=='DEGRADED' and expected['packet_id']==3
                # The worker is the only native control publisher. Startup
                # passes its original C++ destination and skips the old chassis thread.
                threads=[]
                def fake_thread(**kwargs):
                    threads.append(kwargs)
                    return SimpleNamespace(start=lambda:None,join=lambda:None)
                def prepare_worker(*args,**kwargs):app.dynamic.radar.scheduling_setup={}
                with patch.object(transport,'threading',SimpleNamespace(Thread=fake_thread)), \
                        patch.object(app.dynamic.radar,'start',side_effect=prepare_worker) as worker_start:
                    real_start(app.dynamic)
                assert worker_start.call_args.kwargs['control_output']==dict(
                    session_id=app.dynamic.session_id,address=app.dynamic.control_address)
                assert 'dynamic-motion-input' not in {entry['name'] for entry in threads}
                last_packet=dict(packet_id=6,quality='DEGRADED',pose=dict(x_m=.1,y_m=.2,yaw_deg=45.))
                value=dict(last=dict(ready=False,reason='fixture'),have_tracked_pose=False,
                    have_primary_yaw=False,direct_output=dict(packets_sent=7,output_drops=2,last_packet=last_packet))
                app.dynamic.radar.state=value  # Existing 20Hz display snapshot; control slot stays idle.
                waits=[0]
                def next_control(timeout):
                    assert timeout==.05
                    waits[0]+=1
                    if waits[0]>1:app.dynamic.stop_event.set()
                    return None
                with patch.object(app.dynamic.radar.control,'get',side_effect=next_control), \
                        patch.object(app.dynamic.radar,'process',SimpleNamespace(exitcode=None)), \
                        patch.object(app.dynamic,'_schedule_thread'), \
                        patch.object(app.dynamic,'control_packet',side_effect=AssertionError('parent formatted native control')), \
                        patch.object(app.dynamic,'output',SimpleNamespace(sendto=lambda *a: (_ for _ in ()).throw(
                            AssertionError('parent sent duplicate native control')))):
                    app.dynamic._run()
                assert app.dynamic.packet_id==7 and app.dynamic.output_drops==2
                assert app.dynamic.last_control is last_packet
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
    mission.state = CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX
    mission._update_safe_zone_visual_target((216, 0, 941, 113), 3)
    reobserve = mission.step(VisionSnapshot(frame_sequence=4), p,
        replace(s, camera_pitch_cdeg=11500), 6.3)
    assert reobserve.event == 'safe_zone_visual_reobserve'
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
                    planner._record_replay_event(reobserve if i == 4 else a,
                        p,s,VisionSnapshot(frame_sequence=i),1.+i*.13)
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
        assert clip['trigger'] == 'safe_zone_visual_reobserve'
        assert [event['event'] for event in clip['events']] == [
            'safe_zone_visual_reobserve', 'safe_zone_acquire_no_box']
        assert clip['events'][0]['command']['arg_b'] == 11500
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
            dynamic = SimpleNamespace(radar=SimpleNamespace(state={}),state={},request_start=Mock(),
                wheel_policy_address='\0competition-wheel-policy-fixture',session_id='fixture')
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


def test_safe_zone_membership_uses_live_contact_not_cached_bbox_area():
    from types import SimpleNamespace
    from state_machine import CargoBatch, cargo_in_front_region
    from rescue_vision.models import TrackState

    def track(track_id, class_name, bbox):
        return SimpleNamespace(track_id=track_id, class_name=class_name, confidence=.96,
            hits=12, misses=0, state=TrackState.CONFIRMED,
            last_detection=SimpleNamespace(bbox=bbox, ground_xy_mm=None))

    # Actual first-run frame14144: 97.5% of orange1160's box is over the
    # old safe-zone colour, but its foot is outside:205 > 40+162.
    safe = (503, 40, 773, 162)
    front = track(1160, "injured_orange", (728, 84, 151, 121))
    assert runner_module.bbox_overlap_ratio(front.last_detection.bbox, safe) > .97
    front_cargo = runner_module.tracked_cargo((front,), safe)[0]
    assert not front_cargo.inside_safe_zone
    polygon = ((460, 0), (1279, 0), (1279, 1023), (460, 1023))
    assert cargo_in_front_region(front_cargo, polygon, safe)

    # Real supported inside objects retain the original per-class area gate;
    # neither the contact boundary nor black's full-containment rule expands.
    inside_green = track(1161, "green_supply", (700, 60, 100, 100))
    inside_black = track(1162, "core_black", (900, 70, 90, 90))
    boundary_green = track(1163, "green_supply", (700, 120, 100, 82))
    partial_black = track(1164, "core_black", (480, 60, 100, 100))
    values = runner_module.tracked_cargo((inside_green, inside_black, boundary_green,
                                         partial_black), safe)
    assert [item.inside_safe_zone for item in values] == [True, True, True, False]
    assert not cargo_in_front_region(values[0], polygon, None)
    assert not cargo_in_front_region(values[1], polygon, None)

    # The very same rectangles are not independent inside evidence when the
    # current safe bbox was not detected and only the old-angle cache remains.
    mission = CompetitionMission(CompetitionSettings(side="blue", initial_stash_enabled=False))
    mission._select_batch(CargoBatch((1160,), ("injured_orange",), "injury"))
    view = runner_module.make_vision_snapshot([], (front, inside_green),
        SimpleNamespace(calibrated=False), safe,
        StmSnapshot(mode=42, flags=12, camera_pitch_cdeg=11700), mission, 14144,
        safe_zone_filter_blocked=True)
    assert view.safe_zone_filter_blocked
    assert not any(item.inside_safe_zone for item in view.cargo)

    # First-run frame5436 contains one green already in the zone and another
    # still on the carrying platform. Its bbox's 88.5% colour overlap alone
    # must not establish first-delivery proof; the actual inside green stays.
    delivery_safe = (53, 203, 1224, 656)
    delivered = track(579, "green_supply", (472, 421, 176, 198))
    held = track(583, "green_supply", (330, 658, 276, 227))
    mission.state = CompetitionState.DELIVERY_VERIFY
    mission._select_batch(CargoBatch((583,), ("green_supply",), "material"))
    mission.carried_manifest = ("green_supply",)
    mission.carried_total_count = 1
    for expected_id, inside in ((583, False), (579, True)):
        delivery = runner_module.make_vision_snapshot([], (delivered, held),
            SimpleNamespace(calibrated=False), delivery_safe,
            StmSnapshot(mode=10, flags=0, camera_pitch_cdeg=11500), mission, 5436,
            recent_carried_track_ids=(expected_id,))
        assert delivery.delivery_target_inside_safe_zone is inside
        assert delivery.delivery_target_outside_safe_zone is not inside



def test_two_material_capture_keeps_full_count_and_existing_wire():
    from collections import Counter
    from state_machine import CargoBatch, TrackedCargo

    items = (TrackedCargo(1, "green_supply", .9, (430, 420, 80, 100), hits=5),
             TrackedCargo(2, "core_black", .9, (510, 420, 80, 100), hits=5))
    audit = runner_module.audit_from_cargo(items, side_by_track={1: "left", 2: "left"})
    assert audit.total_count == audit.left_count == 2 and audit.right_count == 0
    assert audit.left_class == "mixed_material" and not audit.left_invalid
    m = CompetitionMission(CompetitionSettings(side="blue", initial_stash_enabled=False))
    m.first_common_delivered = True
    m._select_batch(CargoBatch((1,), ("green_supply",), "material"))
    assert m._audit_valid(audit)
    frame = m._audit_payload(audit).frame(7)
    assert frame[6] == 6 and frame[7] == 0 and frame[8] == 2 and frame[11] == 2
    m._latch_carried_manifest(audit)
    assert m.carried_total_count == m.selected_batch.total_count == 2
    assert Counter(m.carried_manifest) == Counter({"green_supply": 1, "core_black": 1})


def test_two_material_delivery_confirmation_requires_both_known_items():
    from types import SimpleNamespace
    from state_machine import CargoBatch
    from rescue_vision.models import TrackState

    def track(track_id, name, bbox):
        return SimpleNamespace(track_id=track_id, class_name=name, confidence=.95,
            hits=8, misses=0, state=TrackState.CONFIRMED,
            last_detection=SimpleNamespace(bbox=bbox, ground_xy_mm=None))
    m = CompetitionMission(CompetitionSettings(side="blue", initial_stash_enabled=False))
    m.first_common_delivered = True
    m._select_batch(CargoBatch((1, 2), ("green_supply", "core_black"), "material"))
    m.carried_manifest, m.carried_total_count = m.selected_batch.classes, 2
    m.state = CompetitionState.DELIVERY_VERIFY
    known = (track(1, "green_supply", (400, 300, 80, 100)),
             track(2, "core_black", (540, 300, 80, 100)))
    def view(tracks):
        return runner_module.make_vision_snapshot([], tracks,
            SimpleNamespace(calibrated=False), (0, 0, 1280, 700),
            StmSnapshot(mode=15, flags=0, age_ms=0, camera_pitch_cdeg=12000), m, 100,
            recent_carried_track_ids=(1, 2))
    assert view(known).delivery_target_inside_safe_zone
    assert not view(known[:1]).delivery_target_inside_safe_zone
    for changed in (track(3, "core_black", (540, 300, 80, 100)),
                    track(2, "injured_orange", (540, 300, 80, 100)),
                    track(2, "core_black", (540, 750, 80, 100))):
        assert not view(known[:1] + (changed,)).delivery_target_inside_safe_zone
    m.carried_manifest, m.carried_total_count = ("green_supply",), 1
    assert view(known[:1]).delivery_target_inside_safe_zone


def test_held_mixture_survives_detector_gap_and_clears_on_physical_separation():
    from dataclasses import replace
    from types import SimpleNamespace
    from unittest.mock import patch
    from state_machine import CargoBatch, TrackedCargo
    from capture_roi import load_capture_rois

    m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    m._select_batch(CargoBatch((204,), ("injured_orange",), "injury"))
    m.first_common_delivered = True
    m.state = CompetitionState.POST_GRAB_AUDIT
    memory = runner_module.HeldCargoEvidence()
    rois = load_capture_rois(PROJECT_ROOT / "vision/config/capture_roi.json")
    orange = TrackedCargo(204, "injured_orange", .85, (528, 167, 402, 318), hits=398)
    green = TrackedCargo(224, "green_supply", .32, (306, 276, 264, 217), hits=91)

    def observe(items, frame, mode=48, flags=3, pitch=14000):
        with patch.object(runner_module, "tracked_cargo", return_value=items):
            return runner_module.make_vision_snapshot([], [], SimpleNamespace(calibrated=False), None,
                StmSnapshot(mode=mode, flags=flags, camera_pitch_cdeg=pitch, age_ms=0),
                m, frame, capture_rois=rois, held_cargo_evidence=memory)

    observe((orange,), 2900, mode=23)  # Existing camera-ready view; no new waiting.
    m.state = CompetitionState.NAVIGATE
    observe((orange, green), 3083, mode=10, flags=2)
    m.state = CompetitionState.TRANSPORT_AUDIT
    observe((orange,), 3119, flags=2)
    for frame in (3123, 3126):
        actual = observe((orange,), frame)
        assert actual.capture_audit.total_count == 2 and actual.capture_audit.injury_mixed
        assert not m._audit_valid(actual.capture_audit)
        assert actual.capture_cargo == (orange,)  # Not a fabricated detector box.
        remembered = [d for d in actual.roi_decisions if "remembered_source_frame" in d]
        assert remembered[0]["remembered_source_frame"] == 3083
    assert observe((), 3127).capture_audit.total_count == 0
    observe((orange,), 3128, mode=25, flags=2)  # Actual separation clears old load evidence.
    clean = observe((orange,), 3130)
    assert clean.capture_audit.total_count == 1 and m._audit_valid(clean.capture_audit)
    observe((orange, green), 3131)
    observe((), 3132, flags=0)
    assert not memory.items
    # An outside/truncated green cannot create incompatible load evidence.
    outside = replace(green, bbox=(0, 0, 70, 70))
    observe((orange, outside), 3133)
    assert observe((orange,), 3134).capture_audit.total_count == 1


def test_injury_delivery_distinguishes_platform_mixture_floor_obstacle_and_zone_contents():
    from dataclasses import replace
    from types import SimpleNamespace
    from unittest.mock import patch
    from state_machine import CargoBatch, TrackedCargo

    m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    m._select_batch(CargoBatch((239,), ("injured_orange",), "injury"))
    m.first_common_delivered = True
    m.confirmed_delivery_destination = "injury"
    m.carried_manifest = ("injured_orange",)
    m.state = CompetitionState.SAFE_ZONE_CORRIDOR_CHECK
    orange = TrackedCargo(239, "injured_orange", .8, (528, 451, 400, 317), hits=6)
    green = TrackedCargo(238, "green_supply", .8, (309, 554, 278, 216), hits=6)
    inside = TrackedCargo(241, "green_supply", .8, (30, 0, 109, 85), hits=6, inside_safe_zone=True)
    with patch.object(runner_module, "tracked_cargo", return_value=(orange, green, inside)):
        snapshot = runner_module.make_vision_snapshot([], [], SimpleNamespace(calibrated=False),
            (24, 1, 751, 130), StmSnapshot(mode=11, flags=2, age_ms=0, camera_pitch_cdeg=12000),
            m, 3139, recent_carried_track_ids=(239,))
    assert snapshot.carried_incompatible_cargo == (green,)
    floor = replace(green, bbox=(600, 400, 80, 100))
    assert runner_module.safe_zone_push_corridor((floor,), (200, 0, 800, 100),
        "injury", True, claw_polygon=((201,203),(1067,203),(1067,727),(201,727)))[1] == floor
    assert runner_module.safe_zone_push_corridor((floor,), (200, 0, 800, 100),
        "material", True, claw_polygon=((201,203),(1067,203),(1067,727),(201,727)))[1] is None
    assert runner_module.safe_zone_push_corridor((inside,), (200, 0, 800, 100),
        "injury", True, claw_polygon=((201,203),(1067,203),(1067,727),(201,727)))[1] is None


def test_pending_visual_lateral_updates_same_action_and_source_frame():
    from dataclasses import replace
    from protocol import CMD_ALIGN_SAFE_ZONE, CMD_VISUAL_CORRECTION_VALID
    from state_machine import CargoBatch
    with tempfile.TemporaryDirectory() as d:
        root = Path(d)
        m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
        m._select_batch(CargoBatch((1,), ("injured_orange",), "injury"))
        m.state = CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX
        m._update_safe_zone_visual_target((216, 0, 941, 113), 10)
        planner = CompetitionPlanner(m, root/"pose", root/"stm", root/"command",
            root/"diag", JsonlLog(root/"events"), 50)
        planner._publish(CompetitionOutput(m.state, m._safe_zone_visual_align_command(), "old"))
        task, action = planner.protocol_task, planner.protocol_action
        status = StmSnapshot(mode=11, age_ms=500, camera_pitch_cdeg=12000, context_valid=True,
            task_id=task, action_id=(action-1)&65535, action_status=1)
        view = VisionSnapshot(frame_sequence=11, safe_bbox=(116, 0, 941, 113))
        out = planner._output_for_cycle(view, PoseSnapshot(valid=True), status, False, 1.)
        assert out.command.opcode == CMD_ALIGN_SAFE_ZONE and out.command.flags & CMD_VISUAL_CORRECTION_VALID
        assert out.command.arg_a == 103 and out.command.source_vision_frame == 11
        planner._publish(out)
        assert (planner.protocol_task, planner.protocol_action) == (task, action)
        assert planner.protocol_vision_frame == 11
        assert int.from_bytes((root/"command").read_bytes()[8:12], "big") == 11
        missing = planner._output_for_cycle(replace(view, frame_sequence=12, safe_bbox=None),
            PoseSnapshot(valid=True), status, False, 1.01)
        planner._publish(missing)
        assert missing.event == "safe_zone_visual_reobserve"
        assert missing.command.arg_a == 0 and missing.command.arg_b == 11500
        assert (planner.protocol_task, planner.protocol_action) == (task, action)
        assert int.from_bytes((root/"command").read_bytes()[8:12], "big") == 0
        waiting = planner._output_for_cycle(replace(view, frame_sequence=13),
            PoseSnapshot(valid=True), status, False, 1.02)
        assert waiting.state == CompetitionState.ACQUIRE_SAFE_ZONE and not waiting.event
        assert waiting.command == missing.command
        raised = replace(status, camera_pitch_cdeg=11500)
        planner._publish(planner._output_for_cycle(replace(view, frame_sequence=14),
            PoseSnapshot(valid=True), raised, False, 1.03))
        for frame, now in ((15, 1.04), (16, 1.05), (17, 1.06)):
            recovered = planner._output_for_cycle(replace(view, frame_sequence=frame),
                PoseSnapshot(valid=True), raised, False, now)
            planner._publish(recovered)
        assert recovered.command.opcode == CMD_ALIGN_SAFE_ZONE
        assert recovered.state == CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX
        assert (planner.protocol_task, planner.protocol_action) == (task, action)
        assert planner.protocol_vision_frame == 17
        assert int.from_bytes((root/"command").read_bytes()[8:12], "big") == 17


def test_bulldozer_corridor_keeps_compatible_targets_and_prioritizes_mixed_danger():
    from state_machine import TrackedCargo
    from obstacle_route import handling_kind
    green = TrackedCargo(2,'green_supply',.9,(500,550,80,80),hits=4)
    blue = TrackedCargo(3,'danger_cyan',.9,(700,300,80,80),hits=4)
    orange = TrackedCargo(4,'injured_orange',.9,(850,400,80,80),hits=4)
    polygon = ((637,203),(1067,727),(201,715),(201,600))
    classes = []
    old = runner_module.safe_zone_push_corridor((green,), (0,0,1280,100), 'material',True,
        claw_polygon=polygon)
    assert old[1] is None  # Unenabled deployment retains its previous behavior.
    same = runner_module.safe_zone_push_corridor((green,), (0,0,1280,100), 'material',True,
        claw_polygon=polygon,include_compatible_obstacles=True,obstacle_classes_out=classes)
    assert same[1].track_id == 2 and classes == ['green_supply']
    classes.clear()
    mixed = runner_module.safe_zone_push_corridor((green,blue,orange), (0,0,1280,100), 'material',True,
        claw_polygon=polygon,include_compatible_obstacles=True,obstacle_classes_out=classes)
    assert mixed[1].track_id == 3 and set(classes)=={'green_supply','danger_cyan','injured_orange'}
    assert handling_kind('material',tuple(classes),first=False)=='sweep'


def test_bulldozer_steps_pair_actions_and_keep_live_distance_updates():
    from dataclasses import replace
    from obstacle_route import RouteStep
    from protocol import CMD_ALIGN_SAFE_ZONE, CMD_BULLDOZER_STEP
    from state_machine import CargoBatch
    with tempfile.TemporaryDirectory() as d:
        root=Path(d)
        m=CompetitionMission(CompetitionSettings(side='red',initial_stash_enabled=False,bulldozer_enabled=True))
        m._select_batch(CargoBatch((1,),('green_supply',),'material'))
        m.confirmed_delivery_destination='material'
        p=PoseSnapshot(True,-.15,.54,90,0)
        v=VisionSnapshot(frame_sequence=10,safe_corridor_obstacle_class='danger_cyan')
        s=StmSnapshot(mode=11,flags=2,age_ms=0,camera_pitch_cdeg=12000,
                      context_valid=True,accepted_command=CMD_ALIGN_SAFE_ZONE,action_status=3)
        initial=m._blocked_corridor_output(v,s,1.,pose=p)
        planner=CompetitionPlanner(m,root/'pose',root/'stm',root/'command',root/'diag',JsonlLog(root/'events'),50)
        planner._publish(initial)
        ids=(planner.protocol_task,planner.protocol_action)
        old=replace(s,mode=25,task_id=ids[0],action_id=(ids[1]-1)&65535,
                     accepted_command=CMD_BULLDOZER_STEP)
        waiting=planner._output_for_cycle(v,p,old,False,1.1)
        assert m.obstacle_route.step==RouteStep.TURN_RIGHT
        planner._publish(waiting)
        assert (planner.protocol_task,planner.protocol_action)==ids
        paired=replace(old,action_id=ids[1])
        moving=planner._output_for_cycle(v,p,paired,False,1.2)
        assert m.obstacle_route.step==RouteStep.FORWARD_PARK
        planner._publish(moving)
        assert planner.protocol_action != ids[1]
        # A live leg's remaining distance changes without allocating another action.
        m.obstacle_route.index=m.obstacle_route.steps.index(RouteStep.BACK_TO_SWEEP)
        live=planner._output_for_cycle(v,replace(p,x_m=0.),old,False,1.3)
        planner._publish(live);live_ids=(planner.protocol_task,planner.protocol_action)
        updated=planner._output_for_cycle(v,replace(p,x_m=-.2),old,False,1.4)
        assert updated.command.arg_a==360 and live.command.arg_a==560
        planner._publish(updated)
        assert (planner.protocol_task,planner.protocol_action)==live_ids
        assert (root/'command').read_bytes()[19]==CMD_BULLDOZER_STEP


def test_published_enter_cannot_roll_back_on_a_later_obstacle_or_missing_box():
    from dataclasses import replace
    from protocol import CMD_ALIGN_SAFE_ZONE, CMD_ENTER_SAFE_ZONE
    from state_machine import CargoBatch
    with tempfile.TemporaryDirectory() as d:
        root=Path(d)
        m=CompetitionMission(CompetitionSettings(side='red',initial_stash_enabled=False,bulldozer_enabled=True))
        m._select_batch(CargoBatch((1,),('green_supply',),'material'))
        m.confirmed_delivery_destination='material';m.carried_manifest=('green_supply',)
        m.state=CompetitionState.SAFE_ZONE_CORRIDOR_CHECK;m.safe_zone_visual_locked=True
        m._update_safe_zone_visual_target((320,0,960,100),10)
        p=PoseSnapshot(True,-.15,.54,90,0)
        s=StmSnapshot(mode=11,flags=2,age_ms=0,camera_pitch_cdeg=12000,
            context_valid=True,context_matches=True,accepted_command=CMD_ALIGN_SAFE_ZONE,action_status=3)
        v=VisionSnapshot(frame_sequence=11,safe_bbox=(320,0,960,100),
            safe_corridor_polygon=((201,727),(1067,727),(1067,100),(201,100)))
        out=m.step(v,p,s,1.)
        assert out.command.opcode==CMD_ENTER_SAFE_ZONE and not m.enter_command_published
        planner=CompetitionPlanner(m,root/'pose',root/'stm',root/'command',root/'diag',JsonlLog(root/'events'),50)
        planner._publish(out)
        task,action=planner.protocol_task,planner.protocol_action
        assert m.enter_command_published and not m.enter_command_accepted
        late=replace(v,frame_sequence=12,safe_bbox=None,safe_zone_filter_blocked=True,
            safe_corridor_obstacle_class='danger_cyan')
        # Directly exercise the ACK-in-flight window as well as actual ACCEPT.
        waiting=m.step(late,p,replace(s,context_matches=False),1.01)
        assert waiting.command.opcode==CMD_ENTER_SAFE_ZONE and m.state==CompetitionState.ENTER_SAFE_ZONE
        accepted=replace(s,mode=10,task_id=task,action_id=action,
                         accepted_command=CMD_ENTER_SAFE_ZONE,action_status=1)
        continued=planner._output_for_cycle(late,p,accepted,False,1.02)
        assert continued.command.opcode==CMD_ENTER_SAFE_ZONE and m.obstacle_route is None
        assert m.enter_command_accepted and m.carried_manifest==('green_supply',)
        planner._publish(continued)
        assert (planner.protocol_task,planner.protocol_action)==(task,action)
        paused=planner._output_for_cycle(late,p,accepted,True,1.03)
        assert paused.command.opcode==runner_module.CMD_PAUSE
        done=planner._output_for_cycle(late,p,replace(accepted,mode=15),False,1.04)
        assert done.state==CompetitionState.DELIVERY_VERIFY


def test_explicit_align_rejection_restores_the_exact_accepted_enter_action():
    from dataclasses import replace
    from protocol import CMD_ENTER_SAFE_ZONE
    from state_machine import CargoBatch
    with tempfile.TemporaryDirectory() as d:
        root=Path(d)
        m=CompetitionMission(CompetitionSettings(side='red',initial_stash_enabled=False))
        m._select_batch(CargoBatch((1,),('green_supply',),'material'))
        m.carried_manifest=('green_supply',);m.confirmed_delivery_destination='material'
        m.state=CompetitionState.ENTER_SAFE_ZONE;m.safe_zone_visual_locked=True
        planner=CompetitionPlanner(m,root/'pose',root/'stm',root/'command',root/'diag',JsonlLog(root/'events'),50)
        planner.protocol_task=planner.protocol_task_cursor=24051
        planner.protocol_action_cursor=31235
        enter=m._enter_safe_zone_command()
        planner._publish(CompetitionOutput(m.state,enter,'fixture'))
        assert planner.protocol_action==31236
        # Reproduce the recorded old rollback: ENTER31236, then ALIGN31237.
        m._begin_safe_zone_acquire(VisionSnapshot(frame_sequence=100),1.)
        m.safe_zone_reobserve_visual=True;m.safe_zone_view_pitch_cdeg=11500
        planner._publish(CompetitionOutput(m.state,m._safe_zone_acquire_command(),'old rollback'))
        assert planner.protocol_action==31237 and not m.safe_zone_visual_locked
        rejected=StmSnapshot(mode=10,flags=2,age_ms=0,camera_pitch_cdeg=11500,
            context_valid=True,task_id=24051,action_id=31236,accepted_command=CMD_ENTER_SAFE_ZONE,
            action_status=5,rejected_action_id=31237)
        view=VisionSnapshot(frame_sequence=101)
        pose=PoseSnapshot(True,-.15,.54,90,0)
        for unrelated in (replace(rejected,task_id=24052),replace(rejected,rejected_action_id=31235),
                          replace(rejected,action_status=1)):
            assert planner._reconcile_safe_zone_rejection(view,pose,unrelated,1.1) is None
        out=planner._output_for_cycle(view,pose,rejected,False,1.2)
        assert out.event=='safe_zone_request_reconciled' and out.command==enter
        assert m.enter_command_published and m.enter_command_accepted and m.safe_zone_visual_locked
        assert m.state==CompetitionState.ENTER_SAFE_ZONE and m.carried_manifest==('green_supply',)
        planner._publish(out)
        assert planner.protocol_action==31236 and planner.protocol_action_cursor==31237
        assert (root/'command').read_bytes()[19:27]==enter.to_frame(0)[4:12]
        again=planner._output_for_cycle(view,pose,rejected,False,1.3)
        assert again.command==enter and again.event!='safe_zone_request_reconciled'
        planner._publish(again);assert planner.protocol_action==31236
        finished=m._finish_delivery(rejected,2.,basis='fixture',message='fixture',event='fixture')
        planner._publish(finished)
        assert planner.protocol_action==31238  # Never reuse rejected31237.


def test_rejected_enter_returns_to_accepted_align_then_rechecks_for_bulldozer():
    from dataclasses import replace
    from protocol import CMD_ALIGN_SAFE_ZONE, CMD_ENTER_SAFE_ZONE, CMD_BULLDOZER_STEP
    from state_machine import CargoBatch
    with tempfile.TemporaryDirectory() as d:
        root=Path(d)
        m=CompetitionMission(CompetitionSettings(side='red',initial_stash_enabled=False,bulldozer_enabled=True))
        m._select_batch(CargoBatch((1,),('green_supply',),'material'))
        m.confirmed_delivery_destination='material'
        m.state=CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX
        m.safe_zone_view_pitch_cdeg=11500
        m._update_safe_zone_visual_target((320,0,960,100),42)
        planner=CompetitionPlanner(m,root/'pose',root/'stm',root/'command',root/'diag',JsonlLog(root/'events'),50)
        align=m._safe_zone_visual_align_command()
        planner._publish(CompetitionOutput(m.state,align,'fixture'))
        task,action=planner.protocol_task,planner.protocol_action
        m._set_state(CompetitionState.ENTER_SAFE_ZONE,1.);m.safe_zone_visual_locked=True
        planner._publish(CompetitionOutput(m.state,m._enter_safe_zone_command(),'fixture'))
        denied=planner.protocol_action
        status=StmSnapshot(mode=10,flags=2,age_ms=0,camera_pitch_cdeg=11500,context_valid=True,
            task_id=task,action_id=action,accepted_command=CMD_ALIGN_SAFE_ZONE,action_status=5,
            rejected_action_id=denied)
        view=VisionSnapshot(frame_sequence=43,safe_bbox=(320,0,960,100))
        p=PoseSnapshot(True,-.15,.54,90,0)
        restored=planner._output_for_cycle(view,p,status,False,1.1)
        assert restored.event=='safe_zone_request_reconciled' and restored.command==align
        assert not m.enter_command_published and not m.enter_command_accepted
        planner._publish(restored)
        assert planner.protocol_action==action and planner.protocol_action_cursor==denied
        waiting=planner._output_for_cycle(view,p,status,False,1.2)
        assert waiting.state==CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX
        planner._publish(waiting)
        done=replace(status,mode=11,action_status=3,rejected_action_id=0)
        corridor=planner._output_for_cycle(replace(view,frame_sequence=44),p,done,False,1.3)
        assert corridor.state==CompetitionState.SAFE_ZONE_CORRIDOR_CHECK
        planner._publish(corridor)
        obstacle=replace(view,frame_sequence=45,safe_corridor_obstacle_class='danger_cyan',
            safe_corridor_polygon=((201,727),(1067,727),(1067,100),(201,100)))
        swept=planner._output_for_cycle(obstacle,p,done,False,1.4)
        assert swept.state==CompetitionState.OBSTACLE_ROUTE and swept.command.opcode==CMD_BULLDOZER_STEP
        planner._publish(swept)
        assert planner.protocol_action!=denied


def test_locked_single_target_keeps_whole_claw_audit_in_every_view():
    from dataclasses import replace
    from types import SimpleNamespace
    from unittest.mock import patch
    from capture_roi import CaptureRois, split_capture_roi
    from state_machine import CargoBatch, TrackedCargo, SweepPickupContext

    polygon = ((100, 100), (1200, 100), (1200, 950), (100, 950))
    rois = CaptureRois(polygon, *split_capture_roi(polygon))
    greens = tuple(TrackedCargo(i, 'green_supply', .9, (x, 500, 100, 100), hits=4)
                   for i, x in ((1, 400), (2, 520), (3, 760)))
    for mode in (21, 38, 23, 48, 43):
        for items in (greens, (greens[0], replace(greens[2], class_name='danger_cyan'))):
            m = CompetitionMission(CompetitionSettings(side='red', initial_stash_enabled=False))
            m._select_batch(CargoBatch((1,), ('green_supply',), 'material'))
            m.locked_target_track_id = 1
            m.safe_sweep_pickup = SweepPickupContext(target_id=1, target_class='green_supply',
                target_bbox=greens[0].bbox, excluded_ids=frozenset())
            status = StmSnapshot(mode=mode, flags=1, age_ms=0, camera_pitch_cdeg=14000)
            with patch.object(runner_module, 'tracked_cargo', return_value=items):
                view = runner_module.make_vision_snapshot([], [], SimpleNamespace(calibrated=False),
                    None, status, m, 10, capture_rois=rois, observed_s=1.)
            assert view.capture_observation_valid
            for audit in (view.capture_audit, view.safe_sweep_capture_audit):
                assert audit.total_count == len(items)
                assert audit.danger_present == any(x.class_name == 'danger_cyan' for x in items)
                assert not m._audit_valid(audit)
            assert len(view.capture_cargo) == len(items) and m.selected_batch.track_ids == (1,)


def test_retired_sweep_cannot_restart_through_planner_context_recovery():
    from state_machine import CargoBatch
    from protocol import CMD_PAUSE
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        m = CompetitionMission(CompetitionSettings(side='red', initial_stash_enabled=False))
        m._select_batch(CargoBatch((1,), ('green_supply',), 'material'))
        m.carried_manifest = ('green_supply',)
        m.carried_total_count = 1
        m.state = CompetitionState.CLEAR_SAFE_ZONE
        p = CompetitionPlanner(m, root/'pose', root/'stm', root/'cmd', root/'diag', JsonlLog(root/'events'), 50)
        out = p._output_for_cycle(VisionSnapshot(), PoseSnapshot(valid=True),
            StmSnapshot(mode=43, age_ms=0, camera_pitch_cdeg=14000, flags=1), False, 1.)
        assert out.command.opcode == CMD_PAUSE and out.reason == 'legacy_safe_sweep_disabled'
        assert m.state == CompetitionState.WAIT_SAFE_ZONE_CLEAR
        assert m.carried_manifest == ('green_supply',) and m.safe_sweep_pickup is None


def test_corridor_is_built_only_after_alignment_and_does_not_reuse_pixel_memory():
    from types import SimpleNamespace
    from unittest.mock import patch
    from capture_roi import CaptureRois, split_capture_roi
    from state_machine import CargoBatch, TrackedCargo
    polygon = ((200, 200), (1080, 200), (1080, 730), (200, 730))
    rois = CaptureRois(polygon, *split_capture_roi(polygon))
    m = CompetitionMission(CompetitionSettings(side='red', initial_stash_enabled=False))
    m._select_batch(CargoBatch((1,), ('green_supply',), 'material'))
    m.confirmed_delivery_destination = 'material'
    obstacle = TrackedCargo(2, 'danger_cyan', .9, (550, 400, 100, 100), hits=4)
    remembered = TrackedCargo(3, 'green_supply', .8, (500, 450, 80, 80), visible=False)
    status = StmSnapshot(mode=11, flags=2, age_ms=0, camera_pitch_cdeg=12000)
    def snapshot(items=(), memory=remembered):
        with patch.object(runner_module, 'tracked_cargo', return_value=items):
            return runner_module.make_vision_snapshot([], [], SimpleNamespace(calibrated=False),
                (200, 0, 1000, 100), status, m, 20, capture_rois=rois,
                recent_corridor_obstacle=memory)
    for state in (CompetitionState.SEARCH, CompetitionState.NAVIGATE,
                  CompetitionState.ALIGN_SAFE_ZONE_BY_POSE, CompetitionState.ACQUIRE_SAFE_ZONE,
                  CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX):
        m.state = state
        v = snapshot((obstacle,))
        assert v.safe_corridor_polygon == () and v.safe_corridor_obstacle_track_id is None
    for state in (CompetitionState.SAFE_ZONE_CORRIDOR_CHECK,
                  CompetitionState.WAIT_SAFE_ZONE_CLEAR, CompetitionState.ENTER_SAFE_ZONE):
        m.state = state
        v = snapshot()
        assert v.safe_corridor_polygon and v.safe_corridor_obstacle_track_id is None
        assert not v.safe_corridor_obstacle_remembered


def test_claw_template_rectangle_has_fixed_width_and_reaches_side_tangent_end():
    from state_machine import TrackedCargo
    template = ((637, 203), (1067, 727), (201, 715), (201, 600))
    for destination in ('material', 'injury'):
        for bbox in ((50, 0, 500, 120), (700, 20, 350, 100)):
            polygon = runner_module.safe_zone_corridor_polygon(bbox, destination, template)
            assert polygon == ((201,727),(1067,727),(1067,120),(201,120))
    # An interior bottom point is not one of the two side tangents.
    extended = template + ((640, 900),)
    assert runner_module.safe_zone_corridor_polygon((50,0,500,120),'material',extended) == (
        (201,727),(1067,727),(1067,120),(201,120))
    # The former narrow bottom missed this real blue foot near(621,636).
    blue = TrackedCargo(740,'danger_cyan',.97,(515,395,212,241),hits=8)
    for destination in ('material', 'injury'):
        polygon, obstacle, _, _ = runner_module.safe_zone_push_corridor((blue,),
            (352,0,913,97), destination, False, claw_polygon=template)
        assert obstacle is blue
    outside = TrackedCargo(5,'danger_cyan',.9,(100,400,80,100),hits=4)
    behind = TrackedCargo(6,'danger_cyan',.9,(550,700,100,100),hits=4)
    assert runner_module.safe_zone_push_corridor((outside,behind), (352,0,913,97),
        'material', False, claw_polygon=template)[1] is None
    assert runner_module.safe_zone_corridor_polygon(None,'material',template)==()
    collapsed, obstacle, _, status = runner_module.safe_zone_push_corridor((blue,),
        (100,700,1000,100),'material',False,claw_polygon=template)
    assert collapsed == ((201,727),(1067,727),(1067,727),(201,727))
    assert obstacle is None and status == 'CLEAR'


def test_rectangle_excludes_both_confirmed_green_black_loads_and_keeps_extra_blue():
    from state_machine import TrackedCargo
    template=((201,203),(1067,203),(1067,727),(201,727))
    green=TrackedCargo(1,'green_supply',.9,(400,450,100,100),hits=4)
    black=TrackedCargo(2,'core_black',.9,(600,450,100,100),hits=4)
    blue=TrackedCargo(3,'danger_cyan',.9,(800,450,100,100),hits=4)
    polygon,obstacle,_,status=runner_module.safe_zone_push_corridor((green,black),
        (100,0,1000,100),'material',False,frozenset({1,2}),
        carried_manifest=('green_supply','core_black'),carried_total_count=2,
        claw_polygon=template)
    assert polygon == ((201,727),(1067,727),(1067,100),(201,100))
    assert obstacle is None and status=='CLEAR'
    assert runner_module.safe_zone_push_corridor((green,black,blue),
        (100,0,1000,100),'material',True,frozenset({1,2}),
        carried_manifest=('green_supply','core_black'),carried_total_count=2,
        claw_polygon=template)[1] is blue


def test_rectangle_snapshot_keeps_same_template_at_both_delivery_pitches():
    from types import SimpleNamespace
    from unittest.mock import patch
    from capture_roi import CaptureRois, split_capture_roi
    from state_machine import CargoBatch, TrackedCargo
    polygon=((637,203),(1067,727),(201,715),(201,600))
    rois=CaptureRois(polygon,*split_capture_roi(polygon))
    m=CompetitionMission(CompetitionSettings(side='red',initial_stash_enabled=False))
    m._select_batch(CargoBatch((1,2),('green_supply','core_black'),'material'))
    m.carried_manifest=('green_supply','core_black');m.carried_total_count=2
    m.confirmed_delivery_destination='material';m.first_common_delivered=True
    m.state=CompetitionState.SAFE_ZONE_CORRIDOR_CHECK
    green=TrackedCargo(1,'green_supply',.9,(400,450,100,100),hits=4)
    black=TrackedCargo(2,'core_black',.9,(600,450,100,100),hits=4)
    blue=TrackedCargo(3,'danger_cyan',.9,(800,450,100,100),hits=4)
    for pitch in (11500,12000):
        status=StmSnapshot(mode=11,flags=2,age_ms=0,camera_pitch_cdeg=pitch)
        with patch.object(runner_module,'tracked_cargo',return_value=(green,black,blue)):
            view=runner_module.make_vision_snapshot([],[],SimpleNamespace(calibrated=False),
                (340,0,900,100),status,m,20,capture_rois=rois,recent_carried_track_ids=(1,2))
        assert view.safe_corridor_polygon==((201,727),(1067,727),(1067,100),(201,100))
        assert view.carried_reference_track_ids==(1,2)
        assert view.safe_corridor_obstacle_track_id==3


def test_rectangle_entry_checks_current_extra_objects_and_preserves_local_enter():
    from dataclasses import replace
    from types import SimpleNamespace
    from unittest.mock import patch
    from capture_roi import CaptureRois, split_capture_roi
    from state_machine import CargoBatch, TrackedCargo
    from protocol import CMD_ENTER_SAFE_ZONE, CMD_PAUSE, CMD_CLEAR_SAFE_ZONE
    polygon=((637,203),(1067,727),(201,715),(201,600))
    rois=CaptureRois(polygon,*split_capture_roi(polygon))
    for name,x,blocked in (('green_supply',1100,False),('green_supply',600,True),('danger_cyan',600,True)):
        m=CompetitionMission(CompetitionSettings(side='red',initial_stash_enabled=False))
        m._select_batch(CargoBatch((1,),('green_supply',),'material'))
        m.carried_manifest=('green_supply',);m.carried_total_count=1
        m.confirmed_delivery_destination='material';m.safe_zone_visual_locked=True
        m.state=CompetitionState.SAFE_ZONE_CORRIDOR_CHECK;m.safe_corridor_frame_floor=19
        held=TrackedCargo(1,'green_supply',.9,(400,450,100,100),hits=4)
        extra=TrackedCargo(2,name,.9,(x,400,80,100),hits=4)
        status=StmSnapshot(mode=11,flags=2,age_ms=0,camera_pitch_cdeg=12000)
        def snapshot(items,frame):
            with patch.object(runner_module,'tracked_cargo',return_value=items):
                return runner_module.make_vision_snapshot([],[],SimpleNamespace(calibrated=False),
                    (340,0,900,100),status,m,frame,capture_rois=rois,recent_carried_track_ids=(1,),
                    observed_s=1.+(frame-20)*.1)
        view=snapshot((held,extra),20)
        out=m.step(view,PoseSnapshot(valid=True,yaw_deg=90),status,1.)
        assert out.command.opcode!=CMD_CLEAR_SAFE_ZONE
        if blocked:
            assert out.command.opcode==CMD_PAUSE and m.state==CompetitionState.WAIT_SAFE_ZONE_CLEAR
            assert m.carried_manifest==('green_supply',)
            out=m.step(snapshot((held,),21),PoseSnapshot(valid=True,yaw_deg=90),status,1.1)
        assert out.command.opcode==CMD_ENTER_SAFE_ZONE
        assert out.command.arg_a==out.command.arg_b==out.command.aux==0


def test_search_and_approach_exclude_all_current_red_and_blue_safe_boxes():
    from types import SimpleNamespace
    from rescue_vision.models import TrackState
    from state_machine import CargoBatch
    def track(i,name,box):
        return SimpleNamespace(track_id=i,class_name=name,confidence=.95,hits=6,misses=0,
            state=TrackState.CONFIRMED,last_detection=SimpleNamespace(bbox=box,ground_xy_mm=None))
    boxes={'safe_red':(80,100,300,260),'safe_blue':(500,350,400,350)}
    detections=[SimpleNamespace(class_name=k,bbox=v) for k,v in boxes.items()]
    detections.append(SimpleNamespace(class_name='safe_red',bbox=(1020,100,180,180)))
    items=(track(1,'green_supply',(150,150,60,60)),track(2,'green_supply',(550,400,60,60)),
           track(3,'core_black',(750,450,70,80)),track(4,'green_supply',(980,750,100,100)),
           track(5,'injured_orange',(600,640,60,65)),track(6,'green_supply',(1080,120,40,60)),
           track(7,'danger_cyan',(600,450,80,100)))
    for side in ('red','blue'):
        m=CompetitionMission(CompetitionSettings(side=side,initial_stash_enabled=False))
        m.state=CompetitionState.SEARCH
        own=boxes['safe_'+side]
        def snapshot(values,regions=detections,blocked=False):
            return runner_module.make_vision_snapshot(regions,values,SimpleNamespace(calibrated=False),
                own,StmSnapshot(mode=3,age_ms=0,camera_pitch_cdeg=12000),m,10,
                safe_zone_filter_blocked=blocked,observed_s=1.)
        v=snapshot(items)
        assert {x.track_id for x in v.cargo if x.inside_safe_zone}=={1,2,3,6,7}
        assert m._choose_initial_green(v).track_id==4
        assert not v.danger_ahead
        m._select_batch(CargoBatch((2,),('green_supply',),'material'))
        m.locked_target_track_id=2;m.state=CompetitionState.APPROACH
        assert m._target_for_batch(v).track_id==4
        m.state=CompetitionState.SEARCH
        protected=tuple(x for x in items if x.track_id in {1,2,3,6,7})
        out=m.step(snapshot(protected),PoseSnapshot(valid=True),StmSnapshot(mode=3,age_ms=0),1.)
        assert out.state==CompetitionState.SEARCH and out.command.opcode==CMD_HOLD
        # A real opponent box remains usable while only the OWN cached box is missing.
        other='safe_blue' if side=='red' else 'safe_red'
        v=snapshot(items,[SimpleNamespace(class_name=other,bbox=boxes[other])],True)
        own_id,other_id=(1,2) if side=='red' else (2,1)
        flags={x.track_id:x.inside_safe_zone for x in v.cargo}
        assert flags[other_id] and not flags[own_id]


def test_opponent_exclusion_does_not_confirm_delivery_or_hide_claw_load():
    from types import SimpleNamespace
    from rescue_vision.models import TrackState
    from capture_roi import CaptureRois,split_capture_roi
    from state_machine import CargoBatch
    def track(i,name,box):
        return SimpleNamespace(track_id=i,class_name=name,confidence=.95,hits=6,misses=0,
            state=TrackState.CONFIRMED,last_detection=SimpleNamespace(bbox=box,ground_xy_mm=None))
    boxes={'safe_red':(100,100,300,250),'safe_blue':(500,350,400,350)}
    detections=[SimpleNamespace(class_name=k,bbox=v) for k,v in boxes.items()]
    red=track(1,'green_supply',(150,150,60,60));blue=track(2,'green_supply',(550,400,60,60))
    for side in ('red','blue'):
        m=CompetitionMission(CompetitionSettings(side=side,initial_stash_enabled=False))
        m._select_batch(CargoBatch((1,),('green_supply',),'material'))
        m.carried_manifest=('green_supply',);m.carried_total_count=1
        m.state=CompetitionState.DELIVERY_VERIFY
        for known in (1,2):
            v=runner_module.make_vision_snapshot(detections,(red,blue),SimpleNamespace(calibrated=False),
                boxes['safe_'+side],StmSnapshot(mode=15,age_ms=0,camera_pitch_cdeg=12000),m,20,
                recent_carried_track_ids=(known,))
            assert all(x.inside_safe_zone for x in v.cargo)
            inside=(known==(1 if side=='red' else 2))
            assert v.delivery_target_inside_safe_zone==inside
            assert v.delivery_target_outside_safe_zone!=inside
    # Already-held cargo remains a WHOLE claw audit even when projected on a safe colour.
    m.state=CompetitionState.POST_GRAB_AUDIT
    cyan=track(3,'danger_cyan',(650,450,80,100))
    poly=((1,1),(1278,1),(1278,1022),(1,1022));rois=CaptureRois(poly,*split_capture_roi(poly))
    v=runner_module.make_vision_snapshot(detections,(blue,cyan),SimpleNamespace(calibrated=False),
        boxes['safe_blue'],StmSnapshot(mode=23,flags=3,age_ms=0,camera_pitch_cdeg=14000),m,21,
        capture_rois=rois)
    assert v.capture_audit.total_count==2 and v.capture_audit.danger_present
