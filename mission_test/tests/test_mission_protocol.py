#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
import tempfile
import time
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT / "vision"))
sys.path.insert(0, str(PROJECT / "mission_test"))

from run_mission_test import (
    MissionPlanner,
    load_stm_status,
    observation,
    validate_start_pose,
    write_contact_pose,
)
from state_machine import (
    DeliveryConfirmation,
    MissionOutput,
    MissionSettings,
    MissionState,
    PoseInput,
    RescueMission,
)

from rescue_vision.mission_protocol import (
    CMD_DRIVE_STRAIGHT,
    CMD_DISTANCE_VALID,
    CMD_GRAB_CONFIRMED,
    CMD_NAVIGATE_WAYPOINT,
    CMD_RED_SIDE,
    CMD_USE_FINAL_HEADING,
    CMD_VALID,
    MissionCommand,
    Stm32Status,
    write_command_frame,
)
from types import SimpleNamespace


def main() -> None:
    validate_start_pose(
        {"start_zone": 1},
        PoseInput(True, -1.350, 1.350, 135.0, 2.0),
        20.0,
    )
    try:
        validate_start_pose(
            {"start_zone": 1},
            PoseInput(True, -1.329, 1.350, 135.0, 2.0),
            20.0,
        )
        raise AssertionError("21 mm start-pose mismatch accepted")
    except RuntimeError as error:
        assert "START_POSE_MISMATCH" in str(error)
    command = MissionCommand(
        CMD_NAVIGATE_WAYPOINT,
        CMD_VALID | CMD_DRIVE_STRAIGHT | CMD_USE_FINAL_HEADING |
        CMD_RED_SIDE | CMD_DISTANCE_VALID,
        target_x_mm=1374,
        target_y_mm=0,
        heading_cdeg=12821,
    )
    packet = command.to_frame(0x20)
    assert packet.hex(" ").upper() == (
        "A3 B3 18 20 03 1F 05 5E 00 00 32 15 BA 5A C3"
    )
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        path = root / "uart_command.bin"
        write_command_frame(path, packet)
        assert path.read_bytes() == packet
        status_path = root / "stm32_status.json"
        status_path.write_text(json.dumps({
            "timestamp_monotonic_ns": time.monotonic_ns(),
            "flags": 3 | 32,
            "mode": 4,
            "relay": {
                "tx_frames": 123,
                "tx_errors": 0,
                "last_sequence": 77,
                "last_tx_age_ms": 4.5,
            },
        }), encoding="utf-8")
        status = load_stm_status(status_path)
        assert status.claw_visible and status.gripper_closed
        assert status.distance_done
        assert status.mode == 4 and status.age_ms < 250.0
        assert status.relay_tx_frames == 123
        assert status.relay_last_sequence == 77
        assert status.relay_last_tx_age_ms == 4.5

        contact_path = root / "delivery_contact_pose.json"
        write_contact_pose(
            contact_path,
            PoseInput(True, -0.13, 1.045, 89.0),
            (-0.13, 1.035, 89.0),
            "red",
            "green_supply",
            1,
        )
        contact = json.loads(contact_path.read_text(encoding="utf-8"))
        assert contact["side"] == "red"
        assert contact["cargo_class"] == "green_supply"
        assert contact["delivery_count"] == 1
        assert contact["applied_to_localization"] is False
        assert contact["constraint_axis"] == "y"
        assert abs(contact["suggested_position_correction_m"]["x"]) < 1e-9
        assert abs(contact["suggested_position_correction_m"]["y"] + 0.01) < 1e-9
    repeated_a = MissionCommand(CMD_GRAB_CONFIRMED).to_frame(0x30)
    repeated_b = MissionCommand(CMD_GRAB_CONFIRMED).to_frame(0x31)
    assert repeated_a[4] == repeated_b[4] == CMD_GRAB_CONFIRMED
    assert repeated_a != repeated_b and repeated_a[3] + 1 == repeated_b[3]
    test_observation_prefers_targets_outside_safe_zone()
    test_independent_planner_updates()
    test_delivery_observation_log()
    print("mission protocol PASS")


def test_observation_prefers_targets_outside_safe_zone() -> None:
    safe = SimpleNamespace(class_name="safe_red", bbox=(500, 400, 300, 300))
    inside = SimpleNamespace(class_name="green_supply", bbox=(600, 520, 80, 80))
    outside = SimpleNamespace(class_name="green_supply", bbox=(100, 100, 80, 80))

    selected = observation(
        [safe, inside, outside], ("green_supply",), "safe_red"
    )
    assert selected.target_bbox == outside.bbox
    assert selected.safe_found and selected.safe_bbox == safe.bbox
    assert selected.delivery_target_found
    assert not selected.delivery_target_inside_safe_zone

    blocked = observation([safe, inside], ("green_supply",), "safe_red")
    assert blocked.target_bbox == inside.bbox
    assert blocked.safe_found and blocked.safe_bbox == safe.bbox
    assert blocked.delivery_target_found
    assert blocked.delivery_target_inside_safe_zone


def test_independent_planner_updates() -> None:
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        pose_path = root / "pose.json"
        status_path = root / "status.json"
        command_path = root / "command.bin"
        diagnostics_path = root / "diagnostics.json"
        mission = RescueMission(MissionSettings(side="red"))
        mission.state = MissionState.NAVIGATE
        mission.selected_class = "green_supply"
        planner = MissionPlanner(
            mission, pose_path, status_path, command_path,
            diagnostics_path, rate_hz=50.0
        )

        def write_pose(x_m: float, y_m: float, *, age_ms: float = 0.0) -> None:
            pose_path.write_text(json.dumps({
                "timestamp_monotonic_ns": time.monotonic_ns() - round(age_ms * 1_000_000),
                "quality": "GOOD",
                "pose": {"x_m": x_m, "y_m": y_m, "yaw_deg": 0.0},
            }), encoding="utf-8")

        status_path.write_text(json.dumps({
            "timestamp_monotonic_ns": time.monotonic_ns(),
            "flags": 2,
            "mode": 10,
        }), encoding="utf-8")
        write_pose(0.60, 0.0)
        planner.tick()
        first = command_path.read_bytes()
        first_remaining = int.from_bytes(first[6:8], "big", signed=True)
        first_heading = int.from_bytes(first[10:12], "big")

        write_pose(0.30, 0.30)
        planner.tick()
        second = command_path.read_bytes()
        second_remaining = int.from_bytes(second[6:8], "big", signed=True)
        second_heading = int.from_bytes(second[10:12], "big")
        assert second_remaining != first_remaining
        assert second_heading != first_heading

        status_path.write_text(json.dumps({
            "timestamp_monotonic_ns": time.monotonic_ns(),
            "flags": 2 | 4,
            "mode": 10,
        }), encoding="utf-8")
        planner.last_remaining_mm = second_remaining
        planner.remaining_changed_s = time.monotonic() - 0.6
        planner.tick()
        assert "剩余距离" in planner.snapshot()[4]
        last_valid = command_path.read_bytes()

        write_pose(0.20, 0.40, age_ms=300.0)
        planner.tick()
        assert command_path.read_bytes() == last_valid

        write_pose(0.10, 0.50)
        planner.tick()
        resumed = command_path.read_bytes()
        assert resumed != first and resumed != second
        resumed_remaining = int.from_bytes(resumed[6:8], "big", signed=True)
        assert resumed_remaining < first_remaining
        diagnostics = json.loads(diagnostics_path.read_text(encoding="utf-8"))
        for key in (
            "state_machine_state", "stm_mode", "stm_fault_code",
            "stm_acknowledged_sequence", "last_command",
            "last_command_sequence", "pose_x_mm", "pose_y_mm",
            "pose_yaw_deg", "pose_valid", "motors_active",
            "gripper_closed", "planner_pose_age_ms",
            "planner_command_age_ms", "command_heading_deg",
            "command_remaining_mm", "relay_tx_age_ms",
            "vision_target_in_safe_zone", "vision_safe_zone_found",
            "vision_frame_sequence", "delivery_target_found",
            "delivery_target_inside_safe_zone", "delivery_outside_seen",
            "delivery_inside_hits", "delivery_visual_confirmed",
        ):
            assert key in diagnostics


def test_delivery_observation_log() -> None:
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        observation_log = root / "delivery_observation.jsonl"
        planner = MissionPlanner(
            RescueMission(MissionSettings(side="red")),
            root / "pose.json",
            root / "status.json",
            root / "command.bin",
            root / "diagnostics.json",
            rate_hz=50.0,
            delivery_observation_log_path=observation_log,
        )
        confirmation = DeliveryConfirmation(
            cargo_class="green_supply",
            outside_seen=True,
            outside_frame_sequence=11,
            outside_target_bbox=(100, 100, 40, 40),
            outside_safe_bbox=None,
            inside_hits=5,
            frame_sequence=16,
            target_bbox=(600, 500, 40, 40),
            safe_bbox=(500, 400, 300, 300),
        )
        planner._write_delivery_observation(
            MissionOutput(
                MissionState.COMPLETE,
                None,
                None,
                "visual confirmation",
                delivery_confirmation=confirmation,
            ),
            PoseInput(True, -0.15, 1.03, 90.0),
            Stm32Status(mode=15, age_ms=5.0),
            time.monotonic(),
        )
        saved = json.loads(observation_log.read_text(encoding="utf-8"))
        assert saved["source"] == "visual_safe_zone_transition"
        assert saved["outside_seen"] and saved["outside_frame_sequence"] == 11
        assert saved["inside_hits"] == 5 and saved["inside_frame_sequence"] == 16
        assert saved["target_bbox"] == [600, 500, 40, 40]


if __name__ == "__main__":
    main()
