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

from run_mission_test import load_stm_status, write_contact_pose
from state_machine import PoseInput

from rescue_vision.mission_protocol import (
    CMD_DRIVE_STRAIGHT,
    CMD_GRAB_CONFIRMED,
    CMD_NAVIGATE_WAYPOINT,
    CMD_RED_SIDE,
    CMD_USE_FINAL_HEADING,
    CMD_VALID,
    MissionCommand,
    write_command_frame,
)


def main() -> None:
    command = MissionCommand(
        CMD_NAVIGATE_WAYPOINT,
        CMD_VALID | CMD_DRIVE_STRAIGHT | CMD_USE_FINAL_HEADING | CMD_RED_SIDE,
        target_x_mm=-150,
        target_y_mm=1200,
        heading_cdeg=9713,
    )
    packet = command.to_frame(0x20)
    assert packet.hex(" ").upper() == (
        "A3 B3 18 20 03 0F FF 6A 04 B0 25 F1 01 69 C3"
    )
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        path = root / "uart_command.bin"
        write_command_frame(path, packet)
        assert path.read_bytes() == packet
        status_path = root / "stm32_status.json"
        status_path.write_text(json.dumps({
            "timestamp_monotonic_ns": time.monotonic_ns(),
            "flags": 3,
            "mode": 4,
        }), encoding="utf-8")
        status = load_stm_status(status_path)
        assert status.claw_visible and status.gripper_closed
        assert status.mode == 4 and status.age_ms < 250.0

        contact_path = root / "delivery_contact_pose.json"
        write_contact_pose(
            contact_path,
            PoseInput(True, -0.13, 1.09, 89.0),
            (-0.13, 1.08, 89.0),
            "red",
        )
        contact = json.loads(contact_path.read_text(encoding="utf-8"))
        assert contact["side"] == "red"
        assert contact["applied_to_localization"] is False
        assert contact["constraint_axis"] == "y"
        assert abs(contact["suggested_position_correction_m"]["x"]) < 1e-9
        assert abs(contact["suggested_position_correction_m"]["y"] + 0.01) < 1e-9
    repeated_a = MissionCommand(CMD_GRAB_CONFIRMED).to_frame(0x30)
    repeated_b = MissionCommand(CMD_GRAB_CONFIRMED).to_frame(0x31)
    assert repeated_a[4] == repeated_b[4] == CMD_GRAB_CONFIRMED
    assert repeated_a != repeated_b and repeated_a[3] + 1 == repeated_b[3]
    print("mission protocol PASS")


if __name__ == "__main__":
    main()
