#!/usr/bin/env python3
from __future__ import annotations

import sys
import tempfile
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT / "vision"))

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
        target_x_mm=0,
        target_y_mm=950,
        heading_cdeg=9000,
    )
    packet = command.to_frame(0x20)
    assert packet.hex(" ").upper() == (
        "A3 B3 18 20 03 0F 00 00 03 B6 23 28 2E 20 C3"
    )
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "uart_command.bin"
        write_command_frame(path, packet)
        assert path.read_bytes() == packet
    repeated_a = MissionCommand(CMD_GRAB_CONFIRMED).to_frame(0x30)
    repeated_b = MissionCommand(CMD_GRAB_CONFIRMED).to_frame(0x31)
    assert repeated_a[4] == repeated_b[4] == CMD_GRAB_CONFIRMED
    assert repeated_a != repeated_b and repeated_a[3] + 1 == repeated_b[3]
    print("mission protocol PASS")


if __name__ == "__main__":
    main()
