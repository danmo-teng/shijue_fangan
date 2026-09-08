#!/usr/bin/env python3
"""Protocol regression checks for the F407 UART motion-debug branch."""
from __future__ import annotations

from f407_motion_protocol import (
    FRAME_HEAD,
    FRAME_TAIL,
    MOTION_CMD_TURN_REL,
    MOTION_STATE_RUNNING,
    MSG_MOTION_STATUS,
    MSG_ODOM,
    StreamParser,
    crc16_modbus,
    motion_move_frame,
    motion_stop_frame,
    motion_turn_frame,
)


def inbound(message_type: int, sequence: int, payload: bytes) -> bytes:
    body = bytes((message_type, sequence)) + payload
    return FRAME_HEAD + body + crc16_modbus(body).to_bytes(2, "little") + bytes((FRAME_TAIL,))


def main() -> None:
    assert motion_turn_frame(0, 90.0, 300).hex(" ").upper() == (
        "A3 B3 17 00 01 00 23 28 01 2C 00 00 A6 E4 C3"
    )
    assert motion_move_frame(3, 90.0, 1000, 300).hex(" ").upper() == (
        "A3 B3 17 03 02 00 23 28 03 E8 01 2C B2 09 C3"
    )
    assert motion_stop_frame(5).hex(" ").upper() == (
        "A3 B3 17 05 00 00 00 00 00 00 00 00 FF 18 C3"
    )

    odom = inbound(MSG_ODOM, 7, bytes((0, 1, 0, 2, 0, 3, 10, 7)))
    status = inbound(
        MSG_MOTION_STATUS, 4,
        bytes((MOTION_STATE_RUNNING, MOTION_CMD_TURN_REL, 0, 100, 0, 200, 3, 9)),
    )
    seen = []
    parser = StreamParser(
        on_odom=lambda value: seen.append(("odom", value)),
        on_motion_status=lambda value: seen.append(("status", value)),
    )
    parser.feed(b"noise" + odom[:5])
    parser.feed(odom[5:] + status)
    assert seen[0][0] == "odom" and seen[0][1]["m3_count"] == 3
    assert seen[1][0] == "status" and seen[1][1]["command_sequence"] == 9
    assert parser.stats["frames"] == 2
    assert parser.stats["crc_errors"] == 0
    print("F407 motion protocol PASS")


if __name__ == "__main__":
    main()
