#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from protocol import (  # noqa: E402
    CARGO_MIXED_MATERIAL,
    CMD_CARGO_AUDIT,
    CMD_VALID,
    CargoAuditPayload,
    FRAME_SIZE,
    crc16_modbus,
    mission_frame,
)


def main() -> None:
    packet = mission_frame(7, CMD_CARGO_AUDIT, CMD_VALID, 1, 6, 0x1234)
    assert len(packet) == FRAME_SIZE
    assert packet[:4] == bytes((0xA3, 0xB3, 0x18, 7))
    assert packet[4] == CMD_CARGO_AUDIT
    expected = crc16_modbus(packet[2:12])
    assert packet[12:14] == expected.to_bytes(2, "little")
    assert packet[-1] == 0xC3

    audit = CargoAuditPayload(
        left_class="mixed_material",
        right_class="danger_cyan",
        left_count=2,
        right_count=1,
        total_count=3,
        danger_present=True,
        stable=True,
        audit_id=9,
    )
    audit_packet = audit.frame(8)
    assert audit_packet[4] == CMD_CARGO_AUDIT
    assert audit_packet[6] == CARGO_MIXED_MATERIAL
    assert audit_packet[7] == 4
    assert audit_packet[8] == 0x06  # left=2, right=1
    assert audit_packet[9] & (1 << 1)  # danger flag
    assert audit_packet[9] & (1 << 4)  # stable flag
    assert audit_packet[10] == 9
    assert audit_packet[11] == 3
    print("competition protocol PASS")


if __name__ == "__main__":
    main()
