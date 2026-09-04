"""Mission-level frames layered on the common 15-byte F407 protocol."""
from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

from .vision_protocol import TYPE_MISSION_COMMAND, frame


CMD_STOP = 0x00
CMD_GRAB_CONFIRMED = 0x02
CMD_NAVIGATE_WAYPOINT = 0x03
CMD_ALIGN_SAFE_ZONE = 0x04
CMD_ENTER_SAFE_ZONE = 0x05
CMD_TASK_COMPLETE = 0x06
CMD_ABORT = 0x07

CMD_VALID = 1 << 0
CMD_DRIVE_STRAIGHT = 1 << 1
CMD_USE_FINAL_HEADING = 1 << 2
CMD_RED_SIDE = 1 << 3

STM_CLAW_VISIBLE = 1 << 0
STM_GRIPPER_CLOSED = 1 << 1
STM_MOTORS_ACTIVE = 1 << 2
STM_AUTO_APPROACH = 1 << 3
STM_FAULT = 1 << 7


def _i16be(value: int, name: str) -> bytes:
    if not -32768 <= value <= 32767:
        raise ValueError(f"{name} must be in -32768..32767")
    return int(value).to_bytes(2, "big", signed=True)


def _u16be(value: int, name: str) -> bytes:
    if not 0 <= value <= 65535:
        raise ValueError(f"{name} must be in 0..65535")
    return int(value).to_bytes(2, "big")


@dataclass(frozen=True)
class MissionCommand:
    command: int
    flags: int = CMD_VALID
    target_x_mm: int = 0
    target_y_mm: int = 0
    heading_cdeg: int = 0

    def payload(self) -> bytes:
        if not 0 <= self.command <= 0xFF or not 0 <= self.flags <= 0xFF:
            raise ValueError("command and flags must be bytes")
        if not 0 <= self.heading_cdeg < 36000:
            raise ValueError("heading_cdeg must be in 0..35999")
        return (
            bytes((self.command, self.flags))
            + _i16be(self.target_x_mm, "target_x_mm")
            + _i16be(self.target_y_mm, "target_y_mm")
            + _u16be(self.heading_cdeg, "heading_cdeg")
        )

    def to_frame(self, sequence: int) -> bytes:
        return frame(TYPE_MISSION_COMMAND, sequence, self.payload())


@dataclass(frozen=True)
class Stm32Status:
    flags: int = 0
    mode: int = 0
    camera_pitch_cdeg: int = 0
    acknowledged_sequence: int = 0
    fault_code: int = 0
    age_ms: float = float("inf")

    @property
    def claw_visible(self) -> bool:
        return bool(self.flags & STM_CLAW_VISIBLE)

    @property
    def gripper_closed(self) -> bool:
        return bool(self.flags & STM_GRIPPER_CLOSED)

    @property
    def fault(self) -> bool:
        return bool(self.flags & STM_FAULT) or self.fault_code != 0

    @classmethod
    def from_json(cls, data: dict) -> "Stm32Status":
        return cls(
            flags=int(data.get("flags", 0)),
            mode=int(data.get("mode", 0)),
            camera_pitch_cdeg=int(data.get("camera_pitch_cdeg", 0)),
            acknowledged_sequence=int(data.get("acknowledged_sequence", 0)),
            fault_code=int(data.get("fault_code", 0)),
            age_ms=float(data.get("age_ms", float("inf"))),
        )


def write_command_frame(path: Path, packet: bytes) -> None:
    if len(packet) != 15:
        raise ValueError("relayed UART command must be exactly 15 bytes")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_bytes(packet)
    os.replace(temporary, path)
