#!/usr/bin/env python3
"""Upper-computer protocol extensions for the complete rescue flow.

The legacy 15-byte frame and existing 0x18 command remain the transport.  The
new opcodes are intentionally kept in this separate module so the original
mission test program is not changed while F407 firmware is being updated.
"""
from __future__ import annotations

from dataclasses import dataclass


FRAME_HEAD = bytes((0xA3, 0xB3))
FRAME_TAIL = 0xC3
FRAME_SIZE = 15
TYPE_MISSION_COMMAND = 0x18

CMD_STOP = 0x00
CMD_PAUSE = 0x01
CMD_GRAB_CONFIRMED = 0x02
CMD_NAVIGATE_WAYPOINT = 0x03
CMD_ENTER_SAFE_ZONE = 0x05
CMD_TASK_COMPLETE = 0x06
CMD_ABORT = 0x07
CMD_RETURN_CENTER = 0x08

# New complete-flow opcodes.  They are sent as TYPE=0x18 and require the
# matching F407 firmware described in docs/f407_competition_flow_handoff.md.
CMD_APPROACH_TARGET = 0x09
CMD_HOLD = 0x0A
CMD_YIELD_BACKOFF = 0x0B
CMD_ESCAPE_MANEUVER = 0x0C
CMD_RELEASE_LEFT = 0x0D
CMD_RELEASE_RIGHT = 0x0E
CMD_RELEASE_BOTH = 0x0F
CMD_DISPERSE_PILE = 0x10
CMD_CHANGE_LANE = 0x11
CMD_CARGO_AUDIT = 0x12

CMD_VALID = 1 << 0
CMD_DRIVE_STRAIGHT = 1 << 1
CMD_USE_FINAL_HEADING = 1 << 2
CMD_RED_SIDE = 1 << 3
CMD_DISTANCE_VALID = 1 << 4
CMD_CLUSTER_TARGET = 1 << 5

AUDIT_INITIAL_STASH = 1 << 0
AUDIT_DANGER_PRESENT = 1 << 1
AUDIT_UNKNOWN_PRESENT = 1 << 2
AUDIT_INJURY_MIXED = 1 << 3
AUDIT_STABLE = 1 << 4
AUDIT_DESTINATION_INJURY = 1 << 5

CARGO_NONE = 0
CARGO_GREEN = 1
CARGO_CORE = 2
CARGO_INJURED = 3
CARGO_DANGER = 4
CARGO_UNKNOWN = 5
CARGO_MIXED_MATERIAL = 6

CARGO_CLASS_CODES = {
    "": CARGO_NONE,
    "green_supply": CARGO_GREEN,
    "core_black": CARGO_CORE,
    "injured_orange": CARGO_INJURED,
    "danger_cyan": CARGO_DANGER,
    "unknown": CARGO_UNKNOWN,
    "mixed_material": CARGO_MIXED_MATERIAL,
}


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def _i16be(value: int, name: str) -> bytes:
    if not -32768 <= value <= 32767:
        raise ValueError(f"{name} must be in -32768..32767")
    return int(value).to_bytes(2, "big", signed=True)


def _u16be(value: int, name: str) -> bytes:
    if not 0 <= value <= 65535:
        raise ValueError(f"{name} must be in 0..65535")
    return int(value).to_bytes(2, "big")


def mission_frame(
    sequence: int,
    command: int,
    flags: int = CMD_VALID,
    arg_a: int = 0,
    arg_b: int = 0,
    aux_cdeg: int = 0,
) -> bytes:
    """Build one complete-flow TYPE=0x18 frame.

    ``arg_a`` and ``arg_b`` are signed int16 values whose meaning depends on
    the opcode.  ``aux_cdeg`` is the legacy absolute heading field for
    navigation commands and an opcode-specific unsigned field otherwise.
    """
    if not 0 <= sequence <= 0xFF:
        raise ValueError("sequence must be in 0..255")
    if not 0 <= command <= 0xFF or not 0 <= flags <= 0xFF:
        raise ValueError("command and flags must be bytes")
    if flags & CMD_CLUSTER_TARGET and command != CMD_APPROACH_TARGET:
        raise ValueError("CLUSTER_TARGET is only valid for APPROACH_TARGET")
    payload = (
        bytes((command, flags))
        + _i16be(arg_a, "arg_a")
        + _i16be(arg_b, "arg_b")
        + _u16be(aux_cdeg, "aux_cdeg")
    )
    body = bytes((TYPE_MISSION_COMMAND, sequence)) + payload
    crc = crc16_modbus(body)
    return FRAME_HEAD + body + crc.to_bytes(2, "little") + bytes((FRAME_TAIL,))


@dataclass(frozen=True)
class CargoAuditPayload:
    """Compact left/right cargo audit sent before movement or release."""

    left_class: str = ""
    right_class: str = ""
    left_count: int = 0
    right_count: int = 0
    total_count: int = 0
    danger_present: bool = False
    unknown_present: bool = False
    injury_mixed: bool = False
    stable: bool = False
    initial_stash: bool = False
    destination_injury: bool = False
    audit_id: int = 0

    def frame(self, sequence: int) -> bytes:
        left = CARGO_CLASS_CODES.get(self.left_class, CARGO_UNKNOWN)
        right = CARGO_CLASS_CODES.get(self.right_class, CARGO_UNKNOWN)
        if not 0 <= self.left_count <= 3 or not 0 <= self.right_count <= 3:
            raise ValueError("each claw count must be in 0..3")
        if not 0 <= self.total_count <= 255 or not 0 <= self.audit_id <= 255:
            raise ValueError("audit count and id must be bytes")
        flags = 0
        flags |= AUDIT_INITIAL_STASH if self.initial_stash else 0
        flags |= AUDIT_DANGER_PRESENT if self.danger_present else 0
        flags |= AUDIT_UNKNOWN_PRESENT if self.unknown_present else 0
        flags |= AUDIT_INJURY_MIXED if self.injury_mixed else 0
        flags |= AUDIT_STABLE if self.stable else 0
        flags |= AUDIT_DESTINATION_INJURY if self.destination_injury else 0
        packed_counts = self.left_count | (self.right_count << 2)
        payload = bytes((
            CMD_CARGO_AUDIT,
            CMD_VALID,
            left,
            right,
            packed_counts,
            flags,
            self.audit_id,
            self.total_count,
        ))
        body = bytes((TYPE_MISSION_COMMAND, sequence)) + payload
        crc = crc16_modbus(body)
        return FRAME_HEAD + body + crc.to_bytes(2, "little") + bytes((FRAME_TAIL,))


def approach_target_frame(
    sequence: int, x_px: int, y_px: int, *, cluster_target: bool = False
) -> bytes:
    flags = CMD_VALID | (CMD_CLUSTER_TARGET if cluster_target else 0)
    return mission_frame(sequence, CMD_APPROACH_TARGET, flags, x_px, y_px)


def hold_frame(sequence: int) -> bytes:
    return mission_frame(sequence, CMD_HOLD, CMD_VALID)


def pause_frame(sequence: int) -> bytes:
    return mission_frame(sequence, CMD_PAUSE, CMD_VALID)


def grab_frame(sequence: int, red_side: bool) -> bytes:
    flags = CMD_VALID | (CMD_RED_SIDE if red_side else 0)
    return mission_frame(sequence, CMD_GRAB_CONFIRMED, flags)


def navigation_frame(
    sequence: int,
    remaining_m: float,
    heading_deg: float,
    red_side: bool,
) -> bytes:
    if remaining_m < 0.0:
        raise ValueError("remaining distance must be nonnegative")
    flags = CMD_VALID | CMD_DRIVE_STRAIGHT | CMD_USE_FINAL_HEADING | CMD_DISTANCE_VALID
    flags |= CMD_RED_SIDE if red_side else 0
    return mission_frame(
        sequence,
        CMD_NAVIGATE_WAYPOINT,
        flags,
        max(0, min(32767, round(remaining_m * 1000.0))),
        0,
        round(heading_deg % 360.0 * 100.0) % 36000,
    )


def enter_safe_zone_frame(sequence: int, heading_deg: float, red_side: bool) -> bytes:
    flags = CMD_VALID | CMD_DRIVE_STRAIGHT | CMD_USE_FINAL_HEADING
    flags |= CMD_RED_SIDE if red_side else 0
    return mission_frame(
        sequence,
        CMD_ENTER_SAFE_ZONE,
        flags,
        0,
        0,
        round(heading_deg % 360.0 * 100.0) % 36000,
    )


def task_complete_frame(sequence: int, red_side: bool) -> bytes:
    return mission_frame(
        sequence,
        CMD_TASK_COMPLETE,
        CMD_VALID | (CMD_RED_SIDE if red_side else 0),
    )


def return_center_frame(
    sequence: int,
    remaining_m: float,
    heading_deg: float,
    red_side: bool,
) -> bytes:
    flags = CMD_VALID | CMD_DRIVE_STRAIGHT | CMD_USE_FINAL_HEADING | CMD_DISTANCE_VALID
    flags |= CMD_RED_SIDE if red_side else 0
    return mission_frame(
        sequence,
        CMD_RETURN_CENTER,
        flags,
        max(0, min(32767, round(remaining_m * 1000.0))),
        0,
        round(heading_deg % 360.0 * 100.0) % 36000,
    )


def yield_backoff_frame(sequence: int, distance_m: float) -> bytes:
    return mission_frame(
        sequence,
        CMD_YIELD_BACKOFF,
        CMD_VALID,
        -max(0, min(32767, round(distance_m * 1000.0))),
        0,
    )


def escape_maneuver_frame(
    sequence: int,
    spin_deg: int,
    lateral_m: float,
) -> bytes:
    return mission_frame(
        sequence,
        CMD_ESCAPE_MANEUVER,
        CMD_VALID,
        max(-32768, min(32767, int(spin_deg))),
        max(-32768, min(32767, round(lateral_m * 1000.0))),
    )


def release_frame(sequence: int, side: str) -> bytes:
    commands = {
        "left": CMD_RELEASE_LEFT,
        "right": CMD_RELEASE_RIGHT,
        "both": CMD_RELEASE_BOTH,
    }
    try:
        command = commands[side]
    except KeyError as error:
        raise ValueError("release side must be left, right or both") from error
    return mission_frame(sequence, command, CMD_VALID)


def disperse_frame(sequence: int) -> bytes:
    return mission_frame(sequence, CMD_DISPERSE_PILE, CMD_VALID)


def change_lane_frame(sequence: int, lateral_m: float, forward_m: float) -> bytes:
    return mission_frame(
        sequence,
        CMD_CHANGE_LANE,
        CMD_VALID,
        max(-32768, min(32767, round(lateral_m * 1000.0))),
        max(-32768, min(32767, round(forward_m * 1000.0))),
    )
