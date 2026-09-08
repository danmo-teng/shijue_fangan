#!/usr/bin/env python3
"""Protocol adapter for F407 ``feat/uart-motion-debug``."""
from __future__ import annotations

import math
from collections import deque
from typing import Callable


FRAME_HEAD = bytes((0xA3, 0xB3))
FRAME_TAIL = 0xC3
FRAME_SIZE = 15
PAYLOAD_SIZE = 8
MSG_ODOM = 0x15
MSG_MOTION_COMMAND = 0x17
MSG_MOTION_STATUS = 0x18

MOTION_CMD_STOP = 0x00
MOTION_CMD_TURN_REL = 0x01
MOTION_CMD_MOVE_DISTANCE = 0x02
MOTION_TURN_NEGATIVE = 0x01
MOTION_MOVE_FIELD_FRAME = 0x01

MOTION_STATE_IDLE = 0x00
MOTION_STATE_RUNNING = 0x01
MOTION_STATE_DONE = 0x02
MOTION_STATE_FAULT = 0x03
MOTION_STATE_STOPPED = 0x04

MOTION_STATUS_IMU_READY = 0x01
MOTION_STATUS_ODOM_VALID = 0x02
MOTION_STATUS_MOTOR_FAULT = 0x04
MOTION_MAX_DISTANCE_MM = 10_000
MOTION_MIN_SPEED_MM_S = 50
MOTION_MAX_SPEED_MM_S = 700
MOTION_MAX_TURN_CDEG = 36_000

STATE_NAMES = {
    MOTION_STATE_IDLE: "IDLE",
    MOTION_STATE_RUNNING: "RUNNING",
    MOTION_STATE_DONE: "DONE",
    MOTION_STATE_FAULT: "FAULT",
    MOTION_STATE_STOPPED: "STOPPED",
}
COMMAND_NAMES = {
    MOTION_CMD_STOP: "STOP",
    MOTION_CMD_TURN_REL: "TURN_REL",
    MOTION_CMD_MOVE_DISTANCE: "MOVE_DISTANCE",
}


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if crc & 1 else crc >> 1
    return crc & 0xFFFF


def build_frame(message_type: int, sequence: int, payload: bytes) -> bytes:
    if message_type not in (MSG_MOTION_COMMAND,):
        raise ValueError("this adapter only builds TYPE=0x17 motion commands")
    if not 0 <= sequence <= 0xFF or len(payload) != PAYLOAD_SIZE:
        raise ValueError("invalid sequence or payload length")
    body = bytes((message_type, sequence)) + payload
    return FRAME_HEAD + body + crc16_modbus(body).to_bytes(2, "little") + bytes((FRAME_TAIL,))


def _speed(speed_mm_s: int) -> int:
    if speed_mm_s == 0:
        return 0
    if not MOTION_MIN_SPEED_MM_S <= speed_mm_s <= MOTION_MAX_SPEED_MM_S:
        raise ValueError(f"speed must be 0 or {MOTION_MIN_SPEED_MM_S}..{MOTION_MAX_SPEED_MM_S} mm/s")
    return int(speed_mm_s)


def motion_stop_frame(sequence: int) -> bytes:
    return build_frame(MSG_MOTION_COMMAND, sequence, bytes(8))


def motion_turn_frame(sequence: int, angle_deg: float, speed_mm_s: int = 300) -> bytes:
    if not math.isfinite(angle_deg) or not -360.0 <= angle_deg <= 360.0:
        raise ValueError("angle must be in -360..360 degrees")
    angle_cdeg = int(round(abs(angle_deg) * 100.0))
    if not 0 < angle_cdeg <= MOTION_MAX_TURN_CDEG:
        raise ValueError("angle must be non-zero and no more than 360 degrees")
    payload = bytes((MOTION_CMD_TURN_REL,
                     MOTION_TURN_NEGATIVE if angle_deg < 0.0 else 0))
    payload += angle_cdeg.to_bytes(2, "big") + _speed(speed_mm_s).to_bytes(2, "big") + bytes(2)
    return build_frame(MSG_MOTION_COMMAND, sequence, payload)


def motion_move_frame(sequence: int, direction_deg: float, distance_mm: int,
                      speed_mm_s: int = 300, field_frame: bool = False) -> bytes:
    if not math.isfinite(direction_deg) or not 0.0 <= direction_deg < 360.0:
        raise ValueError("direction must be in 0..360 degrees")
    if not 0 < distance_mm <= MOTION_MAX_DISTANCE_MM:
        raise ValueError(f"distance must be in 1..{MOTION_MAX_DISTANCE_MM} mm")
    direction_cdeg = int(round(direction_deg * 100.0)) % 36000
    payload = bytes((MOTION_CMD_MOVE_DISTANCE,
                     MOTION_MOVE_FIELD_FRAME if field_frame else 0))
    payload += direction_cdeg.to_bytes(2, "big")
    payload += int(distance_mm).to_bytes(2, "big")
    payload += _speed(speed_mm_s).to_bytes(2, "big")
    return build_frame(MSG_MOTION_COMMAND, sequence, payload)


def parse_frame(frame: bytes) -> tuple[int, int, bytes]:
    if len(frame) != FRAME_SIZE or frame[:2] != FRAME_HEAD or frame[-1] != FRAME_TAIL:
        raise ValueError("invalid frame envelope")
    if crc16_modbus(frame[2:12]) != int.from_bytes(frame[12:14], "little"):
        raise ValueError("invalid frame CRC")
    if frame[2] not in (MSG_ODOM, MSG_MOTION_STATUS):
        raise ValueError("unexpected F407 frame type")
    return frame[2], frame[3], frame[4:12]


def parse_odometry(frame: bytes) -> dict[str, int]:
    message_type, sequence, payload = parse_frame(frame)
    if message_type != MSG_ODOM:
        raise ValueError("not an odometry frame")
    dt_ms = payload[6]
    if dt_ms == 0:
        raise ValueError("odometry period cannot be zero")
    return {
        "sequence": sequence,
        "m1_count": int.from_bytes(payload[0:2], "big"),
        "m2_count": int.from_bytes(payload[2:4], "big"),
        "m3_count": int.from_bytes(payload[4:6], "big"),
        "dt_ms": dt_ms,
        "status": payload[7],
    }


def parse_motion_status(frame: bytes) -> dict[str, int | bool | str]:
    message_type, sequence, payload = parse_frame(frame)
    if message_type != MSG_MOTION_STATUS:
        raise ValueError("not a motion status frame")
    health = payload[6]
    return {
        "sequence": sequence,
        "state": payload[0],
        "state_name": STATE_NAMES.get(payload[0], "UNKNOWN"),
        "command": payload[1],
        "command_name": COMMAND_NAMES.get(payload[1], "UNKNOWN"),
        "progress": int.from_bytes(payload[2:4], "big"),
        "remaining": int.from_bytes(payload[4:6], "big"),
        "health": health,
        "imu_ready": bool(health & MOTION_STATUS_IMU_READY),
        "odom_valid": bool(health & MOTION_STATUS_ODOM_VALID),
        "motor_fault": bool(health & MOTION_STATUS_MOTOR_FAULT),
        "command_sequence": payload[7],
    }


class StreamParser:
    """Byte-stream parser for mixed F407 ODOM and motion-status frames."""

    def __init__(self, on_odom: Callable[[dict[str, int]], None] | None = None,
                 on_motion_status: Callable[[dict[str, int | bool | str]], None] | None = None) -> None:
        self.on_odom = on_odom
        self.on_motion_status = on_motion_status
        self.buffer = bytearray()
        self.latest_status: dict[str, int | bool | str] | None = None
        self.stats = {
            "bytes": 0,
            "frames": 0,
            "odom_frames": 0,
            "motion_status_frames": 0,
            "crc_errors": 0,
            "malformed": 0,
            "odom_sequence_gaps": 0,
            "status_sequence_gaps": 0,
        }
        self._last_odom_sequence: int | None = None
        self._last_status_sequence: int | None = None

    def feed(self, data: bytes) -> None:
        self.stats["bytes"] += len(data)
        self.buffer.extend(data)
        while True:
            start = self.buffer.find(FRAME_HEAD)
            if start < 0:
                if self.buffer and self.buffer[-1] == FRAME_HEAD[0]:
                    del self.buffer[:-1]
                else:
                    self.buffer.clear()
                return
            if start:
                del self.buffer[:start]
            if len(self.buffer) < FRAME_SIZE:
                return
            candidate = bytes(self.buffer[:FRAME_SIZE])
            try:
                message_type, sequence, _ = parse_frame(candidate)
            except ValueError as error:
                if "CRC" in str(error):
                    self.stats["crc_errors"] += 1
                else:
                    self.stats["malformed"] += 1
                del self.buffer[:1]
                continue
            del self.buffer[:FRAME_SIZE]
            self.stats["frames"] += 1
            if message_type == MSG_ODOM:
                if self._last_odom_sequence is not None:
                    gap = (sequence - self._last_odom_sequence) & 0xFF
                    if gap > 1:
                        self.stats["odom_sequence_gaps"] += gap - 1
                self._last_odom_sequence = sequence
                self.stats["odom_frames"] += 1
                if self.on_odom is not None:
                    self.on_odom(parse_odometry(candidate))
            else:
                if self._last_status_sequence is not None:
                    gap = (sequence - self._last_status_sequence) & 0xFF
                    if gap > 1:
                        self.stats["status_sequence_gaps"] += gap - 1
                self._last_status_sequence = sequence
                self.stats["motion_status_frames"] += 1
                self.latest_status = parse_motion_status(candidate)
                if self.on_motion_status is not None:
                    self.on_motion_status(self.latest_status)
