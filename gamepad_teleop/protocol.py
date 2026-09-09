#!/usr/bin/env python3
"""The fixed-length UART protocol used by the F407 Gamepad firmware.

The wire format intentionally lives outside the GUI and input-device code so
it can be tested without evdev, a serial device, or a display server.
"""

from __future__ import annotations

from dataclasses import dataclass

FRAME_HEAD = bytes((0xA3, 0xB3))
FRAME_TAIL = 0xC3
FRAME_SIZE = 15
PAYLOAD_SIZE = 8

MSG_MOTION_COMMAND = 0x19
MSG_MOTION_STATUS = 0x1A
MSG_ODOM = 0x15

CMD_STOP = 0x00
CMD_TELEOP = 0x06

FLAG_VALID = 0x01
FLAG_ACK_REQUIRED = 0x08
FLAG_TELEOP_ENABLE = 0x20

BUTTON_CLAW_CLOSE = 0x01
BUTTON_CLAW_OPEN = 0x02
BUTTON_LIFT_UP = 0x04
BUTTON_LIFT_DOWN = 0x08
BUTTON_A = 0x10
BUTTON_B = 0x20
BUTTON_X = 0x40
BUTTON_Y = 0x80

STATE_IDLE = 0
STATE_RUNNING = 1
STATE_DONE = 2
STATE_ERROR = 3
STATE_STOPPED = 4


@dataclass(frozen=True)
class Frame:
    """A validated fixed-length frame."""

    message_type: int
    sequence: int
    payload: bytes


@dataclass(frozen=True)
class MotionStatus:
    """Decoded F407 Gamepad status (TYPE=0x1A)."""

    frame_sequence: int
    command_sequence: int
    state: int
    fault: int
    command: int
    progress: int
    heading_cdeg: int


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if crc & 1 else crc >> 1
    return crc & 0xFFFF


def _signed_byte(value: int) -> int:
    if not -100 <= value <= 100:
        raise ValueError("teleop axis must be in -100..100")
    return value & 0xFF


def build_frame(message_type: int, sequence: int, payload: bytes) -> bytes:
    if not 0 <= message_type <= 0xFF:
        raise ValueError("message type must be a byte")
    if not 0 <= sequence <= 0xFF:
        raise ValueError("sequence must be a byte")
    if len(payload) != PAYLOAD_SIZE:
        raise ValueError("payload must contain eight bytes")
    body = bytes((message_type, sequence)) + payload
    crc = crc16_modbus(body)
    return FRAME_HEAD + body + crc.to_bytes(2, "little") + bytes((FRAME_TAIL,))


def teleop_frame(
    sequence: int,
    forward: int,
    left: int,
    yaw: int,
    camera: int,
    buttons: int,
    speed_percent: int,
    enabled: bool,
) -> bytes:
    if not 0 <= buttons <= 0xFF:
        raise ValueError("buttons must be a byte")
    if not 0 <= speed_percent <= 100:
        raise ValueError("speed percentage must be in 0..100")
    flags = FLAG_VALID | FLAG_ACK_REQUIRED
    if enabled:
        flags |= FLAG_TELEOP_ENABLE
    payload = bytes(
        (
            CMD_TELEOP,
            flags,
            _signed_byte(forward),
            _signed_byte(left),
            _signed_byte(yaw),
            _signed_byte(camera),
            buttons,
            speed_percent,
        )
    )
    return build_frame(MSG_MOTION_COMMAND, sequence, payload)


def stop_frame(sequence: int) -> bytes:
    return build_frame(
        MSG_MOTION_COMMAND,
        sequence,
        bytes((CMD_STOP, FLAG_VALID | FLAG_ACK_REQUIRED, 0, 0, 0, 0, 0, 0)),
    )


def parse_frame(frame: bytes, accepted_types: set[int] | None = None) -> Frame:
    if len(frame) != FRAME_SIZE:
        raise ValueError("invalid frame length")
    if frame[:2] != FRAME_HEAD or frame[-1] != FRAME_TAIL:
        raise ValueError("invalid frame header or tail")
    if accepted_types is not None and frame[2] not in accepted_types:
        raise ValueError("unexpected message type")
    expected = crc16_modbus(frame[2:12])
    actual = int.from_bytes(frame[12:14], "little")
    if expected != actual:
        raise ValueError("CRC mismatch")
    return Frame(frame[2], frame[3], bytes(frame[4:12]))


def decode_motion_status(frame: Frame) -> MotionStatus:
    if frame.message_type != MSG_MOTION_STATUS:
        raise ValueError("not a motion-status frame")
    payload = frame.payload
    progress = int.from_bytes(payload[4:6], "big", signed=True)
    heading = int.from_bytes(payload[6:8], "big")
    if payload[1] > STATE_STOPPED or heading >= 36000:
        raise ValueError("invalid motion-status values")
    return MotionStatus(
        frame_sequence=frame.sequence,
        command_sequence=payload[0],
        state=payload[1],
        fault=payload[2],
        command=payload[3],
        progress=progress,
        heading_cdeg=heading,
    )


class StreamParser:
    """Resynchronizing parser for the shared byte stream."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.frames = 0
        self.crc_errors = 0
        self.malformed_frames = 0

    def feed(self, data: bytes) -> list[Frame]:
        if data:
            self._buffer.extend(data)
        result: list[Frame] = []
        while True:
            start = self._buffer.find(FRAME_HEAD)
            if start < 0:
                if self._buffer and self._buffer[-1] == FRAME_HEAD[0]:
                    self._buffer[:] = self._buffer[-1:]
                else:
                    self._buffer.clear()
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < FRAME_SIZE:
                break
            candidate = bytes(self._buffer[:FRAME_SIZE])
            try:
                frame = parse_frame(candidate)
            except ValueError as error:
                if "CRC" in str(error):
                    self.crc_errors += 1
                else:
                    self.malformed_frames += 1
                # Drop one byte and search for the next possible header. This
                # handles a corrupted frame containing another A3 B3 pair.
                del self._buffer[0]
                continue
            del self._buffer[:FRAME_SIZE]
            self.frames += 1
            result.append(frame)
        return result


def state_name(state: int) -> str:
    return {
        STATE_IDLE: "IDLE",
        STATE_RUNNING: "RUNNING",
        STATE_DONE: "DONE",
        STATE_ERROR: "ERROR",
        STATE_STOPPED: "STOPPED",
    }.get(state, "UNKNOWN")
