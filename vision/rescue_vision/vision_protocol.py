"""F407 fixed-length UART protocol used by the closed-loop vision runner."""
from __future__ import annotations

from dataclasses import dataclass


FRAME_HEAD = bytes((0xA3, 0xB3))
FRAME_TAIL = 0xC3
FRAME_SIZE = 15
TYPE_CONFIG = 0x11
TYPE_REPORT = 0x12
TYPE_STM32_STATUS = 0x17
TYPE_MISSION_COMMAND = 0x18

IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 1024

REPORT_FOUND = 0x01
REPORT_NEAR = 0x02
REPORT_CLASS_VALID = 0x08
REPORT_DISTANCE_VALID = 0x40

CARGO_COUNT_BITS = {
    "green_supply": 0x01,
    "core_black": 0x04,
    "injured_orange": 0x10,
    "danger_cyan": 0x40,
}


def modbus_crc16(data: bytes) -> int:
    """Return the CRC-16/Modbus value used by the F407 parser."""
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def _u16be(value: int, name: str) -> bytes:
    if not 0 <= value <= 0xFFFF:
        raise ValueError(f"{name} must be in 0..65535, got {value}")
    return value.to_bytes(2, "big")


def frame(message_type: int, sequence: int, payload: bytes) -> bytes:
    if not 0 <= message_type <= 0xFF:
        raise ValueError("message_type must be a byte")
    if not 0 <= sequence <= 0xFF:
        raise ValueError("sequence must be a byte")
    if len(payload) != 8:
        raise ValueError("F407 payload must contain exactly 8 bytes")
    body = bytes((message_type, sequence)) + payload
    crc = modbus_crc16(body)
    packet = FRAME_HEAD + body + crc.to_bytes(2, "little") + bytes((FRAME_TAIL,))
    assert len(packet) == FRAME_SIZE
    return packet


def config_frame(sequence: int, color: int, start_zone: int) -> bytes:
    if color not in (0x11, 0x12):
        raise ValueError("color must be 0x11 (red) or 0x12 (blue)")
    if start_zone not in (1, 2, 3, 4):
        raise ValueError("start_zone must be in 1..4")
    return frame(TYPE_CONFIG, sequence, bytes((color, start_zone, 0, 0, 0, 0, 0, 0)))


@dataclass(frozen=True)
class NormalSupplyReport:
    """One selected cargo in a complete ``TYPE=0x12`` report."""

    x_px: int = 0
    y_px: int = 0
    distance_mm: int = 0
    found: bool = False
    near: bool = False
    distance_valid: bool = False
    cargo_class: str = "green_supply"

    def payload(self) -> bytes:
        if not self.found:
            if self.near:
                raise ValueError("near cannot be set when found is false")
            return bytes(8)
        if not 0 <= self.x_px < IMAGE_WIDTH or not 0 <= self.y_px < IMAGE_HEIGHT:
            raise ValueError(
                f"target pixel must be inside the {IMAGE_WIDTH}x{IMAGE_HEIGHT} F407 image contract"
            )
        if self.distance_valid:
            if not 1 <= self.distance_mm <= 0xFFFF:
                raise ValueError("valid distance must be in 1..65535 mm")
        elif self.distance_mm != 0:
            raise ValueError("distance_mm must be zero when distance_valid is false")
        try:
            cargo_counts = CARGO_COUNT_BITS[self.cargo_class]
        except KeyError as error:
            raise ValueError(f"unsupported cargo class: {self.cargo_class}") from error
        flags = REPORT_FOUND | REPORT_CLASS_VALID
        if self.near:
            flags |= REPORT_NEAR
        if self.distance_valid:
            flags |= REPORT_DISTANCE_VALID
        return (
            _u16be(self.x_px, "x_px")
            + _u16be(self.y_px, "y_px")
            + _u16be(self.distance_mm, "distance_mm")
            + bytes((cargo_counts, flags))
        )

    def to_frame(self, sequence: int) -> bytes:
        return frame(TYPE_REPORT, sequence, self.payload())
