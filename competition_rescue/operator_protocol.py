"""One-shot operator feedback to the existing C++ UART owner.

True means the local IPC datagram was delivered, not that firmware played it.
Firmware TYPE 0x20 support is a separate lower-computer implementation task.
"""
from __future__ import annotations

import socket

try:
    from .protocol import FRAME_HEAD, FRAME_TAIL, crc16_modbus, mission_frame, CMD_ABORT
except ImportError:
    from protocol import FRAME_HEAD, FRAME_TAIL, crc16_modbus, mission_frame, CMD_ABORT

TYPE_OPERATOR_BEEP = 0x20


def beep_frame(count: int = 1, long: bool = False, sequence: int = 0) -> bytes:
    if count not in (1, 2, 3, 4) or not 0 <= sequence <= 255:
        raise ValueError("beep count must be 1..4 and sequence 0..255")
    body = bytes((TYPE_OPERATOR_BEEP, sequence, 1, count, int(bool(long)), 0, 0, 0, 0, 0))
    return FRAME_HEAD + body + crc16_modbus(body).to_bytes(2, "little") + bytes((FRAME_TAIL,))


def _send(socket_name: str, frame: bytes) -> bool:
    if not socket_name.startswith("@") or not 1 < len(socket_name.encode()) < 108:
        raise ValueError("operator socket requires a short @abstract name")
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as client:
            client.setblocking(False)
            return client.sendto(frame, "\0" + socket_name[1:]) == len(frame)
    except OSError:
        return False


def send_beep(socket_name: str, count: int = 1, long: bool = False, sequence: int = 0) -> bool:
    return _send(socket_name, beep_frame(count, long, sequence))


def send_abort(socket_name: str, sequence: int = 0) -> bool:
    return _send(socket_name, mission_frame(sequence, CMD_ABORT))
