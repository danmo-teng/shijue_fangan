"""STL50B2 protocol functions, without Tk imports.
Command/CRC helpers retain the user reference implementation. The streaming
parser additionally handles CRC-valid short status frames observed in the
supplied 2026-10-28 recording and resynchronizes one byte after a bad CRC.
Raw packet angles are unchanged; mounting conversion is handled by static.polar_points.
"""

def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x4D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def command_frame(mode, value=0):
    if not 0 <= value <= 0xFFFFFFFF:
        raise ValueError("参数必须是 0–4294967295 的整数")
    frame = bytearray((0x54, mode, 0x04))
    frame.extend(value.to_bytes(4, "little"))
    frame.append(crc8(frame))
    return bytes(frame)


def validate_value(mode, value):
    if mode in (0xA2, 0xA5) and not 720 <= value <= 3600:
        raise ValueError("转速参数必须为 720–3600（2–10 Hz）")
    if mode == 0xA9 and value not in (0, 1):
        raise ValueError("A9 参数只能是 0 或 1")
    if mode == 0xA4 and value < 50:
        raise ValueError("A4 波特率不能低于 50")
    if mode == 0xBE and value not in range(4):
        raise ValueError("BE 参数只能是 0、1、2、3")
    return value


def parse_frames(buffer, *, include_status=False, on_frame=None):
    """Consume a mutable bytearray, preserving existing scan/ack/crc tuples.

    Captured 5-byte A1 and 6-byte 1F frames have opaque status payloads, NOT a
    verified timestamp meaning. Legacy callers receive them in the ``ack``
    compatibility category; opt-in callers receive ``('status', mode, payload)``.
    A1 valid ordinary length-0/1 acknowledgements retain their original payload
    interpretation. Other A1 short/length-prefixed ambiguity requires protocol
    evidence; no payload meaning is guessed here.
    """
    frames = []
    initial_size = len(buffer)
    def emit(event, offset, raw_frame):
        frames.append(event)
        if on_frame is not None:
            on_frame(event, offset, raw_frame)
    while True:
        start = buffer.find(b"\x54")
        if start < 0:
            buffer.clear()
            break
        del buffer[:start]
        frame_offset = initial_size - len(buffer)
        if len(buffer) < 3:
            break
        mode = buffer[1]
        short_status = False
        if mode == 0xA1:
            ordinary_size = 4 + buffer[2]
            ordinary_small = (ordinary_size <= 5 and len(buffer) >= ordinary_size and
                              crc8(buffer[:ordinary_size-1]) == buffer[ordinary_size-1])
            if ordinary_small:
                points, size = 0, ordinary_size
            elif len(buffer) < 5:
                break
            elif crc8(buffer[:4]) == buffer[4]:
                points, size, short_status = 0, 5, True
            else:
                points, size = 0, ordinary_size
        elif mode == 0x1F:
            points, size, short_status = 0, 6, True
        elif mode & 0xE0 == 0x20 and mode & 0x1F:
            points, size = mode & 0x1F, 11 + 3 * (mode & 0x1F)
        elif 0xA0 <= mode <= 0xBF:
            points, size = 0, 4 + buffer[2]
        else:
            del buffer[0]
            continue
        if len(buffer) < size:
            break
        frame = bytes(buffer[:size])
        if crc8(frame[:-1]) != frame[-1]:
            # The alleged length may itself be corrupt. Consuming the whole
            # candidate can swallow a subsequent good scan or an A1 short frame.
            del buffer[0]
            emit(("crc",), frame_offset, frame)
            continue
        del buffer[:size]
        if short_status:
            emit(("status" if include_status else "ack", mode, frame[2:-1]), frame_offset, frame)
        elif points:
            tail = 6 + 3 * points
            speed = int.from_bytes(frame[2:4], "little")
            start_angle = int.from_bytes(frame[4:6], "little") / 100
            end_angle = int.from_bytes(frame[tail:tail + 2], "little") / 100
            delta = (end_angle - start_angle + 540) % 360 - 180
            step = delta / max(points - 1, 1)
            samples = []
            for index in range(points):
                offset = 6 + 3 * index
                samples.append(((start_angle + index * step) % 360,
                                int.from_bytes(frame[offset:offset + 2], "little"),
                                frame[offset + 2]))
            emit(("scan", speed, start_angle, end_angle,
                  int.from_bytes(frame[tail + 2:tail + 4], "little"), samples), frame_offset, frame)
        else:
            emit(("ack", mode, frame[3:-1]), frame_offset, frame)
    return frames
