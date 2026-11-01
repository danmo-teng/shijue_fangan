"""Reference codec only: no UART, SDK, or robot actions on import/execution."""
from __future__ import annotations
import struct

MAGIC = b"\xd4\x4d"
VERSION = 1
MAX_PAYLOAD = 160
ID_REQ, ID_REPLY, SYNC_REQ, SYNC_REPLY = 1, 2, 3, 4
CONFIG_REQ, CONFIG_REPLY, WHEEL, GYRO, STATS_REQ, STATS_REPLY = 5, 6, 7, 8, 9, 10
ERROR_REPLY = 127
STATS_NAMES = (
    "requests", "crc_errors", "bad_headers", "request_queue_drops",
    "partial_timeouts", "wheel_measurements", "wheel_invalid", "wheel_skipped",
    "gyro_enqueued", "gyro_repeated", "gyro_publish_slots_skipped", "enqueue_rejected",
    "extended_started", "extended_completed", "timed_coalesced_or_expired",
    "max_wheel_capture_us", "max_wheel_end_to_dma_return_us", "max_queue_us",
    "max_yaw_receive_gap_us", "max_rate_receive_gap_us", "max_wheel_interval_us",
    "sync_replies", "identity_replies", "max_oldest_gyro_receive_to_dma_return_us",
)


def crc16(data: bytes) -> int:
    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0xA001 if value & 1 else 0)
    return value


def packet(kind: int, sequence: int, payload: bytes, version: int = 1) -> bytes:
    if not 0 <= kind <= 255 or not 0 <= sequence <= 65535 or len(payload) > MAX_PAYLOAD:
        raise ValueError("invalid extended packet")
    body = struct.pack(">BBHH", version, kind, len(payload), sequence) + payload
    return MAGIC + body + crc16(body).to_bytes(2, "little") + b"\xe4"


def parse(raw: bytes) -> dict:
    if len(raw) < 11 or raw[:2] != MAGIC or raw[-1] != 0xE4:
        raise ValueError("bad extended envelope")
    version, kind, length, sequence = struct.unpack(">BBHH", raw[2:8])
    if version != VERSION or length > MAX_PAYLOAD or len(raw) != length + 11:
        raise ValueError("bad version or length")
    if crc16(raw[2:-3]) != int.from_bytes(raw[-3:-1], "little"):
        raise ValueError("bad CRC")
    return {"version": version, "type": kind, "sequence": sequence, "payload": raw[8:-3]}


def request(kind: int, nonce: int, sequence: int = 0, *, enable: bool = False) -> bytes:
    if kind not in (ID_REQ, SYNC_REQ, CONFIG_REQ, STATS_REQ):
        raise ValueError("unsupported request")
    payload = struct.pack(">Q", nonce)
    if kind == CONFIG_REQ:
        payload += bytes((int(enable), 1, 0, 0))
    return packet(kind, sequence, payload)


def decode(raw: bytes) -> dict:
    frame = parse(raw)
    p, kind = frame.pop("payload"), frame["type"]
    if kind == WHEEL:
        values = struct.unpack(">16sIIQQiiIIIiiiIII", p)
        names = ("boot_epoch", "reference_epoch", "measurement_id", "t0_us", "t1_us",
                 "forward_um_s", "left_um_s", "raw1_u32", "raw2_u32", "raw3_u32",
                 "delta1", "delta2", "delta3", "flags", "source_uncertainty_us", "capture_window_us")
    elif kind == GYRO:
        values = struct.unpack(">16sIIqQIIIiQIIIQ", p)
        names = ("boot_epoch", "reference_epoch", "publication_id", "yaw_mdeg",
                 "yaw_receive_us", "yaw_source_id", "yaw_flags", "yaw_uncertainty_us",
                 "rate_mdps", "rate_receive_us", "rate_source_id", "rate_flags",
                 "rate_uncertainty_us", "publish_us")
    elif kind == ID_REPLY:
        values = struct.unpack(">QH16sIQHBBBBIIII12sIIH32s20sBH", p)
        names = ("nonce", "epoch_fault", "boot_epoch", "reference_epoch", "capabilities",
                 "board", "mode", "service", "protocol_major", "protocol_minor",
                 "tick_hz", "uart_baud", "clock_flags", "drift_bound_ppm", "mcu_uid",
                 "device_idcode", "cpu_id", "flash_kib", "firmware_id", "git_commit",
                 "source_dirty", "max_payload")
    elif kind == SYNC_REPLY:
        values = struct.unpack(">Q16sQQIH", p)
        names = ("nonce", "boot_epoch", "receive_event_us", "response_prepare_us",
                 "tick_resolution_us", "epoch_valid")
    elif kind == CONFIG_REPLY:
        values = struct.unpack(">Q16sIHBBIIII", p)
        names = ("nonce", "boot_epoch", "reference_epoch", "status", "profile", "enabled",
                 "uart_baud", "wheel_period_us", "gyro_period_us", "required_baud")
    elif kind == STATS_REPLY:
        values = struct.unpack(">Q16sIQ" + "I" * 24, p)
        names = ("nonce", "boot_epoch", "reference_epoch", "snapshot_us") + STATS_NAMES
    elif kind == ERROR_REPLY:
        values, names = struct.unpack(">QHBB", p), ("nonce", "status", "request_type", "supported_version")
    else:
        return {**frame, "payload_hex": p.hex()}
    frame.update(zip(names, values))
    for name in ("boot_epoch", "mcu_uid", "firmware_id", "git_commit"):
        if name in frame:
            frame[name] = frame[name].hex()
    return frame


def offset_interval_ns(host_before_ns: int, host_after_ns: int, sync: dict) -> tuple[int, int]:
    """MCU_ns - host_ns bounds from causality; no path symmetry assumption.

    Not a drift/extrapolation guarantee. Unknown oscillator or source timing
    bounds must still fail the upper computer's eligibility gate.
    """
    if host_after_ns < host_before_ns or sync["response_prepare_us"] < sync["receive_event_us"]:
        raise ValueError("non-monotonic synchronization exchange")
    quantum = sync["tick_resolution_us"] * 1000
    lower = sync["response_prepare_us"] * 1000 - host_after_ns - quantum
    upper = sync["receive_event_us"] * 1000 - host_before_ns + quantum
    if lower > upper:
        raise ValueError("inconsistent synchronization exchange")
    return lower, upper


def stream_frames(data: bytes) -> list[bytes]:
    """Decode a complete captured batch, skipping valid legacy 15-byte frames.

    This is an offline helper, not a stateful live UART client.
    """
    result, cursor = [], 0
    while cursor + 2 <= len(data):
        if data[cursor:cursor+2] == b"\xa3\xb3" and cursor+15 <= len(data):
            legacy = data[cursor:cursor+15]
            if legacy[-1] == 0xC3 and crc16(legacy[2:12]) == int.from_bytes(legacy[12:14], "little"):
                cursor += 15
                continue
        if data[cursor:cursor+2] == MAGIC and cursor+8 <= len(data):
            length = int.from_bytes(data[cursor+4:cursor+6], "big") + 11
            raw = data[cursor:cursor+length]
            try:
                parse(raw)
            except ValueError:
                pass
            else:
                result.append(raw)
                cursor += length
                continue
        cursor += 1
    return result


class MixedStream:
    """RDK live receiver added to the audited reference codec.

    Consume each CRC-valid container atomically; an embedded magic in either
    payload is never treated as a second sample. No I/O or source-time inference.
    """
    def __init__(self, on_legacy, on_extended):
        self.on_legacy, self.on_extended = on_legacy, on_extended
        self.buffer = bytearray()
        self.partial_since_ns = None
        self.stats = dict(uart_rx_bytes=0, legacy_frames=0, extended_frames=0,
                          legacy_ack4=0, mixed_crc_errors=0, mixed_malformed=0,
                          discarded_bytes=0, partial_timeouts=0, buffer_overflows=0)

    def expire(self, now_ns):
        if self.partial_since_ns is not None and now_ns-self.partial_since_ns > 200_000_000:
            self.stats["partial_timeouts"] += 1
            self.stats["discarded_bytes"] += len(self.buffer)
            self.buffer.clear()
            self.partial_since_ns = None

    def feed(self, data, receive_ns):
        self.expire(receive_ns)
        self.stats["uart_rx_bytes"] += len(data)
        if len(self.buffer)+len(data)>16384:
            self.stats["buffer_overflows"] += 1
            self.stats["discarded_bytes"] += len(self.buffer)+len(data)
            self.buffer.clear();self.partial_since_ns=None
            return
        self.buffer.extend(data)
        while self.buffer:
            starts=[i for magic in (b"\xa3\xb3",MAGIC) if (i:=self.buffer.find(magic))>=0]
            if not starts:
                keep=1 if self.buffer[-1] in (0xa3,0xd4) else 0
                self.stats["discarded_bytes"] += len(self.buffer)-keep
                if keep: self.buffer[:]=self.buffer[-1:]
                else: self.buffer.clear()
                break
            start=min(starts)
            if start:
                del self.buffer[:start];self.stats["discarded_bytes"]+=start
                self.partial_since_ns=None
            legacy=self.buffer[:2]==b"\xa3\xb3"
            if legacy and self.buffer[:4]==b"\xa3\xb3\x01\xc3":
                del self.buffer[:4];self.stats["legacy_ack4"]+=1
                self.partial_since_ns=None
                continue
            if legacy:
                size=15
            else:
                if len(self.buffer)<8: break
                length=int.from_bytes(self.buffer[4:6],"big")
                if self.buffer[2]!=VERSION or length>MAX_PAYLOAD:
                    del self.buffer[0];self.stats["mixed_malformed"]+=1
                    self.partial_since_ns=None
                    continue
                size=length+11
            if len(self.buffer)<size: break
            raw=bytes(self.buffer[:size])
            if legacy:
                valid=raw[-1]==0xc3 and crc16(raw[2:12])==int.from_bytes(raw[12:14],"little")
            else:
                try: parse(raw);valid=True
                except ValueError:valid=False
            if not valid:
                del self.buffer[0];self.stats["mixed_crc_errors"]+=1
                self.partial_since_ns=None
                continue
            del self.buffer[:size]
            self.partial_since_ns=None
            self.stats["legacy_frames" if legacy else "extended_frames"]+=1
            if legacy:self.on_legacy(raw,receive_ns)
            else:self.on_extended(raw,receive_ns)
        if self.buffer:
            if self.partial_since_ns is None:self.partial_since_ns=receive_ns
        else:self.partial_since_ns=None
