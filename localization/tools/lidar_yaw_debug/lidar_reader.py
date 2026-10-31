"""Bounded bytes-only lidar processor. Never opens serial or sends commands.

No old mounting file is loaded. A receive-delay profile may establish a fresh
host-monotonic anchor ONLY when its event semantics and deterministic bounds
have independent calibration evidence. Ordinary arrival priming stays unqualified.
"""
from __future__ import annotations
from dataclasses import asdict, dataclass
import math

from .protocol import parse_frames
from .lidar_timing import (AffineClockMap, LidarTimingError, PacketTimingAdapter,
                           PacketTimingConfig, TimedSectorAssembler)
from .types import require_time_ns


@dataclass(frozen=True)
class ReceiveDelayCalibration:
    verified: bool = False
    evidence_id: str = "unverified"
    delay_min_ns: int | None = None
    delay_max_ns: int | None = None
    host_timestamp_error_bound_ns: int | None = None
    anchor_tick_error_bound_ticks: float | None = None
    clock_rate_error_ppm_bound: float | None = None
    validity_ns: int | None = None

    def __post_init__(self):
        if not isinstance(self.verified, bool) or not isinstance(self.evidence_id, str):
            raise ValueError("receive calibration verification must be explicit boolean")
        for name in ("delay_min_ns", "delay_max_ns", "host_timestamp_error_bound_ns", "validity_ns"):
            value = getattr(self, name)
            if value is not None: require_time_ns(value, name)
        for name in ("anchor_tick_error_bound_ticks", "clock_rate_error_ppm_bound"):
            value = getattr(self, name)
            if value is not None and (isinstance(value, bool) or not math.isfinite(value) or value < 0):
                raise ValueError(name + " must be a nonnegative deterministic bound")
        if self.delay_min_ns is not None and self.delay_max_ns is not None and self.delay_min_ns > self.delay_max_ns:
            raise ValueError("receive delay lower bound exceeds upper bound")
        if self.verified and (any(getattr(self, name) is None for name in (
            "delay_min_ns", "delay_max_ns", "host_timestamp_error_bound_ns",
            "anchor_tick_error_bound_ticks", "clock_rate_error_ppm_bound", "validity_ns")) or
            not self.evidence_id.strip() or self.evidence_id.strip().lower() == "unverified" or self.validity_ns <= 0):
            raise ValueError("verified receive calibration requires every bound, positive validity and evidence")


def unverified_example_config():
    """JSON-serializable placeholders, NOT current mounting or measured timing."""
    return {
        "bearing_sign": 1, "bearing_zero_deg": 0., "center_raw_deg": 0., "fov_deg": 145.,
        "physical_direction": 1,
        "packet_timing": asdict(PacketTimingConfig()),
        "receive_delay_calibration": asdict(ReceiveDelayCalibration()),
        "sector_limits": {"max_points": 2048, "max_span_ns": 300_000_000,
                          "max_point_gap_ns": 5_000_000, "max_angle_step_deg": 3., "max_stall_ns": 20_000_000},
        "max_rx_chunk_bytes": 65536, "max_buffer_bytes": 131072,
        "max_batch_span_ns": 200_000_000,
        "note": "UNVERIFIED placeholders; replace geometry and calibrate time before dynamic localization",
    }


class LidarStream:
    def __init__(self, config: dict, *, sensor_epoch: str, clock_map: AffineClockMap | None = None):
        if not isinstance(config, dict) or not isinstance(sensor_epoch, str) or not sensor_epoch:
            raise ValueError("explicit configuration and connection/source epoch required")
        names = ("bearing_sign", "bearing_zero_deg", "center_raw_deg", "fov_deg", "physical_direction")
        if any(name not in config for name in names):
            raise ValueError("explicit bearing sign/zero, retained centre/width and physical direction required")
        sign, direction = config["bearing_sign"], config["physical_direction"]
        if isinstance(sign, bool) or sign not in (-1, 1) or isinstance(direction, bool) or direction not in (-1, 1):
            raise ValueError("bearing sign and physical direction are separate explicit +/-1 fields")
        for name in ("bearing_zero_deg", "center_raw_deg", "fov_deg"):
            if isinstance(config[name], bool) or not math.isfinite(config[name]):
                raise ValueError(name + " must be finite")
        self.sensor_epoch = sensor_epoch
        self._bearing_sign, self._bearing_zero = sign, math.radians(config["bearing_zero_deg"])
        packet_config = dict(config.get("packet_timing", {}))
        if "direction" in packet_config and packet_config["direction"] != direction:
            raise ValueError("packet and physical scan directions conflict")
        packet_config["direction"] = direction
        self.packet_config = PacketTimingConfig(**packet_config)
        self.receive_calibration = ReceiveDelayCalibration(**config.get("receive_delay_calibration", {}))
        if clock_map is not None and self.receive_calibration.verified:
            raise ValueError("choose supplied current-boot clock map OR receive-delay calibration, not both")
        if self.receive_calibration.verified and (not self.packet_config.timestamp_semantics_verified or
                self.packet_config.timestamp_anchor != "last_sample" or self.packet_config.sample_time_error_bound_ns is None):
            raise ValueError("qualified receive anchor requires verified last-sample semantics and reconstruction bound")
        self.max_rx_chunk_bytes = config.get("max_rx_chunk_bytes", 65536)
        self.max_buffer_bytes = config.get("max_buffer_bytes", 131072)
        self.max_batch_span_ns = config.get("max_batch_span_ns", 200_000_000)
        for name in ("max_rx_chunk_bytes", "max_buffer_bytes", "max_batch_span_ns"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(name + " must be positive integer")
        if self.max_rx_chunk_bytes > 1_048_576 or self.max_buffer_bytes > 2_097_152 or self.max_buffer_bytes < self.max_rx_chunk_bytes:
            raise ValueError("reader exceeds bounded computational memory limits")
        self.adapter = PacketTimingAdapter(self.packet_config, clock_map, self._polar, sensor_epoch=sensor_epoch)
        self.sector = TimedSectorAssembler(config["center_raw_deg"], config["fov_deg"], direction=direction,
            sensor_epoch=sensor_epoch, time_map_version=self.adapter.time_map_version,
            **config.get("sector_limits", {}))
        self._buffer, self._byte_count, self._last_arrival = bytearray(), 0, None
        self.needs_new_epoch = False
        self.last_error = ""
        self.stats = {k: 0 for k in ("rx_batches", "rx_bytes", "scan_packets", "status_packets", "ack_packets",
                                     "crc_errors", "timing_rejections", "sector_rejections", "scans_emitted")}

    def _polar(self, raw_angle_deg, distance_mm):
        bearing = self._bearing_sign * math.radians(raw_angle_deg) + self._bearing_zero
        radius = distance_mm * .001
        return radius * math.cos(bearing), radius * math.sin(bearing)

    def _first_batch_ticks(self, events):
        ticks = [self.adapter._fields(e)[2] for e in events]
        unwrapped = [ticks[0]]
        for a, b in zip(ticks, ticks[1:]):
            delta = (b-a) % self.packet_config.timestamp_modulus
            if not 0 < delta <= self.packet_config.max_counter_gap_ticks:
                raise LidarTimingError("first RX batch has ambiguous counter epoch")
            unwrapped.append(unwrapped[-1] + delta)
        return unwrapped

    def _make_receive_clock(self, events, arrival_ns):
        ticks = self._first_batch_ticks(events)
        history_ns = (ticks[-1]-ticks[0]) * self.packet_config.nominal_tick_ns
        if history_ns > self.max_batch_span_ns:
            raise LidarTimingError("initial receive batch spans too much acquisition history")
        cal = self.receive_calibration
        if not cal.verified:
            return self.adapter.prime_arrival_clock(events, arrival_ns)
        midpoint = (cal.delay_min_ns + cal.delay_max_ns) // 2
        radius = max(midpoint-cal.delay_min_ns, cal.delay_max_ns-midpoint)
        drift = max(float(cal.validity_ns), history_ns) * cal.clock_rate_error_ppm_bound * 1e-6
        bound = math.ceil(radius + cal.host_timestamp_error_bound_ns +
                          cal.anchor_tick_error_bound_ticks * self.packet_config.nominal_tick_ns + drift)
        if bound + self.packet_config.sample_time_error_bound_ns > self.packet_config.max_total_time_error_ns:
            raise LidarTimingError("declared receive/drift/reconstruction bound exceeds time budget")
        anchor_ns = require_time_ns(arrival_ns-midpoint, "calibrated receive anchor")
        clock = AffineClockMap(ticks[-1], anchor_ns, self.packet_config.nominal_tick_ns,
            "receive-calibrated:"+self.sensor_epoch, True, bound,
            (ticks[0], ticks[-1]+cal.validity_ns/self.packet_config.nominal_tick_ns), cal.evidence_id)
        self.adapter = PacketTimingAdapter(self.packet_config, clock, self._polar, sensor_epoch=self.sensor_epoch)
        return clock

    def _fault(self, message):
        self.needs_new_epoch, self.last_error = True, str(message)
        self._buffer.clear()
        self.sector.reset()
        raise LidarTimingError(str(message))

    def feed(self, data: bytes, arrival_ns: int):
        require_time_ns(arrival_ns, "arrival_ns")
        if self.needs_new_epoch:
            raise LidarTimingError("reader requires explicit new source/clock epoch")
        if not isinstance(data, (bytes, bytearray)):
            raise TypeError("bytes or bytearray required")
        if self._last_arrival is not None and arrival_ns < self._last_arrival:
            self._fault("receive timestamp reversal")
        if len(data) > self.max_rx_chunk_bytes or len(self._buffer)+len(data) > self.max_buffer_bytes:
            self._fault("bounded reader input/buffer budget exceeded")
        self._last_arrival = arrival_ns
        self.stats["rx_batches"] += 1; self.stats["rx_bytes"] += len(data)
        self._buffer.extend(data); self._byte_count += len(data)
        base_offset = self._byte_count-len(self._buffer)
        records = []
        parse_frames(self._buffer, include_status=True,
                     on_frame=lambda event, offset, raw: records.append((event, base_offset+offset)))
        scans = [event for event, _ in records if event[0] == "scan"]
        if scans and self.adapter.clock_map is None:
            try:
                clock = self._make_receive_clock(scans, arrival_ns)
                self.sector.reset(time_map_version=clock.version)
            except (ValueError, TypeError) as exc:
                self._fault(exc)
        output = []
        for event, offset in records:
            kind = event[0]
            if kind == "crc":
                self.stats["crc_errors"] += 1
                self.adapter.reset_segment(); self.sector.reset()
                continue
            if kind in ("ack", "status"):
                self.stats["ack_packets" if kind == "ack" else "status_packets"] += 1
                continue
            if kind != "scan": continue
            self.stats["scan_packets"] += 1
            try:
                points = self.adapter.feed_scan(event, arrival_ns=arrival_ns,
                    packet_id=f"byte:{offset}")
                clock = self.adapter.clock_map
                if clock.qualified and not clock.qualified_at(self.adapter._unwrapped):
                    self._fault("finite calibrated clock interval expired; explicit new epoch/requalification required")
            except LidarTimingError as exc:
                self.stats["timing_rejections"] += 1
                self.last_error = str(exc)
                if self.needs_new_epoch or self.adapter.needs_new_epoch:
                    self._fault(exc)
                self.adapter.reset_segment(); self.sector.reset()
                continue
            try:
                output.extend(self.sector.feed(points))
            except LidarTimingError as exc:
                self.stats["sector_rejections"] += 1
                self.last_error = str(exc)
                self.adapter.reset_segment(); self.sector.reset()
        self.stats["scans_emitted"] += len(output)
        return output
