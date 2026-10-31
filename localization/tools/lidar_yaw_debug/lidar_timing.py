"""STL50B2 packet-to-point time adapter and contiguous retained-sector grouping.

The supplied old capture supports byte layout and observed uint16 wrap, not
absolute sampling-time calibration. Defaults therefore NEVER qualify timing.
All I/O, calibration evidence acquisition and mounting conversion stay outside.
"""
from __future__ import annotations
from dataclasses import dataclass
import math

from .types import RawScan, TimedPoint, require_time_ns


class LidarTimingError(ValueError):
    """Discard the open sector; reset segment or create a new epoch as stated."""


@dataclass(frozen=True)
class AffineClockMap:
    anchor_device_tick: float
    anchor_host_ns: int
    ns_per_tick: float
    version: str
    qualified: bool = False
    error_bound_ns: int | None = None
    valid_device_tick_range: tuple[float, float] | None = None
    evidence_id: str = "unverified"

    def __post_init__(self):
        require_time_ns(self.anchor_host_ns)
        if not isinstance(self.qualified, bool) or not isinstance(self.evidence_id, str):
            raise ValueError("clock qualification must be explicit boolean with evidence ID")
        if isinstance(self.anchor_device_tick, bool) or isinstance(self.ns_per_tick, bool) or not math.isfinite(self.anchor_device_tick) or not math.isfinite(self.ns_per_tick) or self.ns_per_tick <= 0 or not isinstance(self.version, str) or not self.version.strip():
            raise ValueError("finite centred affine clock map and version required")
        if self.error_bound_ns is not None:
            require_time_ns(self.error_bound_ns, "clock error bound")
        if self.valid_device_tick_range is not None:
            a, b = self.valid_device_tick_range
            if not all(math.isfinite(x) for x in (a, b)) or a > b:
                raise ValueError("invalid clock calibration interval")
            object.__setattr__(self, "valid_device_tick_range", (float(a), float(b)))
        if self.qualified and (self.error_bound_ns is None or self.valid_device_tick_range is None or
                               not self.evidence_id.strip() or self.evidence_id.strip().lower() == "unverified"):
            raise ValueError("qualified map requires bounded error, finite validity interval and evidence ID")

    def map_tick(self, tick):
        if not math.isfinite(tick):
            raise LidarTimingError("nonfinite device time")
        return require_time_ns(self.anchor_host_ns + round((tick - self.anchor_device_tick) * self.ns_per_tick))

    def qualified_at(self, tick):
        return bool(self.qualified and self.valid_device_tick_range is not None and
                    self.valid_device_tick_range[0] <= tick <= self.valid_device_tick_range[1])


@dataclass(frozen=True)
class PacketTimingConfig:
    timestamp_modulus: int = 65536
    nominal_tick_ns: float = 1_000_000.
    timestamp_anchor: str = "unknown"
    timestamp_semantics_verified: bool = False
    sample_time_error_bound_ns: int | None = None
    max_total_time_error_ns: int = 2_000_000
    max_packet_tick_delta: int = 5
    max_counter_gap_ticks: int = 200
    max_receive_gap_ns: int = 200_000_000
    max_raw_angle_gap_deg: float = 3.
    max_interpacket_step_factor: float = 3.
    angle_gap_quantization_margin_deg: float = .05
    direction: int = 1
    max_points_per_packet: int = 31

    def __post_init__(self):
        if not isinstance(self.timestamp_semantics_verified, bool):
            raise ValueError("timestamp semantics verification must be an explicit boolean")
        for name in ("timestamp_modulus", "max_total_time_error_ns", "max_packet_tick_delta",
                     "max_counter_gap_ticks", "max_receive_gap_ns", "max_points_per_packet"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(name + " must be a positive integer")
        if self.timestamp_modulus < 2 or self.max_counter_gap_ticks >= self.timestamp_modulus // 2:
            raise ValueError("counter gap must be unambiguous within this modulus")
        if self.max_packet_tick_delta > self.max_counter_gap_ticks or self.max_points_per_packet > 31:
            raise ValueError("invalid packet computational bounds")
        if isinstance(self.nominal_tick_ns, bool) or not math.isfinite(self.nominal_tick_ns) or self.nominal_tick_ns <= 0:
            raise ValueError("positive finite tick scale required")
        if self.timestamp_anchor not in ("unknown", "last_sample"):
            raise ValueError("this adapter supports last-sample anchors only; do not relabel other events")
        if isinstance(self.direction, bool) or self.direction not in (-1, 1) or not math.isfinite(self.max_raw_angle_gap_deg) or not 0 < self.max_raw_angle_gap_deg < 180:
            raise ValueError("explicit scan direction and small angular-gap limit required")
        if not math.isfinite(self.max_interpacket_step_factor) or not 1 <= self.max_interpacket_step_factor <= 10 or not math.isfinite(self.angle_gap_quantization_margin_deg) or not 0 <= self.angle_gap_quantization_margin_deg < 1:
            raise ValueError("bounded relative angular gap criterion required")
        if self.sample_time_error_bound_ns is not None:
            require_time_ns(self.sample_time_error_bound_ns, "sample time error bound")


class PacketTimingAdapter:
    """N intervals between adjacent packet LAST anchors; first packet is held.

    Per-packet integer-tick quantization is correlated, not exact point cadence.
    Qualification requires a verified reconstruction error bound covering it.
    A counter or angle gap is rejected, never stretched over the next N points.
    packet_id must identify original bytes once, not a new ID on every replay.
    """
    def __init__(self, config: PacketTimingConfig, clock_map: AffineClockMap | None,
                 point_to_lidar, *, sensor_epoch: str):
        if not isinstance(config, PacketTimingConfig) or not callable(point_to_lidar) or not sensor_epoch:
            raise ValueError("config, explicit geometric conversion and sensor epoch required")
        if clock_map is not None and not isinstance(clock_map, AffineClockMap):
            raise TypeError("clock_map must be AffineClockMap or None")
        self.config, self._clock_map = config, clock_map
        self.point_to_lidar, self.sensor_epoch = point_to_lidar, sensor_epoch
        self._raw_tick = self._unwrapped = self._last_arrival = None
        self._previous_packet = None
        self.needs_new_epoch = False
        self.last_quality = {"timing_qualified": False, "reason": "NO_PACKET_ANCHOR"}

    @property
    def clock_map(self):
        return self._clock_map

    @property
    def time_map_version(self):
        return self._clock_map.version if self._clock_map else "uninitialized"

    def reset_segment(self):
        """After CRC/loss: retain known clock unwrap, discard interpolation anchor."""
        self._previous_packet = None
        self.last_quality = {"timing_qualified": False, "reason": "SEGMENT_REACQUIRE"}

    def _fields(self, event):
        if not isinstance(event, (tuple, list)) or len(event) != 6 or event[0] != "scan":
            raise LidarTimingError("expected legacy six-field scan event")
        _, speed, first, last, tick, samples = event
        if isinstance(tick, bool) or not isinstance(tick, int) or not 0 <= tick < self.config.timestamp_modulus:
            raise LidarTimingError("raw device tick outside configured modulus")
        if not 1 <= len(samples) <= self.config.max_points_per_packet:
            raise LidarTimingError("packet point count outside bounded protocol profile")
        if not all(math.isfinite(x) and 0 <= x < 360 for x in (first, last)):
            raise LidarTimingError("invalid packet angle anchors")
        return first, last, tick, samples

    def prime_arrival_clock(self, scan_events, arrival_ns):
        """Diagnostic-only clock using LAST complete scan in the first RX batch.

        Call before feed_scan when no calibrated map exists. An arrival anchor
        is NOT a sampling timestamp. This avoids assigning remaining packets in
        the same first read to the future merely because they were parsed later.
        """
        require_time_ns(arrival_ns, "arrival_ns")
        if self._clock_map is not None:
            return self._clock_map
        events = list(scan_events)
        if not events or len(events) > 4096 or self._raw_tick is not None:
            raise LidarTimingError("prime once using a bounded nonempty first batch")
        ticks = [self._fields(e)[2] for e in events]
        unwrapped = ticks[0]
        for a, b in zip(ticks, ticks[1:]):
            delta = (b - a) % self.config.timestamp_modulus
            if not 0 < delta <= self.config.max_counter_gap_ticks:
                raise LidarTimingError("ambiguous first-batch counter progression")
            unwrapped += delta
        self._clock_map = AffineClockMap(unwrapped, arrival_ns, self.config.nominal_tick_ns,
            "arrival-unverified:" + self.sensor_epoch)
        return self._clock_map

    def feed_scan(self, event, *, arrival_ns: int, packet_id: str):
        require_time_ns(arrival_ns, "arrival_ns")
        if self.needs_new_epoch:
            raise LidarTimingError("new qualified source clock epoch required")
        if not isinstance(packet_id, str) or not packet_id or len(packet_id) > 512:
            raise ValueError("bounded immutable packet ID required")
        first, last, raw, samples = self._fields(event)
        if self._clock_map is None:
            raise LidarTimingError("supply calibrated clock map or explicitly prime diagnostic arrival clock")
        if self._last_arrival is not None and (arrival_ns < self._last_arrival or
                arrival_ns - self._last_arrival > self.config.max_receive_gap_ns):
            self.needs_new_epoch = True
            raise LidarTimingError("receive discontinuity: new source epoch required")
        if self._raw_tick is None:
            unwrapped = raw
        else:
            delta = (raw - self._raw_tick) % self.config.timestamp_modulus
            if not 0 < delta <= self.config.max_counter_gap_ticks:
                self.needs_new_epoch = True
                raise LidarTimingError("reset/repeated tick/ambiguous wrap: new source epoch required")
            unwrapped = self._unwrapped + delta
        self._raw_tick, self._unwrapped, self._last_arrival = raw, unwrapped, arrival_ns
        previous = self._previous_packet
        packet_span = (self.config.direction * (last-first)) % 360
        if packet_span >= 180:
            raise LidarTimingError("packet angle span/direction outside local interpolation model")
        packet_step = packet_span / (len(samples)-1) if len(samples) > 1 else None
        self._previous_packet = (unwrapped, last, packet_id, packet_step)
        if previous is None:
            self.last_quality = {"timing_qualified": False, "reason": "FIRST_PACKET_ANCHOR_ONLY"}
            return ()
        prev_tick, prev_angle, prev_id, previous_step = previous
        gap = (self.config.direction * (first - prev_angle)) % 360
        step_reference = max(packet_step or 0., previous_step or 0.)
        relative_gap_limit = (self.config.max_interpacket_step_factor * step_reference +
                              self.config.angle_gap_quantization_margin_deg)
        if packet_id == prev_id or unwrapped - prev_tick > self.config.max_packet_tick_delta or gap > self.config.max_raw_angle_gap_deg or gap > relative_gap_limit:
            raise LidarTimingError("missing/repeated packet or angle gap; reset open sector, do not stretch time")
        count = len(samples)
        device_times = [prev_tick + (j + 1) * (unwrapped - prev_tick) / count for j in range(count)]
        times = [self._clock_map.map_tick(t) for t in device_times]
        previous_end_ns = self._clock_map.map_tick(prev_tick)
        if times[0] <= previous_end_ns or any(b <= a for a, b in zip(times, times[1:])):
            raise LidarTimingError("clock scale cannot resolve ordered point times")
        sample_bound, clock_bound = self.config.sample_time_error_bound_ns, self._clock_map.error_bound_ns
        total_bound = None if sample_bound is None or clock_bound is None else sample_bound + clock_bound
        qualified = bool(self.config.timestamp_anchor == "last_sample" and self.config.timestamp_semantics_verified
                         and total_bound is not None and total_bound <= self.config.max_total_time_error_ns
                         and all(self._clock_map.qualified_at(t) for t in device_times))
        if times[-1] > arrival_ns + self.config.max_total_time_error_ns:
            qualified = False
        output = []
        for j, (sample, time_ns) in enumerate(zip(samples, times)):
            if len(sample) != 3:
                raise LidarTimingError("malformed raw point")
            angle, distance, intensity = sample
            if not all(math.isfinite(v) for v in (angle, distance, intensity)) or not 0 <= angle < 360 or not 0 <= distance <= 65535:
                raise LidarTimingError("invalid raw point values")
            xy = tuple(self.point_to_lidar(angle, distance))
            output.append(TimedPoint(f"{self.sensor_epoch}:{packet_id}:{j}", time_ns, xy,
                packet_id, j, float(intensity), qualified, float(angle), bool(distance > 0)))
        self.last_quality = {
            "timing_qualified": qualified, "timestamp_anchor": self.config.timestamp_anchor,
            "reconstruction": "adjacent_last_anchors_N_intervals_with_bounded_quantization",
            "sample_time_error_bound_ns": sample_bound, "clock_common_error_bound_ns": clock_bound,
            "total_deterministic_time_error_bound_ns": total_bound,
            "host_arrival_is_not_acquisition": True, "time_map_version": self.time_map_version,
        }
        return tuple(output)


class TimedSectorAssembler:
    """Keep physical time order and re-phase the retained sector across zero.

    Feed all timed rays, including invalid ranges and outside-sector rays.
    Defaults are bounded diagnostic limits, not hardware qualification evidence.
    """
    def __init__(self, center_raw_deg, fov_deg, *, direction=1, sensor_epoch,
                 time_map_version, max_points=2048, max_span_ns=300_000_000,
                 max_point_gap_ns=5_000_000, max_angle_step_deg=3., max_stall_ns=20_000_000):
        if not all(math.isfinite(x) for x in (center_raw_deg, fov_deg, max_angle_step_deg)) or not 0 < fov_deg < 360 or direction not in (-1, 1):
            raise ValueError("finite proper sector and explicit direction required")
        if any(isinstance(v, bool) or not isinstance(v, int) or v <= 0 for v in (max_points, max_span_ns, max_point_gap_ns, max_stall_ns)) or max_points > 20000:
            raise ValueError("bounded positive integer sector limits required")
        if not 0 < max_angle_step_deg < 180 or not sensor_epoch or not time_map_version:
            raise ValueError("valid phase limit and source/map identity required")
        self.entry = direction * (center_raw_deg % 360) - fov_deg / 2
        self.width, self.direction = fov_deg, direction
        self.sensor_epoch, self.time_map_version = sensor_epoch, time_map_version
        self.max_points, self.max_span_ns = max_points, max_span_ns
        self.max_point_gap_ns, self.max_angle_step_deg, self.max_stall_ns = max_point_gap_ns, max_angle_step_deg, max_stall_ns
        self.reset()

    def reset(self, *, sensor_epoch=None, time_map_version=None):
        if sensor_epoch is not None: self.sensor_epoch = sensor_epoch
        if time_map_version is not None: self.time_map_version = time_map_version
        self._previous = self._phase = self._last_progress_ns = None
        self._active, self._active_turn, self._entry_bracketed = [], None, False
        self.faulted = False

    def feed(self, points):
        if self.faulted:
            raise LidarTimingError("sector assembler fault-latched; explicit reset required")
        results = []
        try:
            for point in points:
                if not isinstance(point, TimedPoint) or point.raw_angle_deg is None or not math.isfinite(point.raw_angle_deg) or not 0 <= point.raw_angle_deg < 360:
                    raise LidarTimingError("TimedPoint with original angle required")
                if self._previous is None:
                    phase = (self.direction * point.raw_angle_deg) % 360
                    self._last_progress_ns = point.time_ns
                else:
                    dt = point.time_ns - self._previous.time_ns
                    gap = (self.direction * (point.raw_angle_deg - self._previous.raw_angle_deg)) % 360
                    if dt <= 0 or dt > self.max_point_gap_ns or gap > self.max_angle_step_deg:
                        raise LidarTimingError("point time/phase gap or reversal; discard sector")
                    phase = self._phase + gap
                    if gap > 1e-9: self._last_progress_ns = point.time_ns
                    elif point.time_ns - self._last_progress_ns > self.max_stall_ns:
                        raise LidarTimingError("physical scan stalled")
                if self._active:
                    if point.time_ns - self._active[0].time_ns > self.max_span_ns:
                        raise LidarTimingError("sector acquisition span exceeded")
                    end = self.entry + self._active_turn * 360 + self.width
                    if phase >= end - 1e-9:
                        if self._entry_bracketed:
                            uid = f"{self.sensor_epoch}:sector:{self._active[0].raw_point_id}:{self._active[-1].raw_point_id}"
                            results.append(RawScan(uid, self._active[-1].time_ns, tuple(self._active),
                                self.sensor_epoch, self.time_map_version,
                                all(p.time_qualified for p in self._active)))
                        self._active, self._active_turn = [], None
                turns = (phase - self.entry) / 360
                if abs(turns - round(turns)) < 1e-12: turns = float(round(turns))
                turn = math.floor(turns)
                entrance = self.entry + turn * 360
                if phase >= entrance - 1e-9 and phase < entrance + self.width - 1e-9:
                    if not self._active:
                        self._active_turn = turn
                        self._entry_bracketed = self._previous is not None and self._phase < entrance - 1e-9
                    if len(self._active) >= self.max_points:
                        raise LidarTimingError("sector point resource bound exceeded")
                    self._active.append(point)
                self._previous, self._phase = point, phase
        except (ValueError, TypeError) as exc:
            self._active, self._active_turn, self.faulted = [], None, True
            if isinstance(exc, LidarTimingError): raise
            raise LidarTimingError(str(exc)) from exc
        return results
