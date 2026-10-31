"""Only immutable time/scan types needed by the reference lidar reader."""
from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np

def require_time_ns(value, name="time_ns"):
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or not 0 <= value < 2**63:
        raise ValueError(f"{name} must be a nonnegative int64 monotonic timestamp")
    return int(value)


def frozen_array(value, shape=None):
    a = np.array(value, dtype=float, copy=True)
    if shape is not None and a.shape != shape:
        raise ValueError(f"expected shape {shape}, got {a.shape}")
    if not np.all(np.isfinite(a)):
        raise ValueError("nonfinite array")
    a.flags.writeable = False
    return a


@dataclass(frozen=True)
class TimedPoint:
    raw_point_id: str
    time_ns: int
    point_lidar_m: tuple[float, float]
    packet_id: str = ""
    point_index: int = 0
    intensity: float = 0.0
    time_qualified: bool = False
    raw_angle_deg: float | None = None
    valid_return: bool = True

    def __post_init__(self):
        require_time_ns(self.time_ns)
        object.__setattr__(self,'point_lidar_m',tuple(map(float,frozen_array(self.point_lidar_m,(2,)))))
        if type(self.valid_return) is not bool or type(self.time_qualified) is not bool:
            raise ValueError('point validity/time qualification must be explicit bool')
        if not self.raw_point_id or not math.isfinite(self.intensity):
            raise ValueError("point identity and finite intensity required")


@dataclass(frozen=True)
class RawScan:
    scan_id: str
    reference_ns: int
    points: tuple[TimedPoint, ...]
    sensor_epoch: str
    time_map_version: str
    timing_qualified: bool = False

    def __post_init__(self):
        require_time_ns(self.reference_ns)
        object.__setattr__(self, "points", tuple(self.points))
        if not self.points or not self.scan_id:
            raise ValueError("nonempty scan with identity required")
        times = [p.time_ns for p in self.points]
        if any(b <= a for a, b in zip(times, times[1:])) or self.reference_ns != times[-1]:
            raise ValueError("scan must preserve increasing point times and frozen raw last-point reference")
        if len({p.raw_point_id for p in self.points}) != len(self.points):
            raise ValueError("duplicate physical point")

    @property
    def first_point_ns(self):
        return self.points[0].time_ns

    @property
    def last_point_ns(self):
        return self.points[-1].time_ns
