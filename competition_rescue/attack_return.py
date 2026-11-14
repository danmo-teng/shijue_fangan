"""Opening-attack return waypoints after facing the opponent's safe zone."""
from __future__ import annotations

from dataclasses import dataclass, field
import math


def opponent_heading_deg(side: str) -> float:
    return 270.0 if side == "red" else 90.0


@dataclass
class AttackReturnRoute:
    side: str
    start_zone: int
    release_xy_m: tuple[float, float]
    release_yaw_deg: float
    return_point: tuple[float, float]
    tolerance_m: float = .05
    index: int = 0
    arrival_latched: bool = False
    waypoints: tuple[tuple[float, float], ...] = field(init=False)

    def __post_init__(self) -> None:
        # Actual release yaw remains evidence, never the definition of left/back.
        angle = math.radians(self.return_yaw_deg)
        forward = math.cos(angle), math.sin(angle)
        left = -forward[1], forward[0]
        back10 = tuple(p - .10*f for p, f in zip(self.release_xy_m, forward))
        left50 = tuple(p + .50*l for p, l in zip(back10, left))
        # Reach the return point's longitudinal level before moving right.
        along = sum((r-p)*f for r, p, f in zip(self.return_point, left50, forward))
        back_to_row = tuple(p + along*f for p, f in zip(left50, forward))
        self.waypoints = back10, left50, back_to_row, self.return_point

    @property
    def return_yaw_deg(self) -> float:
        return opponent_heading_deg(self.side)

    @property
    def start_same_side(self) -> bool:
        return (self.start_zone <= 2) == (self.side == "red")

    @property
    def finished_motion(self) -> bool:
        return self.index >= len(self.waypoints)

    def advance(self) -> None:
        self.index += 1
        self.arrival_latched = False

    def payload(self, x_m: float, y_m: float) -> tuple[int, int, int]:
        target = self.waypoints[self.index]
        dx, dy = target[0]-x_m, target[1]-y_m
        distance = math.hypot(dx, dy)
        self.arrival_latched = self.arrival_latched or distance <= self.tolerance_m + 1e-9
        heading = math.degrees(math.atan2(dy, dx))
        return (0 if self.arrival_latched else min(5000, round(distance*1000)),
                round(heading*100), round(self.return_yaw_deg*100))
