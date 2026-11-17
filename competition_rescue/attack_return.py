"""Opening: wheel retreat, lateral translation, reverse, face field centre."""
from __future__ import annotations
from dataclasses import dataclass
import math


def opponent_heading_deg(side: str) -> float:
    return 270.0 if side == 'red' else 90.0


def centre_heading_deg(x_m: float, y_m: float) -> float:
    return math.degrees(math.atan2(-y_m, -x_m)) % 360.0


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
    after_back_xy_m: tuple[float, float] | None = None
    centre_yaw_deg: float | None = None

    @property
    def return_yaw_deg(self):
        return self.centre_yaw_deg if self.turning and self.centre_yaw_deg is not None else opponent_heading_deg(self.side)

    @property
    def finite_back(self): return self.index == 0

    @property
    def turning(self): return self.index == 3

    @property
    def finished_motion(self): return self.index >= 4

    @property
    def start_same_side(self):
        return (self.start_zone <= 2) == (self.side == 'red')

    def finish_retreat(self, x_m, y_m):
        assert self.finite_back
        self.after_back_xy_m = (x_m, y_m)
        self.index = 1
        self.arrival_latched = False

    def advance(self):
        assert self.index in {1, 2, 3}
        self.index += 1
        self.arrival_latched = False

    def payload(self, x_m, y_m):
        yaw = round(opponent_heading_deg(self.side)*100)
        if self.finite_back:
            return -100, 0, yaw
        if self.turning:
            if self.centre_yaw_deg is None:
                self.centre_yaw_deg = centre_heading_deg(x_m, y_m)
            return 0, 0, round(self.centre_yaw_deg*100)%36000
        assert self.index in {1, 2}
        dx = self.return_point[0]-x_m if self.index == 1 else 0.0
        dy = self.return_point[1]-y_m if self.index == 2 else 0.0
        distance = math.hypot(dx, dy)
        self.arrival_latched |= distance <= self.tolerance_m+1e-9
        return (0 if self.arrival_latched else min(5000,round(distance*1000)),
                round(math.degrees(math.atan2(dy,dx))*100), yaw)
