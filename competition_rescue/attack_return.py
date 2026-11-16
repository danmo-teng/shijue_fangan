"""Opening attack: wheel retreat, turn tail to station 1, direct reverse."""
from __future__ import annotations

from dataclasses import dataclass
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
    after_back_xy_m: tuple[float, float] | None = None
    tail_heading_deg: float | None = None

    @property
    def return_yaw_deg(self) -> float:
        return opponent_heading_deg(self.side) if self.tail_heading_deg is None else self.tail_heading_deg

    @property
    def finite_back(self) -> bool:
        return self.index == 0

    @property
    def turning(self) -> bool:
        return self.index == 1

    @property
    def start_same_side(self) -> bool:
        return (self.start_zone <= 2) == (self.side == "red")

    @property
    def finished_motion(self) -> bool:
        return self.index >= 3

    def finish_retreat(self, x_m: float, y_m: float) -> None:
        """Called only for this wheel action's DONE and an available position."""
        assert self.finite_back
        self.after_back_xy_m = x_m, y_m
        dx, dy = self.return_point[0]-x_m, self.return_point[1]-y_m
        if math.hypot(dx, dy) <= self.tolerance_m:
            self.arrival_latched = True
            self.index = 3  # Already at the station; no undefined atan2(0, 0) turn.
            return
        # Front points away from the destination so rearward travel reaches it.
        self.tail_heading_deg = (math.degrees(math.atan2(dy, dx)) + 180.0) % 360.0
        self.index = 1

    def advance(self) -> None:
        assert self.index in {1, 2}
        self.index += 1
        self.arrival_latched = False

    def payload(self, x_m: float, y_m: float) -> tuple[int, int, int]:
        if self.finite_back:
            return -150, 0, round(opponent_heading_deg(self.side)*100)
        assert self.tail_heading_deg is not None
        yaw = round(self.tail_heading_deg*100) % 36000
        if self.turning:
            return 0, 0, yaw
        assert self.index == 2
        dx, dy = self.return_point[0]-x_m, self.return_point[1]-y_m
        distance = math.hypot(dx, dy)
        self.arrival_latched = self.arrival_latched or distance <= self.tolerance_m+1e-9
        return (0 if self.arrival_latched else min(5000, round(distance*1000)),
                round(math.degrees(math.atan2(dy, dx))*100), yaw)
