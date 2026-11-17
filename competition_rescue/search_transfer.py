"""Direct transfers between search stations 1/3 and failed-target context."""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from attack_return import centre_heading_deg


@dataclass
class SearchTransferRoute:
    source_point: int
    target_point: int
    target_xy_m: tuple[float, float]
    yaw_deg: float
    tolerance_m: float = .05
    index: int = 0
    arrival_latched: bool = False
    centre_yaw_deg: float | None = None
    waypoints: tuple[tuple[float, float], ...] = field(init=False)

    def __post_init__(self) -> None:
        if self.source_point not in {1, 3} or self.target_point not in {1, 3}:
            raise ValueError("search stations are 1 and 3 only")
        self.waypoints = (self.target_xy_m,)

    @property
    def search_ready(self) -> bool:
        return self.index >= 2

    @property
    def turning(self) -> bool:
        return self.index == 1

    def finish_leg(self) -> None:
        self.index += 1

    def payload(self, x_m: float, y_m: float) -> tuple[int, int, int]:
        if self.turning:
            if self.centre_yaw_deg is None:
                self.centre_yaw_deg = centre_heading_deg(x_m, y_m)
            return 0, 0, round(self.centre_yaw_deg*100)%36000
        dx, dy = self.target_xy_m[0]-x_m, self.target_xy_m[1]-y_m
        distance = math.hypot(dx, dy)
        self.arrival_latched = self.arrival_latched or distance <= self.tolerance_m+1e-9
        return (0 if self.arrival_latched else min(5000, round(distance*1000)),
                round(math.degrees(math.atan2(dy, dx))*100), round(self.yaw_deg*100)%36000)


@dataclass(frozen=True)
class FailedSearchTarget:
    visit: int
    track_ids: tuple[int, ...]
    class_name: str
    bbox: tuple[int, int, int, int] | None
    observer: tuple[float, float, float, int] | None
    first_bbox: tuple[int, int, int, int] | None = None
    first_observer: tuple[float, float, float, int] | None = None

    def matches(self, item, observer: tuple[float, float, float, int] | None) -> bool:
        if item.class_name != self.class_name:
            return False
        if item.track_id in self.track_ids:
            return True
        # Pixel boxes are view-local, never field coordinates. Only associate
        # a replacement ID when pitch, viewing direction and car place agree.
        if observer is None:
            return False
        x, y, yaw, pitch = observer
        for bbox, view in ((self.bbox, self.observer), (self.first_bbox, self.first_observer)):
            if bbox is None or view is None:
                continue
            old_x, old_y, old_yaw, old_pitch = view
            yaw_delta = (yaw-old_yaw+180)%360-180
            if pitch != old_pitch or abs(yaw_delta) > 5 or math.hypot(x-old_x,y-old_y) > .25:
                continue
            bx, by, bw, bh = bbox
            area = max(1, bw*bh)
            ratio = max(1,item.area_px)/area
            if (.25 <= ratio <= 4 and math.dist(item.center_px, (bx+bw/2,by+bh/2)) <=
                    max(40.,math.sqrt(area),math.sqrt(max(1,item.area_px)))):
                return True
        return False
