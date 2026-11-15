"""Closed-claw deflection geometry; no device IO or new distance calibration."""
from __future__ import annotations

from dataclasses import dataclass
import math

VIEW = 0
PULL = 1
STAGE = 2
CARRYING = 0x02
END_AT_ORIGINAL = 0x04
PRE_YAW_CDEG = 3500


@dataclass(frozen=True)
class DeflectionPlan:
    distance_mm: int
    heading_cdeg: int
    pre_yaw_cdeg: int  # Positive: pre-turn left, then pull right.
    end_at_original: bool


def choose_plan(side: str, xy: tuple[float, float], yaw_deg: float,
                distance_mm: int) -> DeflectionPlan:
    """Choose away from the whole own-zone rectangle, in field coordinates.

    The calibrated distance is vehicle travel, not a measured object centre.
    Its endpoint is only a direction estimate; no ground-position claim.
    """
    angle = math.radians(yaw_deg)
    ux, uy = math.cos(angle), math.sin(angle)
    left = (-uy, ux)
    x, y = xy[0]+ux*distance_mm/1000, xy[1]+uy*distance_mm/1000
    ymin, ymax = (1.14, 1.50) if side == 'red' else (-1.50, -1.14)
    qx, qy = min(.33, max(-.33, x)), min(ymax, max(ymin, y))
    away = (x-qx, y-qy)
    lateral = away[0]*left[0]+away[1]*left[1]
    if abs(lateral) < 1e-9:
        # Directly in front/behind: head toward the nearer outer side of the
        # rectangle. At exact centre choose field +X deterministically.
        desired_x = 1 if x >= 0 else -1
        lateral = desired_x*left[0]
    pull_left = lateral > 0
    behind = (qx-x)*ux+(qy-y)*uy < -1e-9
    return DeflectionPlan(distance_mm, round(yaw_deg*100)%36000,
                          -PRE_YAW_CDEG if pull_left else PRE_YAW_CDEG, behind)


def nearest_blocker(cargo, ranges: dict[int, int], *, target_id: int,
                    target_distance_mm: int, centre_px: int, half_width_px: int):
    """Only a visible danger in front of the aligned useful target qualifies."""
    candidates = [item for item in cargo
                  if item.track_id != target_id and item.visible and not item.inside_safe_zone
                  and item.class_name == 'danger_cyan' and item.track_id in ranges
                  and 0 <= ranges[item.track_id] < target_distance_mm
                  and item.bbox[0] <= centre_px+half_width_px
                  and item.bbox[0]+item.bbox[2] >= centre_px-half_width_px]
    return min(candidates, key=lambda item: (ranges[item.track_id], item.track_id), default=None)
