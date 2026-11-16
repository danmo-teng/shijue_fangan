"""Ordinary pickup paths against the existing field's two safety areas.

The user-selected centre allowance is 200 mm, with no extra padding. This
planning envelope is not a measured outline of the 270 mm open claw tips.
"""
from __future__ import annotations

import math

PICKUP_RADIUS_M = .20
SAFE_ZONE_RECTS = {
    "red": (-.33, 1.14, .33, 1.50),
    "blue": (-.33, -1.50, .33, -1.14),
}


def forward_endpoint(x: float, y: float, yaw_deg: float, distance_m: float) -> tuple[float, float]:
    angle = math.radians(yaw_deg)
    return x + distance_m * math.cos(angle), y + distance_m * math.sin(angle)


def _point_segment_sq(point, start, end) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_sq = dx * dx + dy * dy
    t = (max(0., min(1., ((point[0]-start[0])*dx + (point[1]-start[1])*dy) / length_sq))
         if length_sq else 0.)
    return (point[0]-start[0]-t*dx)**2 + (point[1]-start[1]-t*dy)**2


def _segment_rect_sq(start, end, rect) -> float:
    x0, y0, x1, y1 = rect
    lo, hi = 0., 1.
    intersects = True
    for a, b, lower, upper in ((start[0], end[0], x0, x1), (start[1], end[1], y0, y1)):
        delta = b-a
        if abs(delta) < 1e-12:
            if not lower <= a <= upper:
                intersects = False
                break
        else:
            u, v = sorted(((lower-a)/delta, (upper-a)/delta))
            lo, hi = max(lo, u), min(hi, v)
            if lo > hi:
                intersects = False
                break
    if intersects:
        return 0.
    def point_rect_sq(p):
        return max(x0-p[0], 0., p[0]-x1)**2 + max(y0-p[1], 0., p[1]-y1)**2
    return min(point_rect_sq(start), point_rect_sq(end),
               *(_point_segment_sq(corner, start, end)
                 for corner in ((x0,y0), (x0,y1), (x1,y0), (x1,y1))))


def crossed_safe_zone(start: tuple[float, float], end: tuple[float, float]) -> str | None:
    if not all(math.isfinite(v) for v in (*start, *end)):
        return None
    for side, rect in SAFE_ZONE_RECTS.items():
        x0, y0, x1, y1 = rect
        nearest = (max(x0, min(x1, start[0])), max(y0, min(y1, start[1])))
        away = (start[0]-nearest[0], start[1]-nearest[1])
        motion = (end[0]-start[0], end[1]-start[1])
        end_distance_sq = max(x0-end[0], 0., end[0]-x1)**2 + max(y0-end[1], 0., end[1]-y1)**2
        start_distance_sq = away[0]**2 + away[1]**2
        if start_distance_sq == 0 and start != end:
            exits = ((start[0]-x0, -motion[0]), (x1-start[0], motion[0]),
                     (start[1]-y0, -motion[1]), (y1-start[1], motion[1]))
            nearest_edge = min(gap for gap, _ in exits)
            if any(gap <= nearest_edge + 1e-12 and outward > 0 for gap, outward in exits):
                continue  # Already over the edge: allow the nearest way out.
        if (0 < start_distance_sq <= PICKUP_RADIUS_M**2 and
                away[0]*motion[0] + away[1]*motion[1] >= 0 and
                end_distance_sq > start_distance_sq + 1e-12):
            # Already inside the selected planning allowance, but this move
            # only increases clearance. Cancelling into reverse would be worse.
            continue
        if _segment_rect_sq(start, end, rect) <= PICKUP_RADIUS_M**2 + 1e-12:
            return side
    return None
