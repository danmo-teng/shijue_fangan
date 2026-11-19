"""140° open-claw separation of one core and one blue block (1280 x 1024)."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class BlackBluePlan:
    keep_side: str
    # Zero means ready, None means no reachable common angle.
    turn_deg: int | None


def black_blue_plan(black_box, blue_box):
    """Intersect the blue-edge clearance and black reach along one in-place turn."""
    kx, ky, kw, kh = black_box
    bx, by, bw, bh = blue_box
    black_center = kx + kw / 2
    blue_center = bx + bw / 2
    if black_center == blue_center:
        return None
    # Small roll of the camera: use the lateral projection along the push plate.
    # A1: blue-right inner edge 757 at foot y506, core center 700 at foot y480.
    # A2: blue-left inner edge 554 at foot y462, core center 659 at foot y475.
    # The 23:39 clip (frames 4643 to 4656) gives +16 and +12 px/degree CCW.
    slope = .06
    if blue_center > black_center:
        side = "left"
        low = (757 - (bx + slope * (by + bh - 506))) / 16
        high = (700 - (black_center + slope * (ky + kh - 480))) / 12
    else:
        side = "right"
        low = (659 - (black_center + slope * (ky + kh - 475))) / 12
        high = (554 - (bx + bw + slope * (by + bh - 462))) / 16

    if low <= 1 and high >= -1:
        return BlackBluePlan(side, 0)
    # Both constraints must fit, with the agreed one-degree tolerance. Prefer
    # an angle inside both nominal bounds when a whole degree fits there.
    for slack in (0, 1):
        begin = max(-20, math.ceil(low - slack))
        end = min(20, math.floor(high + slack))
        if begin <= end:
            angle = max(begin, min(0, end))
            # The finite-turn controller already considers +/-1 degree done.
            if abs(angle) == 1:
                angle *= 2
            if begin <= angle <= end:
                return BlackBluePlan(side, angle)
    return BlackBluePlan(side, None)
