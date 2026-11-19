"""140° open-claw separation of one core and one blue block (1280 x 1024)."""
from dataclasses import dataclass


@dataclass(frozen=True)
class BlackBluePlan:
    keep_side: str
    turn_deg: int  # Zero means the blue block already clears the claw.


def black_blue_plan(black_box, blue_box):
    black_x = black_box[0] + black_box[2] / 2
    blue_x = blue_box[0] + blue_box[2] / 2
    if blue_x == black_x:
        return None

    # Field approximation near the claws: about 16 image pixels per degree
    # of chassis yaw. A1/A2 put the blue center at x=898/412.5 respectively.
    blue_offset_deg = (blue_x - 640) / 16
    if blue_x < black_x:
        keep_side = "right"
        missing_deg = 14 + blue_offset_deg
        turn_sign = -1  # Turning right moves blue farther left in the image.
    else:
        keep_side = "left"
        missing_deg = 16 - blue_offset_deg
        turn_sign = 1

    if missing_deg <= 1:
        return BlackBluePlan(keep_side, 0)
    return BlackBluePlan(keep_side, turn_sign * min(20, max(2, round(missing_deg))))
