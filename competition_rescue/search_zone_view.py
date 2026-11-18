"""One camera observation of a target covered by a clipped zone detection."""
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from state_machine import TrackedCargo, CommandRequest


@dataclass
class ZoneObservation:
    zone: str
    target: 'TrackedCargo'
    pixel_frame: int
    request: 'CommandRequest | None' = None
    completed_frame: int | None = None
    completed_s: float = 0.
    realign: 'CommandRequest | None' = None
    leaving: bool = False


def clipped(box):
    x, y, w, h = box
    return x <= 1 or y <= 1 or x+w >= 1279 or y+h >= 1023


def clearly_outside(item, boxes, zone):
    related = [box for name, box in boxes if name == zone]
    if len(related) != 1 or clipped(related[0]) or item.inside_safe_zone:
        return False
    x, y, w, h = item.bbox
    if clipped(item.bbox):
        return False
    return all(x+w < sx-1 or x > sx+sw+1 or y+h < sy-1 or y+h > sy+sh+1
               for _, (sx, sy, sw, sh) in boxes)
