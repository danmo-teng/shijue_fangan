"""Keep zone membership only along a continuous, same-view cargo track."""
from dataclasses import replace
import math


class SafeZoneCargoMemory:
    def __init__(self):
        self.clear()

    def clear(self):
        self.pitch = None
        self.previous = {}

    @staticmethod
    def continuous(old, item):
        if old.class_name != item.class_name or item.hits <= old.hits:
            return False
        x, y, w, h = old.bbox
        nx, ny, nw, nh = item.bbox
        if min(w, h, nw, nh) <= 0:
            return False
        # Do not transfer a zone label across a large box/identity jump.
        return (0.5 <= nw / w <= 2 and 0.5 <= nh / h <= 2 and
                math.hypot(nx + nw / 2 - x - w / 2,
                           ny + nh / 2 - y - h / 2) <= max(w, h, nw, nh))

    def apply(self, cargo, pitch, *, reviewed_outside_id=None):
        if pitch != self.pitch:
            self.clear()
            self.pitch = pitch
        result = []
        for item in cargo:
            old = self.previous.get(item.track_id)
            held_missing_track = (old is not None and not item.visible and
                                  old.class_name == item.class_name and
                                  old.hits == item.hits and old.bbox == item.bbox)
            if (item.track_id != reviewed_outside_id and not item.inside_safe_zone and old is not None and old.inside_safe_zone and
                    (held_missing_track or (item.visible and self.continuous(old, item)))):
                item = replace(item, inside_safe_zone=True, safe_zone_name=old.safe_zone_name)
            result.append(item)
        # Keep the tracker's existing missed-frame continuity, but removal of
        # the track breaks the chain. No permanent ID blacklist or old rectangle.
        self.previous = {item.track_id: item for item in result}
        return tuple(result)
