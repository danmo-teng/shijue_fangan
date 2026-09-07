"""Small, backend-independent image-space safety-zone geometry helpers."""
from __future__ import annotations


def bbox_center_in_safe_zone(
    target_bbox: tuple[int, int, int, int] | None,
    safe_bbox: tuple[int, int, int, int] | None,
    *,
    margin_ratio: float = 0.03,
    minimum_margin_px: float = 4.0,
) -> bool:
    """Return whether a detected target centre is inside a detected zone.

    The centre test avoids rejecting an object merely because its detection box
    touches the zone boundary.  A small inward margin also prevents a target
    straddling the visual border from being treated as already delivered.
    """
    if target_bbox is None or safe_bbox is None:
        return False
    tx, ty, tw, th = (float(value) for value in target_bbox)
    sx, sy, sw, sh = (float(value) for value in safe_bbox)
    if tw <= 0.0 or th <= 0.0 or sw <= 0.0 or sh <= 0.0:
        return False
    center_x = tx + tw * 0.5
    center_y = ty + th * 0.5
    margin_x = max(float(minimum_margin_px), sw * float(margin_ratio))
    margin_y = max(float(minimum_margin_px), sh * float(margin_ratio))
    return (
        sx + margin_x <= center_x <= sx + sw - margin_x
        and sy + margin_y <= center_y <= sy + sh - margin_y
    )
