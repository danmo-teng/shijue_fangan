#!/usr/bin/env python3
"""Shared claw-capture polygon used by the editor and competition runner."""
from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 1024
DEFAULT_CAPTURE_POLYGON = (
    (0, IMAGE_HEIGHT // 4),
    (IMAGE_WIDTH - 1, IMAGE_HEIGHT // 4),
    (IMAGE_WIDTH - 1, IMAGE_HEIGHT - 1),
    (0, IMAGE_HEIGHT - 1),
)
CLAW_SPLIT_X = IMAGE_WIDTH // 2


@dataclass(frozen=True)
class CaptureRois:
    overall: tuple[tuple[int, int], ...]
    left: tuple[tuple[int, int], ...]
    right: tuple[tuple[int, int], ...]


def _clip_polygon_x(
    polygon: tuple[tuple[int, int], ...],
    *,
    keep_left: bool,
) -> tuple[tuple[int, int], ...]:
    def inside(point: tuple[float, float]) -> bool:
        return point[0] <= CLAW_SPLIT_X if keep_left else point[0] >= CLAW_SPLIT_X

    def intersection(
        start: tuple[float, float], end: tuple[float, float]
    ) -> tuple[float, float]:
        dx = end[0] - start[0]
        if dx == 0:
            return float(CLAW_SPLIT_X), start[1]
        ratio = (CLAW_SPLIT_X - start[0]) / dx
        return float(CLAW_SPLIT_X), start[1] + ratio * (end[1] - start[1])

    output: list[tuple[float, float]] = []
    values = [(float(x), float(y)) for x, y in polygon]
    if not values:
        return ()
    start = values[-1]
    for end in values:
        if inside(end):
            if not inside(start):
                output.append(intersection(start, end))
            output.append(end)
        elif inside(start):
            output.append(intersection(start, end))
        start = end
    return tuple((round(x), round(y)) for x, y in output)


def split_capture_roi(
    overall: tuple[tuple[int, int], ...]
) -> tuple[tuple[tuple[int, int], ...], tuple[tuple[int, int], ...]]:
    return (
        _clip_polygon_x(overall, keep_left=True),
        _clip_polygon_x(overall, keep_left=False),
    )


def _validate_points(points) -> tuple[tuple[int, int], ...]:
    if not isinstance(points, list) or len(points) < 3:
        raise ValueError("capture ROI must contain at least three points")
    polygon: list[tuple[int, int]] = []
    for point in points:
        if not isinstance(point, list) or len(point) != 2:
            raise ValueError("capture ROI points must be [x, y]")
        x, y = int(point[0]), int(point[1])
        if not 0 <= x < IMAGE_WIDTH or not 0 <= y < IMAGE_HEIGHT:
            raise ValueError("capture ROI point is outside 1280x1024 image")
        polygon.append((x, y))
    return tuple(polygon)


def load_capture_rois(path: Path) -> CaptureRois:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        left, right = split_capture_roi(DEFAULT_CAPTURE_POLYGON)
        return CaptureRois(DEFAULT_CAPTURE_POLYGON, left, right)
    if int(data.get("image_width", 0)) != IMAGE_WIDTH or int(
        data.get("image_height", 0)
    ) != IMAGE_HEIGHT:
        raise ValueError("capture ROI image size must be 1280x1024")
    overall = _validate_points(data.get("polygon_px"))
    left, right = split_capture_roi(overall)
    return CaptureRois(overall, left, right)


def load_capture_roi(path: Path) -> tuple[tuple[int, int], ...]:
    return load_capture_rois(path).overall


def save_capture_roi(path: Path, points: list[tuple[int, int]]) -> None:
    current = load_capture_rois(path)
    save_capture_rois(
        path,
        CaptureRois(
            _validate_points([[x, y] for x, y in points]),
            current.left,
            current.right,
        ),
    )


def save_capture_rois(path: Path, rois: CaptureRois) -> None:
    overall = _validate_points([[x, y] for x, y in rois.overall])
    left, right = split_capture_roi(overall)
    payload = {
        "schema_version": 2,
        "image_width": IMAGE_WIDTH,
        "image_height": IMAGE_HEIGHT,
        "point_semantics": "bbox_bottom_center_and_overlap_ratio",
        "minimum_overlap_ratio": 0.80,
        "polygon_px": [[x, y] for x, y in overall],
        "left_polygon_px": [[x, y] for x, y in left],
        "right_polygon_px": [[x, y] for x, y in right],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def bbox_capture_roi_overlap_ratio(
    bbox: tuple[int, int, int, int],
    polygon: tuple[tuple[int, int], ...],
) -> float:
    x, y, width, height = bbox
    if width <= 0 or height <= 0 or len(polygon) < 3:
        return 0.0
    local_polygon = np.asarray(
        [(px - x, py - y) for px, py in polygon],
        dtype=np.int32,
    ).reshape((-1, 1, 2))
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(mask, [local_polygon], 1)
    return float(np.count_nonzero(mask)) / float(width * height)


def bbox_overlaps_capture_roi(
    bbox: tuple[int, int, int, int],
    polygon: tuple[tuple[int, int], ...],
    minimum_ratio: float = 0.80,
) -> bool:
    return bbox_capture_roi_overlap_ratio(bbox, polygon) > minimum_ratio


def bbox_bottom_center_in_roi(
    bbox: tuple[int, int, int, int],
    polygon: tuple[tuple[int, int], ...],
) -> bool:
    x, y, width, height = bbox
    if width <= 0 or height <= 0 or len(polygon) < 3:
        return False
    bottom_center = (
        max(0, min(IMAGE_WIDTH - 1, x + width * 0.5)),
        max(0, min(IMAGE_HEIGHT - 1, y + height - 1)),
    )
    contour = np.asarray(polygon, dtype=np.float32).reshape((-1, 1, 2))
    return cv2.pointPolygonTest(contour, bottom_center, False) >= 0.0


def capture_side_for_bbox(
    bbox: tuple[int, int, int, int],
    rois: CaptureRois,
    *,
    require_full_overlap: bool = False,
    minimum_overlap_ratio: float = 0.80,
) -> str | None:
    if not bbox_in_capture_roi(
        bbox,
        rois.overall,
        require_full_overlap=require_full_overlap,
        minimum_overlap_ratio=minimum_overlap_ratio,
    ):
        return None
    left_valid = bbox_bottom_center_in_roi(bbox, rois.left)
    right_valid = bbox_bottom_center_in_roi(bbox, rois.right)
    if left_valid != right_valid:
        return "left" if left_valid else "right"
    if left_valid and right_valid:
        x, _, width, _ = bbox
        return "left" if x + width * 0.5 < CLAW_SPLIT_X else "right"
    return None


def bbox_in_capture_roi(
    bbox: tuple[int, int, int, int],
    polygon: tuple[tuple[int, int], ...],
    *,
    require_full_overlap: bool = False,
    minimum_overlap_ratio: float = 0.80,
) -> bool:
    required_ratio = 1.0 if require_full_overlap else minimum_overlap_ratio
    return (
        bbox_bottom_center_in_roi(bbox, polygon) and
        bbox_capture_roi_overlap_ratio(bbox, polygon) >= required_ratio
    )
