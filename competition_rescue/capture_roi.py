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
DEFAULT_LEFT_CLAW_POLYGON = (
    (0, IMAGE_HEIGHT // 4),
    (600, IMAGE_HEIGHT // 4),
    (600, IMAGE_HEIGHT - 1),
    (0, IMAGE_HEIGHT - 1),
)
DEFAULT_RIGHT_CLAW_POLYGON = (
    (680, IMAGE_HEIGHT // 4),
    (IMAGE_WIDTH - 1, IMAGE_HEIGHT // 4),
    (IMAGE_WIDTH - 1, IMAGE_HEIGHT - 1),
    (680, IMAGE_HEIGHT - 1),
)


@dataclass(frozen=True)
class CaptureRois:
    overall: tuple[tuple[int, int], ...]
    left: tuple[tuple[int, int], ...]
    right: tuple[tuple[int, int], ...]


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
        return CaptureRois(
            DEFAULT_CAPTURE_POLYGON,
            DEFAULT_LEFT_CLAW_POLYGON,
            DEFAULT_RIGHT_CLAW_POLYGON,
        )
    if int(data.get("image_width", 0)) != IMAGE_WIDTH or int(
        data.get("image_height", 0)
    ) != IMAGE_HEIGHT:
        raise ValueError("capture ROI image size must be 1280x1024")
    overall = _validate_points(data.get("polygon_px"))
    left_points = data.get("left_polygon_px")
    right_points = data.get("right_polygon_px")
    return CaptureRois(
        overall,
        (
            _validate_points(left_points)
            if left_points is not None else
            DEFAULT_LEFT_CLAW_POLYGON
        ),
        (
            _validate_points(right_points)
            if right_points is not None else
            DEFAULT_RIGHT_CLAW_POLYGON
        ),
    )


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
    left = _validate_points([[x, y] for x, y in rois.left])
    right = _validate_points([[x, y] for x, y in rois.right])
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
    required_ratio = 1.0 if require_full_overlap else minimum_overlap_ratio
    overall_ratio = bbox_capture_roi_overlap_ratio(bbox, rois.overall)
    if (
        not bbox_bottom_center_in_roi(bbox, rois.overall) or
        overall_ratio < required_ratio
    ):
        return None
    left_ratio = bbox_capture_roi_overlap_ratio(bbox, rois.left)
    right_ratio = bbox_capture_roi_overlap_ratio(bbox, rois.right)
    left_valid = (
        bbox_bottom_center_in_roi(bbox, rois.left) and
        left_ratio >= required_ratio
    )
    right_valid = (
        bbox_bottom_center_in_roi(bbox, rois.right) and
        right_ratio >= required_ratio
    )
    if left_valid != right_valid:
        return "left" if left_valid else "right"
    if left_valid and right_valid:
        if left_ratio > right_ratio:
            return "left"
        if right_ratio > left_ratio:
            return "right"
    return None
