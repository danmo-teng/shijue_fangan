#!/usr/bin/env python3
"""Shared claw-capture polygon used by the editor and competition runner."""
from __future__ import annotations

import json
import os
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


def load_capture_roi(path: Path) -> tuple[tuple[int, int], ...]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return DEFAULT_CAPTURE_POLYGON
    if int(data.get("image_width", 0)) != IMAGE_WIDTH or int(
        data.get("image_height", 0)
    ) != IMAGE_HEIGHT:
        raise ValueError("capture ROI image size must be 1280x1024")
    return _validate_points(data.get("polygon_px"))


def save_capture_roi(path: Path, points: list[tuple[int, int]]) -> None:
    polygon = _validate_points([[x, y] for x, y in points])
    payload = {
        "schema_version": 1,
        "image_width": IMAGE_WIDTH,
        "image_height": IMAGE_HEIGHT,
        "point_semantics": "bbox_overlap_ratio",
        "polygon_px": [[x, y] for x, y in polygon],
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
