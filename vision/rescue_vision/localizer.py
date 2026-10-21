from __future__ import annotations

from pathlib import Path
from typing import Optional
import json

import cv2
import numpy as np


class GroundLocalizer:
    def __init__(self, matrix: Optional[np.ndarray] = None) -> None:
        self.matrix = None if matrix is None else np.asarray(matrix, dtype=np.float64)

    @property
    def calibrated(self) -> bool:
        return self.matrix is not None and self.matrix.shape == (3, 3)

    def image_to_ground(self, point: tuple[float, float]) -> Optional[tuple[float, float]]:
        if not self.calibrated:
            return None
        src = np.array([[[point[0], point[1]]]], dtype=np.float32)
        dst = cv2.perspectiveTransform(src, self.matrix)[0, 0]
        return float(dst[0]), float(dst[1])

    def image_segment_size_mm(
        self,
        center: tuple[float, float],
        width_px: float,
        height_px: float,
    ) -> Optional[tuple[float, float]]:
        if not self.calibrated:
            return None
        cx, cy = center
        left = self.image_to_ground((cx - width_px / 2.0, cy))
        right = self.image_to_ground((cx + width_px / 2.0, cy))
        top = self.image_to_ground((cx, cy - height_px / 2.0))
        bottom = self.image_to_ground((cx, cy + height_px / 2.0))
        if None in (left, right, top, bottom):
            return None
        width = float(np.hypot(right[0] - left[0], right[1] - left[1]))
        height = float(np.hypot(bottom[0] - top[0], bottom[1] - top[1]))
        return width, height

    @staticmethod
    def calibrate(
        image_points: list[tuple[float, float]],
        ground_points_mm: list[tuple[float, float]],
    ) -> tuple["GroundLocalizer", float]:
        if len(image_points) < 4 or len(image_points) != len(ground_points_mm):
            raise ValueError("单应标定至少需要4组一一对应的点")
        image = np.asarray(image_points, dtype=np.float32)
        ground = np.asarray(ground_points_mm, dtype=np.float32)
        matrix, inliers = cv2.findHomography(image, ground, cv2.RANSAC, 3.0)
        if matrix is None:
            raise ValueError("单应矩阵计算失败")
        projected = cv2.perspectiveTransform(image.reshape(-1, 1, 2), matrix).reshape(-1, 2)
        rmse = float(np.sqrt(np.mean(np.sum((projected - ground) ** 2, axis=1))))
        return GroundLocalizer(matrix), rmse

    def save(self, path: str | Path, resolution: tuple[int, int] | None = None) -> None:
        if not self.calibrated:
            raise ValueError("尚未完成地面标定")
        target = Path(path)
        np.savetxt(str(target), self.matrix, fmt="%.12g")
        if resolution is not None:
            metadata = target.with_suffix(target.suffix + ".meta.json")
            metadata.write_text(
                json.dumps({"width": int(resolution[0]), "height": int(resolution[1])}, indent=2),
                encoding="utf-8",
            )

    @staticmethod
    def load(
        path: str | Path,
        expected_resolution: tuple[int, int] | None = None,
    ) -> "GroundLocalizer":
        target = Path(path)
        if not target.exists():
            return GroundLocalizer()
        if expected_resolution is not None:
            metadata = target.with_suffix(target.suffix + ".meta.json")
            if not metadata.exists():
                return GroundLocalizer()
            try:
                info = json.loads(metadata.read_text(encoding="utf-8"))
                stored = (int(info["width"]), int(info["height"]))
            except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError):
                return GroundLocalizer()
            if stored != tuple(map(int, expected_resolution)):
                return GroundLocalizer()
        return GroundLocalizer(np.loadtxt(str(target), dtype=np.float64))


class ApproachRangeCalibration:
    """Measured centre-Y bands at a fixed camera pitch; never uses homography."""

    def __init__(self, data: dict | None = None) -> None:
        self.data = data or {}

    @classmethod
    def load(cls, path: str | Path) -> "ApproachRangeCalibration":
        try:
            return cls(json.loads(Path(path).read_text(encoding="utf-8")))
        except (OSError, ValueError):
            return cls()

    def distance(self, pitch: int, point: tuple[float, float], resolution: tuple[int, int]) -> tuple[int | None, str]:
        try:
            entry = self.data["angles"][str(pitch)]
            if (pitch not in {10500, 12000} or entry["measured"] is not True or
                    (entry["width"], entry["height"]) != resolution or
                    entry["pixel_reference"] != "bbox_center" or
                    entry["distance_reference"] != "measured_chassis_travel_to_claw_entry" or
                    not entry["camera_mount"]):
                return None, "range_calibration_metadata_invalid"
            x, y = point
            if not (0 <= x < resolution[0] and 0 <= y < resolution[1]):
                return None, "range_pixel_outside_image"
            bands = entry["bands"]
            previous = 0
            for band in sorted(bands, key=lambda b: b["y_min"]):
                low, high, distance = band["y_min"], band["y_max"], band["advance_mm"]
                if not (previous <= low < high <= resolution[1] and
                        type(distance) is int and 0 <= distance <= 2500):
                    return None, "range_calibration_bands_invalid"
                previous = high
            matches = [b["advance_mm"] for b in bands if b["y_min"] <= y < b["y_max"]]
            return (matches[0], "range_measured") if len(matches) == 1 else (None, "range_band_unmeasured")
        except (KeyError, TypeError, ValueError):
            return None, "range_calibration_missing"
