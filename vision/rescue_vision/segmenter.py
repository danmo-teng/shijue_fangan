from __future__ import annotations

from typing import Any
import math

import cv2
import numpy as np


def odd_kernel(value: int) -> int:
    value = max(0, int(value))
    if value == 0:
        return 0
    return value if value % 2 else value + 1


class Segmenter:
    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config

    def update_config(self, config: dict[str, Any]) -> None:
        self.config = config

    @staticmethod
    def color_spaces(frame: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return (
            cv2.cvtColor(frame, cv2.COLOR_BGR2HSV),
            cv2.cvtColor(frame, cv2.COLOR_BGR2LAB),
        )

    @staticmethod
    def _range(image: np.ndarray, bounds: list[int], hue: bool = False) -> np.ndarray:
        low = np.array(bounds[:3], dtype=np.uint8)
        high = np.array(bounds[3:], dtype=np.uint8)
        if hue and bounds[0] > bounds[3]:
            low_a = np.array([bounds[0], bounds[1], bounds[2]], dtype=np.uint8)
            high_a = np.array([179, bounds[4], bounds[5]], dtype=np.uint8)
            low_b = np.array([0, bounds[1], bounds[2]], dtype=np.uint8)
            high_b = np.array([bounds[3], bounds[4], bounds[5]], dtype=np.uint8)
            return cv2.bitwise_or(cv2.inRange(image, low_a, high_a), cv2.inRange(image, low_b, high_b))
        return cv2.inRange(image, low, high)

    def segment(
        self,
        frame: np.ndarray,
        class_name: str,
        hsv: np.ndarray | None = None,
        lab: np.ndarray | None = None,
    ) -> tuple[np.ndarray, dict[str, np.ndarray]]:
        profile = self.config["classes"][class_name]
        return self.segment_profile(frame, profile, hsv, lab)

    def segment_profile(
        self,
        frame: np.ndarray,
        profile: dict[str, Any],
        hsv: np.ndarray | None = None,
        lab: np.ndarray | None = None,
        *,
        apply_roi: bool = True,
        scale_resolution: tuple[int, int] | None = None,
        collect_stages: bool = True,
    ) -> tuple[np.ndarray, dict[str, np.ndarray]]:
        fusion = profile.get("fusion", "and")
        need_hsv = fusion != "lab"
        need_lab = fusion != "hsv"
        if need_hsv and hsv is None:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if need_lab and lab is None:
            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        hsv_mask = self._range(hsv, profile["hsv"], hue=True) if need_hsv else None
        lab_mask = self._range(lab, profile["lab"]) if need_lab else None
        if fusion == "or":
            mask = cv2.bitwise_or(hsv_mask, lab_mask)
        elif fusion == "hsv":
            mask = hsv_mask
        elif fusion == "lab":
            mask = lab_mask
        else:
            mask = cv2.bitwise_and(hsv_mask, lab_mask)

        roi = self.config.get("roi_polygon", [])
        if apply_roi and len(roi) >= 3:
            camera = self.config.get("camera", {})
            source_width = max(1, int(camera.get("width", frame.shape[1])))
            source_height = max(1, int(camera.get("height", frame.shape[0])))
            sx = frame.shape[1] / source_width
            sy = frame.shape[0] / source_height
            scaled_roi = np.asarray(
                [[round(point[0] * sx), round(point[1] * sy)] for point in roi],
                dtype=np.int32,
            )
            roi_mask = np.zeros(mask.shape, dtype=np.uint8)
            cv2.fillPoly(roi_mask, [scaled_roi], 255)
            mask = cv2.bitwise_and(mask, roi_mask)

        morph = profile.get("morphology", {})
        reference_width, reference_height = self.config.get(
            "threshold_reference_resolution", [frame.shape[1], frame.shape[0]]
        )
        scale_width, scale_height = scale_resolution or (frame.shape[1], frame.shape[0])
        linear_scale = math.sqrt((scale_width * scale_height) / max(reference_width * reference_height, 1))
        open_size = odd_kernel(round(float(morph.get("open", 0)) * linear_scale))
        close_size = odd_kernel(round(float(morph.get("close", 0)) * linear_scale))
        iterations = max(1, int(morph.get("iterations", 1)))
        if open_size:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_size, open_size))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=iterations)
        if close_size:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_size, close_size))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=iterations)
        if not collect_stages:
            return mask, {}
        stages = {"final": mask}
        if hsv_mask is not None:
            stages["hsv"] = hsv_mask
        if lab_mask is not None:
            stages["lab"] = lab_mask
        return mask, stages
