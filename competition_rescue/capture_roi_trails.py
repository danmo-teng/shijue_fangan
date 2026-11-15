"""Editor-only object image trails; never used to count cargo in competition."""
from __future__ import annotations

import cv2
import numpy as np


class CaptureTrail:
    """Keep changed object pixels against a fixed, empty-claw background."""

    def __init__(self, background: np.ndarray):
        self.background = background.copy()
        self.image = background.copy()
        self.strength = np.zeros(background.shape[:2], dtype=np.uint8)
        self.samples = 0

    def add(self, image: np.ndarray) -> None:
        difference = cv2.absdiff(image, self.background).max(axis=2)
        # Keep the strongest real appearance, not a running average that
        # fades after the object leaves. No detector/ROI eligibility required.
        update = (difference > 24) & (difference > self.strength)
        self.image[update] = image[update]
        self.strength[update] = difference[update]
        self.samples += int(np.any(update))

    def paint(self, canvas: np.ndarray, color: tuple[int, int, int]) -> None:
        mask = self.strength > 0
        canvas[mask] = self.image[mask]
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL,
                                      cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(canvas, contours, -1, color, 1)
