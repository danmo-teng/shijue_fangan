#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rescue_vision.config import load_config
from rescue_vision.tuning import auto_sample_profile, diagnose_frame


def main() -> None:
    config = load_config(ROOT / "config" / "rescue_vision.json")
    frame = np.full((480, 640, 3), 210, np.uint8)
    cv2.rectangle(frame, (230, 190), (350, 310), (0, 245, 0), -1)
    profile = config["classes"]["green_supply"]
    result = auto_sample_profile(frame, (225, 185, 355, 315), profile)
    assert result["hsv"][0] <= 60 <= result["hsv"][3]
    assert result["estimated_shape"]["area_px"] > 10000
    diagnosis = diagnose_frame(frame, 10)
    assert "mean_luma" in diagnosis and diagnosis["suggested_exposure"] is not None
    print("tuning PASS", result["hsv"], diagnosis["mean_luma"])


if __name__ == "__main__":
    main()
