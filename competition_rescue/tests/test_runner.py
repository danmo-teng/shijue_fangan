#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = ROOT.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "vision"))

from run_competition_rescue import bbox_center_px, detection_log_record  # noqa: E402
from rescue_vision.models import Detection  # noqa: E402


def main() -> None:
    assert bbox_center_px((10, 20, 31, 41)) == (25, 40)
    detection = Detection(
        class_name="green_supply",
        confidence=0.8,
        bbox=(10, 20, 31, 41),
        bottom_point=(25.5, 61.0),
        ground_xy_mm=(100.0, 200.0),
        size_mm=None,
        contour=np.empty((0, 1, 2), dtype=np.int32),
    )
    record = detection_log_record(
        7,
        {"total_ms": 12.5},
        [detection],
        (),
    )
    assert record["detections"][0]["center"] == [25, 40]
    assert record["detections"][0]["ground_xy_mm"] == [100.0, 200.0]
    print("competition runner PASS")


if __name__ == "__main__":
    main()
