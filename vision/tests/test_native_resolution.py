#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rescue_vision.config import load_config
from rescue_vision.vision_protocol import IMAGE_HEIGHT, IMAGE_WIDTH, NormalSupplyReport


def main() -> None:
    config = load_config(ROOT / "config/rescue_vision.json")
    assert config["camera"]["width"] == IMAGE_WIDTH
    assert config["camera"]["height"] == IMAGE_HEIGHT
    assert config["threshold_reference_resolution"] == [IMAGE_WIDTH, IMAGE_HEIGHT]
    assert config["performance"]["two_stage"] is True
    assert config["performance"]["coarse_width"] == 320
    assert config["performance"]["coarse_height"] == 256
    for profile in config["classes"].values():
        for variant in [profile] + profile.get("references", []):
            low, high = variant["candidate"]["area_px"]
            assert 0 <= low <= high <= IMAGE_WIDTH * IMAGE_HEIGHT
            for key in ("open", "close"):
                kernel = int(variant["morphology"].get(key, 0))
                assert kernel == 0 or kernel % 2 == 1
    assert NormalSupplyReport(1279, 1023, found=True).payload()
    print("native 1280x1024 contract PASS")


if __name__ == "__main__":
    main()
