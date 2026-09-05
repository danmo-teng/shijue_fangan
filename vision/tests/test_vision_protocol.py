#!/usr/bin/env python3
"""Byte-level tests against the F407 README examples."""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rescue_vision.vision_protocol import IMAGE_HEIGHT, IMAGE_WIDTH, NormalSupplyReport, config_frame


def main() -> None:
    assert config_frame(0, 0x11, 1).hex(" ").upper() == (
        "A3 B3 11 00 11 01 00 00 00 00 00 00 F0 57 C3"
    )
    report = NormalSupplyReport(640, 512, 0, found=True)
    expected = report.to_frame(0x10).hex(" ").upper()
    assert expected == "A3 B3 12 10 02 80 02 00 00 00 01 09 DD FD C3"
    distance_report = NormalSupplyReport(640, 512, 350, found=True, distance_valid=True)
    assert distance_report.to_frame(0x10).hex(" ").upper() == (
        "A3 B3 12 10 02 80 02 00 01 5E 01 49 BC 23 C3"
    )
    assert NormalSupplyReport(IMAGE_WIDTH - 1, IMAGE_HEIGHT - 1, found=True).payload()
    assert NormalSupplyReport(640, 512, found=True, cargo_class="core_black").payload()[6] == 0x04
    assert NormalSupplyReport(640, 512, found=True, cargo_class="injured_orange").payload()[6] == 0x10
    assert NormalSupplyReport(640, 512, found=True, cargo_class="danger_cyan").payload()[6] == 0x40
    for x, y in ((IMAGE_WIDTH, 0), (0, IMAGE_HEIGHT)):
        try:
            NormalSupplyReport(x, y, found=True).payload()
            raise AssertionError("out-of-range native pixel accepted")
        except ValueError:
            pass
    assert NormalSupplyReport().to_frame(0x11)[4:12] == bytes(8)
    print("PASS")


if __name__ == "__main__":
    main()
