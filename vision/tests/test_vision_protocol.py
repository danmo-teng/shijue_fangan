#!/usr/bin/env python3
"""Byte-level tests against the F407 README examples."""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rescue_vision.vision_protocol import NormalSupplyReport, config_frame


def main() -> None:
    assert config_frame(0, 0x11, 1).hex(" ").upper() == (
        "A3 B3 11 00 11 01 00 00 00 00 00 00 F0 57 C3"
    )
    report = NormalSupplyReport(320, 240, 0, found=True)
    assert report.to_frame(0x10).hex(" ").upper() == (
        "A3 B3 12 10 01 40 00 F0 00 00 01 09 1C 13 C3"
    )
    distance_report = NormalSupplyReport(320, 240, 350, found=True, distance_valid=True)
    assert distance_report.to_frame(0x10).hex(" ").upper() == (
        "A3 B3 12 10 01 40 00 F0 01 5E 01 49 7D CD C3"
    )
    assert NormalSupplyReport().to_frame(0x11)[4:12] == bytes(8)
    print("PASS")


if __name__ == "__main__":
    main()
