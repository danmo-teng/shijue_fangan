#!/usr/bin/env python3
"""Dispatch the competition detector; YOLO is the default backend."""
from __future__ import annotations

import argparse
import sys


def main() -> int:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--detector", choices=("yolo", "traditional"), default="yolo")
    selected, remaining = parser.parse_known_args()
    sys.argv = [sys.argv[0], *remaining]
    if selected.detector == "traditional":
        from run_traditional_detector import main as selected_main
    else:
        from run_yolo_x5 import main as selected_main
    return int(selected_main())


if __name__ == "__main__":
    raise SystemExit(main())
