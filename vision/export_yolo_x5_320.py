#!/usr/bin/env python3
"""Export an Ultralytics checkpoint with X5-friendly split heads at 320x320.

Run this file on the x86 training PC from the D-Robotics conversion directory,
next to the official ``export_monkey_patch.py``.
"""
from __future__ import annotations

import argparse

from ultralytics import YOLO

from export_monkey_patch import modelZooOptimizer


def main() -> None:
    parser = argparse.ArgumentParser(description="Export split-head YOLO ONNX for RDK X5")
    parser.add_argument("--pt", required=True, help="trained Ultralytics best.pt")
    parser.add_argument("--imgsz", type=int, default=320)
    parser.add_argument("--opset", type=int, default=11)
    args = parser.parse_args()
    if args.imgsz <= 0 or args.imgsz % 32:
        parser.error("--imgsz must be a positive multiple of 32")

    model = YOLO(args.pt)
    modelZooOptimizer(model.model.model)
    model.export(
        format="onnx",
        imgsz=args.imgsz,
        simplify=False,
        opset=args.opset,
        dynamic=False,
    )


if __name__ == "__main__":
    main()
