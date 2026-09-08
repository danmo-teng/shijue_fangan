#!/usr/bin/env python3
"""Backward-compatible entry point for the F407 motion-debug map."""
from __future__ import annotations

from t265_f407_motion_map import main


if __name__ == "__main__":
    raise SystemExit(main())
