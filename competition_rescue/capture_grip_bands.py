"""Manually calibrated 140-degree pre-grab bands, independent of claw ROI."""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

CLASSES = ("green_supply", "injured_orange")
IMAGE_SIZE = (1280, 1024)


@dataclass(frozen=True)
class GripBands:
    green_supply: tuple[int, int] | None = None
    injured_orange: tuple[int, int] | None = None

    def __post_init__(self) -> None:
        for name in CLASSES:
            lines = getattr(self, name)
            if lines is not None and (len(lines) != 2 or
                    any(type(y) is not int for y in lines) or
                    not 0 < lines[0] < lines[1] < IMAGE_SIZE[1]):
                raise ValueError(f"{name}: require 0 < upper_y < lower_y < 1024")

    def region(self, class_name: str, center_y: float) -> str | None:
        lines = getattr(self, class_name, None) if class_name in CLASSES else None
        if lines is None:
            return None  # Unmarked is not a guessed calibration.
        return "upper" if center_y < lines[0] else "middle" if center_y < lines[1] else "lower"

    @classmethod
    def load(cls, path: Path) -> "GripBands":
        if not path.exists():
            return cls()
        data = json.loads(path.read_text(encoding="utf-8"))
        if data.get("camera_pitch_cdeg") != 14000 or data.get("image_size") != list(IMAGE_SIZE):
            raise ValueError("grip bands require the 140-degree 1280x1024 image")
        return cls(**{name: None if data["classes"].get(name) is None else
                      tuple(data["classes"][name]) for name in CLASSES})

    def save(self, path: Path) -> None:
        data = {"camera_pitch_cdeg": 14000, "image_size": list(IMAGE_SIZE),
                "classes": {name: getattr(self, name) for name in CLASSES}}
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(path.name + ".tmp")
        temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(path)
