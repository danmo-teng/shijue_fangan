from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np


class TrackState(str, Enum):
    TENTATIVE = "TENTATIVE"
    CONFIRMED = "CONFIRMED"
    LOST = "LOST"


@dataclass
class Candidate:
    class_name: str
    contour: np.ndarray
    bbox: tuple[int, int, int, int]
    rotated_box: np.ndarray
    bottom_point: tuple[float, float]
    area_px: float
    aspect: float
    extent: float
    solidity: float
    circularity: float
    vertices: int
    mean_hsv: tuple[float, float, float]
    mean_lab: tuple[float, float, float]
    color_fill: float
    contrast: float
    score: float
    reject_reason: str = ""
    ground_xy_mm: Optional[tuple[float, float]] = None
    size_mm: Optional[tuple[float, float]] = None
    reference_index: int = 0
    reference_name: str = "base"


@dataclass
class Detection:
    class_name: str
    confidence: float
    bbox: tuple[int, int, int, int]
    bottom_point: tuple[float, float]
    ground_xy_mm: Optional[tuple[float, float]]
    size_mm: Optional[tuple[float, float]]
    contour: np.ndarray
    features: dict[str, float] = field(default_factory=dict)


@dataclass
class Track:
    track_id: int
    class_name: str
    position: tuple[float, float]
    confidence: float
    state: TrackState
    hits: int
    misses: int
    age: int
    last_detection: Detection
