"""Upper-owned obstacle route; no device IO or independent cargo audit."""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math


class RouteStep(IntEnum):
    TURN_RIGHT = 0
    FORWARD_PARK = 1
    RELEASE_LOAD = 2
    BACK_200 = 3
    TURN_LEFT = 4
    BACK_TO_SWEEP = 5
    LEFT_TO_SWEEP = 6
    FULL_OPEN = 7
    SWEEP_1000 = 8
    RAISE_RETURN = 9
    BACK_TO_STAGE = 10
    RIGHT_TO_STAGE = 11
    PICKUP_READY = 12


LIVE_STEPS = frozenset({RouteStep.BACK_TO_SWEEP, RouteStep.LEFT_TO_SWEEP,
                        RouteStep.BACK_TO_STAGE, RouteStep.RIGHT_TO_STAGE})
LOCAL_DISTANCES = {RouteStep.FORWARD_PARK: 200, RouteStep.BACK_200: -200,
                   RouteStep.SWEEP_1000: 1000}
TURN_STEPS = frozenset({RouteStep.TURN_RIGHT, RouteStep.TURN_LEFT})
ACTUATOR_STEPS = frozenset({RouteStep.RELEASE_LOAD, RouteStep.FULL_OPEN,
                            RouteStep.RAISE_RETURN, RouteStep.PICKUP_READY})


def handling_kind(destination: str, obstacle_classes: tuple[str, ...], *, first: bool) -> str:
    compatible = ({"green_supply", "core_black"} if destination == "material"
                  else {"injured_orange"})
    return ("pickup" if not first and obstacle_classes and
            set(obstacle_classes).issubset(compatible) else "sweep")


@dataclass
class ObstacleRoute:
    side: str
    original_ids: tuple[int, ...]
    original_classes: tuple[str, ...]
    original_destination: str
    return_point: tuple[float, float]
    park_origin: tuple[float, float]
    obstacle_classes: tuple[str, ...]
    kind: str
    sweep_tolerance_m: float = .03
    steps: tuple[RouteStep, ...] = ()
    index: int = 0
    arrival_latched: bool = False
    deposit_vehicle_point: tuple[float, float] | None = None
    pickup_frame_floor: int = 0
    pickup_role: str = "original"
    awaiting_obstacle_delivery: bool = False
    recover_after_delivery: bool = False
    replaced_stash_ids: tuple[int, ...] = ()

    def __post_init__(self) -> None:
        common = (RouteStep.TURN_RIGHT, RouteStep.FORWARD_PARK, RouteStep.RELEASE_LOAD)
        self.steps = common + (
            (RouteStep.BACK_200, RouteStep.TURN_LEFT, RouteStep.PICKUP_READY)
            if self.kind == "pickup" else
            (RouteStep.BACK_TO_SWEEP, RouteStep.LEFT_TO_SWEEP, RouteStep.FULL_OPEN,
             RouteStep.SWEEP_1000, RouteStep.RAISE_RETURN,
             RouteStep.BACK_TO_STAGE, RouteStep.RIGHT_TO_STAGE, RouteStep.PICKUP_READY)
        )
        self.pickup_role = "obstacle" if self.kind == "pickup" else "original"

    @property
    def side_sign(self) -> int:
        return 1 if self.side == "red" else -1

    @property
    def sweep_point(self) -> tuple[float, float]:
        # Right-turn frame: behind wall 94cm, left wall 54cm, robot CENTRE.
        return -.56 * self.side_sign, .96 * self.side_sign

    @property
    def sweep_heading_cdeg(self) -> int:
        return 0 if self.side == "red" else 18000

    @property
    def step(self) -> RouteStep | None:
        return self.steps[self.index] if self.index < len(self.steps) else None

    def advance(self) -> None:
        self.index += 1
        self.arrival_latched = False

    def begin_original_recovery(self) -> None:
        self.steps = (RouteStep.TURN_RIGHT, RouteStep.PICKUP_READY)
        self.index = 0
        self.arrival_latched = False
        self.pickup_role = "original"
        self.awaiting_obstacle_delivery = False
        self.recover_after_delivery = False

    def payload(self, x_m: float, y_m: float, *, pose_valid: bool) -> tuple[int, int, int] | None:
        """P2..P7; live legs publish remaining H/D, finite legs execute once."""
        step = self.step
        if step is None:
            return None
        heading = self.sweep_heading_cdeg
        if step in TURN_STEPS:
            return 0, 0, (heading if step == RouteStep.TURN_RIGHT
                           else (heading + 9000) % 36000)
        if step in LOCAL_DISTANCES:
            return LOCAL_DISTANCES[step], 0, heading
        if step in ACTUATOR_STEPS:
            return 0, 0, 0
        if not pose_valid:
            return None
        target = self.sweep_point if step in {
            RouteStep.BACK_TO_SWEEP, RouteStep.LEFT_TO_SWEEP} else self.return_point
        dx, dy = target[0] - x_m, target[1] - y_m
        xtol, ytol = ((self.sweep_tolerance_m,) * 2 if step in {
            RouteStep.BACK_TO_SWEEP, RouteStep.LEFT_TO_SWEEP} else (.10, .05))
        if step in {RouteStep.BACK_TO_SWEEP, RouteStep.BACK_TO_STAGE}:
            dy = 0.0  # First the rearward leg; the following lateral leg owns Y.
        if abs(dx) <= xtol + 1e-9:
            dx = 0.0
        if abs(dy) <= ytol + 1e-9:
            dy = 0.0
        self.arrival_latched = self.arrival_latched or dx == dy == 0.0
        distance = 0 if self.arrival_latched else round(math.hypot(dx, dy) * 1000)
        translation = round(math.degrees(math.atan2(dy, dx)) * 100)
        return distance, translation, heading
