#!/usr/bin/env python3
from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(PROJECT / "vision"))

from state_machine import MissionSettings, MissionState, PoseInput, RescueMission, VisionInput
from rescue_vision.mission_protocol import (
    CMD_ALIGN_SAFE_ZONE,
    CMD_ENTER_SAFE_ZONE,
    CMD_GRAB_CONFIRMED,
    CMD_NAVIGATE_WAYPOINT,
    CMD_TASK_COMPLETE,
    CMD_USE_FINAL_HEADING,
    STM_CLAW_VISIBLE,
    STM_GRIPPER_CLOSED,
    Stm32Status,
)


def target(x=640, y=512, bbox=(600, 470, 80, 80), safe=None):
    return VisionInput(True, x, y, bbox, safe is not None, safe)


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def run_side(side: str, desired_y: int, desired_heading: int, safe_bbox):
    clock = FakeClock()
    mission = RescueMission(
        MissionSettings(side=side, confirmation_frames=3, grab_timeout_s=3.0),
        clock=clock,
    )
    output = mission.step(VisionInput(), PoseInput(), Stm32Status())
    assert output.state == MissionState.SEARCH and not output.report.found
    output = mission.step(target(), PoseInput(), Stm32Status())
    assert output.state == MissionState.APPROACH and output.report.found

    stm = Stm32Status(flags=STM_CLAW_VISIBLE, age_ms=5)
    output = mission.step(target(), PoseInput(), stm)
    assert output.state == MissionState.GRAB_CHECK

    # Once the camera is down, position no longer matters. A missing target or
    # stale STM32 camera status resets the consecutive confirmation count.
    anywhere = target(x=20, y=20, bbox=(0, 0, 40, 40))
    output = mission.step(anywhere, PoseInput(), stm)
    assert output.state == MissionState.GRAB_CHECK
    output = mission.step(VisionInput(), PoseInput(), stm)
    assert output.state == MissionState.GRAB_CHECK
    stale = Stm32Status(flags=STM_CLAW_VISIBLE, age_ms=300)
    output = mission.step(anywhere, PoseInput(), stale)
    assert output.state == MissionState.GRAB_CHECK
    for _ in range(2):
        output = mission.step(anywhere, PoseInput(), stm)
        assert output.state == MissionState.GRAB_CHECK
    output = mission.step(anywhere, PoseInput(), stm)
    assert output.state == MissionState.GRABBING
    assert output.command.command == CMD_GRAB_CONFIRMED

    # GRAB is resent every mission tick until a fresh closed status arrives.
    for _ in range(3):
        clock.advance(0.02)
        output = mission.step(VisionInput(), PoseInput(True, 0.7, 0.0, 0), stm)
        assert output.state == MissionState.GRABBING
        assert output.command.command == CMD_GRAB_CONFIRMED
    stale_closed = Stm32Status(flags=STM_GRIPPER_CLOSED, age_ms=300)
    output = mission.step(VisionInput(), PoseInput(True, 0.7, 0.0, 0), stale_closed)
    assert output.state == MissionState.GRABBING

    closed = Stm32Status(flags=STM_CLAW_VISIBLE | STM_GRIPPER_CLOSED, age_ms=5)
    pose = PoseInput(True, 0.7, 0.0, 0)
    output = mission.step(VisionInput(), pose, closed)
    assert output.state == MissionState.NAVIGATE
    assert output.command.command == CMD_NAVIGATE_WAYPOINT
    assert output.command.target_y_mm == desired_y
    assert output.command.flags & CMD_USE_FINAL_HEADING
    expected_bearing = math.degrees(math.atan2(desired_y / 1000 - pose.y_m, -pose.x_m)) % 360
    assert output.command.heading_cdeg == round(expected_bearing * 100) % 36000

    output = mission.step(VisionInput(), PoseInput(True, 0.0, desired_y / 1000, 0), closed)
    assert output.state == MissionState.ALIGN
    assert output.command.command == CMD_ALIGN_SAFE_ZONE
    assert output.command.flags & CMD_USE_FINAL_HEADING
    assert output.command.heading_cdeg == desired_heading * 100
    output = mission.step(VisionInput(), PoseInput(True, 0.0, desired_y / 1000, desired_heading - 20), closed)
    assert output.command.command == CMD_ALIGN_SAFE_ZONE
    output = mission.step(VisionInput(), PoseInput(True, 0.0, desired_y / 1000, desired_heading), closed)
    assert output.state == MissionState.ENTER_SAFE_ZONE
    assert output.command.command == CMD_ENTER_SAFE_ZONE
    assert output.command.flags & CMD_USE_FINAL_HEADING
    assert output.command.heading_cdeg == desired_heading * 100

    contained = target(y=700, bbox=(590, 680, 100, 80), safe=safe_bbox)
    for _ in range(2):
        output = mission.step(contained, PoseInput(True, 0.0, desired_y / 1000, desired_heading), closed)
        assert output.state == MissionState.ENTER_SAFE_ZONE
    output = mission.step(contained, PoseInput(True, 0.0, desired_y / 1000, desired_heading), closed)
    assert output.state == MissionState.COMPLETE
    assert output.command.command == CMD_TASK_COMPLETE


def test_grab_timeout():
    clock = FakeClock()
    mission = RescueMission(
        MissionSettings(side="red", confirmation_frames=1, grab_timeout_s=3.0),
        clock=clock,
    )
    stm = Stm32Status(flags=STM_CLAW_VISIBLE, age_ms=5)
    mission.step(target(), PoseInput(), Stm32Status())
    mission.step(target(), PoseInput(), stm)
    output = mission.step(target(), PoseInput(), stm)
    assert output.state == MissionState.GRABBING
    clock.advance(2.99)
    output = mission.step(target(), PoseInput(), stm)
    assert output.command.command == CMD_GRAB_CONFIRMED
    clock.advance(0.02)
    output = mission.step(target(), PoseInput(), stm)
    assert output.state == MissionState.FAULT
    assert output.command.command == 0


def main():
    run_side("red", 950, 90, (400, 600, 480, 300))
    run_side("blue", -950, 270, (400, 600, 480, 300))
    test_grab_timeout()
    print("mission state machine PASS")


if __name__ == "__main__":
    main()
