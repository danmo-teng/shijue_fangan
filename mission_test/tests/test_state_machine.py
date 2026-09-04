#!/usr/bin/env python3
from __future__ import annotations

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
    STM_CLAW_VISIBLE,
    Stm32Status,
)


def target(x=640, y=512, bbox=(600, 470, 80, 80), safe=None):
    return VisionInput(True, x, y, bbox, safe is not None, safe)


def run_side(side: str, desired_y: int, desired_heading: int, safe_bbox):
    mission = RescueMission(MissionSettings(side=side, confirmation_frames=3))
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
    assert output.state == MissionState.NAVIGATE
    assert output.command.command == CMD_GRAB_CONFIRMED

    output = mission.step(VisionInput(), PoseInput(True, 0.7, 0.0, 0), stm)
    assert output.command.command == CMD_NAVIGATE_WAYPOINT
    assert output.command.target_y_mm == desired_y
    output = mission.step(VisionInput(), PoseInput(True, 0.0, desired_y / 1000, 0), stm)
    assert output.state == MissionState.ALIGN
    output = mission.step(VisionInput(), PoseInput(True, 0.0, desired_y / 1000, desired_heading - 20), stm)
    assert output.command.command == CMD_ALIGN_SAFE_ZONE
    output = mission.step(VisionInput(), PoseInput(True, 0.0, desired_y / 1000, desired_heading), stm)
    assert output.state == MissionState.ENTER_SAFE_ZONE
    assert output.command.command == CMD_ENTER_SAFE_ZONE

    contained = target(y=700, bbox=(590, 680, 100, 80), safe=safe_bbox)
    for _ in range(2):
        output = mission.step(contained, PoseInput(True, 0.0, desired_y / 1000, desired_heading), stm)
        assert output.state == MissionState.ENTER_SAFE_ZONE
    output = mission.step(contained, PoseInput(True, 0.0, desired_y / 1000, desired_heading), stm)
    assert output.state == MissionState.COMPLETE
    assert output.command.command == CMD_TASK_COMPLETE


def main():
    run_side("red", 950, 90, (400, 600, 480, 300))
    run_side("blue", -950, 270, (400, 600, 480, 300))
    print("mission state machine PASS")


if __name__ == "__main__":
    main()
