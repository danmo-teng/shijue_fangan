#!/usr/bin/env python3
from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(PROJECT / "vision"))

from state_machine import (
    MissionSettings,
    MissionState,
    PoseInput,
    RescueMission,
    VisionInput,
    robot_intersects_safe_zone,
)
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


def target(x=640, y=512, bbox=(600, 470, 80, 80), class_name="green_supply"):
    return VisionInput(True, x, y, bbox, class_name)


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def run_side(side: str, desired_y: int, desired_heading: int):
    clock = FakeClock()
    mission = RescueMission(
        MissionSettings(side=side, confirmation_frames=3),
        clock=clock,
    )
    output = mission.step(VisionInput(), PoseInput(), Stm32Status())
    assert output.state == MissionState.SEARCH and not output.report.found
    output = mission.step(
        target(class_name="core_black"), PoseInput(), Stm32Status()
    )
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
    expected_x = -150 if side == "red" else 150
    assert output.command.target_x_mm == expected_x
    assert output.command.target_y_mm == desired_y
    assert output.command.flags & CMD_USE_FINAL_HEADING
    expected_bearing = math.degrees(math.atan2(
        desired_y / 1000 - pose.y_m,
        expected_x / 1000 - pose.x_m,
    )) % 360
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

    # Target visibility no longer decides delivery. Map contact and stationary
    # fused position are the only completion inputs.
    separated_y = 0.95 if side == "red" else -0.95
    for _ in range(4):
        output = mission.step(
            VisionInput(), PoseInput(True, 0.0, separated_y, desired_heading),
            Stm32Status(mode=14, age_ms=5),
        )
        assert output.state == MissionState.ENTER_SAFE_ZONE
    contact_y = 1.08 if side == "red" else -1.08
    assert robot_intersects_safe_zone(
        PoseInput(True, 0.0, contact_y, desired_heading), mission.settings
    )
    contact = PoseInput(True, 0.0, contact_y, desired_heading)
    # Being stationary while the claw is only opening cannot prematurely
    # complete delivery; the F407 must have reached RAM_FORWARD/RAM_VERIFY.
    output = mission.step(VisionInput(), contact, Stm32Status(mode=12, age_ms=5))
    clock.advance(2.0)
    output = mission.step(VisionInput(), contact, Stm32Status(mode=12, age_ms=5))
    assert output.state == MissionState.ENTER_SAFE_ZONE
    output = mission.step(VisionInput(), contact, Stm32Status(mode=14, age_ms=5))
    assert output.state == MissionState.ENTER_SAFE_ZONE
    clock.advance(0.70)
    moved = PoseInput(True, 0.016, contact_y, desired_heading)
    output = mission.step(VisionInput(), moved, Stm32Status(mode=14, age_ms=5))
    assert output.state == MissionState.ENTER_SAFE_ZONE
    clock.advance(0.79)
    output = mission.step(VisionInput(), moved, Stm32Status(mode=14, age_ms=5))
    assert output.state == MissionState.ENTER_SAFE_ZONE
    clock.advance(0.02)
    output = mission.step(
        VisionInput(), moved, Stm32Status(mode=14, age_ms=5)
    )
    assert output.state == MissionState.COMPLETE
    assert output.command.command == CMD_TASK_COMPLETE
    assert output.contact_pose == (moved.x_m, contact_y, float(desired_heading))
    assert mission.delivered_common and mission.delivery_count == 1

    # F407 opens the claw, backs out, faces the field centre and then reports
    # SEARCH. The RDK waits for those mode acknowledgements before a new cycle.
    output = mission.step(VisionInput(), moved, Stm32Status(mode=16, age_ms=5))
    assert output.state == MissionState.RETURN_CENTER and output.command is None
    output = mission.step(VisionInput(), moved, Stm32Status(mode=17, age_ms=5))
    assert output.state == MissionState.RETURN_CENTER
    output = mission.step(VisionInput(), moved, Stm32Status(mode=3, age_ms=5))
    assert output.state == MissionState.SEARCH
    assert mission.selected_class is None
    assert set(mission.allowed_classes) == {
        "green_supply", "core_black", "danger_cyan", "injured_orange"
    }

    # After the mandatory ordinary supply, an injured target is accepted and
    # uses the centre of the opposite (injured-person) half-zone.
    output = mission.step(target(class_name="injured_orange"), moved, Stm32Status())
    assert output.state == MissionState.APPROACH
    assert output.report.cargo_class == "injured_orange"
    injured_x = 150 if side == "red" else -150
    assert round(mission.approach_point[0] * 1000) == injured_x


def test_grab_wait_has_no_timeout():
    clock = FakeClock()
    mission = RescueMission(
        MissionSettings(side="red", confirmation_frames=1),
        clock=clock,
    )
    stm = Stm32Status(flags=STM_CLAW_VISIBLE, age_ms=5)
    mission.step(target(), PoseInput(), Stm32Status())
    mission.step(target(), PoseInput(), stm)
    output = mission.step(target(), PoseInput(), stm)
    assert output.state == MissionState.GRABBING
    clock.advance(3600.0)
    output = mission.step(target(), PoseInput(), stm)
    assert output.state == MissionState.GRABBING
    assert output.command.command == CMD_GRAB_CONFIRMED


def test_safe_zone_circle_geometry():
    red = MissionSettings(side="red")
    blue = MissionSettings(side="blue")
    assert not robot_intersects_safe_zone(PoseInput(), red)
    # Regression for the recorded premature stop: this blue-side position is
    # still about 326 mm away from first circle/zone contact.
    assert not robot_intersects_safe_zone(PoseInput(True, 0.043, -0.754, 267.4), blue)
    assert not robot_intersects_safe_zone(PoseInput(True, 0.0, 1.079, 90), red)
    assert robot_intersects_safe_zone(PoseInput(True, 0.0, 1.08, 90), red)
    assert robot_intersects_safe_zone(PoseInput(True, 0.0, -1.08, 270), blue)
    assert not robot_intersects_safe_zone(PoseInput(True, 0.0, -1.079, 270), blue)
    assert not robot_intersects_safe_zone(PoseInput(True, 0.421, 1.20, 90), red)
    assert robot_intersects_safe_zone(PoseInput(True, 0.42, 1.20, 90), red)


def main():
    run_side("red", 1200, 90)
    run_side("blue", -1200, 270)
    test_grab_wait_has_no_timeout()
    test_safe_zone_circle_geometry()
    print("mission state machine PASS")


if __name__ == "__main__":
    main()
