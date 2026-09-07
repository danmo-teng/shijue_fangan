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
    CMD_DISTANCE_VALID,
    CMD_ENTER_SAFE_ZONE,
    CMD_GRAB_CONFIRMED,
    CMD_NAVIGATE_WAYPOINT,
    CMD_RETURN_CENTER,
    CMD_TASK_COMPLETE,
    CMD_USE_FINAL_HEADING,
    STM_CLAW_VISIBLE,
    STM_DISTANCE_DONE,
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
    target_x = -0.15 if side == "red" else 0.15
    target_y = 1.0275 if side == "red" else -1.0275
    expected_distance = math.hypot(target_x - pose.x_m, target_y - pose.y_m)
    assert output.command.target_x_mm == round(expected_distance * 1000)
    assert output.command.target_y_mm == 0
    assert output.command.flags & CMD_USE_FINAL_HEADING
    assert output.command.flags & CMD_DISTANCE_VALID
    expected_bearing = math.degrees(math.atan2(
        target_y - pose.y_m,
        target_x - pose.x_m,
    )) % 360
    assert output.command.heading_cdeg == round(expected_bearing * 100) % 36000

    # Navigation is closed-loop on the RDK: every fresh pose produces a new
    # heading and remaining distance instead of replaying the initial route.
    updated_pose = PoseInput(True, 0.45, -0.20 if side == "blue" else 0.20, 0)
    updated = mission.step(VisionInput(), updated_pose, closed)
    updated_distance = math.hypot(
        target_x - updated_pose.x_m, target_y - updated_pose.y_m
    )
    updated_bearing = math.degrees(math.atan2(
        target_y - updated_pose.y_m, target_x - updated_pose.x_m,
    )) % 360
    assert updated.command.target_x_mm == round(updated_distance * 1000)
    assert updated.command.heading_cdeg == round(updated_bearing * 100) % 36000
    assert updated.command.target_x_mm != output.command.target_x_mm

    # A fresh NAV distance-done status must release the strict map-contact
    # gate even with a 40 mm localization bias outside the tangent boundary.
    arrival_y = 0.9875 if side == "red" else -0.9875
    arrival_x = target_x
    biased_arrival = PoseInput(True, arrival_x, arrival_y, desired_heading)
    assert not robot_intersects_safe_zone(biased_arrival, mission.settings)
    nav_done = Stm32Status(
        flags=STM_GRIPPER_CLOSED | STM_DISTANCE_DONE,
        mode=10,
        age_ms=5,
    )
    output = mission.step(VisionInput(), biased_arrival, nav_done)
    assert output.state == MissionState.ENTER_SAFE_ZONE
    assert mission.delivery_arrival_confirmed
    assert output.command.command == CMD_ENTER_SAFE_ZONE
    assert output.command.flags & CMD_USE_FINAL_HEADING
    assert output.command.heading_cdeg == desired_heading * 100

    # Only fresh mode 15 (CHECK/RAM_VERIFY) may confirm placement.
    contact_y = 1.035 if side == "red" else -1.035
    biased_check = PoseInput(True, arrival_x, arrival_y, desired_heading)
    output = mission.step(VisionInput(), biased_check, Stm32Status(mode=14, age_ms=5))
    clock.advance(0.49)
    output = mission.step(VisionInput(), biased_check, Stm32Status(mode=14, age_ms=5))
    assert output.state == MissionState.ENTER_SAFE_ZONE
    output = mission.step(VisionInput(), biased_check, Stm32Status(mode=15, age_ms=5))
    assert output.state == MissionState.ENTER_SAFE_ZONE
    clock.advance(0.70)
    moved = PoseInput(True, arrival_x + 0.026, arrival_y, desired_heading)
    output = mission.step(VisionInput(), moved, Stm32Status(mode=15, age_ms=5))
    assert output.state == MissionState.ENTER_SAFE_ZONE
    clock.advance(0.79)
    output = mission.step(VisionInput(), moved, Stm32Status(mode=15, age_ms=5))
    assert output.state == MissionState.ENTER_SAFE_ZONE
    clock.advance(0.02)
    output = mission.step(
        VisionInput(), moved, Stm32Status(mode=15, age_ms=5)
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
    assert output.command.command == CMD_RETURN_CENTER
    assert output.command.flags & CMD_DISTANCE_VALID
    expected_return_distance = max(0.0, math.hypot(moved.x_m, moved.y_m) - 0.60)
    assert output.command.target_x_mm == round(expected_return_distance * 1000)
    assert output.command.target_y_mm == 0
    expected_return_heading = math.degrees(math.atan2(-moved.y_m, -moved.x_m)) % 360
    assert output.command.heading_cdeg == round(expected_return_heading * 100) % 36000
    closer = PoseInput(
        True,
        moved.x_m * 0.7,
        moved.y_m * 0.7,
        desired_heading,
    )
    updated_return = mission.step(
        VisionInput(), closer, Stm32Status(mode=17, age_ms=5)
    )
    updated_return_distance = max(0.0, math.hypot(closer.x_m, closer.y_m) - 0.60)
    updated_return_heading = math.degrees(
        math.atan2(-closer.y_m, -closer.x_m)
    ) % 360
    assert updated_return.command.target_x_mm == round(updated_return_distance * 1000)
    assert updated_return.command.heading_cdeg == round(updated_return_heading * 100) % 36000
    assert updated_return.command.target_x_mm < output.command.target_x_mm
    output = mission.step(VisionInput(), moved, Stm32Status(mode=3, age_ms=5))
    assert output.state == MissionState.SEARCH
    assert mission.selected_class is None
    assert not mission.delivery_arrival_confirmed
    assert mission.delivery_stationary_started_s is None
    assert mission.delivery_stationary_anchor is None
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
    assert red.robot_body_radius_m == 0.130
    assert red.push_plate_offset_m == 0.105
    assert red.front_pusher_offset_m == 0.150
    assert red.zone_center_x_abs_m == 0.150
    red_mission = RescueMission(red)
    red_mission.selected_class = "green_supply"
    assert math.isclose(red_mission.fence_contact_center[0], -0.15)
    assert math.isclose(red_mission.fence_contact_center[1], 1.035)
    assert math.isclose(red_mission.fence_stop_point[0], -0.15)
    assert math.isclose(red_mission.fence_stop_point[1], 1.0275)
    assert red_mission._at_fence_stop(PoseInput(True, -0.15, 1.0275, 90))
    assert not red_mission._at_fence_stop(PoseInput(True, -0.15, 1.0275, 55))
    assert not robot_intersects_safe_zone(PoseInput(), red)
    # Regression for the recorded premature stop: this blue-side position is
    # still about 326 mm away from first circle/zone contact.
    assert not robot_intersects_safe_zone(PoseInput(True, 0.043, -0.754, 267.4), blue)
    assert not robot_intersects_safe_zone(PoseInput(True, 0.0, 1.069, 90), red)
    assert robot_intersects_safe_zone(PoseInput(True, 0.0, 1.07, 90), red)
    assert robot_intersects_safe_zone(PoseInput(True, 0.0, -1.07, 270), blue)
    assert not robot_intersects_safe_zone(PoseInput(True, 0.0, -1.069, 270), blue)
    assert not robot_intersects_safe_zone(PoseInput(True, 0.431, 1.20, 90), red)
    assert robot_intersects_safe_zone(PoseInput(True, 0.43, 1.20, 90), red)


def test_distance_done_requires_fresh_nav_status():
    mission = RescueMission(MissionSettings(side="red"))
    mission.state = MissionState.NAVIGATE
    mission.selected_class = "green_supply"
    pose = PoseInput(True, -0.15, 0.99, 90)
    assert not robot_intersects_safe_zone(pose, mission.settings)
    stale = Stm32Status(
        flags=STM_GRIPPER_CLOSED | STM_DISTANCE_DONE,
        mode=10,
        age_ms=251,
    )
    assert mission.step(VisionInput(), pose, stale).state == MissionState.NAVIGATE
    wrong_mode = Stm32Status(
        flags=STM_GRIPPER_CLOSED | STM_DISTANCE_DONE,
        mode=9,
        age_ms=5,
    )
    assert mission.step(VisionInput(), pose, wrong_mode).state == MissionState.NAVIGATE
    fresh = Stm32Status(
        flags=STM_GRIPPER_CLOSED | STM_DISTANCE_DONE,
        mode=10,
        age_ms=5,
    )
    output = mission.step(VisionInput(), pose, fresh)
    assert output.state == MissionState.ENTER_SAFE_ZONE
    assert output.command.command == CMD_ENTER_SAFE_ZONE


def main():
    run_side("red", 1200, 90)
    run_side("blue", -1200, 270)
    test_grab_wait_has_no_timeout()
    test_safe_zone_circle_geometry()
    test_distance_done_requires_fresh_nav_status()
    print("mission state machine PASS")


if __name__ == "__main__":
    main()
