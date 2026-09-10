#!/usr/bin/env python3
from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from protocol import (  # noqa: E402
    CMD_APPROACH_TARGET,
    CMD_CARGO_AUDIT,
    CMD_ESCAPE_MANEUVER,
    CMD_DISPERSE_PILE,
    CMD_GRAB_CONFIRMED,
    CMD_HOLD,
    CMD_NAVIGATE_WAYPOINT,
    CMD_RELEASE_BOTH,
    CMD_RELEASE_LEFT,
    CMD_RELEASE_RIGHT,
    CMD_RETURN_CENTER,
    CMD_YIELD_BACKOFF,
)
from state_machine import (  # noqa: E402
    CargoAudit,
    CargoBatch,
    CompetitionMission,
    CompetitionSettings,
    CompetitionState,
    PoseSnapshot,
    StmSnapshot,
    TrackedCargo,
    VisionSnapshot,
)


def pose(x=-1.0, y=1.0) -> PoseSnapshot:
    return PoseSnapshot(True, x, y, 135.0, 5.0)


def stm(mode=3, flags=0) -> StmSnapshot:
    return StmSnapshot(mode, flags, 5.0, 0, 0)


def cargo(track_id, name, x, y, hits=3, relative=None):
    return TrackedCargo(track_id, name, 0.75, (x, y, 40, 40), relative, hits, 0, True, False)


def start_search(mission: CompetitionMission) -> None:
    output = mission.step(VisionSnapshot(), pose(), stm(), 0.0)
    assert output.state == CompetitionState.SEARCH


def test_start_handshake_suppresses_task_frames_until_start_clear() -> None:
    mission = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False)
    )
    start_pose = PoseSnapshot(True, -1.35, 1.35, 135.0, 5.0)

    for now, mode in ((0.0, 1), (0.1, 2)):
        output = mission.step(VisionSnapshot(), start_pose, stm(mode=mode), now)
        assert output.state == CompetitionState.WAIT_START
        assert output.command is None
        assert output.suppress_command_tx
        assert output.suppression_reason == "f407_autonomous_start"

    output = mission.step(
        VisionSnapshot(),
        PoseSnapshot(True, -1.10, 1.35, 135.0, 5.0),
        stm(mode=3),
        0.2,
    )
    assert output.state == CompetitionState.SEARCH
    assert output.event == "start_clear"
    assert output.command and output.command.opcode == CMD_HOLD
    assert not output.suppress_command_tx


def test_first_green_and_stuck_recovery() -> None:
    mission = CompetitionMission(CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False))
    start_search(mission)
    green = cargo(1, "green_supply", 600, 520, relative=(0.2, 0.7))
    vision = VisionSnapshot(
        frame_sequence=1,
        cargo=(green,),
        capture_cargo=(green,),
        capture_audit=CargoAudit(left_class="green_supply", left_count=1, total_count=1),
    )

    output = mission.step(vision, pose(), stm(), 0.1)
    assert output.state == CompetitionState.APPROACH
    assert output.batch and output.batch.classes == ("green_supply",)
    assert output.command and output.command.opcode == CMD_APPROACH_TARGET

    output = mission.step(vision, pose(), stm(flags=1), 0.2)
    assert output.state == CompetitionState.CAPTURE_AUDIT
    assert output.command and output.command.opcode == CMD_CARGO_AUDIT
    vision = replace(vision, frame_sequence=2)
    output = mission.step(vision, pose(), stm(flags=1), 0.3)
    assert output.state == CompetitionState.CAPTURE_AUDIT
    vision = replace(vision, frame_sequence=3)
    output = mission.step(vision, pose(), stm(flags=1), 0.4)
    assert output.state == CompetitionState.GRAB
    assert output.command and output.command.opcode == CMD_GRAB_CONFIRMED

    output = mission.step(vision, pose(), stm(mode=22, flags=3), 0.5)
    assert output.state == CompetitionState.NAVIGATE
    assert output.command and output.command.opcode == CMD_NAVIGATE_WAYPOINT

    # The first progress sample establishes the motion baseline; the next
    # stalled sample triggers lower-level yield before any spin escape.
    mission.step(vision, pose(), stm(mode=10, flags=4), 0.6)
    output = mission.step(vision, pose(), stm(mode=10, flags=4), 2.2)
    assert output.state == CompetitionState.STUCK_YIELD
    assert output.command and output.command.opcode == CMD_YIELD_BACKOFF
    output = mission.step(vision, pose(), stm(mode=30, flags=0), 4.1)
    assert output.state == CompetitionState.STUCK_ESCAPE
    assert output.command and output.command.opcode == CMD_ESCAPE_MANEUVER

    no_observation = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False)
    )
    start_search(no_observation)
    no_observation.selected_batch = CargoBatch((1,), ("green_supply",), "material")
    no_observation._set_state(CompetitionState.CAPTURE_AUDIT, 3.0)
    output = no_observation.step(VisionSnapshot(), pose(), stm(flags=1), 3.1)
    for now in (3.2, 3.3, 3.4):
        output = no_observation.step(VisionSnapshot(), pose(), stm(flags=1), now)
    assert output.state == CompetitionState.CAPTURE_AUDIT
    assert output.command and output.command.opcode == CMD_CARGO_AUDIT


def test_initial_stash_and_invalid_release() -> None:
    mission = CompetitionMission(CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=True))
    output = mission.step(VisionSnapshot(), pose(), stm(), 0.0)
    assert output.state == CompetitionState.INITIAL_OBSERVE
    pile = (
        cargo(1, "green_supply", 400, 450),
        cargo(2, "core_black", 540, 470),
    )
    vision = VisionSnapshot(
        frame_sequence=1,
        cargo=pile,
        capture_cargo=pile,
        capture_audit=CargoAudit(
            left_class="mixed_material",
            right_class="",
            left_count=2,
            right_count=0,
            total_count=2,
        ),
    )
    output = mission.step(vision, pose(), stm(), 0.1)
    assert output.state == CompetitionState.INITIAL_APPROACH
    output = mission.step(vision, pose(), stm(flags=1), 0.2)
    assert output.state == CompetitionState.CAPTURE_AUDIT
    for now, sequence in ((0.3, 2), (0.4, 3)):
        output = mission.step(
            replace(vision, frame_sequence=sequence), pose(), stm(flags=1), now
        )
    assert output.state == CompetitionState.GRAB
    output = mission.step(vision, pose(), stm(mode=22, flags=3), 0.5)
    assert output.state == CompetitionState.INITIAL_STASH_NAV
    output = mission.step(vision, PoseSnapshot(True, -0.85, 0.55, 135.0, 5.0), stm(mode=10, flags=4), 1.0)
    assert output.state == CompetitionState.INITIAL_RELEASE
    output = mission.step(vision, PoseSnapshot(True, -0.85, 0.55, 135.0, 5.0), stm(mode=34, flags=0), 1.6)
    assert output.state == CompetitionState.RETURN_CENTER
    assert mission.initial_stash_done

    invalid_mission = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=True)
    )
    invalid_mission._set_state(CompetitionState.INITIAL_OBSERVE, 2.0)
    invalid_mission.initial_stash_done = False
    invalid_mission.selected_batch = CargoBatch(
        (4, 5), ("green_supply", "danger_cyan"), "stash", initial_stash=True
    )
    invalid_mission._set_state(CompetitionState.CAPTURE_AUDIT, 2.0)
    invalid = VisionSnapshot(
        frame_sequence=2,
        capture_audit=CargoAudit(
            left_class="green_supply",
            right_class="danger_cyan",
            left_count=1,
            right_count=1,
            total_count=2,
            danger_present=True,
        ),
    )
    invalid_mission.step(invalid, pose(), stm(flags=1), 2.1)
    invalid_mission.step(replace(invalid, frame_sequence=3), pose(), stm(flags=1), 2.2)
    output = invalid_mission.step(replace(invalid, frame_sequence=4), pose(), stm(flags=1), 2.3)
    assert output.state == CompetitionState.INVALID_RELEASE
    assert output.command and output.command.opcode == CMD_RELEASE_RIGHT


def test_material_priority_excludes_danger() -> None:
    mission = CompetitionMission(CompetitionSettings(side="blue", start_zone=4, initial_stash_enabled=False))
    start_search(mission)
    mission.first_common_delivered = True
    material = cargo(1, "green_supply", 450, 450, relative=(0.2, 0.5))
    core = cargo(2, "core_black", 520, 450, relative=(0.25, 0.55))
    injury = cargo(3, "injured_orange", 760, 450, relative=(0.9, 0.8))
    danger = cargo(4, "danger_cyan", 900, 450, relative=(0.4, 0.3))
    output = mission.step(
        VisionSnapshot(cargo=(material, core, injury, danger)),
        pose(1.0, -1.0),
        stm(),
        0.1,
    )
    assert output.state == CompetitionState.APPROACH
    assert output.batch and set(output.batch.classes) == {"green_supply", "core_black"}
    assert "danger_cyan" not in output.batch.classes

    mission = CompetitionMission(CompetitionSettings(side="blue", start_zone=4, initial_stash_enabled=False))
    start_search(mission)
    mission.first_common_delivered = True
    far_material = cargo(5, "green_supply", 450, 450, relative=(1.2, 1.0))
    near_injury = cargo(6, "injured_orange", 520, 450, relative=(0.3, 0.4))
    output = mission.step(
        VisionSnapshot(cargo=(far_material, near_injury)),
        pose(1.0, -1.0),
        stm(),
        0.2,
    )
    assert output.state == CompetitionState.APPROACH
    assert output.batch and output.batch.destination == "injury"
    assert output.batch.classes == ("injured_orange",)


def _audit_mission(destination: str = "material") -> CompetitionMission:
    mission = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False)
    )
    mission.first_common_delivered = True
    classes = ("green_supply", "core_black") if destination == "material" else ("injured_orange",)
    mission.selected_batch = CargoBatch((1, 2), classes, destination)
    mission._set_state(CompetitionState.CAPTURE_AUDIT, 0.0)
    return mission


def _run_invalid_audit(mission: CompetitionMission, audit: CargoAudit) -> object:
    output = None
    for sequence in (1, 2, 3):
        output = mission.step(
            VisionSnapshot(frame_sequence=sequence, capture_audit=audit),
            pose(),
            stm(flags=1),
            sequence * 0.1,
        )
    assert output is not None
    return output


def test_disperse_requires_fresh_green_and_respects_limit() -> None:
    crowded_green = cargo(1, "green_supply", 400, 450, relative=(0.20, 0.50))
    neighbour = cargo(2, "core_black", 460, 450, relative=(0.24, 0.52))
    crowded = VisionSnapshot(cargo=(crowded_green, neighbour))

    active = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False)
    )
    start_search(active)
    output = active.step(crowded, pose(), stm(), 0.1)
    assert output.state == CompetitionState.DISPERSE
    assert output.command and output.command.opcode == CMD_DISPERSE_PILE
    assert active.disperse_attempts == 1

    stale = replace(crowded, observed_monotonic_s=0.0)
    mission = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False)
    )
    start_search(mission)
    output = mission.step(stale, pose(), stm(), 1.0)
    assert output.state == CompetitionState.SEARCH
    assert output.command and output.command.opcode == CMD_HOLD

    no_target = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False)
    )
    start_search(no_target)
    output = no_target.step(VisionSnapshot(), pose(), stm(), 0.1)
    assert output.command and output.command.opcode == CMD_HOLD

    closed = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False)
    )
    start_search(closed)
    output = closed.step(crowded, pose(), stm(flags=2), 0.1)
    assert output.command and output.command.opcode == CMD_HOLD

    limited = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False)
    )
    start_search(limited)
    limited.disperse_attempts = limited.settings.disperse_limit
    output = limited.step(crowded, pose(), stm(), 0.1)
    assert output.state == CompetitionState.SEARCH
    assert output.command and output.command.opcode == CMD_HOLD

    output = active.step(VisionSnapshot(), pose(), stm(), 0.5)
    assert output.state == CompetitionState.DISPERSE
    assert output.command and output.command.opcode == CMD_HOLD
    output = active.step(crowded, pose(), StmSnapshot(35, 0, 300.0, 0, 0), 1.0)
    assert output.state == CompetitionState.DISPERSE
    output = active.step(crowded, pose(), stm(mode=35), 1.1)
    assert output.state == CompetitionState.SEARCH


def test_release_side_selection() -> None:
    danger_left = CargoAudit(
        left_class="danger_cyan", right_class="green_supply",
        left_count=1, right_count=1, total_count=2, danger_present=True,
    )
    assert _run_invalid_audit(_audit_mission(), danger_left).command.opcode == CMD_RELEASE_LEFT

    danger_right = replace(danger_left, left_class="green_supply", right_class="danger_cyan")
    assert _run_invalid_audit(_audit_mission(), danger_right).command.opcode == CMD_RELEASE_RIGHT

    injury_left = CargoAudit(
        left_class="injured_orange", right_class="green_supply",
        left_count=1, right_count=1, total_count=2, injury_mixed=True,
    )
    assert _run_invalid_audit(_audit_mission(), injury_left).command.opcode == CMD_RELEASE_LEFT

    injury_task = _audit_mission("injury")
    injury_right = replace(injury_left, left_class="green_supply", right_class="injured_orange")
    assert _run_invalid_audit(injury_task, injury_right).command.opcode == CMD_RELEASE_LEFT

    overflow = CargoAudit(
        left_class="green_supply", right_class="core_black",
        left_count=3, right_count=1, total_count=4,
        left_selected_count=3, right_selected_count=0,
    )
    assert _run_invalid_audit(_audit_mission(), overflow).command.opcode == CMD_RELEASE_RIGHT

    one_side_overflow = CargoAudit(
        left_class="green_supply", left_count=4, total_count=4,
    )
    assert _run_invalid_audit(_audit_mission(), one_side_overflow).command.opcode == CMD_RELEASE_BOTH

    unknown = CargoAudit(
        left_class="unknown", right_class="unknown",
        left_count=1, right_count=1, total_count=2, unknown_present=True,
    )
    assert _run_invalid_audit(_audit_mission(), unknown).command.opcode == CMD_RELEASE_BOTH
    unknown_with_material = replace(
        unknown, right_class="green_supply", right_count=1, total_count=2
    )
    assert _run_invalid_audit(_audit_mission(), unknown_with_material).command.opcode == CMD_RELEASE_BOTH
    unknown_without_flag = replace(unknown_with_material, unknown_present=False)
    assert _run_invalid_audit(_audit_mission(), unknown_without_flag).command.opcode == CMD_RELEASE_BOTH


def test_single_release_recheck_and_final_both_release() -> None:
    mission = _audit_mission()
    invalid = VisionSnapshot(
        frame_sequence=1,
        capture_audit=CargoAudit(
            left_class="danger_cyan", right_class="green_supply",
            left_count=1, right_count=1, total_count=2, danger_present=True,
        ),
    )
    output = _run_invalid_audit(mission, invalid.capture_audit)
    assert output.command and output.command.opcode == CMD_RELEASE_LEFT

    output = mission.step(invalid, pose(), StmSnapshot(32, 0, 5.0, 0, 0), 0.4)
    assert output.state == CompetitionState.INVALID_BACKOFF
    assert output.command and output.command.opcode == CMD_YIELD_BACKOFF
    output = mission.step(invalid, pose(), StmSnapshot(30, 0, 5.0, 0, 0), 0.5)
    assert output.state == CompetitionState.CAPTURE_AUDIT
    assert output.command and output.command.opcode == CMD_HOLD
    assert mission.cargo_recheck_pending and mission.selected_batch is not None

    valid = CargoAudit(right_class="green_supply", right_count=1, total_count=1)
    for sequence in (2, 3, 4):
        output = mission.step(
            VisionSnapshot(frame_sequence=sequence, capture_audit=valid),
            pose(), stm(flags=1), 0.5 + sequence * 0.1,
        )
    assert output.state == CompetitionState.GRAB
    output = mission.step(
        VisionSnapshot(frame_sequence=5, capture_audit=valid),
        pose(), StmSnapshot(22, 3, 5.0, 0, 0), 1.0,
    )
    assert output.state == CompetitionState.NAVIGATE
    assert not mission.cargo_recheck_pending

    final = _audit_mission()
    _run_invalid_audit(final, invalid.capture_audit)
    final.step(invalid, pose(), StmSnapshot(32, 0, 5.0, 0, 0), 0.4)
    final.step(invalid, pose(), StmSnapshot(30, 0, 5.0, 0, 0), 0.5)
    for sequence in (2, 3, 4):
        output = final.step(
            VisionSnapshot(frame_sequence=sequence, capture_audit=invalid.capture_audit),
            pose(), stm(flags=1), 0.5 + sequence * 0.1,
        )
    assert output.state == CompetitionState.INVALID_RELEASE
    assert output.command and output.command.opcode == CMD_RELEASE_BOTH
    output = final.step(invalid, pose(), StmSnapshot(34, 0, 5.0, 0, 0), 1.2)
    assert output.state == CompetitionState.INVALID_BACKOFF
    output = final.step(invalid, pose(), StmSnapshot(30, 0, 5.0, 0, 0), 1.3)
    assert output.state == CompetitionState.SEARCH
    assert final.selected_batch is None


def test_return_center_sends_zero_until_search() -> None:
    mission = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False)
    )
    mission._set_state(CompetitionState.RETURN_CENTER, 0.0)
    output = mission.step(
        VisionSnapshot(), PoseSnapshot(True, 0.60, 0.0, 180.0, 5.0),
        StmSnapshot(17, 0, 5.0, 0, 0), 0.1,
    )
    assert output.state == CompetitionState.RETURN_CENTER
    assert output.command and output.command.opcode == CMD_RETURN_CENTER
    assert output.command.arg_a == 600
    output = mission.step(
        VisionSnapshot(), PoseSnapshot(True, 0.01, 0.0, 180.0, 5.0),
        StmSnapshot(17, 0, 5.0, 0, 0), 0.2,
    )
    assert output.state == CompetitionState.RETURN_CENTER
    assert output.command and output.command.arg_a == 0
    output = mission.step(
        VisionSnapshot(), PoseSnapshot(True, 0.0, 0.0, 180.0, 5.0),
        StmSnapshot(3, 0, 5.0, 0, 0), 0.3,
    )
    assert output.state == CompetitionState.SEARCH


def test_delivery_requires_outside_to_inside_transition() -> None:
    mission = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False)
    )
    mission.first_common_delivered = False
    mission.selected_batch = CargoBatch((1,), ("green_supply",), "material")
    mission._set_state(CompetitionState.NAVIGATE, 0.0)
    fence_pose = PoseSnapshot(True, -0.15, 1.0275, 90.0, 5.0)
    outside = VisionSnapshot(
        frame_sequence=1,
        delivery_target_found=True,
        delivery_target_outside_safe_zone=True,
    )
    output = mission.step(outside, fence_pose, stm(mode=10, flags=34), 0.1)
    assert output.state == CompetitionState.ENTER_SAFE_ZONE

    for sequence in range(2, 7):
        inside = VisionSnapshot(
            frame_sequence=sequence,
            delivery_target_found=True,
            delivery_target_inside_safe_zone=True,
        )
        output = mission.step(inside, fence_pose, stm(mode=15, flags=34), sequence * 0.1)
    assert output.state == CompetitionState.TASK_COMPLETE
    assert mission.delivery_count == 1
    assert mission.first_common_delivered


def test_boundary_guard_points_back_into_field() -> None:
    mission = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False)
    )
    assert mission._boundary_risk(
        PoseSnapshot(True, 1.40, 0.0, 0.0, 5.0), (1.45, 0.0)
    )
    assert not mission._boundary_risk(
        PoseSnapshot(True, 1.40, 0.0, 0.0, 5.0), (0.8, 0.0)
    )


def main() -> None:
    test_start_handshake_suppresses_task_frames_until_start_clear()
    test_first_green_and_stuck_recovery()
    test_initial_stash_and_invalid_release()
    test_material_priority_excludes_danger()
    test_disperse_requires_fresh_green_and_respects_limit()
    test_release_side_selection()
    test_single_release_recheck_and_final_both_release()
    test_return_center_sends_zero_until_search()
    test_delivery_requires_outside_to_inside_transition()
    test_boundary_guard_points_back_into_field()
    print("competition state machine PASS")


if __name__ == "__main__":
    main()
