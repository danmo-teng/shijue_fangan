#!/usr/bin/env python3
from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from protocol import (  # noqa: E402
    CMD_APPROACH_TARGET,
    CMD_ABORT,
    CMD_CARGO_AUDIT,
    CMD_CHANGE_LANE,
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
    STM_MODE_NAVIGATE,
    STM_MODE_RAM_VERIFY,
    STM_MODE_SEARCH,
    StmSnapshot,
    TrackedCargo,
    VisionSnapshot,
)


def pose(x=-1.0, y=1.0) -> PoseSnapshot:
    return PoseSnapshot(True, x, y, 135.0, 5.0)


def stm(mode=3, flags=0, age_ms=5.0, acknowledged_sequence=0) -> StmSnapshot:
    return StmSnapshot(mode, flags, age_ms, 0, acknowledged_sequence)


def relay_stm(
    output,
    *,
    mode=21,
    flags=1,
    acknowledged_sequence=1,
    relay_mission_tx_frames=1,
    age_ms=5.0,
) -> StmSnapshot:
    assert output.command is not None
    frame = output.command.to_frame(0)
    return StmSnapshot(
        mode=mode,
        flags=flags,
        age_ms=age_ms,
        acknowledged_sequence=acknowledged_sequence,
        relay_mission_tx_frames=relay_mission_tx_frames,
        relay_last_mission_command=frame[4],
        relay_last_mission_sequence=0,
        relay_last_mission_payload=tuple(frame[4:12]),
    )


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
    assert output.state == CompetitionState.AUDIT_CONFIRM
    assert output.command and output.command.opcode == CMD_CARGO_AUDIT
    output = mission.step(vision, pose(), relay_stm(output), 0.5)
    assert output.state == CompetitionState.GRAB
    assert output.command and output.command.opcode == CMD_GRAB_CONFIRMED

    output = mission.step(vision, pose(), stm(mode=22, flags=3), 0.5)
    assert output.state == CompetitionState.NAVIGATE
    assert output.command and output.command.opcode == CMD_NAVIGATE_WAYPOINT

    # The first progress sample establishes the motion baseline; the next
    # stalled sample triggers lower-level yield before any spin escape.
    mission.step(vision, pose(), stm(mode=10, flags=4, acknowledged_sequence=10), 0.6)
    output = mission.step(vision, pose(), stm(mode=10, flags=4, acknowledged_sequence=11), 2.2)
    assert output.state == CompetitionState.STUCK_YIELD
    assert output.command and output.command.opcode == CMD_YIELD_BACKOFF
    # A moving but still-running remote action must not resume the original
    # NAVIGATE command before F407 reports this YIELD_DONE.
    output = mission.step(
        vision,
        PoseSnapshot(True, -0.95, 1.0, 135.0, 5.0),
        stm(mode=25, flags=4, acknowledged_sequence=12),
        4.0,
    )
    assert output.state == CompetitionState.STUCK_YIELD
    assert output.command and output.command.opcode == CMD_YIELD_BACKOFF
    # A stale completion with the right mode is not enough either.
    output = mission.step(
        vision,
        PoseSnapshot(True, -0.95, 1.0, 135.0, 300.0),
        stm(mode=30, age_ms=300.0, acknowledged_sequence=12),
        3.9,
    )
    assert output.state == CompetitionState.STUCK_YIELD
    output = mission.step(
        vision,
        PoseSnapshot(True, -0.95, 1.0, 135.0, 5.0),
        stm(mode=30, acknowledged_sequence=12),
        4.0,
    )
    assert output.state == CompetitionState.NAVIGATE
    assert output.command and output.command.opcode == CMD_NAVIGATE_WAYPOINT

    # No progress after a fresh YIELD_DONE permits ESCAPE_MANEUVER, but the
    # escape action itself must also finish freshly before resuming.
    mission._set_state(CompetitionState.NAVIGATE, 5.0)
    mission.step(vision, pose(), stm(mode=10, flags=4, acknowledged_sequence=20), 5.0)
    output = mission.step(vision, pose(), stm(mode=10, flags=4, acknowledged_sequence=21), 6.6)
    assert output.state == CompetitionState.STUCK_YIELD
    output = mission.step(vision, pose(), stm(mode=30, acknowledged_sequence=22), 6.7)
    assert output.state == CompetitionState.STUCK_ESCAPE
    assert output.command and output.command.opcode == CMD_ESCAPE_MANEUVER
    output = mission.step(
        vision,
        PoseSnapshot(True, -0.95, 1.0, 135.0, 5.0),
        stm(mode=25, flags=4, acknowledged_sequence=23),
        7.0,
    )
    assert output.state == CompetitionState.STUCK_ESCAPE
    assert output.command and output.command.opcode == CMD_ESCAPE_MANEUVER
    output = mission.step(
        vision,
        PoseSnapshot(True, -0.95, 1.0, 135.0, 5.0),
        stm(mode=31, age_ms=300.0, acknowledged_sequence=23),
        7.1,
    )
    assert output.state == CompetitionState.STUCK_ESCAPE
    output = mission.step(
        vision,
        PoseSnapshot(True, -0.95, 1.0, 135.0, 5.0),
        stm(mode=31, acknowledged_sequence=23),
        7.2,
    )
    assert output.state == CompetitionState.NAVIGATE
    assert output.command and output.command.opcode == CMD_NAVIGATE_WAYPOINT

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


def test_search_recovery_handshake_and_safety_gates() -> None:
    mission = CompetitionMission(
        CompetitionSettings(
            side="red",
            start_zone=1,
            initial_stash_enabled=False,
            stuck_timeout_s=10.0,
            search_recovery_timeout_s=2.0,
        )
    )
    start_search(mission)
    target = cargo(7, "green_supply", 600, 520, relative=(0.2, 0.7))
    visible = VisionSnapshot(
        frame_sequence=1,
        observed_monotonic_s=0.0,
        cargo=(target,),
    )
    output = mission.step(visible, pose(), stm(mode=3, acknowledged_sequence=1), 0.0)
    assert output.state == CompetitionState.APPROACH

    missing = VisionSnapshot(frame_sequence=2, observed_monotonic_s=0.1)
    output = mission.step(
        missing,
        pose(),
        stm(mode=20, flags=4, acknowledged_sequence=2),
        0.1,
    )
    assert output.state == CompetitionState.APPROACH
    assert output.command and output.command.opcode == CMD_HOLD

    output = mission.step(
        replace(visible, frame_sequence=3, observed_monotonic_s=0.2),
        pose(),
        stm(mode=20, flags=4, acknowledged_sequence=3),
        0.2,
    )
    assert output.state == CompetitionState.APPROACH
    assert output.command and output.command.opcode == CMD_APPROACH_TARGET
    assert mission.target_lost_started_s is None

    output = mission.step(
        replace(missing, frame_sequence=4, observed_monotonic_s=0.3),
        pose(),
        stm(mode=20, flags=4, acknowledged_sequence=4),
        0.3,
    )
    assert output.state == CompetitionState.APPROACH
    output = mission.step(
        replace(missing, frame_sequence=5, observed_monotonic_s=1.6),
        pose(),
        stm(mode=20, flags=4, acknowledged_sequence=5),
        1.6,
    )
    assert output.state == CompetitionState.WAIT_SEARCH_RECOVERY
    assert output.command is None
    assert output.suppress_command_tx
    assert output.suppression_reason == "f407_search_recovery"
    assert output.event == "search_recovery_start"

    output = mission.step(
        replace(missing, frame_sequence=6, observed_monotonic_s=1.7),
        pose(),
        stm(mode=20, flags=4, acknowledged_sequence=6),
        1.7,
    )
    assert output.state == CompetitionState.WAIT_SEARCH_RECOVERY
    assert output.suppress_command_tx

    # An unrelated remote action is not an allowed recovery state.
    output = mission.step(
        replace(missing, frame_sequence=7, observed_monotonic_s=1.8),
        pose(),
        stm(mode=25, flags=4, acknowledged_sequence=7),
        1.8,
    )
    assert output.state == CompetitionState.WAIT_SEARCH_RECOVERY
    assert output.command and output.command.opcode == CMD_HOLD
    assert not output.suppress_command_tx

    output = mission.step(
        replace(missing, frame_sequence=8, observed_monotonic_s=1.9),
        pose(),
        stm(mode=24, flags=4, acknowledged_sequence=8),
        1.9,
    )
    assert output.state == CompetitionState.WAIT_SEARCH_RECOVERY
    assert output.suppress_command_tx

    output = mission.step(
        replace(missing, frame_sequence=9, observed_monotonic_s=2.0),
        pose(),
        stm(mode=3, acknowledged_sequence=9),
        2.0,
    )
    assert output.state == CompetitionState.SEARCH
    assert output.event == "search_recovery_complete"
    assert output.command and output.command.opcode == CMD_HOLD

    def recovery_mission() -> CompetitionMission:
        result = CompetitionMission(
            CompetitionSettings(
                side="red",
                start_zone=1,
                initial_stash_enabled=False,
                search_recovery_timeout_s=2.0,
            )
        )
        result._begin_search_recovery(CompetitionState.SEARCH, 0.0)
        return result

    stale_vision = recovery_mission()
    output = stale_vision.step(
        VisionSnapshot(observed_monotonic_s=0.0),
        pose(),
        stm(mode=20, flags=4, acknowledged_sequence=1),
        0.4,
    )
    assert output.command and output.command.opcode == CMD_HOLD
    assert not output.suppress_command_tx
    assert output.reason == "vision_stale"

    stale_stm = recovery_mission()
    output = stale_stm.step(
        VisionSnapshot(observed_monotonic_s=0.5),
        pose(),
        stm(mode=20, flags=4, age_ms=300.0, acknowledged_sequence=1),
        0.5,
    )
    assert output.command and output.command.opcode == CMD_HOLD
    assert output.reason == "stm_stale"

    closed = recovery_mission()
    output = closed.step(
        VisionSnapshot(observed_monotonic_s=0.5),
        pose(),
        stm(mode=20, flags=2, acknowledged_sequence=1),
        0.5,
    )
    assert output.command and output.command.opcode == CMD_HOLD
    assert output.reason == "gripper_closed"

    pending_audit = recovery_mission()
    pending_audit.cargo_recheck_pending = True
    output = pending_audit.step(
        VisionSnapshot(observed_monotonic_s=0.5),
        pose(),
        stm(mode=20, flags=4, acknowledged_sequence=1),
        0.5,
    )
    assert output.command and output.command.opcode == CMD_HOLD
    assert output.reason == "cargo_recheck_pending"

    timed_out = recovery_mission()
    output = timed_out.step(
        VisionSnapshot(observed_monotonic_s=2.1),
        pose(),
        stm(mode=25, flags=4, acknowledged_sequence=1),
        2.1,
    )
    assert output.state == CompetitionState.FAULT
    assert output.command and output.command.opcode == CMD_ABORT
    assert output.event == "search_recovery_timeout"

    faulted = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False)
    )
    output = faulted.step(
        VisionSnapshot(),
        pose(),
        StmSnapshot(mode=20, age_ms=5.0, fault_code=6),
        0.1,
    )
    assert output.state == CompetitionState.FAULT
    assert faulted.first_fault_code == 6
    faulted.step(VisionSnapshot(), pose(), stm(mode=20), 0.2)
    assert faulted.first_fault_code == 6


def test_return_stash_requires_confirmed_cargo_and_search_handoff() -> None:
    no_stash = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False)
    )
    no_stash.first_common_delivered = True
    start_search(no_stash)
    output = no_stash.step(
        VisionSnapshot(), pose(), stm(mode=STM_MODE_SEARCH, acknowledged_sequence=1), 2.1
    )
    assert output.state == CompetitionState.SEARCH
    assert output.command and output.command.opcode == CMD_HOLD
    assert not no_stash.stash_has_cargo

    waiting = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False)
    )
    waiting.first_common_delivered = True
    waiting.stash_has_cargo = True
    start_search(waiting)
    waiting.step(
        VisionSnapshot(), pose(), stm(mode=STM_MODE_SEARCH, acknowledged_sequence=2), 0.1
    )
    output = waiting.step(
        VisionSnapshot(), pose(), stm(mode=20, flags=4, acknowledged_sequence=3), 2.1
    )
    assert output.state == CompetitionState.SEARCH
    assert output.reason == "stash_return_start_gate"

    mission = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False)
    )
    mission.first_common_delivered = True
    mission.stash_has_cargo = True
    start_search(mission)
    near_stash = PoseSnapshot(True, -0.85, 0.56, 135.0, 5.0)
    mission.step(
        VisionSnapshot(), near_stash,
        stm(mode=STM_MODE_SEARCH, acknowledged_sequence=10), 0.1
    )
    output = mission.step(
        VisionSnapshot(), near_stash, stm(mode=STM_MODE_SEARCH, acknowledged_sequence=10), 2.1
    )
    assert output.state == CompetitionState.RETURN_STASH
    assert output.command and output.command.opcode == CMD_NAVIGATE_WAYPOINT
    assert output.command.arg_a == 0
    assert output.command.arg_b == 0
    assert output.command.flags == 0x1F
    assert 26900 <= output.command.aux <= 27100

    # D=0 must remain the published request until the relay snapshot proves
    # that this payload was actually transmitted.
    zero = output.command
    output = mission.step(
        VisionSnapshot(),
        near_stash,
        stm(mode=STM_MODE_NAVIGATE, flags=1 << 5, acknowledged_sequence=11),
        2.2,
    )
    assert output.state == CompetitionState.RETURN_STASH
    assert output.command and output.command.arg_a == 0
    zero_frame = zero.to_frame(0)
    output = mission.step(
        VisionSnapshot(),
        near_stash,
        StmSnapshot(
            mode=STM_MODE_NAVIGATE,
            flags=1 << 5,
            age_ms=5.0,
            acknowledged_sequence=11,
            relay_mission_tx_frames=1,
            relay_last_mission_command=zero_frame[4],
            relay_last_mission_sequence=11,
            relay_last_mission_payload=tuple(zero_frame[4:12]),
        ),
        2.3,
    )
    assert output.state == CompetitionState.WAIT_STASH_SEARCH_HANDOFF
    assert output.command and output.command.opcode == CMD_HOLD

    hold_frame = output.command.to_frame(0)
    output = mission.step(
        VisionSnapshot(),
        near_stash,
        StmSnapshot(
            mode=STM_MODE_SEARCH,
            flags=0,
            age_ms=5.0,
            acknowledged_sequence=12,
            relay_mission_tx_frames=2,
            relay_last_mission_command=hold_frame[4],
            relay_last_mission_sequence=12,
            relay_last_mission_payload=tuple(hold_frame[4:12]),
        ),
        2.4,
    )
    assert output.state == CompetitionState.SEARCH
    assert output.event == "stash_search_handoff_complete"
    assert mission.stash_checked
    assert not mission.stash_has_cargo
    output = mission.step(
        VisionSnapshot(), near_stash, stm(mode=STM_MODE_SEARCH, acknowledged_sequence=13), 4.5
    )
    assert output.state == CompetitionState.SEARCH
    assert output.command and output.command.opcode == CMD_HOLD


def _delivery_vision(
    sequence: int,
    *,
    outside: bool = False,
    inside: bool = False,
    inside_ids: tuple[int, ...] = (),
    outside_ids: tuple[int, ...] = (),
) -> VisionSnapshot:
    return VisionSnapshot(
        frame_sequence=sequence,
        delivery_target_found=outside or inside,
        delivery_target_inside_safe_zone=inside,
        delivery_target_outside_safe_zone=outside,
        delivery_target_inside_track_ids=inside_ids,
        delivery_target_outside_track_ids=outside_ids,
    )


def _delivery_mission() -> CompetitionMission:
    mission = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False)
    )
    mission.first_common_delivered = True
    mission.selected_batch = CargoBatch((1,), ("green_supply",), "material")
    mission._set_state(CompetitionState.NAVIGATE, 0.0)
    return mission


def test_delivery_window_and_two_observation_timeout() -> None:
    mission = _delivery_mission()
    fence_pose = PoseSnapshot(True, -0.15, 1.0275, 90.0, 5.0)
    output = mission.step(
        _delivery_vision(1, outside=True, outside_ids=(1,)),
        fence_pose,
        stm(mode=STM_MODE_NAVIGATE, flags=0, acknowledged_sequence=1),
        0.0,
    )
    assert output.state == CompetitionState.ENTER_SAFE_ZONE
    assert mission.delivery_outside_seen

    output = mission.step(
        _delivery_vision(2, inside=True, inside_ids=(1,)),
        fence_pose,
        stm(mode=STM_MODE_RAM_VERIFY, acknowledged_sequence=2),
        0.1,
    )
    assert output.state == CompetitionState.DELIVERY_VERIFY
    assert mission.delivery_observation_attempt == 1

    pattern = (True, False, True, False, True, True, True)
    for sequence, inside in enumerate(pattern, 3):
        output = mission.step(
            _delivery_vision(
                sequence,
                inside=inside,
                inside_ids=(1,) if inside else (),
            ),
            fence_pose,
            stm(mode=STM_MODE_RAM_VERIFY, acknowledged_sequence=sequence),
            0.1 * (sequence - 1),
        )
    assert output.state == CompetitionState.TASK_COMPLETE
    assert mission.delivery_visual_confirmed
    assert mission.delivery_inside_hits >= 5
    assert len(mission.delivery_window) <= 7

    conflict = _delivery_mission()
    conflict._set_state(CompetitionState.DELIVERY_VERIFY, 0.0)
    conflict._start_delivery_observation(0.0, 1)
    output = conflict.step(
        _delivery_vision(1, outside=True, inside=True, inside_ids=(2,), outside_ids=(1,)),
        fence_pose,
        stm(mode=STM_MODE_RAM_VERIFY, acknowledged_sequence=1),
        0.1,
    )
    assert output.state == CompetitionState.DELIVERY_VERIFY
    assert conflict.delivery_inside_hits == 0
    assert conflict.delivery_conflict_frames == 1

    no_outside = _delivery_mission()
    no_outside._set_state(CompetitionState.DELIVERY_VERIFY, 0.0)
    no_outside._start_delivery_observation(0.0, 1)
    for sequence in range(1, 6):
        output = no_outside.step(
            _delivery_vision(sequence, inside=True, inside_ids=(1,)),
            fence_pose,
            stm(mode=STM_MODE_RAM_VERIFY, acknowledged_sequence=sequence),
            sequence * 0.1,
        )
    assert output.state == CompetitionState.DELIVERY_VERIFY
    assert no_outside.delivery_inside_hits == 0
    assert not no_outside.delivery_visual_confirmed
    assert no_outside.delivery_outside_seen is False

    # No new frame for more than one second invalidates the old inside window.
    expiry = _delivery_mission()
    expiry._set_state(CompetitionState.DELIVERY_VERIFY, 0.0)
    expiry._start_delivery_observation(0.0, 1)
    expiry.step(
        _delivery_vision(1, outside=True, outside_ids=(1,)),
        fence_pose,
        stm(mode=STM_MODE_RAM_VERIFY, acknowledged_sequence=1),
        0.1,
    )
    for sequence in range(2, 6):
        expiry.step(
            _delivery_vision(sequence, inside=True, inside_ids=(1,)),
            fence_pose,
            stm(mode=STM_MODE_RAM_VERIFY, acknowledged_sequence=sequence),
            0.1 + 0.1 * sequence,
        )
    expiry.step(
        _delivery_vision(6, inside=True, inside_ids=(1,)),
        fence_pose,
        stm(mode=STM_MODE_RAM_VERIFY, acknowledged_sequence=7),
        2.0,
    )
    assert expiry.delivery_inside_hits == 1
    assert len(expiry.delivery_window) == 1

    timeout = _delivery_mission()
    timeout._set_state(CompetitionState.DELIVERY_VERIFY, 0.0)
    timeout._start_delivery_observation(0.0, 1)
    output = timeout.step(
        VisionSnapshot(),
        fence_pose,
        stm(mode=STM_MODE_RAM_VERIFY, acknowledged_sequence=1),
        5.0,
    )
    assert output.state == CompetitionState.DELIVERY_VERIFY
    assert timeout.delivery_observation_attempt == 2
    assert timeout.delivery_timeout_reason == "first_observation_timeout"
    output = timeout.step(
        VisionSnapshot(),
        fence_pose,
        stm(mode=STM_MODE_RAM_VERIFY, acknowledged_sequence=2),
        10.0,
    )
    assert output.state == CompetitionState.FAULT
    assert output.command and output.command.opcode == CMD_ABORT
    assert timeout.delivery_timeout_reason == "second_observation_timeout"


def test_detour_completion_requires_fresh_changed_ack() -> None:
    mission = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False)
    )
    mission.first_common_delivered = True
    mission.selected_batch = CargoBatch((1,), ("green_supply",), "material")
    mission._set_state(CompetitionState.APPROACH, 0.0)
    danger = VisionSnapshot(frame_sequence=1, danger_ahead=True, danger_side="left")
    output = mission.step(
        danger,
        pose(),
        stm(mode=20, flags=4, acknowledged_sequence=10),
        0.1,
    )
    assert output.state == CompetitionState.DETOUR
    assert output.command and output.command.opcode == CMD_CHANGE_LANE

    output = mission.step(
        danger,
        pose(),
        stm(mode=36, acknowledged_sequence=10),
        0.2,
    )
    assert output.state == CompetitionState.DETOUR
    output = mission.step(
        danger,
        pose(),
        stm(mode=36, age_ms=300.0, acknowledged_sequence=11),
        0.3,
    )
    assert output.state == CompetitionState.DETOUR
    output = mission.step(
        danger,
        pose(),
        stm(mode=36, acknowledged_sequence=11),
        0.4,
    )
    assert output.state == CompetitionState.APPROACH


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
    assert output.state == CompetitionState.AUDIT_CONFIRM
    output = mission.step(
        replace(vision, frame_sequence=4), pose(), relay_stm(output), 0.5
    )
    assert output.state == CompetitionState.GRAB
    output = mission.step(vision, pose(), stm(mode=22, flags=3), 0.6)
    assert output.state == CompetitionState.INITIAL_STASH_NAV
    output = mission.step(vision, PoseSnapshot(True, -0.85, 0.55, 135.0, 5.0), stm(mode=10, flags=4, acknowledged_sequence=20), 1.0)
    assert output.state == CompetitionState.INITIAL_RELEASE
    output = mission.step(vision, PoseSnapshot(True, -0.85, 0.55, 135.0, 5.0), stm(mode=34, flags=0, acknowledged_sequence=21), 1.6)
    assert output.state == CompetitionState.RETURN_CENTER
    assert mission.initial_stash_done
    assert mission.stash_has_cargo

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
    assert output.state == CompetitionState.AUDIT_CONFIRM
    output = invalid_mission.step(
        replace(invalid, frame_sequence=5), pose(), relay_stm(output), 2.4
    )
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
    assert output.state == CompetitionState.AUDIT_CONFIRM
    return mission.step(
        VisionSnapshot(frame_sequence=4, capture_audit=audit),
        pose(),
        relay_stm(output),
        0.4,
    )


def test_stable_audit_is_invalidated_by_change_or_expiry() -> None:
    audit = CargoAudit(left_class="green_supply", left_count=1, total_count=1)
    mission = _audit_mission()
    mission.selected_batch = CargoBatch((1,), ("green_supply",), "material")
    output = None
    for sequence in (1, 2, 3):
        output = mission.step(
            VisionSnapshot(frame_sequence=sequence, capture_audit=audit),
            pose(),
            stm(flags=1),
            sequence * 0.1,
        )
    assert output is not None
    assert output.state == CompetitionState.AUDIT_CONFIRM
    assert output.command and output.command.opcode == CMD_CARGO_AUDIT
    assert output.command.audit is not None and output.command.audit.stable
    changed = replace(audit, left_class="core_black")
    output = mission.step(
        VisionSnapshot(frame_sequence=4, capture_audit=changed),
        pose(),
        stm(flags=1),
        0.4,
    )
    assert output.state == CompetitionState.CAPTURE_AUDIT
    assert output.command and output.command.opcode == CMD_CARGO_AUDIT

    expired = _audit_mission()
    expired.selected_batch = CargoBatch((1,), ("green_supply",), "material")
    for sequence in (1, 2, 3):
        output = expired.step(
            VisionSnapshot(
                frame_sequence=sequence,
                observed_monotonic_s=0.0,
                capture_audit=audit,
            ),
            pose(),
            stm(flags=1),
            sequence * 0.05,
        )
    assert output is not None and output.state == CompetitionState.AUDIT_CONFIRM
    output = expired.step(
        VisionSnapshot(
            frame_sequence=4,
            observed_monotonic_s=0.0,
            capture_audit=audit,
        ),
        pose(),
        stm(flags=1),
        1.0,
    )
    assert output.state == CompetitionState.CAPTURE_AUDIT
    assert output.command and output.command.opcode == CMD_HOLD


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
    assert output.command and output.command.opcode == CMD_DISPERSE_PILE
    assert output.tx_policy == "disperse_command"
    output = active.step(crowded, pose(), StmSnapshot(35, 0, 300.0, 0, 0), 1.0)
    assert output.state == CompetitionState.FAULT
    assert output.command and output.command.opcode == CMD_ABORT

    completed = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False)
    )
    start_search(completed)
    output = completed.step(crowded, pose(), stm(acknowledged_sequence=4), 0.1)
    assert output.state == CompetitionState.DISPERSE
    output = completed.step(
        VisionSnapshot(), pose(), stm(mode=3, acknowledged_sequence=4), 0.5
    )
    assert output.state == CompetitionState.DISPERSE
    assert output.command and output.command.opcode == CMD_DISPERSE_PILE
    output = completed.step(
        VisionSnapshot(), pose(), stm(mode=35, acknowledged_sequence=5), 1.1
    )
    assert output.state == CompetitionState.SEARCH

    stale_done = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False)
    )
    start_search(stale_done)
    output = stale_done.step(crowded, pose(), stm(acknowledged_sequence=8), 0.1)
    assert output.state == CompetitionState.DISPERSE
    output = stale_done.step(
        VisionSnapshot(), pose(), stm(mode=35, acknowledged_sequence=8), 0.2
    )
    assert output.state == CompetitionState.DISPERSE
    output = stale_done.step(
        VisionSnapshot(), pose(), stm(mode=35, age_ms=300.0, acknowledged_sequence=9), 0.3
    )
    assert output.state == CompetitionState.FAULT

    stale_start = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False)
    )
    start_search(stale_start)
    output = stale_start.step(
        replace(crowded, observed_monotonic_s=0.0),
        pose(),
        stm(mode=3),
        1.0,
    )
    assert output.state == CompetitionState.SEARCH
    assert output.command and output.command.opcode == CMD_HOLD


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

    output = mission.step(invalid, pose(), StmSnapshot(32, 0, 5.0, 0, 10), 0.4)
    assert output.state == CompetitionState.INVALID_BACKOFF
    assert output.command and output.command.opcode == CMD_YIELD_BACKOFF
    output = mission.step(invalid, pose(), StmSnapshot(30, 0, 5.0, 0, 11), 0.5)
    assert output.state == CompetitionState.CAPTURE_AUDIT
    assert output.command and output.command.opcode == CMD_HOLD
    assert mission.cargo_recheck_pending and mission.selected_batch is not None

    valid = CargoAudit(right_class="green_supply", right_count=1, total_count=1)
    for sequence in (2, 3, 4):
        output = mission.step(
            VisionSnapshot(frame_sequence=sequence, capture_audit=valid),
            pose(), stm(flags=1), 0.5 + sequence * 0.1,
        )
    assert output.state == CompetitionState.AUDIT_CONFIRM
    output = mission.step(
        VisionSnapshot(frame_sequence=5, capture_audit=valid),
        pose(), relay_stm(output), 0.9,
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
    final.step(invalid, pose(), StmSnapshot(32, 0, 5.0, 0, 20), 0.4)
    final.step(invalid, pose(), StmSnapshot(30, 0, 5.0, 0, 21), 0.5)
    for sequence in (2, 3, 4):
        output = final.step(
            VisionSnapshot(frame_sequence=sequence, capture_audit=invalid.capture_audit),
            pose(), stm(flags=1), 0.5 + sequence * 0.1,
        )
    assert output.state == CompetitionState.AUDIT_CONFIRM
    output = final.step(
        VisionSnapshot(frame_sequence=5, capture_audit=invalid.capture_audit),
        pose(), relay_stm(output), 0.9,
    )
    assert output.state == CompetitionState.INVALID_RELEASE
    assert output.command and output.command.opcode == CMD_RELEASE_BOTH
    output = final.step(invalid, pose(), StmSnapshot(34, 0, 5.0, 0, 30), 1.2)
    assert output.state == CompetitionState.INVALID_BACKOFF
    output = final.step(invalid, pose(), StmSnapshot(30, 0, 5.0, 0, 31), 1.3)
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

    for sequence in range(2, 8):
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
    test_search_recovery_handshake_and_safety_gates()
    test_return_stash_requires_confirmed_cargo_and_search_handoff()
    test_delivery_window_and_two_observation_timeout()
    test_detour_completion_requires_fresh_changed_ack()
    test_initial_stash_and_invalid_release()
    test_material_priority_excludes_danger()
    test_disperse_requires_fresh_green_and_respects_limit()
    test_stable_audit_is_invalidated_by_change_or_expiry()
    test_release_side_selection()
    test_single_release_recheck_and_final_both_release()
    test_return_center_sends_zero_until_search()
    test_delivery_requires_outside_to_inside_transition()
    test_boundary_guard_points_back_into_field()
    print("competition state machine PASS")


if __name__ == "__main__":
    main()
