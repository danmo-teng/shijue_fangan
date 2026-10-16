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
    CMD_DISPERSE_PILE,
    CMD_GRAB_CONFIRMED,
    CMD_HOLD,
    CMD_NAVIGATE_WAYPOINT,
    CMD_RELEASE_BOTH,
    CMD_RELEASE_LEFT,
    CMD_RELEASE_RIGHT,
    CMD_RETURN_CENTER,
    CMD_TARGET_RIGHT,
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


def test_first_green_flow_and_navigation_does_not_use_t265_stuck_actions() -> None:
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

    # Stationary XY during F407 heading alignment or final-distance handling
    # never starts an upper-computer YIELD/ESCAPE sequence.
    for now in (0.6, 2.2, 6.0):
        output = mission.step(
            vision,
            pose(),
            stm(mode=10, flags=0, acknowledged_sequence=10),
            now,
        )
        assert output.state == CompetitionState.NAVIGATE
        assert output.command and output.command.opcode == CMD_NAVIGATE_WAYPOINT
    output = mission.step(
        vision,
        PoseSnapshot(True, -0.15, 0.95, 90.0, 5.0),
        stm(mode=10, flags=0, acknowledged_sequence=11),
        7.0,
    )
    assert output.state == CompetitionState.NAVIGATE
    assert output.command and 0 < output.command.arg_a <= 100

    no_observation = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False)
    )
    start_search(no_observation)
    no_observation._select_batch(CargoBatch((1,), ("green_supply",), "material"))
    no_observation._set_state(CompetitionState.CAPTURE_AUDIT, 3.0)
    output = no_observation.step(VisionSnapshot(), pose(), stm(flags=1), 3.1)
    for now in (3.2, 3.3, 3.4):
        output = no_observation.step(VisionSnapshot(), pose(), stm(flags=1), now)
    assert output.state == CompetitionState.CAPTURE_AUDIT
    assert output.command and output.command.opcode == CMD_CARGO_AUDIT


def test_search_recovery_follows_f407_modes_without_local_timeout() -> None:
    mission = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False)
    )
    start_search(mission)
    target = cargo(7, "green_supply", 600, 520, relative=(0.2, 0.7))
    visible = VisionSnapshot(
        frame_sequence=1, observed_monotonic_s=0.0, cargo=(target,)
    )
    output = mission.step(visible, pose(), stm(mode=3), 0.0)
    assert output.state == CompetitionState.APPROACH

    # After the dynamic missing-frame window, explicit HOLD starts the F407
    # 500 ms recovery timer without clearing the target early.
    output = mission.step(
        VisionSnapshot(frame_sequence=2, observed_monotonic_s=0.4),
        pose(),
        stm(mode=20, flags=0),
        0.4,
    )
    assert output.state == CompetitionState.APPROACH
    assert output.command and output.command.opcode == CMD_HOLD
    assert mission.selected_batch is not None

    output = mission.step(
        VisionSnapshot(frame_sequence=3, observed_monotonic_s=0.9),
        pose(),
        stm(mode=24),
        0.9,
    )
    assert output.state == CompetitionState.WAIT_SEARCH_RECOVERY
    assert output.command and output.command.opcode == CMD_HOLD
    assert mission.selected_batch is None
    assert mission.locked_target_track_id is None
    assert mission.target_last_center_px is None

    # No local recovery timer may turn a long mode24 wait into ABORT.
    output = mission.step(
        VisionSnapshot(frame_sequence=4, observed_monotonic_s=20.0),
        PoseSnapshot(False, age_ms=999.0),
        stm(mode=24),
        20.0,
    )
    assert output.state == CompetitionState.WAIT_SEARCH_RECOVERY
    assert output.command and output.command.opcode == CMD_HOLD

    # Fresh mode3 alone completes the handoff; visual and pose freshness are
    # intentionally irrelevant here.
    output = mission.step(
        VisionSnapshot(frame_sequence=5, observed_monotonic_s=0.0),
        PoseSnapshot(False, age_ms=999.0),
        stm(mode=3),
        21.0,
    )
    assert output.state == CompetitionState.SEARCH
    assert output.command and output.command.opcode == CMD_HOLD
    assert mission.search_epoch_frame_floor == 5


def test_target_reassociation_and_only_real_abort_sources() -> None:
    mission = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False)
    )
    start_search(mission)
    target = cargo(7, "green_supply", 600, 520, relative=(0.2, 0.7))
    mission.step(VisionSnapshot(frame_sequence=1, cargo=(target,)), pose(), stm(), 0.0)
    rebuilt = cargo(19, "green_supply", 608, 524, relative=(0.21, 0.69))
    output = mission.step(
        VisionSnapshot(frame_sequence=2, cargo=(rebuilt,)),
        pose(),
        stm(mode=20, flags=0),
        0.1,
    )
    assert output.state == CompetitionState.APPROACH
    assert output.command and output.command.opcode == CMD_APPROACH_TARGET
    assert mission.locked_target_track_id == 19

    faulted = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False)
    )
    output = faulted.step(
        VisionSnapshot(), pose(), StmSnapshot(mode=20, age_ms=5.0, fault_code=6), 0.1
    )
    assert output.state == CompetitionState.FAULT
    assert output.command and output.command.opcode == CMD_ABORT

    lost = CompetitionMission(
        CompetitionSettings(
            side="red", start_zone=1, initial_stash_enabled=False, stm_loss_abort_s=2.0
        )
    )
    output = lost.step(VisionSnapshot(), pose(), stm(age_ms=300.0), 1.0)
    assert output.command is None and output.suppress_command_tx
    output = lost.step(VisionSnapshot(), pose(), stm(age_ms=300.0), 3.1)
    assert output.state == CompetitionState.FAULT
    assert output.command and output.command.opcode == CMD_ABORT

    outside = CompetitionMission(
        CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=False)
    )
    outside.first_common_delivered = True
    outside._select_batch(CargoBatch((1,), ("green_supply",), "material"))
    outside._set_state(CompetitionState.NAVIGATE, 0.0)
    output = outside.step(
        VisionSnapshot(), PoseSnapshot(True, 1.49, 0.0, 0.0, 5.0), stm(mode=10), 0.1
    )
    assert output.state == CompetitionState.FAULT
    assert output.command and output.command.opcode == CMD_ABORT


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
    assert output.state == CompetitionState.DELIVERY_VERIFY
    assert output.command and output.command.opcode != CMD_ABORT
    assert timeout.delivery_timeout_reason == "observation_wait_extended"


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
        20.2,
    )
    assert output.state == CompetitionState.DETOUR
    assert output.command and output.command.opcode == CMD_CHANGE_LANE
    output = mission.step(
        danger,
        pose(),
        stm(mode=36, age_ms=300.0, acknowledged_sequence=11),
        20.3,
    )
    assert output.state == CompetitionState.DETOUR
    output = mission.step(
        danger,
        pose(),
        stm(mode=36, acknowledged_sequence=11),
        20.4,
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
    assert output.state == CompetitionState.DISPERSE
    assert output.command is None and output.suppress_command_tx

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
    assert output.state == CompetitionState.DISPERSE
    assert output.command is None and output.suppress_command_tx

    output = stale_done.step(
        VisionSnapshot(), pose(), stm(mode=25, acknowledged_sequence=8), 30.0
    )
    assert output.state == CompetitionState.DISPERSE
    assert output.command and output.command.opcode == CMD_DISPERSE_PILE

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
    initial = StmSnapshot(mode=17, age_ms=0, context_valid=True,
                          context_matches=True, task_id=1, action_id=2,
                          accepted_command=CMD_RETURN_CENTER, action_status=1)
    mission._begin_return(initial, 0.0)
    output = mission.step(VisionSnapshot(), pose(0.60, 0.0), initial, 0.1)
    assert output.command and output.command.arg_a == 600
    output = mission.step(VisionSnapshot(), pose(0.08, 0.0), initial, 0.2)
    assert output.command and output.command.arg_a == 0
    # Jitter across the center and outside the arrival radius cannot restart RETURN.
    output = mission.step(VisionSnapshot(), pose(-0.12, 0.0), initial, 0.3)
    assert output.command and output.command.arg_a == 0
    assert output.command.aux == 13500
    output = mission.step(VisionSnapshot(), pose(), replace(initial, mode=3, age_ms=1000), 0.4)
    assert mission.state == CompetitionState.RETURN_CENTER
    output = mission.step(VisionSnapshot(), pose(), replace(initial, mode=3), 0.5)
    assert output.state == CompetitionState.SEARCH
    assert output.command.opcode == CMD_HOLD
    mission._begin_return(initial, 0.6)
    assert not mission.return_arrival_latched


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


def test_4071_separation_evidence_and_fault_latch() -> None:
    mission = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    mission.first_common_delivered = True
    mission._select_batch(CargoBatch((1,), ("green_supply",), "material"))
    mixed = CargoAudit(left_class="mixed_material", left_count=2, left_green_count=1,
                       total_count=2, class_counts=(("green_supply", 1), ("core_black", 1)))
    assert mission._side_is_disperse_candidate(mixed, "left")
    assert not mission._audit_valid(mixed)
    assert not mission._side_is_disperse_candidate(replace(mixed, class_counts=()), "left")
    assert not mission._side_is_disperse_candidate(replace(mixed, unknown_present=True), "left")
    assert not mission._side_is_disperse_candidate(replace(mixed, danger_present=True), "left")
    vote_mission = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    vote_mission._select_batch(CargoBatch((1,), ("green_supply",), "material"))
    greens = CargoAudit(left_class="green_supply", right_class="green_supply",
                        left_count=2, right_count=1, left_green_count=2, right_green_count=1, total_count=3)
    assert vote_mission._preferred_keep_side(greens) == "left"
    vote_mission.separation_keep_side = "right"
    tied = replace(greens, left_count=1, left_green_count=1, total_count=2)
    assert vote_mission._preferred_keep_side(tied) == "right"
    assert not vote_mission._should_first_green_bump(greens)
    vote_mission._observe_disperse_side_vote(1, greens, 1.0)
    vote_mission._observe_disperse_side_vote(2, greens, 1.09)
    assert not vote_mission._disperse_side_vote_result()[2]
    vote_mission._observe_disperse_side_vote(3, greens, 1.1)
    assert vote_mission._disperse_side_vote_result()[2]
    changed = replace(greens, right_class="core_black", right_green_count=0)
    vote_mission._observe_disperse_side_vote(4, changed, 1.2)
    assert not vote_mission._disperse_side_vote_result()[2]
    vote_mission.cluster_keep_side = "left"
    vote_mission.cluster_keep_side_count = 0
    curved = vote_mission._start_disperse(greens, stm(mode=37), 1.3)
    from protocol import CMD_SIDE_VALID, CMD_FIRST_GREEN_BUMP
    assert curved.command.flags & CMD_SIDE_VALID and not curved.command.flags & CMD_FIRST_GREEN_BUMP
    assert vote_mission.cluster_keep_side_count == 2
    mission._observe_disperse_side_vote(1, mixed, 1.0)
    mission._observe_disperse_side_vote(1, mixed, 1.2)
    mission._observe_disperse_side_vote(2, mixed, 1.05)
    assert not mission._disperse_side_vote_result()[2]
    mission._observe_disperse_side_vote(3, mixed, 1.1)
    assert mission._disperse_side_vote_result() == ("left", 2, True)
    swapped = replace(mixed, left_class="", left_count=0, left_green_count=0,
                      right_class="mixed_material", right_count=2, right_green_count=1)
    assert mixed.signature == swapped.signature
    mission._observe_disperse_side_vote(5, swapped, 1.21)
    assert not mission._disperse_side_vote_result()[2]
    mission.cluster_keep_side = "left"
    mission.cluster_keep_side_count = 2
    output = mission._start_disperse(mixed, stm(mode=37), 2.0)
    assert output.command.opcode == CMD_DISPERSE_PILE and mission.disperse_attempts == 0
    mission._count_separation_acceptance("left")
    mission._count_separation_acceptance("left")
    assert mission.disperse_attempts == 1
    mission.cluster_keep_side = "right"
    mission.cluster_keep_side_count = 2
    output = mission._start_disperse(swapped, stm(mode=37), 2.1)
    assert output.command.flags & CMD_TARGET_RIGHT
    mission._count_separation_acceptance("right")
    mission._count_separation_acceptance("right")
    assert mission.disperse_attempts == 2 and mission.separation_side_attempts == 1
    output = mission._start_disperse(swapped, stm(mode=37), 2.2)
    assert output.command.opcode == CMD_RELEASE_BOTH
    assert mission.disperse_attempts == 2
    legal = CargoAudit(left_class="green_supply", left_count=1, left_green_count=1, total_count=1)
    assert mission._audit_valid(legal)  # A legal result after the second attempt remains eligible for transport.
    same_pile = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    same_pile.disperse_attempts = 2
    same_pile.separation_keep_side = "left"
    same_pile.separation_side_attempts = 2
    target = cargo(1, "green_supply", 400, 450)
    same_pile._choose_cluster_target = lambda vision: target
    same_pile._same_cluster_target = lambda item: True
    same_pile._cluster_approach_command = lambda vision: same_pile._hold()
    same_pile._start_cluster_approach(VisionSnapshot(), stm(), 2.5)
    assert same_pile.task_generation == 0 and same_pile.disperse_attempts == 2
    assert same_pile.separation_keep_side == "left" and same_pile.separation_side_attempts == 2
    mission.state = CompetitionState.SEARCH
    output = mission.step(VisionSnapshot(), pose(), replace(stm(), fault_code=5), 3.0)
    assert output.suppress_command_tx and output.command is None
    assert "锁存" in output.message and mission.selected_batch is not None
    for status in (stm(), stm(age_ms=1000)):
        output = mission.step(VisionSnapshot(), pose(), status, 4.0)
        assert output.suppress_command_tx and output.command is None


def test_4071_injury_pending_and_missing_audit_exit() -> None:
    mission = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    mission.first_common_delivered = True
    mission._select_batch(CargoBatch((1,), ("injured_orange",), "injury"))
    vision = VisionSnapshot(frame_sequence=10)
    output = mission._post_grab_half_close_output(vision, stm(mode=49), 1.0)
    assert output.command.audit_vision_frame == 0
    assert not output.command.audit.stable and output.command.audit.destination_injury
    assert mission.pending_audit is None and mission.audit_recheck_frame_floor == 10
    audit = CargoAudit(left_class="injured_orange", left_count=1, total_count=1, stable=True)
    mission.audit_last_frame_sequence = 20
    mission._begin_audit_confirmation(audit, mission._audit_payload(audit), True, "both", False,
                                      "post_grab_valid", stm(mode=23), 2.0)
    pending = mission.pending_audit
    output = mission._audit_confirmation_output(VisionSnapshot(frame_sequence=21), stm(mode=50), 2.2)
    assert mission.pending_audit is pending and output.command.opcode == CMD_CARGO_AUDIT
    assert output.command.audit_vision_frame == 20
    for mode in (22, 40, 50):
        missing = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
        missing.state = CompetitionState.POST_GRAB_AUDIT
        first = missing.step(VisionSnapshot(), pose(), stm(mode=mode), 1.0)
        assert first.reason == "missing_committed_audit"
        timeout = missing.step(VisionSnapshot(), pose(), stm(mode=mode), 18.1)
        assert timeout.command.opcode == CMD_ABORT


def test_mode51_reference_audit_exits_and_budget() -> None:
    from protocol import CMD_PAUSE
    def setup():
        m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
        m.first_common_delivered = True
        m._select_batch(CargoBatch((1,), ("green_supply",), "material"))
        m.state = CompetitionState.NAVIGATE
        m.disperse_attempts = 2
        m.delivery_count = 4
        m.stash_has_cargo = True
        m.stm_fault_waiting = True
        return m
    def status(flags=0, command=CMD_CARGO_AUDIT, mode=51):
        return StmSnapshot(mode=mode, flags=flags, age_ms=0, camera_pitch_cdeg=14000,
                           context_valid=True, context_matches=True, task_id=10, action_id=20,
                           accepted_command=command, action_status=1)
    def observe(audit, frame, now):
        return VisionSnapshot(capture_audit=audit, frame_sequence=frame, observed_monotonic_s=now,
                              capture_observation_valid=True)
    def stable_audit(m, audit):
        first = m.step(observe(audit, 1, 1.0), pose(), status(), 1.0)
        assert first.command.opcode == CMD_HOLD and not m.stm_fault_waiting
        assert m.delivery_count == 4 and m.stash_has_cargo and m.disperse_attempts == 2
        paused = m._recovery_wait_output(observe(audit, 2, 1.1), pose(), status(1), 1.1, manual_paused=True)
        assert paused.command.opcode == CMD_PAUSE
        m.step(observe(audit, 3, 1.2), pose(), status(1), 1.2)
        for frame, now in [(4, 1.8), (5, 1.91)]:
            out = m.step(observe(audit, frame, now), pose(), status(1), now)
        assert out.command.opcode == CMD_CARGO_AUDIT and out.command.audit.stable
        assert not status(1).audit_ready  # STABLE was sent before READY.
        duplicate = m.step(observe(audit, 5, 1.91), pose(), status(1), 2.03)
        assert m.audit_hits == 2 and duplicate.command.audit.stable
        return m
    single = CargoAudit(left_class="green_supply", left_count=1, left_green_count=1, total_count=1)
    legal = stable_audit(setup(), single)
    out = legal.step(observe(single, 6, 2.02), pose(), status(1 | 64 | 16), 2.04)
    assert out.command.opcode == CMD_GRAB_CONFIRMED and legal.disperse_attempts == 2
    legal.step(observe(single, 7, 2.1), pose(), status(3, CMD_GRAB_CONFIRMED, 23), 2.1)
    assert not legal.recovery_wait_active and legal.post_grab_audit_active
    assert legal.delivery_count == 4
    empty = stable_audit(setup(), CargoAudit())
    out = empty.step(observe(CargoAudit(), 6, 2.02), pose(), status(1 | 64), 2.04)
    assert out.command.opcode == CMD_RETURN_CENTER and out.command.arg_a > 0
    empty.deferred_cluster_class = "green_supply"
    out = empty.step(observe(CargoAudit(), 7, 2.2), pose(), status(0, CMD_RETURN_CENTER, 17), 2.2)
    assert empty.disperse_attempts == 2
    out = empty.step(observe(CargoAudit(), 8, 2.3), pose(), status(0, CMD_RETURN_CENTER, 3), 2.3)
    assert empty.disperse_attempts == 0 and not empty.recovery_task_locked
    assert empty.deferred_cluster_class == "green_supply" and empty.delivery_count == 4
    invalid_audit = CargoAudit(left_class="green_supply", left_count=2, left_green_count=2, total_count=2)
    invalid = stable_audit(setup(), invalid_audit)
    out = invalid.step(observe(invalid_audit, 6, 2.02), pose(), status(1 | 64), 2.04)
    assert out.command.opcode == CMD_CARGO_AUDIT and invalid.state == CompetitionState.RECOVERY_WAIT
    absent = invalid.step(observe(None, 7, 40.0), pose(), status(1), 40.0)
    assert absent.command.opcode == CMD_CARGO_AUDIT and absent.command.audit_vision_frame == 0
    assert invalid.state != CompetitionState.FAULT and invalid.disperse_attempts == 2
    for frame, now in [(8, 40.1), (9, 40.21), (10, 40.32)]:
        invalid.step(observe(invalid_audit, frame, now), pose(), status(1), now)
    invalid.recovery_abandon_requested = True
    out = invalid.step(observe(invalid_audit, 10, 40.32), pose(), status(1 | 64), 40.33)
    assert out.command.opcode == CMD_RELEASE_BOTH and invalid.delivery_count == 4


def main() -> None:
    test_mode51_reference_audit_exits_and_budget()
    test_4071_separation_evidence_and_fault_latch()
    test_4071_injury_pending_and_missing_audit_exit()
    test_start_handshake_suppresses_task_frames_until_start_clear()
    test_first_green_flow_and_navigation_does_not_use_t265_stuck_actions()
    test_search_recovery_follows_f407_modes_without_local_timeout()
    test_target_reassociation_and_only_real_abort_sources()
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
