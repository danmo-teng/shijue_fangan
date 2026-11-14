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


def test_initial_stash_preserves_existing_nonempty_audit_semantics() -> None:
    mission = CompetitionMission(CompetitionSettings(side="red", start_zone=1, initial_stash_enabled=True))
    mission._select_batch(CargoBatch((), (), "stash", initial_stash=True))
    # Opening relocation retains its original nonempty-pile rule. Formal
    # delivery exclusions must not be accidentally imposed on this probe.
    for audit in (
        CargoAudit(left_class="mixed_material", left_count=2, total_count=2),
        CargoAudit(left_class="green_supply", left_count=1,
                   right_class="danger_cyan", right_count=1, total_count=2,
                   danger_present=True),
    ):
        assert mission._audit_valid(audit)
    assert not mission._audit_valid(CargoAudit())


def test_opening_checks_real_claw_view_before_tracking_pile() -> None:
    m = CompetitionMission(CompetitionSettings(side="blue"))
    p = PoseSnapshot(True, 0.0, 0.0, 135.0, 5.0)
    pile = (cargo(7, "green_supply", 900, 700),)
    old_view = VisionSnapshot(frame_sequence=10, cargo=pile, capture_cargo=pile,
        capture_audit=CargoAudit(left_class="green_supply", left_count=1, total_count=1))
    m.step(old_view, p, StmSnapshot(mode=3, camera_pitch_cdeg=10500, age_ms=5), 1.0)
    out = m.step(old_view, p, StmSnapshot(mode=3, camera_pitch_cdeg=10500, age_ms=5), 1.02)
    assert out.state == CompetitionState.INITIAL_CLAW_CHECK
    assert out.command.opcode == CMD_CARGO_AUDIT
    assert out.command.audit.initial_stash and not out.command.audit.stable
    assert out.command.audit.total_count == 0 and out.command.audit_vision_frame == 0
    assert m.selected_batch.total_count == 0  # The request is not a load observation.
    # The moving camera and its old picture cannot authorize GRAB.
    out = m.step(old_view, p, StmSnapshot(mode=20, camera_pitch_cdeg=14000, age_ms=5), 1.4)
    assert out.command.audit_vision_frame == 0 and not out.command.audit.stable
    ready = StmSnapshot(mode=21, flags=1, camera_pitch_cdeg=14000, age_ms=5)
    out = m.step(old_view, p, ready, 1.7)
    assert out.command.opcode == CMD_CARGO_AUDIT and not out.command.audit.stable
    for now, frame in ((1.8, 11), (1.91, 12)):
        out = m.step(replace(old_view, frame_sequence=frame, observed_monotonic_s=now), p, ready, now)
    assert out.state == CompetitionState.AUDIT_CONFIRM
    assert out.command.audit.stable and out.command.audit.total_count == 1
    accepted = replace(relay_stm(out), camera_pitch_cdeg=14000, flags=81)
    out = m.step(replace(old_view, frame_sequence=12, observed_monotonic_s=1.91), p, accepted, 1.92)
    assert out.state == CompetitionState.GRAB and out.command.opcode == CMD_GRAB_CONFIRMED
    assert m.selected_batch.track_ids == (7,)
    assert m.selected_batch.classes == ("green_supply",) and m.selected_batch.total_count == 1
    # Continue the original physical close -> real post-close audit -> NAV.
    closed_view = replace(old_view, frame_sequence=13, observed_monotonic_s=2.0)
    post = StmSnapshot(mode=23, flags=3, camera_pitch_cdeg=14000, age_ms=5)
    out = m.step(closed_view, p, post, 2.0)
    assert out.state == CompetitionState.POST_GRAB_AUDIT
    for now, frame in ((2.12, 14), (2.24, 15)):
        closed_view = replace(old_view, frame_sequence=frame, observed_monotonic_s=now)
        out = m.step(closed_view, p, post, now)
    done = replace(relay_stm(out, mode=22, flags=83, acknowledged_sequence=3,
                            relay_mission_tx_frames=3), camera_pitch_cdeg=14000,
                   context_valid=True, context_matches=True,
                   accepted_command=CMD_CARGO_AUDIT, action_status=1)
    out = m.step(closed_view, p, done, 2.25)
    assert out.reason == "post_grab_audit_confirmed"
    out = m.step(closed_view, p, done, 2.26)
    assert out.state == CompetitionState.INITIAL_STASH_NAV
    assert out.command.opcode == CMD_NAVIGATE_WAYPOINT and m.carried_total_count == 1


def test_opening_empty_probe_returns_to_approach_without_repeating_probe() -> None:
    m = CompetitionMission(CompetitionSettings(side="blue"))
    p = PoseSnapshot(True, 0.0, 0.0, 135.0, 5.0)
    search = StmSnapshot(mode=3, camera_pitch_cdeg=10500, age_ms=5)
    m.step(VisionSnapshot(), p, search, 1.0)
    request = m.step(VisionSnapshot(frame_sequence=10), p, search, 1.02)
    assert request.state == CompetitionState.INITIAL_CLAW_CHECK
    m.step(VisionSnapshot(frame_sequence=11), p,
        StmSnapshot(mode=20, camera_pitch_cdeg=13000, age_ms=5), 1.5)
    ready = StmSnapshot(mode=21, flags=1, camera_pitch_cdeg=14000, age_ms=5)
    m.step(VisionSnapshot(frame_sequence=12), p, ready, 1.8)
    for frame, now in ((13, 1.9), (14, 2.01)):
        out = m.step(VisionSnapshot(frame_sequence=frame, observed_monotonic_s=now,
            capture_audit=CargoAudit()), p, ready, now)
    assert out.command.audit.stable and out.command.audit.total_count == 0
    assert out.command.opcode == CMD_CARGO_AUDIT  # No blind GRAB or release.
    out = m.step(VisionSnapshot(frame_sequence=15), p, search, 2.04)
    assert out.state == CompetitionState.INITIAL_OBSERVE
    target = cargo(8, "green_supply", 600, 500)
    view = VisionSnapshot(frame_sequence=16, observed_monotonic_s=2.1, cargo=(target,))
    out = m.step(view, p, search, 2.1)
    assert out.state == CompetitionState.INITIAL_APPROACH
    assert out.command.opcode == CMD_APPROACH_TARGET
    # A later target-loss recovery must not restart the one-off empty probe.
    m.step(replace(view, frame_sequence=17), p,
        StmSnapshot(mode=24, age_ms=5), 2.2)
    m.step(replace(view, frame_sequence=18), p, search, 2.8)
    out = m.step(replace(view, frame_sequence=19, observed_monotonic_s=2.9), p, search, 2.9)
    assert out.state == CompetitionState.INITIAL_APPROACH
    assert out.command.opcode == CMD_APPROACH_TARGET


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
    assert mission.state == CompetitionState.SEARCH  # Actual completion is not rejected by age.
    output = mission.step(VisionSnapshot(), pose(), replace(initial, mode=3), 0.5)
    assert output.state == CompetitionState.SEARCH
    assert output.command.opcode == CMD_HOLD
    mission._begin_return(initial, 0.6)
    assert not mission.return_arrival_latched


def test_post_delivery_returns_to_own_side_with_loose_latched_arrival() -> None:
    for side, sign, heading in (("red", 1.0, 27000), ("blue", -1.0, 9000)):
        for exit_mode in (16, 17):
            mission = CompetitionMission(CompetitionSettings(side=side))
            mission.state = CompetitionState.TASK_COMPLETE
            mission.selected_batch = CargoBatch((1,), ("green_supply",), "material")
            initial = StmSnapshot(mode=exit_mode, age_ms=0, context_valid=True,
                                  context_matches=True, task_id=1, action_id=2,
                                  accepted_command=CMD_RETURN_CENTER, action_status=1)
            output = mission.step(VisionSnapshot(), pose(0.0, sign * 0.8), initial, 0.1)
            assert output.state == CompetitionState.RETURN_CENTER
            assert mission.return_after_delivery and mission.selected_batch is None
            assert output.command.opcode == CMD_RETURN_CENTER
            assert output.command.arg_a == 500 and output.command.aux == heading
            assert mission.settings.post_delivery_return_point == (0.0, sign * 0.30)

            # The field centre is not the new arrival point. A small diagonal
            # error outside 15 cm still moves, rather than checking each axis.
            output = mission.step(VisionSnapshot(), pose(0.0, 0.0), initial, 0.2)
            assert output.command.arg_a == 300
            assert output.command.aux == (9000 if side == "red" else 27000)
            output = mission.step(VisionSnapshot(), pose(0.12, sign * 0.40), initial, 0.3)
            assert output.command.arg_a == 156
            output = mission.step(VisionSnapshot(), pose(0.12, sign * 0.38), initial, 0.4)
            assert output.command.arg_a == 0
            assert output.state == CompetitionState.RETURN_CENTER

            # Once close enough, pose jitter cannot restart fine correction;
            # retain the original return acceptance and SEARCH handoff.
            output = mission.step(VisionSnapshot(), pose(0.18, sign * 0.50), initial, 0.5)
            assert output.command.arg_a == 0
            output = mission.step(VisionSnapshot(), pose(), replace(initial, mode=3), 0.6)
            assert output.state == CompetitionState.SEARCH
            assert output.command.opcode == CMD_HOLD
            assert output.event == "return_search_gate_start"

            # A later abandoned capture/search return retains its own centre
            # target and original 8 cm tolerance; it cannot inherit delivery.
            mission._begin_return(initial, 0.7)
            assert not mission.return_after_delivery and not mission.return_arrival_latched
            output = mission.step(VisionSnapshot(), pose(0.0, sign * 0.30), initial, 0.8)
            assert output.command.arg_a == 300 and output.command.aux == heading
            output = mission.step(VisionSnapshot(), pose(0.0, sign * 0.12), initial, 0.9)
            assert output.command.arg_a == 120


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
    for mode in (21, 38):
        initial = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
        initial._select_batch(CargoBatch((1,), ("green_supply",), "material"))
        initial.state = CompetitionState.CAPTURE_AUDIT
        status = replace(stm(mode=mode, flags=1), camera_pitch_cdeg=14000)
        for frame, timestamp in ((1, 1.0), (2, 1.11)):
            view = VisionSnapshot(frame_sequence=frame, observed_monotonic_s=timestamp,
                                  capture_audit=CargoAudit(), capture_observation_valid=True)
            out = initial._audit_output(view, status, timestamp)
        assert out.reason == "capture_empty_approach" and out.command.audit.stable
        assert initial.pending_audit is None and initial.state == CompetitionState.CAPTURE_AUDIT
        duplicate = initial._audit_output(view, status, 1.12)
        assert duplicate.command.audit.stable and initial.audit_hits == 2
    ordinary = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    ordinary._select_batch(CargoBatch((1,), ("green_supply",), "material"))
    ordinary.state = CompetitionState.CAPTURE_AUDIT
    mixed_observation = CargoAudit(left_class="unknown", left_count=2, left_green_count=1,
        left_invalid=True, injury_mixed=True, total_count=2,
        class_counts=(("green_supply", 1), ("injured_orange", 1)),
        left_class_counts=(("green_supply", 1), ("injured_orange", 1)))
    status = replace(stm(mode=21, flags=1), camera_pitch_cdeg=14000)
    for frame, timestamp in ((1, 1.0), (2, 1.11)):
        view = VisionSnapshot(frame_sequence=frame, observed_monotonic_s=timestamp,
                              capture_audit=mixed_observation, capture_observation_valid=True)
        out = ordinary._audit_output(view, status, timestamp)
    assert ordinary.pending_audit_release_context == "disperse_reaudit"
    assert not ordinary.cluster_audit_active and out.command.audit.stable
    acknowledged = replace(relay_stm(out, mode=21, flags=65), camera_pitch_cdeg=14000,
                           context_valid=True, context_matches=True,
                           accepted_command=CMD_CARGO_AUDIT, action_status=1)
    out = ordinary._audit_confirmation_output(view, acknowledged, 1.12)
    from protocol import CMD_SIDE_VALID
    assert out.command.opcode == CMD_DISPERSE_PILE and out.command.flags & CMD_SIDE_VALID
    mission = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    mission.first_common_delivered = True
    mission._select_batch(CargoBatch((1,), ("green_supply",), "material"))
    mixed = CargoAudit(left_class="mixed_material", left_count=2, left_green_count=1,
                       total_count=2, class_counts=(("green_supply", 1), ("core_black", 1)))
    assert mission._side_is_disperse_candidate(mixed, "left")
    assert not mission._audit_valid(mixed)
    assert not mission._side_is_disperse_candidate(replace(mixed, class_counts=()), "left")
    assert not mission._side_is_disperse_candidate(replace(mixed, unknown_present=True), "left")
    assert mission._side_is_disperse_candidate(replace(mixed, danger_present=True), "left")
    assert not mission._audit_valid(replace(mixed,danger_present=True))
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
    invalid_audit = CargoAudit(left_class="green_supply", left_count=3, left_green_count=3, total_count=3)
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


def test_cluster_approach_tracks_anchor_when_neighbours_change() -> None:
    from protocol import CMD_CLUSTER_TARGET

    for first_delivered in (False, True):
        m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
        m.first_common_delivered = first_delivered
        anchor = TrackedCargo(101, "green_supply", .9, (540, 360, 200, 200), hits=5)
        left = TrackedCargo(102, "green_supply", .8, (360, 380, 160, 160), hits=5)
        right = TrackedCargo(103, "core_black", .8, (760, 380, 160, 160), hits=5)
        first = VisionSnapshot(frame_sequence=10, cargo=(anchor, left))
        out = m._start_cluster_approach(first, stm(), 1.0)
        assert out.command.opcode == CMD_APPROACH_TARGET
        assert out.command.flags & CMD_CLUSTER_TARGET
        assert (out.command.arg_a, out.command.arg_b) == (640, 460)
        assert m.cluster_target_track_id == 101
        left_extent = m.cluster_bbox

        status = replace(stm(mode=20), camera_pitch_cdeg=13000)
        for frame, items in ((11, (anchor, right)), (12, (anchor, left)), (13, (anchor,))):
            out = m.step(VisionSnapshot(frame_sequence=frame, cargo=items), pose(), status, 1.0 + frame / 100.)
            assert out.state == CompetitionState.CLUSTER_APPROACH
            assert out.command.flags & CMD_CLUSTER_TARGET
            assert (out.command.arg_a, out.command.arg_b) == (640, 460)
            assert m.cluster_target_track_id == m.locked_target_track_id == 101
            if frame == 11:
                assert m.cluster_bbox != left_extent  # Neighbours remain available for pile context.

        moved = replace(anchor, bbox=(564, 380, 200, 200))
        out = m.step(VisionSnapshot(frame_sequence=14, cargo=(moved, right)), pose(), status, 1.2)
        assert (out.command.arg_a, out.command.arg_b) == (664, 480)  # Track A, do not freeze its pixels.
        assert m.selected_batch.track_ids == (101,)


def test_cluster_range_uses_same_anchor_resolver_and_keeps_finite_request() -> None:
    from protocol import CMD_APPROACH_RANGE, CMD_CLUSTER_TARGET

    for reassociated in (False, True):
        m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
        m.first_common_delivered = True
        anchor = TrackedCargo(101, "green_supply", .9, (540, 360, 200, 200), hits=5)
        left = TrackedCargo(102, "green_supply", .8, (360, 380, 160, 160), hits=5)
        first = VisionSnapshot(frame_sequence=10, observed_monotonic_s=1., cargo=(anchor, left))
        m._start_cluster_approach(first, stm(), 1.)
        status = replace(stm(mode=52), camera_pitch_cdeg=12000)
        m.step(replace(first, range_pitch_cdeg=12000), pose(), status, 1.)
        if reassociated:
            # Existing spatial reassociation may resolve a changed tracker ID.
            # Ranging must use that same decision instead of a second selector.
            anchor = replace(anchor, track_id=104, bbox=(530, 360, 200, 200))
            neighbour = replace(anchor, track_id=105, bbox=(650, 360, 200, 200))
        else:
            neighbour = left
        view = VisionSnapshot(frame_sequence=11, observed_monotonic_s=1.1,
            cargo=(anchor, neighbour), range_pitch_cdeg=12000,
            range_distances=((anchor.track_id, 230), (neighbour.track_id, 550)))
        out = m.step(view, pose(), status, 1.1)
        assert out.reason == "approach_range_request"
        assert out.command.flags & CMD_APPROACH_RANGE and out.command.flags & CMD_CLUSTER_TARGET
        assert out.command.arg_a == 230 and out.command.arg_b == 12000
        assert out.command.source_vision_frame == 11
        assert m.cluster_target_track_id == m.locked_target_track_id == anchor.track_id
        assert m.selected_batch.track_ids == (anchor.track_id,)
        direction = m._cluster_approach_command(view)
        assert (direction.arg_a, direction.arg_b) == anchor.center_px
        repeated = m.step(replace(view, frame_sequence=12, observed_monotonic_s=1.2,
            cargo=(), range_distances=((anchor.track_id, 900),)), pose(), replace(status, mode=53), 1.2)
        assert repeated.command == out.command  # Do not restart a finite move on a new image.


def test_range_recheck_accepts_small_error_and_realigns_large_overshoot() -> None:
    from protocol import CMD_APPROACH_RANGE
    m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    m._select_batch(CargoBatch((1,), ("green_supply",), "material"))
    m.state = CompetitionState.APPROACH
    status = replace(stm(mode=52), camera_pitch_cdeg=12000)
    first = VisionSnapshot(frame_sequence=1, observed_monotonic_s=1.0,
        range_pitch_cdeg=12000, range_distances=((1, 380),))
    m._range_approach_output(first, status, 1.0)
    # The new-frame requirement already exists; this adds no settling wait.
    for cx in (813, 858, 705):
        target = TrackedCargo(1, "green_supply", .9, (cx - 20, 200, 40, 40))
        out = m._range_approach_output(replace(first, frame_sequence=2,
            observed_monotonic_s=1.1, cargo=(target,)), status, 1.1)
        assert out.reason == "approach_range_realign"
        assert not out.command.flags & CMD_APPROACH_RANGE
        assert out.command.arg_a == cx
        assert m.target_last_seen_s == 1.1 and m.target_last_center_px == target.center_px
        assert m.locked_target_track_id == target.track_id
    for cx in (576, 640, 704):
        target = TrackedCargo(1, "green_supply", .9, (cx - 20, 200, 40, 40))
        out = m._range_approach_output(replace(first, frame_sequence=3,
            observed_monotonic_s=1.2, cargo=(target,)), status, 1.2)
        assert out.command.flags & CMD_APPROACH_RANGE and out.command.arg_a == 380
        repeated = m._range_approach_output(replace(first, frame_sequence=4),
            replace(status, mode=53), 1.3)
        assert repeated.command == out.command
        m.range_command = None


def test_loaded_empty_reobserves_once_without_stable_empty_or_fake_load() -> None:
    for mode in (23, 48):
        m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
        m.state = CompetitionState.POST_GRAB_AUDIT if mode == 23 else CompetitionState.TRANSPORT_AUDIT
        m.post_grab_audit_active = mode == 23
        m._select_batch(CargoBatch((802,), ("injured_orange",), "injury"))
        status = replace(stm(mode=mode, flags=3), camera_pitch_cdeg=14000)
        positive = CargoAudit(left_class="injured_orange", left_count=1, total_count=1)

        def observe(frame, now, audit=positive, current=status, captured=None):
            return m.step(VisionSnapshot(frame_sequence=frame,
                observed_monotonic_s=now if captured is None else captured,
                capture_audit=audit), pose(), current, now)

        first = observe(10, 1.)
        assert first.reason == "audit_camera_frame_floor"
        # No additional 250 ms after the MCU reported its camera ready.
        old = observe(11, 1.005, captured=.99)
        assert old.reason == "audit_camera_frame_floor" and m.audit_hits == 0
        first_positive = observe(12, 1.01)
        assert first_positive.command.audit.total_count == 1 and m.audit_hits == 1
        out = observe(13, 1.04, CargoAudit())
        assert out.reason == "load_empty_reobserve" and out.event == "load_empty_reobserve"
        assert out.command.opcode == CMD_CARGO_AUDIT
        assert not out.command.audit.stable and out.command.audit.total_count == 0
        assert out.command.audit_vision_frame == 13 and m.audit_hits == 1
        epoch = m.audit_epoch
        again = observe(14, 1.05, CargoAudit())
        assert again.reason == "load_empty_gap" and not again.event
        assert again.command.audit_vision_frame == 0 and m.audit_hits == 1
        # Reuse the MCU's existing camera nudge. No new timer or request ACK.
        observe(15, 1.06, CargoAudit(), replace(status, camera_pitch_cdeg=13800, flags=2))
        returned = observe(16, 1.10, CargoAudit())
        assert returned.reason == "audit_camera_frame_floor" and m.empty_load_recheck_done
        observe(17, 1.11)
        assert m.audit_hits == 1
        gap = observe(18, 1.14, CargoAudit())
        assert gap.command.audit_vision_frame == 0 and not gap.command.audit.stable
        assert m.audit_hits == 1 and m.pending_audit is None and m.transport_audit_command is None
        stable = observe(19, 1.23)
        assert stable.command.audit.stable and stable.command.audit.total_count == 1
        assert m.audit_hits == 2 and m.audit_epoch == epoch
        # A later gap revokes cached STABLE authorization as well.
        revoked = observe(20, 1.25, CargoAudit())
        assert revoked.command.audit_vision_frame == 0 and not revoked.command.audit.stable
        assert m.pending_audit is None and m.transport_audit_command is None
        wire_started = m.audit_wire_mode_started_s
        for frame, now in ((21, 1.26), (22, 2.0), (23, 2.04)):
            missing = observe(frame, now, CargoAudit())
            assert not missing.command.audit.stable and missing.command.audit_vision_frame == 0
        assert m.audit_epoch == epoch and m.audit_wire_mode_started_s == wire_started
        # Real nonempty contradictory evidence still reaches the original audit.
        danger = replace(positive, danger_present=True)
        assert m._audit_observation_wait(VisionSnapshot(frame_sequence=24,
            observed_monotonic_s=2.1, capture_audit=danger), status, 2.1) is None


def test_safe_zone_no_box_requests_one_higher_view_and_blocks_retired_sweep() -> None:
    from protocol import CMD_ALIGN_SAFE_ZONE, CMD_CLEAR_SAFE_ZONE
    m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    m._select_batch(CargoBatch((1,), ("green_supply",), "material"))
    m.confirmed_delivery_destination = "material"
    m.carried_manifest = ("green_supply",)
    m.carried_total_count = 1
    m._begin_safe_zone_acquire(VisionSnapshot(frame_sequence=1), 1.)
    status = replace(stm(mode=11, flags=2), camera_pitch_cdeg=12000)
    view = VisionSnapshot(frame_sequence=2, observed_monotonic_s=6.1)
    out = m.step(view, pose(), status, 6.1)
    assert out.command.opcode == CMD_ALIGN_SAFE_ZONE and out.command.arg_b == 11500
    assert m.step(replace(view, frame_sequence=3, observed_monotonic_s=6.2),
        pose(), status, 6.2).command == out.command
    m.state = CompetitionState.ENTER_SAFE_ZONE
    obstacle = replace(view, safe_corridor_obstacle_class="green_supply",
        safe_corridor_obstacle_track_id=258, safe_corridor_obstacle_bbox=(308,566,258,239))
    out = m.step(obstacle, pose(), replace(status, mode=10), 6.3)
    assert out.state == CompetitionState.WAIT_SAFE_ZONE_CLEAR
    assert out.command.opcode != CMD_CLEAR_SAFE_ZONE
    assert out.reason == "legacy_safe_sweep_disabled"

    def acquiring():
        mission = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
        mission._select_batch(CargoBatch((1,), ("green_supply",), "material"))
        mission._begin_safe_zone_acquire(VisionSnapshot(frame_sequence=1), 1.)
        return mission

    clipped = (200, 0, 800, 90)
    normal = acquiring()
    for frame, now in ((2, 1.01), (3, 1.06), (4, 1.10)):
        fast = normal.step(VisionSnapshot(frame_sequence=frame,
            observed_monotonic_s=now, safe_bbox=clipped), pose(), status, now)
    assert fast.state == CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX
    assert normal.safe_zone_view_pitch_cdeg == 12000  # Three real frames take the fast path.

    retry = acquiring()
    retry.step(VisionSnapshot(frame_sequence=2, observed_monotonic_s=1.01,
        safe_bbox=clipped), pose(), status, 1.01)
    lost = VisionSnapshot(frame_sequence=3, observed_monotonic_s=1.05,
        safe_bbox=clipped, safe_zone_filter_blocked=True)  # Cached box, actual detection missing.
    higher = retry.step(lost, pose(), status, 1.05)
    assert higher.event == "safe_zone_acquire_reobserve" and higher.command.arg_b == 11500
    assert retry.safe_zone_freeze_hits == 0
    waiting = retry.step(replace(lost, frame_sequence=4, observed_monotonic_s=1.10),
        pose(), status, 1.10)
    assert not waiting.event and waiting.command == higher.command
    reported = replace(status, camera_pitch_cdeg=11500)
    # Reported commanded angle is not physical servo feedback; exclude old
    # pipeline images, then count three real new captures with no extra timer.
    retry.step(VisionSnapshot(frame_sequence=5, observed_monotonic_s=1.20,
        safe_bbox=clipped), pose(), reported, 1.30)
    old = retry.step(VisionSnapshot(frame_sequence=6, observed_monotonic_s=1.25,
        safe_bbox=clipped), pose(), reported, 1.32)
    assert old.reason == "safe_zone_higher_view_frame_wait" and retry.safe_zone_freeze_hits == 0
    for frame, now, captured in ((7, 1.34, 1.31), (8, 1.37, 1.35), (9, 1.41, 1.39)):
        observed = retry.step(VisionSnapshot(frame_sequence=frame,
            observed_monotonic_s=captured, safe_bbox=(200, 0, 800, 155)), pose(), reported, now)
    assert observed.state == CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX
    assert retry.safe_zone_freeze_hits == 3 and retry.safe_zone_view_pitch_cdeg == 11500


def test_range_cumulative_and_transport_sort() -> None:
    from protocol import CMD_APPROACH_RANGE, CMD_SIDE_VALID, CMD_STAGE_ONLY, CMD_DRIVE_STRAIGHT, CMD_USE_FINAL_HEADING
    m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    m._select_batch(CargoBatch((1,), ("green_supply",), "material"))
    m.state = CompetitionState.APPROACH
    target = cargo(1, "green_supply", 600, 500)
    status = replace(stm(mode=52), camera_pitch_cdeg=10500)
    view = VisionSnapshot(frame_sequence=1, observed_monotonic_s=1.0, cargo=(target,),
                          range_pitch_cdeg=10500, range_distances=((1, 650),))
    assert m._range_approach_output(view, status, 1.0).command.opcode == CMD_HOLD
    request = m._range_approach_output(replace(view, frame_sequence=2, observed_monotonic_s=1.1), status, 1.1)
    assert request.command.flags & CMD_APPROACH_RANGE and request.command.arg_a == 650
    assert request.command.source_vision_frame == 2
    repeated = m._range_approach_output(replace(view, frame_sequence=3, range_distances=((1, 100),)), replace(status, mode=53), 1.2)
    assert repeated.command == request.command
    m._clear_range_approach()
    m.state = CompetitionState.CAPTURE_AUDIT
    status = replace(stm(mode=21, flags=1), camera_pitch_cdeg=14000)
    audit = CargoAudit(left_class="green_supply", left_count=1, left_green_count=1, total_count=1)
    v1 = VisionSnapshot(frame_sequence=10, observed_monotonic_s=2.0, capture_audit=audit)
    m._audit_output(v1, status, 2.0)
    absent = replace(v1, frame_sequence=11, observed_monotonic_s=2.05, capture_audit=None, capture_observation_valid=False)
    out = m._audit_output(absent, status, 2.05)
    assert not out.command.audit.stable and m.audit_hits == 1
    assert not m._audit_output(v1, status, 2.06).command.audit.stable
    out = m._audit_output(replace(v1, frame_sequence=12, observed_monotonic_s=2.12), status, 2.12)
    assert out.command.audit.stable and m.pending_audit is not None
    m._audit_confirmation_output(replace(absent, frame_sequence=13, observed_monotonic_s=2.15), status, 2.15)
    assert m.pending_audit is None and m.audit_hits == 2
    assert not m._audit_output(replace(v1, frame_sequence=12, observed_monotonic_s=2.12), status, 2.16).command.audit.stable
    renewed = m._audit_output(replace(v1, frame_sequence=14, observed_monotonic_s=2.30), status, 2.30)
    assert renewed.command.audit.stable and renewed.command.audit_vision_frame == 14
    m._audit_confirmation_output(replace(absent, frame_sequence=15, observed_monotonic_s=2.85), status, 2.85)
    renewed = m._audit_output(replace(v1, frame_sequence=16, observed_monotonic_s=2.90), status, 2.90)
    assert not renewed.command.audit.stable and m.audit_hits == 1
    # Transport keeps budget, commands a curve, then waits for a NEW capture audit.
    t = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    t.first_common_delivered = True
    t._select_batch(CargoBatch((1,), ("green_supply",), "material"))
    t._latch_carried_manifest(audit)
    multi = replace(audit, left_count=3, left_green_count=3, total_count=3)
    status48 = replace(stm(mode=48, flags=3), camera_pitch_cdeg=14000)
    t._begin_transport_audit(VisionSnapshot(frame_sequence=1), pose(), status48, 3.0)
    for frame, now in ((2,3.3),(3,3.41)):
        view = VisionSnapshot(frame_sequence=frame, observed_monotonic_s=now, capture_audit=multi)
        out = t._transport_audit_output(view, pose(), status48, now)
    ack = replace(relay_stm(out, mode=48, flags=67), camera_pitch_cdeg=14000,
                  context_valid=True, context_matches=True, accepted_command=CMD_CARGO_AUDIT, action_status=1)
    curve = t._transport_audit_output(view, pose(), ack, 3.42)
    assert curve.command.opcode == CMD_DISPERSE_PILE and curve.command.flags & CMD_SIDE_VALID
    assert t.transport_separation_active and t.pending_audit is None
    accepted = replace(relay_stm(curve, mode=25, flags=3), camera_pitch_cdeg=14000,
                       context_valid=True, context_matches=True, accepted_command=CMD_DISPERSE_PILE, action_status=1)
    assert t._disperse_output(view, accepted, 3.5).command == curve.command
    assert t.disperse_attempts == 1
    after = t._disperse_output(replace(view, frame_sequence=4), replace(accepted, mode=35), 4.0)
    assert after.command.opcode == CMD_CARGO_AUDIT and t.disperse_attempts == 1
    # Same destination but physically wrong half must produce a new straight STAGE NAV.
    t = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    t.first_common_delivered = True
    t._select_batch(CargoBatch((1,), ("green_supply",), "material"))
    t._latch_carried_manifest(audit)
    tx, ty = t._target_point()
    wrong = pose(-tx, ty)
    t._begin_transport_audit(VisionSnapshot(frame_sequence=1), wrong, status48, 5.0)
    for frame, now in ((2,5.3),(3,5.41)):
        view = VisionSnapshot(frame_sequence=frame, observed_monotonic_s=now, capture_audit=audit)
        out = t._transport_audit_output(view, wrong, status48, now)
    ack = replace(relay_stm(out, mode=48, flags=83), camera_pitch_cdeg=14000,
                  context_valid=True, context_matches=True, accepted_command=CMD_CARGO_AUDIT, action_status=1)
    nav = t._transport_audit_output(view, wrong, ack, 5.42)
    assert nav.command.opcode == CMD_NAVIGATE_WAYPOINT
    assert nav.command.flags & (CMD_STAGE_ONLY | CMD_DRIVE_STRAIGHT | CMD_USE_FINAL_HEADING) == (CMD_STAGE_ONLY | CMD_DRIVE_STRAIGHT | CMD_USE_FINAL_HEADING)


def test_transport_audit_continuing_frames_allow_delayed_relay_handoff() -> None:
    from protocol import CMD_ALIGN_SAFE_ZONE

    m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    m._select_batch(CargoBatch((1,), ("green_supply",), "material"))
    audit = CargoAudit(left_class="green_supply", left_count=1,
                       left_green_count=1, total_count=1)
    m._latch_carried_manifest(audit)
    x, y = m._target_point()
    current_pose = pose(x, y)
    status = replace(stm(mode=48, flags=3), camera_pitch_cdeg=14000,
                     context_valid=True, context_matches=True,
                     accepted_command=CMD_CARGO_AUDIT, action_status=1)
    m._begin_transport_audit(VisionSnapshot(frame_sequence=1), current_pose, status, 1.0)
    outputs = []
    for frame in range(2, 22):
        now = 1.0 + (frame - 1) * 0.13
        received = status
        if len(outputs) >= 2:
            # Actual relay reporting trails the live camera by two frames.
            received = replace(relay_stm(outputs[-2], mode=48, flags=83,
                                        relay_mission_tx_frames=frame - 2),
                               camera_pitch_cdeg=14000, context_valid=True,
                               context_matches=True, accepted_command=CMD_CARGO_AUDIT,
                               action_status=1)
        view = VisionSnapshot(frame_sequence=frame, observed_monotonic_s=now,
                              capture_audit=audit)
        out = m._transport_audit_output(view, current_pose, received, now)
        if out.command.opcode == CMD_ALIGN_SAFE_ZONE:
            break
        assert out.command.opcode == CMD_CARGO_AUDIT
        assert out.command.audit_vision_frame == frame  # Never freeze the source image.
        outputs.append(out)
    assert out.command.opcode == CMD_ALIGN_SAFE_ZONE
    assert frame <= 6
    # Before MCU READY, distinct visual samples still have distinct IDs.
    assert outputs[0].command.audit.audit_id != outputs[1].command.audit.audit_id
    assert m.carried_total_count == 1


def test_transport_audit_handoff_retains_exact_payload_and_observation_checks() -> None:
    for changed in ("side", "missing", "camera", "context", "not_ready"):
        m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
        m._select_batch(CargoBatch((1,), ("green_supply",), "material"))
        audit = CargoAudit(left_class="green_supply", left_count=1,
                           left_green_count=1, total_count=1)
        m._latch_carried_manifest(audit)
        x, y = m._target_point()
        current_pose = pose(x, y)
        status = replace(stm(mode=48, flags=3), camera_pitch_cdeg=14000,
                         context_valid=True, context_matches=True,
                         accepted_command=CMD_CARGO_AUDIT, action_status=1)
        m._begin_transport_audit(VisionSnapshot(frame_sequence=1), current_pose, status, 1.0)
        for frame in (2, 3):
            now = 1.0 + frame * 0.13
            view = VisionSnapshot(frame_sequence=frame, observed_monotonic_s=now,
                                  capture_audit=audit)
            out = m._transport_audit_output(view, current_pose, status, now)
        published = out.command
        received = replace(relay_stm(out, mode=48, flags=83, relay_mission_tx_frames=5),
                           camera_pitch_cdeg=14000, context_valid=True, context_matches=True,
                           accepted_command=CMD_CARGO_AUDIT, action_status=1)
        view = replace(view, frame_sequence=4, observed_monotonic_s=1.52)
        if changed == "side":
            view = replace(view, capture_audit=CargoAudit(right_class="green_supply",
                right_count=1, right_green_count=1, total_count=1))
        elif changed == "missing":
            view = replace(view, capture_audit=None, capture_observation_valid=False)
        elif changed == "camera":
            received = replace(received, camera_pitch_cdeg=13800)
        elif changed == "context":
            received = replace(received, context_matches=False)
        else:
            received = replace(received, flags=3)
        out = m._transport_audit_output(view, current_pose, received, 1.52)
        assert m.state == CompetitionState.TRANSPORT_AUDIT, changed
        if changed in {"missing", "camera"}:
            assert not out.command.audit.stable
            assert m.transport_audit_command is None
        else:
            assert out.command.audit.audit_id != published.audit.audit_id, changed


def test_safe_staging_arrival_keeps_zero_handoff_and_stash_window() -> None:
    from protocol import CMD_STAGE_ONLY

    # Reproduce the logged 42.7 mm axial miss for both field sides and cargo
    # destinations. Arrival requests D=0; it must not skip the lower handoff.
    for side in ("red", "blue"):
        for destination, class_name in (
            ("material", "green_supply"), ("injury", "injured_orange")
        ):
            m = CompetitionMission(CompetitionSettings(side=side))
            m._select_batch(CargoBatch((1,), (class_name,), destination))
            m.confirmed_delivery_destination = destination
            m.state = CompetitionState.NAVIGATE
            x, y = m._target_point()
            status = stm(mode=STM_MODE_NAVIGATE, flags=2)

            # Widen only the STAGE lateral window; retain its axial window
            # and the original invalid-pose behaviour.
            for sample in (
                pose(x, y + 0.051), pose(x + 0.101, y),
                replace(pose(x, y), valid=False),
            ):
                m._navigate(VisionSnapshot(), sample, status, 1.0)
                assert m.staging_zero_tx_baseline is None

            sample = pose(x + 0.09, y + m.settings.side_sign * 0.0427)
            assert not m._at_target(sample, (x, y))  # Generic/stash window stays narrow.
            requested = m._navigate(VisionSnapshot(), sample, status, 1.1)
            assert requested.state == CompetitionState.NAVIGATE
            assert requested.command and requested.command.opcode == CMD_NAVIGATE_WAYPOINT
            assert requested.command.flags & CMD_STAGE_ONLY
            assert requested.command.arg_a == 0
            assert not m.staging_zero_accepted

            # Pose jitter after arrival does not restart a nonzero correction.
            pending = m._navigate(VisionSnapshot(), pose(x, y + 0.08), status, 1.2)
            assert pending.command and pending.command.arg_a == 0
            assert pending.state == CompetitionState.NAVIGATE
            done = m._navigate(VisionSnapshot(), sample, replace(status, flags=2 | 32), 1.3)
            assert done.state == CompetitionState.TRANSPORT_AUDIT
            assert done.command and done.command.opcode == CMD_CARGO_AUDIT

    # The same 42.7 mm error at a stash point still requires correction.
    stash = CompetitionMission(CompetitionSettings(side="red"))
    stash._select_batch(CargoBatch((1,), ("green_supply",), "stash", initial_stash=True))
    stash.state = CompetitionState.INITIAL_STASH_NAV
    stash.navigation_initial_ack = 0
    x, y = stash._target_point()
    output = stash._navigate(VisionSnapshot(), pose(x, y + 0.0427),
        stm(mode=STM_MODE_NAVIGATE, acknowledged_sequence=1), 2.0)
    assert output.state == CompetitionState.INITIAL_STASH_NAV
    assert output.command and output.command.arg_a == 43
    assert stash.staging_zero_tx_baseline is None
    output = stash._navigate(VisionSnapshot(), pose(x + 0.09, y),
        stm(mode=STM_MODE_NAVIGATE, acknowledged_sequence=1), 2.1)
    assert output.state == CompetitionState.INITIAL_STASH_NAV
    assert output.command and output.command.arg_a == 90
    assert stash.staging_zero_tx_baseline is None


def test_safe_staging_terminal_only_corrects_out_of_window_axes() -> None:
    from protocol import CMD_STAGE_ONLY

    for side in ("red", "blue"):
        for destination, class_name in (
            ("material", "green_supply"), ("injury", "injured_orange")
        ):
            m = CompetitionMission(CompetitionSettings(side=side))
            m._select_batch(CargoBatch((1,), (class_name,), destination))
            m.confirmed_delivery_destination = destination
            target = m._target_point()
            for x_sign in (-1, 1):
                for y_sign in (-1, 1):
                    def command(dx, dy, *, stage=True):
                        return m._navigation_command(
                            pose(target[0] + dx * x_sign, target[1] + dy * y_sign),
                            target, staging_only=stage,
                        )

                    # X is already acceptable: move only along field Y.
                    axial = command(0.09, 0.07)
                    assert axial.flags & CMD_STAGE_ONLY
                    assert axial.arg_a == 70
                    assert axial.aux == (27000 if y_sign > 0 else 9000)
                    # Y is already acceptable: move only along field X.
                    lateral = command(0.11, 0.04)
                    assert lateral.arg_a == 110
                    assert lateral.aux == (18000 if x_sign > 0 else 0)
                    # Both axes need correction, so retain the diagonal vector.
                    diagonal, geometric = command(0.11, 0.07), command(0.11, 0.07, stage=False)
                    assert diagonal.arg_a == geometric.arg_a == 130
                    assert diagonal.aux == geometric.aux
                    # Outside the original 18 cm fine-entry range, keep the
                    # coarse H/D even when one axis lies inside the window.
                    coarse, geometric = command(0.09, 0.20), command(0.09, 0.20, stage=False)
                    assert coarse.arg_a == geometric.arg_a == 219
                    assert coarse.aux == geometric.aux
                    # Non-STAGE navigation must not inherit per-axis masking.
                    generic = command(0.09, 0.07, stage=False)
                    assert generic.arg_a == 114
                    assert generic.aux != axial.aux


def test_green_sorting_uses_one_keep_policy_and_visual_singleton_exception() -> None:
    m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    m.first_common_delivered = True
    m._select_batch(CargoBatch((10,), ("injured_orange",), "injury"))
    greens = CargoAudit(left_class="green_supply", right_class="green_supply",
                        left_count=3, right_count=1, left_green_count=3,
                        right_green_count=1, total_count=4)
    for audit, expected in (
        (greens, "left"),
        (replace(greens, left_count=1, right_count=3,
                 left_green_count=1, right_green_count=3), "right"),
        (replace(greens, right_count=2, right_green_count=2, total_count=5), "left"),
        (replace(greens, right_isolated_green=True), "right"),
        (replace(greens, right_isolated_green=True, right_count=2,
                 right_green_count=2, total_count=5), "left"),
    ):
        assert m._preferred_keep_side(audit) == expected
        assert m._choose_release_side(audit) == ("right" if expected == "left" else "left")
        assert not m._audit_valid(audit)  # Choosing a group never authorizes its delivery.
    mixed = CargoAudit(left_class="injured_orange", right_class="green_supply",
                       left_count=1, right_count=2, right_green_count=2,
                       total_count=3, injury_mixed=True, right_stable_track_id=20)
    assert m._preferred_keep_side(mixed) == "right"
    assert not m._audit_payload(mixed).destination_injury
    generation = m.task_generation
    m._prefer_green_batch(mixed)
    assert m.selected_batch.destination == "material"
    assert m.selected_batch.classes == ("green_supply",) and m.selected_batch.track_ids == (20,)
    assert m.task_generation == generation  # No new task/handshake to change sorting priority.
    m.first_common_delivered = False
    assert not m._audit_valid(CargoAudit(left_class="injured_orange", left_count=1, total_count=1))
    assert m._audit_valid(CargoAudit(left_class="green_supply", left_count=1, total_count=1))


def test_progressive_sorting_allows_third_action_and_counts_only_real_attempts() -> None:
    def greens(left, right):
        return CargoAudit(left_class="green_supply" if left else "",
            right_class="green_supply" if right else "", left_count=left,
            right_count=right, left_green_count=left, right_green_count=right,
            total_count=left + right, stable=True)

    m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    m._select_batch(CargoBatch((1,), ("green_supply",), "material"))
    for attempt, audit in enumerate((greens(3, 2), greens(2, 1), greens(2, 0)), 1):
        m.cluster_keep_side = m._preferred_keep_side(audit)
        out = m._start_disperse(audit, stm(mode=37), float(attempt))
        assert out.state == CompetitionState.DISPERSE and out.command.opcode == CMD_DISPERSE_PILE
        assert m.disperse_attempts == attempt - 1  # Sending is not acceptance.
        m._count_separation_acceptance("left")
        m._count_separation_acceptance("left")
        assert m.disperse_attempts == attempt
        assert not m._separation_limit_reached()  # An unreviewed action is not a failure.
    m._review_separation_result(greens(1, 0))
    assert m.disperse_attempts == m.separation_progress_attempt == 3
    assert m._audit_valid(greens(1, 0)) and not m._separation_limit_reached()
    # Repeated planner cycles on the same completed action cannot reset its result.
    m._review_separation_result(greens(1, 0))
    assert m.separation_progress_attempt == 3

    stalled = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    stalled._select_batch(CargoBatch((1,), ("green_supply",), "material"))
    unchanged = greens(2, 1)
    for attempt in range(2):
        stalled.cluster_keep_side = "left"
        assert stalled._start_disperse(unchanged, stm(mode=37), float(attempt)).state == CompetitionState.DISPERSE
        stalled._count_separation_acceptance("left")
    out = stalled._start_disperse(unchanged, stm(mode=37), 3.0)
    assert out.command.opcode == CMD_RELEASE_BOTH and stalled.disperse_attempts == 2
    assert out.state == CompetitionState.INVALID_RELEASE
    # Losing the desired green is not successful reduction of an unwanted group.
    lost = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    lost._select_batch(CargoBatch((1,), ("green_supply",), "material"))
    lost.separation_start_audit = CargoAudit(left_class="green_supply", right_class="injured_orange",
                                            left_count=1, right_count=1, total_count=2)
    lost._count_separation_acceptance("left")
    lost._review_separation_result(CargoAudit(right_class="injured_orange", right_count=1, total_count=1))
    assert lost.separation_progress_attempt == 0


def test_post_grab_multigreen_reuses_accepted_audit_without_release_backoff() -> None:
    from protocol import CMD_SIDE_VALID
    for isolated, expected in ((False, "left"), (True, "right")):
        m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
        m.first_common_delivered = True
        m._select_batch(CargoBatch((1,), ("injured_orange",), "injury"))
        audit = CargoAudit(left_class="green_supply", right_class="green_supply",
            left_count=3, right_count=1, left_green_count=3, right_green_count=1,
            total_count=4, stable=True, right_isolated_green=isolated)
        m.post_grab_audit_active = True
        status = replace(stm(mode=23, flags=67), camera_pitch_cdeg=14000)
        for frame, now in ((1, 1.0), (2, 1.3), (3, 1.41)):
            view = VisionSnapshot(frame_sequence=frame, observed_monotonic_s=now, capture_audit=audit)
            pending = m._audit_output(view, status, now)
        assert m.pending_audit_release_context == "post_grab_invalid"
        ack = replace(relay_stm(pending, mode=23, flags=67), camera_pitch_cdeg=14000,
                      context_valid=True, context_matches=True,
                      accepted_command=CMD_CARGO_AUDIT, action_status=1)
        out = m._audit_confirmation_output(view, ack, 1.42)
        assert out.state == CompetitionState.DISPERSE
        assert out.command.opcode == CMD_DISPERSE_PILE and out.command.flags & CMD_SIDE_VALID
        assert bool(out.command.flags & CMD_TARGET_RIGHT) == (expected == "right")
        assert m.cluster_keep_side == expected and not m.post_grab_audit_active
        assert m.selected_batch.destination == "material"


def main() -> None:
    test_green_sorting_uses_one_keep_policy_and_visual_singleton_exception()
    test_progressive_sorting_allows_third_action_and_counts_only_real_attempts()
    test_post_grab_multigreen_reuses_accepted_audit_without_release_backoff()
    test_safe_staging_arrival_keeps_zero_handoff_and_stash_window()
    test_safe_staging_terminal_only_corrects_out_of_window_axes()
    test_range_cumulative_and_transport_sort()
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
    test_post_delivery_returns_to_own_side_with_loose_latched_arrival()
    test_delivery_requires_outside_to_inside_transition()
    test_boundary_guard_points_back_into_field()
    print("competition state machine PASS")


if __name__ == "__main__":
    main()


def test_sweep_uses_ground_foot_without_grabbing_inside_safe_zone() -> None:
    from state_machine import SweepPickupContext, cargo_in_front_region
    safe_bbox = (352, 4, 926, 245)
    polygon = ((460, 1023), (820, 1023), (815, 249), (352, 249))
    # Actual second-run frame4502: centre(712,232.5) projects over the
    # safe-zone box; foot(712,310) is in the ground corridor in front.
    blue = TrackedCargo(335, "danger_cyan", .85, (644, 155, 136, 155), hits=8)
    assert cargo_in_front_region(blue, polygon, safe_bbox)
    mission = CompetitionMission(CompetitionSettings(side="blue", initial_stash_enabled=False))
    mission.safe_sweep_pickup = SweepPickupContext(
        target_id=335, target_class="danger_cyan", target_bbox=blue.bbox,
        excluded_ids=frozenset())
    assert mission.safe_sweep_target((blue,), polygon, safe_bbox).track_id == 335
    # Explicit inside evidence and the complete contact threshold remain
    # exclusions, even when the upper part of a tall object spans both regions.
    assert not cargo_in_front_region(replace(blue, inside_safe_zone=True), polygon, safe_bbox)
    inside = replace(blue, bbox=(662, 70, 100, 130))
    assert not cargo_in_front_region(inside, polygon, safe_bbox)
    threshold = replace(blue, bbox=(662, 94, 100, 155))  # Foot exactly at y249.
    assert not cargo_in_front_region(threshold, polygon, safe_bbox)
    assert cargo_in_front_region(replace(threshold, bbox=(662, 95, 100, 155)),
                                 polygon, safe_bbox)
    outside_path = replace(blue, bbox=(950, 155, 100, 155))
    assert not cargo_in_front_region(outside_path, polygon, safe_bbox)
    assert mission.safe_sweep_target((inside, outside_path), polygon, safe_bbox) is None




def test_post_first_two_materials_follow_existing_grab_and_transport() -> None:
    from collections import Counter
    from run_competition_rescue import audit_from_cargo
    cases = ((("green_supply", "core_black"), ("left", "left")),
             (("green_supply", "core_black"), ("left", "right")),
             (("green_supply", "green_supply"), ("left", "right")),
             (("core_black", "core_black"), ("right", "right")))
    for names, sides, entry_mode in ((n, s, mode) for n, s in cases for mode in (21, 38)):
        items = tuple(cargo(i + 1, name, 460 + 65 * i, 470) for i, name in enumerate(names))
        audit = audit_from_cargo(items, side_by_track=dict(zip(range(1, len(items) + 1), sides)))
        m = CompetitionMission(CompetitionSettings(side="blue", initial_stash_enabled=False))
        m.first_common_delivered = True
        m._select_batch(CargoBatch((1,), (names[0],), "material"))
        m.state = CompetitionState.CAPTURE_AUDIT
        assert m._audit_valid(audit)
        m.cluster_audit_active = entry_mode == 38
        status = replace(stm(mode=entry_mode, flags=1), camera_pitch_cdeg=14000)
        for frame, now in ((1, 1.), (2, 1.12)):
            view = VisionSnapshot(frame_sequence=frame, observed_monotonic_s=now,
                                  capture_audit=audit)
            out = m._audit_output(view, status, now)
        assert out.command.opcode == CMD_CARGO_AUDIT and m.pending_audit_valid
        if entry_mode == 38:
            # The cluster audit is observed in38, then the board exposes37
            # after accepting its stable audit. An ACK still in38 cannot
            # authorize GRAB before the existing physical handoff.
            still_auditing = replace(relay_stm(out, mode=38, flags=81),
                                     camera_pitch_cdeg=14000)
            waiting = m._audit_confirmation_output(view, still_auditing, 1.125)
            assert waiting.state == CompetitionState.AUDIT_CONFIRM
            assert waiting.command.opcode == CMD_CARGO_AUDIT
        confirmation_mode = 37 if entry_mode == 38 else 21
        acknowledged = replace(relay_stm(out, mode=confirmation_mode, flags=81),
                               camera_pitch_cdeg=14000)
        out = m._audit_confirmation_output(view, acknowledged, 1.13)
        assert out.state == CompetitionState.GRAB and out.command.opcode == CMD_GRAB_CONFIRMED
        assert m.disperse_attempts == 0
        # Exercise the real mode23->22 handoff. Every actual item is bound;
        # count is not reduced to the one SEARCH anchor track.
        m._begin_post_grab_audit(view, 2.)
        for frame, now in ((3, 2.), (4, 2.12), (5, 2.24)):
            view = VisionSnapshot(frame_sequence=frame, observed_monotonic_s=now,
                                  capture_audit=audit)
            out = m._audit_output(view, replace(stm(mode=23, flags=3),
                                               camera_pitch_cdeg=14000), now)
        acknowledged = replace(relay_stm(out, mode=22, flags=83,
            acknowledged_sequence=3, relay_mission_tx_frames=3),
            camera_pitch_cdeg=14000, context_valid=True, context_matches=True,
            accepted_command=CMD_CARGO_AUDIT, action_status=1)
        out = m._audit_confirmation_output(view, acknowledged, 2.25)
        assert out.reason == "post_grab_audit_confirmed"
        assert m.carried_total_count == len(names)
        assert Counter(m.carried_manifest) == Counter(names)
        assert m.selected_batch.total_count == len(names)
        assert m.selected_batch.destination == "material"
        assert m.carried_delivery_count() == len(names)
        assert m.carried_delivery_classes() == frozenset(names)
        # At mode48 the same legal batch aligns normally, never separates.
        tx, ty = m._target_point()
        staging_pose = PoseSnapshot(True, tx, ty, 270., 0.)
        transport_status = replace(stm(mode=48, flags=3), camera_pitch_cdeg=14000)
        view = VisionSnapshot(frame_sequence=6, observed_monotonic_s=3., capture_audit=audit)
        m._begin_transport_audit(view, staging_pose, transport_status, 3.)
        for frame, now in ((7, 3.12), (8, 3.24)):
            view = VisionSnapshot(frame_sequence=frame, observed_monotonic_s=now, capture_audit=audit)
            out = m._transport_audit_output(view, staging_pose, transport_status, now)
        acknowledged = replace(relay_stm(out, mode=48, flags=83,
            acknowledged_sequence=4, relay_mission_tx_frames=4),
            camera_pitch_cdeg=14000, context_valid=True, context_matches=True,
            accepted_command=CMD_CARGO_AUDIT, action_status=1)
        out = m._transport_audit_output(view, staging_pose, acknowledged, 3.25)
        assert out.reason == "transport_audit_valid" and m.disperse_attempts == 0
        assert m.carried_total_count == len(names)


def test_two_material_policy_keeps_first_green_and_excludes_other_loads() -> None:
    from run_competition_rescue import audit_from_cargo
    m = CompetitionMission(CompetitionSettings(side="blue", initial_stash_enabled=False))
    assert m.settings.max_batch_count == 2
    m._select_batch(CargoBatch((1,), ("green_supply",), "material"))
    def audited(names):
        items = tuple(cargo(i + 1, name, 450 + 45 * i, 480) for i, name in enumerate(names))
        return audit_from_cargo(items, side_by_track={item.track_id: "left" for item in items})
    assert m._audit_valid(audited(("green_supply",)))
    for names in (("green_supply", "green_supply"), ("green_supply", "core_black"),
                  ("core_black",), ("injured_orange",)):
        assert not m._audit_valid(audited(names))
    m.first_common_delivered = True
    assert m._audit_valid(audited(("injured_orange",)))
    for names in (("green_supply",) * 3, ("injured_orange",) * 2,
                  ("green_supply", "injured_orange"), ("green_supply", "danger_cyan")):
        assert not m._audit_valid(audited(names))
    legal = audited(("green_supply", "core_black"))
    for invalid in (replace(legal, unknown_present=True), replace(legal, left_invalid=True),
                    replace(legal, total_count=3), replace(legal, left_green_count=0)):
        assert not m._audit_valid(invalid)
    assert m._side_is_legal_for_task("mixed_material", 2)
    assert not m._side_is_legal_for_task("injured_orange", 2)
    for invalid_max in (0, 3):
        try:
            CompetitionSettings(side="red", max_batch_count=invalid_max)
        except ValueError:
            pass
        else:
            raise AssertionError("material batch must stay within one or two")


def test_two_material_retrieval_keeps_actual_single_remaining():
    m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    m.first_common_delivered = True
    m._select_batch(CargoBatch((1,), ("green_supply",), "material"))
    m._latch_carried_manifest(CargoAudit(left_class="mixed_material", left_count=2,
                                       left_green_count=1, total_count=2))
    m.safe_sweep_reaudit_active = True
    status = replace(stm(mode=23, flags=3), camera_pitch_cdeg=14000)
    m._begin_post_grab_audit(VisionSnapshot(frame_sequence=1), 1.0)
    actual = CargoAudit(right_class="core_black", right_count=1, total_count=1)
    for frame, now in ((2, 1.01), (3, 1.13), (4, 1.25)):
        view = VisionSnapshot(frame_sequence=frame, observed_monotonic_s=now,
                              capture_audit=actual)
        out = m._audit_output(view, status, now)
    ack = replace(relay_stm(out, mode=40, flags=83, relay_mission_tx_frames=3),
                  camera_pitch_cdeg=14000, context_valid=True, context_matches=True,
                  accepted_command=CMD_CARGO_AUDIT, action_status=1)
    result = m._audit_confirmation_output(view, ack, 1.26)
    assert result.reason == "safe_zone_clear_realign_start"
    assert m.carried_manifest == ("core_black",)
    assert m.carried_total_count == m.selected_batch.total_count == 1


def test_visual_lateral_uses_live_target_not_ack_and_keeps_local_straight_enter():
    from protocol import CMD_ALIGN_SAFE_ZONE, CMD_ENTER_SAFE_ZONE
    for side in ("red", "blue"):
        m = CompetitionMission(CompetitionSettings(side=side, initial_stash_enabled=False))
        m._select_batch(CargoBatch((1,), ("injured_orange",), "injury"))
        m.first_common_delivered = True
        m.state = CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX
        m._update_safe_zone_visual_target((216, 0, 941, 113), 10)
        status = StmSnapshot(mode=11, flags=2, age_ms=0, camera_pitch_cdeg=12000,
            context_valid=True, context_matches=True, accepted_command=CMD_ALIGN_SAFE_ZONE, action_status=3)
        view = VisionSnapshot(frame_sequence=11, safe_bbox=(216, 0, 941, 113))
        out = m.step(view, pose(), status, 1.)
        assert out.state == CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX
        assert out.command.arg_a == 203  # A fake/legacy completed ACK cannot bypass visual error.
        centered = replace(view, frame_sequence=13, safe_bbox=(16, 0, 936, 113))
        accepted_only = m.step(centered, pose(), replace(status, action_status=1), 1.02)
        assert accepted_only.state == CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX
        ready = m.step(centered, pose(), status, 1.03)
        assert ready.state == CompetitionState.SAFE_ZONE_CORRIDOR_CHECK
        corridor = replace(centered, frame_sequence=14,
            safe_corridor_polygon=((460,1023),(820,1023),(952,113),(484,113)))
        # A bad map position cannot steer the final local-odometry ENTER.
        enter = m.step(corridor, PoseSnapshot(True, -.8, -.5, 170., 0), status, 1.04)
        assert enter.command.opcode == CMD_ENTER_SAFE_ZONE
        assert enter.command.arg_a == enter.command.arg_b == enter.command.aux == 0
        assert enter.state == CompetitionState.ENTER_SAFE_ZONE


def test_locked_safe_zone_loss_reacquires_higher_view_without_losing_cargo():
    from protocol import CMD_ALIGN_SAFE_ZONE, CMD_VISUAL_CORRECTION_VALID
    for side in ("red", "blue"):
        for pitch, lost_kind in ((12000, "missing"),
                                 (12000, "pitch"), (11500, "missing")):
            m = CompetitionMission(CompetitionSettings(side=side, initial_stash_enabled=False))
            batch = CargoBatch((1,), ("green_supply",), "material")
            m._select_batch(batch)
            m.carried_manifest = batch.classes
            m.carried_total_count = 1
            m.confirmed_delivery_destination = "material"
            m.state = CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX
            m.safe_zone_view_pitch_cdeg = pitch
            m._update_safe_zone_visual_target((216, 0, 941, 113), 10)
            status = StmSnapshot(mode=11, flags=2, camera_pitch_cdeg=pitch, age_ms=0,
                context_valid=True, context_matches=True,
                accepted_command=CMD_ALIGN_SAFE_ZONE, action_status=3)
            view = VisionSnapshot(frame_sequence=11, observed_monotonic_s=1.,
                safe_bbox=None if lost_kind == "missing" else (216, 0, 941, 113),
                safe_zone_filter_blocked=lost_kind == "filtered")
            if lost_kind == "pitch":
                status = replace(status, camera_pitch_cdeg=14000)
            lost = m.step(view, pose(), status, 1.)
            assert lost.state == CompetitionState.ACQUIRE_SAFE_ZONE
            assert lost.event == "safe_zone_visual_reobserve"
            assert lost.command.opcode == CMD_ALIGN_SAFE_ZONE
            assert lost.command.flags & CMD_VISUAL_CORRECTION_VALID
            assert lost.command.arg_a == lost.command.source_vision_frame == 0
            assert lost.command.arg_b == 11500
            assert m.locked_safe_bbox is None and not m.safe_zone_lateral_active
            assert m.selected_batch == batch and m.carried_manifest == batch.classes
            assert m.carried_total_count == 1 and m.confirmed_delivery_destination == "material"
            # A DONE for the zero-translation request is not a visual fix.
            reported = replace(status, camera_pitch_cdeg=11500)
            waiting = m.step(replace(view, frame_sequence=12, safe_bbox=None), pose(), reported, 1.01)
            assert waiting.state == CompetitionState.ACQUIRE_SAFE_ZONE
            assert not waiting.event and waiting.command == lost.command
            # Existing new-view ownership and three-frame acquisition are reused.
            centered = (320, 0, 960, 113)
            for frame, now in ((13, 1.02), (14, 1.03), (15, 1.04)):
                recovered = m.step(VisionSnapshot(frame_sequence=frame, safe_bbox=centered,
                    observed_monotonic_s=now), pose(), reported, now)
                if frame < 15:
                    assert recovered.state == CompetitionState.ACQUIRE_SAFE_ZONE
            assert recovered.state == CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX
            assert recovered.command.source_vision_frame == 15
            assert recovered.command.arg_b == 11500 and recovered.command.arg_a == 0
            assert recovered.event == "safe_zone_bbox_locked"
            assert m.step(VisionSnapshot(frame_sequence=16, safe_bbox=centered), pose(),
                reported, 1.05).state == CompetitionState.SAFE_ZONE_CORRIDOR_CHECK
            m._reset_safe_zone_alignment()
            assert not m.safe_zone_reobserve_visual and m.safe_zone_higher_view_ready_s is None


def test_locked_safe_zone_short_gap_preserves_real_action_without_authorizing_entry():
    from protocol import CMD_ALIGN_SAFE_ZONE
    for state in (CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX,
                  CompetitionState.SAFE_ZONE_CORRIDOR_CHECK,
                  CompetitionState.WAIT_SAFE_ZONE_CLEAR):
        m = CompetitionMission(CompetitionSettings(side="blue", initial_stash_enabled=False,
                                                   bulldozer_enabled=True))
        m._select_batch(CargoBatch((1,), ("injured_orange",), "injury"))
        m.state = state
        m.safe_zone_view_pitch_cdeg = 11500
        m._update_safe_zone_visual_target((28, 9, 717, 140), 10)
        m.safe_zone_visual_locked = state != CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX
        command = m._safe_zone_visual_align_command()
        status = StmSnapshot(mode=11, flags=2, camera_pitch_cdeg=11500, age_ms=0,
            context_valid=True, context_matches=True, accepted_command=CMD_ALIGN_SAFE_ZONE, action_status=3)
        for frame in (11, 12):
            cached = VisionSnapshot(frame_sequence=frame, safe_bbox=(28, 9, 717, 140),
                safe_zone_filter_blocked=True,
                safe_corridor_polygon=((1,1),(2,1),(2,2)),
                safe_corridor_obstacle_class="danger_cyan")
            out = m.step(cached, pose(), status, frame * .1)
            assert out.state == state and out.reason == "safe_zone_visual_short_gap"
            assert out.command == command and out.command.source_vision_frame == 10
            assert m.safe_zone_visual_frame == 10 and m.obstacle_route is None
        if state == CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX:
            live = m.step(VisionSnapshot(frame_sequence=13, safe_bbox=(42,15,775,134)),
                          pose(), status, 1.3)
            assert live.state == state and live.command.source_vision_frame == 13
            assert live.command.arg_a == -81 and live.reason == "safe_zone_visual_align_wait"
        # Once the runner's original two-frame cache is gone, reuse full reobserve.
        lost = m.step(VisionSnapshot(frame_sequence=14, safe_bbox=None), pose(), status, 1.4)
        assert lost.state == CompetitionState.ACQUIRE_SAFE_ZONE
        assert lost.event == "safe_zone_visual_reobserve" and lost.command.source_vision_frame == 0


def test_platform_mixture_returns_to_existing_load_audit_before_enter():
    from protocol import CMD_ALIGN_SAFE_ZONE, CMD_CLEAR_SAFE_ZONE
    m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    m._select_batch(CargoBatch((239,), ("injured_orange",), "injury"))
    m.first_common_delivered = True
    m.carried_manifest = ("injured_orange",)
    m.state = CompetitionState.SAFE_ZONE_CORRIDOR_CHECK
    green = TrackedCargo(238, "green_supply", .8, (309,554,278,216), hits=6)
    view = VisionSnapshot(frame_sequence=3139, carried_incompatible_cargo=(green,))
    status = StmSnapshot(mode=11, flags=2, camera_pitch_cdeg=12000, age_ms=0)
    out = m.step(view, pose(), status, 1.)
    assert out.state == CompetitionState.TRANSPORT_AUDIT and out.command.opcode == CMD_CARGO_AUDIT
    assert out.command.audit_vision_frame == 0 and not out.command.audit.stable
    assert out.command.audit.total_count == 0 and out.command.opcode != CMD_CLEAR_SAFE_ZONE
    assert out.reason == "carried_incompatible_reaudit"
    assert m.carried_manifest == ("injured_orange",)  # Reobserve, don't invent an approved new manifest.
    m.state = CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX
    moving = replace(status, mode=10, context_valid=True, context_matches=True,
        accepted_command=CMD_ALIGN_SAFE_ZONE)
    assert m.step(view, pose(), moving, 1.01).reason == "carried_incompatible_reaudit"
    # A generic NAV mode10 is not proof that the MCU reached its ALIGN state.
    m.state = CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX
    still_staging = replace(moving, accepted_command=CMD_NAVIGATE_WAYPOINT, context_matches=False)
    assert m.step(view, pose(), still_staging, 1.02).state != CompetitionState.TRANSPORT_AUDIT


def test_preentry_locked_safe_zone_loss_reobserves_without_bypassing_obstacles():
    from protocol import CMD_ALIGN_SAFE_ZONE, CMD_PAUSE
    for state in (CompetitionState.SAFE_ZONE_CORRIDOR_CHECK,
                  CompetitionState.WAIT_SAFE_ZONE_CLEAR):
        m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
        batch = CargoBatch((1,), ("green_supply",), "material")
        m._select_batch(batch)
        m.state = state
        m.safe_zone_visual_locked = True
        m._update_safe_zone_visual_target((320, 0, 960, 113), 10)
        status = StmSnapshot(mode=11, flags=2, age_ms=0, camera_pitch_cdeg=12000)
        blocked = VisionSnapshot(frame_sequence=11, safe_bbox=(320, 0, 960, 113),
            safe_corridor_polygon=((201, 727), (1067, 727), (1067, 113), (201, 113)),
            safe_corridor_obstacle_class="danger_cyan")
        obstacle = m.step(blocked, pose(), status, 1.)
        assert obstacle.command.opcode == CMD_PAUSE
        assert m.state == CompetitionState.WAIT_SAFE_ZONE_CLEAR
        missing = m.step(replace(blocked, frame_sequence=12, safe_bbox=None,
            safe_corridor_polygon=()), pose(), status, 1.01)
        assert missing.event == "safe_zone_visual_reobserve"
        assert m.state == CompetitionState.ACQUIRE_SAFE_ZONE
        assert missing.command.opcode == CMD_ALIGN_SAFE_ZONE
        assert missing.command.arg_a == 0 and missing.command.arg_b == 11500
        assert m.selected_batch == batch and not m.safe_zone_visual_locked


def test_safe_zone_lateral_only_starts_for_large_error_and_finishes_without_zero_chasing():
    from protocol import CMD_ALIGN_SAFE_ZONE
    m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    m._select_batch(CargoBatch((1,), ("injured_orange",), "injury"))
    m.first_common_delivered = True
    m.state = CompetitionState.ALIGN_SAFE_ZONE_BY_LOCKED_BOX
    status = StmSnapshot(mode=11, flags=2, age_ms=0, camera_pitch_cdeg=12000,
        context_valid=True, context_matches=True, accepted_command=CMD_ALIGN_SAFE_ZONE, action_status=3)
    # Injury point = x + 936*2/3; x=80 gives +64 px. Small errors skip lateral movement.
    m._update_safe_zone_visual_target((80, 0, 936, 113), 10)
    assert m.safe_zone_visual_pixel_error == 64 and not m.safe_zone_lateral_active
    out = m.step(VisionSnapshot(frame_sequence=10, safe_bbox=(80,0,936,113)), pose(), status, 1.)
    assert out.state == CompetitionState.SAFE_ZONE_CORRIDOR_CHECK
    # Beyond 64 px activates one correction; it finishes inside 32 px, not at zero.
    m._update_safe_zone_visual_target((81, 0, 936, 113), 11)
    assert m.safe_zone_visual_pixel_error == 65 and m.safe_zone_lateral_active
    m._update_safe_zone_visual_target((66, 0, 936, 113), 12)
    assert m.safe_zone_visual_pixel_error == 50 and m.safe_zone_lateral_active
    m._update_safe_zone_visual_target((48, 0, 936, 113), 13)
    assert m.safe_zone_visual_pixel_error == 32 and not m.safe_zone_lateral_active
    m._update_safe_zone_visual_target((66, 0, 936, 113), 14)
    assert not m.safe_zone_lateral_active  # Do not restart for detection jitter in the small band.


def test_all_legacy_sweep_entries_keep_load_without_clear_or_blind_enter():
    from protocol import CMD_PAUSE, CMD_CLEAR_SAFE_ZONE, CMD_ENTER_SAFE_ZONE
    for side in ('red', 'blue'):
        for state in (CompetitionState.SAFE_ZONE_CORRIDOR_CHECK,
                      CompetitionState.ENTER_SAFE_ZONE, CompetitionState.CLEAR_SAFE_ZONE):
            for obstruction in ('green_supply', 'danger_cyan'):
                m = CompetitionMission(CompetitionSettings(side=side, initial_stash_enabled=False))
                m._select_batch(CargoBatch((1,), ('green_supply',), 'material'))
                m.carried_manifest = ('green_supply',)
                m.carried_total_count = 1
                m.state = state
                m.safe_corridor_frame_floor = 1
                view = VisionSnapshot(frame_sequence=2, safe_bbox=(340, 0, 900, 100),
                    safe_corridor_polygon=((460, 1023), (820, 1023), (600, 100), (200, 100)),
                    safe_corridor_obstacle_class=obstruction)
                out = m.step(view, PoseSnapshot(valid=True, yaw_deg=m.settings.safe_heading_deg),
                    StmSnapshot(mode=11, flags=2, age_ms=0, camera_pitch_cdeg=12000), 1.)
                assert out.command.opcode == CMD_PAUSE
                assert out.command.opcode not in (CMD_CLEAR_SAFE_ZONE, CMD_ENTER_SAFE_ZONE)
                assert out.reason == 'legacy_safe_sweep_disabled'
                assert m.state == CompetitionState.WAIT_SAFE_ZONE_CLEAR
                assert m.safe_sweep_attempts == 0 and m.safe_sweep_pickup is None
                assert m.carried_manifest == ('green_supply',) and m.carried_total_count == 1


def test_bulldozer_route_policy_geometry_and_three_centimetre_arrival():
    from obstacle_route import ObstacleRoute, RouteStep, handling_kind
    for destination, same, other in (("material", "green_supply", "injured_orange"),
                                     ("injury", "injured_orange", "green_supply")):
        assert handling_kind(destination, (same,), first=False) == "pickup"
        for classes in ((same,), (other,), ("danger_cyan",), (same, other)):
            assert handling_kind(destination, classes, first=True) == "sweep"
        assert handling_kind(destination, (same, "danger_cyan"), first=False) == "sweep"
    for side, sign in (("red", 1), ("blue", -1)):
        r = ObstacleRoute(side, (1,), ("green_supply",), "material", (-.15*sign, .54*sign),
                          (-.15*sign, .54*sign), ("danger_cyan",), "sweep")
        assert r.sweep_point == (-.56*sign, .91*sign)
        assert abs(1.5 - abs(r.sweep_point[1]) - .59) < 1e-9
        r.index = r.steps.index(RouteStep.LEFT_TO_SWEEP)
        x, y = r.sweep_point
        assert r.payload(x+.031, y, pose_valid=True)[0] > 0
        assert r.payload(x, y-.031, pose_valid=True)[0] > 0
        assert r.payload(x+.03, y-.03, pose_valid=True)[0] == 0
        assert r.payload(x+.08, y, pose_valid=True)[0] == 0  # D=0 stays latched for its completion.
        assert r.payload(x, y, pose_valid=False) is None


def _bulldozer_to_pickup(*, same=False, side="red"):
    from obstacle_route import LIVE_STEPS, RouteStep
    from protocol import CMD_ALIGN_SAFE_ZONE, CMD_BULLDOZER_STEP
    sign = 1 if side == "red" else -1
    m = CompetitionMission(CompetitionSettings(side=side, initial_stash_enabled=False,
                                               bulldozer_enabled=True))
    m.first_common_delivered = True
    m._select_batch(CargoBatch((1,), ("green_supply",), "material"))
    m.carried_manifest = ("green_supply",); m.carried_total_count = 1
    m.confirmed_delivery_destination = "material"
    p = PoseSnapshot(True, -.15*sign, .54*sign, 90 if sign == 1 else 270, 0)
    v = VisionSnapshot(frame_sequence=10, safe_corridor_obstacle_class=(
        "core_black" if same else "danger_cyan"))
    s = StmSnapshot(mode=11, flags=2, age_ms=0, camera_pitch_cdeg=12000,
                    context_valid=True, context_matches=True, action_status=3,
                    accepted_command=CMD_ALIGN_SAFE_ZONE)
    out = m._blocked_corridor_output(v, s, 1., pose=p)
    assert out.command.opcode == CMD_BULLDOZER_STEP
    steps = []
    for i in range(20):
        r = m.obstacle_route; step = r.step; steps.append(step)
        if step in LIVE_STEPS:
            target = r.sweep_point if step in {RouteStep.BACK_TO_SWEEP, RouteStep.LEFT_TO_SWEEP} else r.return_point
            p = replace(p, x_m=target[0], y_m=target[1])
        v = replace(v, frame_sequence=11+i)
        s = replace(s, mode=25, accepted_command=CMD_BULLDOZER_STEP)
        out = m.step(v, p, s, 1.1+i*.1)
        if step == RouteStep.RELEASE_LOAD:
            assert m.selected_batch is None and not m.carried_manifest and m.carried_total_count == 0
            assert r.original_classes == ("green_supply",) and r.original_ids == (1,)
        if m.state == CompetitionState.OBSTACLE_PICKUP:
            break
    assert m.state == CompetitionState.OBSTACLE_PICKUP
    assert RouteStep.SWEEP_1000 in steps if not same else RouteStep.SWEEP_1000 not in steps
    if same:
        assert steps == [RouteStep.TURN_RIGHT, RouteStep.FORWARD_PARK, RouteStep.RELEASE_LOAD,
                         RouteStep.BACK_200, RouteStep.TURN_LEFT, RouteStep.PICKUP_READY]
    return m, out, p, replace(s, mode=3, flags=0), v


def test_bulldozer_route_keeps_stash_and_abandons_a_missing_recovery_target():
    for side in ("red", "blue"):
        m, _, p, s, v = _bulldozer_to_pickup(side=side)
        out = m.step(replace(v, frame_sequence=v.frame_sequence+1, cargo=()), p, s, 4.)
        assert out.event == "bulldozer_pickup_abandoned"
        assert m.state == CompetitionState.SEARCH and m.obstacle_route is None
        assert out.command.opcode == CMD_HOLD and m.selected_batch is None
    m, _, _, _, _ = _bulldozer_to_pickup()
    m._set_state(CompetitionState.SEARCH, 5.)
    assert m.obstacle_route is None  # An ordinary recovery/abandon cannot resurrect it.


def test_bulldozer_same_zone_pickup_uses_normal_whole_claw_audit_and_returns_for_stash():
    from protocol import CMD_BULLDOZER_STEP, CMD_APPROACH_TARGET
    m, _, p, s, v = _bulldozer_to_pickup(same=True)
    target = TrackedCargo(99, "core_black", .9, (600, 500, 80, 100), hits=4)
    out = m.step(replace(v, frame_sequence=v.frame_sequence+1, cargo=(target,)), p, s, 4.)
    assert out.state == CompetitionState.APPROACH and out.command.opcode == CMD_APPROACH_TARGET
    assert m.obstacle_route.awaiting_obstacle_delivery
    assert m.selected_batch.track_ids == (99,) and not m.carried_manifest
    # The ordinary full-claw audit, including a neighbouring blue, still owns legality.
    audit = CargoAudit(left_class="core_black", right_class="danger_cyan", left_count=1,
                       right_count=1, total_count=2, danger_present=True)
    for frame, now in ((30, 4.1), (31, 4.25), (32, 4.4)):
        audited = m.step(VisionSnapshot(frame_sequence=frame, capture_audit=audit,
            capture_observation_valid=True), p, replace(s, mode=21, flags=1,
            camera_pitch_cdeg=14000), now)
    assert audited.command.opcode == CMD_CARGO_AUDIT
    assert audited.command.audit.total_count == 2 and audited.command.audit.danger_present
    # Successful obstacle delivery schedules the one pending original load.
    m.carried_manifest = ("core_black",); m.carried_total_count = 1
    m._finish_delivery(s, 5., basis="fixture", message="fixture", event="fixture")
    out = m.step(VisionSnapshot(frame_sequence=40), p, replace(s, mode=17), 5.1)
    assert out.state == CompetitionState.RETURN_CENTER
    at_stage = replace(p, x_m=m.obstacle_route.return_point[0], y_m=m.obstacle_route.return_point[1])
    done = replace(s, accepted_command=CMD_RETURN_CENTER, action_status=3)
    out = m.step(VisionSnapshot(frame_sequence=41), at_stage, done, 5.2)
    assert out.state == CompetitionState.OBSTACLE_ROUTE and out.command.opcode == CMD_BULLDOZER_STEP
    assert m.obstacle_route.pickup_role == "original"


def test_blue_with_recoverable_cargo_uses_existing_disperse_without_rearming():
    for side in ('left','right'):
        m=CompetitionMission(CompetitionSettings(side='red',initial_stash_enabled=False))
        m._select_batch(CargoBatch((1,),('green_supply',),'material'))
        a=CargoAudit(left_class='green_supply' if side=='left' else 'danger_cyan',
            right_class='danger_cyan' if side=='left' else 'green_supply',
            left_count=1,right_count=1,total_count=2,danger_present=True,
            left_green_count=int(side=='left'),right_green_count=int(side=='right'),stable=True)
        s=StmSnapshot(mode=21,flags=1,age_ms=0,camera_pitch_cdeg=14000)
        out=m._start_danger_release(a,s,1.)
        assert out.command.opcode==CMD_DISPERSE_PILE and out.command.flags&64
        assert bool(out.command.flags&128)==(side=='right')
        assert not m._audit_valid(a)
        epoch=m.separation_request_epoch
        waiting=m._disperse_output(VisionSnapshot(frame_sequence=10,capture_audit=a),s,1.1)
        assert waiting.command==out.command and m.separation_request_epoch==epoch
    m=CompetitionMission(CompetitionSettings(side='red',initial_stash_enabled=False))
    m._select_batch(CargoBatch((1,),('green_supply',),'material'))
    same=CargoAudit(left_class='unknown',left_count=2,right_count=0,total_count=2,
        left_green_count=1,danger_present=True,left_invalid=True,
        left_class_counts=(('green_supply',1),('danger_cyan',1)),
        class_counts=(('green_supply',1),('danger_cyan',1)),stable=True)
    assert m._start_danger_release(same,s,2.).command.opcode==CMD_DISPERSE_PILE
    only=CargoAudit(left_class='danger_cyan',left_count=1,total_count=1,danger_present=True,stable=True)
    assert not m._side_is_disperse_candidate(only,'left')  # No useful cargo to retain.


def test_single_anchor_audits_actual_load_before_grab_and_separates_overload():
    from protocol import CMD_CARGO_AUDIT, CMD_DISPERSE_PILE, CMD_GRAB_CONFIRMED, CMD_NAVIGATE_WAYPOINT
    for danger in (False, True):
        m = CompetitionMission(CompetitionSettings(side='red', initial_stash_enabled=False))
        m._select_batch(CargoBatch((1,), ('green_supply',), 'material'))
        m.locked_target_track_id = 1
        m.state = CompetitionState.APPROACH
        audit = (CargoAudit(left_class='danger_cyan', right_class='green_supply',
                           left_count=1, right_count=1, right_green_count=1, total_count=2,
                           danger_present=True) if danger else
                 CargoAudit(left_class='green_supply', right_class='green_supply',
                           left_count=2, right_count=1, left_green_count=2,
                           right_green_count=1, total_count=3))
        status = StmSnapshot(mode=21, flags=1, age_ms=0, camera_pitch_cdeg=14000)
        for frame in range(1, 5):
            now = 1.0 + frame * .15
            # Anchor no longer visible: the MCU's real claw view still wins over tracking.
            view = VisionSnapshot(frame_sequence=frame, observed_monotonic_s=now,
                                  capture_audit=audit, capture_observation_valid=True)
            out = m.step(view, PoseSnapshot(valid=True), status, now)
            assert out.command.opcode not in (CMD_GRAB_CONFIRMED, CMD_NAVIGATE_WAYPOINT)
        assert out.command.opcode == CMD_CARGO_AUDIT
        assert out.command.audit.total_count == audit.total_count and out.command.audit.stable
        ready = replace(status, flags=1|64, acknowledged_sequence=1, relay_mission_tx_frames=2,
            relay_last_mission_command=CMD_CARGO_AUDIT,
            relay_last_mission_payload=tuple(out.command.to_frame(0)[4:12]))
        decided = m.step(replace(view, frame_sequence=5, observed_monotonic_s=1.8),
                         PoseSnapshot(valid=True), ready, 1.8)
        assert decided.command.opcode == CMD_DISPERSE_PILE
        assert decided.command.opcode not in (CMD_GRAB_CONFIRMED, CMD_NAVIGATE_WAYPOINT)


def test_blue_separation_after_closed_and_transport_audit_keeps_original_handoff():
    audit = CargoAudit(left_class="danger_cyan", right_class="green_supply",
        left_count=1, right_count=1, right_green_count=1, total_count=2,
        danger_present=True)
    for mode in (23, 48):
        m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
        m._select_batch(CargoBatch((1,), ("green_supply",), "material"))
        status = replace(stm(mode=mode, flags=3), camera_pitch_cdeg=14000)
        if mode == 23:
            m.post_grab_audit_active = True
            m._set_state(CompetitionState.POST_GRAB_AUDIT, 1.)
        else:
            m._begin_transport_audit(VisionSnapshot(frame_sequence=1), pose(), status, 1.)
        for frame in (2, 3, 4):
            view = VisionSnapshot(frame_sequence=frame, observed_monotonic_s=1.+frame*.15,
                capture_audit=audit, capture_observation_valid=True)
            out = m.step(view, pose(), status, 1.+frame*.15)
            assert out.command.opcode == CMD_CARGO_AUDIT  # No separation before the existing ACK.
        accepted = replace(relay_stm(out, mode=mode, flags=67), camera_pitch_cdeg=14000,
            context_valid=True, context_matches=True, accepted_command=CMD_CARGO_AUDIT,
            action_status=1)
        out = m.step(replace(view, frame_sequence=5, observed_monotonic_s=1.8),
            pose(), accepted, 1.8)
        assert out.command.opcode == CMD_DISPERSE_PILE and out.command.flags & CMD_TARGET_RIGHT
        assert not m.post_grab_audit_active and not m._audit_valid(audit)
        if mode == 48:
            assert m.transport_separation_active and not m.transport_audit_handoff_active
            assert m.transport_audit_command is None and m.transport_audit_value is None


def test_green_search_prefers_near_pile_and_preserves_class_priority():
    near = TrackedCargo(1, "green_supply", .9, (600, 650, 100, 100), hits=5)
    neighbour = TrackedCargo(2, "core_black", .9, (710, 650, 100, 100), hits=5)
    far = TrackedCargo(3, "green_supply", .9, (100, 100, 180, 180), hits=5)
    view = VisionSnapshot(cargo=(far, near, neighbour), range_pitch_cdeg=12000,
                          range_distances=((near.track_id, 100), (far.track_id, 460)))
    m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    assert m._is_isolated(far, view.cargo) and not m._is_isolated(near, view.cargo)
    assert m._choose_initial_green(view) is None  # Nearest green uses the existing pile path.
    assert m._choose_cluster_target(view).track_id == near.track_id
    for first in (False, True):
        m.first_common_delivered = first
        assert m._choose_material_batch(view).track_ids == (near.track_id,)
    assert m._choose_near_green(replace(view, range_distances=()), (far, near)) == near
    # A measured/finite ground distance wins over pixel order when all have it.
    measured_near = replace(far, relative_xy_m=(.2, 0))
    measured_far = replace(near, relative_xy_m=(1., 0))
    assert m._choose_near_green(view, (measured_far, measured_near)) == measured_near
    # The existing band estimate wins even if an unusually large box has a lower foot.
    assert m._choose_near_green(replace(view, range_distances=((1, 460), (3, 100))),
                                (far, near)) == far
    # Do not change the post-first green-vs-black/casualty policy.
    casualty = TrackedCargo(4, "injured_orange", .9, (800, 850, 100, 100), (.05, 0), hits=5)
    assert m._choose_next_batch(replace(view, cargo=(far, neighbour, casualty))).classes == ("green_supply",)


def test_green_anchor_keeps_identity_then_reselects_after_existing_recovery():
    near = TrackedCargo(1, "green_supply", .9, (600, 650, 100, 100), hits=5)
    far = TrackedCargo(3, "green_supply", .9, (100, 100, 60, 60), hits=5)
    m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
    m._select_batch(CargoBatch((far.track_id,), (far.class_name,), "material"))
    m.state = CompetitionState.APPROACH
    m._mark_target_seen(far, 1.)
    m.last_approach_command = m._approach_command(far)
    assert m._target_for_batch(VisionSnapshot(cargo=(near, far))) == far
    changed_id = replace(far, track_id=30, bbox=(105, 102, 60, 60))
    assert m._target_for_batch(VisionSnapshot(cargo=(near, changed_id))) == changed_id
    assert m._target_for_batch(VisionSnapshot(cargo=(near,))) is None
    window = m._approach_missing_hold_window_s()
    brief = VisionSnapshot(frame_sequence=10, observed_monotonic_s=1.+window*.5, cargo=(near,))
    out = m._approach_output(brief, stm(mode=20), 1.+window*.5, message="fixture")
    assert out.command == m.last_approach_command and m.locked_target_track_id == far.track_id
    now = 1.+window+.01
    lost = replace(brief, frame_sequence=11, observed_monotonic_s=now)
    out = m._approach_output(lost, stm(mode=20), now, message="fixture")
    assert out.event == "green_anchor_lost_reselect" and out.command.opcode == CMD_HOLD
    assert m.state == CompetitionState.WAIT_SEARCH_RECOVERY and m.selected_batch is None
    m.step(lost, pose(), stm(mode=24), now+.1)
    m.step(lost, pose(), stm(mode=3), now+.2)
    out = m.step(replace(lost, frame_sequence=12, observed_monotonic_s=now+.3), pose(), stm(mode=3), now+.3)
    assert out.state == CompetitionState.APPROACH and m.locked_target_track_id == near.track_id


def test_cluster_green_loss_and_range_wait_do_not_borrow_a_far_anchor():
    for in_range_wait in (False, True):
        m = CompetitionMission(CompetitionSettings(side="red", initial_stash_enabled=False))
        anchor = TrackedCargo(1, "green_supply", .9, (580, 400, 120, 120), hits=5)
        neighbour = replace(anchor, track_id=2, bbox=(690, 400, 120, 120), class_name="core_black")
        first = VisionSnapshot(frame_sequence=1, observed_monotonic_s=1., cargo=(anchor, neighbour))
        m._start_cluster_approach(first, stm(), 1.)
        far = replace(anchor, track_id=3, bbox=(100, 100, 60, 60))
        assert m._cluster_target_for_vision(replace(first, cargo=(far,))) is None
        # An ambiguous pair near the old box is not a proven continuation either.
        ambiguous = (replace(anchor, track_id=4, bbox=(570,400,120,120)),
                     replace(anchor, track_id=5, bbox=(590,400,120,120)))
        assert m._cluster_target_for_vision(replace(first, cargo=ambiguous)) is None
        status = replace(stm(mode=52 if in_range_wait else 20), camera_pitch_cdeg=12000)
        if in_range_wait:
            m._range_approach_output(replace(first, range_pitch_cdeg=12000), status, 1.)
        now = 1.+m._approach_missing_hold_window_s()+.01
        missing = replace(first, frame_sequence=2, observed_monotonic_s=now,
                          cargo=(far,), range_pitch_cdeg=12000)
        out = (m._range_approach_output(missing, status, now) if in_range_wait else
               m._cluster_approach_output(missing, status, now, pose=pose()))
        assert out.event == "green_anchor_lost_reselect" and out.command.opcode == CMD_HOLD
        assert m.cluster_target_track_id is None and m.selected_batch is None


def test_attack_return_opponent_frame_geometry_for_all_start_sides():
    import math
    from attack_return import AttackReturnRoute
    for side, sign in (("red",1),("blue",-1)):
        for zone in (1,2,3,4):
            for yaw in (0., 135., 315.709, 90., 270.):
                cfg=CompetitionSettings(side=side,start_zone=zone)
                origin=cfg.stash_point
                r=AttackReturnRoute(side,zone,origin,yaw,cfg.post_delivery_return_point)
                angle=math.radians(270 if side=="red" else 90)
                f=(math.cos(angle),math.sin(angle));left=(-f[1],f[0])
                assert r.release_yaw_deg == yaw
                assert r.return_yaw_deg == (270 if side == "red" else 90)
                assert r.payload(*origin)[2] == round(r.return_yaw_deg * 100)
                assert r.start_same_side == ((zone<=2)==(side=="red"))
                assert r.waypoints[-1] == (0.,.30*sign)
                assert math.dist(origin,r.waypoints[0]) == .1 or math.isclose(math.dist(origin,r.waypoints[0]),.1)
                assert math.isclose(math.dist(r.waypoints[0],r.waypoints[1]),.5)
                for i in (0,1):
                    delta=tuple(a-b for a,b in zip(r.waypoints[i],origin if i==0 else r.waypoints[0]))
                    axis=tuple(-v for v in f) if i==0 else left
                    assert math.isclose(sum(a*b for a,b in zip(delta,axis)),(.1,.5)[i])
                back=tuple(a-b for a,b in zip(r.waypoints[2],r.waypoints[1]))
                right=tuple(a-b for a,b in zip(r.waypoints[3],r.waypoints[2]))
                assert sum(a*b for a,b in zip(back,f)) < 0
                assert sum(a*b for a,b in zip(right,left)) < 0
                assert abs(sum(a*b for a,b in zip(back,left))) < 1e-10
                assert abs(sum(a*b for a,b in zip(right,f))) < 1e-10
                # D=0 remains latched after the permitted 5cm arrival.
                target=r.waypoints[0]
                assert r.payload(target[0]+.05,target[1])[0]==0
                assert r.payload(target[0]+.20,target[1])[0]==0


def test_attack_faces_opponent_before_release_and_does_not_adopt_old_done():
    from protocol import CMD_BULLDOZER_STEP
    from obstacle_route import RouteStep
    for side in ("red", "blue"):
        for zone in (1, 2, 3, 4):
            cfg = CompetitionSettings(side=side, start_zone=zone)
            m = CompetitionMission(cfg)
            m._select_batch(CargoBatch((1,), ("green_supply",), "stash", initial_stash=True))
            m._set_state(CompetitionState.INITIAL_STASH_NAV, 1.)
            m.navigation_initial_ack = 1
            p = PoseSnapshot(True, *cfg.stash_point, 315.709, 0.)
            nav_done = StmSnapshot(mode=10, age_ms=0, flags=114, acknowledged_sequence=2,
                context_valid=True, context_matches=True,
                accepted_command=CMD_NAVIGATE_WAYPOINT, action_status=3)
            running = replace(nav_done, flags=82, action_status=1)
            finish = m.step(VisionSnapshot(), p, running, 1.8)
            assert finish.state == CompetitionState.INITIAL_STASH_NAV
            assert finish.command.opcode == CMD_NAVIGATE_WAYPOINT and finish.command.arg_a == 0
            jitter = replace(p, x_m=p.x_m + .08)
            assert m.step(VisionSnapshot(), jitter, running, 1.9).command.arg_a == 0
            out = m.step(VisionSnapshot(), p, nav_done, 2.)
            assert out.state == CompetitionState.INITIAL_ATTACK_ALIGN
            assert out.command.opcode == CMD_BULLDOZER_STEP
            assert out.command.flags >> 4 == RouteStep.ATTACK_FACE_OPPONENT
            assert out.command.aux == (27000 if side == "red" else 9000)
            out.command.to_frame(3)
            assert m.step(VisionSnapshot(), p, nav_done, 2.1).state == CompetitionState.INITIAL_ATTACK_ALIGN
            turn = replace(nav_done, mode=25, accepted_command=CMD_BULLDOZER_STEP)
            assert m.step(VisionSnapshot(), p, replace(turn, context_matches=False), 2.2).state == CompetitionState.INITIAL_ATTACK_ALIGN
            assert m.step(VisionSnapshot(), p, replace(turn, action_status=1), 2.3).state == CompetitionState.INITIAL_ATTACK_ALIGN
            out = m.step(VisionSnapshot(), p, turn, 2.4)
            assert out.state == CompetitionState.INITIAL_RELEASE and out.command.opcode == CMD_RELEASE_BOTH
            assert m.selected_batch.initial_stash and not m.initial_stash_done
            # Turning DONE cannot complete the newly allocated release action.
            assert m.step(VisionSnapshot(), p, turn, 2.5).state == CompetitionState.INITIAL_RELEASE
            release = replace(turn, mode=34, accepted_command=CMD_RELEASE_BOTH, flags=0)
            out = m.step(VisionSnapshot(), p, release, 2.6)
            assert out.state == CompetitionState.INITIAL_ATTACK_RETURN
            assert out.command.aux == (27000 if side == "red" else 9000)


def test_opening_attack_return_requires_paired_done_for_each_leg_and_search_ready():
    from protocol import CMD_BULLDOZER_STEP
    from obstacle_route import RouteStep
    for side, sign in (("red",1),("blue",-1)):
        m=CompetitionMission(CompetitionSettings(side=side))
        m._select_batch(CargoBatch((1,),('green_supply',),'stash',initial_stash=True))
        m._set_state(CompetitionState.INITIAL_RELEASE,1.)
        p=PoseSnapshot(True,.15*sign,-.69*sign,270 if sign==1 else 90,0)
        release=StmSnapshot(mode=34,flags=0,age_ms=0,context_valid=True,context_matches=True,
                            accepted_command=CMD_RELEASE_BOTH,action_status=3)
        out=m.step(VisionSnapshot(frame_sequence=10),p,release,1.)
        assert out.state==CompetitionState.INITIAL_ATTACK_RETURN and out.command.opcode==CMD_BULLDOZER_STEP
        assert out.command.flags>>4==RouteStep.EMPTY_BODY_MOVE and out.command.arg_a==100
        assert out.command.arg_b==(9000 if sign==1 else -9000)
        assert m.initial_stash_done and m.stash_has_cargo and not m.first_common_delivered
        old=replace(release,mode=25,accepted_command=CMD_BULLDOZER_STEP,context_matches=False)
        for index in range(4):
            target=m.attack_return_route.waypoints[index]
            at=replace(p,x_m=target[0],y_m=target[1])
            out=m.step(VisionSnapshot(frame_sequence=11+index),at,old,2.+index)
            assert m.attack_return_route.index==index and out.command.arg_a==0
            out=m.step(VisionSnapshot(frame_sequence=11+index),at,replace(old,context_matches=True),2.1+index)
            assert m.attack_return_route.index==index+1
        assert out.command.flags>>4==RouteStep.PICKUP_READY
        # A completed movement is not completion of the new SEARCH-ready action.
        out=m.step(VisionSnapshot(frame_sequence=20),at,replace(old,context_matches=True),7.)
        assert m.state==CompetitionState.INITIAL_ATTACK_RETURN
        out=m.step(VisionSnapshot(frame_sequence=21),at,replace(old,context_matches=True,mode=3),7.1)
        assert m.state==CompetitionState.SEARCH and out.command.opcode==CMD_HOLD
        assert m.return_search_frame_floor==21 and m.attack_return_route is None
    defense=CompetitionMission(CompetitionSettings(side='red',opening_strategy='defense'))
    defense._select_batch(CargoBatch((1,),('green_supply',),'stash',initial_stash=True))
    defense._set_state(CompetitionState.INITIAL_RELEASE,1.)
    out=defense.step(VisionSnapshot(),pose(),release,1.)
    assert out.state==CompetitionState.RETURN_CENTER and out.command.opcode==CMD_RETURN_CENTER


def test_two_search_stations_alternate_via_side_lane_without_returning_center():
    from protocol import CMD_BULLDOZER_STEP
    from obstacle_route import RouteStep
    for side, own in [('red',1),('blue',-1)]:
        m=CompetitionMission(CompetitionSettings(side=side,initial_stash_enabled=False))
        m.initial_stash_done=True
        m._set_state(CompetitionState.SEARCH,1.)
        p=PoseSnapshot(True,0.,.30*own,135.,0)
        v=VisionSnapshot(frame_sequence=10)
        idle=StmSnapshot(mode=3,flags=0,age_ms=0,context_valid=True,context_matches=False)
        for cycle in range(2):
            m.return_search_mode_seen=False;m.search_yaw_accum_deg=720.
            out=m.step(v,p,idle,2.+cycle*5)
            assert out.state==CompetitionState.SEARCH_TRANSFER and out.command.opcode==CMD_BULLDOZER_STEP
            r=m.search_transfer_route
            expected=-own if cycle==0 else own
            assert r.target_sign==expected and r.waypoints[-1]==(0.,.30*expected)
            assert abs(r.lane_x_m)==.5 and r.waypoints[0][1]==p.y_m
            assert r.waypoints[1][0]==r.waypoints[0][0]
            original_yaw=out.command.aux
            done=replace(idle,mode=25,accepted_command=CMD_BULLDOZER_STEP,action_status=3)
            for index in range(3):
                p=replace(p,x_m=r.waypoints[index][0],y_m=r.waypoints[index][1])
                out=m.step(v,p,done,2.1+cycle*5+index)
                assert r.index==index and out.command.arg_a==0
                out=m.step(v,p,replace(done,context_matches=True),2.2+cycle*5+index)
                assert r.index==index+1
                if index<2:assert out.command.aux==original_yaw
            assert out.command.flags>>4==RouteStep.PICKUP_READY
            out=m.step(v,p,replace(done,mode=3,context_matches=True),6.+cycle*5)
            assert m.state==CompetitionState.SEARCH and m.search_point_sign==expected
            assert m.search_visit==cycle+1 and m.search_site_turn_deg==0
        # A legacy/disabled opening can start at centre: establish the own
        # point before claiming that the own station has been searched.
        initial=CompetitionMission(CompetitionSettings(side=side,initial_stash_enabled=False))
        initial._set_state(CompetitionState.SEARCH,1.);initial.search_yaw_accum_deg=720.
        initial.step(v,replace(p,y_m=0.),idle,2.)
        assert initial.search_transfer_route.target_sign==own


def test_search_transfer_target_seen_stops_owned_move_then_uses_normal_capture():
    from protocol import CMD_BULLDOZER_STEP
    from obstacle_route import RouteStep
    m=CompetitionMission(CompetitionSettings(side='red',initial_stash_enabled=False))
    m.initial_stash_done=True;m._set_state(CompetitionState.SEARCH,1.)
    p=PoseSnapshot(True,0.,.3,270.,0)
    status=StmSnapshot(mode=3,age_ms=0,context_valid=True,context_matches=False)
    m.search_yaw_accum_deg=720.
    m.step(VisionSnapshot(frame_sequence=1),p,status,1.)
    blue=cargo(7,'danger_cyan',600,500)
    moving=replace(status,mode=25,accepted_command=CMD_BULLDOZER_STEP,action_status=1)
    out=m.step(VisionSnapshot(frame_sequence=2,cargo=(blue,)),p,moving,1.1)
    assert not m.search_transfer_route.interrupted and out.command.arg_a>0
    green=cargo(8,'green_supply',620,550)
    out=m.step(VisionSnapshot(frame_sequence=3,cargo=(green,)),p,moving,1.2)
    assert out.event=='search_transfer_target_seen' and out.command.arg_a==0
    assert m.search_transfer_route.index==0
    out=m.step(VisionSnapshot(frame_sequence=4,cargo=(green,)),p,
        replace(moving,context_matches=True,action_status=3),1.3)
    assert m.search_transfer_route.search_ready and out.command.flags>>4==RouteStep.PICKUP_READY
    m.step(VisionSnapshot(frame_sequence=5,cargo=(green,)),p,
        replace(moving,context_matches=True,action_status=3,mode=3),1.4)
    assert m.state==CompetitionState.SEARCH and m.search_point_sign==1 and m.search_visit==0
    out=m.step(VisionSnapshot(frame_sequence=6,cargo=(green,)),p,
        replace(status,camera_pitch_cdeg=12000),1.5)
    assert m.state==CompetitionState.APPROACH and out.command.opcode==CMD_APPROACH_TARGET
    assert m.selected_batch.track_ids==(8,)


def test_failed_target_context_survives_recovery_without_hiding_claw_audit():
    m=CompetitionMission(CompetitionSettings(side='red',initial_stash_enabled=False))
    m.initial_stash_done=True
    target=cargo(9,'green_supply',600,500)
    p=PoseSnapshot(True,0.,.3,270.,0)
    s=StmSnapshot(mode=20,age_ms=0,camera_pitch_cdeg=12000)
    m.search_observer=(0.,.3,270.,12000)
    m._select_batch(CargoBatch((9,),('green_supply',),'material'))
    m._mark_target_seen(target,1.);m.state=CompetitionState.APPROACH
    m.target_last_observer=(0.,.3,270.,13000)  # Retain the original SEARCH120 view too.
    m.search_site_turn_deg=400.
    m._begin_search_recovery(VisionSnapshot(frame_sequence=10),1.5)
    assert len(m.failed_search_targets)==1
    m.step(VisionSnapshot(frame_sequence=11),p,replace(s,mode=3),1.6)
    assert m.search_site_turn_deg==400. and m.search_yaw_accum_deg==0.
    audit=CargoAudit(left_class='green_supply',right_class='danger_cyan',left_count=1,right_count=1,total_count=2,danger_present=True)
    replacement=replace(target,track_id=90,bbox=(608,500,40,40))
    v=VisionSnapshot(cargo=(target,replacement),capture_cargo=(target,),capture_audit=audit)
    filtered=m._search_vision_without_failures(v)
    assert filtered.cargo==() and filtered.capture_cargo==v.capture_cargo and filtered.capture_audit is audit
    m.search_observer=(0.,.3,290.,12000)
    assert m._search_vision_without_failures(v).cargo==(replacement,)
    m.search_observer=(0.,.3,270.,10500)
    assert m._search_vision_without_failures(v).cargo==(replacement,)
    # A new target selection does not erase the old failure.
    m._select_batch(CargoBatch((30,),('green_supply',),'material'))
    assert len(m.failed_search_targets)==1
    # At a newly reached station, previous failures are retained as history
    # but permitted a new view rather than becoming a permanent blacklist.
    m.search_visit+=1
    assert m._search_vision_without_failures(v).cargo==v.cargo
    # Cumulative SEARCH motion, rather than repeated recovery resets, ends a visit.
    m.search_site_turn_deg=720.;m.search_yaw_accum_deg=0.
    out=m.step(VisionSnapshot(frame_sequence=12),p,replace(s,mode=3),2.)
    assert out.state==CompetitionState.SEARCH_TRANSFER


def test_cluster_range_rechecks_real_anchor_without_dropping_cluster_identity():
    from protocol import CMD_CLUSTER_TARGET, CMD_APPROACH_RANGE
    m=CompetitionMission(CompetitionSettings(side='red',initial_stash_enabled=False))
    m._select_batch(CargoBatch((98,),('green_supply',),'material'));m.state=CompetitionState.CLUSTER_APPROACH
    m.cluster_target_track_id=98;m.cluster_target_class='green_supply';m.cluster_target_bbox=(653,142,122,75)
    m.range_started_s=31.415;m.range_frame_floor=1980;m.range_pitch_cdeg=12000
    s=StmSnapshot(mode=52,age_ms=0,camera_pitch_cdeg=12000)
    item=TrackedCargo(98,'green_supply',.8,(653,142,122,75),hits=10)
    view=VisionSnapshot(frame_sequence=1989,observed_monotonic_s=31.55,cargo=(item,),
                        range_pitch_cdeg=12000,range_distances=((98,380),))
    out=m._range_approach_output(view,s,31.55)
    assert out.reason=='approach_range_realign' and out.command.flags&CMD_CLUSTER_TARGET
    assert not out.command.flags&CMD_APPROACH_RANGE and out.command.arg_a==714
    assert m.cluster_target_track_id==98 and m.range_command is None
    for x in (576,640,704):
        item=replace(item,bbox=(x-61,142,122,75))
        out=m._range_approach_output(replace(view,frame_sequence=1990,cargo=(item,),observed_monotonic_s=31.6),s,31.6)
        assert out.command.flags&CMD_APPROACH_RANGE and out.command.flags&CMD_CLUSTER_TARGET
        assert out.command.arg_a==380
        m.range_command=None


def test_near_claw_keep_audits_real_blue_only_before_requesting_soft_separation():
    from protocol import CMD_DISPERSE_NEAR_KEEP,CMD_SIDE_VALID
    for cluster,mode in ((False,21),(True,38)):
        m=CompetitionMission(CompetitionSettings(side='red',initial_stash_enabled=False))
        m._select_batch(CargoBatch((836,),('green_supply',),'material'))
        m.cluster_audit_active=cluster;m.state=CompetitionState.CAPTURE_AUDIT
        status=StmSnapshot(mode=mode,flags=1,age_ms=0,camera_pitch_cdeg=14000)
        audit=CargoAudit(right_class='danger_cyan',right_count=1,total_count=1,danger_present=True,
                         near_claw_keep_side='left',near_claw_green_id=836,near_claw_frame=14113)
        for frame in (1,2,3,4):
            now=1.+frame*.15
            view=VisionSnapshot(frame_sequence=frame,observed_monotonic_s=now,capture_audit=audit)
            out=m.step(view,pose(),status,now)
        assert out.command.opcode==CMD_CARGO_AUDIT and out.command.audit.total_count==1
        assert out.command.audit.right_class=='danger_cyan' and out.command.audit.left_count==0
        assert not m._audit_valid(audit)
        paired=replace(relay_stm(out,mode=37 if cluster else 21,flags=65),camera_pitch_cdeg=14000,
                       context_valid=True,context_matches=True,accepted_command=CMD_CARGO_AUDIT,action_status=3)
        out=m.step(replace(view,frame_sequence=5,observed_monotonic_s=1.8),pose(),paired,1.8)
        assert out.command.opcode==CMD_DISPERSE_PILE
        assert out.command.flags&CMD_DISPERSE_NEAR_KEEP and out.command.flags&CMD_SIDE_VALID
        assert not out.command.flags&CMD_TARGET_RIGHT and m.cluster_keep_side_count==0
        assert out.command.source_vision_frame==14113
        epoch=m.separation_request_epoch
        again=m._disperse_output(view,status,1.9)
        assert again.command==out.command and m.separation_request_epoch==epoch
        # Side evidence cannot survive a changed audit as transport permission.
        assert m._audit_signature_for_state(audit,False)!=m._audit_signature_for_state(
            replace(audit,near_claw_keep_side=None),False)
