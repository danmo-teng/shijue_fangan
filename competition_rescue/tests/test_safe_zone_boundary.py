"""Pickup boundary and the existing cancellation / empty-sweep handoffs."""
import sys
from pathlib import Path
from dataclasses import replace
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from safe_zone_boundary import crossed_safe_zone
from state_machine import (CompetitionMission, CompetitionSettings, CompetitionState,
                           CargoBatch, CargoAudit, PoseSnapshot, StmSnapshot, VisionSnapshot, TrackedCargo)
from protocol import CMD_RELEASE_BOTH, CMD_APPROACH_TARGET, CMD_BULLDOZER_STEP
from obstacle_route import RouteStep, LIVE_STEPS


def mission(side='red', name='green_supply'):
    m = CompetitionMission(CompetitionSettings(side=side, initial_stash_enabled=False))
    m.first_common_delivered = True
    m._select_batch(CargoBatch((1,), (name,), 'injury' if name == 'injured_orange' else 'material'))
    m.state = CompetitionState.APPROACH
    return m


def status(mode=52, **kwargs):
    return StmSnapshot(mode=mode, age_ms=0, camera_pitch_cdeg=12000, **kwargs)


def ranged(m, y, side_sign=1):
    m.range_started_s = 1.
    m.range_frame_floor = 1
    m.range_pitch_cdeg = 12000
    item = TrackedCargo(1, 'green_supply', .9, (600,300,80,80), hits=4)
    v = VisionSnapshot(frame_sequence=2, observed_monotonic_s=1.2, cargo=(item,),
                       range_pitch_cdeg=12000, range_distances=((1,230),))
    p = PoseSnapshot(True, 0, side_sign*y, 90 if side_sign==1 else 270, 0)
    return m._range_approach_output(v, status(), 1.2, p), v, p


def test_twenty_centimetres_has_no_rectangular_corner_or_twenty_seven_margin():
    for sign, zone in ((1,'red'),(-1,'blue')):
        assert crossed_safe_zone((0,sign*.7),(0,sign*.95)) == zone
        assert crossed_safe_zone((0,sign*.7),(0,sign*.939)) is None
        assert crossed_safe_zone((.55,sign*1.),(.55,sign*1.4)) is None
        assert crossed_safe_zone((.49,sign*.98),(.49,sign*.98)) is None
        assert crossed_safe_zone((.53,sign*1.2),(.53,sign*1.2)) == zone


def test_range_crossing_cancels_both_sides_and_only_own_side_sweeps():
    for team in ('red','blue'):
        for sign, zone in ((1,'red'),(-1,'blue')):
            m = mission(team)
            out, v, p = ranged(m, .8, sign)
            assert out.command.opcode == CMD_RELEASE_BOTH
            assert out.reason == 'safe_zone_motion_cancel'
            assert m.range_command is None
            # Receipt of cancellation is not yet physical recovery.
            m.step(v,p,status(24),1.3)
            assert m.state == CompetitionState.WAIT_SEARCH_RECOVERY
            m.step(v,p,status(3),1.4)
            assert m.selected_batch is None
            if team == zone:
                assert m.state == CompetitionState.OBSTACLE_ROUTE
                assert m.obstacle_route.empty_entry
                assert not m.obstacle_route.original_ids
                assert m.obstacle_route.step == RouteStep.EMPTY_BODY_MOVE
            else:
                assert m.state == CompetitionState.SEARCH and m.obstacle_route is None


def test_actual_230mm_leg_is_not_counted_again_then_empty_crawl_is_blocked():
    m = mission()
    out,v,p = ranged(m,.692)
    assert out.command.opcode == CMD_APPROACH_TARGET
    end = m.range_boundary_end
    p = replace(p,y_m=.85)
    out = m.step(replace(v,frame_sequence=3),p,status(53),1.3)
    assert out.command.opcode == CMD_APPROACH_TARGET and m.range_boundary_end == end
    # The recorded failure entered old mode21 empty-observation crawl here.
    v = replace(v,frame_sequence=4,capture_audit=CargoAudit(),capture_observation_valid=True)
    out = m.step(v,replace(p,y_m=.868),replace(status(21),camera_pitch_cdeg=14000,flags=1),1.4)
    assert out.reason == 'safe_zone_motion_cancel'


def test_visible_load_near_edge_does_not_inherit_empty_crawl_distance_but_core_advance_does():
    m = mission()
    m.state = CompetitionState.CAPTURE_AUDIT
    p = PoseSnapshot(True,0,.91,90,0)
    green = CargoAudit(left_class='green_supply',left_count=1,total_count=1)
    v = VisionSnapshot(capture_audit=green)
    assert m._safe_zone_motion_exit(v,p,status(21),1.) is None
    m.state = CompetitionState.AUDIT_CONFIRM
    m.pending_audit = CargoAudit(left_class='core_black',left_count=1,total_count=1)
    m.pending_audit_valid = True
    assert m._safe_zone_motion_exit(v,p,status(21),1.).command.opcode == CMD_RELEASE_BOTH
    # Closed cargo and normal delivery must not be cancelled by pickup rules.
    m = mission();m.state = CompetitionState.NAVIGATE
    assert m._safe_zone_motion_exit(v,replace(p,y_m=1.1),status(10,flags=2),1.) is None


def test_empty_sweep_stages_first_skips_parking_and_never_looks_for_nonexistent_load():
    for side,sign in (('red',1),('blue',-1)):
        m = mission(side)
        m.boundary_sweep_point = (m.settings.material_target_x_m,m.settings.safe_staging_y_m)
        m._begin_pending_boundary_sweep(status(3),1.)
        route = m.obstacle_route
        assert RouteStep.FORWARD_PARK not in route.steps and RouteStep.RELEASE_LOAD not in route.steps
        p=PoseSnapshot(True,0,sign*.8,90 if sign==1 else 270,0)
        v=VisionSnapshot(frame_sequence=5)
        out=m._obstacle_route_output(v,p,status(3),1.1)
        assert out.command.opcode==CMD_BULLDOZER_STEP and out.command.flags>>4==13 and out.command.arg_a>0
        out.command.to_frame(1)
        done=status(25,context_valid=True,context_matches=True,accepted_command=CMD_BULLDOZER_STEP,action_status=3)
        # An old DONE while remaining distance is nonzero cannot skip staging.
        m._obstacle_route_output(v,p,done,1.2)
        assert route.step==RouteStep.EMPTY_BODY_MOVE
        for _ in range(12):
            if m.obstacle_route is None:break
            step=route.step
            if step in LIVE_STEPS:
                target=route.sweep_point if step in {RouteStep.BACK_TO_SWEEP,RouteStep.LEFT_TO_SWEEP} else route.return_point
                p=replace(p,x_m=target[0],y_m=target[1])
            # step12 actually completes in SEARCH, unlike movement steps.
            receipt=replace(done,mode=3 if step==RouteStep.PICKUP_READY else 25)
            out=m._obstacle_route_output(v,p,receipt,2.)
            if out.command:out.command.to_frame(1)
        assert m.state==CompetitionState.SEARCH and m.obstacle_route is None
        assert not m.carried_manifest


def test_pre_grab_fifty_mm_uses_same_boundary_before_sending_motion():
    m=mission()
    m.settings.pre_grab_advance_enabled=True
    m.state=CompetitionState.CAPTURE_AUDIT
    m.search_observer=(0,.91,90,14000)
    item=TrackedCargo(1,'green_supply',.9,(600,200,80,80),hits=4)
    view=VisionSnapshot(frame_sequence=10,cargo=(item,),pre_grab_regions=((1,'upper'),),
                        pre_grab_configured_classes=('green_supply',))
    out=m._pre_grab_depth_output(view,replace(status(21),camera_pitch_cdeg=14000,flags=1),1.)
    assert out.command.opcode==CMD_RELEASE_BOTH
    assert m.pre_grab_command is None


def test_empty_crawl_reuses_this_capture_budget_instead_of_adding_200mm_each_frame():
    m=mission();m.state=CompetitionState.CAPTURE_AUDIT
    v=VisionSnapshot(capture_audit=CargoAudit())
    p=PoseSnapshot(True,0,.6,90,0)
    assert m._safe_zone_motion_exit(v,p,status(21),1.) is None
    end=m.capture_crawl_boundary_end
    assert m._safe_zone_motion_exit(v,replace(p,y_m=.75),status(21),1.1) is None
    assert m.capture_crawl_boundary_end==end


def test_near_zone_outward_move_is_allowed_instead_of_reversing_towards_zone():
    for sign in (1,-1):
        assert crossed_safe_zone((0,sign*1.0),(0,sign*.8)) is None
        assert crossed_safe_zone((0,sign*1.0),(0,sign*1.1)) == ('red' if sign==1 else 'blue')
        assert crossed_safe_zone((0,sign*1.2),(0,sign*1.16)) is None
        assert crossed_safe_zone((0,sign*1.2),(0,sign*1.25)) == ('red' if sign==1 else 'blue')
    m=mission();m.state=CompetitionState.CAPTURE_AUDIT
    p=PoseSnapshot(True,0,1.,270,0)
    assert m._safe_zone_motion_exit(VisionSnapshot(capture_audit=CargoAudit()),p,status(21),1.) is None


def test_lower_boundary_reason_distinguishes_own_sweep_from_opponent_recovery():
    for team in ('red','blue'):
        for zone,bits in (('red',0x10),('blue',0x20)):
            m=mission(team)
            st=status(24,context_valid=True,context_matches=True,action_status=1|bits)
            assert st.pickup_boundary_side==zone
            out=m._local_boundary_recovery_output(VisionSnapshot(),st,1.)
            assert out.reason=='lower_pickup_boundary_cancel'
            assert m.state==CompetitionState.WAIT_SEARCH_RECOVERY
            m._local_boundary_recovery_output(VisionSnapshot(),replace(st,mode=3,action_status=3|bits),1.1)
            assert m.state==(CompetitionState.OBSTACLE_ROUTE if team==zone else CompetitionState.SEARCH)
    assert status(24,context_valid=True,action_status=0x30).pickup_boundary_side is None


def test_real_zone_detection_cancels_during_pre_grab_and_before_core_closes():
    for state,mode in ((CompetitionState.PRE_GRAB_ADVANCE,53),
                       (CompetitionState.AUDIT_CONFIRM,21),(CompetitionState.GRAB,8)):
        m=mission();m.state=state
        item=TrackedCargo(1,'green_supply',.9,(600,200,80,80),hits=4,inside_safe_zone=True)
        out=m.step(VisionSnapshot(cargo=(item,)),PoseSnapshot(True,0,0,90,0),status(mode),1.)
        assert out.reason=='safe_zone_target_cancel' and out.command.opcode==CMD_RELEASE_BOTH
        assert m.pre_grab_command is None
