"""Orange-only camera change before alignment; no hardware or model execution."""
import sys
import pytest
from pathlib import Path
from dataclasses import replace

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT.parent / "vision")]
from protocol import (CMD_APPROACH_TARGET, CMD_APPROACH_RANGE, CMD_CLUSTER_TARGET,
                      CMD_DISTANCE_VALID, CMD_HOLD, APPROACH_ORANGE_TOP_VIEW, mission_frame)
from state_machine import (CompetitionMission, CompetitionSettings, CompetitionState,
    CargoBatch, TrackedCargo, VisionSnapshot, StmSnapshot, PoseSnapshot, CompetitionOutput)


def scenario(cluster=False):
    m = CompetitionMission(CompetitionSettings(side="blue", initial_stash_enabled=False))
    m.first_common_delivered = True
    m._select_batch(CargoBatch((335,), ("injured_orange",), "injury"))
    m.state = CompetitionState.CLUSTER_APPROACH if cluster else CompetitionState.APPROACH
    # Actual00:48 frame6925: the orange disappears while rotating at120deg.
    target = TrackedCargo(335, "injured_orange", .9, (971, 0, 92, 36), hits=4)
    m._mark_target_seen(target, 1.)
    if cluster:
        m.cluster_target_class = target.class_name
        m.cluster_target_track_id = target.track_id
        m.cluster_target_bbox = m.cluster_bbox = target.bbox
        m.cluster_command_observed_s = 1.
    view = VisionSnapshot(cargo=(target,), frame_sequence=10, observed_monotonic_s=1.,
        range_pitch_cdeg=12000, range_distances=((335, 460),), range_top_band_ids=(335,))
    status = StmSnapshot(mode=20, age_ms=0, camera_pitch_cdeg=12000, context_valid=True,
        context_matches=True, task_id=1, action_id=2, accepted_command=CMD_APPROACH_TARGET,
        action_status=1)
    return m, view, status


def test_trigger_scope_and_real_b1_partition():
    from rescue_vision.localizer import ApproachRangeCalibration
    table = ApproachRangeCalibration.load(ROOT.parent / "vision/config/approach_range.json")
    for cluster in (False, True):
        m, v, s = scenario(cluster)
        assert table.distance(12000, v.cargo[0].center_px, (1280,1024))[0] == 460
        out = m._orange_top_view_output(v, s, 1.)
        assert out.command.aux == APPROACH_ORANGE_TOP_VIEW
        assert out.command.arg_a == 0 and out.command.arg_b == 10500
        assert bool(out.command.flags & CMD_CLUSTER_TARGET) == cluster
        assert m.range_command is None
    for change in ("green", "not_top", "near", "105", "moving", "closed", "unaccepted", "stash"):
        m, v, s = scenario()
        if change == "green": v = replace(v, cargo=(replace(v.cargo[0], class_name="green_supply"),))
        if change == "not_top": v = replace(v, cargo=(replace(v.cargo[0], bbox=(971,1,92,36)),))
        if change == "near": v = replace(v, range_top_band_ids=())
        if change == "105": s = replace(s, camera_pitch_cdeg=10500)
        if change == "moving": s = replace(s, mode=53)
        if change == "closed": s = replace(s, flags=2)
        if change == "unaccepted": s = replace(s, context_matches=False)
        if change == "stash": m.selected_batch = replace(m.selected_batch, initial_stash=True)
        assert m._orange_top_view_output(v, s, 1.) is None, change


def test_camera_action_keeps_target_and_rebinds_unique_horizontal_orange():
    for cluster in (False, True):
        m, v, s = scenario(cluster)
        request = m._orange_top_view_output(v, s, 1.).command
        generation, epoch = m.task_generation, m.range_epoch
        empty = replace(v, frame_sequence=11, cargo=(), observed_monotonic_s=1.1)
        assert m._orange_top_view_output(empty, s, 1.1).command == request
        wrong = replace(s, camera_pitch_cdeg=10500, action_status=3, context_matches=False)
        assert m._orange_top_view_output(empty, wrong, 1.1).command == request
        done = replace(wrong, context_matches=True)
        assert m._orange_top_view_output(empty, done, 1.1).command.opcode == CMD_HOLD
        assert m._orange_top_view_output(empty, done, 1.1).command.opcode == CMD_HOLD
        target = replace(v.cargo[0], track_id=400, bbox=(950,180,120,130))
        new = replace(v, frame_sequence=12, observed_monotonic_s=1.2, cargo=(target,),
                      range_pitch_cdeg=10500, range_top_band_ids=())
        assert m._orange_top_view_output(new, done, 1.2) is None
        assert m.selected_batch.track_ids == (400,)
        assert m._target_for_batch(new).track_id == 400
        assert (m.task_generation, m.range_epoch) == (generation, epoch)
        assert m._orange_top_view_output(v, s, 1.3) is None  # No second lift in this attempt.


def test_ambiguity_and_missing_frame_use_existing_recovery():
    for ambiguous in (False, True):
        m, v, s = scenario(True)
        m._orange_top_view_output(v, s, 1.)
        done = replace(s, camera_pitch_cdeg=10500, action_status=3)
        m._orange_top_view_output(v, done, 1.1)
        candidates = tuple(replace(v.cargo[0], track_id=i, bbox=(980,200,90,90))
                           for i in (401,402)) if ambiguous else ()
        view = replace(v, frame_sequence=11, cargo=candidates, range_pitch_cdeg=10500)
        out = m._orange_top_view_output(view, done, 1.2)
        assert out.command.opcode == CMD_HOLD
        if ambiguous:
            assert m.state == CompetitionState.WAIT_SEARCH_RECOVERY
        else:
            # No new timer. Existing MCU lost-target recovery owns the exit.
            assert m._orange_top_view_output(view, replace(done, mode=24), 1.7) is None
            out = m.step(view, PoseSnapshot(), replace(done, mode=24), 1.7)
            assert out.state == CompetitionState.WAIT_SEARCH_RECOVERY
        assert m.orange_top_view_command is None
        assert m.orange_top_view_frame_floor is None


def test_changed_id_inside_zone_is_not_approached():
    m,v,s = scenario()
    m._orange_top_view_output(v,s,1.)
    done = replace(s,camera_pitch_cdeg=10500,action_status=3)
    m._orange_top_view_output(v,done,1.1)
    item = replace(v.cargo[0],track_id=500,bbox=(970,200,100,100),inside_safe_zone=True)
    view = replace(v,frame_sequence=11,range_pitch_cdeg=10500,cargo=(item,))
    assert m._orange_top_view_output(view,done,1.2) is None
    assert m._safe_zone_target_exit(view,done,1.2).event == "safe_zone_target_cancel"


def test_protocol_no_translation_and_aux3_unchanged():
    import pytest
    for side in (0,8):
        for cluster in (0,CMD_CLUSTER_TARGET):
            flags = 1 | side | cluster | CMD_APPROACH_RANGE | CMD_DISTANCE_VALID
            packet = mission_frame(7,CMD_APPROACH_TARGET,flags,0,10500,APPROACH_ORANGE_TOP_VIEW)
            assert packet[6:8] == bytes(2)
            for a,b,aux in [(1,10500,4),(0,12000,4),(0,14000,4),(0,10500,3)]:
                with pytest.raises(ValueError): mission_frame(7,CMD_APPROACH_TARGET,flags,a,b,aux)


@pytest.mark.parametrize("cluster", [False, True])
def test_runner_camera_then_pixels_then_105_range_have_distinct_actions(tmp_path, cluster):
    from run_competition_rescue import CompetitionPlanner, JsonlLog
    m,v,s = scenario(cluster)
    planner = CompetitionPlanner(m,tmp_path/'pose',tmp_path/'stm',tmp_path/'command',
                                tmp_path/'diagnostic',JsonlLog(tmp_path/'events'),50)
    planner.protocol_vision = v
    pixels = m._cluster_approach_command(v) if cluster else m._approach_command(v.cargo[0])
    planner._publish(CompetitionOutput(m.state,pixels,"align"))
    task, old_action = planner.protocol_task, planner.protocol_action
    s = replace(s,task_id=task,action_id=old_action)
    out = planner._output_for_cycle(v,PoseSnapshot(),s,False,1.)
    planner._publish(out)
    lift_action = planner.protocol_action
    assert lift_action != old_action and planner.protocol_task == task
    # Delayed old ACCEPT/DONE cannot finish the new camera request.
    for status in (s,replace(s,action_status=3,camera_pitch_cdeg=10500)):
        out = planner._output_for_cycle(v,PoseSnapshot(),status,False,1.01)
        assert out.command.aux == APPROACH_ORANGE_TOP_VIEW
        planner._publish(out)
        assert planner.protocol_action == lift_action
    done = replace(s,action_id=lift_action,action_status=3,camera_pitch_cdeg=10500)
    out = planner._output_for_cycle(v,PoseSnapshot(),done,False,1.1)
    assert out.command.opcode == CMD_HOLD
    planner._publish(out)
    target = replace(v.cargo[0],track_id=400,bbox=(950,180,120,130))
    view = replace(v,frame_sequence=11,observed_monotonic_s=1.2,cargo=(target,),
                   range_pitch_cdeg=10500,range_distances=((400,500),),range_top_band_ids=())
    out = planner._output_for_cycle(view,PoseSnapshot(),done,False,1.2)
    assert not out.command.flags & CMD_APPROACH_RANGE and out.command.arg_a == 1010
    planner._publish(out)
    align_action = planner.protocol_action
    assert align_action != lift_action and planner.protocol_task == task
    status = replace(done,action_id=align_action,mode=52,action_status=1)
    centred = replace(view,cargo=(replace(target,bbox=(600,180,80,130)),))
    planner._output_for_cycle(centred,PoseSnapshot(),status,False,1.3)
    centred = replace(centred,frame_sequence=12,observed_monotonic_s=1.4)
    out = planner._output_for_cycle(centred,PoseSnapshot(),status,False,1.4)
    assert out.command.arg_a == 500 and out.command.arg_b == 10500 and out.command.aux == 0
    planner._publish(out)
    assert planner.protocol_action != align_action and planner.protocol_task == task
    assert int.from_bytes(planner.command_path.read_bytes()[8:12],'big') == 12


def test_runner_rejected_camera_request_does_not_wait_forever(tmp_path):
    from run_competition_rescue import CompetitionPlanner, JsonlLog
    m,v,s = scenario()
    p = CompetitionPlanner(m,tmp_path/'pose',tmp_path/'stm',tmp_path/'command',
                          tmp_path/'diagnostic',JsonlLog(tmp_path/'events'),50)
    p.protocol_vision = v
    p._publish(CompetitionOutput(m.state,m._approach_command(v.cargo[0]),"align"))
    s = replace(s,task_id=p.protocol_task,action_id=p.protocol_action)
    out=p._output_for_cycle(v,PoseSnapshot(),s,False,1.)
    p._publish(out)
    rejected=replace(s,action_status=5,rejected_action_id=p.protocol_action)
    out=p._output_for_cycle(v,PoseSnapshot(),rejected,False,1.1)
    assert out.reason == 'orange_top_view_rejected'
    assert m.state == CompetitionState.WAIT_SEARCH_RECOVERY and out.command.opcode == CMD_HOLD
