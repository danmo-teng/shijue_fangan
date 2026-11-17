"""Replay actual segmented-range DONE records through the production planner."""
from dataclasses import fields, replace
from pathlib import Path
import json
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'competition_rescue'),str(ROOT/'vision')]
from run_competition_rescue import CompetitionPlanner, JsonlLog
from state_machine import (CompetitionMission,CompetitionSettings,CompetitionState,
    CargoBatch,TrackedCargo,CommandRequest,CompetitionOutput,StmSnapshot,PoseSnapshot,VisionSnapshot)
from protocol import CMD_APPROACH_TARGET,CMD_APPROACH_RANGE,CMD_HOLD


def fixture(folder,index):
    cases=json.loads((ROOT/'docs/reports/competition_20261118_range_stall/reproduction.json').read_text())
    d=cases[index]['replay_record']
    m=CompetitionMission(CompetitionSettings(side='blue',initial_stash_enabled=False,
                                           top_edge_range_reobserve_enabled=True))
    m.first_common_delivered=True
    m._select_batch(CargoBatch(((301,368)[index],),('green_supply',),'material'))
    m.state=CompetitionState(d['upper_state'])
    items=tuple(TrackedCargo(x['id'],x['class'],x['confidence'],tuple(x['bbox']),
        hits=x['hits'],misses=x['misses'],visible=x['visible']) for x in d['tracks'])
    target=next(x for x in items if x.track_id==(301,368)[index])
    now=d['snapshot_monotonic_ns']/1e9
    m._mark_target_seen(target,now)
    if index:
        m.cluster_target_class=target.class_name;m.cluster_target_track_id=target.track_id
        m.cluster_target_bbox=m.cluster_bbox=target.bbox
    m.range_epoch=1;m.range_started_s=now-3;m.range_pitch_cdeg=12000
    m.range_frame_floor=d['command']['source_vision_frame']-1
    m.range_command=CommandRequest(**d['command'])
    status=StmSnapshot(**{k:v for k,v in d['stm'].items() if k in {f.name for f in fields(StmSnapshot)}})
    p=CompetitionPlanner(m,folder/'pose',folder/'stm',folder/'command',folder/'diag',JsonlLog(folder/'events'),50)
    p.protocol_task=p.protocol_task_cursor=status.task_id
    p.protocol_action=p.protocol_action_cursor=status.action_id
    p.protocol_task_generation=m.task_generation
    p.protocol_action_key=(CMD_APPROACH_TARGET,m.range_command.flags,None,None,1)
    p.protocol_last_output=CompetitionOutput(m.state,m.range_command,'recorded range')
    view=VisionSnapshot(frame_sequence=d['frame_sequence'],observed_monotonic_s=now,
        cargo=items,range_pitch_cdeg=12000,range_distances=((target.track_id,100),))
    pose=PoseSnapshot(True,d['pose']['x_m'],d['pose']['y_m'],d['pose']['yaw_deg'],0.)
    return p,view,pose,status,target,now


def test_recorded_done_resumes_measurement_then_distinct_final_segment(tmp_path):
    for index in (0,1):
        p,v,pose,st,target,now=fixture(tmp_path/str(index),index);m=p.mission
        old_action=st.action_id
        assert st.context_matches is False  # Actual load_stm/replay default.
        out=p._output_for_cycle(v,pose,st,False,now)
        assert out.reason=='approach_range_wait' and out.command.opcode==CMD_HOLD
        assert m.range_command is None and m.range_epoch==2
        p._publish(out)
        assert p.protocol_action==old_action
        p._output_for_cycle(v,pose,st,False,now+.01)
        assert m.range_epoch==2  # Repeated DONE must not restart or advance twice.
        # The first new picture after completion still requires pixel alignment.
        v=replace(v,frame_sequence=v.frame_sequence+1,observed_monotonic_s=now+.1)
        out=p._output_for_cycle(v,pose,st,False,now+.1)
        # Recorded cluster368 is already inside its40px alignment tolerance.
        if out.reason=='approach_range_realign':
            p._publish(out)
            st=replace(st,action_id=p.protocol_action,action_status=1)
            x,y,w,h=target.bbox;target=replace(target,bbox=(round(640-w/2),y,w,h))
            v=replace(v,cargo=tuple(target if item.track_id==target.track_id else item for item in v.cargo),
                      frame_sequence=v.frame_sequence+1,observed_monotonic_s=now+.2)
            out=p._output_for_cycle(v,pose,st,False,now+.2)
        assert out.reason=='approach_range_request'
        assert out.command.arg_a==100 and out.command.arg_b==12000 and out.command.aux==0
        p._publish(out)
        assert p.protocol_action!=old_action
        # The final aux0 segment still hands off to the original140-degree path.
        current=replace(st,action_id=p.protocol_action,action_status=1,mode=20,camera_pitch_cdeg=12500)
        handoff=p._output_for_cycle(v,pose,current,False,now+.3)
        assert handoff.reason=='approach_range_handoff' and handoff.command.opcode==CMD_HOLD


def test_wrong_task_or_action_done_cannot_finish_current_segment(tmp_path):
    for index in (0,1):
        for mismatch in ('task','action'):
            p,v,pose,st,_,now=fixture(tmp_path/(str(index)+mismatch),index);m=p.mission
            request=m.range_command
            # Even an externally supplied True cannot override actual identity.
            st=replace(st,task_id=st.task_id-1 if mismatch=='task' else st.task_id,
                       action_id=st.action_id-1 if mismatch=='action' else st.action_id,
                       context_matches=True)
            out=p._output_for_cycle(v,pose,st,False,now)
            assert out.command==request and m.range_command==request and m.range_epoch==1


def test_camera_pause_records_done_without_launching_another_distance(tmp_path):
    p,v,pose,st,_,now=fixture(tmp_path,0)
    p.pause_reason='camera_recovery'
    out=p._output_for_cycle(v,pose,st,True,now)
    assert out.command.opcode==CMD_HOLD and p.mission.range_command is None
    assert p.mission.range_epoch==2
