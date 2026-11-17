"""Read saved evidence and call the planner without starting devices or its thread."""
from pathlib import Path
from dataclasses import fields,replace
from types import SimpleNamespace
import json,sys,tempfile
R=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(R/'competition_rescue'),str(R/'vision')]
from run_competition_rescue import CompetitionPlanner,JsonlLog
from state_machine import *
P=R/'rescue_map/runtime/history/competition_wheel_input/20000101_081300_competition_wheel'
records=[]
for clip,target in [(5,4378),(7,6041)]:
 for line in (P/f'safe_zone_replay_{clip:04d}.jsonl').open():
  try:d=json.loads(line)
  except ValueError:continue
  if d['frame_sequence']==target:break
 else:raise AssertionError('recorded frame missing')
 m=CompetitionMission(CompetitionSettings(side='blue',initial_stash_enabled=False,top_edge_range_reobserve_enabled=True))
 m.first_common_delivered=True;m.state=CompetitionState(d['upper_state'])
 items=tuple(TrackedCargo(x['id'],x['class'],x['confidence'],tuple(x['bbox']),hits=x['hits'],misses=x['misses'],visible=x['visible']) for x in d['tracks'])
 target_item=next(x for x in items if x.track_id==(301 if target==4378 else 368))
 m._select_batch(CargoBatch((target_item.track_id,),('green_supply',),'material'))
 if m.state==CompetitionState.CLUSTER_APPROACH:
  m.cluster_target_class=target_item.class_name;m.cluster_target_track_id=target_item.track_id;m.cluster_target_bbox=m.cluster_bbox=target_item.bbox
 m.range_epoch=1;m.range_started_s=d['snapshot_monotonic_ns']/1e9-3
 m.range_frame_floor=d['command']['source_vision_frame']-1;m.range_pitch_cdeg=12000
 m.range_command=CommandRequest(**d['command'])
 status=StmSnapshot(**{k:v for k,v in d['stm'].items() if k in {f.name for f in fields(StmSnapshot)}})
 view=VisionSnapshot(frame_sequence=d['frame_sequence'],observed_monotonic_s=d['capture_monotonic_ns']/1e9,cargo=items,range_pitch_cdeg=12000)
 now=d['snapshot_monotonic_ns']/1e9;pose=PoseSnapshot(True,d['pose']['x_m'],d['pose']['y_m'],d['pose']['yaw_deg'],0.)
 with tempfile.TemporaryDirectory() as folder:
  root=Path(folder);planner=CompetitionPlanner(m,root/'pose',root/'stm',root/'command',root/'diag',JsonlLog(root/'events'),50)
  planner.protocol_task=status.task_id;planner.protocol_action=status.action_id
  actual=planner._output_for_cycle(view,pose,status,False,now)
  before=dict(reason=actual.reason,aux=actual.command.aux,range_epoch=m.range_epoch,range_command_present=m.range_command is not None)
  assert before['reason']=='approach_range_move' and before['range_epoch']==1
  matched=replace(status,context_matches=(status.task_id==planner.protocol_task and status.action_id==planner.protocol_action))
  expected=m._range_approach_output(view,matched,now,pose)
  after=dict(reason=expected.reason,range_epoch=m.range_epoch,range_command_present=m.range_command is not None)
  assert after['range_epoch']==2 and not after['range_command_present']
 records.append(dict(frame=d['frame_sequence'],replay_record=d,current_runner=before,same_existing_handler_with_actual_match=after))
print(json.dumps(records,ensure_ascii=False,indent=2))
