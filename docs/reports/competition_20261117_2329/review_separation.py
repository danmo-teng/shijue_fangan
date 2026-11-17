"""Recompute recorded 140-degree composition using the actual ROI and code; no devices."""
from pathlib import Path
import json,sys
from unittest.mock import patch
from types import SimpleNamespace
R=Path(__file__).resolve().parents[3];sys.path[:0]=[str(R/'competition_rescue'),str(R/'vision')]
import run_competition_rescue as runner
from capture_roi import CaptureRois
from state_machine import CompetitionMission,CompetitionSettings,CompetitionState,CargoBatch,TrackedCargo,StmSnapshot
from protocol import crc16_modbus,CMD_DISPERSE_PILE
r=R/'rescue_map/runtime';events=[json.loads(l) for l in (r/'competition_events.jsonl').read_text().splitlines()]
start=next(d for d in events if d['event']=='competition_started');t0=start['timestamp_monotonic_ns']/1e9
rois=CaptureRois(*(tuple(tuple(p) for p in start[k]) for k in ['capture_polygon_px','capture_left_polygon_px','capture_right_polygon_px']))
frames={d['frame_sequence']:d for d in (json.loads(l) for l in (r/'competition_detections.jsonl').read_text().splitlines())}
results=[]
for target in [111.876,157.71,81.553]:
 ev=min((d for d in events if d['event']=='disperse_start'),key=lambda d:abs(d['timestamp_monotonic_ns']/1e9-t0-target))
 fr=ev['vision_frame_sequence'];data=frames.get(fr)
 if data is None:print('NO exact frame',fr);continue
 items=tuple(TrackedCargo(x['id'],x['class'],x['confidence'],tuple(x['bbox']),hits=x['hits'],misses=x['misses'],visible=x['visible']) for x in data['tracks'])
 m=CompetitionMission(CompetitionSettings(side='blue',initial_stash_enabled=False));m.first_common_delivered=True;m.state=CompetitionState.CAPTURE_AUDIT
 m._select_batch(CargoBatch((items[0].track_id,),('injured_orange',),'injury'))
 # The157.71s case carries left from the preceding selective separation.
 if target==157.71:m.separation_keep_side='left'
 with patch.object(runner,'tracked_cargo',return_value=items):
  view=runner.make_vision_snapshot([],[],SimpleNamespace(calibrated=False),None,StmSnapshot(mode=21,flags=1,age_ms=0,camera_pitch_cdeg=14000),m,fr,capture_rois=rois,observed_s=data['timestamp_monotonic_ns']/1e9)
 a=view.capture_audit
 result={'relative_s':ev['timestamp_monotonic_ns']/1e9-t0,'frame':fr,'recorded_event':{k:v for k,v in ev.items() if k!='failed_search_targets'},'tracks':data['tracks'],'recomputed_observation_valid':view.capture_observation_valid,'left_composition':a.left_class_counts if a else None,'right_composition':a.right_class_counts if a else None,'preferred_keep':m._preferred_keep_side(a) if a else None,'destination':m._audit_destination(a) if a else None,'note':'original image-quality/held-memory not fabricated; direct recorded-frame composition recomputation'}
 results.append(result);print(fr,result['left_composition'],result['right_composition'],result['preferred_keep'])
# Raw actual acceptance: mode25 plus matching task/action. This is not proof of servo measured angle.
p=Path(json.loads((r/'session.json').read_text())['localization_log_dir']);buf=b'';ctx=None;seen=set();status=[]
with (p/'uart_rx_native.jsonl').open() as f:
 for ln,l in enumerate(f,1):
  try:d=json.loads(l)
  except ValueError:continue
  t=d['timestamp_monotonic_ns']/1e9-t0
  if not (110<t<115 or 156<t<161):continue
  buf+=bytes.fromhex(d['data_hex'])
  while len(buf)>=15:
   i=buf.find(b'\xa3\xb3')
   if i<0:buf=buf[-1:];break
   buf=buf[i:]
   if len(buf)<15:break
   f0=buf[:15]
   if f0[14]!=195 or crc16_modbus(f0[2:12])!=int.from_bytes(f0[12:14],'little'):buf=buf[1:];continue
   buf=buf[15:]
   if f0[2]==30:ctx=f0
   elif f0[2]==23:
    if ctx and ctx[3]==f0[3] and ctx[8]==CMD_DISPERSE_PILE:
     key=(int.from_bytes(ctx[4:6],'big'),int.from_bytes(ctx[6:8],'big'),ctx[9],f0[5])
     if key not in seen:seen.add(key);status.append({'t':t,'line':ln,'task_action_status_mode':key,'context':ctx.hex(),'frame':f0.hex()})
    ctx=None
out=Path(__file__).parent
(out/'evidence.json').write_text(json.dumps({'t0':t0,'session':str(p.relative_to(R)),'cases':results,'accepted_separations':status},ensure_ascii=False,indent=2)+'\n')
print('acceptance',status)
