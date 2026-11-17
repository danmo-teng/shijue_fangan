"""Read saved evidence only; no camera/UART/model execution."""
from pathlib import Path
import json,sys
R=Path(__file__).resolve().parents[3];sys.path[:0]=[str(R/'competition_rescue'),str(R/'vision')]
from protocol import crc16_modbus
from state_machine import TrackedCargo,cargo_in_front_region
from run_competition_rescue import cargo_on_carrying_platform,safe_zone_push_corridor
P=R/'rescue_map/runtime/history/competition_wheel_input/20261117_200258_competition_wheel'
T0=7489.939864557
rows=[];buf=b'';ctx=None;bad=0
with (P/'uart_rx_native.jsonl').open('rb') as stream:
 for lineno,line in enumerate(stream,1):
  try:d=json.loads(line)
  except (ValueError,UnicodeDecodeError):bad+=1;continue
  t=d['timestamp_monotonic_ns']/1e9-T0
  if not (5<t<16 or 315<t<370):continue
  buf+=bytes.fromhex(d['data_hex'])
  while len(buf)>=15:
   k=buf.find(b'\xa3\xb3')
   if k<0:buf=buf[-1:];break
   buf=buf[k:]
   if len(buf)<15:break
   f=buf[:15]
   if f[14]!=195 or crc16_modbus(f[2:12])!=int.from_bytes(f[12:14],'little'):buf=buf[1:];continue
   buf=buf[15:]
   if f[2]==30:ctx=f
   elif f[2]==23:
    row={'t_s':t,'line':lineno,'raw_hex':f.hex(),'mode':f[5],'flags':f[4],'pitch':int.from_bytes(f[6:8],'big')}
    if ctx is not None and ctx[3]==f[3]:row.update(context_hex=ctx.hex(),task=int.from_bytes(ctx[4:6],'big'),action=int.from_bytes(ctx[6:8],'big'),command=ctx[8],status=ctx[9],rejected_action=int.from_bytes(ctx[10:12],'big'))
    rows.append(row);ctx=None
out=Path(__file__).parent
summary=[];key=None
for d in rows:
 k=(d['mode'],d.get('action'),d.get('command'),d.get('status'),d.get('rejected_action'))
 if k!=key:summary.append(d);key=k
(out/'uart_transitions.json').write_text(json.dumps({'source':str(P.relative_to(R))+'/uart_rx_native.jsonl','bad_json_lines':bad,'transitions':summary},indent=2)+'\n')
for d in summary:
 if d['t_s']<16 or (325<d['t_s']<333) or d.get('rejected_action',0):print({k:v for k,v in d.items() if k not in ['raw_hex','context_hex']})
replay=[json.loads(l) for l in (P/'safe_zone_replay_0010.jsonl').read_text().splitlines()]
geometry=[]
for idx,old_id in [(33,513),(122,564)]:
 d=next(x for x in replay if x['video_frame_index']==idx)
 items=tuple(TrackedCargo(x['id'],x['class'],x['confidence'],tuple(x['bbox']),hits=x['hits'],visible=x['visible']) for x in d['tracks'])
 black=next(x for x in items if x.class_name=='core_black')
 poly=tuple(tuple(p) for p in d['roi_polygons'][1]);safe=tuple(d['safe_bbox'])
 import unittest.mock
 # Use the exact saved polygon, not a newer manually edited config.
 with unittest.mock.patch('run_competition_rescue.safe_zone_corridor_polygon',return_value=poly):
  result=safe_zone_push_corridor(items,safe,'material',True,frozenset({old_id}),carried_manifest=('core_black',),carried_total_count=1,claw_polygon=tuple(tuple(p) for p in d['roi_polygons'][0]),include_compatible_obstacles=True,camera_pitch_cdeg=12000)
 row={'frame':d['frame_sequence'],'black_id':black.track_id,'bbox':black.bbox,'foot':[black.bbox[0]+black.bbox[2]/2,black.bbox[1]+black.bbox[3]],'in_saved_sweep_polygon':cargo_in_front_region(black,poly,safe),'in_existing_carried_platform':cargo_on_carrying_platform(black,12000),'recomputed_obstacle_id':result[1].track_id if result[1] else None,'excluded_id_example':old_id}
 geometry.append(row);print('GEOMETRY',row)
(out/'geometry_check.json').write_text(json.dumps(geometry,indent=2)+'\n')
