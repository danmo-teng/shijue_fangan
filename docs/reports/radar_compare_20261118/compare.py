"""Offline comparison only. Reads saved scans/pose velocities; no devices or runtime changes."""
from pathlib import Path
import json,math,collections,statistics,bisect,hashlib
R=Path(__file__).resolve().parents[3];ROOT=R/'rescue_map/runtime/history/competition_wheel_input'
NAMES=['20261117_180151_competition_wheel','20261117_200258_competition_wheel','20261117_222902_competition_wheel','20261117_232906_competition_wheel','20261118_004831_competition_wheel','20000101_080057_competition_wheel','20261118_034119_competition_wheel','20000101_081057_competition_wheel','20000101_081300_competition_wheel']
CUTS={'20000101_080057_competition_wheel':495.387582133,'20261118_004831_competition_wheel':397.226546103,'20261118_034119_competition_wheel':299.742,'20000101_081057_competition_wheel':99.672,'20000101_081300_competition_wheel':496.15}
def read(p):
 for n,l in enumerate(open(p),1):
  try:yield n,json.loads(l)
  except ValueError:pass
def median(values):
 vals=[x for x in values if x is not None and math.isfinite(x)];return statistics.median(vals) if vals else None
def stats(rows):
 if not rows:return {'n':0}
 counts=collections.Counter(','.join(d.get('position_observed_axes',[])) or '-' for _,d in rows)
 failed=[d for _,d in rows if not d.get('position_observed_axes')];reasons=collections.Counter();axis_reasons=collections.Counter();onewall=[]
 for d in failed:
  rr={r for reasons in (d.get('axis_rejections') or {}).values() for r in reasons}
  reasons['+'.join(sorted(rr)) or d.get('reason','unknown')]+=1
  for a,rs in (d.get('axis_rejections') or {}).items():
   for why in rs:axis_reasons[a+':'+why]+=1
  if d.get('supported_lines')==1 and len(d.get('wall_ids',[]))==1 and 'wall_residual' in rr:onewall.append(d)
 return dict(n=len(rows),any_axis_pct=100*(len(rows)-counts['-'])/len(rows),axes=dict(counts),failure_reasons=dict(reasons),axis_failure_reasons=dict(axis_reasons),single_line_wall_residual=len(onewall),points_median=median(d.get('physical_points') for _,d in rows),used_median=median(d.get('used_points') for _,d in rows),lines_median=median(d.get('supported_lines') for _,d in rows),fit_residual_median_m=median(d.get('residual_rms_m') for _,d in rows),compute_cpu_median_ms=median(d.get('geometry_compute_cpu_ms') for _,d in rows),roundtrip_median_ms=median(d.get('geometry_roundtrip_ms') for _,d in rows),unsupported_ms_median=median(d.get('motion_unsupported_span_ns',0)/1e6 for _,d in rows))
output=[]
for name in NAMES:
 p=ROOT/name;meta=json.load(open(p/'metadata.json'));rows=list(read(p/'dynamic_localization.jsonl'));t0=rows[0][1]['reference_ns']/1e9
 cutoff=min(CUTS.get(name,float('inf')),t0+180)
 rows=[(n,d) for n,d in rows if d['reference_ns']/1e9<=cutoff]
 # Use measured wheel speed and primary-yaw change, independent of T265 visual confidence.
 # Low wheel/gyro motion is a recorded state, not external ground truth.
 velocities=[];last=-float('inf')
 for _,d in read(p/'motion_feedback.jsonl'):
  if d.get('type')!=7:continue
  t=d['receive_monotonic_ns']/1e9
  if t<t0-.1:continue
  if t>cutoff+.1:break
  if t-last>=.04:
   velocities.append((t,math.hypot(d['forward_um_s'],d['left_um_s'])/1e6));last=t
 times=[x[0] for x in velocities];groups=collections.defaultdict(list);yawbins=collections.defaultdict(list);prev=None;examples=[]
 for n,d in rows:
  t=d['reference_ns']/1e9;yaw=d['primary_yaw_field_rad'];bin0=int((math.degrees(yaw)%90)//30)*30;yawbins[str(bin0)].append((n,d))
  if prev and t>prev[0] and velocities:
   rate=abs(math.degrees(math.atan2(math.sin(yaw-prev[1]),math.cos(yaw-prev[1])))/(t-prev[0]))
   i=min(len(times)-1,bisect.bisect_left(times,t));v=velocities[i][1]
   group='turning' if rate>=3 else 'low_wheel_and_gyro' if v<.03 else 'straight_motion'
   groups[group].append((n,d))
  prev=(t,yaw)
  rr={r for rs in (d.get('axis_rejections') or {}).values() for r in rs}
  if len(examples)<3 and not d.get('position_observed_axes') and d.get('supported_lines')==1 and 'wall_residual' in rr:examples.append(dict(source_line=n,record=d))
 mount=meta['lidar_installation'];fingerprint={k:mount.get(k) for k in ['raw_start_deg','raw_end_deg','extrinsic','fov_deg','excluded_raw_ranges_deg']}
 info=dict(session=name,side=meta['side'],zone=meta['start_zone'],upper=meta['upper_git_commit'],start_reference_s=t0,cutoff_s=cutoff,mount=fingerprint,config_sha256=hashlib.sha256((p/'localization.conf').read_bytes()).hexdigest(),first30=stats([(n,d) for n,d in rows if d['reference_ns']/1e9<=t0+30]),first180_or_saved=stats(rows),motion_groups={k:stats(v) for k,v in groups.items()},yaw_mod90_bins={k:stats(v) for k,v in yawbins.items()},single_line_examples=examples)
 output.append(info)
 print(name,meta['side'],meta['start_zone'],'first30',round(info['first30']['any_axis_pct'],1),'first180',round(info['first180_or_saved']['any_axis_pct'],1),'quiet',[(k,v['n'],round(v.get('any_axis_pct',0),1)) for k,v in info['motion_groups'].items()],flush=True)
Path(__file__).with_name('comparison.json').write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n')
