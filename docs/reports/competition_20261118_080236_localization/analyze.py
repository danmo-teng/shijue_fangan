"""Offline saved-log analysis only; no device or runtime writes."""
from pathlib import Path
import collections
import json
import math
import statistics

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
RUN = ROOT / 'rescue_map/runtime/history/competition_wheel_input/20000101_080236_competition_wheel'
T0 = 149.423017624
# User confirms lifting the car at approximately +310s. Stop before that
# transition; 307s is a conservative end at the last search-station arrival.
CUTOFF = 307.0

def read(path):
    for line, text in enumerate(path.open(), 1):
        try:
            yield line, json.loads(text)
        except ValueError:
            continue  # Do not repair or overwrite incomplete original logs.

rows = [(n, (d['reference_ns'] / 1e9 - T0), d)
        for n, d in read(RUN / 'dynamic_localization.jsonl')]
ground = [(n, t, d) for n, t, d in rows if 0 <= t < CUTOFF]

def stats(a, b):
    selected = [(n, t, d) for n, t, d in ground if a <= t < b]
    counts = collections.Counter(','.join(d['position_observed_axes']) or '-'
                                 for _, _, d in selected)
    reasons = collections.Counter(reason for _, _, d in selected
        for reasons in (d.get('axis_rejections') or {}).values() for reason in reasons)
    sources = collections.Counter(d['motion_source'] for _, _, d in selected)
    return dict(seconds=[a, b], n=len(selected), axes=dict(counts),
        any_axis_percent=100*(len(selected)-counts['-'])/len(selected),
        source_samples=dict(sources), axis_rejection_counts=dict(reasons),
        physical_points_median=statistics.median(d['physical_points'] for _, _, d in selected),
        associated_wall_ids=dict(collections.Counter(str(d['wall_ids']) for _, _, d in selected)))

summary = dict(session=json.loads((RUN / 'metadata.json').read_text())['sdk_session_id'],
    upper=json.loads((RUN / 'metadata.json').read_text())['upper_git_commit'],
    actual_lower=json.loads((RUN / 'chassis_identity.json').read_text()),
    reference_monotonic_s=T0, ground_cutoff_s=CUTOFF,
    windows=[stats(a,b) for a,b in [(0,30),(0,60),(60,180),(180,270),(270,307),(279,307),(0,307)]],
    last_pre_return_axis_correction={axis:dict(source_line=n,seconds=t,record=d)
        for axis in ['x','y'] for n,t,d in
        [[(n,t,d) for n,t,d in ground if axis in d['position_observed_axes']][-1]]},
    policy_events=[dict(source_line=n,seconds=d['timestamp_monotonic_ns']/1e9-T0,record=d)
                   for n,d in read(RUN/'native_motion_events.jsonl')
                   if d['event']=='competition_wheel_policy_applied'],
    source_events=[dict(source_line=n,seconds=d['timestamp_monotonic_ns']/1e9-T0,record=d)
                   for n,d in read(RUN/'lidar_worker_events.jsonl')
                   if d['event']=='motion_source_changed'],
    sdk_discontinuities=[dict(source_line=n,seconds=d['timestamp_monotonic_ns']/1e9-T0,record=d)
                        for n,d in read(RUN/'localization_events.jsonl')
                        if d['event']=='t265_position_discontinuity'],
    radar_examples=[dict(source_line=n,seconds=t,record=d) for n,t,d in ground
                    if n in [2782,2833,2997,3013]],
    large_adopted_corrections=[dict(source_line=n,seconds=t,
        axes=d['position_observed_axes'],delta_xy_m=[d['candidate_center_xy_m'][j]-d['prior_center_xy_m'][j] for j in range(2)])
        for n,t,d in ground if d['position_observed_axes'] and
        math.dist(d['candidate_center_xy_m'],d['prior_center_xy_m']) > .08])
# Preserve selected current-run business records. The similarly named history
# directory contains old task IDs and is NOT this run's business log.
events=list(read(ROOT/'rescue_map/runtime/competition_events.jsonl'))
matched=any(d.get('policy',{}).get('session_id')==summary['session'] for _,d in events)
if not matched:
    raise SystemExit('runtime business log has changed session; use saved evidence.json, do not mix runs')
summary['business_evidence']=[dict(source_line=n,record=d) for n,d in events
    if d['event'] in {'warm_reference_bound','localization_wheel_policy_sent'} or
    (d['event']=='state_changed' and 279<=d['timestamp_monotonic_ns']/1e9-T0<307)]
summary['view_comparison']=[]
for clip,index in [(2,30),(7,35)]:
    records=list(read(RUN/f'safe_zone_replay_{clip:04}.jsonl'))
    n,d=records[index]
    summary['view_comparison'].append(dict(clip=clip,source_line=n,record=d))
(OUT/'evidence.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
for w in summary['windows']:
    print(w['seconds'],w['n'],round(w['any_axis_percent'],2),w['axes'])
