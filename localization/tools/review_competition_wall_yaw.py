"""Offline wall-heading review. Positive angles add CCW to reported field yaw.
The reference assumes correctly placed, straight fence sections. A skewed wall
biases the absolute correction; viewing a curved section from another position
can also change the estimate between visits.
"""
import argparse
import json
import math
from collections import Counter
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from lidar_yaw_debug.dynamic import accepted_points, rotation
from lidar_yaw_debug.static import extract_lines, outer_supported_lines


def records(path):
    with path.open() as stream:
        for line in stream:
            try:
                yield json.loads(line)
            except ValueError:
                continue  # A power cut can leave a partial final line.


def wall_angles(scan, fit, mount):
    points = accepted_points(SimpleNamespace(points=[SimpleNamespace(**p) for p in scan['points']]), mount)
    if not points:
        return None
    yaw = fit['primary_yaw_field_rad']
    x, y, angle = mount['extrinsic']
    body = np.array([p.point_lidar_m for p in points]) @ rotation(math.radians(angle)).T + (x,y)
    prior = fit['prior_center_xy_m']
    by_axis = [[], []]
    for line in outer_supported_lines(extract_lines(body)):
        normal = rotation(yaw) @ line['normal']
        axis = int(abs(normal[1]) > abs(normal[0]))
        angle = math.degrees(math.atan2(normal[1], normal[0]))
        correction = (90*axis - angle + 90) % 180 - 90
        projected = line['points'] @ rotation(yaw).T + prior
        wall = 1.5 if np.median(projected[:,axis]) > 0 else -1.5
        if (abs(correction) <= 12 and np.median(abs(projected[:,axis]-wall)) < .20
                and np.mean(abs(projected[:,1-axis]) <= 1.55) > .85):
            by_axis[axis].append(correction)
    if not all(by_axis):
        return None
    # Reject incompatible lines rather than selecting the closest pair after seeing the answer.
    if np.ptp(by_axis[0]+by_axis[1]) > 3:
        return None
    return tuple(float(np.median(g)) for g in by_axis)


def review(path):
    meta = json.loads((path/'metadata.json').read_text())
    if not meta.get('round_reference_token'):
        raise ValueError('no formal start token')
    reference = next(d['reference'] for d in records(path/'lidar_worker_events.jsonl')
                     if d.get('event') == 'field_reference_set' and
                     d.get('reference',{}).get('round_token') == meta['round_reference_token'])
    start = reference['timestamp_monotonic_ns']
    next_reference = min((d['reference']['timestamp_monotonic_ns']
        for d in records(path/'lidar_worker_events.jsonl') if d.get('event') == 'field_reference_set'
        and d['reference']['timestamp_monotonic_ns'] > start), default=float('inf'))
    fits = {d['reference_ns']: d for d in records(path/'dynamic_localization.jsonl')
            if start <= d['reference_ns'] < next_reference}
    rates, speeds = [], []
    for d in records(path/'motion_feedback.jsonl'):
        if d.get('type') == 8:
            rates.append((d['receive_monotonic_ns'], abs(d['rate_mdps'])/1000))
        elif d.get('type') == 7:
            speeds.append((d['receive_monotonic_ns'], math.hypot(d['left_um_s'], d['forward_um_s'])/1e6))
    rates, speeds = np.asarray(rates), np.asarray(speeds)
    def quiet(arr,t,limit):
        if not len(arr):
            return False
        a,b=np.searchsorted(arr[:,0], [t-150_000_000,t+150_000_000])
        return b-a>=20 and np.max(arr[a:b,1])<=limit
    stats=Counter()
    samples=[]
    for scan in records(path/'dynamic_scans.jsonl'):
        t=scan['reference_ns']
        fit=fits.get(t)
        if not fit or fit['scan_id']!=scan['scan_id']:
            continue
        stats['matched']+=1
        if not (quiet(rates,t,3) and quiet(speeds,t,.06)):
            continue
        stats['quiet']+=1
        angles=wall_angles(scan,fit,meta['lidar_installation'])
        if angles:
            samples.append(((t-start)/1e9,*angles,*fit['prior_center_xy_m'],fit['primary_yaw_field_rad']))
    result=dict(session=path.name, sdk_session=meta['sdk_session_id'], **stats,
                two_wall_samples=len(samples))
    if samples:
        arr=np.asarray(samples)
        correction=arr[:,1:3].mean(axis=1)
        result.update(seconds=[round(v,2) for v in (arr[0,0],arr[-1,0])],
            correction_deg_median=round(float(np.median(correction)),3),
            inter_axis_difference_deg_p90=round(float(np.percentile(abs(arr[:,1]-arr[:,2]),90)),3))
        # Give signed per-window offsets; do not claim changing views prove gyro drift.
        result['windows']=[]
        for slot in np.unique((arr[:,0]//30).astype(int)):
            mask=(arr[:,0]//30)==slot
            result['windows'].append(dict(start_s=int(slot)*30, n=int(mask.sum()),
                median_deg=round(float(np.median(correction[mask])),3)))
        # Compare actual revisits with similar map position and heading. Use
        # medians of stationary observations rather than adjacent noisy scans.
        stops=np.split(np.arange(len(arr)), np.flatnonzero(np.diff(arr[:,0])>.5)+1)
        stops=[group for group in stops if len(group)>=3]
        pairs=[]
        for i, now in enumerate(stops):
            for prev in stops[:i]:
                dt=float(np.median(arr[now,0])-np.median(arr[prev,0]))
                if dt<30:
                    continue
                pos=np.linalg.norm(np.median(arr[now,3:5],axis=0)-np.median(arr[prev,3:5],axis=0))
                heading=math.degrees(np.median(arr[now,5])-np.median(arr[prev,5]))
                if pos<=.10 and abs((heading+180)%360-180)<=5:
                    pairs.append((dt,60*float(np.median(correction[now])-np.median(correction[prev]))/dt))
        result['comparable_revisit_pairs']=len(pairs)
        if pairs:
            dt,rate=max(pairs)
            result['longest_revisit_interval_s']=round(dt,2)
            result['wall_relative_correction_change_deg_per_min']=round(rate,3)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session',type=Path,nargs='+')
    for run in parser.parse_args().session:
        print(json.dumps(review(run),ensure_ascii=False))
