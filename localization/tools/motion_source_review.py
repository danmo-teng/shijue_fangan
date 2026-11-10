"""Offline candidates for SENSOR-ALIGN-01 / WHEEL-ARBITRATION-02 / T265-BASE-01.

Only the existing t265_f407_debug offline CLI imports this module. It has no
device, motor, SDK, runtime-policy or configuration writer. Historical walls
and the SDK are correlated observations, never independent ground truth.
"""
from bisect import bisect_right
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from wheel_gyro_residual import METRES_PER_RAW_COUNT, _statistics


def _jsonl(path, counts):
    if not path.is_file():return []
    result=[]
    with path.open() as stream:
        for line in stream:
            try:
                row=json.loads(line)
                if not isinstance(row,dict):raise ValueError('not an object')
                result.append(row)
            except (ValueError,UnicodeDecodeError):counts[path.name+'_invalid_json']+=1
    return result


class _History:
    """Offline interpolation with explicit frame/interval coverage, no age test."""
    def __init__(self, times, values, breaks=None):
        self.t=np.asarray(times,dtype=np.int64)
        self.v=np.asarray(values,dtype=float)
        self.breaks=np.asarray([] if breaks is None else breaks,dtype=np.int64)
        if len(self.t)<2 or np.any(np.diff(self.t)<=0):
            raise ValueError('Need strictly increasing retained source support')
        self.seconds=(self.t-self.t[0])/1e9

    def covers(self,a,b):
        if a<self.t[0] or b>self.t[-1]:return False
        lo=np.searchsorted(self.breaks,a,side='right')
        return lo==len(self.breaks) or self.breaks[lo]>b

    def at(self, queries):
        q=np.asarray(queries,dtype=np.int64)
        # Subtract the integer origin before converting to float seconds.
        t=self.seconds;x=(q-self.t[0])/1e9
        values=self.v[:,None] if self.v.ndim==1 else self.v
        result=np.column_stack([np.interp(x,t,values[:,i]) for i in range(values.shape[1])])
        return result[:,0] if self.v.ndim==1 else result


def _rotation(yaw):
    c,s=np.cos(yaw),np.sin(yaw)
    return np.array([[c,-s],[s,c]])


def _integrated(times, velocities, breaks=()):
    times=np.asarray(times,dtype=np.int64);v=np.asarray(velocities,float)
    delta=(v[:-1]+v[1:])*.5*((np.diff(times)/1e9)[:,None])
    for stamp in breaks:
        i=np.searchsorted(times,stamp)
        if 0<i<len(times):delta[i-1]=0.  # Unknown motion is not fabricated.
    return _History(times,np.vstack([np.zeros(2),np.cumsum(delta,axis=0)]),breaks)


def _axis(text):
    if text in ('x','+x','-x','y','+y','-y','z','+z','-z'):
        result=np.zeros(3);result['xyz'.index(text[-1])]=-1. if text[0]=='-' else 1.
        return result
    result=np.asarray([float(x) for x in text.split(',')])
    if result.shape!=(3,):raise ValueError('Saved axis must have three components')
    return result/np.linalg.norm(result)


def _sdk_velocity(raw, forward, up, offset, initial_yaw):
    """Full quaternion and omega-cross-r, then the SAME fixed XZ field plane.

    Do not rotate planar body speed with yaw alone over tilted/obstacle frames.
    SDK pose velocity/omega are in its world axes. No internal IMU offset.
    """
    q=np.array([[float(r[k]) for k in ('qx','qy','qz','qw')] for r in raw])
    norms=np.linalg.norm(q,axis=1);zero=norms<1e-12
    q[zero]=[0.,0.,0.,1.];norms[zero]=1.
    q=q/norms[:,None]
    def rotated(v):
        t=2*np.cross(q[:,:3],v)
        return np.asarray(v)+q[:,3,None]*t+np.cross(q[:,:3],t)
    f=rotated(forward);l=rotated(np.cross(up,forward))
    lever=offset[0]*f+offset[1]*l
    omega=np.array([[float(r[k]) for k in ('wx_radps','wy_radps','wz_radps')] for r in raw])
    velocity=np.array([[float(r[k]) for k in ('vx_mps','vy_mps','vz_mps')] for r in raw])
    center=velocity-np.cross(omega,lever)
    initial_forward=f[0].copy();initial_forward[1]=0.
    initial_forward/=np.linalg.norm(initial_forward)
    initial_left=np.cross([0.,1.,0.],initial_forward)
    return np.column_stack([center@initial_forward,center@initial_left])@_rotation(initial_yaw).T


def _summary(rows, key):
    return _statistics([float(r[key]) for r in rows if r.get(key) is not None])


def review_session(session, *, gyro_phase_ms=28., arm_m=.115, include_scans=True):
    """Replay candidate increments; never promote them to the live chain.

    28ms/115mm are fixed candidates from the earlier, separate analysis. No
    delay, rotation, scale or lever fitting is performed on these inputs.
    """
    session=Path(session).resolve()
    if not math.isfinite(gyro_phase_ms) or not math.isfinite(arm_m) or arm_m<=0:
        raise ValueError('Finite phase and positive finite arm required')
    counts=Counter();used=[]
    def read_jsonl(name):
        path=session/name
        if path.is_file():used.append(path)
        return _jsonl(path,counts)
    config_path=session/'localization.conf';used.append(config_path);config={}
    for line in config_path.read_text().splitlines():
        line=line.split('#',1)[0]
        if '=' in line:
            k,v=line.split('=',1);config[k.strip()]=v.strip()
    forward=_axis(config['camera_robot_forward_axis']);up=_axis(config['camera_robot_up_axis'])
    offset=[float(config[k]) for k in ('camera_offset_forward_m','camera_offset_left_m')]
    metadata_path=session/'metadata.json';used.append(metadata_path)
    metadata=json.loads(metadata_path.read_text())
    source=next((name for name in ('motion_feedback.jsonl','chassis_extended.jsonl')
                 if (session/name).is_file() and (session/name).stat().st_size),None)
    if source is None:raise ValueError('No retained 07/08 feedback')
    packets=[r for r in read_jsonl(source) if r.get('type') in (7,8)]
    epochs={(p.get('boot_epoch'),p.get('reference_epoch')) for p in packets}
    if len(epochs)!=1:raise ValueError('Review each boot/reference epoch separately; no implicit joining')
    clock_ns=int(np.median([int(p['receive_monotonic_ns'])-int(p['publish_us'])*1000
                            for p in packets if p.get('publish_us') is not None]))
    counts['feedback_records']=len(packets)
    gyro=[];wheels=[];seen_yaw=set();seen_wheel=set()
    for p in packets:
        if p['type']==8:
            if p['yaw_source_id'] in seen_yaw:continue
            seen_yaw.add(p['yaw_source_id'])
            if not (p['yaw_flags']&1) or p['yaw_flags']&32:
                counts['gyro_value_unavailable']+=1;continue
            gyro.append((int(p['yaw_receive_us'])*1000+clock_ns,math.radians(p['yaw_mdeg']/1000),p))
        else:
            if p['measurement_id'] in seen_wheel:continue
            seen_wheel.add(p['measurement_id'])
            if not(p['flags']&1) or p['flags']&(8|16|32) or p['t0_us']>=p['t1_us']:
                counts['wheel_value_unavailable']+=1;continue
            wheels.append(p)
    gyro.sort(key=lambda x:x[0]);gyro=[r for i,r in enumerate(gyro) if i==0 or r[0]>gyro[i-1][0]]
    gh=_History([r[0] for r in gyro],[r[1] for r in gyro])
    # Arrival order, separate from source time. Only packets received by a
    # wheel callback may enter the causal candidate; future history cannot.
    arrivals=sorted((int(r[2]['receive_monotonic_ns']),r[0],r[1],r[2]) for r in gyro)
    arrival_times=[r[0] for r in arrivals]
    projected={int(p['frame_number']):p for p in read_jsonl('pose_motion.jsonl')}
    csv_path=session/'localization_debug.csv';used.append(csv_path)
    with csv_path.open(newline='') as f:debug={int(r['pose_frame_number']):r for r in csv.DictReader(f)}
    raw_path=session/'t265_raw_pose.csv';used.append(raw_path)
    with raw_path.open(newline='') as f:raw=list(csv.DictReader(f))
    first_debug=next(iter(debug.values()))
    initial_yaw=math.radians(float(first_debug['t265_yaw_deg'])-float(first_debug['relative_yaw_deg']))
    all_velocities=_sdk_velocity(raw,forward,up,offset,initial_yaw)
    sdk=[];breaks=[];last_raw=None
    for rawrow,velocity in zip(raw,all_velocities):
        frame=int(rawrow['frame_number']);row=debug.get(frame);p=projected.get(frame)
        # capture_age was computed on the consumer, not in the SDK callback.
        # Subtract it from the recorded processing time, never callback RX.
        # When retained, the actual published capture conversion is preferable.
        t=(int(p['timestamp_monotonic_ns']) if p is not None else
           int(rawrow['processing_monotonic_ns'])-int(round(float(rawrow['capture_age_ms'])*1e6)))
        valid=int(rawrow['tracker_confidence'])>0 and row is not None
        if valid:
            position=[float(row[k]) for k in ('t265_x_m','t265_y_m')]
            yaw=math.radians(float(row['t265_yaw_deg']))
            valid=all(math.isfinite(x) for x in (*position,yaw,*velocity))
        if not valid:
            counts['sdk_value_unavailable']+=1;last_raw=None;continue
        if sdk and t<=sdk[-1][0]:counts['sdk_nonmonotonic']+=1;last_raw=None;continue
        if sdk and (last_raw is None or frame!=last_raw+1):breaks.append(t)
        sdk.append((t,*position,yaw,*velocity,frame));last_raw=frame
    counts['raw_sdk_rows']=len(raw);counts['sdk_retained_rows']=len(sdk);counts['sdk_frame_breaks']=len(breaks)
    a=np.asarray(sdk,float);sdk_times=np.asarray([r[0] for r in sdk],dtype=np.int64)
    pos=_History(sdk_times,a[:,1:3],breaks);sy=_History(sdk_times,np.unwrap(a[:,3]),breaks)
    sv=_integrated(sdk_times,a[:,4:6],breaks)
    # Default local SDK frame, replaced with the actual formal reference when
    # present. Neither the debugger's centered start nor an inferred origin is
    # written back to a competition configuration.
    control=read_jsonl('control_pose.jsonl')
    reference=next((p['reference'] for p in control if isinstance(p.get('reference'),dict)),None)
    if reference is None:
        reference=dict(vio_rotation=0.,pose_origin=list(a[0,1:3]),start_center_xy_m=[0.,0.],
                       yaw_offset=float(a[0,3]-gh.at([sdk_times[0]])[0]))
    rotate=_rotation(reference['vio_rotation']);origin=np.asarray(reference['pose_origin'])
    start=np.asarray(reference['start_center_xy_m']);yaw_offset=reference['yaw_offset']
    phase_ns=int(round(gyro_phase_ms*1e6))
    wheel_times=[];raw_xy=[];aligned_xy=[];wheel_breaks=[];previous=None
    wheel_raw=np.zeros(2);wheel_aligned=np.zeros(2);sync=[]
    for w in wheels:
        t0=int(w['t0_us'])*1000+clock_ns;t1=int(w['t1_us'])*1000+clock_ns;dt=(t1-t0)/1e9
        if not gh.covers(t0,t1+max(0,phase_ns)) or t0+phase_ns<gh.t[0]:
            counts['wheel_gyro_common_coverage_unavailable']+=1;continue
        ds=np.array([w['delta1'],w['delta2'],w['delta3']])*METRES_PER_RAW_COUNT
        # Position uses the actual lower board's published measured velocity.
        # Raw counts provide a separate wheel angular diagnostic only.
        v=np.array([w['forward_um_s'],w['left_um_s']],float)/1e6
        theta=gh.at([t0,t1]);aligned=gh.at([t0+phase_ns,t1+phase_ns])
        if not wheel_times:
            wheel_times.append(t0);raw_xy.append(wheel_raw.copy());aligned_xy.append(wheel_aligned.copy())
        if previous is not None and (w['t0_us']!=previous['t1_us'] or ((w['measurement_id']-previous['measurement_id'])&0xffffffff)!=1):
            wheel_breaks.append(t1);counts['wheel_interval_breaks']+=1
        wheel_raw=wheel_raw+_rotation(float(theta.mean()+yaw_offset))@v*dt
        wheel_aligned=wheel_aligned+_rotation(float(aligned.mean()+yaw_offset))@v*dt
        if t1<=wheel_times[-1]:counts['wheel_nonmonotonic']+=1;continue
        wheel_times.append(t1);raw_xy.append(wheel_raw.copy());aligned_xy.append(wheel_aligned.copy());previous=w
        encoder_delta=float(ds.sum()/(3*arm_m))
        row=dict(t0_ns=t0,t1_ns=t1,measurement_id=w['measurement_id'],duration_s=dt,
                 encoder_omega_deg_s=math.degrees(encoder_delta/dt),
                 residual_raw_deg_s=math.degrees((encoder_delta-(theta[1]-theta[0]))/dt),
                 residual_aligned_deg_s=math.degrees((encoder_delta-(aligned[1]-aligned[0]))/dt))
        j=bisect_right(arrival_times,int(w['receive_monotonic_ns']))-1
        if j>=0:
            _,event_time,y,p=arrivals[j]
            rate_valid=bool(p['rate_flags']&1) and not bool(p['rate_flags']&32)
            if rate_valid:
                rate=math.radians(p['rate_mdps']/1000)
                horizon=(t1-(event_time-phase_ns))/1e9
                prediction=y+rate*horizon
                row.update(causal_prediction_horizon_ms=horizon*1000,
                    held_vs_aligned_yaw_error_deg=math.degrees(y-aligned[1]),
                    predicted_vs_aligned_yaw_error_deg=math.degrees(prediction-aligned[1]))
        sync.append(row)
    wr=_History(wheel_times,raw_xy,wheel_breaks);wa=_History(wheel_times,aligned_xy,wheel_breaks)
    jumps=[sdk_times[i] for i in range(1,len(sdk)) if sdk[i][-1]==sdk[i-1][-1]+1
           and np.linalg.norm(a[i,1:3]-a[i-1,1:3])>.05]
    wall_fits=read_jsonl('dynamic_localization.jsonl');windows=[];baseline=None
    suspect_times=[int(p['timestamp_monotonic_ns']) for p in control
                   if p.get('motion_selection',{}).get('wheel_motion_suspect')]
    dual_events=[];previous_mode=None
    for p in control:
        m=p.get('motion_selection',{});mode=m.get('mode')
        if previous_mode=='T265_PRIMARY_WHEEL_SUSPECT' and mode=='WHEEL_FALLBACK' and m.get('wheel_motion_suspect'):
            dual_events.append(int(p['timestamp_monotonic_ns']))
        previous_mode=mode
    counts['sdk_single_steps_over_50mm']=len(jumps)
    counts['recorded_dual_suspect_transitions']=len(dual_events)
    models=('wheel_raw_gyro','wheel_aligned_gyro','sdk_position','sdk_velocity')
    def increments(t0,t1):
        histories=(wr,wa,pos,sv)
        if not all(h.covers(t0,t1) for h in histories):return None
        result={}
        for name,h in zip(models,histories):
            delta=np.diff(h.at([t0,t1]),axis=0)[0]
            result[name]=rotate@delta if name.startswith('sdk') else delta
        return result
    def window_row(t0,xy0,fit,kind):
        t=int(fit['reference_ns']);deltas=increments(t0,t)
        if deltas is None:return None
        wall_delta=np.asarray(fit['candidate_center_xy_m'])-xy0
        motion=fit.get('motion_selection',{});jump_count=bisect_right(jumps,t)-bisect_right(jumps,t0)
        row=dict(window_kind=kind,t0_ns=t0,t1_ns=t,duration_s=(t-t0)/1e9,scan_id=fit['scan_id'],
                 wall_dx_m=float(wall_delta[0]),wall_dy_m=float(wall_delta[1]),sdk_jump_count=jump_count,
                 wheel_suspect=(bisect_right(suspect_times,t)>bisect_right(suspect_times,t0)),
                 phase_wheel_suppressed=bool(motion.get('phase_wheel_suppressed')),
                 recorded_source=motion.get('mode'),wall_rms_m=fit.get('residual_rms_m'))
        for name,delta in deltas.items():
            row[name+'_dx_m']=float(delta[0]);row[name+'_dy_m']=float(delta[1])
            row[name+'_wall_error_m']=float(np.linalg.norm(delta-wall_delta))
        return row
    for fit in wall_fits:
        if set(fit.get('position_observed_axes',[]))!={'x','y'}:
            counts['wall_not_full_xy']+=1;baseline=None;continue
        if fit.get('position_reacquired'):
            counts['wall_reacquisition_baseline_reset']+=1;baseline=None;continue
        t=int(fit['reference_ns']);xy=np.array(fit['candidate_center_xy_m'],float)
        if baseline is None:baseline=(t,xy);continue
        t0,xy0=baseline;deltas=increments(t0,t)
        if deltas is None:counts['wall_window_common_coverage_unavailable']+=1;baseline=(t,xy);continue
        wall_delta=xy-xy0
        if max(np.linalg.norm(wall_delta),*(np.linalg.norm(v) for v in deltas.values()))<.25:continue
        windows.append(window_row(t0,xy0,fit,'spatial'));baseline=(t,xy)
    full_fits=[f for f in wall_fits if set(f.get('position_observed_axes',[]))=={'x','y'}]
    fit_times=[int(f['reference_ns']) for f in full_fits]
    for event_ns in dual_events:
        i=bisect_right(fit_times,event_ns)
        if i==0 or i==len(full_fits):counts['dual_event_wall_pair_unavailable']+=1;continue
        before=full_fits[i-1]
        # Both immediate post-event wall observations are retained; no best
        # looking endpoint selection, no distance/angle command used as truth.
        for j,fit in enumerate(full_fits[i:i+2],1):
            row=window_row(int(before['reference_ns']),np.array(before['candidate_center_xy_m']),fit,'dual_event_wall_'+str(j))
            if row is None:counts['dual_event_common_coverage_unavailable']+=1;continue
            row['dual_event_ns']=event_ns;windows.append(row)
    scan_rows=[]
    if include_scans and metadata.get('lidar_installation'):
        scans=read_jsonl('dynamic_scans.jsonl')
        scan_rows=_review_scans(scans,metadata['lidar_installation'],wr,wa,pos,sv,gh,sy,
                                phase_ns,yaw_offset,reference,counts)
    # Equal-duration residuals, rather than comparing quantized 10ms wheel
    # counts directly with a 100ms statistic from the earlier investigation.
    sync100=[];batch=[]
    for row in sync:
        if batch and row['t0_ns']!=batch[-1]['t1_ns']:batch=[]
        batch.append(row)
        duration=sum(r['duration_s'] for r in batch)
        if duration<.099:continue
        combined=dict(row,t0_ns=batch[0]['t0_ns'],duration_s=duration,window='100ms')
        for key in ('encoder_omega_deg_s','residual_raw_deg_s','residual_aligned_deg_s'):
            combined[key]=sum(r[key]*r['duration_s'] for r in batch)/duration
        sync100.append(combined);batch=[]
    for row in sync:row['window']='raw_interval'
    sync.extend(sync100)
    report=dict(schema_version=1,analysis='offline_motion_source_candidates',runtime_connected=False,
        deployed=False,session=str(session),counts=dict(counts),
        wheel_input_requested=metadata.get('wheel_input_requested'),
        candidates=dict(gyro_phase_ms=gyro_phase_ms,effective_arm_m=arm_m,fitted_on_this_data=False,
            wheel_translation='published measured F405 07 forward/left; raw counts only for angular crosscheck',
            sdk_velocity='full 3D SDK world v minus omega cross lever, projected onto original fixed ground plane',
            history_alignment_uses_future_gyro=True,causal_prediction_uses_only_received_packets=True,
            defaults_or_runtime_sources_modified=False),
        synchronization={group:{key:_summary([r for r in sync if r['window']==group],key) for key in
            ('residual_raw_deg_s','residual_aligned_deg_s','held_vs_aligned_yaw_error_deg',
             'predicted_vs_aligned_yaw_error_deg','causal_prediction_horizon_ms')}
            for group in ('raw_interval','100ms')},
        synchronization_active_rotation={key:_summary([r for r in sync if r['window']=='100ms' and
                max(abs(r['encoder_omega_deg_s']),abs(r['encoder_omega_deg_s']-r['residual_aligned_deg_s']))>=5.],key)
            for key in ('residual_raw_deg_s','residual_aligned_deg_s','held_vs_aligned_yaw_error_deg','predicted_vs_aligned_yaw_error_deg')},
        wall_windows={group:{name:_summary([r for r in windows if predicate(r)],name+'_wall_error_m') for name in models}
            for group,predicate in [('all',lambda r:r['window_kind']=='spatial'),
                ('no_sdk_jump',lambda r:r['window_kind']=='spatial' and r['sdk_jump_count']==0),
                ('sdk_jump',lambda r:r['window_kind']=='spatial' and r['sdk_jump_count']>0),
                ('wheel_suspect',lambda r:r['window_kind']=='spatial' and r['wheel_suspect']),
                ('dual_event',lambda r:r['window_kind'].startswith('dual_event'))]},
        scan_metrics={name:{key:_summary([r for r in scan_rows if r['model']==name],key)
                           for key in ('tls_rms_m','axis_rms_m')}
                      for name in sorted({r['model'] for r in scan_rows})},
        source_files={str(p.relative_to(session)):dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in dict.fromkeys(used)},
        limitations=['No independent ground truth, slip labels or detection accuracy; nominal actions are not truth.',
            'Wall windows use accepted two-axis historical fits; their priors/deskew depend on motion and main yaw.',
            'SDK velocity/rotation retain previous wheel-input influence; pause does not clear that state.',
            'A fixed 28ms phase is a comparison candidate, not a calibrated universal sensor delay.',
            'Prediction is compared to later received gyro as a proxy, not an external angle reference.',
            'SDK source time uses retained pose_motion, or processing_monotonic_ns minus consumer capture_age; callback RX is a different clock observation.',
            'Scan metrics freeze supports from raw lines; TLS/axis residuals are proxies, not XY accuracy or independent wall identity.',
            'All raw records stay unchanged. Missing common support only limits offline comparisons.',
            'No online threshold, new timer, age gate, handshake, stop or firmware change is introduced.'])
    return report,windows,sync,scan_rows


def _review_scans(scans,mount,wr,wa,pos,sv,gh,sy,phase_ns,yaw_offset,reference,counts):
    from lidar_yaw_debug.dynamic import deskew_points,accepted_points
    from lidar_yaw_debug.static import extract_lines,outer_supported_lines
    from types import SimpleNamespace
    rows=[];rotation=_rotation(reference['vio_rotation']);origin=np.array(reference['pose_origin'])
    start=np.array(reference['start_center_xy_m']);extrinsic=np.array(mount['extrinsic'],float)
    extrinsic[2]=math.radians(extrinsic[2])
    for scan in scans:
        counts['raw_scans']+=1
        pts=accepted_points(SimpleNamespace(points=[SimpleNamespace(**p) for p in scan['points']]),mount)
        if len(pts)<12:counts['scan_few_accepted_points']+=1;continue
        t0=pts[0].time_ns;t1=int(scan['reference_ns'])
        if not all(h.covers(t0,t1) for h in (wr,wa,pos,sv,sy)) or not gh.covers(t0+min(0,phase_ns),t1+max(0,phase_ns)):
            counts['scan_common_coverage_unavailable']+=1;continue
        xy=np.array([p.point_lidar_m for p in pts]);times=np.array([p.time_ns for p in pts],dtype=np.int64)
        # Freeze supports once in RAW scan coordinates. Reuse exact physical
        # indices for every candidate; never reselect a prettier line per model.
        lines=outer_supported_lines(extract_lines(xy))
        supports=[]
        for line in lines:
            d=np.sum((xy[:,None,:]-np.asarray(line['points'])[None,:,:])**2,axis=2)
            supports.append(np.argmin(d,axis=0))
        if not supports:counts['scan_no_shared_raw_line_support']+=1;continue
        for name,ph,yh,shift,anchor in [('wheel_main',wr,gh,0,False),('wheel_aligned',wa,gh,phase_ns,False),
                ('wheel_sdk_rotation',wr,sy,0,False),('sdk_position',pos,sy,0,False),('sdk_velocity',sv,sy,0,False),
                ('wheel_sdk_increment',wr,sy,0,True),('wheel_sdk_increment_aligned',wa,sy,phase_ns,True),
                ('sdk_velocity_increment_aligned',sv,sy,phase_ns,True)]:
            def trajectory(query):
                p=ph.at(query)
                if ph in (pos,sv):p=(p-origin)@rotation.T+start
                if anchor:
                    # SDK contributes ONLY within-scan rotation. Preserve the
                    # same main-yaw field reference, rather than silently
                    # replacing it with the SDK's absolute startup heading.
                    y=sy.at(query)-sy.at([t1])[0]+gh.at([t1+shift])[0]+yaw_offset
                else:y=yh.at(query+shift)+(yaw_offset if yh is gh else reference['vio_rotation'])
                return np.column_stack([p,y])
            adjusted,ref=deskew_points(xy,times,t1,trajectory,extrinsic)
            # Translation and absolute wall offset are nuisance variables here.
            # TLS checks straightness; fixed-axis residual also checks rotation.
            world=adjusted@_rotation(ref[2]).T;tls=[];axis=[]
            for ix in supports:
                points=world[ix];centered=points-points.mean(axis=0)
                _,_,vh=np.linalg.svd(centered,full_matrices=False)
                tls.extend((centered@vh[-1])**2)
                axis.extend(centered[:,int(np.argmin(np.sum(centered**2,axis=0)))]**2)
            rows.append(dict(scan_id=scan['scan_id'],reference_ns=t1,model=name,
                shared_lines=len(supports),physical_points=len(pts),
                tls_rms_m=math.sqrt(float(np.mean(tls))),axis_rms_m=math.sqrt(float(np.mean(axis)))))
    return rows


def export_review(session,output,result):
    session=Path(session).resolve();output=Path(output).resolve()
    if output==session or session in output.parents or output in session.parents:
        raise ValueError('Output must be outside the source session and its ancestors')
    report,windows,sync,scans=result
    paths=[output/name for name in ('motion_source_review.json','wall_windows.csv','synchronization.csv','scan_candidates.csv')]
    if any(p.exists() for p in paths):raise FileExistsError('Review output exists: '+str(output))
    output.mkdir(parents=True,exist_ok=True)
    with paths[0].open('x') as f:json.dump(report,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
    for path,records in zip(paths[1:],(windows,sync,scans)):
        with path.open('x',newline='') as f:
            fields=sorted({k for r in records for k in r})
            if fields:
                writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(records)
    return paths
