"""Explicit offline F405 wheel/gyro interval comparison; never a runtime policy.

The raw-count sign convention is the current F405 convention. The effective
rotation arm is an uncalibrated mechanical assumption, not the wheel radius.
A residual cannot identify a slipping
wheel, prove ground motion, or detect every translational slip.
"""
from bisect import bisect_left
from collections import Counter, defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path

METRES_PER_RAW_COUNT = math.pi*.070/1061.268
DEFAULT_ARM_M = .13


def _number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('not a finite number')
    return value


def _group(packet):
    boot=packet['boot_epoch'];ref=packet['reference_epoch']
    if not isinstance(boot,str) or not boot or not isinstance(ref,int):
        raise ValueError('missing epoch')
    return boot,ref


class _Series:
    def __init__(self, rows, counters):
        self.rows=[]
        for row in sorted(rows):
            if self.rows and row[0]==self.rows[-1][0]:
                counters['duplicate_source_time']+=1
                if row[1]!=self.rows[-1][1]:
                    counters['conflicting_source_time']+=1
                    self.rows[-1]=(row[0],self.rows[-1][1],False)
            else:self.rows.append(row)
        self.times=[row[0] for row in self.rows]

    def interval(self, start, end, *, integrate=False):
        if not self.rows or start<self.times[0] or end>self.times[-1]:
            return None,'outside_source_coverage'
        def endpoint(t):
            i=bisect_left(self.times,t)
            if self.times[i]==t:return self.rows[i][1],self.rows[i][2]
            a,b=self.rows[i-1],self.rows[i]
            return a[1]+(b[1]-a[1])*(t-a[0])/(b[0]-a[0]),a[2] and b[2]
        a,valid_a=endpoint(start);b,valid_b=endpoint(end)
        middle=[row for row in self.rows[bisect_left(self.times,start):bisect_left(self.times,end)]
                if start<row[0]<end]
        if not valid_a or not valid_b or not all(row[2] for row in middle):
            return None,'source_flags_or_duplicate_time_invalid'
        if not integrate:return b-a,'covered'
        points=[(start,a)]+[(row[0],row[1]) for row in middle]+[(end,b)]
        return sum((y0+y1)*.5*(t1-t0)/1e6 for (t0,y0),(t1,y1) in zip(points,points[1:])),'covered'


def _statistics(values):
    values=sorted(values)
    if not values:return dict(count=0,min=None,max=None,mean=None,rms=None,median=None,p95_abs=None)
    def percentile(rows, fraction):
        index=(len(rows)-1)*fraction;lo=int(index);hi=min(lo+1,len(rows)-1)
        return rows[lo]+(rows[hi]-rows[lo])*(index-lo)
    return dict(count=len(values),min=values[0],max=values[-1],mean=sum(values)/len(values),
        rms=math.sqrt(sum(v*v for v in values)/len(values)),median=percentile(values,.5),
        p95_abs=percentile(sorted(abs(v) for v in values),.95))


def analyze_records(records, arm_m=DEFAULT_ARM_M):
    """Compare retained raw packets. Coverage failures are offline limitations only."""
    if not math.isfinite(arm_m) or arm_m<=0:raise ValueError('effective wheel arm must be positive and finite')
    counts=Counter();flags=defaultdict(Counter);groups=defaultdict(lambda:dict(wheels=[],yaw=[],rate=[]))
    wheel_ids={};source_ids={'yaw':{},'rate':{}}
    for packet in records:
        counts['raw_records']+=1
        if not isinstance(packet,dict):counts['malformed_records']+=1;continue
        kind=packet.get('type');counts['type_'+str(kind)]+=1
        if kind not in (7,8):continue
        try:
            key=_group(packet);group=groups[key]
            if kind==7:
                identity=(key,int(_number(packet['measurement_id'])))
                start,end=(_number(packet[k]) for k in ('t0_us','t1_us'))
                deltas=tuple(_number(packet['delta'+str(i)]) for i in (1,2,3))
                flag=int(_number(packet['flags']));flags['wheel'][str(flag)]+=1
                signature=(start,end,*deltas)
                if identity in wheel_ids:
                    counts['duplicate_wheel_id']+=1
                    if wheel_ids[identity]!=signature:counts['conflicting_wheel_id']+=1
                    continue
                wheel_ids[identity]=signature
                if start>=end:counts['invalid_wheel_interval']+=1;continue
                valid=bool(flag&1) and not bool(flag&(8|16|32))
                if not valid:counts['invalid_wheel_flags']+=1
                group['wheels'].append(dict(id=identity[1],start=start,end=end,deltas=deltas,
                    flags=flag,valid=valid,receive=packet.get('receive_monotonic_ns')))
            else:
                for source,field,scale in (('yaw','yaw_mdeg',.001),('rate','rate_mdps',.001)):
                    tick=_number(packet[source+'_receive_us']);value=_number(packet[field])*scale
                    flag=int(_number(packet[source+'_flags']));flags[source][str(flag)]+=1
                    identity=(key,int(_number(packet[source+'_source_id'])))
                    if identity in source_ids[source]:
                        counts['duplicate_'+source+'_source_id']+=1
                        if source_ids[source][identity]!=(tick,value):counts['conflicting_'+source+'_source_id']+=1
                        continue
                    source_ids[source][identity]=(tick,value)
                    # Current 1.3 representable-source rule; no READY/age/time-bound gate.
                    valid=bool(flag&1) and not bool(flag&32)
                    if not valid:counts['invalid_'+source+'_flags']+=1
                    group[source].append((tick,value,valid))
        except (KeyError,TypeError,ValueError,OverflowError):counts['malformed_records']+=1
    rows=[];group_summary=[]
    for (boot,ref),group in groups.items():
        series={name:_Series(group[name],counts) for name in ('yaw','rate')}
        blocks=[];block=[];previous=None
        for wheel in group['wheels']:
            if previous is not None and (((wheel['id']-previous['id'])&0xffffffff)!=1 or wheel['start']!=previous['end']):
                counts['wheel_continuity_breaks']+=1
                if block:blocks.append(block)
                block=[]
            block.append(wheel);previous=wheel
        if block:blocks.append(block)
        def compare(samples,window,block_id,partial=False):
            start,end=samples[0]['start'],samples[-1]['end'];dt=(end-start)/1e6
            d=[sum(w['deltas'][i] for w in samples) for i in range(3)]
            encoder_deg=math.degrees(METRES_PER_RAW_COUNT*sum(d)/(3*arm_m))
            yaw,status_yaw=series['yaw'].interval(start,end)
            rate,status_rate=series['rate'].interval(start,end,integrate=True)
            valid=all(w['valid'] for w in samples)
            if not valid:yaw=rate=None;status_yaw=status_rate='wheel_flags_invalid'
            return dict(window=window,boot_epoch=boot,reference_epoch=ref,continuous_block=block_id,
                t0_us=start,t1_us=end,duration_s=dt,interval_count=len(samples),
                first_wheel_id=samples[0]['id'],last_wheel_id=samples[-1]['id'],partial_window=partial,
                delta1=d[0],delta2=d[1],delta3=d[2],wheel_flags=sorted(set(w['flags'] for w in samples)),
                wheel_value_valid=valid,encoder_delta_deg=encoder_deg,encoder_omega_deg_s=encoder_deg/dt,
                yaw_delta_deg=yaw,yaw_omega_deg_s=None if yaw is None else yaw/dt,
                rate_integral_deg=rate,rate_mean_deg_s=None if rate is None else rate/dt,
                residual_yaw_deg_s=None if yaw is None else (encoder_deg-yaw)/dt,
                residual_rate_deg_s=None if rate is None else (encoder_deg-rate)/dt,
                yaw_comparison=status_yaw,rate_comparison=status_rate,
                vx_m_s=METRES_PER_RAW_COUNT*(d[0]-d[1])/(math.sqrt(3)*dt),
                vy_m_s=METRES_PER_RAW_COUNT*(d[0]+d[1]-2*d[2])/(3*dt),
                receive_monotonic_ns_first=samples[0]['receive'],receive_monotonic_ns_last=samples[-1]['receive'])
        for block_id,block in enumerate(blocks):
            rows.extend(compare([wheel],'raw_interval',block_id) for wheel in block)
            for target in (50000,100000):
                batch=[]
                for i,wheel in enumerate(block):
                    batch.append(wheel);duration=wheel['end']-batch[0]['start']
                    following=block[i+1]['end']-batch[0]['start'] if i+1<len(block) else None
                    if duration>=target or (following is not None and following>target and target-duration<=following-target):
                        rows.append(compare(batch,str(target//1000)+'ms',block_id));batch=[]
                if batch:rows.append(compare(batch,str(target//1000)+'ms',block_id,partial=True))
        wheels=group['wheels']
        totals=[sum(w['deltas'][i] for w in wheels) for i in range(3)]
        group_summary.append(dict(boot_epoch=boot,reference_epoch=ref,wheel_intervals=len(wheels),
            yaw_sources=len(group['yaw']),rate_sources=len(group['rate']),continuous_blocks=len(blocks),
            wheel_span_s=(max(w['end'] for w in wheels)-min(w['start'] for w in wheels))/1e6 if wheels else 0,
            summed_interval_s=sum((w['end']-w['start'])/1e6 for w in wheels),
            nonzero_wheel_intervals=sum(any(w['deltas']) for w in wheels),raw_delta_sum=totals,
            encoder_delta_deg=math.degrees(METRES_PER_RAW_COUNT*sum(totals)/(3*arm_m))))
    windows={}
    for name in ('raw_interval','50ms','100ms'):
        selected=[row for row in rows if row['window']==name]
        nonpartial=[row for row in selected if not row['partial_window']]
        yaw_values=[row['residual_yaw_deg_s'] for row in nonpartial if row['residual_yaw_deg_s'] is not None]
        rate_values=[row['residual_rate_deg_s'] for row in nonpartial if row['residual_rate_deg_s'] is not None]
        windows[name]=dict(rows=len(selected),partial_windows=sum(row['partial_window'] for row in selected),
            summary_includes_partial=True,
            covered_count=dict(yaw=sum(row['yaw_delta_deg'] is not None for row in selected),
                               rate=sum(row['rate_integral_deg'] is not None for row in selected)),
            covered_nonpartial=dict(rows=len(nonpartial),yaw_count=len(yaw_values),rate_count=len(rate_values),
                residual_yaw_deg_s=_statistics(yaw_values),residual_rate_deg_s=_statistics(rate_values)),
            interval_duration_s=_statistics([row['duration_s'] for row in selected]),
            encoder_omega_deg_s=_statistics([row['encoder_omega_deg_s'] for row in selected]),
            residual_yaw_deg_s=_statistics([row['residual_yaw_deg_s'] for row in selected if row['residual_yaw_deg_s'] is not None]),
            residual_rate_deg_s=_statistics([row['residual_rate_deg_s'] for row in selected if row['residual_rate_deg_s'] is not None]),
            yaw_comparison=dict(Counter(row['yaw_comparison'] for row in selected)),
            rate_comparison=dict(Counter(row['rate_comparison'] for row in selected)))
    for name in ('duplicate_wheel_id','conflicting_wheel_id','duplicate_yaw_source_id','duplicate_rate_source_id',
                 'conflicting_yaw_source_id','conflicting_rate_source_id','duplicate_source_time',
                 'conflicting_source_time','wheel_continuity_breaks','invalid_wheel_interval',
                 'invalid_wheel_flags','invalid_yaw_flags','invalid_rate_flags','malformed_records'):
        counts.setdefault(name,0)
    report=dict(schema_version=1,analysis='offline_wheel_gyro_residual',runtime_connected=False,
        model=dict(metres_per_raw_count=METRES_PER_RAW_COUNT,effective_wheel_arm_m=arm_m,arm_calibrated=False,
            metres_per_raw_count_formula='pi * 0.070 / 1061.268',
            axes='X forward / Y left / CCW positive',count_sign='raw F405, no second encoder negation',
            encoder_angle='k*(delta1+delta2+delta3)/(3*L)',residual='encoder minus gyro, degrees/second'),
        alignment=dict(clock='same boot/reference epoch MCU event microseconds',
            yaw='cumulative yaw linear interpolation at t0/t1',rate='piecewise-linear trapezoid over the same t0/t1',
            extrapolation=False,missing_coverage='offline comparison unavailable; no online admission effect'),
        flags_interpretation='Current F405 telemetry 1.3 representable-value flags only; not all historical protocol versions.',
        counts=dict(counts),flags={key:dict(value) for key,value in flags.items()},groups=group_summary,windows=windows,
        data_duration_s=sum(group['wheel_span_s'] for group in group_summary),
        summed_wheel_interval_s=sum(group['summed_interval_s'] for group in group_summary),
        limitations=['No slip threshold, radius/delay fit, detection accuracy or runtime policy is produced.',
            'Yaw and rate may come from the same gyro; neither is independent ground truth.',
            'Near-zero rotational residual cannot rule out translational slip against a wall.',
            'The residual cannot uniquely identify a bad wheel or reconstruct true XY.',
            'MCU receive events are not guaranteed physical acquisition timestamps.'])
    return report,rows


def analyze_session(session, arm_m=DEFAULT_ARM_M):
    session=Path(session).resolve()
    source=next((session/name for name in ('motion_feedback.jsonl','chassis_extended.jsonl')
                 if (session/name).is_file() and (session/name).stat().st_size),None)
    if source is None:raise FileNotFoundError('No motion_feedback.jsonl or chassis_extended.jsonl in '+str(session))
    records=[];bad=0;lines=0;digest=hashlib.sha256()
    with source.open('rb') as stream:
        for line in stream:
            digest.update(line);lines+=1
            try:records.append(json.loads(line))
            except (ValueError,UnicodeDecodeError):bad+=1
    report,rows=analyze_records(records,arm_m)
    files={source.name:dict(sha256=digest.hexdigest(),bytes=source.stat().st_size)};attachments={}
    for name in ('metadata.json','chassis_identity.json','session_complete.json'):
        path=session/name
        if not path.is_file():continue
        raw=path.read_bytes();files[name]=dict(sha256=hashlib.sha256(raw).hexdigest(),bytes=len(raw))
        try:attachments[name]=json.loads(raw)
        except (ValueError,UnicodeDecodeError):attachments[name]=dict(read_error='invalid JSON')
    metadata=attachments.get('metadata.json',{});identity=attachments.get('chassis_identity.json',{})
    report.update(session=str(session),source_file=source.name,source_files=files,
        actual_identity={key:identity[key] for key in ('git_commit','source_dirty','board','mode','service',
            'protocol_major','protocol_minor','uart_baud','boot_epoch') if key in identity},
        metadata={key:metadata[key] for key in ('actual_lower_firmware','actual_lower_mode','upper_git_commit',
            'wheel_input_requested','pose_source','mode') if key in metadata},
        session_complete=attachments.get('session_complete.json'),
        ground_truth_present=(session/'ground_truth.jsonl').is_file(),
        context_files_present=[name for name in ('commands.jsonl','chassis_actions.jsonl','events.jsonl') if (session/name).is_file()])
    report['counts'].update(source_lines=lines,bad_json_lines=bad)
    return report,rows


def export_report(session, output, report, rows):
    session=Path(session).resolve();output=Path(output).resolve()
    if output==session or session in output.parents or output in session.parents:
        raise ValueError('Output must be outside the original session and its ancestors')
    json_path=output/'wheel_gyro_residual.json';csv_path=output/'wheel_gyro_residual.csv'
    if json_path.exists() or csv_path.exists():raise FileExistsError('Analysis output already exists: '+str(output))
    output.mkdir(parents=True,exist_ok=True)
    with json_path.open('x',encoding='utf-8') as stream:
        json.dump(report,stream,ensure_ascii=False,indent=2,allow_nan=False);stream.write('\n')
    with csv_path.open('x',encoding='utf-8',newline='') as stream:
        if rows:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    return json_path,csv_path
