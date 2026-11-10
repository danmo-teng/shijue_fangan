"""Shared exclusive translation source, with measured wheel bridging.

Positions live in the original T265 local frame, before any lidar correction.
No age gate, pose weighting, motor I/O, or extrapolation of SLAM position jumps.
Thresholds are engineering starting values, not calibrated sensor covariance.
"""
from bisect import bisect_left
from collections import deque
import math
import numpy as np


POLICY = dict(single_increment_conflict_m=.05, displacement_conflict_m=.05,
              comparison_travel_m=.25, recovery_residual_m=.025,
              recovery_travel_m=.50, recovery_two_axis_fixes=2)


def sample(history, t):
    """Interpolate actual samples; outside support hold the endpoint, not velocity."""
    if t<=history[0][0]:return np.asarray(history[0][1:],float)
    if t>=history[-1][0]:return np.asarray(history[-1][1:],float)
    i=bisect_left(history,t,key=lambda x:x[0])
    if i==0:return np.asarray(history[0][1:],float)
    if i==len(history):return np.asarray(history[-1][1:],float)
    a,b=history[i-1],history[i];f=(t-a[0])/(b[0]-a[0])
    return np.asarray(a[1:],float)*(1-f)+np.asarray(b[1:],float)*f


def _norm2(vector):
    """Euclidean 2D magnitude without NumPy dot's tiny-array GIL release."""
    return math.hypot(float(vector[0]),float(vector[1]))


class MotionSource:
    def __init__(self):
        self.wheels=deque(maxlen=4096)  # host time, native-yaw XY, actual wheel travel
        self.native_yaws=deque(maxlen=4096)
        self.pending=deque(maxlen=4096)
        self.history=deque(maxlen=4096)  # host time, selected local XY, unused yaw
        self.events=deque(maxlen=128)
        self.reference=None;self.basis=np.eye(2);self.last_wheel=None
        self.anchor=None;self.anchor_wheel=None;self.bias=np.zeros(2)
        self.previous=None;self.window=None;self.recovery_window=None
        self.mode='T265_NO_WHEEL_REFERENCE';self.reason='no_measured_wheel_reference'
        self.recovery_travel=0.;self.wall_fixes=0;self.last_wall=None
        self.last_paired_ns=None;self.last_raw_ns=None;self.last_confidence=None
        self.last_conflict_m=0.;self.wheel_time_basis=None
        self.wheel_yaw_support=None
        self.pending_pose_drops=0
        self.last_raw_xy=None
        self.last_tracked_ns=None;self.last_tracked_xy=None
        self.phase_wheel_suppressed=False;self.phase_wheel_reason=''
        self.resume_anchor_ns=None;self.resume_anchor_xy=None
        self.wheel_suspect=False
        self.wall_window=None;self.wall_evidence=None
        self.evidence_verdict=None;self.evidence_confirmations=0;self.last_evidence_scan=None

    def wheel_input_paused(self):
        return self.phase_wheel_suppressed or self.wheel_suspect

    def wheel_input_reason(self):
        return (self.phase_wheel_reason if self.phase_wheel_suppressed else
                'two_axis_wall_motion_wheel_suspect' if self.wheel_suspect else 'measured_wheel_input_enabled')

    def set_wheel_suppressed(self, suppressed, reason):
        suppressed=bool(suppressed)
        if suppressed==self.phase_wheel_suppressed:
            if suppressed:self.phase_wheel_reason=str(reason);self.reason=str(reason)
            return
        previous_mode=self.mode
        self.phase_wheel_suppressed=suppressed
        self.phase_wheel_reason=str(reason) if suppressed else ''
        selected=(np.asarray(self.history[-1][1:3],float) if self.history else
                  self.last_tracked_xy.copy() if self.last_tracked_xy is not None else None)
        t=self.history[-1][0] if self.history else self.last_tracked_ns
        if selected is not None and self.last_tracked_xy is not None:
            self.bias=selected-self.last_tracked_xy
        self.pending.clear();self.previous=None;self.window=None;self.recovery_window=None
        self.recovery_travel=0.;self.wall_fixes=0;self.last_wall=None
        self.wall_window=None;self.wall_evidence=None
        self.evidence_verdict=None;self.evidence_confirmations=0;self.last_evidence_scan=None
        self.resume_anchor_ns=None;self.resume_anchor_xy=None
        if suppressed:
            self.mode='T265_PHASE_NO_WHEEL';self.reason=str(reason)
            if not self.history and self.reference is not None and self.last_tracked_ns is not None:
                self._publish_vio(self.last_tracked_ns,self.last_tracked_xy)
        elif self.wheel_suspect:
            self.mode='T265_PRIMARY_WHEEL_SUSPECT'
            self.reason='two_axis_walls_support_T265_over_wheel'
        elif self.reference is not None and selected is not None:
            # Wait for actual wheel support when Pose is ahead. The excluded
            # wheel total is an anchor only, never motion to catch up later.
            self.resume_anchor_ns=max(t,self.wheels[-1][0]) if self.wheels else t
            self.resume_anchor_xy=selected.copy()
            self.anchor=None;self.anchor_wheel=None
            self.mode='T265_WITH_WHEEL_TAIL';self.reason='phase_exit_rebased'
            self._process()
        else:
            self.mode='T265_NO_WHEEL_REFERENCE';self.reason='no_measured_wheel_reference'
        self.events.append(dict(event='wheel_phase_suppression_changed',sample_monotonic_ns=t,
            previous_source=previous_mode,motion_source=self.mode,suppressed=suppressed,reason=str(reason),
            selected_anchor_m=self.anchor.tolist() if self.anchor is not None else None,
            wheel_anchor_m=self.anchor_wheel.tolist() if self.anchor_wheel is not None else None))

    def yaw(self, host_ns, yaw_rad, source):
        raw=source.get('raw',{})
        if 'yaw_receive_us' not in raw:return
        t=int(raw['yaw_receive_us'])*1000
        if self.native_yaws and t<self.native_yaws[-1][0]:return
        if self.native_yaws and t==self.native_yaws[-1][0]:self.native_yaws.pop()
        self.native_yaws.append((t,float(yaw_rad)))

    def configure(self, reference):
        self.reference=dict(reference);self.history.clear();self.anchor=None
        self.bias[:]=0;self.previous=None;self.window=None;self.recovery_window=None
        self.recovery_travel=0.;self.wall_fixes=0;self.last_wall=None
        self.last_paired_ns=None;self.mode='T265_NO_WHEEL_REFERENCE'
        self.reason='no_measured_wheel_reference'
        self.wheel_suspect=False;self.wall_window=None;self.wall_evidence=None
        self.evidence_verdict=None;self.evidence_confirmations=0;self.last_evidence_scan=None
        self.resume_anchor_ns=None;self.resume_anchor_xy=None
        angle=reference['yaw_offset']-reference['vio_rotation']
        c,s=math.cos(angle),math.sin(angle);self.basis=np.array([[c,-s],[s,c]])
        t=reference.get('reference_sample_ns',reference.get('timestamp_monotonic_ns',0))
        self.pending=deque((p for p in self.pending if p[0]>=t),maxlen=4096)
        if self.phase_wheel_suppressed:
            self.pending.clear();self.mode='T265_PHASE_NO_WHEEL';self.reason=self.phase_wheel_reason
            if self.last_tracked_ns is not None and self.last_tracked_ns>=t:
                self._publish_vio(self.last_tracked_ns,self.last_tracked_xy)
            return
        self._process()

    def pose(self, p):
        t=int(p['timestamp_monotonic_ns']);xy=np.array([p['x_m'],p['y_m']],float)
        if not np.isfinite(xy).all():return
        if self.last_raw_ns is not None and t<=self.last_raw_ns:return
        raw_step=(_norm2(xy-self.last_raw_xy) if self.mode=='T265_PRIMARY_WHEEL_SUSPECT' and self.last_raw_xy is not None else 0.)
        self.last_raw_ns=t;self.last_raw_xy=xy;self.last_confidence=p['tracker_confidence']
        if p['tracker_confidence']>0:self.last_tracked_ns=t;self.last_tracked_xy=xy.copy()
        if self.phase_wheel_suppressed:
            if p['tracker_confidence']>0 and self.reference is not None:self._publish_vio(t,xy)
            return
        if self.resume_anchor_ns is not None and p['tracker_confidence']>0:
            self._publish_vio(t,xy)
        if len(self.pending)==self.pending.maxlen:self.pending_pose_drops+=1
        self.pending.append((t,*xy,int(p['tracker_confidence'])))
        if p['tracker_confidence']<=0 and self.anchor is not None:
            self._fallback('SDK_tracking_lost',t)
        elif self.mode=='T265_PRIMARY_WHEEL_SUSPECT' and raw_step>POLICY['single_increment_conflict_m']:
            self._fallback('SDK_single_step_jump_while_wheel_suspect',t)
        elif self.mode=='T265_PRIMARY_WHEEL_SUSPECT':
            # An actual new Pose, not a repeated wheel tail or a synthetic
            # 100Hz timestamp. Heading still comes from the main gyro outside.
            self._publish_vio(t,xy)
        self._process()

    def wheel(self, p, host_yaws):
        if (p.get('source')!='measured_wheel_feedback' or not p.get('measurement_valid')
                or p.get('mcu_epoch') is None):return
        raw=p['raw_mcu_sample'];key=(p['mcu_epoch'],p['sample_id'])
        if self.last_wheel and key[0]!=self.last_wheel[0]:return  # Owner invalidates/references explicitly.
        if self.last_wheel and not 0<((key[1]-self.last_wheel[1])&0xffffffff)<0x80000000:return
        a,b=int(raw['t0_us'])*1000,int(raw['t1_us'])*1000
        v=np.asarray(p['v_body_mps'][:2],float)
        if a>=b or not np.isfinite(v).all() or not host_yaws:return
        end=p.get('interval_end_ns');start=p.get('interval_start_ns')
        self.wheel_time_basis='mapped_MCU_event_estimate'
        if start is None or end is None or (self.wheels and end<self.wheels[-1][0]):
            end=int(p['receive_monotonic_ns']);start=end-(b-a)
            self.wheel_time_basis='receive_end_with_native_interval_duration'
        if self.wheels and end<self.wheels[-1][0]:return
        # Primary yaw and wheel averaging interval share MCU ticks. Endpoints
        # outside the received yaw support hold the actual nearest yaw value.
        if self.native_yaws:
            ya=sample(self.native_yaws,a)[0];yb=sample(self.native_yaws,b)[0]
            self.wheel_yaw_support=('MCU_interval_bracketed' if self.native_yaws[0][0]<=a and b<=self.native_yaws[-1][0]
                                    else 'MCU_interval_endpoint_held')
        else:
            ya=sample(host_yaws,start)[0];yb=sample(host_yaws,end)[0]
            self.wheel_yaw_support='host_interval_estimate'
        angle=(ya+yb)/2;dt=(b-a)/1e9
        delta=dt*np.array([math.cos(angle)*v[0]-math.sin(angle)*v[1],
                           math.sin(angle)*v[0]+math.cos(angle)*v[1]])
        if not self.wheels:self.wheels.append((int(start),0.,0.,0.))
        old=self.wheels[-1];xy=np.array(old[1:3])+delta;travel=old[3]+_norm2(v)*dt
        if end==old[0]:self.wheels.pop()
        self.wheels.append((int(end),*xy,travel));self.last_wheel=key
        self._process()

    def _wheel_at(self,t):
        w=sample(self.wheels,t)
        result=np.empty(3,dtype=float)
        result[:2]=self.basis@w[:2];result[2]=w[2]
        return result

    def _fallback(self,reason,t):
        if self.phase_wheel_suppressed:return
        if self.mode=='T265_PRIMARY_WHEEL_SUSPECT' and self.wheels and self.history:
            end=self.wheels[-1][0]
            self.anchor=sample(self.history,end)[:2]
            self.anchor_wheel=self._wheel_at(end)[:2]
        entering=self.mode!='WHEEL_FALLBACK'
        if entering:
            self.events.append(dict(event='motion_source_changed',sample_monotonic_ns=t,
                previous_source=self.mode,motion_source='WHEEL_FALLBACK',reason=reason,
                spatial_conflict_m=self.last_conflict_m))
        self.mode='WHEEL_FALLBACK'
        # Preserve the cause of this fallback episode. A paired Pose arriving
        # later must not hide an SDK jump behind a generic displacement error.
        # An explicit new wall verdict may still replace the old diagnosis.
        if entering or reason=='two_axis_walls_support_wheel_over_T265':self.reason=reason
        self.recovery_window=None;self.recovery_travel=0.;self.wall_fixes=0

    def _process(self):
        if self.phase_wheel_suppressed:return
        if self.reference is None or not self.wheels:return
        if self.resume_anchor_ns is not None:
            if self.wheels[-1][0]<self.resume_anchor_ns:return
            t=max(self.resume_anchor_ns,self.wheels[0][0])
            self.anchor=(sample(self.history,t)[:2] if self.history else self.resume_anchor_xy.copy())
            self.anchor_wheel=self._wheel_at(t)[:2]
            self.pending=deque((p for p in self.pending if p[0]>=t),maxlen=4096)
            self.resume_anchor_ns=None;self.resume_anchor_xy=None
        ref_ns=self.reference.get('reference_sample_ns',self.reference.get('timestamp_monotonic_ns',0))
        if self.anchor is None and self.wheels[0][0]<=ref_ns<=self.wheels[-1][0]:
            # The established wall reference is already an anchor. Do not
            # replace it blindly with the first subsequent (possibly jumped) Pose.
            xy=np.array(self.reference['pose_origin'],float);w=self._wheel_at(ref_ns)
            self.anchor=xy.copy();self.anchor_wheel=w[:2].copy()
            self.previous=(xy,w);self.window=(xy,w)
            self.mode='T265_WITH_WHEEL_TAIL';self.reason='paired_field_reference'
            self.last_paired_ns=ref_ns
        while self.pending and self.pending[0][0]<=self.wheels[-1][0]:
            p=self.pending.popleft();t=p[0];xy=np.array(p[1:3]);confidence=p[3]
            if t<self.wheels[0][0]:continue  # No equal-interval reference retained, not an age decision.
            w=self._wheel_at(t)
            if self.anchor is None:
                if confidence<=0:continue
                self.anchor=xy.copy();self.anchor_wheel=w[:2].copy()
                self.mode='T265_WITH_WHEEL_TAIL';self.reason='paired_actual_feedback'
                self.previous=(xy,w);self.window=(xy,w)
            elif confidence<=0:
                self._fallback('SDK_tracking_lost',t);self.previous=None;self.window=None
                continue
            elif self.previous is None:
                self.previous=(xy,w);self.window=(xy,w)
            else:
                prev,wp=self.previous;step=_norm2((xy-prev)-(w[:2]-wp[:2]))
                base,wb=self.window
                conflict=_norm2((xy-base)-(w[:2]-wb[:2]))
                self.last_conflict_m=max(step,conflict)
                bad=step>POLICY['single_increment_conflict_m'] or conflict>POLICY['displacement_conflict_m']
                if bad:
                    if self.mode!='T265_PRIMARY_WHEEL_SUSPECT':
                        self._fallback('T265_wheel_displacement_conflict',t)
                    else:
                        self.recovery_window=None;self.recovery_travel=0.
                    self.window=(xy,w)
                elif self.mode=='T265_WITH_WHEEL_TAIL':
                    self.anchor=xy+self.bias;self.anchor_wheel=w[:2].copy()
                    if w[2]-wb[2]>=POLICY['comparison_travel_m']:self.window=(xy,w)
                else:
                    if self.recovery_window is None:self.recovery_window=(xy,w)
                    rb,rw=self.recovery_window
                    residual=_norm2((xy-rb)-(w[:2]-rw[:2]))
                    if residual>POLICY['recovery_residual_m']:
                        self.recovery_window=(xy,w);self.recovery_travel=0.
                    elif w[2]-rw[2]>=POLICY['comparison_travel_m']:
                        self.recovery_travel+=w[2]-rw[2];self.recovery_window=(xy,w)
                    if (self.recovery_travel>=POLICY['recovery_travel_m'] and
                            self.wall_fixes>=POLICY['recovery_two_axis_fixes'] and
                            (not self.wheel_suspect or (self.evidence_verdict=='BOTH_AGREE' and self.evidence_confirmations>=2))):
                        previous_mode=self.mode
                        continuous=(xy+self.bias if self.mode=='T265_PRIMARY_WHEEL_SUSPECT'
                                    else self.anchor+w[:2]-self.anchor_wheel)
                        self.bias=continuous-xy;self.anchor=continuous;self.anchor_wheel=w[:2].copy()
                        self.mode='T265_WITH_WHEEL_TAIL';self.reason='motion_and_two_axis_walls_agree_rebased'
                        self.wheel_suspect=False
                        self.window=(xy,w)
                        self.events.append(dict(event='motion_source_changed',sample_monotonic_ns=t,
                            previous_source=previous_mode,motion_source=self.mode,reason=self.reason,
                            t265_translation_rebase_m=self.bias.tolist(),recovery_travel_m=self.recovery_travel))
                self.previous=(xy,w)
            self.last_paired_ns=t
        self._publish()

    def _publish(self):
        if self.phase_wheel_suppressed or self.resume_anchor_ns is not None:return
        if self.anchor is None or not self.wheels:return
        if self.mode=='T265_PRIMARY_WHEEL_SUSPECT':return
        t=self.wheels[-1][0];w=self._wheel_at(t)
        xy=self.anchor+w[:2]-self.anchor_wheel
        if self.history and t<self.history[-1][0]:return
        if self.history and t==self.history[-1][0]:self.history.pop()
        self.history.append((t,*xy,0.))

    def _publish_vio(self,t,xy):
        if self.history and t<self.history[-1][0]:return
        if self.history and t==self.history[-1][0]:self.history.pop()
        self.history.append((int(t),*(xy+self.bias),0.))

    def _select_wall_supported_vio(self,t,raw_poses):
        if self.phase_wheel_suppressed:return
        previous_mode=self.mode
        # Anchor at the scan capture time, then replace only subsequent motion
        # with real VIO increments. The caller applies that wall posterior once.
        # Anchoring at fit completion would leave the wheel slip in its tail.
        selected=sample(self.history or raw_poses,t)[:2]
        raw=sample(raw_poses,t)[:2]
        self.bias=selected-raw
        while self.history and self.history[-1][0]>=t:self.history.pop()
        self.history.append((int(t),*selected,0.))
        for pose in raw_poses:
            if pose[0]>t:self._publish_vio(pose[0],np.asarray(pose[1:3]))
        self.anchor=selected.copy();self.anchor_wheel=self._wheel_at(t)[:2]
        self.mode='T265_PRIMARY_WHEEL_SUSPECT';self.wheel_suspect=True
        self.reason='two_axis_walls_support_T265_over_wheel'
        self.recovery_window=None;self.recovery_travel=0.;self.wall_fixes=0
        self.events.append(dict(event='motion_source_changed',sample_monotonic_ns=t,
            previous_source=previous_mode,motion_source=self.mode,reason=self.reason,
            wheel_input_pause_requested=True,wall_motion_evidence=dict(self.wall_evidence)))

    def _select_wall_supported_wheel(self,t):
        if self.phase_wheel_suppressed:return
        selected=sample(self.history,t)[:2];w=self._wheel_at(t)
        while self.history and self.history[-1][0]>=t:self.history.pop()
        self.history.append((int(t),*selected,0.))
        for row in self.wheels:
            if row[0]>t:
                xy=selected+self._wheel_at(row[0])[:2]-w[:2]
                self.history.append((int(row[0]),*xy,0.))
        self.wheel_suspect=False
        self._fallback('two_axis_walls_support_wheel_over_T265',t)
        self.previous=None;self.window=None

    def observe_walls(self,scan_id,axes,*,reference_ns=None,center_xy=None,raw_poses=None,reacquired=False):
        if self.phase_wheel_suppressed:return  # Outer geometric correction still applies normally.
        if (self.mode=='WHEEL_FALLBACK' or self.wheel_suspect) and scan_id!=self.last_wall:
            self.last_wall=scan_id
            self.wall_fixes=self.wall_fixes+1 if set(axes)=={'x','y'} else 0
        if reference_ns is None:return  # Existing geometry-only callers/fixtures.
        if scan_id==self.last_evidence_scan:return
        self.last_evidence_scan=scan_id
        if (set(axes)!={'x','y'} or center_xy is None or self.reference is None or
                not raw_poses or not self.wheels):
            self.wall_window=None;self.evidence_verdict=None;self.evidence_confirmations=0
            self.wall_evidence=dict(status='NO_FULL_XY_WALL_EVIDENCE',scan_id=scan_id)
            return
        t=int(reference_ns)
        # Equal-interval displacement comparison needs actual interpolation
        # support. Missing support suppresses ONLY this source verdict; it
        # never rejects feedback/poses, clears VALID, stops, or gates injection.
        if not (raw_poses[0][0]<=t<=raw_poses[-1][0] and self.wheels[0][0]<=t<=self.wheels[-1][0]):
            self.wall_window=None;self.evidence_verdict=None;self.evidence_confirmations=0
            self.wall_evidence=dict(status='NO_COMMON_INTERVAL_SUPPORT',scan_id=scan_id)
            return
        angle=self.reference['vio_rotation'];c,s=math.cos(angle),math.sin(angle)
        r=np.array([[c,-s],[s,c]])
        vio=r@(sample(raw_poses,t)[:2]-self.reference['pose_origin'])+self.reference['start_center_xy_m']
        wheel=self._wheel_at(t);wxy=r@wheel[:2]
        now=(t,np.asarray(center_xy,float),vio,wxy,wheel[2])
        if self.wall_window is None or reacquired:
            self.wall_window=now;self.evidence_verdict=None;self.evidence_confirmations=0
            self.wall_evidence=dict(status='WALL_MOTION_BASELINE',scan_id=scan_id,reference_ns=t)
            return
        old=self.wall_window
        if t<=old[0]:return
        wall_delta=now[1]-old[1];vio_delta=vio-old[2];wheel_delta=wxy-old[3]
        activity=max(float(np.linalg.norm(wall_delta)),float(np.linalg.norm(vio_delta)),wheel[2]-old[4])
        if activity<POLICY['comparison_travel_m']:return
        ve=float(np.linalg.norm(vio_delta-wall_delta));we=float(np.linalg.norm(wheel_delta-wall_delta))
        good=POLICY['recovery_residual_m'];bad=POLICY['displacement_conflict_m']
        verdict=('WHEEL_SUSPECT' if ve<=good and we>bad else
                 'T265_SUSPECT' if we<=good and ve>bad else
                 'BOTH_AGREE' if max(ve,we)<=good else 'AMBIGUOUS')
        self.evidence_confirmations=self.evidence_confirmations+1 if verdict==self.evidence_verdict else 1
        self.evidence_verdict=verdict
        self.wall_evidence=dict(status=verdict,scan_id=scan_id,reference_ns=t,baseline_ns=old[0],
            movement_m=activity,t265_wall_displacement_error_m=ve,wheel_wall_displacement_error_m=we,
            confirmations=self.evidence_confirmations,t265_depends_on_sdk_wheel_input=True,
            walls_depend_on_primary_yaw_and_selected_deskew=True,independent_ground_truth=False)
        if self.evidence_confirmations>=2:
            if verdict=='WHEEL_SUSPECT' and self.mode!='T265_PRIMARY_WHEEL_SUSPECT':
                self._select_wall_supported_vio(t,raw_poses)
            elif verdict=='T265_SUSPECT' and self.wheel_suspect:
                # Two-axis geometry now specifically supports wheel motion.
                # Resume real samples only; never reload/reset the estimator.
                self._select_wall_supported_wheel(t)
        if activity>=POLICY['recovery_travel_m'] and (self.evidence_confirmations>=2 or verdict=='AMBIGUOUS'):
            self.wall_window=now  # Spatial window, no age/freshness threshold.

    def snapshot(self,*,include_evidence=True):
        result=dict(mode=self.mode,reason=self.reason,spatial_conflict_m=self.last_conflict_m,
            last_paired_pose_ns=self.last_paired_ns,last_raw_pose_ns=self.last_raw_ns,
            last_wheel_sample_id=self.last_wheel[1] if self.last_wheel else None,
            wheel_time_basis=self.wheel_time_basis,wheel_yaw_support=self.wheel_yaw_support,
            pending_pose_drops=self.pending_pose_drops,
            recovery_travel_m=self.recovery_travel,recovery_two_axis_fixes=self.wall_fixes,
            t265_translation_rebase_m=self.bias.tolist(),policy=POLICY,
            wheel_motion_suspect=self.wheel_suspect,wall_motion_evidence=self.wall_evidence,
            phase_wheel_suppressed=self.phase_wheel_suppressed,
            phase_wheel_suppression_reason=self.phase_wheel_reason,
            wheel_input_policy=dict(pause_requested=self.wheel_input_paused(),reason=self.wheel_input_reason(),
                dependency_persists_after_pause=True),
            wheel_and_gyro_are_sdk_dependencies=True,independent_ground_truth=False)
        if not include_evidence:result.pop('wall_motion_evidence')
        return result
