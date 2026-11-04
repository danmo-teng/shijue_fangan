"""Test-only exclusive translation source, with measured wheel bridging.

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
    i=bisect_left(history,t,key=lambda x:x[0])
    if i==0:return np.asarray(history[0][1:],float)
    if i==len(history):return np.asarray(history[-1][1:],float)
    a,b=history[i-1],history[i];f=(t-a[0])/(b[0]-a[0])
    return np.asarray(a[1:],float)*(1-f)+np.asarray(b[1:],float)*f


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
        angle=reference['yaw_offset']-reference['vio_rotation']
        c,s=math.cos(angle),math.sin(angle);self.basis=np.array([[c,-s],[s,c]])
        t=reference.get('reference_sample_ns',reference.get('timestamp_monotonic_ns',0))
        self.pending=deque((p for p in self.pending if p[0]>=t),maxlen=4096)
        self._process()

    def pose(self, p):
        t=int(p['timestamp_monotonic_ns']);xy=np.array([p['x_m'],p['y_m']],float)
        if not np.isfinite(xy).all():return
        if self.last_raw_ns is not None and t<=self.last_raw_ns:return
        self.last_raw_ns=t;self.last_confidence=p['tracker_confidence']
        if len(self.pending)==self.pending.maxlen:self.pending_pose_drops+=1
        self.pending.append((t,*xy,int(p['tracker_confidence'])))
        if p['tracker_confidence']<=0 and self.anchor is not None:
            self._fallback('SDK_tracking_lost',t)
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
        old=self.wheels[-1];xy=np.array(old[1:3])+delta;travel=old[3]+float(np.linalg.norm(v))*dt
        if end==old[0]:self.wheels.pop()
        self.wheels.append((int(end),*xy,travel));self.last_wheel=key
        self._process()

    def _wheel_at(self,t):
        w=sample(self.wheels,t);return np.r_[self.basis@w[:2],w[2]]

    def _fallback(self,reason,t):
        if self.mode!='WHEEL_FALLBACK':
            self.events.append(dict(event='motion_source_changed',sample_monotonic_ns=t,
                previous_source=self.mode,motion_source='WHEEL_FALLBACK',reason=reason,
                spatial_conflict_m=self.last_conflict_m))
        self.mode='WHEEL_FALLBACK';self.reason=reason
        self.recovery_window=None;self.recovery_travel=0.;self.wall_fixes=0

    def _process(self):
        if self.reference is None or not self.wheels:return
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
                prev,wp=self.previous;step=float(np.linalg.norm((xy-prev)-(w[:2]-wp[:2])))
                base,wb=self.window
                conflict=float(np.linalg.norm((xy-base)-(w[:2]-wb[:2])))
                self.last_conflict_m=max(step,conflict)
                bad=step>POLICY['single_increment_conflict_m'] or conflict>POLICY['displacement_conflict_m']
                if bad:
                    self._fallback('T265_wheel_displacement_conflict',t)
                    self.window=(xy,w)
                elif self.mode=='T265_WITH_WHEEL_TAIL':
                    self.anchor=xy+self.bias;self.anchor_wheel=w[:2].copy()
                    if w[2]-wb[2]>=POLICY['comparison_travel_m']:self.window=(xy,w)
                else:
                    if self.recovery_window is None:self.recovery_window=(xy,w)
                    rb,rw=self.recovery_window
                    residual=float(np.linalg.norm((xy-rb)-(w[:2]-rw[:2])))
                    if residual>POLICY['recovery_residual_m']:
                        self.recovery_window=(xy,w);self.recovery_travel=0.
                    elif w[2]-rw[2]>=POLICY['comparison_travel_m']:
                        self.recovery_travel+=w[2]-rw[2];self.recovery_window=(xy,w)
                    if (self.recovery_travel>=POLICY['recovery_travel_m'] and
                            self.wall_fixes>=POLICY['recovery_two_axis_fixes']):
                        continuous=self.anchor+w[:2]-self.anchor_wheel
                        self.bias=continuous-xy;self.anchor=continuous;self.anchor_wheel=w[:2].copy()
                        self.mode='T265_WITH_WHEEL_TAIL';self.reason='motion_and_two_axis_walls_agree_rebased'
                        self.window=(xy,w)
                        self.events.append(dict(event='motion_source_changed',sample_monotonic_ns=t,
                            previous_source='WHEEL_FALLBACK',motion_source=self.mode,reason=self.reason,
                            t265_translation_rebase_m=self.bias.tolist(),recovery_travel_m=self.recovery_travel))
                self.previous=(xy,w)
            self.last_paired_ns=t
        self._publish()

    def _publish(self):
        if self.anchor is None or not self.wheels:return
        t=self.wheels[-1][0];w=self._wheel_at(t)
        xy=self.anchor+w[:2]-self.anchor_wheel
        if self.history and t==self.history[-1][0]:self.history.pop()
        self.history.append((t,*xy,0.))

    def observe_walls(self,scan_id,axes):
        if self.mode=='WHEEL_FALLBACK' and scan_id!=self.last_wall:
            self.last_wall=scan_id
            self.wall_fixes=self.wall_fixes+1 if set(axes)=={'x','y'} else 0

    def snapshot(self):
        return dict(mode=self.mode,reason=self.reason,spatial_conflict_m=self.last_conflict_m,
            last_paired_pose_ns=self.last_paired_ns,last_raw_pose_ns=self.last_raw_ns,
            last_wheel_sample_id=self.last_wheel[1] if self.last_wheel else None,
            wheel_time_basis=self.wheel_time_basis,wheel_yaw_support=self.wheel_yaw_support,
            pending_pose_drops=self.pending_pose_drops,
            recovery_travel_m=self.recovery_travel,recovery_two_axis_fixes=self.wall_fixes,
            t265_translation_rebase_m=self.bias.tolist(),policy=POLICY,
            wheel_and_gyro_are_sdk_dependencies=True,independent_ground_truth=False)
