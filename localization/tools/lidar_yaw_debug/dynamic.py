"""Debug-only motion-compensated fence XY + T265 translation + fixed primary yaw.

SE(2) deskew follows the reviewed patch fusion/deskew.py. Motion history comes
only from VIO/gyro; a scan is never fed back into its own motion compensation.
Time quality is diagnostic, not a gate. No wheel-position blend or STOP latch.
"""
from collections import deque
from bisect import bisect_left, bisect_right
from operator import itemgetter
import math
import threading
import time
import numpy as np
from .diagnostic import fixed_yaw_position


def rotation(angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.array([[c,-s],[s,c]])


def interpolate(times,values,queries):
    """Linear interpolation/extrapolation; never relabel prediction as sampling."""
    times=np.asarray(times,dtype=np.int64);queries=np.asarray(queries,dtype=np.int64)
    values=np.asarray(values,float)
    if len(times)==0:raise ValueError("no motion samples")
    if len(times)==1:return np.repeat(values[:1],len(queries),axis=0)
    origin=times[0];t=(times-origin)/1e9;q=(queries-origin)/1e9
    result=np.column_stack([np.interp(q,t,values[:,i]) for i in range(values.shape[1])])
    for mask,i,j in ((q<t[0],0,1),(q>t[-1],-1,-2)):
        if np.any(mask):result[mask]=values[i]+(q[mask]-t[i])[:,None]*(values[i]-values[j])/(t[i]-t[j])
    return result


def deskew_points(points,times,reference_ns,trajectory,extrinsic):
    """Return points in BODY at reference. Sensor lever arm applied once."""
    poses=trajectory(np.r_[times,reference_ns]);yaw=poses[:-1,2];ref=poses[-1]
    body=np.asarray(points)@rotation(extrinsic[2]).T+extrinsic[:2]
    c,s=np.cos(yaw),np.sin(yaw)
    world=np.column_stack([c*body[:,0]-s*body[:,1],s*body[:,0]+c*body[:,1]])+poses[:-1,:2]
    return (world-ref[:2])@rotation(ref[2]),ref


class DynamicFence:
    def __init__(self,installation):
        self.mount=installation;self.poses=deque(maxlen=4096);self.yaws=deque(maxlen=4096)
        self.lock=threading.RLock();self.reference=None;self.revision=0
        self.offset=np.zeros(2);self.last_fit=None;self.scan_count=0;self.fit_count=0
        self.has_anchor=False
        self.latest_points=[];self.last_sample=None;self.last_yaw=None;self.last_scan_ns=None

    def ingest_pose(self,p):
        if p.get('tracker_confidence',0)<=0:return
        values=[p.get(k) for k in ('x_m','y_m','yaw_rad')]
        if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in values):return
        with self.lock:
            if self.last_sample and p['session_id']!=self.last_sample['session_id']:
                self.reference=None;self.revision+=1;self.poses.clear()
            if self.last_sample and p['session_id']==self.last_sample['session_id'] and p['frame_number']<=self.last_sample['frame_number']:return
            t=int(p['timestamp_monotonic_ns'])
            if self.poses and t<=self.poses[-1][0]:t=int(p['receive_monotonic_ns'])
            if self.poses and t<=self.poses[-1][0]:return
            self.poses.append((t,*values));self.last_sample=dict(p)

    def ingest_yaw(self,t,yaw_deg,source):
        if not math.isfinite(yaw_deg):return
        with self.lock:
            if self.yaws and t<self.yaws[-1][0]:return
            if self.yaws and t==self.yaws[-1][0]:self.yaws.pop()
            self.yaws.append((int(t),math.radians(yaw_deg)));self.last_yaw=dict(source)

    def invalidate_reference(self,reason):
        with self.lock:
            self.reference=None;self.revision+=1;self.yaws.clear();self.last_fit=None
            self.has_anchor=False
            return dict(event='reference_invalidated',reason=reason)

    def align(self,description,*,physical_confirmed=True):
        with self.lock:
            if not self.poses or not self.yaws:raise ValueError("等待实际T265 Pose和主yaw数据")
            p=self.poses[-1];y=self.yaws[-1]
            self.reference=dict(pose_origin=list(p[1:3]),vio_rotation=math.radians(135)-p[3],
                yaw_offset=math.radians(135)-y[1],start_center_xy_m=[-1.35,1.35],
                field_heading_deg=135.,description=description,physical_placement_confirmed=physical_confirmed,
                timestamp_monotonic_ns=time.monotonic_ns())
            self.offset[:]=0;self.last_fit=None;self.latest_points=[];self.has_anchor=False;self.revision+=1
            return dict(self.reference)

    def _frozen(self,first_ns=None,last_ns=None,*,latest=False):
        with self.lock:
            if self.reference is None or not self.poses or not self.yaws:raise ValueError("尚未建立1号位场地参考")
            poses,yaws=list(self.poses),list(self.yaws)
            reference,offset,revision=dict(self.reference),self.offset.copy(),self.revision
        if latest:first_ns=last_ns=min(poses[-1][0],yaws[-1][0])
        if first_ns is not None:
            def window(history):
                # Retain bracketing samples, or two endpoints for the SAME
                # extrapolation. Full histories remain stored; no age gate.
                lo=max(0,bisect_left(history,first_ns,key=itemgetter(0))-1)
                hi=min(len(history),bisect_right(history,last_ns,key=itemgetter(0))+1)
                lo=min(lo,max(0,len(history)-2));hi=max(hi,min(len(history),lo+2))
                return history[lo:hi]
            poses,yaws=window(poses),window(yaws)
        return np.array(poses),np.array(yaws),reference,offset,revision

    @staticmethod
    def trajectory(poses,yaws,reference):
        def query(times):
            p=interpolate(poses[:,0].astype(np.int64),poses[:,1:3],times)
            xy=(p-reference['pose_origin'])@rotation(reference['vio_rotation']).T+reference['start_center_xy_m']
            y=interpolate(yaws[:,0].astype(np.int64),yaws[:,1:2],times)[:,0]+reference['yaw_offset']
            return np.column_stack([xy,y])
        return query

    def observe(self,scan):
        began=time.monotonic_ns();cpu_began=time.thread_time_ns();self.scan_count+=1
        poses,yaws,reference,offset,revision=self._frozen(scan.first_point_ns,scan.reference_ns)
        copied=time.monotonic_ns()
        query=self.trajectory(poses,yaws,reference);m=self.mount
        accepted=[]
        for p in scan.points:
            angle=p.raw_angle_deg
            if (p.valid_return and angle is not None and abs((angle-m['center_raw_deg']+180)%360-180)<=m['fov_deg']/2
                and not any(lo<=angle<hi for lo,hi in m['excluded_raw_ranges_deg'])):accepted.append(p)
        if len(accepted)<8:raise ValueError("可用雷达点不足")
        points=np.array([p.point_lidar_m for p in accepted]);times=np.array([p.time_ns for p in accepted],np.int64)
        ext=np.array([*m['extrinsic'][:2],math.radians(m['extrinsic'][2])])
        corrected,ref=deskew_points(points,times,scan.reference_ns,query,ext)
        deskew_done=time.monotonic_ns()
        fit=fixed_yaw_position(corrected,ref[:2]+offset,ref[2],[0.,0.,0.])
        fitted=time.monotonic_ns()
        latest=max(poses[-1,0],yaws[-1,0]);oldest=max(poses[0,0],yaws[0,0])
        result=dict(fit,scan_id=scan.scan_id,reference_ns=scan.reference_ns,
            deskew_applied=True,rotation_compensated=True,translation_compensated=True,
            timestamp_semantics_verified=False,deskew_qualified=False,accuracy_verified=False,
            time_policy='adopt_received_data_timing_diagnostic_only',motion_source='T265_center_translation_and_primary_yaw',
            primary_yaw_field_rad=float(ref[2]),prior_center_xy_m=(ref[:2]+offset).tolist(),
            physical_points=len(scan.points),used_points=len(accepted),scan_span_ms=(scan.last_point_ns-scan.first_point_ns)/1e6,
            motion_support_extrapolation_ns=max(0,int(scan.reference_ns-min(poses[-1,0],yaws[-1,0])),int(oldest-scan.first_point_ns)),
            motion_latest_ns=int(latest),timestamp_monotonic_ns=began,reference_revision=revision)
        with self.lock:
            if revision!=self.revision:raise ValueError("reference changed during scan fit")
            candidate=fit.get('candidate_center_xy_m')
            if candidate is not None:
                for axis in fit['position_observed_axes']:
                    i=('x','y').index(axis);self.offset[i]=candidate[i]-ref[i]
                self.fit_count+=1
                self.has_anchor=True
            self.last_scan_ns=scan.reference_ns
            self.latest_points=(corrected@rotation(ref[2]).T+ref[:2]+self.offset).tolist()
            result['map_translation_offset_m']=self.offset.tolist()
            result['compute_ms']=(time.monotonic_ns()-began)/1e6
            result['compute_thread_cpu_ms']=(time.thread_time_ns()-cpu_began)/1e6
            result['snapshot_ms']=(copied-began)/1e6
            result['deskew_ms']=(deskew_done-copied)/1e6
            result['fit_ms']=(fitted-deskew_done)/1e6
            self.last_fit=result
        return result

    def snapshot(self,now=None):
        now=time.monotonic_ns() if now is None else now
        try:poses,yaws,reference,offset,_=self._frozen(latest=True)
        except ValueError as e:return dict(ready=False,reason=str(e),scans=self.scan_count,fixes=self.fit_count)
        # Publish at latest actual motion support, never stamp extrapolation as a new sensor frame.
        sample_ns=int(min(poses[-1,0],yaws[-1,0]));pose=self.trajectory(poses,yaws,reference)([sample_ns])[0]
        with self.lock:
            fit=dict(self.last_fit) if self.last_fit else None;points=list(self.latest_points)
        return dict(ready=True,center_xy_m=(pose[:2]+offset).tolist(),t265_center_xy_m=pose[:2].tolist(),yaw_deg=math.degrees(pose[2])%360,
            sample_monotonic_ns=sample_ns,motion_age_ms=(now-sample_ns)/1e6,
            last_scan_age_ms=None if self.last_scan_ns is None else (now-self.last_scan_ns)/1e6,
            source='LIDAR_ANCHORED_VIO' if self.has_anchor else 'VIO_FROM_INITIAL_REFERENCE',
            current_wall_axes=fit.get('position_observed_axes',[]) if fit else [],last_fit=fit,
            points_field=points,scans=self.scan_count,fixes=self.fit_count,reference=reference,
            qualified_pose=False,accuracy_verified=False,control_pose_published=False)
