"""Debug-only motion-compensated fence XY + T265 translation + fixed primary yaw.

SE(2) deskew follows the reviewed patch fusion/deskew.py. Motion history comes
from the explicitly selected VIO/wheel source and gyro; a scan is never fed
back into its own motion compensation.
Time quality is diagnostic, not a gate. No wheel-position blend or STOP latch.
"""
from collections import deque
from bisect import bisect_left, bisect_right
from operator import itemgetter
import math
import threading
import time
import numpy as np
from .diagnostic import fixed_yaw_position, reacquire_fixed_yaw_position
from .static import extract_lines, outer_supported_lines
from .global_init import initialize, initial_heading_branch
from .motion_source import MotionSource, sample

# Agreement of independently fitted scan offsets after measured motion has
# been removed. Engineering starting value, not a calibrated position error.
REACQUISITION_OFFSET_AGREEMENT_M = .05


def rotation(angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.array([[c,-s],[s,c]])


def interpolate(times,values,queries):
    """Interpolate actual support; hold endpoints without inventing a velocity."""
    times=np.asarray(times,dtype=np.int64);queries=np.asarray(queries,dtype=np.int64)
    values=np.asarray(values,float)
    if len(times)==0:raise ValueError("no motion samples")
    if len(times)==1:return np.repeat(values[:1],len(queries),axis=0)
    origin=times[0];t=(times-origin)/1e9;q=(queries-origin)/1e9
    result=np.column_stack([np.interp(q,t,values[:,i]) for i in range(values.shape[1])])
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
        self.initialization=None;self.pending_initial_pose=None
        self.last_initial_scan_id=None
        self.lidar_corrections_enabled=True
        self.pending_reacquisition=None;self.reacquisition_count=0
        self.motion=MotionSource()

    def ingest_pose(self,p):
        values=[p.get(k) for k in ('x_m','y_m','yaw_rad')]
        if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in values):return
        with self.lock:
            if self.last_sample and p['session_id']!=self.last_sample['session_id']:
                self.reference=None;self.revision+=1;self.poses.clear();self.motion=MotionSource()
                self.pending_reacquisition=None
            if self.last_sample and p['session_id']==self.last_sample['session_id'] and p['frame_number']<=self.last_sample['frame_number']:return
            t=int(p['timestamp_monotonic_ns'])
            if self.poses and t<=self.poses[-1][0]:t=int(p['receive_monotonic_ns'])
            if self.poses and t<=self.poses[-1][0]:return
            self.motion.pose(dict(p,timestamp_monotonic_ns=t))
            if p.get('tracker_confidence',0)<=0:return
            self.poses.append((t,*values));self.last_sample=dict(p)

    def ingest_yaw(self,t,yaw_deg,source):
        if not math.isfinite(yaw_deg):return
        with self.lock:
            if self.yaws and t<self.yaws[-1][0]:return
            if self.yaws and t==self.yaws[-1][0]:self.yaws.pop()
            self.yaws.append((int(t),math.radians(yaw_deg)));self.last_yaw=dict(source)
            self.motion.yaw(t,math.radians(yaw_deg),source)

    def ingest_wheel(self,sample):
        with self.lock:self.motion.wheel(sample,self.yaws)

    def invalidate_reference(self,reason):
        with self.lock:
            self.reference=None;self.revision+=1;self.yaws.clear();self.last_fit=None
            self.motion=MotionSource()
            self.has_anchor=False
            self.initialization=None;self.pending_initial_pose=None
            self.pending_reacquisition=None
            return dict(event='reference_invalidated',reason=reason)

    def request_initialization(self,description,*,physical_confirmed=True):
        with self.lock:
            if not self.poses or not self.yaws:raise ValueError('等待实际T265 Pose和主yaw数据')
            self.reference=None;self.revision+=1;self.offset[:]=0;self.has_anchor=False
            self.motion.reference=None;self.motion.history.clear()
            self.last_fit=None;self.latest_points=[];self.pending_initial_pose=None
            self.pending_reacquisition=None
            self.last_initial_scan_id=None
            self.initialization=dict(status='SEARCHING_WALLS',description=description,
                physical_stationary_confirmed=physical_confirmed,initial_primary_yaw_rad=self.yaws[-1][1],
                initial_pose_yaw_rad=self.poses[-1][3],started_ns=time.monotonic_ns(),
                confirmations=0,attempts=0,branch_policy='initial_heading_nearest_positive_X_mod_90')
            return dict(self.initialization)

    def set_lidar_enabled(self,enabled):
        with self.lock:
            self.lidar_corrections_enabled=bool(enabled);self.last_fit=None;self.latest_points=[]
            self.motion.wall_fixes=0
            self.pending_initial_pose=None
            self.pending_reacquisition=None
            if self.initialization:self.initialization['confirmations']=0

    def accepted_points(self,scan):
        m=self.mount
        return [p for p in scan.points if p.valid_return and p.raw_angle_deg is not None
            and abs((p.raw_angle_deg-m['center_raw_deg']+180)%360-180)<=m['fov_deg']/2
            and not any(lo<=p.raw_angle_deg<hi for lo,hi in m['excluded_raw_ranges_deg'])]

    def try_initialize(self,scan):
        """Stationary global wall initialization; no prescribed XY or red/blue.

        Raw single-scan wall geometry is valid only under the operator's flat,
        stationary placement declaration. This does not certify calibration.
        """
        with self.lock:
            state=self.initialization
            if state is None:return dict(status='INITIALIZATION_NOT_REQUESTED')
            if not self.lidar_corrections_enabled:return dict(status='LIDAR_DISABLED_FOR_OBSTACLE')
            if scan.scan_id==self.last_initial_scan_id:return dict(status='DUPLICATE_INITIAL_SCAN')
            self.last_initial_scan_id=scan.scan_id
            poses=np.array(self.poses);yaws=np.array(self.yaws)
            if not len(poses) or not len(yaws):return dict(status='WAITING_MOTION_REFERENCE')
            state['attempts']+=1
        accepted=self.accepted_points(scan)
        points=np.array([p.point_lidar_m for p in accepted]).reshape(-1,2)
        ext=[*self.mount['extrinsic'][:2],math.radians(self.mount['extrinsic'][2])]
        found,reason=initialize(points,extrinsic=ext)
        if found is None:
            self.pending_initial_pose=None;state.update(status=reason,confirmations=0)
            return dict(state,scan_id=scan.scan_id,reference_ns=scan.reference_ns,accuracy_verified=False)
        t=[scan.reference_ns]
        raw_yaw=interpolate(yaws[:,0].astype(np.int64),yaws[:,1:2],t)[0,0]
        pose_values=poses[:,1:4].copy();pose_values[:,2]=np.unwrap(pose_values[:,2])
        raw_pose=interpolate(poses[:,0].astype(np.int64),pose_values,t)[0]
        # Choose by heading at the REQUEST, not whichever heading a later scan
        # happened to have. Hold the first chosen square branch during confirmation.
        candidate=np.asarray(initial_heading_branch(found,raw_yaw-state['initial_primary_yaw_rad'],self.pending_initial_pose))
        old=self.pending_initial_pose
        consistent=old is not None and np.linalg.norm(candidate[:2]-old[:2])<.05 and abs((candidate[2]-old[2]+np.pi)%(2*np.pi)-np.pi)<math.radians(2)
        state['confirmations']=state['confirmations']+1 if consistent else 1
        self.pending_initial_pose=candidate
        state.update(status='CONFIRMING_WALL_MATCH',candidate_center_xy_m=(candidate[:2]-1.5).tolist(),
            candidate_yaw_deg=math.degrees(candidate[2])%360,support_fraction=found['support_fraction'],
            equivalent_center_poses=[[q[0]-1.5,q[1]-1.5,math.degrees(q[2])%360] for q in found['equivalent_poses']])
        if state['confirmations']>=3:
            theta=float(candidate[2]);xy=(candidate[:2]-1.5).tolist()
            with self.lock:
                self.reference=dict(pose_origin=raw_pose[:2].tolist(),vio_rotation=theta-float(raw_pose[2]),
                    yaw_offset=theta-float(raw_yaw),start_center_xy_m=xy,field_heading_deg=math.degrees(theta)%360,
                    description=state['description'],physical_placement_confirmed=False,
                    physical_stationary_confirmed=state['physical_stationary_confirmed'],
                    source='lidar_global_initialization',branch_policy=state['branch_policy'],
                    timestamp_monotonic_ns=time.monotonic_ns(),reference_sample_ns=scan.reference_ns,
                    square_absolute_side_identified=False)
                self.offset[:]=0;self.has_anchor=True;self.revision+=1
                self.motion.configure(self.reference)
                state['status']='GLOBAL_SYMMETRIC_FIX'
        return dict(state,scan_id=scan.scan_id,reference_ns=scan.reference_ns,accuracy_verified=False,
            reference=dict(self.reference) if self.reference else None)

    def align(self,description,*,physical_confirmed=True):
        """Legacy known-pose fixture for existing offline checks, not GUI startup."""
        with self.lock:
            if not self.poses or not self.yaws:raise ValueError("等待实际T265 Pose和主yaw数据")
            p=self.poses[-1];y=self.yaws[-1]
            self.reference=dict(pose_origin=list(p[1:3]),vio_rotation=math.radians(135)-p[3],
                yaw_offset=math.radians(135)-y[1],start_center_xy_m=[-1.35,1.35],
                field_heading_deg=135.,description=description,physical_placement_confirmed=physical_confirmed,
                timestamp_monotonic_ns=time.monotonic_ns())
            self.offset[:]=0;self.last_fit=None;self.latest_points=[];self.has_anchor=False;self.revision+=1
            self.pending_reacquisition=None
            self.reference['reference_sample_ns']=p[0]
            self.motion.configure(self.reference)
            return dict(self.reference)

    def _frozen(self,first_ns=None,last_ns=None,*,latest=False):
        with self.lock:
            if self.reference is None or not self.poses or not self.yaws:raise ValueError("尚未从围栏建立场地参考")
            poses,yaws=list(self.motion.history or self.poses),list(self.yaws)
            reference,offset,revision=dict(self.reference),self.offset.copy(),self.revision
        if latest:first_ns=last_ns=poses[-1][0]
        if first_ns is not None:
            def window(history):
                # Retain bracketing samples, or endpoints for the same hold.
                # Full histories remain stored; no age gate.
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

    def _fit_position(self,points,ref,offset,scan_id):
        """Local tracking, or two-scan enclosure reacquisition at fixed yaw.

        Share line extraction. A bad wheel/T265 XY prior cannot veto a new
        two-axis wall solution after local XY tracking is lost. Preserve a
        surviving single-axis track: a distant parallel interior surface must
        not be promoted to the missing arena wall just to complete XY.
        Compare correction offsets, not absolute positions of a moving car.
        """
        lines=outer_supported_lines(extract_lines(points))
        prior=ref[:2]+offset
        local=fixed_yaw_position(points,prior,ref[2],[0.,0.,0.],_lines=lines)
        local.update(position_reacquired=False,reacquisition_status='LOCAL_TRACKING')
        if local['rank_xy']>0:
            self.pending_reacquisition=None
            return local
        found=reacquire_fixed_yaw_position(points,ref[2],_lines=lines)
        if found['rank_xy']!=2:
            self.pending_reacquisition=None
            return dict(local,reacquisition_status=found['reason'])
        candidate=np.asarray(found['candidate_center_xy_m'])
        proposed=candidate-ref[:2]
        previous=self.pending_reacquisition
        agrees=(previous is not None and previous['scan_id']!=scan_id and
                np.linalg.norm(proposed-previous['offset'])<=REACQUISITION_OFFSET_AGREEMENT_M)
        if not agrees:
            self.pending_reacquisition=dict(scan_id=scan_id,offset=proposed)
            return dict(local,reacquisition_status='CONFIRMING_ENCLOSURE',
                reacquisition_candidate_xy_m=candidate.tolist())
        self.pending_reacquisition=None;self.reacquisition_count+=1
        return dict(found,reason='FIXED_YAW_ENCLOSURE_REACQUIRED',position_reacquired=True,
            reacquisition_status='REACQUIRED',reacquisition_confirmations=2,
            correction_m=float(np.linalg.norm(candidate-prior)),
            local_reason=local['reason'],local_observed_axes=local['position_observed_axes'])

    def observe(self,scan):
        if self.last_fit and self.last_fit['scan_id']==scan.scan_id:
            return dict(self.last_fit,duplicate_scan=True,position_reacquired=False)
        began=time.monotonic_ns();cpu_began=time.thread_time_ns();self.scan_count+=1
        poses,yaws,reference,offset,revision=self._frozen(scan.first_point_ns,scan.reference_ns)
        copied=time.monotonic_ns()
        query=self.trajectory(poses,yaws,reference);m=self.mount
        accepted=self.accepted_points(scan)
        if len(accepted)<8:
            self.pending_reacquisition=None
            raise ValueError("可用雷达点不足")
        points=np.array([p.point_lidar_m for p in accepted]);times=np.array([p.time_ns for p in accepted],np.int64)
        ext=np.array([*m['extrinsic'][:2],math.radians(m['extrinsic'][2])])
        corrected,ref=deskew_points(points,times,scan.reference_ns,query,ext)
        deskew_done=time.monotonic_ns()
        fit=(self._fit_position(corrected,ref,offset,scan.scan_id) if self.lidar_corrections_enabled
            else dict(position_observed_axes=[],rank_xy=0,candidate_center_xy_m=None,
                inherited_axes=['x','y'],covariance=None,qualified_pose=False,
                reason='OPERATOR_OBSTACLE_NO_LIDAR_CORRECTION'))
        fitted=time.monotonic_ns()
        latest=max(poses[-1,0],yaws[-1,0]);oldest=max(poses[0,0],yaws[0,0])
        result=dict(fit,scan_id=scan.scan_id,reference_ns=scan.reference_ns,
            deskew_applied=True,rotation_compensated=True,translation_compensated=True,
            timestamp_semantics_verified=False,deskew_qualified=False,accuracy_verified=False,
            time_policy='adopt_received_data_timing_diagnostic_only',motion_source=self.motion.mode,
            motion_selection=self.motion.snapshot(),motion_boundary_policy='hold_outside_measured_support',
            primary_yaw_field_rad=float(ref[2]),prior_center_xy_m=(ref[:2]+offset).tolist(),
            physical_points=len(scan.points),used_points=len(accepted),scan_span_ms=(scan.last_point_ns-scan.first_point_ns)/1e6,
            motion_unsupported_span_ns=max(0,int(scan.reference_ns-min(poses[-1,0],yaws[-1,0])),int(oldest-scan.first_point_ns)),
            motion_latest_ns=int(latest),timestamp_monotonic_ns=began,reference_revision=revision)
        with self.lock:
            if revision!=self.revision:raise ValueError("reference changed during scan fit")
            candidate=fit.get('candidate_center_xy_m')
            self.motion.observe_walls(scan.scan_id,fit['position_observed_axes'])
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
        except ValueError as e:return dict(ready=False,reason=str(e),scans=self.scan_count,fixes=self.fit_count,
            initialization=dict(self.initialization) if self.initialization else None,
            lidar_corrections_enabled=self.lidar_corrections_enabled)
        # Publish at latest actual motion support, never stamp extrapolation as a new sensor frame.
        sample_ns=int(poses[-1,0]);pose=self.trajectory(poses,yaws,reference)([sample_ns])[0]
        with self.lock:
            fit=dict(self.last_fit) if self.last_fit else None;points=list(self.latest_points)
            raw=sample(self.poses,sample_ns);raw_stamp=int(self.poses[-1][0])
        raw_xy=(raw[:2]-reference['pose_origin'])@rotation(reference['vio_rotation']).T+reference['start_center_xy_m']
        source=self.motion.mode if self.motion.history else ('VIO' if self.has_anchor else 'VIO_FROM_INITIAL_REFERENCE')
        return dict(ready=True,center_xy_m=(pose[:2]+offset).tolist(),t265_center_xy_m=raw_xy.tolist(),yaw_deg=math.degrees(pose[2])%360,
            sample_monotonic_ns=sample_ns,motion_age_ms=(now-sample_ns)/1e6,
            last_scan_age_ms=None if self.last_scan_ns is None else (now-self.last_scan_ns)/1e6,
            source=('LIDAR_ANCHORED_'+source) if self.has_anchor else source,
            motion_selection=self.motion.snapshot(),motion_boundary_policy='hold_outside_measured_support',
            raw_t265_sample_ns=raw_stamp,
            position_reacquisitions=self.reacquisition_count,
            current_wall_axes=fit.get('position_observed_axes',[]) if fit else [],last_fit=fit,
            lidar_corrections_enabled=self.lidar_corrections_enabled,
            points_field=points,scans=self.scan_count,fixes=self.fit_count,reference=reference,
            qualified_pose=False,accuracy_verified=False,control_pose_published=False)
