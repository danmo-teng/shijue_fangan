"""Native high-rate owner; Python retains the original slow fence decisions.

The C ABI never calls Python. Queries copy actual measured support; no freshness
admission, replacement pose, extra UART or SDK owner is introduced here.
"""
import ctypes
import json
import threading
import time
from collections import deque
from pathlib import Path
import numpy as np
from .dynamic import DynamicFence


class NativeMotionRuntime:
    def __init__(self, config):
        path=Path(__file__).resolve().parents[2]/'build/libmotion_runtime.so'
        self.lib=ctypes.CDLL(str(path))
        self.lib.omni_motion_runtime_create.argtypes=[ctypes.c_char_p]
        self.lib.omni_motion_runtime_create.restype=ctypes.c_void_p
        for name in ('command','snapshot'):
            function=getattr(self.lib,'omni_motion_runtime_'+name)
            function.argtypes=[ctypes.c_void_p,ctypes.c_char_p];function.restype=ctypes.c_void_p
        self.lib.omni_motion_runtime_free_string.argtypes=[ctypes.c_void_p]
        self.lib.omni_motion_runtime_free_string.restype=None
        self.lib.omni_motion_runtime_close.argtypes=[ctypes.c_void_p]
        self.lib.omni_motion_runtime_close.restype=None
        self.lib.omni_motion_runtime_finish.argtypes=[ctypes.c_void_p]
        self.lib.omni_motion_runtime_finish.restype=ctypes.c_void_p
        self.lib.omni_motion_runtime_last_error.argtypes=[]
        self.lib.omni_motion_runtime_last_error.restype=ctypes.c_char_p
        self.handle=self.lib.omni_motion_runtime_create(self._encode(config))
        if not self.handle:
            error=self.lib.omni_motion_runtime_last_error()
            raise RuntimeError(error.decode() if error else 'native motion runtime create failed')
        self._lock=threading.RLock();self.final=None

    @staticmethod
    def _encode(value):
        return json.dumps(value,ensure_ascii=False,allow_nan=False,separators=(',',':')).encode()

    def _call(self, name, value):
        with self._lock:
            if not self.handle:
                if name=='snapshot':
                    result=dict(self.final or {})
                    if value.get('consume_events') and self.final is not None:self.final['events']=[]
                    return result
                raise RuntimeError('native motion runtime is closed')
            pointer=getattr(self.lib,'omni_motion_runtime_'+name)(self.handle,self._encode(value))
            if not pointer:
                error=self.lib.omni_motion_runtime_last_error()
                raise RuntimeError(error.decode() if error else 'native motion '+name+' failed')
            try:result=json.loads(ctypes.string_at(pointer))
            finally:self.lib.omni_motion_runtime_free_string(pointer)
        if isinstance(result,dict) and result.get('ok') is False:
            raise ValueError(result.get('error',result.get('reason','native motion '+name+' failed')))
        return result

    def command(self, kind, payload):
        return self._call('command',dict(kind=kind,payload=payload))

    def snapshot(self, query=None):
        return self._call('snapshot',query or dict(kind='state'))

    def close(self):
        with self._lock:
            if not self.handle:return
            previous=self.snapshot()
            pointer=self.lib.omni_motion_runtime_finish(self.handle)
            self.handle=None  # finish consumes ownership, including on error.
            if pointer:
                try:self.final=json.loads(ctypes.string_at(pointer))
                finally:self.lib.omni_motion_runtime_free_string(pointer)
            else:self.final=dict(ok=False,error='native motion final logging receipt unavailable')
            if self.final.get('ok') is False:
                error=self.final.get('error','native motion finish failed')
                self.final=dict(previous,error=error,logging=dict(previous.get('logging',{}),
                    native_finalize=dict(failed=True,dropped=0)))


class NativeSourceView:
    """Low-rate view only; source state and wall arbitration live in C++."""
    def __init__(self, engine):
        self.engine=engine;self.events=deque(maxlen=128)

    def snapshot(self, *, include_evidence=True):
        result=dict(self.engine.native_state.get('motion_selection',{}))
        if not include_evidence:result.pop('wall_motion_evidence',None)
        return result

    @property
    def mode(self):return self.snapshot().get('mode','T265_NO_WHEEL_REFERENCE')
    @property
    def wheel_suspect(self):return self.snapshot().get('wheel_motion_suspect',False)
    @property
    def wall_evidence(self):return self.snapshot().get('wall_motion_evidence')

    def configure(self, reference):
        self.engine.runtime.command('field_reference',dict(reference=reference,
            revision=self.engine.revision,has_anchor=self.engine.has_anchor))
        self.engine.refresh_native()


class NativeDynamicFence(DynamicFence):
    """Keep original initialization/reacquisition; native motion is authoritative."""
    def __init__(self, installation, runtime):
        super().__init__(installation)
        self.runtime=runtime;self.native_state={};self.motion=NativeSourceView(self)
        self.refresh_native()

    def ingest_pose(self,p):
        self.runtime.command('pose',p);self.refresh_native()

    def ingest_yaw(self,t,yaw_deg,source):
        self.runtime.command('yaw',dict(time_ns=t,yaw_deg=yaw_deg,source=source));self.refresh_native()

    def ingest_wheel(self,sample):
        self.runtime.command('wheel',sample);self.refresh_native()

    def context(self,payload):
        with self.lock:
            self.runtime.command('context',payload)
            if payload.get('kind')=='device_identity_changed':
                # The native owner already invalidated once. Mirror only the
                # original Python geometry bookkeeping, without another reset.
                self.last_fit=None;self.has_anchor=False;self.initialization=None
                self.pending_initial_pose=None;self.pending_reacquisition=None
                self.motion.events.clear()
            return self.refresh_native()

    def refresh_native(self):
        # This lock protects only Python's slow geometry/reference view; the
        # native input/control thread never acquires it or waits for the GIL.
        with self.lock:
            state=self.runtime.snapshot(dict(kind='state',consume_events=True))
            self.native_state=state
            revision=state.get('revision',self.revision)
            if revision!=self.revision:
                self.revision=revision;self.pending_reacquisition=None
            self.reference=state.get('reference')
            self.offset=np.asarray(state.get('offset',[0.,0.]),float)
            self.has_anchor=state.get('has_anchor',self.has_anchor)
            self.last_sample=state.get('last_sample') or None
            self.last_yaw=state.get('last_yaw_source',state.get('last_yaw')) or None
            def endpoints(first,last):
                return deque(([first] if first else [])+([last] if last and last!=first else []),maxlen=4096)
            self.poses=endpoints(state.get('first_pose'),state.get('last_pose'))
            self.yaws=endpoints(state.get('first_yaw'),state.get('last_yaw_row',state.get('last_yaw')))
            self.motion.events.extend(state.get('events',[]))
            return state

    def _frozen(self,first_ns=None,last_ns=None,*,latest=False):
        state=self.runtime.snapshot(dict(kind='window',first_ns=first_ns,last_ns=last_ns,latest=latest))
        if state.get('reference') is None:raise ValueError('尚未从围栏建立场地参考')
        return (np.asarray(state['poses'],float),np.asarray(state['yaws'],float),state['reference'],
                np.asarray(state['offset'],float),state['revision'])

    def align(self,*args,**kwargs):
        self.refresh_native()
        return super().align(*args,**kwargs)

    def request_initialization(self,description,*,physical_confirmed=True):
        self.refresh_native()
        with self.lock:
            if not self.poses or not self.yaws:raise ValueError('等待实际T265 Pose和主yaw数据')
            self.reference=None;self.revision+=1;self.offset[:]=0;self.has_anchor=False
            self.last_fit=None;self.latest_points=[];self.pending_initial_pose=None
            self.pending_reacquisition=None;self.last_initial_scan_id=None
            self.initialization=dict(status='SEARCHING_WALLS',description=description,
                physical_stationary_confirmed=physical_confirmed,initial_primary_yaw_rad=self.yaws[-1][1],
                initial_pose_yaw_rad=self.poses[-1][3],started_ns=time.monotonic_ns(),
                confirmations=0,attempts=0,branch_policy='initial_heading_nearest_positive_X_mod_90')
            self.runtime.command('clear_reference',dict(revision=self.revision,searching=True))
            return dict(self.initialization)

    def _initial_support(self,scan):
        # Keep the complete raw yaw support for the original unwrap rule;
        # initialization freeze is inside the original Python reference lock.
        state=self.runtime.snapshot(dict(kind='histories'))
        return state['poses'],state['yaws']

    def invalidate_reference(self,reason):
        with self.lock:
            self.reference=None;self.revision+=1;self.yaws.clear();self.last_fit=None
            self.has_anchor=False;self.initialization=None;self.pending_initial_pose=None
            self.pending_reacquisition=None
            self.runtime.command('invalidate',dict(reason=reason,revision=self.revision))
            self.refresh_native()
            return dict(event='reference_invalidated',reason=reason)

    def set_lidar_enabled(self,enabled):
        with self.lock:
            self.revision+=1;self.lidar_corrections_enabled=bool(enabled)
            self.last_fit=None;self.latest_points=[];self.pending_initial_pose=None
            self.pending_reacquisition=None
            if self.initialization:self.initialization['confirmations']=0
            self.runtime.command('lidar_state',dict(enabled=bool(enabled),revision=self.revision,
                reset_wall_fixes=True))

    def _apply_wall_motion(self,scan,fit,reference):
        result=self.runtime.command('apply_wall',dict(revision=self.revision,scan_id=scan.scan_id,
            reference_ns=scan.reference_ns,candidate_center_xy_m=fit.get('candidate_center_xy_m'),
            position_observed_axes=fit['position_observed_axes'],
            position_reacquired=fit.get('position_reacquired',False)))
        if result.get('applied') is False:raise ValueError('reference changed during scan fit')
        self.refresh_native()
        return np.asarray(result['current_xy'],float)

    def snapshot(self,now=None,*,include_display=True):
        state=self.refresh_native()
        result=dict(state.get('last') or dict(ready=False,reason='尚未从围栏建立场地参考'))
        result.update(scans=self.scan_count,fixes=self.fit_count)
        if not result.get('ready'):
            result['initialization']=dict(self.initialization) if self.initialization else None
            result['lidar_corrections_enabled']=self.lidar_corrections_enabled
        if include_display:
            result.update(last_fit=dict(self.last_fit) if self.last_fit else None,
                          points_field=list(self.latest_points))
        return result
