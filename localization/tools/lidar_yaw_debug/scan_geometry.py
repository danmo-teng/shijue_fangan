"""Numerical-only scan work over immutable motion/reference support.

No live estimator, reference, pending reacquisition, device or control owner is
created here. The caller applies wall results against its current history.
"""
import math
import time
import numpy as np
from .native_geometry import backend_status


def geometry_job(kind,mount,scan,poses=None,yaws=None,reference=None,offset=None,*,extract=True):
    # Import lazily: DynamicFence can use this function as a callback without
    # circular module initialization or a recursive estimator instance.
    from .dynamic import accepted_points,DynamicFence,deskew_points,solve_lines_position
    from .static import extract_lines,outer_supported_lines
    from .global_init import initialize
    cpu_began=time.thread_time_ns();began=time.monotonic_ns()
    accepted=accepted_points(scan,mount)
    result=dict(accepted_count=len(accepted))
    if kind=='initialize':
        points=np.array([p.point_lidar_m for p in accepted]).reshape(-1,2)
        ext=[*mount['extrinsic'][:2],math.radians(mount['extrinsic'][2])]
        found,reason=initialize(points,extrinsic=ext)
        result.update(found=found,reason=reason)
    elif kind=='observe':
        result['lines']=[]
        if len(accepted)>=8:
            points=np.array([p.point_lidar_m for p in accepted])
            times=np.array([p.time_ns for p in accepted],np.int64)
            ext=np.array([*mount['extrinsic'][:2],math.radians(mount['extrinsic'][2])])
            query=DynamicFence.trajectory(poses,yaws,reference)
            corrected,ref=deskew_points(points,times,scan.reference_ns,query,ext)
            deskew_done=time.monotonic_ns()
            lines=outer_supported_lines(extract_lines(corrected)) if extract else []
            extract_done=time.monotonic_ns()
            local,found=(solve_lines_position(corrected,ref,np.zeros(2) if offset is None else offset,lines)
                         if extract else (None,None))
            result.update(corrected=corrected,ref=ref,lines=lines,local=local,found=found,
                deskew_ms=(deskew_done-began)/1e6,extract_ms=(extract_done-deskew_done)/1e6,
                solve_ms=(time.monotonic_ns()-extract_done)/1e6)
    else:
        raise ValueError('unknown numerical geometry job '+str(kind))
    result['backend']=backend_status()
    result['child_geometry_cpu_ms']=(time.thread_time_ns()-cpu_began)/1e6
    return result
