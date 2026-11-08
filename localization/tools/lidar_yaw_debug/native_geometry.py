"""Optional compiled numeric kernel; absent builds keep the same NumPy path.

No interpreter upgrade or Python ABI extension. CDLL releases the GIL while
the C kernel reads the contiguous input buffers; inputs remain owned here.
"""
import ctypes
import os
from pathlib import Path
import numpy as np

_library = None
_error = None
if os.environ.get('RDK_LIDAR_NATIVE', '1').lower() not in ('0','off','false'):
    try:
        _library = ctypes.CDLL(str(Path(__file__).resolve().parents[2] / 'build/liblidar_geometry.so'))
        _function = _library.lidar_best_hypothesis
        _function.argtypes = [ctypes.POINTER(ctypes.c_double),ctypes.c_size_t,
            ctypes.POINTER(ctypes.c_int64),ctypes.c_size_t,ctypes.c_double,ctypes.c_double,
            ctypes.POINTER(ctypes.c_uint8)]
        _function.restype = ctypes.c_int
    except (OSError, AttributeError) as error:
        _library = None
        _error = str(error)


def backend_status():
    return dict(backend='native_streaming_ransac' if _library is not None else 'numpy_ransac',
                load_error=_error,geometry_policy_changed=False)


def best_hypothesis(points, pairs, threshold, min_length=.15):
    if _library is None:return None
    points=np.ascontiguousarray(points,dtype=np.float64)
    pairs=np.ascontiguousarray(pairs,dtype=np.int64)
    mask=np.empty(len(points),dtype=np.uint8)
    index=_function(points.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),len(points),
        pairs.ctypes.data_as(ctypes.POINTER(ctypes.c_int64)),len(pairs),threshold,min_length,
        mask.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)))
    if index<0:raise ValueError('invalid internal line hypothesis buffers')
    return mask.view(np.bool_)
