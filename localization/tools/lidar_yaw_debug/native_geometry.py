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
_extract = None
_extract_error = None
_extract_calls = _extract_fallbacks = 0
_last_extract_error = None
_svd_library = _svd = None


class _NativeLine(ctypes.Structure):
    _fields_ = [('count',ctypes.c_int64),('normal',ctypes.c_double*2),
                ('span',ctypes.c_double),('points',ctypes.c_double*64)]


_Pairs = ctypes.CFUNCTYPE(ctypes.c_int,ctypes.c_size_t,ctypes.POINTER(ctypes.c_int64))
_Order = ctypes.CFUNCTYPE(ctypes.c_int,ctypes.POINTER(ctypes.c_double),
                         ctypes.c_size_t,ctypes.POINTER(ctypes.c_int64))


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

if (_library is not None and
        os.environ.get('RDK_LIDAR_EXTRACT_NATIVE','1').lower() not in ('0','off','false')):
    try:
        abi = _library.lidar_extract_lines_abi_version
        abi.argtypes = [];abi.restype = ctypes.c_uint32
        if abi()!=1:raise ValueError('unsupported lidar extraction ABI')
        # Resolve the dependency of NumPy's actual linalg extension. Never
        # choose another system BLAS or the first arbitrary numpy.libs file.
        # The 64_ symbol explicitly supplies the required ILP64 signature.
        from numpy.linalg import _umath_linalg
        _svd_library = ctypes.CDLL(_umath_linalg.__file__)
        _svd = ctypes.cast(_svd_library.LAPACKE_dgesdd64_,ctypes.c_void_p)
        _extract = _library.lidar_extract_lines_v1
        _extract.argtypes = [ctypes.POINTER(ctypes.c_double),ctypes.c_size_t,
            ctypes.c_double,ctypes.c_size_t,ctypes.c_double,_Pairs,_Order,ctypes.c_void_p,
            ctypes.POINTER(_NativeLine),ctypes.c_size_t,ctypes.POINTER(ctypes.c_size_t)]
        _extract.restype = ctypes.c_int
    except (OSError,AttributeError,ImportError,ValueError) as error:
        _extract = None
        _extract_error = str(error)


def backend_status():
    return dict(backend='native_streaming_ransac' if _library is not None else 'numpy_ransac',
                load_error=_error,geometry_policy_changed=False,
                extraction_backend='cpp_v1_numpy_rng_sort_tls' if _extract is not None else 'python',
                extraction_load_error=_extract_error,extraction_calls=_extract_calls,
                extraction_fallbacks=_extract_fallbacks,extraction_last_error=_last_extract_error)


def extract_lines(points,seed=0,threshold=.025,min_points=12,min_span=.35):
    """Full numeric loop, or None to run the unchanged original Python path.

    Every call owns its buffers and callbacks. NumPy keeps the original seeded
    pair sequence, equal-projection ordering and exact linked SVD library.
    There is no device ownership, sample-age policy or position-source choice.
    """
    global _extract_calls,_extract_fallbacks,_last_extract_error
    if _extract is None:return None
    # Preserve mutable externally owned RNG semantics in the original path,
    # including a failed native attempt not consuming the caller's generator.
    if isinstance(seed,(np.random.Generator,np.random.BitGenerator)):return None
    points=np.ascontiguousarray(points,dtype=np.float64).reshape(-1,2)
    if not isinstance(min_points,(int,np.integer)) or min_points<2:return None
    rng=np.random.default_rng(seed);errors=[]

    @_Pairs
    def pairs(n,destination):
        try:
            first=rng.integers(n,size=160);second=rng.integers(n-1,size=160)
            second+=second>=first
            out=np.ctypeslib.as_array(destination,shape=(160,2))
            out[:,0]=first;out[:,1]=second
            return 1
        except BaseException as error:
            errors.append(error);return 0

    @_Order
    def sorted_order(projection,n,destination):
        try:
            values=np.ctypeslib.as_array(projection,shape=(n,))
            np.ctypeslib.as_array(destination,shape=(n,))[:]=np.argsort(values)
            return 1
        except BaseException as error:
            errors.append(error);return 0

    capacity=max(1,len(points)//min_points)
    output=(_NativeLine*capacity)();count=ctypes.c_size_t()
    _extract_calls+=1
    code=_extract(points.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),len(points),
        threshold,min_points,min_span,pairs,sorted_order,_svd,output,capacity,ctypes.byref(count))
    if errors:
        # ctypes must not swallow an interrupt inside a Python callback. Unwind
        # C++ normally first, then honour process/user cancellation in Python.
        if not isinstance(errors[0],Exception):raise errors[0]
    if code or errors or count.value>capacity:
        _extract_fallbacks+=1
        _last_extract_error='code='+str(code)+(' '+repr(errors[0]) if errors else '')
        return None  # No partial lines; caller recreates the original seeded run.
    result=[]
    for line in output[:count.value]:
        if not 2<=line.count<=32:
            _extract_fallbacks+=1;_last_extract_error='invalid native support count'
            return None
        result.append(dict(points=np.ctypeslib.as_array(line.points)[:2*line.count].reshape(-1,2).copy(),
            normal=np.ctypeslib.as_array(line.normal).copy(),span=float(line.span)))
    return result


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
