# INTERNAL offline prototype overlay; never loaded by the installed entry.
_python_extract_lines=extract_lines
import ctypes as _ct
from pathlib import Path as _Path
class _NativeLine(_ct.Structure):
    _fields_=[('count',_ct.c_int64),('normal',_ct.c_double*2),('span',_ct.c_double),('points',_ct.c_double*64)]
_Rng=_ct.CFUNCTYPE(_ct.c_int,_ct.c_size_t,_ct.POINTER(_ct.c_int64))
_Order=_ct.CFUNCTYPE(_ct.c_int,_ct.POINTER(_ct.c_double),_ct.c_size_t,_ct.POINTER(_ct.c_int64))
_native=_ct.CDLL('/tmp/rdk_lidar_extract_prototype_20261109.so')
_svdlib=_ct.CDLL(str(next((_Path(np.__file__).parent.parent/'numpy.libs').glob('*openblas*'))))
_extract=_native.lidar_extract_prototype
_extract.argtypes=[_ct.POINTER(_ct.c_double),_ct.c_size_t,_ct.c_double,_ct.c_size_t,_ct.c_double,_Rng,_Order,_ct.c_void_p,_ct.POINTER(_NativeLine),_ct.c_size_t,_ct.POINTER(_ct.c_size_t)]
_extract.restype=_ct.c_int

def extract_lines(points,seed=0,threshold=.025,min_points=12,min_span=.35):
    points=np.ascontiguousarray(points,dtype=np.float64).reshape(-1,2)
    rng=np.random.default_rng(seed);errors=[]
    @_Rng
    def pairs(n,destination):
        try:
            a=rng.integers(n,size=160);b=rng.integers(n-1,size=160);b+=b>=a
            out=np.ctypeslib.as_array(destination,shape=(160,2));out[:,0]=a;out[:,1]=b
            return 1
        except Exception as exc:
            errors.append(str(exc));return 0
    @_Order
    def sorted_order(projection,n,destination):
        try:
            values=np.ctypeslib.as_array(projection,shape=(n,))
            np.ctypeslib.as_array(destination,shape=(n,))[:]=np.argsort(values)
            return 1
        except Exception as exc:
            errors.append(str(exc));return 0
    capacity=max(1,len(points)//min_points);output=(_NativeLine*capacity)();count=_ct.c_size_t()
    code=_extract(points.ctypes.data_as(_ct.POINTER(_ct.c_double)),len(points),threshold,min_points,min_span,
        pairs,sorted_order,_ct.cast(_svdlib.LAPACKE_dgesdd64_,_ct.c_void_p),output,capacity,_ct.byref(count))
    if code:raise ValueError('prototype result '+str(code)+' '+str(errors))
    return [dict(points=np.ctypeslib.as_array(line.points)[:2*line.count].reshape(-1,2).copy(),
        normal=np.ctypeslib.as_array(line.normal).copy(),span=float(line.span)) for line in output[:count.value]]
