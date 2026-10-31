"""Separate display envelope, nominal safe sector, and observed bad-angle mask."""
import numpy as np
from .static import polar_points


def split_samples(samples, center, safe_fov, display_fov, bad_ranges=(), inverted=True):
    a=np.asarray(samples,float).reshape(-1,3)
    valid=np.isfinite(a).all(axis=1)&(a[:,1]>0)
    delta=abs((a[:,0]-center+180)%360-180)
    visible=valid&(delta<=display_fov/2)
    bad=np.zeros(len(a),dtype=bool)
    for lo,hi in bad_ranges:
        bad|=(a[:,0]>=lo)&(a[:,0]<hi)
    accepted=visible&(delta<=safe_fov/2)&~bad
    guard=visible&~accepted&~bad
    rejected=visible&bad
    convert=lambda mask:polar_points(a[mask],center,display_fov,inverted=inverted)
    return convert(accepted),convert(guard),convert(rejected)
