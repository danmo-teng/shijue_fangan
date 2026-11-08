"""Reviewed geometry. SE2 refinement is for startup only; runtime XY keeps primary yaw."""
import numpy as np
from dataclasses import dataclass, asdict
from .native_geometry import best_hypothesis,extract_lines as _native_extract_lines

def polar_points(samples, center_deg=270., fov_deg=180., inverted=True):
    """Raw packet angles are clockwise upright, counterclockwise inverted.

    Base axes: x forward, y left. Inverted default => (cos(raw), +sin(raw)).
    Upright => (cos(raw), -sin(raw)). Crop in RAW angle convention BEFORE
    conversion; neither reverse acquisition order nor edit device timestamps.
    Installation yaw around the new zero axis belongs in laser extrinsics.
    """
    if not 0 < fov_deg <= 360:
        raise ValueError('FOV must be in (0,360]')
    a = np.asarray(samples, dtype=float).reshape(-1, 3)
    d = (a[:, 0]-center_deg+180)%360-180
    keep = np.isfinite(a).all(axis=1) & (a[:, 1] > 0) & (abs(d) <= fov_deg/2)
    a = a[keep]
    theta = np.deg2rad(a[:, 0])
    return np.column_stack((np.cos(theta), (1 if inverted else -1)*np.sin(theta)))*a[:, 1, None]/1000


def transform(points, pose):
    x,y,t = pose
    c,s = np.cos(t),np.sin(t)
    return np.asarray(points) @ np.array([[c,s],[-s,c]]) + [x,y]


def _ordered_quantile(values, order, quantile):
    """NumPy's default linear quantile on an ALREADY sorted support group.

    Keep its nearer-end interpolation arithmetic, without copying, partitioning
    and validating the sorted group again for each of the two fixed quantiles.
    """
    rank=(len(order)-1)*quantile
    lower=int(rank);fraction=rank-lower
    a=float(values[order[lower]])
    b=float(values[order[min(lower+1,len(order)-1)]])
    difference=b-a
    return a+difference*fraction if fraction<.5 else b-difference*(1-fraction)


def extract_lines(points, seed=0, threshold=.025, min_points=12, min_span=.35):
    """RANSAC followed by TLS, with support length and robust residual checks."""
    native=_native_extract_lines(points,seed,threshold,min_points,min_span)
    return _extract_lines_python(points,seed,threshold,min_points,min_span) if native is None else native


def _extract_lines_python(points, seed=0, threshold=.025, min_points=12, min_span=.35):
    """Original extraction, retained for unavailable native builds and A/B."""
    points = np.asarray(points, float).reshape(-1,2)
    points = points[np.isfinite(points).all(axis=1)]
    rng = np.random.default_rng(seed)
    remaining = points.copy()
    result = []
    for _ in range(8):
        if len(remaining) < min_points:
            break
        # Uniform ordered pairs without replacement WITHIN each pair. All
        # n*(n-1) pairs remain equally likely; hypotheses may repeat, as before.
        # Two batch draws replace 160 tiny choice calls. The seeded sequence
        # changes, but hypothesis count, geometry gates and tie rule do not.
        first=rng.integers(len(remaining),size=160)
        second=rng.integers(len(remaining)-1,size=160)
        second += second >= first
        pairs=np.column_stack((first,second))
        best=best_hypothesis(remaining,pairs,threshold)
        if best is None:
            p=remaining[pairs[:,0]];tangent=remaining[pairs[:,1]]-p
            length=np.linalg.norm(tangent,axis=1);valid=length>=.15
            normal=np.column_stack([-tangent[:,1],tangent[:,0]])/np.maximum(length[:,None],1e-12)
            delta=remaining[:,None,:]-p[None,:,:]
            distance=delta[:,:,0]*normal[None,:,0]+delta[:,:,1]*normal[None,:,1]
            masks=(abs(distance)<threshold)&valid[None,:]
            best=masks[:,int(np.argmax(masks.sum(axis=0)))]
        if best.sum()<min_points:
            break
        fit=remaining[best]
        center=fit.mean(axis=0)
        _,_,vh=np.linalg.svd(fit-center,full_matrices=False)
        normal=vh[-1]
        best=abs((remaining-center)@normal)<threshold
        fit=remaining[best]
        remaining=remaining[~best]
        if len(fit)<min_points:
            continue
        center=fit.mean(axis=0)
        _,_,vh=np.linalg.svd(fit-center,full_matrices=False)
        tangent,normal=vh[0],vh[-1]
        projection=(fit-center)@tangent
        order=np.argsort(projection)
        # Do not connect isolated collinear obstacles across large empty gaps.
        groups=np.split(order,np.flatnonzero(np.diff(projection[order])>.15)+1)
        for group in groups:
            if len(group)<min_points:
                continue
            lo,hi=(_ordered_quantile(projection,group,q) for q in (.05,.95))
            if hi-lo < min_span:
                continue
            support=fit[group[np.linspace(0,len(group)-1,min(32,len(group))).astype(int)]]
            result.append({'points':support,'normal':normal,'span':float(hi-lo)})
    return result


def outer_supported_lines(lines, separation=.08, angle_deg=6.):
    """Prefer the outer supported envelope, not isolated maximum-range returns.
    Compare parallel lines on the SAME side of the sensor/body origin. Opposite
    enclosing walls must both survive. RANSAC support/span gates run first.
    """
    oriented=[]
    for line in lines:
        n=np.asarray(line['normal'],float).copy()
        d=float(np.median(line['points']@n))
        if d<0:n=-n;d=-d
        oriented.append((n,d))
    keep=[]
    for i,line in enumerate(lines):
        n,d=oriented[i]
        dominated=any(i!=j and float(n@m)>np.cos(np.deg2rad(angle_deg)) and e>d+separation
                      for j,(m,e) in enumerate(oriented))
        if not dominated:keep.append(line)
    return keep


@dataclass
class Fit:
    status: str
    pose: list
    rank: int
    wall_ids: list
    residual_rms_m: float | None
    information_eigenvalues: list
    unobservable_directions: list
    reason: str

    def json(self):
        return asdict(self)


def localize(points, prior, size=(3.,3.), extrinsic=(0.,0.,0.), max_correction_m=.10, _lines=None):
    """Local scan alignment only. prior=(map x,y,CCW rad), laser extrinsic in base.

    Tracking correction trust radius defaults to 0.10m; this is not global recovery.
    Wall association gate 0.20 m and 12 deg; caller must resolve wall identity/initial pose.
    A single wall updates normal position and yaw, preserving tangent position.
    Eigenvalues describe geometry only, not calibrated pose covariance.
    """
    prior=np.asarray(prior,float)
    if prior.shape!=(3,) or not np.isfinite(prior).all() or min(size)<=0 or not np.isfinite(max_correction_m) or max_correction_m<=0:
        raise ValueError('finite prior and positive arena dimensions required')
    base=transform(np.asarray(points,float).reshape(-1,2),extrinsic)
    lines=outer_supported_lines(extract_lines(base)) if _lines is None else _lines
    pose=prior.copy()
    normals=np.array([[1.,0.],[1.,0.],[0.,1.],[0.,1.]])
    distances=np.array([0.,size[0],0.,size[1]])
    assigned=[]
    for line in lines:
        wp=transform(line['points'],prior)
        n=transform([line['normal']], [0,0,prior[2]])[0]
        alignment=abs(normals@n)>np.cos(np.deg2rad(12))
        cost=np.median(abs(wp@normals.T-distances),axis=0)
        cost[~alignment]=np.inf
        wid=int(np.argmin(cost))
        tangent_axis=1 if wid<2 else 0
        in_segment=np.mean((wp[:,tangent_axis]>=-.05)&(wp[:,tangent_axis]<=size[tangent_axis]+.05))
        if cost[wid]<.20 and in_segment>.85:
            assigned.append((wid,line['points']))
    if not assigned:
        return Fit('ODOM_ONLY',prior.tolist(),0,[],None,[],np.eye(3).tolist(),'No supported wall compatible with the local prior')
    pts=np.concatenate([p for _,p in assigned])
    ids=np.concatenate([np.full(len(p),wid) for wid,p in assigned])
    ns=normals[ids]
    for _ in range(15):
        wp=transform(pts,pose)
        r=np.sum(wp*ns,axis=1)-distances[ids]
        rotated=wp-pose[:2]
        dyaw=np.column_stack((-rotated[:,1],rotated[:,0]))
        j=np.column_stack((ns,np.sum(ns*dyaw,axis=1)))
        w=np.minimum(1.,.02/np.maximum(abs(r),1e-12))
        a=j*np.sqrt(w[:,None]); b=-r*np.sqrt(w)
        delta=np.linalg.lstsq(a,b,rcond=1e-5)[0]
        pose+=delta
        if np.linalg.norm(delta)<1e-7:
            break
    wp=transform(pts,pose)
    r=np.sum(wp*ns,axis=1)-distances[ids]
    rotated=wp-pose[:2]
    j=np.column_stack((ns,np.sum(ns*np.column_stack((-rotated[:,1],rotated[:,0])),axis=1)))
    eig,vec=np.linalg.eigh(j.T@j/len(j))
    observable=eig>max(eig[-1]*1e-5,1e-9)
    rank=int(observable.sum())
    rms=float(np.sqrt(np.mean(r*r)))
    if rms>.035 or np.linalg.norm(pose[:2]-prior[:2])>max_correction_m or abs(pose[2]-prior[2])>np.deg2rad(12) or not (0<=pose[0]<=size[0] and 0<=pose[1]<=size[1]):
        return Fit('REJECTED',prior.tolist(),0,[],rms,eig.tolist(),np.eye(3).tolist(),'Residual, arena bounds, or correction gate failed')
    # Check coherent segments INCLUDING those not selected as mapped walls.
    # Isolated outliers must not veto a fit; a supported far wall outside the
    # enclosure contradicts an apparent inner-obstacle-to-wall alignment.
    for line in lines:
        segment_world=transform(line['points'],pose)
        outside=np.any((segment_world < -.06) | (segment_world > np.asarray(size)+.06),axis=1)
        if outside.sum() >= 6 and outside.mean() > .25:
            return Fit('REJECTED',prior.tolist(),0,[],rms,eig.tolist(),np.eye(3).tolist(),'Supported scan segment contradicts enclosing arena after correction')
    return Fit('WALL_XY_YAW' if rank==3 else 'WALL_PARTIAL',pose.tolist(),rank,sorted(set(ids.tolist())),rms,eig.tolist(),vec[:,~observable].T.tolist(),'Local association only; square-map global identity is not resolved')
