"""Audited geometry subset; no yaw-fitting tracker or control integration."""
import numpy as np

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


def extract_lines(points, seed=0, threshold=.025, min_points=12, min_span=.35):
    """RANSAC followed by TLS, with support length and robust residual checks."""
    points = np.asarray(points, float).reshape(-1,2)
    points = points[np.isfinite(points).all(axis=1)]
    rng = np.random.default_rng(seed)
    remaining = points.copy()
    result = []
    for _ in range(8):
        if len(remaining) < min_points:
            break
        best = np.zeros(len(remaining), dtype=bool)
        for _ in range(160):
            p,q = remaining[rng.choice(len(remaining),2,replace=False)]
            tangent=q-p
            length=np.linalg.norm(tangent)
            if length < .15:
                continue
            normal=np.array([-tangent[1],tangent[0]])/length
            mask=abs((remaining-p)@normal)<threshold
            if mask.sum()>best.sum():
                best=mask
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
            lo,hi=np.percentile(projection[group],[5,95])
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
