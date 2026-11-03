"""Rectangle initialization from lidar walls, without position/yaw prior.
Square solutions are equivalent under 90deg rotation. Not generic room SLAM.
"""
import numpy as np
from .static import extract_lines,transform,localize,outer_supported_lines


def symmetries(pose,size):
    x,y,a=pose;w,h=size
    if abs(w-h)>1e-6:return [list(pose),[w-x,h-y,float((a+np.pi)%(2*np.pi))]]
    return [[float(x),float(y),float(a%(2*np.pi))], [float(w-y),float(x),float((a+np.pi/2)%(2*np.pi))], [float(w-x),float(w-y),float((a+np.pi)%(2*np.pi))], [float(y),float(w-x),float((a+3*np.pi/2)%(2*np.pi))]]


def initialize(points,size=(3,3),extrinsic=(0,0,0)):
    base=transform(points,extrinsic)
    lines=outer_supported_lines(extract_lines(base))
    if len(lines)<2:return None,'INSUFFICIENT_WALLS'
    supported=np.concatenate([line['points'] for line in lines])
    seeds=[]
    # Canonical orientation sector; all four equivalent map orientations are shown.
    period=np.pi/2 if abs(size[0]-size[1])<1e-6 else np.pi
    angles=[]
    for line in sorted(lines,key=lambda l:l['span'],reverse=True):
        n=line['normal'];a=float((-np.arctan2(n[1],n[0]))%period)
        if all(abs((a-old+period/2)%period-period/2)>.015 for old in angles):angles.append(a)
    for a in angles[:6]:
        rotated=transform(supported,[0,0,a]);xs=[];ys=[]
        for line in lines:
            n=transform([line['normal']],[0,0,a])[0]
            q=transform(line['points'],[0,0,a]);axis=int(abs(n[1])>abs(n[0]))
            if abs(n[axis])<np.cos(np.deg2rad(6)):continue
            c=float(np.median(q[:,axis]))
            values=[v for v in [-c,size[axis]-c] if .02<v<size[axis]-.02]
            (xs if axis==0 else ys).extend(values)
        for x in xs:
            for y in ys:
                p=rotated+[x,y]
                residual=np.min(abs(np.c_[p[:,0],size[0]-p[:,0],p[:,1],size[1]-p[:,1]]),axis=1)
                outside=np.any((p<-.06)|(p>np.asarray(size)+.06),axis=1)
                ratio=float(np.mean(residual<.04))
                if ratio<.45 or outside.mean()>.08:continue
                score=ratio-float(np.mean(np.minimum(residual,.1)))*2-outside.mean()*2
                seeds.append((score,[x,y,a]))
    candidates=[]
    for _,prior in sorted(seeds,key=lambda v:v[0],reverse=True)[:8]:
        fit=localize(points,prior,size,extrinsic,_lines=lines)
        if fit.rank!=3 or fit.status!='WALL_XY_YAW':continue
        p=transform(supported,fit.pose)
        residual=np.min(abs(np.c_[p[:,0],size[0]-p[:,0],p[:,1],size[1]-p[:,1]]),axis=1)
        outside=np.any((p<-.06)|(p>np.asarray(size)+.06),axis=1)
        ratio=float(np.mean(residual<.04))
        score=ratio-float(np.mean(np.minimum(residual,.1)))*2-outside.mean()*2
        if ratio<.45 or outside.mean()>.08:continue
        equiv=symmetries(fit.pose,size)
        if any(min(np.linalg.norm(np.array(q[:2])-np.array(c['pose'][:2]))+abs((q[2]-c['pose'][2]+np.pi)%(2*np.pi)-np.pi) for q in equiv)<.08 for c in candidates):continue
        candidates.append({'pose':fit.pose,'score':score,'support_fraction':ratio,'equivalent_poses':equiv})
    if not candidates:return None,'NO_RECTANGLE_MATCH'
    candidates.sort(key=lambda c:c['score'],reverse=True)
    if len(candidates)>1 and candidates[0]['score']-candidates[1]['score']<.08:
        return None,'AMBIGUOUS_NON_SYMMETRIC'
    return candidates[0],'GLOBAL_SYMMETRIC_FIX'


def initial_heading_branch(found, initial_yaw_delta=0., previous=None):
    """Rotate the entire square pose; never independently fold X and Y.

    At first acquisition choose the initial heading nearest +X, retaining its
    measured wall angle. Then hold that branch while confirming the match.
    Input/output here use the patch's [0,3] corner coordinates.
    """
    def wrapped(a):return (a+np.pi)%(2*np.pi)-np.pi
    choices=found['equivalent_poses']
    if previous is not None:
        return min(choices,key=lambda q:np.linalg.norm(np.array(q[:2])-previous[:2])+abs(wrapped(q[2]-previous[2])))
    return min(choices,key=lambda q:(abs(wrapped(q[2]-initial_yaw_delta)),wrapped(q[2]-initial_yaw_delta)))
