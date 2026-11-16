import sys
from pathlib import Path
import numpy as np
sys.path[:0] = [str(Path(__file__).resolve().parents[1]), str(Path(__file__).resolve().parents[2]/'localization/tools')]
from localization_overlay import LocalizationOverlay
from lidar_yaw_debug.dynamic import solve_lines_position, matched_wall_display_points


def test_only_accepted_axis_support_is_drawn():
    def line(axis,d):
        p=np.column_stack([np.full(32,d),np.linspace(-1,1,32)])
        if axis:p=p[:,::-1]
        return dict(points=p,normal=np.eye(2)[axis],span=2.)
    lines=[line(0,1.3),line(1,-1.44),line(1,1.44)]
    fit,_=solve_lines_position(np.concatenate([v['points'] for v in lines]),np.array([.22,0,0]),np.zeros(2),lines)
    assert fit['position_observed_axes']==['x']
    points=matched_wall_display_points(lines,fit,0,np.array(fit['candidate_center_xy_m']))
    assert len(points)==32
    assert np.allclose(np.array(points)[:,0],1.5)
    assert matched_wall_display_points(lines,dict(position_observed_axes=[]),0,np.zeros(2))==[]


def test_overlay_preserves_raw_jump_and_clears_failed_scan_and_new_round():
    overlay=LocalizationOverlay()
    def state(x,reference=1,axes=('x',)):
        return dict(ready=True,reference=reference,t265_center_xy_m=[x,0.],
                    matched_points_field=[[1.5,0.]]*300,last_fit=dict(position_observed_axes=list(axes)))
    overlay.update(state(0));overlay.update(state(.82))
    assert list(overlay.trail)==[(0.,0.),(.82,0.)] and len(overlay.points)==256
    overlay.update(state(.83,axes=()))
    assert overlay.points==[]
    overlay.update(state(.3,reference=2))
    assert list(overlay.trail)==[(.3,0.)]
    for i in range(4100):overlay.update(state(i*.02,reference=2))
    assert len(overlay.trail)==4000
    canvas=np.zeros((120,120,3),dtype=np.uint8);overlay.draw(canvas,10,10,100)
    assert not canvas[:10].any() and not canvas[:,:10].any()  # Bad raw poses stay out of controls.
