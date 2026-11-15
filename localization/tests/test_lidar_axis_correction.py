"""Known square geometry: reproduce lost axes without devices or timing policy."""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from lidar_yaw_debug.dynamic import solve_lines_position, DynamicFence


def line(axis, distance):
    tangent = np.linspace(-1., 1., 32)
    points = np.column_stack((np.full(32, distance), tangent)) if axis == 0 else np.column_stack((tangent, np.full(32, distance)))
    return dict(points=points, normal=np.eye(2)[axis], span=2.)


def solve(lines, prior):
    points = np.concatenate([item['points'] for item in lines])
    return solve_lines_position(points, np.r_[prior, 0.], np.zeros(2), lines)[0]


def test_clear_wall_correction_is_not_vetoed_by_t265_distance():
    fit = solve([line(0, 1.30), line(1, -1.20)], [.34, -.3])
    assert fit['position_observed_axes'] == ['x','y']
    np.testing.assert_allclose(fit['candidate_center_xy_m'], [.2,-.3], atol=1e-9)


def test_bad_y_walls_do_not_discard_clear_x_wall():
    fit = solve([line(0, 1.3), line(1, -1.44), line(1, 1.44)], [.22,0.])
    assert fit['position_observed_axes'] == ['x']
    np.testing.assert_allclose(fit['candidate_center_xy_m'], [.2,0.], atol=1e-9)


def test_opposing_walls_recover_only_their_axis_despite_large_prior_error():
    for axis in (0,1):
        lines = [line(axis,-1.7),line(axis,1.3)]
        prior=np.array([2.,2.]);prior[axis]=-.6
        fit=solve(lines,prior)
        assert fit['position_observed_axes']==[['x'],['y']][axis]
        expected=prior.copy();expected[axis]=.2
        np.testing.assert_allclose(fit['candidate_center_xy_m'],expected,atol=1e-9)
        assert fit['position_reacquired']


def test_interior_parallel_surfaces_and_lone_distant_wall_do_not_recover_axis():
    for lines in ([line(0,-1.35),line(0,1.35)], [line(0,1.3)]):
        assert solve(lines,[-.6,0.])['rank_xy']==0


def test_missing_axis_can_recover_without_throwing_away_good_local_axis():
    lines=[line(0,-1.7),line(0,1.3),line(1,-1.2)]
    fit=solve(lines,[-.6,-.28])
    assert fit['position_observed_axes']==['x','y']
    np.testing.assert_allclose(fit['candidate_center_xy_m'],[.2,-.3],atol=1e-9)
    engine=DynamicFence({})
    applied=engine._fit_lines_position(np.concatenate([v['points'] for v in lines]),
                                     np.array([-.6,-.28,0.]),np.zeros(2),'known-square',lines)
    assert applied['position_observed_axes']==['x','y']
    assert applied['position_reacquired'] and engine.pending_reacquisition is None
