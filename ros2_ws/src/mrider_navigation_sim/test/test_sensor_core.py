import math
import numpy as np
import pytest
from mrider_navigation_sim.sensor_core import KeyframeOdometry, VelocityWindow, register_scan, floor_clear_fraction, slam_ranges

@pytest.mark.parametrize('angle,dx,dy',[(.03,.04,.01),(-.03,.04,-.01),(0.,0.,0.)])
def test_scan_registration(angle,dx,dy):
    rng=np.random.default_rng(19)
    previous=rng.uniform(-3,3,(500,2))
    r=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
    current=(previous-np.array([dx,dy]))@r
    delta,estimated,error=register_scan(current,previous)
    assert np.linalg.norm(delta-[dx,dy])<.005
    assert abs(estimated-angle)<.005 and error<.005

def test_scan_no_matches_fails():
    with pytest.raises(ValueError):register_scan(np.ones((10,2)),np.ones((10,2)))


def test_slam_copy_excludes_no_hit_without_mutating_scan():
    ranges=np.array([.05,2.,14.9,15.,np.nan,np.inf,0.,16.])
    adapted=slam_ranges(ranges,.05,15.,True)
    np.testing.assert_array_equal(adapted[:3],ranges[:3])
    assert np.isnan(adapted[3:]).all()
    assert ranges[3]==15. and np.isinf(ranges[5])
    assert slam_ranges([15.],.05,15.,False)[0]==15.
    with pytest.raises(ValueError):slam_ranges(ranges,0.,float('inf'),True)


def test_velocity_window_moving_then_stationary_noise():
    estimator=VelocityWindow()
    for i in range(10):
        v,w=estimator.update(np.array([i*.03,0.,0.]),1+i*.1)
    assert v==pytest.approx(.3) and w==0.
    for i in range(1,41):
        v,w=estimator.update(np.array([.27+(-1)**i*.001,0.,0.]),1.9+i*.1)
        if i>=4:assert abs(v)<.01
    with pytest.raises(ValueError):estimator.update(np.zeros(3),1.)


def test_keyframe_long_corridor():
    """24 m of noisy range observations; truth is used only to score the test."""
    from mrider_navigation_sim.geometry import ray_distances, obstacles
    angles = np.linspace(-math.pi, math.pi, 360, endpoint=False)
    world = obstacles('large_office')
    rng = np.random.default_rng(17)
    estimator = KeyframeOdometry()
    for x in np.linspace(0., 24., 481):
        ranges = ray_distances(x, 0., angles, world, 15.)
        valid = ranges < 14.95
        ranges += rng.normal(0., .002, len(ranges))
        points = np.column_stack((ranges[valid] * np.cos(angles[valid]),
                                  ranges[valid] * np.sin(angles[valid])))
        pose, _ = estimator.update(points)
    assert np.linalg.norm(pose[:2] - [24., 0.]) < .1, pose
    assert abs(pose[2]) < .01, pose
    before = estimator.pose.copy()
    with pytest.raises(ValueError):
        estimator.update(np.zeros((10, 2)))
    np.testing.assert_array_equal(estimator.pose, before)

def test_camera_floor_obstacle_and_invalid():
    image=np.full((240,320,3),110,dtype=np.uint8)
    assert floor_clear_fraction(image)==1.
    image[168:228,112:208]=[65,100,155]
    assert floor_clear_fraction(image)==0.
    with pytest.raises(ValueError):floor_clear_fraction(np.zeros((10,10)))
