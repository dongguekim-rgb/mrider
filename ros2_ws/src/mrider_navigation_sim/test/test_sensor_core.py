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
    for index,x in enumerate(np.linspace(0., 24., 481)):
        ranges = ray_distances(x, 0., angles, world, 15.)
        valid = ranges < 14.95
        ranges += rng.normal(0., .002, len(ranges))
        points = np.column_stack((ranges[valid] * np.cos(angles[valid]),
                                  ranges[valid] * np.sin(angles[valid])))
        pose, _ = estimator.update(points,1.+index*.1)
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


def test_stationary_scan_with_close_crossing_actor():
    """A moving foreground must not invalidate a well-supported static fit.

    Synthetic geometry generates observations only. The estimator receives
    laser-frame points, never the vehicle/actor pose or a known static mask.
    The old final 80% residual rejected seven consecutive scans in this case.
    """
    from mrider_navigation_sim.geometry import obstacles, ray_distances
    from mrider_navigation_sim.indoor import box
    from mrider_navigation_sim.large_office import triangle
    angles=np.linspace(-math.pi,math.pi,360,endpoint=False)
    rng=np.random.default_rng(42);estimator=KeyframeOdometry()
    world=obstacles('large_office')
    for t in np.arange(109.,118.,.1):
        actors=[box(24.,triangle(t,32.,.5)),box(triangle(t+25.,52.,.6),16.)]
        ranges=ray_distances(24.77,7.846,angles+1.802,world+actors,15.)
        valid=ranges<14.95
        ranges+=rng.normal(0.,.002,len(ranges))
        points=np.column_stack((ranges[valid]*np.cos(angles[valid]),ranges[valid]*np.sin(angles[valid])))
        pose,rmse=estimator.update(points,t)
        assert np.linalg.norm(pose[:2])<.03 and abs(pose[2])<.01
        assert rmse<=.08


def test_small_static_fragment_is_not_enough_for_localization():
    rng=np.random.default_rng(20)
    previous=rng.uniform(-3.,3.,(500,2))
    current=np.vstack((previous[:150],rng.uniform(30.,40.,(350,2))))
    with pytest.raises(ValueError,match='insufficient_static_matches'):
        register_scan(current,previous)


def test_collinear_matches_do_not_establish_full_pose():
    line=np.column_stack((np.linspace(-3.,3.,200),np.zeros(200)))
    with pytest.raises(ValueError,match='degenerate_scan_geometry'):
        register_scan(line,line)


@pytest.mark.parametrize('x',[24.9,25.5,26.])
def test_open_intersection_sampling_does_not_break_odometry(x):
    """Far surfaces change angular sampling at a widening intersection.

    No moving actors; known synthetic poses are evaluator assertions only.
    Scoring every supported return rejected 41--51 scans on these paths.
    """
    from mrider_navigation_sim.geometry import obstacles,ray_distances
    angles=np.linspace(-math.pi,math.pi,360,endpoint=False)
    rng=np.random.default_rng(73);estimator=KeyframeOdometry()
    world=obstacles('large_office')
    for y in np.arange(10.,16.,.035):
        ranges=ray_distances(x,y,angles+math.pi/2,world,15.)
        ranges+=rng.normal(0.,.002,len(ranges));valid=ranges<14.95
        points=np.column_stack((ranges[valid]*np.cos(angles[valid]),ranges[valid]*np.sin(angles[valid])))
        pose,rmse=estimator.update(points,1.+(y-10.)/.35)
        assert np.linalg.norm(pose[:2]-[y-10.,0.])<.1 and abs(pose[2])<.01
        assert rmse<=.08


def test_keyframe_age_refresh_preserves_motion_and_rejects_bad_input():
    rng=np.random.default_rng(91)
    reference=rng.uniform(-3.,3.,(500,2))
    estimator=KeyframeOdometry(max_reference_age=2.)
    estimator.update(reference,1.)
    before=estimator.reference
    estimator.update(reference-[.1,0.],2.)
    assert estimator.reference is before
    pose,_=estimator.update(reference-[.2,0.],3.1)
    assert estimator.reference is not before and estimator.reference_stamp==3.1
    assert np.linalg.norm(pose[:2]-[.2,0.])<.01
    anchor=estimator.reference
    with pytest.raises(ValueError):estimator.update(np.zeros((10,2)),6.)
    assert estimator.reference is anchor and estimator.reference_stamp==3.1
    with pytest.raises(ValueError,match='keyframe_timestamp'):
        estimator.update(reference,3.)
    pose,_=estimator.update(reference-[.25,0.],3.2)
    assert np.linalg.norm(pose[:2]-[.25,0.])<.01  # refresh never resets the pose
