"""Recorded failure and adverse inputs for bounded recent-reference recovery."""
import gzip
import json
import math
from pathlib import Path

import numpy as np
import pytest

from mrider_navigation_sim.sensor_core import KeyframeOdometry, register_scan


def recorded():
    with gzip.open(Path(__file__).parent/'fixtures/parked_dynamic_scans.json.gz','rt') as stream:
        return json.load(stream)


def points(scan):
    ranges=np.asarray(scan['ranges'])
    angles=scan['angle_min']+np.arange(len(ranges))*scan['angle_increment']
    keep=np.isfinite(ranges)&(ranges>scan['range_min'])&(ranges<scan['range_max']-.05)
    return np.column_stack((ranges[keep]*np.cos(angles[keep]),ranges[keep]*np.sin(angles[keep])))


def test_recorded_pair_still_requires_recovery_without_lowering_quality():
    fixture=recorded();reference=points(fixture['scans'][0]);guess=np.zeros(3)
    for scan in fixture['scans'][1:]:
        info={}
        try:
            delta,yaw,_=register_scan(points(scan),reference,guess,.8,info)
            guess=np.r_[delta,yaw]
        except ValueError as error:
            assert str(error)=='insufficient_static_matches'
            assert scan['stamp']==fixture['first_rejected_stamp']
            assert (info['supported_points'],info['input_points'])==(147,217)
            return
    pytest.fail('Original rejection was hidden by changing the acceptance gate')


def test_recorded_parked_vehicle_recovers_with_two_fresh_agreeing_references():
    estimator=KeyframeOdometry();recoveries=[]
    for scan in recorded()['scans']:
        pose,rmse=estimator.update(points(scan),scan['stamp'])
        assert np.linalg.norm(pose[:2])<.03 and abs(pose[2])<.01
        assert rmse<=.08
        if estimator.diagnostics['registration_mode']=='recent_consensus':
            d=estimator.diagnostics;recoveries.append(scan['stamp'])
            assert d['primary_failure']=='insufficient_static_matches'
            assert len(d['recent_fits'])==2
            assert all(f['support_fraction']>=.7 and f['supported_points']>=60
                       and 0<f['age_s']<=.35 and f['rmse']<=.08 for f in d['recent_fits'])
            assert d['consensus_position_gap_m']<=.02 and d['consensus_yaw_gap_rad']<=.01
            assert estimator.reference_stamp==scan['stamp']
    assert recorded()['first_rejected_stamp'] in recoveries


def test_small_fragment_cannot_be_promoted_by_recent_references():
    rng=np.random.default_rng(20);cloud=rng.uniform(-3.,3.,(500,2))
    estimator=KeyframeOdometry()
    for stamp in (1.,1.1,1.2):estimator.update(cloud,stamp)
    old_pose=estimator.pose.copy();old_reference=estimator.reference
    old_recent=list(estimator.recent)
    corrupted=np.vstack((cloud[:150],rng.uniform(30.,40.,(350,2))))
    with pytest.raises(ValueError,match='insufficient_static_matches'):
        estimator.update(corrupted,1.3)
    np.testing.assert_array_equal(estimator.pose,old_pose)
    assert estimator.reference is old_reference and estimator.last_stamp==1.2
    assert all(a is b for a,b in zip(estimator.recent,old_recent))


@pytest.mark.parametrize('only_one_fresh',[False,True])
def test_stale_or_single_reference_cannot_authorize_recovery(only_one_fresh):
    fixture=recorded();scans=fixture['scans'];estimator=KeyframeOdometry()
    estimator.update(points(scans[0]),1.)
    # These are pure-core calls; the ROS boundary additionally rejects gaps.
    previous=next(s for s in scans if s['stamp']==1795719366.7202084)
    estimator.update(points(previous),2.6)
    failed=next(s for s in scans if s['stamp']==fixture['first_rejected_stamp'])
    before=estimator.pose.copy();reference=estimator.reference
    with pytest.raises(ValueError,match='recent_reference_unavailable'):
        estimator.update(points(failed),2.7 if only_one_fresh else 3.)
    np.testing.assert_array_equal(estimator.pose,before)
    assert estimator.reference is reference and estimator.last_stamp==2.6


def test_disagreeing_recent_pose_anchors_do_not_update_state():
    cloud=np.random.default_rng(8).uniform(-3.,3.,(500,2))
    estimator=KeyframeOdometry()
    for stamp in (1.,1.1,1.2):estimator.update(cloud,stamp)
    # Inject an inconsistent historical anchor. Both geometric fits alone
    # pass, but their poses in odom must disagree and prevent recovery.
    estimator.reference=cloud+20.
    estimator.recent[-2][2][0]+=.06
    before=estimator.pose.copy();reference=estimator.reference
    with pytest.raises(ValueError,match='recent_reference_disagreement'):
        estimator.update(cloud,1.3)
    np.testing.assert_array_equal(estimator.pose,before)
    assert estimator.reference is reference and estimator.last_stamp==1.2


@pytest.mark.parametrize('speed',[0.,.15,.3])
def test_two_crossing_actors_do_not_force_stationary_or_reset_moving_pose(speed):
    from mrider_navigation_sim.geometry import obstacles,ray_distances
    from mrider_navigation_sim.indoor import box
    angles=np.linspace(-math.pi,math.pi,360,endpoint=False)
    world=obstacles('large_office');rng=np.random.default_rng(17)
    origin=np.array([24.7632353112,13.3042846171]);heading=1.8494019748
    estimator=KeyframeOdometry();recovered=0
    for t in np.arange(0.,4.,.1):
        actual=origin+np.array([math.cos(heading),math.sin(heading)])*speed*t
        actors=[box(24.,12.65437-.5*t),box(22.214858+.6*t,16.)]
        ranges=ray_distances(*actual,angles+heading,world+actors,15.)
        ranges=np.where(ranges<15.,ranges+rng.normal(0.,.002,len(ranges)),ranges)
        valid=(ranges>.05)&(ranges<14.95)
        observed=np.column_stack((ranges[valid]*np.cos(angles[valid]),ranges[valid]*np.sin(angles[valid])))
        pose,rmse=estimator.update(observed,10.+t)
        assert np.linalg.norm(pose[:2]-[speed*t,0.])<.1 and abs(pose[2])<.02
        assert rmse<=.08
        recovered+=estimator.diagnostics['registration_mode']=='recent_consensus'
    if speed==0:assert recovered>0
    else:assert pose[0]>.5*speed*3.9
