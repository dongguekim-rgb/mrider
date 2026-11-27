import math
import gzip
import json
from pathlib import Path
import numpy as np
import pytest

from mrider_navigation_sim.moving_obstacles import MotionTracks,collision_time,motion_fault
from mrider_navigation_sim.safety import Limits


def scan(pose,actor):
    # Ordered laser observations with three static surfaces and a small object.
    wall1=np.column_stack((np.linspace(-4.,4.,90),np.full(90,5.)))
    wall2=np.column_stack((np.full(90,-4.),np.linspace(-4.,5.,90)))
    wall3=np.column_stack((np.linspace(-4.,4.,90),np.full(90,-4.)))
    obj=np.column_stack((np.linspace(-.25,.25,9),np.zeros(9)))+actor
    world=np.vstack((wall1,wall2,wall3,obj));c,s=math.cos(pose[2]),math.sin(pose[2])
    local=(world-pose[:2])@np.array([[c,-s],[s,c]])
    return local[np.argsort(np.arctan2(local[:,1],local[:,0]))]


def test_moving_cluster_velocity_from_observations():
    tracker=MotionTracks();pose=np.zeros(3)
    for t in np.arange(0.,1.,.1):
        status=tracker.update(scan(pose,[2.,2.-.5*t]),pose,10.+t)
    assert status['valid'] and len(status['tracks'])==1
    track=status['tracks'][0]
    assert abs(track['vx'])<.02 and abs(track['vy']+.5)<.02
    assert track['observations']>=5 and track['residual_m']<.02


def test_ego_motion_does_not_turn_static_cluster_into_moving_actor():
    tracker=MotionTracks()
    for t in np.arange(0.,2.,.1):
        pose=np.array([.35*t,0.,.04*t])
        status=tracker.update(scan(pose,[2.,2.]),pose,10.+t)
        assert not status['tracks']


def test_recorded_static_face_change_does_not_predict_a_crossing_cart():
    with gzip.open(Path(__file__).parent/'fixtures/static_visibility_scans.json.gz','rt') as stream:
        scans=json.load(stream)['scans']
    tracker=MotionTracks()
    limits=Limits(wheelbase=.533,front=.76,rear=.23,half_width=.28,max_steering=.248560871)
    assert len(scans)==100
    for scan in scans:
        ranges=np.asarray(scan['ranges'])
        angles=scan['angle_min']+np.arange(len(ranges))*scan['angle_increment']
        keep=(ranges>scan['range_min'])&(ranges<scan['range_max']-.05)
        points=np.column_stack((ranges[keep]*np.cos(angles[keep]),ranges[keep]*np.sin(angles[keep])))
        x,y,z,w=scan['estimated_pose'];pose=np.array([x,y,2*math.atan2(z,w)])
        status=tracker.update(points,pose,scan['stamp'])
        # The old centre-only fit predicted the static cart moving north at
        # 0.617 m/s and stopped this unobstructed straight approach at x=3.45.
        assert collision_time(pose,.5,-.00135,limits,status['tracks']) is None


def test_short_track_does_not_claim_velocity():
    tracker=MotionTracks();pose=np.zeros(3)
    for t in (0.,.1,.2):
        status=tracker.update(scan(pose,[2.,2.-.5*t]),pose,10.+t)
        assert not status['valid'] and not status['tracks']


@pytest.mark.parametrize('stamp',[10.,9.,11.,math.nan])
def test_gap_or_repeated_motion_input_is_rejected(stamp):
    tracker=MotionTracks();pose=np.zeros(3)
    tracker.update(scan(pose,[2.,2.]),pose,10.)
    with pytest.raises(ValueError,match='motion_scan_gap'):
        tracker.update(scan(pose,[2.,2.]),pose,stamp)


def actor(x,y,vx,vy):return dict(x=x,y=y,vx=vx,vy=vy,radius=.5)


def test_predicts_approach_before_current_stopping_envelope_is_occupied():
    limits=Limits(wheelbase=.533,front=.76,rear=.23,half_width=.28,max_steering=.248560871)
    pose=(25.3,11.,1.9)
    assert collision_time(pose,.35,0.,limits,[actor(24.,15.8,0.,-.5)]) is not None
    # The old 4 s stationary envelope is clear. The extended uncertainty
    # envelope can conservatively reserve even this parked space later.
    assert collision_time(pose,0.,0.,limits,[actor(24.,15.8,0.,-.5)],horizon=4.) is None
    assert collision_time(pose,0.,0.,limits,[actor(24.,15.8,0.,-.5)])>4.


def test_receding_and_side_passing_actors_are_clear():
    limits=Limits()
    assert collision_time((0.,0.,0.),.35,0.,limits,[actor(4.,0.,.7,0.)]) is None
    assert collision_time((0.,0.,0.),.35,0.,limits,[actor(3.,3.,-.5,0.)]) is None
    assert collision_time((0.,0.,0.),.35,0.,limits,[]) is None


def test_head_on_actor_is_detected_before_turn_enters_its_lane():
    limits=Limits(wheelbase=.533,front=.76,rear=.23,half_width=.28,max_steering=.248560871)
    pose=(35.8,17.1,-.245)
    approaching=[actor(43.5,16.,-.6,0.)]
    assert collision_time(pose,.35,0.,limits,approaching,horizon=4.) is None
    assert collision_time(pose,.35,0.,limits,approaching) is not None
    # Staying at this earlier lateral offset leaves physical passing space.
    # The later recorded stop at (37.10,16.68) did not: the actor hit the car.
    assert pose[1]-limits.front*abs(math.sin(pose[2]))-limits.half_width-.25>16.


@pytest.mark.parametrize('change,now,wall,expected',[
    ({},10.1,10.1,None),
    ({'frame':'map'},10.1,10.1,'invalid_motion_prediction'),
    ({'stamp':11.},10.1,10.1,'motion_prediction_timeout'),
    ({},10.5,10.1,'motion_prediction_timeout'),
    ({},10.1,10.5,'motion_prediction_timeout'),
    ({'stamp':math.nan},10.1,10.1,'invalid_motion_prediction'),
    ({'valid':False},10.1,10.1,'motion_prediction_unready'),
    ({'tracks':[actor(2.,0.,math.nan,0.)]},10.1,10.1,'invalid_motion_prediction'),
    ({'tracks':[actor(2.,0.,2.,0.)]},10.1,10.1,'invalid_motion_prediction'),
])
def test_motion_contract(change,now,wall,expected):
    data=dict(frame='odom',stamp=10.,valid=True,tracks=[]);data.update(change)
    assert motion_fault((data,10.),now,wall)==expected


def test_missing_motion_input_fails_closed():
    assert motion_fault(None,10.,10.)=='motion_prediction_missing'
