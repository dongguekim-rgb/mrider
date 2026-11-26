"""The mission must reject a bad planner result BEFORE calling FollowPath."""
import json
import math
import time
from types import SimpleNamespace
import pytest
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.clock import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from mrider_navigation_sim.mission import Mission


class Publisher:
    def __init__(self):self.messages=[]
    def publish(self,msg):self.messages.append(msg)


def exercise(radius,frame='map',map_resolution=.05,stale=False):
    path=Path();path.header.frame_id=frame
    for i in range(91):
        angle=i/60;pose=PoseStamped();pose.pose.position.x=radius*math.sin(angle)
        pose.pose.position.y=radius*(1-math.cos(angle))
        pose.pose.orientation.z=math.sin(angle/2);pose.pose.orientation.w=math.cos(angle/2)
        path.poses.append(pose)
    clock=Clock();tf=TransformStamped();tf.header.stamp=clock.now().to_msg();tf.transform.rotation.w=1.
    if stale:tf.header.stamp.sec-=2
    requests=[]
    def send(req):
        requests.append(req)
        return SimpleNamespace(add_done_callback=lambda cb:None)
    node=SimpleNamespace(sequence=1,goal_extension=0.,state='PLANNING',map_resolution=.05,resume_plan_goal=None,
        local_map=SimpleNamespace(info=SimpleNamespace(resolution=map_resolution)),
        wheelbase=.533,max_steering=math.atan(.533/2.1),path_lookahead=2.,
        tf_buffer=SimpleNamespace(lookup_transform=lambda *args:tf),get_clock=lambda:clock,
        path_status=Publisher(),pub=Publisher(),path_pub=Publisher(),
        get_logger=lambda:SimpleNamespace(error=lambda message:None),
        follower=SimpleNamespace(send_goal_async=send),follow_accepted=lambda *args:None)
    node.poll_plan=lambda:Mission.poll_plan(node)
    node.execute_plan=lambda future,seq,**kwargs:Mission.execute_plan(node,future,seq,**kwargs)
    node.test_tf=tf
    future=SimpleNamespace(result=lambda:SimpleNamespace(status=4,result=SimpleNamespace(path=path)))
    Mission.planned(node,future,1)
    return node,requests


@pytest.mark.parametrize('radius,frame,resolution,reason',[
    (1.75,'map',.05,'path_curvature'),(3.,'wrong',.05,'path_frame'),
    (3.,'map',.1,'map_resolution_changed')])
def test_rejects_before_follow_action(radius,frame,resolution,reason):
    node,requests=exercise(radius,frame,resolution)
    assert not requests and not node.path_pub.messages and node.state=='FAILED'
    assert reason in json.loads(node.path_status.messages[-1].data)['reason']


def test_valid_plan_reaches_follow_action():
    node,requests=exercise(3.)
    assert len(requests)==1 and node.state=='FOLLOWING'
    assert json.loads(node.path_status.messages[-1].data)['accepted']


def test_waits_for_fresh_tf_without_driving():
    node,requests=exercise(3.,stale=True)
    assert node.state=='PLANNING' and not requests and not node.path_status.messages
    node.test_tf.header.stamp=node.get_clock().now().to_msg()
    node.poll_plan()
    assert len(requests)==1 and node.state=='FOLLOWING'


def test_tf_wait_is_bounded_and_does_not_relax_freshness():
    node,requests=exercise(3.,stale=True)
    future,seq,_=node.pending_plan
    node.pending_plan=(future,seq,time.monotonic()-1.1)
    node.poll_plan()
    assert not requests and node.state=='FAILED'
    assert json.loads(node.path_status.messages[-1].data)['reason']=='path_start_transform_stale'


def scan_buffer():
    scans=[]
    for ns in (9800000000,9900000000,10000000000):
        scan=LaserScan();scan.header.frame_id='laser'
        scan.header.stamp.sec,scan.header.stamp.nanosec=divmod(ns,1000000000)
        scans.append(scan)
    requested=[]
    def lookup(parent,child,stamp):
        requested.append(stamp.nanoseconds)
        if stamp.nanoseconds>9900000000:raise TransformException('TF still processing')
        return stamp.nanoseconds
    return SimpleNamespace(scans=scans,local_scan=scans[-1],
        tf_buffer=SimpleNamespace(lookup_transform=lookup)),requested


def test_scan_uses_available_exact_time_tf_within_freshness_limit():
    node,requested=scan_buffer()
    scan,tf=Mission.scan_with_transform(node,10.05)
    assert scan is node.scans[1] and tf==9900000000
    assert requested==[10000000000,9900000000]


def test_scan_buffer_does_not_hide_dropout_or_bad_frame():
    node,_=scan_buffer()
    with pytest.raises(ValueError,match='scan_stale'):Mission.scan_with_transform(node,10.5)
    node.local_scan.header.frame_id='wrong'
    with pytest.raises(ValueError,match='scan_not_ready'):Mission.scan_with_transform(node,10.05)


def test_unavailable_scan_tf_never_falls_back_to_latest_time():
    node,requested=scan_buffer();node.scans=node.scans[-1:]
    with pytest.raises(ValueError,match='scan_transform_not_ready'):
        Mission.scan_with_transform(node,10.05)
    assert requested==[10000000000]


def deferred_goal(reason='no_observed_progress_goal'):
    calls=[];goal=PoseStamped();goal.header.frame_id='map'
    node=SimpleNamespace(staged=True,safety='READY',state='GOAL',goal_pending=None,
        pub=Publisher(),begin=lambda msg:(calls.append(msg) or (False,reason)),
        get_logger=lambda:SimpleNamespace(warn=lambda m:None,error=lambda m:None))
    Mission.goal_cb(node,goal)
    return node,calls


def test_temporary_goal_rejection_is_retained_then_retried():
    node,calls=deferred_goal()
    assert node.state=='WAITING' and node.goal_pending is not None
    node.begin=lambda goal:(calls.append(goal) or (True,'planning'))
    Mission.retry_goal(node)
    assert node.goal_pending is None and len(calls)==2


def test_goal_wait_expires_without_starting_follower():
    node,_=deferred_goal();goal,_=node.goal_pending
    node.goal_pending=(goal,time.monotonic()-5.1)
    Mission.retry_goal(node)
    assert node.goal_pending is None and node.state=='FAILED'


def test_late_observation_cannot_start_an_expired_goal():
    node,calls=deferred_goal();goal,_=node.goal_pending
    node.goal_pending=(goal,time.monotonic()-5.1)
    node.begin=lambda msg:(calls.append(msg) or (True,'planning'))
    Mission.retry_goal(node)
    assert len(calls)==1 and node.goal_pending is None and node.state=='FAILED'


def test_disarm_cancels_pending_goal():
    node,calls=deferred_goal();node.safety='DISARMED'
    Mission.retry_goal(node)
    assert node.goal_pending is None and len(calls)==1


@pytest.mark.parametrize('reason',['invalid_goal','mission_busy','manual_arm_required'])
def test_invalid_or_unauthorized_goals_are_never_queued(reason):
    node,_=deferred_goal(reason)
    assert node.goal_pending is None and node.state=='GOAL'


@pytest.mark.parametrize('recovering,safety,elapsed,expected',[
    (True,'READY',1.,'WAITING'),(False,'READY',1.,'FAILED'),
    (True,'STOPPED',1.,'FAILED'),(True,'READY',16.,'FAILED')])
def test_no_path_retry_requires_live_obstacle_recovery_permission(recovering,safety,elapsed,expected):
    from action_msgs.msg import GoalStatus
    node,requests=exercise(3.)
    requests.clear();node.state='PLANNING';node.safety=safety
    node.resume_plan_goal=PoseStamped() if recovering else None
    node.resume_plan_started=time.monotonic()-elapsed
    node.get_logger=lambda:SimpleNamespace(warn=lambda msg:None)
    future=SimpleNamespace(result=lambda:SimpleNamespace(status=GoalStatus.STATUS_ABORTED,
                                                       result=SimpleNamespace(path=Path())))
    Mission.execute_plan(node,future,1)
    assert node.state==expected and not requests


def test_recovery_retry_only_plans_and_expiration_latches_failure():
    goal=PoseStamped();calls=[]
    node=SimpleNamespace(auto_resume=True,saved_goal=goal,safety='READY',state='WAITING',
        resume_plan_goal=goal,resume_plan_started=time.monotonic(),resume_plan_next=0.,
        pub=Publisher(),begin=lambda g,continuation:(calls.append((g,continuation)) or (True,'planning')))
    Mission.resume_tick(node)
    assert calls==[(goal,True)]
    node.resume_plan_next=0.;node.resume_plan_started=time.monotonic()-16.
    Mission.resume_tick(node)
    assert len(calls)==1 and node.state=='FAILED' and node.resume_plan_goal is None


def test_late_successful_plan_does_not_outlive_recovery_permission():
    node,requests=exercise(3.,stale=True)
    assert not requests
    node.test_tf.header.stamp=node.get_clock().now().to_msg()
    node.resume_plan_goal=PoseStamped()
    node.resume_plan_started=time.monotonic()-15.1
    node.poll_plan()
    assert not requests and node.state=='FAILED' and node.resume_plan_goal is None
