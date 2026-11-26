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
    node=SimpleNamespace(sequence=1,goal_extension=0.,state='PLANNING',map_resolution=.05,
        local_map=SimpleNamespace(info=SimpleNamespace(resolution=map_resolution)),
        wheelbase=.533,max_steering=math.atan(.533/2.1),path_lookahead=2.,
        tf_buffer=SimpleNamespace(lookup_transform=lambda *args:tf),get_clock=lambda:clock,
        path_status=Publisher(),pub=Publisher(),path_pub=Publisher(),
        get_logger=lambda:SimpleNamespace(error=lambda message:None),
        follower=SimpleNamespace(send_goal_async=send),follow_accepted=lambda *args:None)
    node.poll_plan=lambda:Mission.poll_plan(node)
    node.execute_plan=lambda future,seq:Mission.execute_plan(node,future,seq)
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
