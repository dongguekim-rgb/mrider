import math
import pytest
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from mrider_autonomy.core import Controller, Config
from mrider_autonomy.ros_adapter import accept_path, accept_odometry
from mrider_autonomy.nodes import reference


def path():
    msg = Path()
    msg.header.frame_id = 'map'
    msg.header.stamp = Time(sec=10)
    for x in (0., 10.):
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.orientation.w = 1.
        msg.poses.append(pose)
    return msg


@pytest.mark.parametrize('kind', ['mixed_frame', 'mixed_stamp', 'nonplanar', 'quaternion', 'nan'])
def test_invalid_path(kind):
    c = Controller()
    msg = path()
    assert accept_path(c, msg, 10., 10.)
    msg.header.stamp.sec = 11
    if kind == 'mixed_frame': msg.poses[0].header.frame_id = 'wrong'
    if kind == 'mixed_stamp': msg.poses[0].header.stamp.sec = 10
    if kind == 'nonplanar': msg.poses[0].pose.position.z = 1.
    if kind == 'quaternion': msg.poses[0].pose.orientation.w = 0.
    if kind == 'nan': msg.poses[0].pose.position.x = math.nan
    assert not accept_path(c, msg, 11., 11.)
    assert c.path is None


def test_ros2_time_conversion():
    c = Controller(); msg = path(); msg.header.stamp.nanosec = 500000000
    assert accept_path(c, msg, 10.5, 10.5)
    assert c.path_stamp == 10.5


def test_invalid_odometry_clears_cache():
    c = Controller(); msg = Odometry()
    msg.header.frame_id, msg.child_frame_id = 'map', 'base_link'
    msg.header.stamp.sec = 10
    msg.pose.pose.orientation.w = 1.
    assert accept_odometry(c, msg, 10., 10.)
    msg.header.stamp.sec = 11; msg.twist.twist.linear.x = math.inf
    assert not accept_odometry(c, msg, 11., 11.)
    assert c.odom is None


@pytest.mark.parametrize('speed,steer', [(math.nan, 0), (-1., 0), (1., 0), (.2, 1.), (.2, math.inf)])
def test_reference_rejects_invalid(speed, steer):
    with pytest.raises(ValueError): reference(speed, steer, Config())


def test_reference_units_and_stop():
    v, yaw_rate = reference(.5, .2, Config())
    assert yaw_rate == pytest.approx(v*math.tan(.2)/.65)
    assert reference(0., .2, Config()) == (0., 0.)
