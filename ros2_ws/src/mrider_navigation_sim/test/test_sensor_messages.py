"""ROS message contracts at the LiDAR odometry / SLAM boundary."""
import copy
import math
import json
from unittest.mock import patch

import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import LaserScan

from mrider_navigation_sim.geometry import obstacles, ray_distances
from mrider_navigation_sim.sensor_nodes import LaserOdometry


@pytest.fixture
def estimator(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '217')
    rclpy.init()
    node = LaserOdometry()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def sample(node):
    scan = LaserScan()
    scan.header.stamp = node.get_clock().now().to_msg()
    scan.header.frame_id = 'laser'
    scan.angle_min = -math.pi
    scan.angle_increment = 2 * math.pi / 360
    scan.angle_max = math.pi - scan.angle_increment
    scan.range_min, scan.range_max = .05, 15.
    scan.scan_time = .1
    angles = scan.angle_min + np.arange(360) * scan.angle_increment
    scan.ranges = ray_distances(0., 0., angles, obstacles('indoor'), 15.).tolist()
    return scan


def test_odometry_does_not_relabel_source_or_slam_scan(estimator):
    scan = sample(estimator)
    before = copy.deepcopy(scan)
    with patch.object(estimator.pub, 'publish') as odometry, \
            patch.object(estimator.slam_pub, 'publish') as slam, \
            patch.object(estimator.tf, 'sendTransform') as broadcast:
        estimator.scan(scan)
    odometry.assert_called_once()
    slam.assert_called_once()
    broadcast.assert_called_once()
    assert scan == before, 'odometry must not mutate its source LaserScan'
    adapted = slam.call_args.args[0]
    odom = odometry.call_args.args[0]
    transform = broadcast.call_args.args[0]
    assert adapted.header == before.header
    assert adapted.header.frame_id == 'laser'
    assert odom.header.frame_id == transform.header.frame_id == 'odom'
    assert odom.child_frame_id == transform.child_frame_id == 'base_link'
    assert odom.header.stamp == transform.header.stamp == before.header.stamp
    assert adapted.header is not odom.header
    assert adapted.angle_min == before.angle_min
    assert adapted.angle_increment == before.angle_increment
    assert adapted.range_min == before.range_min and adapted.range_max == before.range_max


def test_invalid_source_frame_is_not_forwarded_to_slam(estimator):
    scan = sample(estimator)
    scan.header.frame_id = 'wrong'
    with patch.object(estimator.pub, 'publish') as odometry, \
            patch.object(estimator.slam_pub, 'publish') as slam:
        estimator.scan(scan)
    odometry.assert_not_called()
    slam.assert_not_called()


def test_recent_references_do_not_bypass_source_gap_latch(estimator):
    scan=sample(estimator)
    def at(stamp):
        scan.header.stamp.sec=int(stamp)
        scan.header.stamp.nanosec=round((stamp-int(stamp))*1e9)
        with patch('mrider_navigation_sim.sensor_nodes.seconds',return_value=stamp):
            estimator.scan(scan)
    for stamp in (10.,10.1,10.2):at(stamp)
    assert len(estimator.estimator.recent)==3
    before=estimator.pose.copy()
    with patch.object(estimator.pub,'publish') as odometry, \
            patch.object(estimator.slam_pub,'publish') as slam, \
            patch.object(estimator.tf,'sendTransform') as transform, \
            patch.object(estimator.diagnostics,'publish') as status:
        at(10.6)
        at(10.7)
    odometry.assert_not_called();slam.assert_not_called();transform.assert_not_called()
    assert estimator.stamp==10.2
    np.testing.assert_array_equal(estimator.pose,before)
    assert all(json.loads(call.args[0].data)['reason']=='scan_gap_manual_restart_required'
               for call in status.call_args_list)
