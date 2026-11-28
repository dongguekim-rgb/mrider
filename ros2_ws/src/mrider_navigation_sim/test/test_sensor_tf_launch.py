"""SLAM must receive laser-frame scans and resolve their actual timestamps."""
import os
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from mrider_autonomy.nodes import SENSOR_QOS


@pytest.mark.launch_test
def generate_test_description():
    os.environ['ROS_DOMAIN_ID'] = '216'
    share = get_package_share_directory('mrider_navigation_sim')
    slam = Node(package='slam_toolbox', executable='async_slam_toolbox_node',
                name='slam_toolbox', output='screen',
                parameters=[os.path.join(share, 'config', 'slam_sensor_only.yaml')])
    env = {'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'}
    return launch.LaunchDescription([
        launch.actions.SetEnvironmentVariable('ROS_DOMAIN_ID', '216'),
        Node(package='mrider_navigation_sim', executable='laser_odometry_node',
             additional_env=env, parameters=[{'max_range_is_clear': True}]),
        Node(package='mrider_navigation_sim', executable='world_node', additional_env=env,
             parameters=[os.path.join(share, 'config', 'simulation.yaml'),
                         {'scenario': 'indoor', 'sensor_only': True}]),
        slam,
        launch_testing.actions.ReadyToTest(),
    ]), {'slam': slam}


class ScanTransformContract(unittest.TestCase):
    def test_map_and_scan_time_transforms(self):
        rclpy.init()
        node = rclpy.create_node('scan_transform_contract')
        scans, raw, odometry, maps = [], {}, {}, []
        buffer = Buffer()
        listener = TransformListener(buffer, node)

        def stamp(msg):
            return msg.header.stamp.sec, msg.header.stamp.nanosec

        node.create_subscription(LaserScan, '/scan', lambda m: raw.__setitem__(stamp(m), m), SENSOR_QOS)
        node.create_subscription(LaserScan, '/mrider_sim/slam_scan', scans.append, SENSOR_QOS)
        node.create_subscription(Odometry, '/mrider_sim/odometry',
                                 lambda m: odometry.__setitem__(stamp(m), m), SENSOR_QOS)
        node.create_subscription(OccupancyGrid, '/map', maps.append,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        try:
            deadline = time.monotonic() + 20.
            while time.monotonic() < deadline and (len(scans) < 50 or not maps):
                rclpy.spin_once(node, timeout_sec=.1)
            self.assertGreaterEqual(len(scans), 50)
            self.assertTrue(maps, 'SLAM never produced a map')
            self.assertTrue(all(m.header.frame_id == 'laser' for m in scans))
            matched = [m for m in scans[:-2] if stamp(m) in raw and stamp(m) in odometry]
            self.assertGreaterEqual(len(matched), 30)
            for scan in matched:
                source, odom = raw[stamp(scan)], odometry[stamp(scan)]
                self.assertEqual(scan.header, source.header)
                self.assertEqual(odom.header.frame_id, 'odom')
                self.assertEqual(odom.child_frame_id, 'base_link')
                self.assertTrue(buffer.can_transform('odom', 'laser', Time.from_msg(scan.header.stamp)))
                self.assertTrue(buffer.can_transform('odom', 'base_link', Time.from_msg(scan.header.stamp)))
            self.assertTrue(all(m.header.frame_id == 'map' and m.info.width > 0 for m in maps))
            print('SLAM_TF_CONTRACT scans={} matched={} maps={}'.format(len(scans), len(matched), len(maps)), flush=True)
        finally:
            listener.unregister()
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class ProcessResults(unittest.TestCase):
    def test_no_odom_pose_warning(self, proc_output, slam):
        output = ''.join(event.text.decode(errors='replace') for event in proc_output[slam])
        self.assertNotIn('Failed to compute odom pose', output)
        self.assertNotIn('Failed to compute laser pose', output)

    def test_shutdown(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
