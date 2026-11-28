import time
import unittest
import pytest
import rclpy
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from ackermann_msgs.msg import AckermannDriveStamped
from mrider_autonomy.nodes import SENSOR_QOS, COMMAND_QOS


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(package='mrider_autonomy', executable='autonomy_node'),
        launch_ros.actions.Node(package='mrider_autonomy', executable='reference_preview_node'),
        launch_testing.actions.ReadyToTest(),
    ])


class TopicContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('topic_contract_test')
        cls.commands, cls.references = [], []
        cls.pp = cls.node.create_publisher(Path, '/mrider_sim/path', SENSOR_QOS)
        cls.op = cls.node.create_publisher(Odometry, '/mrider_sim/odometry', SENSOR_QOS)
        cls.node.create_subscription(AckermannDriveStamped, '/mrider_sim/command', cls.commands.append, COMMAND_QOS)
        cls.node.create_subscription(Twist, '/mrider_sim/reference_preview', cls.references.append, COMMAND_QOS)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node(); rclpy.shutdown()

    def feed(self, kind='good'):
        stamp = self.node.get_clock().now().to_msg()
        path = Path(); path.header.stamp = stamp; path.header.frame_id = 'map'
        for x in (0., 10.):
            pose = PoseStamped(); pose.pose.position.x = x; pose.pose.orientation.w = 1.
            if kind == 'left': pose.pose.position.y = x*.3
            if kind == 'right': pose.pose.position.y = -x*.3
            path.poses.append(pose)
        odom = Odometry(); odom.header = path.header; odom.child_frame_id = 'base_link'
        odom.pose.pose.orientation.w = 1.
        if kind == 'frame': odom.child_frame_id = 'rear_axle'
        if kind == 'nan': path.poses[-1].pose.position.x = float('nan')
        if kind == 'future': path.header.stamp.sec += 2
        if kind == 'old': path.header.stamp.sec -= 2
        if kind == 'zero': path.header.stamp.sec = 0; path.header.stamp.nanosec = 0
        if kind == 'repeat':
            path.header.stamp = self.repeated_stamp
            odom.header.stamp = self.repeated_stamp
        self.pp.publish(path); self.op.publish(odom)

    def drive(self, duration, kind=None):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            if kind: self.feed(kind)
            rclpy.spin_once(self.node, timeout_sec=.02)

    def moving(self):
        self.drive(1., 'good')
        self.assertTrue(self.commands)
        self.assertGreater(self.commands[-1].drive.speed, 0.)

    def stopped(self):
        self.assertGreater(len(self.commands), 5)
        self.assertTrue(all(m.drive.speed == 0. for m in self.commands[-5:]))

    def test_sign_limit_and_preview(self):
        self.moving()
        for direction, sign in [('left', 1), ('right', -1)]:
            self.drive(.4, direction)
            cmd = self.commands[-1].drive
            self.assertGreater(sign*cmd.steering_angle, 0)
            self.assertLessEqual(abs(cmd.steering_angle), .3927+1e-6)
            self.assertGreater(sign*self.references[-1].angular.z, 0)

    def test_dropout_and_recovery(self):
        self.moving(); self.drive(.9); self.stopped()
        self.assertEqual(self.references[-1].linear.x, 0.)
        self.moving()

    def test_bad_frame(self):
        self.moving(); self.drive(.4, 'frame'); self.stopped()

    def test_nonfinite(self):
        self.moving(); self.drive(.4, 'nan'); self.stopped()

    def test_timestamps(self):
        for kind in ('future', 'old', 'zero'):
            self.moving(); self.drive(.4, kind); self.stopped()

    def test_repeated_timestamp(self):
        self.moving()
        self.repeated_stamp = self.node.get_clock().now().to_msg()
        self.drive(.8, 'repeat'); self.stopped()


@launch_testing.post_shutdown_test()
class ProcessExit(unittest.TestCase):
    def test_clean_shutdown(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
