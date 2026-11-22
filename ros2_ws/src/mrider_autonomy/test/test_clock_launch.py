import time
import unittest
import pytest
import rclpy
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from mrider_autonomy.nodes import SENSOR_QOS, COMMAND_QOS


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(package='mrider_autonomy', executable='autonomy_node', parameters=[{'use_sim_time': True}]),
        launch_testing.actions.ReadyToTest(),
    ])


class ClockWatchdog(unittest.TestCase):
    def test_freeze_rewind_recovery(self):
        rclpy.init(); node = rclpy.create_node('clock_test')
        commands = []
        clock_pub = node.create_publisher(Clock, '/clock', 10)
        pp = node.create_publisher(Path, '/mrider_sim/path', SENSOR_QOS)
        op = node.create_publisher(Odometry, '/mrider_sim/odometry', SENSOR_QOS)
        node.create_subscription(AckermannDriveStamped, '/mrider_sim/command', commands.append, COMMAND_QOS)
        def send(t):
            clock = Clock(); clock.clock.sec = int(t); clock.clock.nanosec = int((t-int(t))*1e9)
            clock_pub.publish(clock)
            for _ in range(3): rclpy.spin_once(node, timeout_sec=.01)
            path = Path(); path.header.stamp = clock.clock; path.header.frame_id = 'map'
            for x in (0.,10.):
                pose = PoseStamped(); pose.pose.position.x=x; pose.pose.orientation.w=1.; path.poses.append(pose)
            odom = Odometry(); odom.header = path.header; odom.child_frame_id='base_link'; odom.pose.pose.orientation.w=1.
            pp.publish(path); op.publish(odom)
            rclpy.spin_once(node, timeout_sec=.02)
        try:
            for i in range(50): send(10+i*.02)
            self.assertTrue(any(m.drive.speed > 0 for m in commands))
            commands.clear(); end = time.monotonic()+1.
            while time.monotonic() < end: rclpy.spin_once(node, timeout_sec=.02)
            self.assertGreater(len(commands), 5)
            self.assertTrue(all(m.drive.speed == 0 for m in commands[-5:]))
            commands.clear()
            for i in range(40): send(5+i*.02)
            self.assertTrue(any(m.drive.speed > 0 for m in commands))
        finally:
            node.destroy_node(); rclpy.shutdown()


@launch_testing.post_shutdown_test()
class ProcessExit(unittest.TestCase):
    def test_clean_shutdown(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
