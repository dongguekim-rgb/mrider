import math
import time
import unittest
import pytest
import rclpy
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped
from mrider_autonomy.nodes import SENSOR_QOS, COMMAND_QOS


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(package='mrider_autonomy', executable='autonomy_node'),
        launch_ros.actions.Node(package='mrider_autonomy', executable='simulator_node', parameters=[{'path_kind': 'curve'}]),
        launch_testing.actions.ReadyToTest(),
    ])


class ClosedLoop(unittest.TestCase):
    def test_goal_and_limits(self):
        rclpy.init(); node = rclpy.create_node('closed_loop_test')
        odom, commands, states = [], [], []
        node.create_subscription(Odometry, '/mrider_sim/odometry', odom.append, SENSOR_QOS)
        node.create_subscription(AckermannDriveStamped, '/mrider_sim/command', commands.append, COMMAND_QOS)
        node.create_subscription(String, '/mrider_sim/control_status', states.append, COMMAND_QOS)
        try:
            deadline = time.monotonic()+45
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                if states and states[-1].data == 'goal_reached' and odom:
                    break
            self.assertTrue(states and states[-1].data == 'goal_reached')
            pos = odom[-1].pose.pose.position
            error = math.hypot(pos.x-4., pos.y-4.)
            print('curve ROS2 goal_error_m=', error, flush=True)
            self.assertLessEqual(error, .12)
            self.assertTrue(any(.01 < m.drive.speed < .4 for m in commands))
            self.assertTrue(all(0 <= m.drive.speed <= .8+1e-6 and abs(m.drive.steering_angle) <= .3927+1e-6 for m in commands))
            until = time.monotonic()+.4
            while time.monotonic() < until: rclpy.spin_once(node, timeout_sec=.02)
            self.assertEqual(commands[-1].drive.speed, 0.)
        finally:
            node.destroy_node(); rclpy.shutdown()


@launch_testing.post_shutdown_test()
class ProcessExit(unittest.TestCase):
    def test_clean_shutdown(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
