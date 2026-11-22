import os
import unittest
import pytest
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
from mrider_navigation_sim.checks import run

@pytest.mark.launch_test
def generate_test_description():
    # Separate DDS discovery from every other launch test.
    os.environ['ROS_DOMAIN_ID']='204'
    return launch.LaunchDescription([
        launch.actions.SetEnvironmentVariable('ROS_DOMAIN_ID','204'),
        launch_ros.actions.Node(package='mrider_navigation_sim',executable='world_node',parameters=[{'scenario':'clear'}]),
        launch_ros.actions.Node(package='mrider_navigation_sim',executable='safety_node'),
        launch_testing.actions.ReadyToTest(),
    ])

class Faults(unittest.TestCase):
    def test_faults(self):run('faults')

@launch_testing.post_shutdown_test()
class Exit(unittest.TestCase):
    def test_clean_shutdown(self,proc_info):launch_testing.asserts.assertExitCodes(proc_info)
