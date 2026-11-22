import os
import unittest
import pytest
import launch
import launch_testing.actions
import launch_testing.asserts
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from mrider_navigation_sim.checks import run

@pytest.mark.launch_test
def generate_test_description():
    # Separate DDS discovery from every other launch test.
    os.environ['ROS_DOMAIN_ID']='206'
    source=os.path.join(get_package_share_directory('mrider_navigation_sim'),'launch','navigation_sim.launch.py')
    return launch.LaunchDescription([
        launch.actions.SetEnvironmentVariable('ROS_DOMAIN_ID','206'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(source),launch_arguments={'scenario':'indoor','rviz':'false'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])

class Navigation(unittest.TestCase):
    def test_indoor(self):run('indoor')

@launch_testing.post_shutdown_test()
class Exit(unittest.TestCase):
    def test_clean_shutdown(self,proc_info):launch_testing.asserts.assertExitCodes(proc_info)
