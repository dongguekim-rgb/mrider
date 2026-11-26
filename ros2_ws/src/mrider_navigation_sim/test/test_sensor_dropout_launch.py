"""A sensor fault must stop driving AND release the active Nav2 action."""
import json
import os
from pathlib import Path
import time
import unittest

import launch
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from action_msgs.msg import GoalStatusArray
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy
from mrider_autonomy.nodes import SENSOR_QOS
from mrider_navigation_sim.checks import Checks


@pytest.mark.launch_test
def generate_test_description():
    os.environ['ROS_DOMAIN_ID']='218'
    source=os.path.join(get_package_share_directory('mrider_navigation_sim'),'launch','sensor_only_demo.launch.py')
    return launch.LaunchDescription([
        launch.actions.SetEnvironmentVariable('ROS_DOMAIN_ID','218'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(source),launch_arguments={
            'rviz':'false','vehicle':'1'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class SensorDropout(unittest.TestCase):
    def test_mid_drive_dropout_cancels_follow_path(self):
        rclpy.init();node=Checks();truth=[];statuses=[]
        node.create_subscription(Odometry,'/evaluation/ground_truth',truth.append,SENSOR_QOS)
        node.create_subscription(GoalStatusArray,'/follow_path/_action/status',statuses.append,
                                 QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        report={'case':'sensor_dropout'}
        try:
            node.nav_ready();node.assert_sensor_sources();node.assert_single_output()
            node.arm();node.start()
            node.wait(lambda:truth and truth[-1].twist.twist.linear.x>.1 and statuses)
            assert any(s.status==2 for s in statuses[-1].status_list), 'No executing action to cancel'
            before=len(node.commands);start=time.monotonic()
            assert node.service('/mrider_sim/fault/scan_dropout',True).success
            node.wait(lambda:node.states[-1][1]['state']=='STOPPED',1.)
            report['stop_detection_s']=time.monotonic()-start
            node.wait(lambda:abs(truth[-1].twist.twist.linear.x)<.01,1.)
            node.wait(lambda:statuses and statuses[-1].status_list and
                      all(s.status in (4,5,6) for s in statuses[-1].status_list),1.5)
            report['action_terminal_s']=time.monotonic()-start
            node.spin(.4)
            assert all(m.drive.speed==0. for m in node.commands[-10:])
            assert len(node.commands)>before and not node.collision
            assert node.states[-1][1]['state']=='STOPPED'
            report.update(result='PASS',terminal_statuses=[s.status for s in statuses[-1].status_list],
                          reason=node.states[-1][1]['reason'])
        except Exception as error:
            report.update(result='FAIL',error=str(error));raise
        finally:
            directory=Path(os.environ.get('MRIDER_SIM_REPORT_DIR','/tmp'));directory.mkdir(parents=True,exist_ok=True)
            (directory/'sensor_dropout.json').write_text(json.dumps(report))
            print('SENSOR_DROPOUT_RESULT '+json.dumps(report),flush=True)
            node.destroy_node();rclpy.shutdown()


@launch_testing.post_shutdown_test()
class Exit(unittest.TestCase):
    def test_shutdown(self,proc_info):launch_testing.asserts.assertExitCodes(proc_info)
