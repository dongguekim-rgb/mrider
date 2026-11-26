import os
import json
import math
import unittest
from pathlib import Path
import pytest
import launch
import launch_testing.actions
import launch_testing.asserts
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid
from mrider_autonomy.nodes import SENSOR_QOS
from mrider_navigation_sim.checks import Checks
from rclpy.qos import QoSProfile, DurabilityPolicy

@pytest.mark.launch_test
def generate_test_description():
    os.environ['ROS_DOMAIN_ID']='212'
    source=os.path.join(get_package_share_directory('mrider_navigation_sim'),'launch','sensor_only_demo.launch.py')
    return launch.LaunchDescription([
        launch.actions.SetEnvironmentVariable('ROS_DOMAIN_ID','212'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(source),launch_arguments={'rviz':'false','scenario':'indoor','vehicle':os.environ.get('MRIDER_TEST_VEHICLE','0'),'floor_rgb':os.environ.get('MRIDER_TEST_FLOOR_RGB','[110,110,110]'),'image_gain':os.environ.get('MRIDER_TEST_IMAGE_GAIN','1.0')}.items()),
        launch_testing.actions.ReadyToTest(),
    ])

class SensorOnly(unittest.TestCase):
    def test_sensor_only(self):
        rclpy.init();node=Checks();truth=[];maps=[];camera=[];images={}
        node.create_subscription(String,'/mrider_sim/camera_status',lambda m:camera.append(json.loads(m.data)),10)
        for topic in ['/camera/image_raw','/perception/overlay','/perception/labels']:
            node.create_subscription(Image,topic,lambda m,t=topic:images.__setitem__(t,m),SENSOR_QOS)
        node.create_subscription(Odometry,'/evaluation/ground_truth',truth.append,SENSOR_QOS)
        node.create_subscription(OccupancyGrid,'/map',maps.append,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        report={'sensor_only':True,'vehicle':os.environ.get('MRIDER_TEST_VEHICLE','0')}
        try:
            node.wait(lambda:truth and maps and node.odom is not None,25.)
            node.nav_ready();node.assert_sensor_sources();node.assert_single_output();node.arm();node.start()
            node.wait(lambda:node.mission in ('GOAL','FAILED','STOPPED'),70.)
            assert node.mission=='GOAL',node.states[-1]
            node.wait(lambda:abs(truth[-1].twist.twist.linear.x)<.01)
            assert not node.collision
            report['navigation']='GOAL'
            gt=truth[-1].pose.pose.position;estimated=node.odom.pose.pose.position
            report['truth_goal_error_m']=math.hypot(gt.x-8,gt.y)
            report['goal_reached_error_m']=report['truth_goal_error_m']
            report['odometry_position_error_m']=math.hypot(gt.x-estimated.x,gt.y-estimated.y)
            assert report['truth_goal_error_m']<.5,report
            node.arm();node.inject=True
            node.wait(lambda:node.odom.twist.twist.linear.x>.1,5.)
            node.service('/mrider_sim/fault/camera_dropout',True)
            node.wait(lambda:node.states[-1][1]['reason'] in ('camera_input_stale','camera_status_timeout'),2.)
            node.inject=False
            node.service('/mrider_sim/fault/camera_dropout',False)
            node.spin(.7)
            assert node.states[-1][1]['state']=='STOPPED'
            report['camera_dropout_latched']=True
            report['result']='PASS'
        except Exception as error:
            report['result']='FAIL';report['error']=str(error)
            raise
        finally:
            if truth and node.odom:
                gt=truth[-1].pose.pose.position;est=node.odom.pose.pose.position
                report['truth_goal_error_m']=math.hypot(gt.x-8,gt.y)
                report['odometry_position_error_m']=math.hypot(gt.x-est.x,gt.y-est.y)
            report['camera_diagnostics']=camera
            report['trajectory']=node.trace
            report['final_safety']=node.states[-1][1] if node.states else None
            out=Path(os.environ.get('MRIDER_SIM_REPORT_DIR','/tmp'));out.mkdir(parents=True,exist_ok=True)
            (out/'sensor_only.json').write_text(json.dumps(report))
            import cv2
            import numpy as np
            for topic,msg in images.items():
                channels=3 if msg.encoding=='rgb8' else 1
                array=np.frombuffer(bytes(msg.data),np.uint8).reshape(msg.height,msg.step)[:,:msg.width*channels]
                array=array.reshape(msg.height,msg.width,channels) if channels==3 else array
                if channels==3:array=cv2.cvtColor(array,cv2.COLOR_RGB2BGR)
                cv2.imwrite(str(out/(topic.rsplit('/',1)[-1]+'.png')),array)
            print('SENSOR_ONLY_RESULT '+json.dumps({k:v for k,v in report.items() if k not in ('trajectory','camera_diagnostics')}),flush=True)
            node.destroy_node();rclpy.shutdown()

@launch_testing.post_shutdown_test()
class Exit(unittest.TestCase):
    def test_shutdown(self,proc_info):launch_testing.asserts.assertExitCodes(proc_info)
