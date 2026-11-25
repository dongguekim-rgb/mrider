import os
import json
import math
import time
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
    os.environ['ROS_DOMAIN_ID']='213'
    source=os.path.join(get_package_share_directory('mrider_navigation_sim'),'launch','sensor_only_demo.launch.py')
    return launch.LaunchDescription([
        launch.actions.SetEnvironmentVariable('ROS_DOMAIN_ID','213'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(source),launch_arguments={'rviz':'false','scenario':'sensor_turns','auto_resume':'true','goal_x':'12.0','vehicle':os.environ.get('MRIDER_TEST_VEHICLE','0'),'floor_rgb':os.environ.get('MRIDER_TEST_FLOOR_RGB','[110,110,110]'),'image_gain':os.environ.get('MRIDER_TEST_IMAGE_GAIN','1.0')}.items()),
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
        node.turns=True
        report={'scenario':'sensor_turns', 'sensor_only':True,'vehicle':os.environ.get('MRIDER_TEST_VEHICLE','0')}
        try:
            node.wait(lambda:truth and maps and node.odom is not None,25.)
            assert all(p.node_name=='laser_odometry' for p in node.get_publishers_info_by_topic('/mrider_sim/odometry'))
            assert all(p.node_name=='slam_toolbox' for p in node.get_publishers_info_by_topic('/map'))
            subscribers=node.get_subscriptions_info_by_topic('/evaluation/ground_truth')
            assert all(p.node_name=='navigation_checks' for p in subscribers), 'truth leaked to controller'
            node.assert_single_output();node.nav_ready();node.arm();node.start()
            node.wait(lambda:node.states[-1][1]['state']=='STOPPED',50.)
            stopped_at=time.monotonic()
            stop=node.states[-1][1]
            assert stop['reason'] in ('camera_obstacle','obstacle_stop'),stop
            node.wait(lambda:abs(node.odom.twist.twist.linear.x)<.01)
            report['obstacle_stop']=stop['reason']
            node.wait(lambda:node.mission=='FOLLOWING' and node.states[-1][1]['state'] in ('TRACKING','SLOW'),35.)
            report['automatic_resume']=True
            report['stop_to_resume_s']=time.monotonic()-stopped_at
            assert report['stop_to_resume_s']>=3.,report
            node.wait(lambda:node.mission in ('GOAL','FAILED','STOPPED'),80.)
            assert node.mission=='GOAL',node.states[-1]
            node.wait(lambda:abs(truth[-1].twist.twist.linear.x)<.01)
            assert not node.collision
            steering=[m.drive.steering_angle for m in node.commands if m.drive.speed>.05]
            report['steering_min_rad']=min(steering);report['steering_max_rad']=max(steering)
            assert min(steering)<-.04 and max(steering)>.04,report
            headings=[2*math.atan2(m.pose.pose.orientation.z,m.pose.pose.orientation.w) for m in truth]
            turns=[math.atan2(math.sin(b-a),math.cos(b-a)) for a,b in zip(headings,headings[1:])]
            report['left_rotation_rad']=sum(max(0.,v) for v in turns)
            report['right_rotation_rad']=-sum(min(0.,v) for v in turns)
            assert report['left_rotation_rad']>.15 and report['right_rotation_rad']>.15,report
            report['navigation']='GOAL'
            gt=truth[-1].pose.pose.position;estimated=node.odom.pose.pose.position
            report['truth_goal_error_m']=math.hypot(gt.x-12,gt.y)
            report['odometry_position_error_m']=math.hypot(gt.x-estimated.x,gt.y-estimated.y)
            report['goal_reached_error_m']=report['truth_goal_error_m']
            assert report['truth_goal_error_m']<.5,report
            node.arm();node.inject=True
            node.wait(lambda:node.odom.twist.twist.linear.x>.1,5.)
            node.service('/mrider_sim/fault/camera_dropout',True)
            node.wait(lambda:node.states[-1][1]['reason'] in ('camera_input_stale','camera_status_timeout'),2.)
            node.inject=False
            node.service('/mrider_sim/fault/camera_dropout',False)
            node.spin(4.)
            assert node.states[-1][1]['state']=='STOPPED'
            report['camera_dropout_latched']=True
            report['result']='PASS'
        except Exception as error:
            report['result']='FAIL';report['error']=str(error)
            raise
        finally:
            if truth and node.odom:
                gt=truth[-1].pose.pose.position;est=node.odom.pose.pose.position
                report['truth_goal_error_m']=math.hypot(gt.x-12,gt.y)
                report['odometry_position_error_m']=math.hypot(gt.x-est.x,gt.y-est.y)
            report['camera_diagnostics']=camera
            report['trajectory']=node.trace
            report['safety_history']=node.states
            report['mission_history']=node.mission_events
            report['collision']=node.collision
            report['min_clearance_m']=min(node.clearances) if node.clearances else None
            report['final_safety']=node.states[-1][1] if node.states else None
            out=Path(os.environ.get('MRIDER_SIM_REPORT_DIR','/tmp'));out.mkdir(parents=True,exist_ok=True)
            (out/'sensor_turns.json').write_text(json.dumps(report))
            import cv2
            import numpy as np
            for topic,msg in images.items():
                channels=3 if msg.encoding=='rgb8' else 1
                array=np.frombuffer(bytes(msg.data),np.uint8).reshape(msg.height,msg.step)[:,:msg.width*channels]
                array=array.reshape(msg.height,msg.width,channels) if channels==3 else array
                if channels==3:array=cv2.cvtColor(array,cv2.COLOR_RGB2BGR)
                cv2.imwrite(str(out/(topic.rsplit('/',1)[-1]+'.png')),array)
            print('SENSOR_ONLY_RESULT '+json.dumps({k:v for k,v in report.items() if k not in ('trajectory','camera_diagnostics','safety_history','mission_history')}),flush=True)
            node.destroy_node();rclpy.shutdown()

@launch_testing.post_shutdown_test()
class Exit(unittest.TestCase):
    def test_shutdown(self,proc_info):launch_testing.asserts.assertExitCodes(proc_info)
