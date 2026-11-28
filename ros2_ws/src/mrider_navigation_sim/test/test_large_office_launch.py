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
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry, OccupancyGrid
from mrider_autonomy.nodes import SENSOR_QOS
from mrider_navigation_sim.checks import Checks
from rclpy.qos import QoSProfile, DurabilityPolicy

@pytest.mark.launch_test
def generate_test_description():
    os.environ['ROS_DOMAIN_ID']='214'
    source=os.path.join(get_package_share_directory('mrider_navigation_sim'),'launch','sensor_only_demo.launch.py')
    return launch.LaunchDescription([
        launch.actions.SetEnvironmentVariable('ROS_DOMAIN_ID','214'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(source),launch_arguments={'rviz':'false','scenario':'large_office','goal_x':'24.0','auto_resume':'true','vehicle':os.environ.get('MRIDER_TEST_VEHICLE','0'),'floor_rgb':os.environ.get('MRIDER_TEST_FLOOR_RGB','[110,110,110]'),'image_gain':os.environ.get('MRIDER_TEST_IMAGE_GAIN','1.0')}.items()),
        launch_testing.actions.ReadyToTest(),
    ])

class SensorOnly(unittest.TestCase):
    def test_sensor_only(self):
        rclpy.init();node=Checks();truth=[];maps=[];camera=[];images={};goals=[];transforms=[];scan_sample={}
        node.create_subscription(LaserScan,'/scan',lambda m:scan_sample.__setitem__('latest',m),SENSOR_QOS)
        def record_tf(msg):
            for t in msg.transforms:
                if t.header.frame_id in ('map','odom'):
                    q=t.transform.rotation
                    transforms.append([t.header.stamp.sec+t.header.stamp.nanosec*1e-9,
                        t.header.frame_id,t.child_frame_id,t.transform.translation.x,
                        t.transform.translation.y,2*math.atan2(q.z,q.w)])
        node.create_subscription(TFMessage,'/tf',record_tf,SENSOR_QOS)
        node.create_subscription(PoseStamped,'/mrider_sim/local_goal',goals.append,10)
        node.create_subscription(String,'/mrider_sim/camera_status',lambda m:camera.append(json.loads(m.data)),10)
        for topic in ['/camera/image_raw','/perception/overlay','/perception/labels']:
            node.create_subscription(Image,topic,lambda m,t=topic:images.__setitem__(t,m),SENSOR_QOS)
        node.create_subscription(Odometry,'/evaluation/ground_truth',truth.append,SENSOR_QOS)
        node.create_subscription(OccupancyGrid,'/map',maps.append,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        report={'sensor_only':True,'scenario':'large_office','vehicle':os.environ.get('MRIDER_TEST_VEHICLE','0')}
        try:
            node.wait(lambda:truth and maps and node.odom is not None,25.)
            node.nav_ready();node.assert_sensor_sources();node.assert_single_output();node.arm();node.start()
            # Moving actors now trigger early predictive stops. Permit only
            # obstacle-specific automatic restarts, never a sensor fault or
            # an operator arm in the middle of the route.
            deadline=time.monotonic()+180.
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.02)
                assert not node.collision
                assert node.mission!='FAILED',node.states[-1]
                if node.states[-1][1]['state']=='STOPPED':
                    assert node.states[-1][1]['reason'] in ('obstacle_stop','camera_obstacle'),node.states[-1]
                if node.mission=='GOAL':break
            assert node.mission=='GOAL',node.states[-1]
            node.wait(lambda:abs(truth[-1].twist.twist.linear.x)<.01)
            assert not node.collision
            assert len(goals)>=3, len(goals)
            assert goals[0].pose.position.x<10.
            assert abs(goals[-1].pose.position.x-24.)<.01
            report['local_goals']=[(g.pose.position.x,g.pose.position.y) for g in goals]
            report['navigation']='GOAL'
            gt=truth[-1].pose.pose.position;estimated=node.odom.pose.pose.position
            report['truth_goal_error_m']=math.hypot(gt.x-24,gt.y)
            report['odometry_position_error_m']=math.hypot(gt.x-estimated.x,gt.y-estimated.y)
            report['goal_reached_error_m']=report['truth_goal_error_m']
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
            report['evaluation_truth_trace']=[[m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                m.pose.pose.position.x,m.pose.pose.position.y] for m in truth]
            report['evaluation_tf_trace']=transforms
            report['local_goals']=[(g.pose.position.x,g.pose.position.y) for g in goals]
            if truth and node.odom:
                gt=truth[-1].pose.pose.position;est=node.odom.pose.pose.position
                report['truth_goal_error_m']=math.hypot(gt.x-24,gt.y)
                report['odometry_position_error_m']=math.hypot(gt.x-est.x,gt.y-est.y)
            report['camera_diagnostics']=camera
            report['trajectory']=node.trace
            report['mission_history']=node.mission_events
            report['safety_history']=node.states
            report['collision']=node.collision
            report['final_safety']=node.states[-1][1] if node.states else None
            out=Path(os.environ.get('MRIDER_SIM_REPORT_DIR','/tmp'));out.mkdir(parents=True,exist_ok=True)
            if maps:
                m=maps[-1]
                (out/'map.json').write_text(json.dumps({'width':m.info.width,'height':m.info.height,'resolution':m.info.resolution,'origin':[m.info.origin.position.x,m.info.origin.position.y],'data':list(m.data)}))
            if scan_sample:
                s=scan_sample['latest']
                (out/'scan.json').write_text(json.dumps({'stamp':s.header.stamp.sec+s.header.stamp.nanosec*1e-9,
                    'ranges':list(s.ranges),'angle_min':s.angle_min,'angle_increment':s.angle_increment,
                    'range_min':s.range_min,'range_max':s.range_max}))
            (out/'large_office.json').write_text(json.dumps(report))
            import cv2
            import numpy as np
            for topic,msg in images.items():
                channels=3 if msg.encoding=='rgb8' else 1
                array=np.frombuffer(bytes(msg.data),np.uint8).reshape(msg.height,msg.step)[:,:msg.width*channels]
                array=array.reshape(msg.height,msg.width,channels) if channels==3 else array
                if channels==3:array=cv2.cvtColor(array,cv2.COLOR_RGB2BGR)
                cv2.imwrite(str(out/(topic.rsplit('/',1)[-1]+'.png')),array)
            print('SENSOR_ONLY_RESULT '+json.dumps({k:v for k,v in report.items() if k not in ('trajectory','camera_diagnostics','mission_history','safety_history','evaluation_truth_trace','evaluation_tf_trace')}),flush=True)
            node.destroy_node();rclpy.shutdown()

@launch_testing.post_shutdown_test()
class Exit(unittest.TestCase):
    def test_shutdown(self,proc_info):launch_testing.asserts.assertExitCodes(proc_info)
