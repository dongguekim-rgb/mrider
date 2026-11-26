"""Real Nav2/SLAM regression: travel 24 m, turn left, then turn right.

GT is confined to this evaluator. Driving inputs are operator goals only.
"""
import json
import math
import os
import time
import unittest
from pathlib import Path

import launch
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry, Path as NavPath, OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from mrider_autonomy.nodes import COMMAND_QOS, SENSOR_QOS
from mrider_navigation_sim.checks import Checks
from mrider_navigation_sim.vehicle_overlay import measured_parameters


@pytest.mark.launch_test
def generate_test_description():
    os.environ['ROS_DOMAIN_ID']='216'
    source=os.path.join(get_package_share_directory('mrider_navigation_sim'),'launch','sensor_only_demo.launch.py')
    return launch.LaunchDescription([
        launch.actions.SetEnvironmentVariable('ROS_DOMAIN_ID','216'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(source),launch_arguments={
            'rviz':'false','scenario':'large_office','goal_x':'24.0','auto_resume':'true','vehicle':'1'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class LargeOfficeTurns(unittest.TestCase):
    def test_reachable_paths_and_both_turns(self):
        rclpy.init();node=Checks();truth=[];validations=[];candidates=[];goals_seen=[];paths=[];maps={}
        node.create_subscription(Odometry,'/evaluation/ground_truth',truth.append,SENSOR_QOS)
        node.create_subscription(String,'/mrider_sim/path_validation',lambda m:validations.append(json.loads(m.data)),COMMAND_QOS)
        node.create_subscription(Twist,'/mrider_sim/candidate_cmd_vel',lambda m:candidates.append((m.linear.x,m.angular.z)),COMMAND_QOS)
        node.create_subscription(PoseStamped,'/mrider_sim/local_goal',lambda m:goals_seen.append([
            m.pose.position.x,m.pose.position.y,2*math.atan2(m.pose.orientation.z,m.pose.orientation.w)]),COMMAND_QOS)
        node.create_subscription(NavPath,'/mrider_sim/planned_path',lambda m:paths.append([
            [p.pose.position.x,p.pose.position.y,2*math.atan2(p.pose.orientation.z,p.pose.orientation.w)] for p in m.poses]),COMMAND_QOS)
        node.create_subscription(OccupancyGrid,'/map',lambda m:maps.__setitem__('last',m),
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        goal_pub=node.create_publisher(PoseStamped,'/goal_pose',COMMAND_QOS)
        share=Path(get_package_share_directory('mrider_navigation_sim'))
        limits,_=measured_parameters('1',str(share/'config/nav2_sensor_only.yaml'))
        goals=[(24.,0.,0.),(24.,16.,math.pi/2),(40.,16.,0.)]
        report={'case':'large_office_turns','legs':[]};started=time.monotonic()
        try:
            node.wait(lambda:truth and node.odom is not None and node.states,25.)
            node.assert_single_output();node.nav_ready()
            assert {x.node_name for x in node.get_subscriptions_info_by_topic('/evaluation/ground_truth')}=={'navigation_checks'}
            for index,(gx,gy,heading) in enumerate(goals):
                node.arm()
                if index==0:node.start()
                else:
                    goal=PoseStamped();goal.header.frame_id='map';goal.header.stamp=node.get_clock().now().to_msg()
                    goal.pose.position.x=gx;goal.pose.position.y=gy
                    goal.pose.orientation.z=math.sin(heading/2);goal.pose.orientation.w=math.cos(heading/2)
                    offset=len(node.mission_events);goal_pub.publish(goal)
                    node.wait(lambda:any(s in ('PLANNING','FOLLOWING') for s in node.mission_events[offset:]),5.)
                deadline=time.monotonic()+190.
                while time.monotonic()<deadline:
                    rclpy.spin_once(node,timeout_sec=.02)
                    state=node.states[-1][1]
                    assert not node.collision
                    if state['state']=='STOPPED':
                        assert state['reason'] in ('obstacle_stop','camera_obstacle'),state
                    if validations:assert all(v['accepted'] for v in validations),validations[-1]
                    if node.mission=='GOAL':break
                else:raise AssertionError(('goal_timeout',index,node.states[-1]))
                node.wait(lambda:node.states[-1][1]['state']=='GOAL' and abs(truth[-1].twist.twist.linear.x)<.01)
                p=truth[-1].pose.pose.position;q=truth[-1].pose.pose.orientation
                yaw=2*math.atan2(q.z,q.w);error=math.hypot(p.x-gx,p.y-gy)
                heading_error=abs(math.atan2(math.sin(yaw-heading),math.cos(yaw-heading)))
                assert error<.5 and heading_error<.3,(index,error,heading_error)
                report['legs'].append(dict(goal=[gx,gy,heading],goal_error_m=error,yaw=yaw,heading_error_rad=heading_error))
                print('TURN_GOAL '+json.dumps(report['legs'][-1]),flush=True)
            yaw=[leg['yaw'] for leg in report['legs']]
            assert math.atan2(math.sin(yaw[1]-yaw[0]),math.cos(yaw[1]-yaw[0]))>1.2
            assert math.atan2(math.sin(yaw[2]-yaw[1]),math.cos(yaw[2]-yaw[1]))<-1.2
            angles=[math.atan(limits['wheelbase']*w/v) for v,w in candidates if v>1e-6]
            assert max(map(abs,angles))<=limits['max_steering']+1e-3
            assert len(validations)>=7 and all(v['accepted'] for v in validations)
            assert not node.collision
            report.update(result='PASS',max_candidate_steering_rad=max(map(abs,angles)))
        except Exception as error:
            report.update(result='FAIL',error=str(error));raise
        finally:
            report.update(elapsed_s=time.monotonic()-started,validations=validations,
                local_goals=goals_seen,planned_paths=paths,candidate_commands=candidates,
                safety_history=node.states,mission_history=node.mission_events,trajectory=node.trace,
                collision=node.collision,min_clearance_m=min(node.clearances) if node.clearances else None,
                final_safety=node.states[-1][1] if node.states else None,
                evaluation_truth_trace=[[m.pose.pose.position.x,m.pose.pose.position.y,
                    2*math.atan2(m.pose.pose.orientation.z,m.pose.pose.orientation.w)] for m in truth])
            directory=Path(os.environ.get('MRIDER_SIM_REPORT_DIR','/tmp'));directory.mkdir(parents=True,exist_ok=True)
            if 'last' in maps:
                m=maps['last']
                (directory/'map.json').write_text(json.dumps(dict(width=m.info.width,height=m.info.height,
                    resolution=m.info.resolution,origin=[m.info.origin.position.x,m.info.origin.position.y],data=list(m.data))))
            (directory/'large_office_turns.json').write_text(json.dumps(report))
            print('LARGE_OFFICE_TURNS_RESULT '+json.dumps({k:v for k,v in report.items() if k not in
                ('evaluation_truth_trace','safety_history','mission_history','trajectory','planned_paths','candidate_commands')}),flush=True)
            node.destroy_node();rclpy.shutdown()


@launch_testing.post_shutdown_test()
class Exit(unittest.TestCase):
    def test_shutdown(self,proc_info):launch_testing.asserts.assertExitCodes(proc_info)
