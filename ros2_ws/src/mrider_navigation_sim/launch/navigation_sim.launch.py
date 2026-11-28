import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share=get_package_share_directory('mrider_navigation_sim')
    config=os.path.join(share,'config','nav2.yaml')
    simulation=os.path.join(share,'config','simulation.yaml')
    with open(config) as source:
        lookahead=yaml.safe_load(source)['controller_server']['ros__parameters']['FollowPath']['lookahead_dist']
    return LaunchDescription([
        DeclareLaunchArgument('scenario',default_value='static_obstacle',choices=['clear','static_obstacle','blocked','indoor','indoor_dynamic','indoor_turns']),
        DeclareLaunchArgument('goal_x',default_value='8.0'),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('auto_resume',default_value='false'),
        Node(package='mrider_navigation_sim',executable='world_node',output='screen',parameters=[simulation,{'scenario':LaunchConfiguration('scenario')}]),
        Node(package='mrider_navigation_sim',executable='safety_node',output='screen',parameters=[simulation]),
        Node(package='mrider_navigation_sim',executable='mission_node',output='screen',parameters=[simulation,{'path_lookahead':lookahead,'auto_resume':LaunchConfiguration('auto_resume'),'goal_x':ParameterValue(LaunchConfiguration('goal_x'),value_type=float)}]),
        Node(package='nav2_planner',executable='planner_server',name='planner_server',output='screen',parameters=[config]),
        Node(package='nav2_controller',executable='controller_server',name='controller_server',output='screen',parameters=[config],
             remappings=[('cmd_vel','/mrider_sim/candidate_cmd_vel')]),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='navigation_lifecycle',output='screen',
             parameters=[{'autostart':True,'node_names':['planner_server','controller_server'],'bond_timeout':4.}]),
        Node(package='mrider_autonomy',executable='visualization_node',output='screen',parameters=[{'frame':'odom','rear_frame':'base_link','publish_tf':False}]),
        Node(package='rviz2',executable='rviz2',condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d',os.path.join(share,'rviz','navigation.rviz')]),
    ])
