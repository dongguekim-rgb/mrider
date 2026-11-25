"""Experimental camera + LiDAR only; truth is evaluation-only."""
import os
import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import tempfile
import yaml
from mrider_navigation_sim.vehicle_overlay import measured_parameters
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def nodes(context):
    share=get_package_share_directory('mrider_navigation_sim')
    config=os.path.join(share,'config','nav2_sensor_only.yaml')
    simulation=os.path.join(share,'config','simulation.yaml')
    limits,nav=measured_parameters(LaunchConfiguration('vehicle').perform(context),config)
    if LaunchConfiguration('scenario').perform(context) in ('sensor_turns','large_office'):
        if nav is None:
            with open(config) as source:nav=yaml.safe_load(source)
        # Extra curvature margin and lower speed for sensor-estimated detours.
        planner=nav['planner_server']['ros__parameters']['GridBased']
        planner['minimum_turning_radius']=max(3.5,planner['minimum_turning_radius'])
        follower=nav['controller_server']['ros__parameters']['FollowPath']
        follower.update(desired_linear_vel=.35,lookahead_dist=2.,min_lookahead_dist=2.,max_lookahead_dist=2.)
        if LaunchConfiguration('scenario').perform(context)=='sensor_turns':
            # RPP can cut inside the planned arc. Reserve tracking clearance
            # outside the safety gate's 0.15 m margin; never weaken the gate.
            for costmap in ('global_costmap','local_costmap'):
                parameters=nav[costmap][costmap]['ros__parameters']
                parameters['footprint_padding']=max(.25,parameters['footprint_padding'])
    if nav is not None:
        with tempfile.NamedTemporaryFile(mode='w',prefix='mrider-sensor-vehicle-',suffix='.yaml',delete=False) as output:
            yaml.safe_dump(nav,output);config=output.name
    # Small matrix operations must not oversubscribe CPU threads in the sensor pipeline.
    with open(simulation) as source:body=yaml.safe_load(source)['/**']['ros__parameters']
    body.update(limits)
    goal_radius=math.hypot(max(body['front'],body['rear']),body['half_width'])+body['margin']
    sensor_env={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'}
    return [
        Node(package='mrider_navigation_sim',executable='laser_odometry_node',output='screen',additional_env=sensor_env,parameters=[{'max_range_is_clear':True}]),
        Node(package='mrider_navigation_sim',executable='camera_rules_node',output='screen',additional_env=sensor_env,parameters=[os.path.join(share,'config','perception_simulation.yaml')]),
        Node(package='slam_toolbox',executable='async_slam_toolbox_node',name='slam_toolbox',output='screen',parameters=[os.path.join(share,'config','slam_sensor_only.yaml')]),
        Node(package='mrider_navigation_sim',executable='world_node',output='screen',additional_env=sensor_env,parameters=[simulation,limits,{'scenario':LaunchConfiguration('scenario'),'sensor_only':True,'floor_rgb':yaml.safe_load(LaunchConfiguration('floor_rgb').perform(context)),'image_gain':ParameterValue(LaunchConfiguration('image_gain'),value_type=float)}]),
        Node(package='mrider_navigation_sim',executable='safety_node',output='screen',parameters=[simulation,limits,{'require_camera':True}]),
        Node(package='mrider_navigation_sim',executable='mission_node',output='screen',additional_env=sensor_env,parameters=[{'goal_clearance_radius':goal_radius,'max_range_is_clear':True,'staged_goals':LaunchConfiguration('scenario').perform(context)=='large_office','auto_resume':LaunchConfiguration('auto_resume'),'goal_extension':1.0 if LaunchConfiguration('scenario').perform(context) in ('sensor_turns','large_office') else 0.,'goal_x':ParameterValue(LaunchConfiguration('goal_x'),value_type=float)}]),
        Node(package='nav2_planner',executable='planner_server',name='planner_server',output='screen',parameters=[config]),
        Node(package='nav2_controller',executable='controller_server',name='controller_server',output='screen',parameters=[config],
             remappings=[('cmd_vel','/mrider_sim/candidate_cmd_vel')]),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='navigation_lifecycle',output='screen',
             parameters=[{'autostart':True,'node_names':['planner_server','controller_server'],'bond_timeout':4.}]),
        Node(package='mrider_autonomy',executable='visualization_node',output='screen',parameters=[{'frame':'odom','rear_frame':'base_link','publish_tf':False}]),
        Node(package='mrider_navigation_sim',executable='rviz_supervisor',output='screen',condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['--config',os.path.join(share,'rviz','sensor_only.rviz'),'--rendering',LaunchConfiguration('rviz_rendering')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario',default_value='indoor',choices=['clear','static_obstacle','blocked','indoor','indoor_dynamic','indoor_turns','sensor_turns','large_office']),
        DeclareLaunchArgument('goal_x',default_value='8.0'),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('rviz_rendering',default_value='software',choices=['software','auto']),
        DeclareLaunchArgument('auto_resume',default_value='false'),
        DeclareLaunchArgument('floor_rgb',default_value='[110,110,110]'),
        DeclareLaunchArgument('image_gain',default_value='1.0'),
        DeclareLaunchArgument('vehicle',default_value='0',choices=['0','1','2','3']),
        OpaqueFunction(function=nodes),
    ])
