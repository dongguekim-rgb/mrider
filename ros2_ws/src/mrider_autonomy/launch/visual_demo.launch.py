import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('mrider_autonomy')
    gazebo = get_package_share_directory('gazebo_ros')
    return LaunchDescription([
        DeclareLaunchArgument('path_kind', default_value='curve', choices=['straight', 'curve']),
        DeclareLaunchArgument('gazebo', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(gazebo, 'launch', 'gazebo.launch.py')),
            condition=IfCondition(LaunchConfiguration('gazebo')),
            launch_arguments={'world': os.path.join(share, 'worlds', 'visual.world'),
                              'gui': LaunchConfiguration('gui')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'demo.launch.py')),
            launch_arguments={'path_kind': LaunchConfiguration('path_kind'),
                              'use_sim_time': LaunchConfiguration('gazebo')}.items()),
        Node(package='mrider_autonomy', executable='visualization_node', output='screen',
             parameters=[os.path.join(share, 'config', 'simulation.yaml'),
                         {'gazebo': LaunchConfiguration('gazebo'), 'use_sim_time': LaunchConfiguration('gazebo')}]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(share, 'rviz', 'demo.rviz')],
             parameters=[{'use_sim_time': LaunchConfiguration('gazebo')}]),
    ])
