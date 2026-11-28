"""Sensor-only office detours and crossing; no ground-truth control input."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    source=os.path.join(get_package_share_directory('mrider_navigation_sim'),'launch','sensor_only_demo.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('rviz_rendering',default_value='software',choices=['software','auto']),
        DeclareLaunchArgument('vehicle',default_value='1',choices=['0','1','2','3']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(source),launch_arguments={
            'scenario':'sensor_turns','goal_x':'12.0','auto_resume':'true',
            'rviz_rendering':LaunchConfiguration('rviz_rendering'),'vehicle':LaunchConfiguration('vehicle'),'rviz':LaunchConfiguration('rviz')}.items()),
    ])
