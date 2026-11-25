"""Left/right obstacle detour followed by a pedestrian crossing."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    source=os.path.join(get_package_share_directory('mrider_navigation_sim'),'launch','navigation_sim.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('rviz',default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(source),launch_arguments={
            'scenario':'indoor_turns','goal_x':'20.0','auto_resume':'true',
            'rviz':LaunchConfiguration('rviz')}.items()),
    ])
