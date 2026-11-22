"""Building interior with moving pedestrian proxies and RViz2."""
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
        DeclareLaunchArgument('auto_resume',default_value='true'),
        DeclareLaunchArgument('scenario',default_value='indoor_dynamic',choices=['indoor','indoor_dynamic']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(source),launch_arguments={
            'auto_resume':LaunchConfiguration('auto_resume'),'scenario':LaunchConfiguration('scenario'),'rviz':LaunchConfiguration('rviz')}.items()),
    ])
