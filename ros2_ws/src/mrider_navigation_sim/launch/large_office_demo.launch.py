"""Large multi-room sensor world; offline geometry verified, navigation experimental."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    source=os.path.join(get_package_share_directory('mrider_navigation_sim'),'launch','sensor_only_demo.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('rviz',default_value='false'),
        DeclareLaunchArgument('rviz_rendering',default_value='software',choices=['software','auto']),
        DeclareLaunchArgument('vehicle',default_value='1',choices=['0','1','2','3']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(source),launch_arguments={
            'scenario':'large_office','goal_x':'24.0','auto_resume':'false',
            'rviz_rendering':LaunchConfiguration('rviz_rendering'),'vehicle':LaunchConfiguration('vehicle'),'rviz':LaunchConfiguration('rviz')}.items()),
    ])
