import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('mrider_autonomy')
    return LaunchDescription([
        DeclareLaunchArgument('path_kind', default_value='curve', choices=['straight', 'curve']),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'simulation.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'autonomy.launch.py')),
            launch_arguments={'config': LaunchConfiguration('config'), 'use_sim_time': LaunchConfiguration('use_sim_time')}.items()),
        Node(package='mrider_autonomy', executable='simulator_node', output='screen',
             parameters=[LaunchConfiguration('config'), {'path_kind': LaunchConfiguration('path_kind'),
                         'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
