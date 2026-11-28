from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('mrider_autonomy'), 'config', 'simulation.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=cfg),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='mrider_autonomy', executable='autonomy_node', output='screen',
             parameters=[LaunchConfiguration('config'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        Node(package='mrider_autonomy', executable='reference_preview_node', output='screen',
             parameters=[LaunchConfiguration('config'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
