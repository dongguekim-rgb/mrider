"""A2M4-R2 scan-only bringup. No guessed TF or vehicle command publisher."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', description='Verified /dev/serial/by-id/... A2M4-R2 path'),
        DeclareLaunchArgument('frame_id', default_value='laser'),
        Node(package='sllidar_ros2', executable='sllidar_node', output='screen',
             parameters=[{'channel_type': 'serial', 'serial_port': LaunchConfiguration('serial_port'),
                          'serial_baudrate': 115200, 'frame_id': LaunchConfiguration('frame_id'),
                          'inverted': False, 'angle_compensate': True}]),
    ])
