"""Explicit transport-only micro-ROS agent, no actuator command source."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', description='Verified Teensy USB serial path, not LiDAR'),
        Node(package='micro_ros_agent', executable='micro_ros_agent', output='screen',
             arguments=['serial', '--dev', LaunchConfiguration('serial_port')]),
    ])
