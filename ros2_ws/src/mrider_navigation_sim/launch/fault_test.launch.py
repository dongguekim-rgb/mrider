from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='mrider_navigation_sim',executable='world_node',parameters=[{'scenario':'clear'}],output='screen'),
        Node(package='mrider_navigation_sim',executable='safety_node',output='screen'),
    ])
