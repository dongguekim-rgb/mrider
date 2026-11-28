#!/usr/bin/env python3
"""Read-only check of an already running visual_demo (same ROS_DOMAIN_ID)."""
import math
import time
import rclpy
from gazebo_msgs.srv import GetEntityState
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from tf2_msgs.msg import TFMessage
from mrider_autonomy.nodes import SENSOR_QOS, COMMAND_QOS


def main():
    rclpy.init()
    node = rclpy.create_node('check_visualization')
    odom, markers, transforms = [], [], []
    node.create_subscription(Odometry, '/mrider_sim/odometry', odom.append, SENSOR_QOS)
    node.create_subscription(MarkerArray, '/mrider_sim/markers', markers.append, COMMAND_QOS)
    node.create_subscription(TFMessage, '/tf', transforms.append, 10)
    client = node.create_client(GetEntityState, '/gazebo/get_entity_state')
    try:
        deadline = time.monotonic()+45
        while time.monotonic() < deadline and not (odom and markers and transforms and client.service_is_ready()):
            rclpy.spin_once(node, timeout_sec=.1)
        assert odom and markers and transforms, 'missing odometry, markers or TF'
        assert client.service_is_ready(), 'Gazebo state service unavailable'
        req = GetEntityState.Request(); req.name = 'mrider_visual'; req.reference_frame = 'world'
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
        assert future.done() and future.result().success, 'vehicle entity missing'
        pose = future.result().state.pose.position
        actual = odom[-1].pose.pose.position
        error = math.hypot(pose.x-actual.x, pose.y-actual.y)
        assert error < .15, ('Gazebo pose mirror mismatch', error)
        assert any(t.child_frame_id == 'base_link' for m in transforms for t in m.transforms)
        print({'gazebo_pose_error_m': error, 'markers': len(markers[-1].markers),
               'tf': 'map -> base_link', 'result': 'PASS'})
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
