"""Strict ROS message boundary. Core never imports ROS."""
import math
from .core import finite


def pose_values(pose):
    p, q = pose.position, pose.orientation
    if not finite(p.x, p.y, p.z, q.x, q.y, q.z, q.w):
        raise ValueError('nonfinite pose')
    norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
    if abs(norm-1.0) > 0.01 or abs(p.z) > 0.01 or abs(q.x) > 0.01 or abs(q.y) > 0.01:
        raise ValueError('expected planar pose and normalized quaternion')
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    return p.x, p.y, yaw


def accept_path(controller, msg, now, wall):
    try:
        stamp = stamp_seconds(msg.header.stamp)
        points = []
        for pose in msg.poses:
            # Empty pose frame inherits Path frame; zero pose stamp inherits Path stamp.
            if pose.header.frame_id not in ('', msg.header.frame_id):
                raise ValueError('mixed path frames')
            if stamp_seconds(pose.header.stamp) not in (0.0, stamp):
                raise ValueError('mixed path stamps')
            x, y, _ = pose_values(pose.pose)
            points.append((x, y))
        return controller.set_path(points, msg.header.frame_id, stamp, now, wall)
    except (ValueError, TypeError, OverflowError):
        controller.invalidate_path()
        return False


def accept_odometry(controller, msg, now, wall):
    try:
        pose = pose_values(msg.pose.pose)
        twist = msg.twist.twist
        if not finite(twist.linear.x, twist.linear.y, twist.linear.z,
                      twist.angular.x, twist.angular.y, twist.angular.z,
                      *msg.pose.covariance, *msg.twist.covariance):
            raise ValueError('nonfinite odometry')
        return controller.set_odometry(*pose, msg.header.frame_id, msg.child_frame_id,
                                       stamp_seconds(msg.header.stamp), now, wall)
    except (ValueError, TypeError, OverflowError):
        controller.invalidate_odom()
        return False


def stamp_seconds(stamp):
    if not 0 <= stamp.nanosec < 1000000000:
        raise ValueError('invalid nanosecond field')
    return stamp.sec + stamp.nanosec * 1e-9
