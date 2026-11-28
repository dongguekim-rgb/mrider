"""Record what the twin does into a pickle + camera PNGs, for plots/video later."""
import pickle
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan, Image, JointState
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped
import tf2_ros

out = sys.argv[1]
duration = float(sys.argv[2])


class Rec(Node):
    def __init__(self):
        super().__init__('twin_recorder',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.d = dict(pose=[], scan=[], plan=[], map=None, cmd=[], steer=[],
                      goal=[], cam=[])
        be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.buf = tf2_ros.Buffer()
        self.tl = tf2_ros.TransformListener(self.buf, self)
        self.create_subscription(LaserScan, '/scan', self.on_scan, be)
        self.create_subscription(Path, '/plan', self.on_plan, 5)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(JointState, '/joint_states', self.on_js, be)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 5)
        self.create_subscription(Image, '/camera/image_raw', self.on_cam, be)
        self.create_timer(0.1, self.tick)
        self.last_cam = -1e9
        self.last_scan = None

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def pose(self):
        try:
            t = self.buf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return (t.transform.translation.x, t.transform.translation.y, yaw)

    def tick(self):
        p = self.pose()
        if p:
            self.d['pose'].append((self.now(),) + p)

    def on_scan(self, m):
        p = self.pose()
        if p is None:
            return
        r = np.array(m.ranges, dtype=np.float32)
        self.d['scan'].append((self.now(), p, m.angle_min, m.angle_increment, r))

    def on_plan(self, m):
        pts = np.array([(q.pose.position.x, q.pose.position.y) for q in m.poses])
        self.d['plan'].append((self.now(), pts))

    def on_map(self, m):
        self.d['map'] = (self.now(), m.info.resolution, m.info.origin.position.x,
                         m.info.origin.position.y, m.info.width, m.info.height,
                         np.array(m.data, dtype=np.int8))

    def on_cmd(self, m):
        self.d['cmd'].append((self.now(), m.linear.x, m.angular.z))

    def on_js(self, m):
        if 'front_left_steer_joint' in m.name:
            i = m.name.index('front_left_steer_joint')
            j = m.name.index('front_right_steer_joint')
            self.d['steer'].append((self.now(), m.position[i], m.position[j]))

    def on_goal(self, m):
        self.d['goal'].append((self.now(), m.pose.position.x, m.pose.position.y))

    def on_cam(self, m):
        t = self.now()
        if t - self.last_cam < 0.5:
            return
        self.last_cam = t
        a = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
        if m.encoding.startswith('bgr'):
            a = a[:, :, ::-1]
        self.d['cam'].append((t, a.copy()))


rclpy.init()
n = Rec()
t0 = time.time()
while rclpy.ok() and time.time() - t0 < duration:
    rclpy.spin_once(n, timeout_sec=0.05)
with open(out, 'wb') as f:
    pickle.dump(n.d, f)
print('saved', out, {k: (len(v) if isinstance(v, list) else v is not None) for k, v in n.d.items()})
