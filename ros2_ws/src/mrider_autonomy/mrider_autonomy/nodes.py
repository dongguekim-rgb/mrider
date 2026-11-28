"""ROS2 boundaries. No node in this module publishes to a physical DBW topic."""
import math
import signal
import threading
import time
from dataclasses import fields
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped
from .core import Config, Controller, Command, finite
from .simulation import path_points, step
from .ros_adapter import accept_path, accept_odometry, stamp_seconds

# Sensor-compatible subscribers accept best-effort as well as reliable publishers.
SENSOR_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)
COMMAND_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)


def config(node):
    return Config(**{f.name: node.declare_parameter(f.name, f.default).value for f in fields(Config)})


def seconds(node):
    return node.get_clock().now().nanoseconds * 1e-9


class Autonomy(Node):
    def __init__(self):
        super().__init__('mrider_autonomy')
        self.c = config(self)
        self.controller = Controller(self.c)
        self.pub = self.create_publisher(AckermannDriveStamped, '/mrider_sim/command', COMMAND_QOS)
        self.status = self.create_publisher(String, '/mrider_sim/control_status', COMMAND_QOS)
        self.create_subscription(Path, '/mrider_sim/path', self.path_cb, SENSOR_QOS)
        self.create_subscription(Odometry, '/mrider_sim/odometry', self.odom_cb, SENSOR_QOS)
        # Steady-clock timer is essential: ROS time can pause or jump backwards.
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1 / self.c.control_rate, self.tick, clock=self.steady)

    def path_cb(self, msg):
        accept_path(self.controller, msg, seconds(self), time.monotonic())

    def odom_cb(self, msg):
        accept_odometry(self.controller, msg, seconds(self), time.monotonic())

    def publish(self, command):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.c.rear_frame
        msg.drive.speed, msg.drive.steering_angle = float(command.speed), float(command.steering)
        self.pub.publish(msg)
        self.status.publish(String(data=command.reason))

    def tick(self):
        self.publish(self.controller.command(seconds(self), time.monotonic()))

    def stop(self):
        self.publish(Command(reason='shutdown'))


class Simulator(Node):
    def __init__(self):
        super().__init__('mrider_simulator')
        self.c = config(self)
        self.points = path_points(self.declare_parameter('path_kind', 'curve').value)
        self.pose = (0., -0.2, 0.)
        self.command = (0., 0.)
        self.received = -math.inf
        self.previous = seconds(self)
        self.command_stamp = None
        self.path_pub = self.create_publisher(Path, '/mrider_sim/path', SENSOR_QOS)
        self.odom_pub = self.create_publisher(Odometry, '/mrider_sim/odometry', SENSOR_QOS)
        self.create_subscription(AckermannDriveStamped, '/mrider_sim/command', self.callback, COMMAND_QOS)
        self.create_timer(1 / self.c.control_rate, self.tick)

    def callback(self, msg):
        try:
            speed, steer = msg.drive.speed, msg.drive.steering_angle
            stamp = stamp_seconds(msg.header.stamp)
            age = seconds(self) - stamp
            valid = (finite(speed, steer, age) and stamp > 0 and
                     msg.header.frame_id == self.c.rear_frame and
                     -self.c.future_tolerance <= age <= self.c.input_timeout and
                     (self.command_stamp is None or stamp > self.command_stamp) and
                     0 <= speed <= self.c.max_speed + 1e-6 and abs(steer) <= self.c.max_steering + 1e-6)
        except (ValueError, TypeError):
            valid = False
        if valid:
            self.command, self.command_stamp = (speed, steer), stamp
            self.received = time.monotonic()
        else:
            self.command = (0., 0.)

    def tick(self):
        now = self.get_clock().now()
        current = now.nanoseconds * 1e-9
        elapsed = current - self.previous
        self.previous = current
        if elapsed < 0:
            self.command_stamp = None
            self.command = (0., 0.)
        if time.monotonic() - self.received > self.c.input_timeout:
            self.command = (0., 0.)
        self.pose = step(self.pose, *self.command, max(0., min(elapsed, .1)), self.c.wheelbase)
        path = Path()
        path.header.stamp, path.header.frame_id = now.to_msg(), self.c.frame
        for x, y in self.points:
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x, p.pose.position.y = float(x), float(y)
            p.pose.orientation.w = 1.
            path.poses.append(p)
        odom = Odometry()
        odom.header = path.header
        odom.child_frame_id = self.c.rear_frame
        x, y, yaw = self.pose
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        odom.twist.twist.linear.x = self.command[0]
        odom.twist.twist.angular.z = self.command[0]*math.tan(self.command[1])/self.c.wheelbase
        self.path_pub.publish(path)
        self.odom_pub.publish(odom)


def reference(speed, steering, c):
    """Ackermann -> body twist, never steering effort or PWM."""
    if (not finite(speed, steering) or not 0 <= speed <= c.max_speed + 1e-6
            or abs(steering) > c.max_steering + 1e-6):
        raise ValueError('invalid command')
    return speed, speed * math.tan(steering) / c.wheelbase


class ReferencePreview(Node):
    """Inspection-only output; not a replacement for mitt_hardware or safety gate."""
    def __init__(self):
        super().__init__('mrider_reference_preview')
        self.c = config(self)
        self.value = (0., 0.)
        self.received = -math.inf
        self.last_stamp = None
        self.previous_now = seconds(self)
        self.pub = self.create_publisher(Twist, '/mrider_sim/reference_preview', COMMAND_QOS)
        self.create_subscription(AckermannDriveStamped, '/mrider_sim/command', self.callback, COMMAND_QOS)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1/self.c.control_rate, self.tick, clock=self.steady)

    def callback(self, msg):
        try:
            stamp = stamp_seconds(msg.header.stamp)
            age = seconds(self) - stamp
            if (msg.header.frame_id != self.c.rear_frame or stamp <= 0 or
                    not -self.c.future_tolerance <= age <= self.c.input_timeout or
                    (self.last_stamp is not None and stamp <= self.last_stamp)):
                raise ValueError('invalid stamp/frame')
            self.value = reference(msg.drive.speed, msg.drive.steering_angle, self.c)
            self.received, self.last_stamp = time.monotonic(), stamp
        except (ValueError, TypeError, OverflowError):
            self.value = (0., 0.)

    def tick(self):
        now = seconds(self)
        if now < self.previous_now:
            self.value, self.last_stamp = (0., 0.), None
            self.received = -math.inf
        self.previous_now = now
        value = self.value
        if (time.monotonic()-self.received > self.c.input_timeout or
                self.last_stamp is None or now-self.last_stamp > self.c.input_timeout):
            value = (0., 0.)
        msg = Twist()
        msg.linear.x, msg.angular.z = value
        self.pub.publish(msg)


def main(cls):
    # Signals request shutdown; do not raise asynchronous KeyboardInterrupt
    # inside a DDS callback (it can interrupt handle cleanup while locked).
    stopping = threading.Event()
    def request_stop(signum, frame):
        stopping.set()
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = cls()
        executor.add_node(node)
        while rclpy.ok() and not stopping.is_set():
            executor.spin_once(timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None and rclpy.ok() and hasattr(node, 'stop'):
            node.stop()
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
