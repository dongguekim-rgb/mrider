"""Wait for sensor frames, then own only the RViz child process."""
import argparse
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException
from ament_index_python.packages import get_package_prefix


def renderer_env(mode):
    env=os.environ.copy()
    env['QT_X11_NO_MITSHM']='1'
    if mode=='software':
        env.update(LIBGL_ALWAYS_SOFTWARE='1',GALLIUM_DRIVER='llvmpipe',LP_NUM_THREADS='2',
                   __GLX_VENDOR_LIBRARY_NAME='mesa')
    return env


class Readiness(Node):
    def __init__(self):
        super().__init__('mrider_rviz_supervisor')
        self.map_seen=False;self.scan_time=None
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.create_subscription(OccupancyGrid,'/map',self.map_cb,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(LaserScan,'/scan',self.scan_cb,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))

    def map_cb(self,msg):
        self.map_seen=msg.header.frame_id=='map' and msg.info.width>0 and msg.info.height>0

    def scan_cb(self,msg):
        if msg.header.frame_id=='laser':self.scan_time=msg.header.stamp

    def ready(self):
        if not self.map_seen or self.scan_time is None:return False
        try:
            transform=self.buffer.lookup_transform('map','base_link',Time())
            self.buffer.lookup_transform('base_link','laser',Time())
            now=self.get_clock().now().nanoseconds*1e-9
            return all(-.05<=now-(s.sec+s.nanosec*1e-9)<=1.
                       for s in (transform.header.stamp,self.scan_time))
        except TransformException:return False


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--config',required=True)
    parser.add_argument('--rendering',choices=['software','auto'],default='software')
    parser.add_argument('--ready-timeout',type=float,default=60.)
    parser.add_argument('--max-restarts',type=int,default=0)
    options,ros_args=parser.parse_known_args()
    rclpy.init(args=ros_args);node=Readiness();child=None
    try:
        deadline=time.monotonic()+options.ready_timeout;next_log=0.
        while rclpy.ok() and not node.ready():
            if time.monotonic()>deadline:
                node.get_logger().error('RViz not started: map / fresh TF / scan not ready. Check ROS_DOMAIN_ID and SLAM logs.')
                return 1
            if time.monotonic()>=next_log:
                node.get_logger().info('Waiting for map, map->base_link->laser TF and fresh scan ...')
                next_log=time.monotonic()+5.
            rclpy.spin_once(node,timeout_sec=.1)
        if not rclpy.ok():return 0
        command=[os.path.join(get_package_prefix('rviz2'),'lib','rviz2','rviz2'),'-d',options.config]
        for attempt in range(options.max_restarts+1):
            node.get_logger().info(f'Starting RViz ({options.rendering}, attempt {attempt+1})')
            child=subprocess.Popen(command,env=renderer_env(options.rendering),start_new_session=True)
            while rclpy.ok() and child.poll() is None:rclpy.spin_once(node,timeout_sec=.1)
            if not rclpy.ok():return 0
            code=child.returncode
            # Closing the window intentionally must not reopen it.
            if code==0:return 0
            node.get_logger().error(f'RViz exited with {code}; simulation stays running.')
            if attempt<options.max_restarts:
                until=time.monotonic()+2.
                while rclpy.ok() and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.1)
        return 1
    except (KeyboardInterrupt,ExternalShutdownException):
        return 0
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGKILL);child.wait()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
