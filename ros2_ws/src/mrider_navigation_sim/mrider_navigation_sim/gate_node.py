import json
import time
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from mrider_autonomy.nodes import SENSOR_QOS, COMMAND_QOS, seconds
from mrider_autonomy.ros_adapter import pose_values, stamp_seconds
from .safety import Gate, load_limits


class SafetyNode(Node):
    def __init__(self):
        super().__init__('mrider_safety_gate');self.gate=Gate(load_limits(self))
        self.create_subscription(LaserScan,'/scan',self.scan,SENSOR_QOS)
        self.create_subscription(Odometry,'/mrider_sim/odometry',self.odom,SENSOR_QOS)
        self.create_subscription(Twist,'/mrider_sim/candidate_cmd_vel',self.cmd,COMMAND_QOS)
        self.create_subscription(String,'/mrider_sim/mission_state',self.mission,COMMAND_QOS)
        self.pub=self.create_publisher(AckermannDriveStamped,'/mrider_sim/command',COMMAND_QOS)
        self.state_pub=self.create_publisher(String,'/mrider_sim/safety_state',COMMAND_QOS)
        self.create_service(Trigger,'/mrider_sim/arm',self.arm)
        self.create_service(Trigger,'/mrider_sim/disarm',self.disarm)
        self.create_service(Trigger,'/mrider_sim/auto_arm',self.auto_arm)
        self.clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.02,self.tick,clock=self.clock)

    def scan(self,msg):
        try:self.gate.scan_input(msg.ranges,msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max,msg.header.frame_id,stamp_seconds(msg.header.stamp),seconds(self),time.monotonic())
        except (ValueError,TypeError,OverflowError):self.gate.scan=None;self.gate.fault('invalid_scan')

    def odom(self,msg):
        try:
            x,y,yaw=pose_values(msg.pose.pose)
            self.gate.odom_input(x,y,yaw,msg.twist.twist.linear.x,msg.twist.twist.angular.z,msg.header.frame_id,msg.child_frame_id,stamp_seconds(msg.header.stamp),seconds(self),time.monotonic())
        except (ValueError,TypeError,OverflowError):self.gate.odom=None;self.gate.fault('invalid_odometry')

    def cmd(self,msg):
        if any(v!=0. for v in (msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y)):
            self.gate.fault('nonplanar_command');return
        self.gate.command_input(msg.linear.x,msg.angular.z,seconds(self),time.monotonic())

    def mission(self,msg):
        self.gate.mission_state(msg.data)

    def arm(self,req,res):
        res.success,res.message=self.gate.arm(seconds(self),time.monotonic());return res

    def auto_arm(self,req,res):
        res.success,res.message=self.gate.auto_arm(seconds(self),time.monotonic());return res

    def disarm(self,req,res):
        self.gate.disarm();res.success=True;res.message='disarmed';return res

    def tick(self):
        v,s=self.gate.output(seconds(self),time.monotonic())
        msg=AckermannDriveStamped();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='base_link'
        msg.drive.speed=float(v);msg.drive.steering_angle=float(s);self.pub.publish(msg)
        self.state_pub.publish(String(data=json.dumps({'state':self.gate.state,'reason':self.gate.reason,'speed':v})))

    def stop(self):
        self.gate.disarm();self.tick()
