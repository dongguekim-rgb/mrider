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
from .camera_contract import camera_fault
from .moving_obstacles import collision_time,motion_fault


class SafetyNode(Node):
    def __init__(self):
        super().__init__('mrider_safety_gate');self.gate=Gate(load_limits(self))
        self.require_camera=self.declare_parameter('require_camera',False).value
        self.camera=None
        self.require_motion=self.declare_parameter('require_motion_prediction',False).value
        self.motion=None;self.last_motion_stamp=None;self.last_moving_command=(self.gate.c.max_speed,0.)
        self.predicted_collision=None
        self.create_subscription(String,'/mrider_sim/moving_obstacles',self.motion_cb,COMMAND_QOS)
        self.create_subscription(String,'/mrider_sim/camera_status',self.camera_cb,COMMAND_QOS)
        self.create_subscription(LaserScan,'/scan',self.scan,SENSOR_QOS)
        self.create_subscription(Odometry,'/mrider_sim/odometry',self.odom,SENSOR_QOS)
        self.create_subscription(Twist,'/mrider_sim/candidate_cmd_vel',self.cmd,COMMAND_QOS)
        self.create_subscription(String,'/mrider_sim/mission_state',self.mission,COMMAND_QOS)
        self.pub=self.create_publisher(AckermannDriveStamped,'/mrider_sim/command',COMMAND_QOS)
        self.state_pub=self.create_publisher(String,'/mrider_sim/safety_state',COMMAND_QOS)
        self.create_service(Trigger,'/mrider_sim/arm',self.arm)
        self.create_service(Trigger,'/mrider_sim/continue',self.continue_route)
        self.create_service(Trigger,'/mrider_sim/disarm',self.disarm)
        self.create_service(Trigger,'/mrider_sim/auto_arm',self.auto_arm)
        self.clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.02,self.tick,clock=self.clock)

    def camera_cb(self,msg):
        try:
            data=json.loads(msg.data)
            self.camera=(data,time.monotonic())
        except (ValueError,TypeError):self.camera=None

    def motion_cb(self,msg):
        try:
            data=json.loads(msg.data);sample=(data,time.monotonic())
            stamp=data.get('stamp')
            if motion_fault(sample,seconds(self),sample[1]) not in (None,'motion_prediction_unready'):
                raise ValueError('motion_contract')
            if self.last_motion_stamp is not None and stamp<=self.last_motion_stamp:raise ValueError('motion_stamp')
            self.motion=sample;self.last_motion_stamp=stamp
        except (ValueError,TypeError,AttributeError):self.motion=None

    def motion_reason(self):
        return motion_fault(self.motion,seconds(self),time.monotonic()) if self.require_motion else None

    def motion_collision(self,speed,steering):
        if not self.require_motion or self.motion_reason() is not None or self.gate.odom is None:return None
        return collision_time(self.gate.odom[0][:3],speed,steering,self.gate.c,
            self.motion[0]['tracks'],seconds(self)-self.motion[0]['stamp'])

    def camera_reason(self):
        return camera_fault(self.camera,seconds(self),time.monotonic()) if self.require_camera else None

    def camera_ready(self):
        return self.camera_reason() is None

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
        if self.gate.command_input(msg.linear.x,msg.angular.z,seconds(self),time.monotonic()) and msg.linear.x>0:
            self.last_moving_command=self.gate.cmd[0]

    def mission(self,msg):
        self.gate.mission_state(msg.data)

    def arm(self,req,res):
        if not self.camera_ready():res.success=False;res.message='camera_not_ready';return res
        if self.motion_reason():res.success=False;res.message='motion_prediction_not_ready';return res
        res.success,res.message=self.gate.arm(seconds(self),time.monotonic());return res

    def continue_route(self,req,res):
        # Route permission may continue only from a clean completed waypoint.
        # Never use the manual arm service to clear an intervening fault.
        if not self.camera_ready():res.success=False;res.message='camera_not_ready';return res
        if self.motion_reason():res.success=False;res.message='motion_prediction_not_ready';return res
        res.success,res.message=self.gate.continue_route(seconds(self),time.monotonic());return res

    def auto_arm(self,req,res):
        if not self.camera_ready() or self.motion_reason() or self.motion_collision(*self.last_moving_command) is not None:
            self.gate.clear_since=None
            res.success=False;res.message='camera_or_motion_not_clear';return res
        res.success,res.message=self.gate.auto_arm(seconds(self),time.monotonic());return res

    def disarm(self,req,res):
        self.gate.disarm();res.success=True;res.message='disarmed';return res

    def tick(self):
        reason=self.camera_reason()
        if reason:self.gate.fault(reason)
        reason=self.motion_reason()
        if reason:self.gate.fault(reason)
        self.predicted_collision=None
        if self.gate.armed and self.gate.cmd is not None:
            v,s=self.gate.cmd[0]
            self.predicted_collision=self.motion_collision(v,s)
            if self.predicted_collision is not None:self.gate.fault('obstacle_stop')
        v,s=self.gate.output(seconds(self),time.monotonic())
        msg=AckermannDriveStamped();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='base_link'
        msg.drive.speed=float(v);msg.drive.steering_angle=float(s);self.pub.publish(msg)
        self.state_pub.publish(String(data=json.dumps({'state':self.gate.state,'reason':self.gate.reason,'speed':v,
            'predicted_collision_s':self.predicted_collision,
            'camera':self.camera[0] if self.camera and isinstance(self.camera[0],dict) else None,
            'camera_receive_age_s':time.monotonic()-self.camera[1] if self.camera else None})))

    def stop(self):
        self.gate.disarm();self.tick()
