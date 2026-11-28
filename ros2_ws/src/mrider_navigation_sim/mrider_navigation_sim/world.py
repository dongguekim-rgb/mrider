"""Ideal 2D scan + static map + acceleration-limited bicycle; no hardware."""
from array import array
import math
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64, Bool
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from mrider_autonomy.nodes import SENSOR_QOS, COMMAND_QOS, seconds
from mrider_autonomy.ros_adapter import stamp_seconds
from .geometry import obstacles, ray_distance, ray_distances, clearance
from .safety import load_limits
from .indoor import Pedestrians, ROOMS, furniture

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


class World(Node):
    def __init__(self):
        super().__init__('mrider_sensor_world')
        self.sensor_only=self.declare_parameter('sensor_only',False).value
        self.camera_dropout=False
        import numpy as np
        self.rng=np.random.default_rng(17)
        self.scenario=self.declare_parameter('scenario','static_obstacle').value
        if self.scenario=='large_office' and not self.sensor_only:
            raise ValueError('large_office requires sensor_only: no truth map publication')
        self.boxes=obstacles(self.scenario);self.dynamic=None;self.c=load_limits(self)
        self.pose=(0.,0.,0.);self.speed=0.;self.steer=0.;self.command=(0.,0.)
        self.last_command=-math.inf;self.last_stamp=None;self.previous=seconds(self);self.counter=0
        self.collided=False
        self.indoor=self.scenario in ('indoor','indoor_dynamic','indoor_turns','sensor_turns')
        self.pedestrians=Pedestrians() if self.scenario=='indoor_dynamic' else None
        if self.scenario=='indoor_turns':self.pedestrians=Pedestrians(trigger_x=8.5,crossing_x=12.,room_x=20.)
        if self.scenario=='sensor_turns':self.pedestrians=Pedestrians(trigger_x=3.,crossing_x=6.2,room_x=10.,hold_y=.8,hold_duration=8.)
        if self.scenario=='large_office':
            from .large_office import OfficeActors
            self.pedestrians=OfficeActors(self.declare_parameter('actor_time_offset_s',0.).value)
        self.actor_boxes=[]
        self.faults={k:False for k in ['scan_dropout','odom_dropout','scan_bad_frame','scan_future','scan_nan','odom_bad_frame','dynamic_obstacle']}
        for key in self.faults:
            self.create_service(SetBool,'/mrider_sim/fault/'+key,lambda req,res,k=key:self.set_fault(k,req,res))
        self.scan_pub=self.create_publisher(LaserScan,'/scan',SENSOR_QOS)
        self.odom_pub=self.create_publisher(Odometry,'/evaluation/ground_truth' if self.sensor_only else '/mrider_sim/odometry',SENSOR_QOS)
        self.image_pub=self.create_publisher(Image,'/camera/image_raw',SENSOR_QOS)
        self.info_pub=self.create_publisher(CameraInfo,'/camera/camera_info',SENSOR_QOS)
        self.floor_rgb=self.declare_parameter('floor_rgb',[110,110,110]).value
        self.wall_rgb=self.declare_parameter('wall_rgb',[65,100,155]).value
        self.image_gain=self.declare_parameter('image_gain',1.).value
        self.create_service(SetBool,'/mrider_sim/fault/camera_dropout',self.camera_fault)
        self.map_pub=None if self.sensor_only else self.create_publisher(OccupancyGrid,'/map',LATCHED)
        self.marker_pub=self.create_publisher(MarkerArray,'/mrider_sim/world_markers',COMMAND_QOS)
        self.clear_pub=self.create_publisher(Float64,'/mrider_sim/clearance',COMMAND_QOS)
        self.collision_pub=self.create_publisher(Bool,'/mrider_sim/collision',COMMAND_QOS)
        self.create_subscription(AckermannDriveStamped,'/mrider_sim/command',self.command_cb,COMMAND_QOS)
        self.tf=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
        transforms=[]
        for parent,child in ([('base_link','laser'),('base_link','camera_optical_frame')] if self.sensor_only else [('map','odom'),('base_link','laser')]):
            t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.transform.rotation.w=1.
            if self.sensor_only and child=='laser':t.transform.translation.z=.2
            if self.sensor_only and child=='camera_optical_frame':
                t.transform.translation.z=.6
                t.transform.rotation.x=-.5;t.transform.rotation.y=.5;t.transform.rotation.z=-.5;t.transform.rotation.w=.5
            t.header.stamp=self.get_clock().now().to_msg();transforms.append(t)
        self.static.sendTransform(transforms)
        if not self.sensor_only:self.publish_map()
        self.create_timer(.02,self.tick)

    def camera_fault(self,req,res):
        self.camera_dropout=req.data;res.success=True;return res

    def publish_camera(self,now,boxes):
        from .synthetic_camera import render_camera
        h,w,f=240,320,180.
        rgb=render_camera(self.pose,boxes,self.floor_rgb,self.wall_rgb,self.image_gain)
        info=CameraInfo();info.header.stamp=now.to_msg();info.header.frame_id='camera_optical_frame'
        info.width=w;info.height=h;info.distortion_model='plumb_bob';info.d=[0.]*5
        info.k=[f,0.,w/2,0.,f,h/2,0.,0.,1.]
        info.p=[f,0.,w/2,0.,0.,f,h/2,0.,0.,0.,1.,0.]
        self.info_pub.publish(info)
        msg=Image();msg.header.stamp=now.to_msg();msg.header.frame_id='camera_optical_frame'
        msg.height=h;msg.width=w;msg.encoding='rgb8';msg.step=w*3;msg.data=array('B',rgb.tobytes())
        self.image_pub.publish(msg)

    def set_fault(self,key,req,res):
        self.faults[key]=req.data
        if key=='dynamic_obstacle':
            x,y,yaw=self.pose;x+=1.5*math.cos(yaw);y+=1.5*math.sin(yaw)
            self.dynamic=(x-.2,y-.6,x+.2,y+.6) if req.data else None
        res.success=True;res.message=key+'='+str(req.data);return res

    def command_cb(self,msg):
        v,s=msg.drive.speed,msg.drive.steering_angle
        stamp=stamp_seconds(msg.header.stamp);age=seconds(self)-stamp
        if (math.isfinite(v) and math.isfinite(s) and 0<=v<=self.c.max_speed+1e-4 and
            abs(s)<=self.c.max_steering+1e-3 and msg.header.frame_id=='base_link' and
            stamp>0 and -.05<=age<=.35 and (self.last_stamp is None or stamp>self.last_stamp)):
            self.command=(v,s);self.last_command=time.monotonic();self.last_stamp=stamp
        else:self.command=(0.,0.)

    def publish_map(self):
        msg=OccupancyGrid();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg()
        msg.info.resolution=.1;msg.info.width=260 if self.scenario=='indoor_turns' else 140;msg.info.height=100
        msg.info.origin.position.x=-2.;msg.info.origin.position.y=-5.;msg.info.origin.orientation.w=1.
        msg.data=[100 if any(a<=-2+(i+.5)*.1<=c and b<=-5+(j+.5)*.1<=d for a,b,c,d in self.boxes) else 0
                  for j in range(msg.info.height) for i in range(msg.info.width)]
        self.map_pub.publish(msg)

    def tick(self):
        now=self.get_clock().now();t=now.nanoseconds*1e-9;dt=max(0.,min(t-self.previous,.05));self.previous=t
        if time.monotonic()-self.last_command>.35:self.command=(0.,0.)
        v,s=self.command
        change=self.c.braking_deceleration*dt
        self.speed+=max(-change,min(change,v-self.speed))
        self.steer+=max(-.7*dt,min(.7*dt,s-self.steer))
        x,y,yaw=self.pose;w=self.speed*math.tan(self.steer)/self.c.wheelbase
        self.pose=(x+self.speed*math.cos(yaw)*dt,y+self.speed*math.sin(yaw)*dt,yaw+w*dt)
        x,y,yaw=self.pose
        if self.pedestrians:self.actor_boxes=self.pedestrians.step(dt,x)
        boxes=self.boxes+self.actor_boxes+([self.dynamic] if self.dynamic else [])
        gap=clearance(self.pose,boxes,self.c.front,self.c.rear,self.c.half_width);self.collided=self.collided or gap<=0.
        self.clear_pub.publish(Float64(data=gap));self.collision_pub.publish(Bool(data=self.collided))
        if not self.faults['odom_dropout']:
            odom=Odometry();odom.header.frame_id='odom';odom.header.stamp=now.to_msg()
            odom.child_frame_id='wrong' if self.faults['odom_bad_frame'] else 'base_link'
            odom.pose.pose.position.x=x;odom.pose.pose.position.y=y
            odom.pose.pose.orientation.z=math.sin(yaw/2);odom.pose.pose.orientation.w=math.cos(yaw/2)
            odom.twist.twist.linear.x=self.speed;odom.twist.twist.angular.z=w
            self.odom_pub.publish(odom)
            tf=TransformStamped();tf.header=odom.header;tf.child_frame_id='base_link'
            tf.transform.translation.x=x;tf.transform.translation.y=y;tf.transform.rotation=odom.pose.pose.orientation
            if not self.sensor_only:self.tf.sendTransform(tf)
        self.counter+=1
        if self.counter%5==0:
            if not self.faults['scan_dropout']:
                scan=LaserScan();scan.header.stamp=now.to_msg();scan.header.frame_id='wrong' if self.faults['scan_bad_frame'] else 'laser'
                if self.faults['scan_future']:scan.header.stamp.sec+=2
                scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/360;scan.angle_max=math.pi-scan.angle_increment
                scan.range_min=.05;scan.range_max=15.;scan.scan_time=.1;scan.time_increment=0.
                scan.ranges=[max(.05,float(r)) for r in ray_distances(x,y,[yaw+scan.angle_min+i*scan.angle_increment for i in range(360)],boxes)]
                if self.sensor_only:
                    scan.ranges=[min(15.,max(.05,r+float(self.rng.normal(0,.002)))) if r<15. else 15. for r in scan.ranges]
                if self.faults['scan_nan']:scan.ranges[180]=math.nan
                self.scan_pub.publish(scan)
            if self.sensor_only and not self.camera_dropout:self.publish_camera(now,boxes)
            markers=[]
            for i,(a,b,c,d) in enumerate(boxes):
                m=Marker();m.header.frame_id='map';m.header.stamp=now.to_msg();m.ns='obstacles';m.id=i
                m.type=Marker.CUBE;m.action=Marker.ADD;m.pose.orientation.w=1.
                m.pose.position.x=(a+c)/2;m.pose.position.y=(b+d)/2;m.pose.position.z=.3
                m.scale.x=c-a;m.scale.y=d-b;m.scale.z=.6;m.color.r=.8;m.color.a=1.
                if self.indoor:
                    if i>=len(self.boxes):
                        m.color.r=.95;m.color.g=.5;m.scale.z=1.7;m.pose.position.z=.85
                    elif (a,b,c,d) in furniture():
                        m.color.r=.55;m.color.g=.32;m.color.b=.12;m.scale.z=.8;m.pose.position.z=.4
                    else:
                        m.color.r=.55;m.color.g=.6;m.color.b=.68;m.scale.z=1.2;m.pose.position.z=.6
                markers.append(m)
            if self.indoor:
                for i,(lx,ly,label) in enumerate(ROOMS):
                    m=Marker();m.header.frame_id='map';m.header.stamp=now.to_msg();m.ns='rooms';m.id=i
                    m.type=Marker.TEXT_VIEW_FACING;m.action=Marker.ADD;m.pose.orientation.w=1.
                    m.pose.position.x=2*lx if self.scenario=='indoor_turns' else lx;m.pose.position.y=ly;m.pose.position.z=1.6
                    m.scale.z=.3;m.color.r=.1;m.color.g=.1;m.color.b=.1;m.color.a=1.;m.text=label;markers.append(m)
            if not self.dynamic:
                m=Marker();m.ns='obstacles';m.id=len(self.boxes)+len(self.actor_boxes);m.action=Marker.DELETE;markers.append(m)
            self.marker_pub.publish(MarkerArray(markers=markers))
