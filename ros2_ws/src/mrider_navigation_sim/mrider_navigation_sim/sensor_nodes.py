"""Sensor-only odometry and RGB rule gate. Neither subscribes to ground truth."""
from array import array
import math
import json
import time
import copy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from mrider_autonomy.nodes import SENSOR_QOS, COMMAND_QOS, seconds
from mrider_autonomy.ros_adapter import stamp_seconds
from .sensor_core import KeyframeOdometry, VelocityWindow, slam_ranges
from .segmentation import CameraGeometry, segment
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class LaserOdometry(Node):
    def __init__(self):
        super().__init__('laser_odometry')
        self.previous=None;self.stamp=None;self.pose=np.zeros(3)
        self.estimator=KeyframeOdometry()
        self.velocity=VelocityWindow()
        self.max_range_is_clear=self.declare_parameter('max_range_is_clear',False).value
        self.slam_pub=self.create_publisher(LaserScan,'/mrider_sim/slam_scan',SENSOR_QOS)
        self.pub=self.create_publisher(Odometry,'/mrider_sim/odometry',SENSOR_QOS)
        self.tf=TransformBroadcaster(self)
        self.create_subscription(LaserScan,'/scan',self.scan,SENSOR_QOS)
        self.diagnostics=self.create_publisher(String,'/mrider_sim/estimator_status',COMMAND_QOS)

    def scan(self,msg):
        try:
            stamp=stamp_seconds(msg.header.stamp)
            if msg.header.frame_id!='laser' or not -.05<=seconds(self)-stamp<=.35:raise ValueError('scan_frame_or_age')
            if self.stamp is not None and not 0<stamp-self.stamp<=.35:raise ValueError('scan_gap_manual_restart_required')
            matching_ranges=slam_ranges(msg.ranges,msg.range_min,msg.range_max,self.max_range_is_clear)
            ranges=np.asarray(msg.ranges);angles=msg.angle_min+np.arange(len(ranges))*msg.angle_increment
            mask=np.isfinite(ranges)&(ranges>msg.range_min)&(ranges<msg.range_max-.05)
            points=np.column_stack((ranges[mask]*np.cos(angles[mask]),ranges[mask]*np.sin(angles[mask])))
            if len(points)<60:raise ValueError('insufficient_scan')
            self.pose,rmse=self.estimator.update(points)
            v,w=self.velocity.update(self.pose,stamp)
            self.previous=points;self.stamp=stamp
            # ROS Python message fields share objects on assignment. Preserve
            # the source laser frame for SLAM's scan-time TF message filter.
            out=Odometry();out.header=copy.deepcopy(msg.header);out.header.frame_id='odom';out.child_frame_id='base_link'
            out.pose.pose.position.x=float(self.pose[0]);out.pose.pose.position.y=float(self.pose[1])
            out.pose.pose.orientation.z=math.sin(self.pose[2]/2);out.pose.pose.orientation.w=math.cos(self.pose[2]/2)
            out.twist.twist.linear.x=v;out.twist.twist.angular.z=w
            self.pub.publish(out)
            tf=TransformStamped();tf.header=out.header;tf.child_frame_id='base_link'
            tf.transform.translation.x=out.pose.pose.position.x;tf.transform.translation.y=out.pose.pose.position.y
            tf.transform.rotation=out.pose.pose.orientation;self.tf.sendTransform(tf)
            matching_scan=copy.deepcopy(msg);matching_scan.ranges=matching_ranges.tolist()
            self.slam_pub.publish(matching_scan)
            self.diagnostics.publish(String(data=json.dumps({'valid':True,'rmse':rmse})))
        except (ValueError,TypeError,np.linalg.LinAlgError) as error:
            # Do not integrate commands or fall back to truth on lost registration.
            self.diagnostics.publish(String(data=json.dumps({'valid':False,'reason':str(error)})))


class CameraRules(Node):
    def __init__(self):
        super().__init__('camera_floor_rules')
        import cv2
        cv2.setNumThreads(1)
        self.calibrated=self.declare_parameter('extrinsics_verified',False).value
        self.height=self.declare_parameter('camera_height',.6).value
        self.pitch=self.declare_parameter('camera_pitch',0.).value
        self.laser_height=self.declare_parameter('laser_height',.2).value
        self.max_range_is_clear=self.declare_parameter('max_range_is_clear',False).value
        self.scan_msg=None;self.info=None;self.last_stamp=0.
        self.pub=self.create_publisher(String,'/mrider_sim/camera_status',COMMAND_QOS)
        self.labels=self.create_publisher(Image,'/perception/labels',SENSOR_QOS)
        self.overlay=self.create_publisher(Image,'/perception/overlay',SENSOR_QOS)
        latest=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Image,'/camera/image_raw',self.image,latest)
        self.create_subscription(CameraInfo,'/camera/camera_info',lambda m:setattr(self,'info',m),latest)
        self.create_subscription(LaserScan,'/scan',lambda m:setattr(self,'scan_msg',m),latest)

    def image(self,msg):
        start=time.monotonic();stamp=stamp_seconds(msg.header.stamp)
        status={'stamp':stamp,'valid':False,'clear':False,'reason':'unprocessed'}
        try:
            if not self.calibrated:raise ValueError('extrinsics_unverified')
            if msg.encoding!='rgb8' or msg.header.frame_id!='camera_optical_frame':raise ValueError('image_contract')
            if not -.05<=seconds(self)-stamp<=.35:raise ValueError('image_age')
            if stamp<=self.last_stamp:raise ValueError('nonmonotonic_image')
            self.last_stamp=stamp
            info=self.info;scan=self.scan_msg
            if info is None:raise ValueError('camera_info_missing')
            if (info.header.frame_id!=msg.header.frame_id or info.width!=msg.width or info.height!=msg.height
                    or any(not math.isfinite(v) or abs(v)>1e-9 for v in info.d)):
                raise ValueError('camera_info_or_rectification')
            if scan is None:raise ValueError('scan_missing')
            scan_stamp=stamp_seconds(scan.header.stamp)
            if scan.header.frame_id!='laser' or abs(scan_stamp-stamp)>.12:raise ValueError('scan_image_unsynchronized')
            if not -.05<=seconds(self)-scan_stamp<=.35:raise ValueError('scan_age')
            if msg.step<msg.width*3:raise ValueError('image_step')
            rgb=np.frombuffer(bytes(msg.data),dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
            camera=CameraGeometry(info.k[0],info.k[4],info.k[2],info.k[5],self.height,self.pitch,laser_height=self.laser_height)
            labels,overlay,result=segment(rgb,scan.ranges,scan.angle_min,scan.angle_increment,scan.range_min,scan.range_max,camera,self.max_range_is_clear)
            status.update(result);status['valid']=True;status['scan_image_offset_s']=scan_stamp-stamp
            for data,encoding,publisher in [(labels,'mono8',self.labels),(overlay,'rgb8',self.overlay)]:
                output=Image();output.header=msg.header;output.height=msg.height;output.width=msg.width
                output.encoding=encoding;output.step=msg.width*(3 if encoding=='rgb8' else 1)
                output.data=array('B',data.tobytes());publisher.publish(output)
        except (ValueError,TypeError,IndexError) as error:
            status['reason']=str(error)
        status['processing_ms']=(time.monotonic()-start)*1000
        status['source_age_s']=seconds(self)-stamp
        self.pub.publish(String(data=json.dumps(status)))
