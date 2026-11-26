"""Reconcile historical SLAM occupancy with repeated, fresh LiDAR evidence.

The SLAM map remains unchanged. Only a separate navigation map is published.
No simulator geometry, actor positions or evaluation truth are consumed.
"""
import copy
import math
import json
import time
from array import array
from collections import deque

import numpy as np
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException
from mrider_autonomy.nodes import SENSOR_QOS, COMMAND_QOS
from mrider_autonomy.ros_adapter import pose_values
from .local_goal import scan_free_evidence


def reconcile(grid, masks):
    """Require three independent scans, without clearing beyond their returns."""
    result=np.asarray(grid).copy()
    if len(masks)<3:return result
    if any(m.shape!=result.shape for m in masks):raise ValueError('evidence_shape')
    result[np.logical_and.reduce(masks)]=0
    return result


class ObservedMap(Node):
    def __init__(self):
        super().__init__('observed_map')
        self.max_clear=self.declare_parameter('max_range_is_clear',False).value
        self.raw=None;self.scans=deque(maxlen=8);self.cache={}
        self.tf_buffer=Buffer();self.listener=TransformListener(self.tf_buffer,self)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher=self.create_publisher(OccupancyGrid,'/navigation/observed_map',qos)
        self.status=self.create_publisher(String,'/mrider_sim/map_validation',COMMAND_QOS)
        self.create_subscription(OccupancyGrid,'/map',lambda m:setattr(self,'raw',m),qos)
        self.create_subscription(LaserScan,'/scan',self.scan_cb,SENSOR_QOS)
        self.create_timer(.2,self.tick)

    def scan_cb(self,msg):
        # A repeated stamp is not independent evidence. An invalid newest
        # input invalidates the queue, rather than hiding it with older data.
        if (msg.header.frame_id!='laser' or self.scans and
                Time.from_msg(msg.header.stamp)<=Time.from_msg(self.scans[-1].header.stamp)):
            self.scans.clear();self.cache.clear();return
        self.scans.append(msg)

    def tick(self):
        started=time.monotonic()
        self.metrics={'published':False,'confirmations':0}
        self.update_map()
        self.metrics['processing_ms']=1000*(time.monotonic()-started)
        self.status.publish(String(data=json.dumps(self.metrics)))

    def update_map(self):
        m=self.raw
        if m is None or m.header.frame_id!='map':return
        now=self.get_clock().now().nanoseconds*1e-9
        age=now-Time.from_msg(m.header.stamp).nanoseconds*1e-9
        self.metrics.update(stamp=now,raw_map_age=age)
        if not -.05<=age<=2.:return
        if not self.scans:return
        latest_age=now-Time.from_msg(self.scans[-1].header.stamp).nanoseconds*1e-9
        self.metrics['latest_scan_age']=latest_age
        if not -.05<=latest_age<=.35:return
        try:
            ox,oy,yaw=pose_values(m.info.origin)
            if abs(yaw)>1e-6 or not 0<m.info.resolution<=1.:return
            grid=np.asarray(m.data).reshape(m.info.height,m.info.width)
            masks=[];keys=set()
            for scan in reversed(self.scans):
                stamp=Time.from_msg(scan.header.stamp)
                scan_age=now-stamp.nanoseconds*1e-9
                # The newest usable observation must satisfy the input
                # freshness limit. Two earlier confirmations may span 0.6 s;
                # otherwise a normal 0.1 s ICP/TF delay leaves only two of
                # the 10 Hz scans inside a 0.35 s window indefinitely.
                if not -.05<=scan_age<=.6:continue
                try:tf=self.tf_buffer.lookup_transform('map','laser',stamp)
                except TransformException:continue
                if not masks and scan_age>.35:return
                if not masks:self.metrics['newest_confirmed_age']=scan_age
                p=tf.transform.translation;q=tf.transform.rotation
                laser_pose=(p.x,p.y,2*math.atan2(q.z,q.w))
                key=(stamp.nanoseconds,ox,oy,m.info.resolution,*grid.shape,*laser_pose)
                keys.add(key)
                if key not in self.cache:
                    self.cache[key]=scan_free_evidence(grid.shape,m.info.resolution,(ox,oy),laser_pose,
                        scan.ranges,scan.angle_min,scan.angle_increment,scan.range_min,scan.range_max,self.max_clear)
                masks.append(self.cache[key])
                self.metrics['confirmations']=len(masks)
                self.metrics['oldest_confirmed_age']=scan_age
                if len(masks)==3:break
            self.cache={k:v for k,v in self.cache.items() if k in keys}
        except (ValueError,TransformException):
            # Leave raw occupancy intact until the exact scan-time TF exists.
            # Retry next timer tick; never relabel old evidence as current.
            return
        if len(masks)<3:return
        result=copy.deepcopy(m)
        # The generated ROS setter validates Python lists element-by-element.
        # An int8 array preserves the identical wire data without that loop.
        result.data=array('b',reconcile(grid,masks).astype(np.int8).ravel().tobytes())
        elapsed=self.get_clock().now().nanoseconds*1e-9-now
        self.metrics['newest_confirmed_age_at_publish']=self.metrics['newest_confirmed_age']+elapsed
        if (elapsed<0 or latest_age+elapsed>.35 or
                self.metrics['newest_confirmed_age_at_publish']>.35 or
                self.metrics['oldest_confirmed_age']+elapsed>.6 or age+elapsed>2.):return
        # Keep SLAM's source timestamp, so stale mapping cannot look healthy.
        self.publisher.publish(result)
        self.metrics['published']=True
