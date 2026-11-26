"""Bounded, read-only ROS diagnostics for simulation evaluators."""
import gzip
import json
from collections import deque
from geometry_msgs.msg import PointStamped, PolygonStamped, Twist
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from nav2_msgs.msg import Costmap
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
from mrider_autonomy.nodes import SENSOR_QOS, COMMAND_QOS


def header(msg):
    return dict(frame=msg.header.frame_id,
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)


def pose(p):
    return [p.position.x,p.position.y,p.orientation.z,p.orientation.w]


class NavigationTrace:
    def __init__(self,node,costmaps=False):
        self.data={};self.node=node;self.subs=[]
        self.subscribe(String,'/mrider_sim/goal_validation',lambda m:json.loads(m.data),100)
        self.subscribe(String,'/mrider_sim/map_validation',lambda m:json.loads(m.data),100)
        self.subscribe(String,'/mrider_sim/estimator_status',lambda m:json.loads(m.data),200)
        self.subscribe(LaserScan,'/scan',lambda m:dict(**header(m),angle_min=m.angle_min,
            angle_increment=m.angle_increment,range_min=m.range_min,range_max=m.range_max,
            ranges=list(m.ranges)),100,SENSOR_QOS)
        self.subscribe(OccupancyGrid,'/mrider_sim/goal_grid',self.grid,2)
        self.subscribe(TFMessage,'/tf',lambda m:[dict(**header(t),child=t.child_frame_id,
            transform=[t.transform.translation.x,t.transform.translation.y,
                       t.transform.rotation.z,t.transform.rotation.w]) for t in m.transforms],1000)
        self.subscribe(Odometry,'/mrider_sim/odometry',lambda m:dict(**header(m),pose=pose(m.pose.pose)),100,SENSOR_QOS)
        if costmaps:
            latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            for topic in ('/local_costmap/costmap_raw','/global_costmap/costmap_raw'):
                self.subscribe(Costmap,topic,lambda m:dict(**header(m),resolution=m.metadata.resolution,
                    width=m.metadata.size_x,height=m.metadata.size_y,origin=pose(m.metadata.origin),data=list(m.data)),6,latched)
            for topic in ('/received_global_plan','/lookahead_collision_arc','/mrider_sim/planned_path'):
                self.subscribe(Path,topic,lambda m:dict(**header(m),poses=[pose(p.pose) for p in m.poses]),60)
            self.subscribe(PointStamped,'/lookahead_point',lambda m:dict(**header(m),point=[m.point.x,m.point.y]),60)
            self.subscribe(PolygonStamped,'/local_costmap/published_footprint',lambda m:dict(
                **header(m),points=[[p.x,p.y] for p in m.polygon.points]),60)
            self.subscribe(Twist,'/mrider_sim/candidate_cmd_vel',lambda m:dict(
                stamp=node.get_clock().now().nanoseconds*1e-9,v=m.linear.x,w=m.angular.z),100)

    def subscribe(self,typ,topic,convert,count,qos=COMMAND_QOS):
        queue=deque(maxlen=count);self.data[topic]=queue
        self.subs.append(self.node.create_subscription(typ,topic,lambda m:queue.append(convert(m)),qos))

    @staticmethod
    def grid(m):
        return dict(**header(m),resolution=m.info.resolution,width=m.info.width,
                    height=m.info.height,origin=pose(m.info.origin),data=list(m.data))

    def save(self,directory,name):
        with gzip.open(directory/(name+'.trace.json.gz'),'wt') as stream:
            json.dump({k:list(v) for k,v in self.data.items()},stream)
