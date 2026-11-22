"""RViz markers and optional Gazebo Classic pose mirror, not vehicle physics."""
import copy
from collections import deque
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Point
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from .nodes import SENSOR_QOS, COMMAND_QOS, config, main


class Visualization(Node):
    def __init__(self):
        super().__init__('mrider_visualization')
        self.c = config(self)
        self.trail = deque(maxlen=3000)
        self.publish_tf = self.declare_parameter("publish_tf", True).value
        self.tf = TransformBroadcaster(self)
        self.pub = self.create_publisher(MarkerArray, '/mrider_sim/markers', COMMAND_QOS)
        self.create_subscription(Odometry, '/mrider_sim/odometry', self.odometry, SENSOR_QOS)
        self.gazebo = self.declare_parameter('gazebo', False).value
        self.spawned = False
        self.spawn_future = None
        self.move_future = None
        self.pose = None
        if self.gazebo:
            from gazebo_msgs.srv import SpawnEntity, SetEntityState
            self.spawn = self.create_client(SpawnEntity, '/spawn_entity')
            self.move = self.create_client(SetEntityState, '/gazebo/set_entity_state')
            self.create_timer(.05, self.mirror)

    def mirror(self):
        from gazebo_msgs.srv import SpawnEntity, SetEntityState
        if self.spawn_future is not None:
            if not self.spawn_future.done():
                return
            result = self.spawn_future.result()
            self.spawn_future = None
            if not result.success:
                self.get_logger().error(result.status_message)
                self.gazebo = False
                return
            self.spawned = True
        if not self.gazebo or self.pose is None:
            return
        if not self.spawned:
            if not self.spawn.service_is_ready():
                return
            req = SpawnEntity.Request()
            req.name = 'mrider_visual'
            req.initial_pose = self.pose
            req.xml = ('<sdf version="1.6"><model name="mrider_visual"><static>true</static>'
                       '<link name="body"><visual name="body"><pose>0.325 0 0.24 0 0 0</pose>'
                       '<geometry><box><size>0.9 0.42 0.2</size></box></geometry>'
                       '<material><ambient>0.1 0.55 0.9 1</ambient></material></visual></link></model></sdf>')
            self.spawn_future = self.spawn.call_async(req)
        elif self.move.service_is_ready() and (self.move_future is None or self.move_future.done()):
            req = SetEntityState.Request()
            req.state.name, req.state.reference_frame = 'mrider_visual', 'world'
            req.state.pose = self.pose
            self.move_future = self.move.call_async(req)

    def odometry(self, msg):
        if msg.header.frame_id != self.c.frame or msg.child_frame_id != self.c.rear_frame:
            return
        self.pose = copy.deepcopy(msg.pose.pose)
        transform = TransformStamped()
        transform.header, transform.child_frame_id = msg.header, self.c.rear_frame
        transform.transform.translation.x = self.pose.position.x
        transform.transform.translation.y = self.pose.position.y
        transform.transform.rotation = self.pose.orientation
        if self.publish_tf:
            self.tf.sendTransform(transform)
        point = copy.deepcopy(self.pose.position)
        point.z = .035
        if not self.trail or (point.x-self.trail[-1].x)**2+(point.y-self.trail[-1].y)**2 > .0004:
            self.trail.append(point)
        body = Marker()
        body.header.stamp, body.header.frame_id = msg.header.stamp, self.c.rear_frame
        body.ns, body.id, body.type, body.action = 'vehicle', 0, Marker.CUBE, Marker.ADD
        body.pose.orientation.w = 1.
        body.pose.position.x, body.pose.position.z = self.c.wheelbase/2, .24
        body.scale.x, body.scale.y, body.scale.z = self.c.wheelbase+.25, .42, .2
        body.color.r, body.color.g, body.color.b, body.color.a = .1, .55, .9, 1.
        trail = Marker()
        trail.header = msg.header
        trail.ns, trail.id, trail.type, trail.action = 'trail', 0, Marker.LINE_STRIP, Marker.ADD
        trail.pose.orientation.w = 1.
        trail.scale.x = .035
        trail.color.r, trail.color.g, trail.color.b, trail.color.a = .9, .25, .15, 1.
        trail.points = list(self.trail)
        self.pub.publish(MarkerArray(markers=[body, trail]))
