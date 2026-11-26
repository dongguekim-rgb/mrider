"""Explicit one-goal mission: Smac plan -> RPP follow; no reverse/recovery."""
import json
import math
import copy
import time
from collections import deque
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Path, OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy
import numpy as np
from .local_goal import select_pose_goal,add_scan_evidence
from .path_feasibility import validate_path
from sensor_msgs.msg import LaserScan
from mrider_autonomy.nodes import SENSOR_QOS
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from lifecycle_msgs.srv import GetState
from mrider_autonomy.nodes import COMMAND_QOS
from mrider_autonomy.ros_adapter import pose_values


class Mission(Node):
    def __init__(self):
        super().__init__('mrider_mission')
        self.x=self.declare_parameter('goal_x',8.).value;self.y=self.declare_parameter('goal_y',0.).value
        self.wheelbase=self.declare_parameter('wheelbase',.65).value
        self.max_steering=self.declare_parameter('max_steering',.3927).value
        self.path_lookahead=self.declare_parameter('path_lookahead',1.5).value
        self.planning_radius=self.declare_parameter('planning_turning_radius',3.).value
        self.map_resolution=self.declare_parameter('expected_map_resolution',0.).value
        self.path_status=self.create_publisher(String,'/mrider_sim/path_validation',COMMAND_QOS)
        self.lifecycle_clients={name:self.create_client(GetState,'/'+name+'/get_state') for name in ('planner_server','controller_server')}
        self.active={name:False for name in self.lifecycle_clients};self.pending={}
        self.create_timer(.2,self.poll_lifecycle)
        self.auto_resume=self.declare_parameter('auto_resume',False).value
        self.staged=self.declare_parameter('staged_goals',False).value
        self.local_scan=None
        self.scans=deque(maxlen=5)
        self.scan_max_clear=self.declare_parameter('max_range_is_clear',False).value
        self.goal_radius=self.declare_parameter('goal_clearance_radius',1.1).value
        self.create_subscription(LaserScan,'/scan',self.scan_cb,SENSOR_QOS)
        self.destination=None;self.local_map=None;self.stage_final=True;self.stage_pending=None;self.stage_permission=False;self.stage_wait_started=None
        self.stage_client=self.create_client(Trigger,'/mrider_sim/continue')
        self.stage_pub=self.create_publisher(PoseStamped,'/mrider_sim/local_goal',COMMAND_QOS)
        self.create_subscription(OccupancyGrid,'/map',lambda m:setattr(self,'local_map',m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.2,self.continue_tick)
        self.saved_goal=None;self.resume_pending=None;self.resume_ready=False
        self.auto_client=self.create_client(Trigger,'/mrider_sim/auto_arm')
        self.create_timer(.2,self.resume_tick)
        self.goal_extension=self.declare_parameter('goal_extension',0.).value
        self.tf_buffer=Buffer();self.tf_listener=TransformListener(self.tf_buffer,self)
        self.create_timer(.05,self.check_original_goal)
        self.pending_plan=None
        self.create_timer(.05,self.poll_plan)
        self.state='IDLE';self.safety='DISARMED';self.follow_handle=None;self.sequence=0
        self.planner=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
        self.follower=ActionClient(self,FollowPath,'/follow_path')
        self.pub=self.create_publisher(String,'/mrider_sim/mission_state',COMMAND_QOS)
        self.path_pub=self.create_publisher(Path,'/mrider_sim/planned_path',COMMAND_QOS)
        self.create_subscription(String,'/mrider_sim/safety_state',self.safety_cb,COMMAND_QOS)
        self.create_subscription(PoseStamped,'/goal_pose',self.goal_cb,COMMAND_QOS)
        self.create_service(Trigger,'/mrider_sim/start',self.start)
        self.create_timer(.1,lambda:self.pub.publish(String(data=self.state)))

    def check_original_goal(self):
        # A virtual terminal reference prevents a vanishing lookahead. Stop at
        # the ORIGINAL map goal; never use truth, or drive to the extended end.
        if self.goal_extension<=0 or self.state!='FOLLOWING' or self.saved_goal is None:return
        try:
            tf=self.tf_buffer.lookup_transform('map','base_link',Time())
            age=self.get_clock().now().nanoseconds*1e-9-(tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9)
            if not -.05<=age<=.35:raise ValueError('goal_transform_stale')
            q=tf.transform.rotation;yaw=2*math.atan2(q.z,q.w)
            goal=self.saved_goal.pose;_,_,heading=pose_values(goal)
            distance=math.hypot(tf.transform.translation.x-goal.position.x,tf.transform.translation.y-goal.position.y)
            error=abs(math.atan2(math.sin(yaw-heading),math.cos(yaw-heading)))
            along=(tf.transform.translation.x-goal.position.x)*math.cos(heading)+(tf.transform.translation.y-goal.position.y)*math.sin(heading)
            if along>.05 and not (distance<=.2 and error<=.25):raise ValueError('original_goal_missed')
            if distance<=.2 and error<=.25:
                self.complete_leg()
        except (TransformException,ValueError) as error:
            self.get_logger().error('Original-goal monitor stopped motion: '+str(error))
            self.sequence+=1;self.state='FAILED'
            if self.follow_handle:self.follow_handle.cancel_goal_async()
            self.pub.publish(String(data=self.state))

    def complete_leg(self):
        self.sequence+=1
        self.state='WAYPOINT' if self.staged and not self.stage_final else 'GOAL'
        self.stage_wait_started=None
        if self.follow_handle:self.follow_handle.cancel_goal_async()
        self.pub.publish(String(data=self.state))

    def continue_tick(self):
        if not self.staged or self.state!='WAYPOINT' or self.destination is None:return
        if self.stage_pending is not None:
            if not self.stage_pending.done():return
            self.stage_permission=self.stage_pending.result().success;self.stage_pending=None
        if self.stage_permission and self.safety=='READY' and self.follow_handle is None:
            if self.stage_wait_started is None:self.stage_wait_started=time.monotonic()
            ok,reason=self.begin(self.destination,continuation=True)
            if ok:self.stage_permission=False
            elif time.monotonic()-self.stage_wait_started>5.:
                self.stage_permission=False;self.state='FAILED';self.get_logger().error(reason)
        elif self.safety=='GOAL' and self.follow_handle is None and self.stage_client.service_is_ready():
            self.stage_pending=self.stage_client.call_async(Trigger.Request())

    def scan_cb(self,msg):
        self.local_scan=msg;self.scans.append(msg)

    def scan_with_transform(self,now):
        scan=self.local_scan
        if scan is None or scan.header.frame_id!='laser':raise ValueError('scan_not_ready')
        age=now-(scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9)
        if not -.05<=age<=.35:raise ValueError('scan_stale')
        # ICP publishes the transform after receiving the scan. Select the
        # newest fresh scan whose exact-time transform is already available,
        # never a latest-time transform for a different observation instant.
        for scan in reversed(self.scans):
            age=now-(scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9)
            if scan.header.frame_id!='laser' or not -.05<=age<=.35:continue
            try:
                return scan,self.tf_buffer.lookup_transform('map','laser',Time.from_msg(scan.header.stamp))
            except TransformException:continue
        raise ValueError('scan_transform_not_ready')

    def bounded_goal(self,goal):
        m=self.local_map
        if m is None or m.header.frame_id!='map':raise ValueError('map_not_ready')
        if self.map_resolution and not math.isclose(m.info.resolution,self.map_resolution,abs_tol=1e-8):
            raise ValueError('map_resolution_changed')
        now=self.get_clock().now().nanoseconds*1e-9
        age=now-(m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
        if not -.05<=age<=2.:raise ValueError('map_stale')
        ox,oy,angle=pose_values(m.info.origin)
        if abs(angle)>1e-6:raise ValueError('rotated_map_not_supported')
        tf=self.tf_buffer.lookup_transform('map','base_link',Time())
        age=now-(tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9)
        if not -.05<=age<=.35:raise ValueError('goal_transform_stale')
        current=(tf.transform.translation.x,tf.transform.translation.y,
                 2*math.atan2(tf.transform.rotation.z,tf.transform.rotation.w))
        destination=pose_values(goal.pose)
        scan,laser_tf=self.scan_with_transform(now)
        q=laser_tf.transform.rotation
        laser_pose=(laser_tf.transform.translation.x,laser_tf.transform.translation.y,2*math.atan2(q.z,q.w))
        grid=add_scan_evidence(np.asarray(m.data).reshape(m.info.height,m.info.width),m.info.resolution,(ox,oy),laser_pose,scan.ranges,scan.angle_min,scan.angle_increment,scan.range_min,scan.range_max,self.scan_max_clear)
        chosen=select_pose_goal(grid,m.info.resolution,(ox,oy),current,destination,
                               self.planning_radius,radius=self.goal_radius)
        if chosen is None:raise ValueError('no_observed_progress_goal')
        x,y,yaw,self.stage_final=chosen
        local=copy.deepcopy(goal);local.header.stamp=self.get_clock().now().to_msg()
        local.pose.position.x=x;local.pose.position.y=y
        if not self.stage_final:
            local.pose.orientation.z=math.sin(yaw/2);local.pose.orientation.w=math.cos(yaw/2)
        return local

    def resume_tick(self):
        if not self.auto_resume or self.saved_goal is None:return
        if self.safety=='DISARMED':self.resume_ready=False;return
        if self.resume_pending is not None:
            if not self.resume_pending.done():return
            self.resume_ready=self.resume_pending.result().success
            self.resume_pending=None
        if self.resume_ready and self.safety=='READY' and self.follow_handle is None:
            self.resume_ready=False
            self.begin(self.destination if self.staged else self.saved_goal,continuation=True)
        elif self.state in ('STOPPED','FAILED') and self.safety=='STOPPED' and self.auto_client.service_is_ready():
            self.resume_pending=self.auto_client.call_async(Trigger.Request())

    def poll_lifecycle(self):
        for name,client in self.lifecycle_clients.items():
            if name in self.pending:
                future=self.pending[name]
                if future.done():
                    self.active[name]=future.result().current_state.id==3
                    del self.pending[name]
            elif client.service_is_ready():self.pending[name]=client.call_async(GetState.Request())

    def safety_cb(self,msg):
        self.safety=json.loads(msg.data)['state']
        if self.safety=='DISARMED':
            self.saved_goal=None;self.destination=None;self.stage_permission=False;self.resume_ready=False
        if self.safety in ('STOPPED','DISARMED') and self.state in ('PLANNING','FOLLOWING'):
            self.sequence+=1;self.state='STOPPED'
            if self.follow_handle:self.follow_handle.cancel_goal_async()

    def start(self,req,res):
        goal=PoseStamped();goal.header.frame_id='map';goal.header.stamp=self.get_clock().now().to_msg()
        goal.pose.position.x=self.x;goal.pose.position.y=self.y;goal.pose.orientation.w=1.
        res.success,res.message=self.begin(goal);return res

    def goal_cb(self,msg):
        ok,reason=self.begin(msg)
        if not ok:self.get_logger().warn(reason)

    def begin(self,goal,continuation=False):
        if self.state in ('PLANNING','FOLLOWING') or (self.state=='WAYPOINT' and not continuation) or self.follow_handle is not None:return False,'mission_busy'
        if self.safety!='READY':return False,'manual_arm_required'
        try:
            pose_values(goal.pose)
            if goal.header.frame_id!='map':raise ValueError('goal_frame')
        except ValueError:return False,'invalid_goal'
        if not all(self.active.values()) or not self.planner.server_is_ready() or not self.follower.server_is_ready():return False,'nav2_not_ready'
        if not continuation:self.destination=copy.deepcopy(goal)
        if self.staged:
            try:goal=self.bounded_goal(goal)
            except (ValueError,TransformException) as error:return False,str(error)
        self.saved_goal=goal
        self.stage_pub.publish(goal)
        self.sequence+=1;seq=self.sequence;self.state='PLANNING'
        self.pub.publish(String(data=self.state))
        req=ComputePathToPose.Goal();req.goal=goal;req.planner_id='GridBased';req.use_start=False
        self.planner.send_goal_async(req).add_done_callback(lambda f:self.plan_accepted(f,seq))
        return True,'planning'

    def plan_accepted(self,future,seq):
        handle=future.result()
        if not handle.accepted:
            if seq==self.sequence:self.state='FAILED'
            return
        if seq!=self.sequence:handle.cancel_goal_async();return
        handle.get_result_async().add_done_callback(lambda f:self.planned(f,seq))

    def planned(self,future,seq):
        if seq!=self.sequence:return
        self.pending_plan=(future,seq,time.monotonic())
        self.poll_plan()

    def poll_plan(self):
        if self.pending_plan is None:return
        future,seq,received=self.pending_plan
        if seq!=self.sequence:
            self.pending_plan=None;return
        # A planner reply can precede queued TF callbacks after goal selection.
        # Yield the executor, retaining PLANNING/zero output. Never extend the
        # 0.35 s freshness limit or wait indefinitely for missing localization.
        try:
            tf=self.tf_buffer.lookup_transform('map','base_link',Time())
            age=self.get_clock().now().nanoseconds*1e-9-(tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9)
            ready=-.05<=age<=.35
        except TransformException:ready=False
        if not ready and time.monotonic()-received<1.:return
        self.pending_plan=None
        self.execute_plan(future,seq)

    def execute_plan(self,future,seq):
        if seq!=self.sequence:return
        result=future.result()
        if result.status!=GoalStatus.STATUS_SUCCEEDED or len(result.result.path.poses)<2:
            self.state='FAILED';return
        reference=copy.deepcopy(result.result.path)
        if self.goal_extension>0:
            end=reference.poses[-1];_,_,yaw=pose_values(end.pose)
            for i in range(1,9):
                extra=copy.deepcopy(end);distance=self.goal_extension*i/8
                extra.pose.position.x+=distance*math.cos(yaw)
                extra.pose.position.y+=distance*math.sin(yaw)
                reference.poses.append(extra)
        try:
            if reference.header.frame_id!='map':raise ValueError('path_frame')
            tf=self.tf_buffer.lookup_transform('map','base_link',Time())
            age=self.get_clock().now().nanoseconds*1e-9-(tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9)
            if not -.05<=age<=.35:raise ValueError('path_start_transform_stale')
            if self.map_resolution and (self.local_map is None or not math.isclose(
                    self.local_map.info.resolution,self.map_resolution,abs_tol=1e-8)):
                raise ValueError('map_resolution_changed')
            current=(tf.transform.translation.x,tf.transform.translation.y,
                     2*math.atan2(tf.transform.rotation.z,tf.transform.rotation.w))
            metrics=validate_path([pose_values(p.pose) for p in reference.poses],current,
                                  self.wheelbase,self.max_steering,self.path_lookahead)
        except (ValueError,TransformException) as error:
            self.state='FAILED'
            self.path_status.publish(String(data=json.dumps({'accepted':False,'reason':str(error)})))
            self.get_logger().error('Path rejected before FollowPath: '+str(error))
            self.pub.publish(String(data=self.state));return
        self.path_status.publish(String(data=json.dumps({'accepted':True,**metrics})))
        self.path_pub.publish(result.result.path)
        req=FollowPath.Goal();req.path=reference;req.controller_id='FollowPath';req.goal_checker_id='goal_checker'
        self.state='FOLLOWING'
        self.follower.send_goal_async(req).add_done_callback(lambda f:self.follow_accepted(f,seq))

    def follow_accepted(self,future,seq):
        handle=future.result()
        if not handle.accepted:
            if seq==self.sequence:self.state='FAILED'
            return
        self.follow_handle=handle
        if seq!=self.sequence:handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(lambda f:self.finished(f,seq))

    def finished(self,future,seq):
        self.follow_handle=None
        if seq==self.sequence:
            if self.goal_extension<=0 and future.result().status==GoalStatus.STATUS_SUCCEEDED:self.complete_leg()
            else:self.state='FAILED'
