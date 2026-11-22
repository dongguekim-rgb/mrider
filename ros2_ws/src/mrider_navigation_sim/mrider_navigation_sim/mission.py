"""Explicit one-goal mission: Smac plan -> RPP follow; no reverse/recovery."""
import json
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Path
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
        self.lifecycle_clients={name:self.create_client(GetState,'/'+name+'/get_state') for name in ('planner_server','controller_server')}
        self.active={name:False for name in self.lifecycle_clients};self.pending={}
        self.create_timer(.2,self.poll_lifecycle)
        self.auto_resume=self.declare_parameter('auto_resume',False).value
        self.saved_goal=None;self.resume_pending=None;self.resume_ready=False
        self.auto_client=self.create_client(Trigger,'/mrider_sim/auto_arm')
        self.create_timer(.2,self.resume_tick)
        self.state='IDLE';self.safety='DISARMED';self.follow_handle=None;self.sequence=0
        self.planner=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
        self.follower=ActionClient(self,FollowPath,'/follow_path')
        self.pub=self.create_publisher(String,'/mrider_sim/mission_state',COMMAND_QOS)
        self.path_pub=self.create_publisher(Path,'/mrider_sim/planned_path',COMMAND_QOS)
        self.create_subscription(String,'/mrider_sim/safety_state',self.safety_cb,COMMAND_QOS)
        self.create_subscription(PoseStamped,'/goal_pose',self.goal_cb,COMMAND_QOS)
        self.create_service(Trigger,'/mrider_sim/start',self.start)
        self.create_timer(.1,lambda:self.pub.publish(String(data=self.state)))

    def resume_tick(self):
        if not self.auto_resume or self.saved_goal is None:return
        if self.safety=='DISARMED':self.resume_ready=False;return
        if self.resume_pending is not None:
            if not self.resume_pending.done():return
            self.resume_ready=self.resume_pending.result().success
            self.resume_pending=None
        if self.resume_ready and self.safety=='READY' and self.follow_handle is None:
            self.resume_ready=False
            self.begin(self.saved_goal)
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
            self.saved_goal=None;self.resume_ready=False
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

    def begin(self,goal):
        if self.state in ('PLANNING','FOLLOWING') or self.follow_handle is not None:return False,'mission_busy'
        if self.safety!='READY':return False,'manual_arm_required'
        try:
            pose_values(goal.pose)
            if goal.header.frame_id!='map':raise ValueError('goal_frame')
        except ValueError:return False,'invalid_goal'
        if not all(self.active.values()) or not self.planner.server_is_ready() or not self.follower.server_is_ready():return False,'nav2_not_ready'
        self.saved_goal=goal
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
        result=future.result()
        if result.status!=GoalStatus.STATUS_SUCCEEDED or len(result.result.path.poses)<2:
            self.state='FAILED';return
        self.path_pub.publish(result.result.path)
        req=FollowPath.Goal();req.path=result.result.path;req.controller_id='FollowPath';req.goal_checker_id='goal_checker'
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
        if seq==self.sequence:self.state='GOAL' if future.result().status==GoalStatus.STATUS_SUCCEEDED else 'FAILED'
