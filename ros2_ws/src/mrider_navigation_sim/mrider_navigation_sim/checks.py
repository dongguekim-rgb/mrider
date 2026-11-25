"""Executable simulation assertions; all control/service names are mrider_sim."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String, Bool, Float64
from visualization_msgs.msg import MarkerArray
from std_srvs.srv import Trigger, SetBool
from mrider_autonomy.nodes import SENSOR_QOS, COMMAND_QOS


class Checks(Node):
    def __init__(self):
        super().__init__('navigation_checks')
        self.odom=None;self.commands=[];self.states=[];self.mission='';self.path=None
        self.clearances=[];self.collision=False;self.trace=[];self.mission_events=[]
        self.create_subscription(Odometry,'/mrider_sim/odometry',self.odometry,SENSOR_QOS)
        self.create_subscription(AckermannDriveStamped,'/mrider_sim/command',self.commands.append,COMMAND_QOS)
        self.create_subscription(String,'/mrider_sim/safety_state',lambda m:self.states.append((time.monotonic(),json.loads(m.data))),COMMAND_QOS)
        self.create_subscription(String,'/mrider_sim/mission_state',self.mission_cb,COMMAND_QOS)
        self.create_subscription(Path,'/mrider_sim/planned_path',lambda m:setattr(self,'path',m),COMMAND_QOS)
        self.create_subscription(Float64,'/mrider_sim/clearance',lambda m:self.clearances.append(m.data),COMMAND_QOS)
        self.create_subscription(Bool,'/mrider_sim/collision',lambda m:setattr(self,'collision',self.collision or m.data),COMMAND_QOS)
        self.candidate=self.create_publisher(Twist,'/mrider_sim/candidate_cmd_vel',COMMAND_QOS)
        self.inject=False
        self.crossing_y=None;self.actor_trace=[];self.headings=[];self.turns=False;self.video_frames=[];self.video_start=time.monotonic()
        self.create_subscription(MarkerArray,'/mrider_sim/world_markers',self.markers,COMMAND_QOS)
        self.create_timer(.03,self.command)

    def mission_cb(self,msg):
        self.mission=msg.data;self.mission_events.append(msg.data)

    def markers(self,msg):
        from .geometry import obstacles
        for m in msg.markers:
            if m.ns=='obstacles' and m.id==len(obstacles('indoor_turns' if self.turns else 'indoor')) and m.action==m.ADD:
                self.crossing_y=m.pose.position.y
                self.actor_trace.append((m.pose.position.x,m.pose.position.y))

    def odometry(self,msg):
        self.odom=msg
        q=msg.pose.pose.orientation;self.headings.append(2*math.atan2(q.z,q.w))
        self.trace.append((msg.pose.pose.position.x,msg.pose.pose.position.y,msg.twist.twist.linear.x))
        self.video_frames.append({'t':time.monotonic()-self.video_start,
            'x':msg.pose.pose.position.x,'y':msg.pose.pose.position.y,'yaw':self.headings[-1],
            'speed':msg.twist.twist.linear.x,'pedestrian_y':self.crossing_y,
            'mission':self.mission,'safety':self.states[-1][1] if self.states else {},
            'steering':self.commands[-1].drive.steering_angle if self.commands else 0.})

    def command(self):
        if self.inject:
            msg=Twist();msg.linear.x=.4;self.candidate.publish(msg)

    def wait(self,predicate,timeout=5.):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.02)
            if predicate():return
        raise AssertionError({'timeout':timeout,'mission':self.mission,'safety':self.states[-1] if self.states else None})

    def spin(self,duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.02)

    def service(self,name,value=None):
        typ=Trigger if value is None else SetBool
        cli=self.create_client(typ,name)
        if not cli.wait_for_service(timeout_sec=10.):raise AssertionError('missing service '+name)
        req=typ.Request()
        if value is not None:req.data=value
        f=cli.call_async(req);self.wait(f.done,5.);res=f.result();self.destroy_client(cli);return res

    def arm(self):
        self.wait(lambda:self.odom is not None and self.states,10.)
        end=time.monotonic()+10
        while time.monotonic()<end:
            res=self.service('/mrider_sim/arm')
            if res.success:
                self.wait(lambda:self.states[-1][1]['state']=='READY');return
            self.spin(.2)
        raise AssertionError('cannot arm: '+res.message)

    def start(self):
        end=time.monotonic()+15
        while time.monotonic()<end:
            index=len(self.mission_events)
            res=self.service('/mrider_sim/start')
            if res.success:
                self.wait(lambda:any(s in ('PLANNING','FOLLOWING') for s in self.mission_events[index:]),3.)
                return
            self.spin(.2)
        raise AssertionError('cannot start: '+res.message)

    def assert_single_output(self):
        self.wait(lambda:self.odom is not None and self.states,10.)
        self.wait(lambda:len(self.get_publishers_info_by_topic('/mrider_sim/command'))==1,10.)
        publishers=self.get_publishers_info_by_topic('/mrider_sim/command')
        assert len(publishers)==1 and publishers[0].node_name=='mrider_safety_gate', 'command gate bypass'

    def nav_ready(self):
        from lifecycle_msgs.srv import GetState
        for name in ('planner_server','controller_server'):
            client=self.create_client(GetState,'/'+name+'/get_state')
            assert client.wait_for_service(timeout_sec=15.), name
            end=time.monotonic()+15.
            while time.monotonic()<end:
                f=client.call_async(GetState.Request());self.wait(f.done)
                if f.result().current_state.id==3:break
                self.spin(.2)
            else:raise AssertionError('Nav2 not active: '+name)
            self.destroy_client(client)
        self.spin(.5)

    def navigation(self,blocked=False,indoor=False):
        self.assert_single_output();self.nav_ready();self.arm();self.start()
        if blocked:
            self.wait(lambda:self.mission=='FAILED',15.)
            self.wait(lambda:self.states[-1][1]['state']=='STOPPED')
            assert all(m.drive.speed==0 for m in self.commands)
            assert not self.collision
            return {'case':'blocked','mission':self.mission,'max_speed':max(m.drive.speed for m in self.commands),'result':'PASS'}
        self.wait(lambda:self.mission in ('GOAL','FAILED','STOPPED'),110. if self.turns else 60.)
        assert self.mission=='GOAL',(self.mission,self.states[-1])
        self.wait(lambda:self.odom is not None and abs(self.odom.twist.twist.linear.x)<.01)
        error=math.hypot(self.odom.pose.pose.position.x-(20. if self.turns else 8.),self.odom.pose.pose.position.y)
        q=self.odom.pose.pose.orientation;yaw=2*math.atan2(q.z,q.w)
        assert error<=.23 and abs(yaw)<=.25,(error,yaw)
        assert not self.collision and min(self.clearances)>0
        assert all(0<=m.drive.speed<=.5001 and abs(m.drive.steering_angle)<=.3937 for m in self.commands)
        deviation=max(abs(p.pose.position.y) for p in self.path.poses)
        if not indoor:assert deviation>.8,('no obstacle detour',deviation)
        return {'case':'static_obstacle','goal_error_m':error,'yaw_error_rad':abs(yaw),
                'min_clearance_m':min(self.clearances),'max_path_lateral_m':deviation,
                'collision':self.collision,'max_steering_rad':max(abs(m.drive.steering_angle) for m in self.commands),'result':'PASS'}

    def recovery(self):
        self.assert_single_output();self.arm();self.start()
        self.wait(lambda:self.odom and self.odom.twist.twist.linear.x>.35,10.)
        assert self.service('/mrider_sim/fault/dynamic_obstacle',True).success
        self.wait(lambda:self.states[-1][1]['state']=='STOPPED',3.)
        self.wait(lambda:abs(self.odom.twist.twist.linear.x)<.01)
        assert not self.collision
        assert self.service('/mrider_sim/fault/dynamic_obstacle',False).success
        self.spin(.8)
        assert self.states[-1][1]['state']=='STOPPED' and self.mission=='STOPPED'
        assert all(m.drive.speed==0 for m in self.commands[-10:])
        self.arm();self.start()
        self.wait(lambda:self.mission in ('GOAL','FAILED','STOPPED'),110. if self.turns else 60.)
        assert self.mission=='GOAL',(self.mission,self.states[-1])
        self.wait(lambda:abs(self.odom.twist.twist.linear.x)<.01)
        error=math.hypot(self.odom.pose.pose.position.x-(20. if self.turns else 8.),self.odom.pose.pose.position.y)
        assert error<=.23 and not self.collision
        return {'case':'nav2_obstacle_recovery','manual_rearm_required':True,'goal_error_m':error,
                'min_clearance_m':min(self.clearances),'collision':self.collision,'result':'PASS'}

    def indoor_dynamic(self,auto=False):
        self.assert_single_output();self.nav_ready();self.arm();self.start()
        self.wait(lambda:self.mission in ('STOPPED','FAILED','GOAL'),100. if self.turns else 65.)
        assert self.mission in ('STOPPED','FAILED'),(self.mission,self.states[-1])
        self.wait(lambda:self.states[-1][1]['state']=='STOPPED')
        self.wait(lambda:abs(self.odom.twist.twist.linear.x)<.01)
        assert self.crossing_y is not None and not self.collision, (self.crossing_y,self.collision,self.trace[-1],self.states[-1])
        stopped_y=self.crossing_y
        stopped_reason=self.states[-1][1]
        self.wait(lambda:self.crossing_y>=3.29,25.)
        self.spin(.7)
        assert self.crossing_y>stopped_y+.5, 'pedestrian froze with vehicle'
        if not auto:
            assert self.states[-1][1]['state']=='STOPPED' and self.mission in ('STOPPED','FAILED')
            assert all(m.drive.speed==0 for m in self.commands[-10:]), 'automatic restart'
            self.arm();self.start()
        else:
            self.wait(lambda:self.mission in ('PLANNING','FOLLOWING','GOAL'),10.)
        self.wait(lambda:self.mission in ('GOAL','FAILED','STOPPED'),110. if self.turns else 60.)
        assert self.mission=='GOAL',(self.mission,self.states[-1])
        self.wait(lambda:abs(self.odom.twist.twist.linear.x)<.01)
        error=math.hypot(self.odom.pose.pose.position.x-(20. if self.turns else 8.),self.odom.pose.pose.position.y)
        assert error<=.23 and not self.collision and min(self.clearances)>0
        assert all(0<=m.drive.speed<=.5001 and abs(m.drive.steering_angle)<=.3937 for m in self.commands)
        return {'case':'indoor_dynamic','goal_error_m':error,'min_clearance_m':min(self.clearances),
                'collision':self.collision,'manual_rearm_required':not auto,'stop_state':stopped_reason,
                'pedestrian_y_at_stop_m':stopped_y,'pedestrian_y_after_clear_m':self.crossing_y,'result':'PASS'}

    def turns_demo(self):
        self.turns=True
        result=self.indoor_dynamic(auto=True)
        moving=[m.drive.steering_angle for m in self.commands if m.drive.speed>.05]
        left=max(moving);right=min(moving)
        changes=[math.atan2(math.sin(b-a),math.cos(b-a)) for a,b in zip(self.headings,self.headings[1:])]
        left_yaw=sum(d for d in changes if d>0);right_yaw=-sum(d for d in changes if d<0)
        lateral=max(abs(y) for x,y,v in self.trace)
        assert left>.1 and right<-.1, ('missing steering direction',left,right)
        assert left_yaw>math.radians(20) and right_yaw>math.radians(20), ('missing actual turn',left_yaw,right_yaw)
        assert lateral>.8, ('missing detour',lateral)
        result.update(case='indoor_turns',max_left_steering_rad=left,min_right_steering_rad=right,
                      left_yaw_accumulated_rad=left_yaw,right_yaw_accumulated_rad=right_yaw,max_lateral_m=lateral)
        return result

    def faults(self):
        self.assert_single_output()
        results=[]
        for kind in ['scan_dropout','scan_bad_frame','scan_future','scan_nan','odom_bad_frame','odom_dropout','command_dropout','dynamic_obstacle']:
            self.inject=False;self.spin(.15);self.arm();self.inject=True
            self.wait(lambda:self.odom.twist.twist.linear.x>.25)
            start=time.monotonic();initial=(self.odom.pose.pose.position.x,self.odom.pose.pose.position.y)
            if kind=='command_dropout':self.inject=False
            else:assert self.service('/mrider_sim/fault/'+kind,True).success
            self.wait(lambda:self.states[-1][1]['state']=='STOPPED',3.)
            detection=time.monotonic()-start
            # Odometry dropout prevents observing speed until restored; the
            # world still independently publishes clearance/collision.
            if kind=='odom_dropout':
                self.spin(1.);assert self.service('/mrider_sim/fault/'+kind,False).success
            self.wait(lambda:abs(self.odom.twist.twist.linear.x)<.01,3.)
            stopped=(self.odom.pose.pose.position.x,self.odom.pose.pose.position.y)
            distance=math.hypot(stopped[0]-initial[0],stopped[1]-initial[1])
            limit=.8 if kind=='dynamic_obstacle' else (.6 if kind.endswith('dropout') else .3)
            assert detection<limit and distance<.4,(kind,detection,distance)
            if kind not in ('command_dropout','odom_dropout'):assert self.service('/mrider_sim/fault/'+kind,False).success
            self.inject=True;self.spin(.6)
            assert self.states[-1][1]['state']=='STOPPED',('automatic restart',kind)
            assert all(m.drive.speed==0 for m in self.commands[-5:])
            assert not self.collision
            results.append({'fault':kind,'detection_s':detection,'travel_to_stop_m':distance,'rearm_required':True,'result':'PASS'})
        self.inject=False;self.service('/mrider_sim/disarm')
        return results


def run(kind):
    rclpy.init();node=Checks()
    try:
        result=node.turns_demo() if kind=='indoor_turns' else node.indoor_dynamic(auto=kind=='indoor_auto') if kind in ('indoor_dynamic','indoor_auto') else node.faults() if kind=='faults' else (node.recovery() if kind=='recovery' else node.navigation(kind=='blocked',kind=='indoor'))
        if kind=='indoor':result['case']='indoor'
        import os
        from pathlib import Path as FilePath
        directory=os.environ.get('MRIDER_SIM_REPORT_DIR')
        if directory:
            target=FilePath(directory);target.mkdir(parents=True,exist_ok=True)
            payload={'video_frames':node.video_frames,'result':result,'pedestrian_trajectory':node.actor_trace,'trajectory':node.trace,'planned_path':
                     [(p.pose.position.x,p.pose.position.y) for p in node.path.poses] if node.path else []}
            (target/(kind+'.json')).write_text(json.dumps(payload))
        print('SIMULATION_RESULT '+json.dumps(result),flush=True)
        return result
    finally:node.destroy_node();rclpy.shutdown()
