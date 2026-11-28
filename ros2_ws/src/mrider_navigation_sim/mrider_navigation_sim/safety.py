"""Simulation safety gate: strict scan/odom contracts, latch and manual rearm."""
import math
from dataclasses import dataclass, fields
from .geometry import swept_hit


@dataclass(frozen=True)
class Limits:
    wheelbase: float=.65
    max_speed: float=.5
    max_steering: float=.3927
    front: float=.9
    rear: float=.2
    half_width: float=.3
    margin: float=.15
    braking_deceleration: float=.8
    reaction_time: float=.25
    slowdown_distance: float=.7
    timeout: float=.35
    future_tolerance: float=.05

    def __post_init__(self):
        if any(not math.isfinite(v) or v<=0 for v in self.__dict__.values()):raise ValueError('invalid limits')
        if self.max_steering>=math.pi/2:raise ValueError('invalid steering')


class Gate:
    def __init__(self, limits=Limits()):
        self.c=limits;self.scan=None;self.odom=None;self.cmd=None
        self.stamps={};self.armed=False;self.latched=False;self.started=False
        self.state='DISARMED';self.reason='operator_permission_required';self.previous=None
        self.last_mission=None;self.manual_fault=False
        self.clear_since=None

    def mission_state(self, state):
        # Repeated status heartbeats must not undo an explicit manual rearm.
        if state==self.last_mission:return
        self.last_mission=state
        if state in ('GOAL','WAYPOINT'):self.goal()
        elif state=='FAILED':
            obstacle=(not self.manual_fault and
                      ((self.state=='SLOW' and self.reason=='obstacle_slowdown') or
                       (self.state=='STOPPED' and self.reason=='obstacle_stop')))
            self.fault('obstacle_stop' if obstacle else 'planning_or_tracking_failed')

    def continue_route(self,now,wall):
        if self.state!='GOAL' or self.latched:return False,'not_completed_waypoint'
        return self.arm(now,wall)

    def auto_arm(self, now, wall):
        # Only a classified obstacle stop can recover; any other fault wins.
        eligible=(not self.manual_fault and self.state=='STOPPED' and self.reason in ('obstacle_stop','camera_obstacle') and
                  self.sensors_ready(now,wall) and abs(self.odom[0][3])<.01)
        if eligible:
            eligible=swept_hit(self.scan[0],0.,2.,self.c.front,self.c.rear,
                               self.c.half_width,self.c.margin)==math.inf
        if not eligible:
            self.clear_since=None;return False,'not_clear_or_manual_fault'
        if self.clear_since is None:self.clear_since=wall
        if wall-self.clear_since<3.:return False,'clearance_dwell'
        self.clear_since=None
        return self.arm(now,wall)

    def fault(self, reason):
        self.clear_since=None
        if self.armed or self.latched:
            # An obstacle observation must never erase a sensor/control fault.
            recoverable=reason in ('obstacle_stop','camera_obstacle')
            if not self.manual_fault or not recoverable:self.reason=reason
            self.manual_fault=self.manual_fault or not recoverable
            self.latched=True;self.state='STOPPED'
        self.armed=False

    def valid_stamp(self, key, stamp, now):
        valid=(math.isfinite(stamp) and stamp>0 and
               -self.c.future_tolerance<=now-stamp<=self.c.timeout and
               (key not in self.stamps or stamp>self.stamps[key]))
        if valid:self.stamps[key]=stamp
        return valid

    def scan_input(self, ranges, amin, inc, rmin, rmax, frame, stamp, now, wall):
        self.scan=None
        if (frame!='laser' or not self.valid_stamp('scan',stamp,now) or
            not all(math.isfinite(v) for v in (amin,inc,rmin,rmax)) or
            len(ranges)<180 or inc<=0 or (len(ranges)-1)*inc<6. or not 0<rmin<rmax or
            not all(math.isfinite(v) and rmin<=v<=rmax for v in ranges)):
            self.fault('invalid_scan');return False
        points=[(r*math.cos(amin+i*inc),r*math.sin(amin+i*inc)) for i,r in enumerate(ranges) if r<rmax-.01]
        self.scan=(points,stamp,wall);return True

    def odom_input(self, x,y,yaw,v,w,frame,child,stamp,now,wall):
        self.odom=None
        if (frame!='odom' or child!='base_link' or not all(math.isfinite(a) for a in (x,y,yaw,v,w))
            or not self.valid_stamp('odom',stamp,now)):
            self.fault('invalid_odometry');return False
        self.odom=((x,y,yaw,v,w),stamp,wall);return True

    def command_input(self,v,w,now,wall):
        self.cmd=None
        if (not all(math.isfinite(a) for a in (v,w)) or v<0 or v>self.c.max_speed+1e-4 or
            (abs(v)<1e-6 and abs(w)>1e-5)):
            self.fault('invalid_command');return False
        steer=math.atan(self.c.wheelbase*w/v) if v>1e-6 else 0.
        # Regulated Pure Pursuit can request more curvature than the vehicle can
        # execute; never silently change the planned path by saturating it.
        if abs(steer)>self.c.max_steering+1e-3:
            self.fault('untrackable_curvature');return False
        self.cmd=((v,steer),now,wall)
        return True

    def fresh(self, sample, now, wall):
        return sample is not None and -self.c.future_tolerance<=now-sample[1]<=self.c.timeout and 0<=wall-sample[2]<=self.c.timeout

    def sensors_ready(self,now,wall):
        return self.fresh(self.scan,now,wall) and self.fresh(self.odom,now,wall)

    def arm(self,now,wall):
        if not self.sensors_ready(now,wall):return False,'sensors_not_ready'
        if abs(self.odom[0][3])>.02:return False,'vehicle_not_stopped'
        # Check front safety envelope even while stationary; rearm cannot use
        # an old steering command to make a nearby obstacle disappear.
        hit=swept_hit(self.scan[0],0.,.1,self.c.front,self.c.rear,self.c.half_width,self.c.margin)
        if hit<math.inf:return False,'obstacle_not_clear'
        self.armed=True;self.latched=False;self.manual_fault=False;self.started=False;self.cmd=None
        self.state='READY';self.reason='manual_arm';return True,'armed'

    def disarm(self):
        self.armed=False;self.latched=False;self.manual_fault=False;self.started=False;self.cmd=None
        self.state='DISARMED';self.reason='operator_disarm'

    def goal(self):
        if self.latched or not self.armed:return
        self.disarm();self.state='GOAL';self.reason='goal_reached'

    def output(self,now,wall):
        if self.previous is not None and now<self.previous:
            self.scan=self.odom=self.cmd=None;self.stamps.clear();self.fault('clock_rewind')
        self.previous=now
        if self.state=='STOPPED' and self.reason in ('obstacle_stop','camera_obstacle') and not self.sensors_ready(now,wall):
            self.fault('sensor_timeout')
        if not self.armed:return 0.,0.
        if not self.sensors_ready(now,wall):self.fault('sensor_timeout');return 0.,0.
        if not self.fresh(self.cmd,now,wall):
            if self.started:self.fault('command_timeout')
            return 0.,0.
        v,steer=self.cmd[0]
        # RPP sends zero while its collision check waits/aborts. That zero
        # loses the commanded curve; it must not erase an already observed
        # obstacle slowdown and turn its stop into an unrelated manual fault.
        if v==0 and self.started and self.state=='SLOW' and self.reason=='obstacle_slowdown':
            self.fault('obstacle_stop');return 0.,0.
        if v>0:self.started=True
        measured_v=self.odom[0][3];measured_w=self.odom[0][4]
        speed=max(v,abs(measured_v))
        stop_distance=speed*self.c.reaction_time+speed*speed/(2*self.c.braking_deceleration)
        curvature=math.tan(steer)/self.c.wheelbase
        curves=[curvature]
        if abs(measured_v)>.02:curves.append(measured_w/measured_v)
        hit=min(swept_hit(self.scan[0],k,stop_distance+self.c.slowdown_distance,
                         self.c.front,self.c.rear,self.c.half_width,self.c.margin) for k in curves)
        if hit<=stop_distance+.04:
            self.fault('obstacle_stop');return 0.,0.
        if hit<math.inf:
            v*=max(.1,min(1.,(hit-stop_distance)/self.c.slowdown_distance));self.state='SLOW'
        else:self.state='TRACKING' if v>0 else 'READY'
        self.reason='clear' if hit==math.inf else 'obstacle_slowdown'
        return v,steer


def load_limits(node):
    return Limits(**{f.name:node.declare_parameter(f.name,f.default).value for f in fields(Limits)})
