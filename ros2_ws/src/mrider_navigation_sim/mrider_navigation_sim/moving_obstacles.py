"""Short-horizon LiDAR motion tracks and conservative collision prediction.

Uses scan clusters transformed by estimated odometry only. This is neither
semantic pedestrian recognition nor a guarantee about an actor's future.
"""
import math
from collections import deque

import numpy as np


def clusters(points):
    if len(points)<3:return []
    splits=np.flatnonzero(np.linalg.norm(np.diff(points,axis=0),axis=1)>.45)+1
    groups=list(np.split(points,splits))
    if len(groups)>1 and np.linalg.norm(groups[0][0]-groups[-1][-1])<=.45:
        groups[0]=np.vstack((groups[-1],groups[0]));groups.pop()
    result=[]
    for group in groups:
        if not 3<=len(group)<=80 or np.max(np.ptp(group,axis=0))>1.2:continue
        lower,upper=group.min(axis=0),group.max(axis=0)
        center=(lower+upper)/2
        # Visible returns do not bound the unobserved rear of an object.
        radius=max(.2,float(np.max(np.linalg.norm(group-center,axis=1)))+.25)
        result.append((center,radius,lower,upper))
    return result


class MotionTracks:
    def __init__(self):
        self.tracks=[];self.first_stamp=None;self.last_stamp=None;self.next_id=0

    def update(self,points,pose,stamp):
        if (not math.isfinite(stamp) or stamp<=0 or
                self.last_stamp is not None and not 0<stamp-self.last_stamp<=.35):
            raise ValueError('motion_scan_gap')
        if self.first_stamp is None:self.first_stamp=stamp
        c,s=math.cos(pose[2]),math.sin(pose[2])
        observations=clusters(points@np.array([[c,s],[-s,c]])+pose[:2])
        active=[];outputs=[];used=set()
        for center,radius,lower,upper in observations:
            candidates=[]
            for index,track in enumerate(self.tracks):
                dt=stamp-track['history'][-1][0]
                if index in used or not 0<dt<=.35:continue
                distance=np.linalg.norm(center-track['history'][-1][1])
                if distance<=.25+1.5*dt:candidates.append((distance,index))
            candidates.sort()
            # Do not silently swap identities at a crossing/occlusion.
            if candidates and (len(candidates)==1 or candidates[1][0]-candidates[0][0]>.15):
                index=candidates[0][1];track=self.tracks[index];used.add(index)
            else:
                track={'id':self.next_id,'history':deque(maxlen=8)};self.next_id+=1
            track['history'].append((stamp,center.copy(),lower.copy(),upper.copy()));active.append(track)
            history=track['history'];duration=stamp-history[0][0]
            if len(history)<5 or duration<.35:continue
            times=np.array([h[0]-stamp for h in history]);centers=np.array([h[1] for h in history])
            design=np.column_stack((np.ones(len(times)),times))
            coefficients=np.linalg.lstsq(design,centers,rcond=None)[0]
            residual=float(np.max(np.linalg.norm(design@coefficients-centers,axis=1)))
            velocity=coefficients[1];speed=float(np.linalg.norm(velocity))
            if not .15<=speed<=1.5 or residual>.08:continue
            # A changing visible face can move its bounding-box centre even
            # though the object is stationary. A translation hypothesis must
            # be supported by BOTH ends of the observed extent. A face that
            # grows/shrinks behind an occluder is not a velocity measurement.
            low_velocity=np.linalg.lstsq(design,np.array([h[2] for h in history]),rcond=None)[0][1]
            high_velocity=np.linalg.lstsq(design,np.array([h[3] for h in history]),rcond=None)[0][1]
            extent_change=float(np.linalg.norm(high_velocity-low_velocity))
            if extent_change>max(.10,.35*speed):continue
            outputs.append(dict(id=track['id'],x=float(coefficients[0,0]),y=float(coefficients[0,1]),
                vx=float(velocity[0]),vy=float(velocity[1]),radius=radius,
                observations=len(history),span_s=duration,residual_m=residual,extent_change_mps=extent_change))
        self.tracks=active;self.last_stamp=stamp
        return dict(stamp=stamp,frame='odom',valid=stamp-self.first_stamp>=.35,tracks=outputs)


def motion_fault(sample,now,wall):
    if sample is None:return 'motion_prediction_missing'
    data,received=sample
    if not isinstance(data,dict) or data.get('frame')!='odom':return 'invalid_motion_prediction'
    stamp=data.get('stamp')
    if not isinstance(stamp,(int,float)) or not math.isfinite(stamp):return 'invalid_motion_prediction'
    if not -.05<=now-stamp<=.35 or not 0<=wall-received<=.35:return 'motion_prediction_timeout'
    if data.get('valid') is not True:return 'motion_prediction_unready'
    tracks=data.get('tracks')
    if not isinstance(tracks,list) or len(tracks)>128:return 'invalid_motion_prediction'
    for track in tracks:
        if not isinstance(track,dict):return 'invalid_motion_prediction'
        values=[track.get(k) for k in ('x','y','vx','vy','radius')]
        if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in values):return 'invalid_motion_prediction'
        if not 0<values[-1]<=2. or math.hypot(values[2],values[3])>1.5:return 'invalid_motion_prediction'
    return None


def collision_time(pose,speed,steering,limits,tracks,age=0.,horizon=12.):
    """Constant-motion prediction against the inflated car rectangle.

    Only adds stops. It never authorizes driving through current occupancy.
    At 0.35 m/s a quarter-turn with the 2.10 m measured radius takes 9.4 s.
    A 4 s horizon let the car enter an oncoming actor's lane before stopping;
    12 s reserves the turn plus braking/observation time. Growing prediction
    uncertainty is conservative and may cause extra waiting.
    0.1 s sampling is covered by relative-speed padding and growing uncertainty.
    """
    if not tracks:return None
    t=np.arange(0.,horizon+.001,.1);curvature=math.tan(steering)/limits.wheelbase
    travel=max(0.,speed)*t;angle=pose[2]+curvature*travel
    if abs(curvature)<1e-9:
        x=pose[0]+travel*math.cos(pose[2]);y=pose[1]+travel*math.sin(pose[2])
    else:
        x=pose[0]+(np.sin(angle)-math.sin(pose[2]))/curvature
        y=pose[1]-(np.cos(angle)-math.cos(pose[2]))/curvature
    earliest=None
    for track in tracks:
        px=track['x']+track['vx']*(max(0.,age)+t)-x
        py=track['y']+track['vy']*(max(0.,age)+t)-y
        forward=np.cos(angle)*px+np.sin(angle)*py
        lateral=-np.sin(angle)*px+np.cos(angle)*py
        dx=np.maximum(0.,np.maximum(-limits.rear-limits.margin-forward,forward-limits.front-limits.margin))
        dy=np.maximum(0.,np.abs(lateral)-limits.half_width-limits.margin)
        padding=.05*(max(0.,age)+t)+.1*(speed+math.hypot(track['vx'],track['vy']))
        hit=np.flatnonzero(np.hypot(dx,dy)<=track['radius']+padding)
        if len(hit):earliest=float(t[hit[0]]) if earliest is None else min(earliest,float(t[hit[0]]))
    return earliest
