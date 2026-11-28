"""Revalidate a previously accepted route after an obstacle-only stop.

Uses observed occupancy and estimated pose. Planning padding stays unchanged;
the existing controller/gate footprint applies to continuing a valid route.
"""
import math
import numpy as np
from scipy.ndimage import distance_transform_edt

from .geometry import trajectory_point
from .local_goal import footprint_free
from .path_feasibility import validate_path, wrap


def validated_tail(points,current,grid,resolution,origin,footprint,wheelbase,max_steering,lookahead,goal=None):
    points=np.asarray(points,dtype=float);grid=np.asarray(grid)
    if (points.ndim!=2 or points.shape[1]!=3 or len(points)<2 or
            not np.isfinite(points).all() or grid.ndim!=2 or not grid.size or
            not all(math.isfinite(x) for x in (*current,*origin,resolution,*footprint)) or
            resolution<=0 or len(footprint)!=4 or min(footprint)<=0):
        raise ValueError('invalid_retained_path')
    nearest=int(np.argmin(np.linalg.norm(points[:,:2]-current[:2],axis=1)))
    # RPP prunes to the nearest sample. Adding its predecessor can falsely
    # violate the 0.35 m connection check on a valid, coarsely sampled path.
    # The current-pose tracking arc is checked separately below.
    start=nearest;tail=points[start:]
    metrics=validate_path(tail,current,wheelbase,max_steering,lookahead)
    if abs(wrap(tail[0,2]-current[2]))>.3:raise ValueError('retained_heading_mismatch')
    physical=tail
    if goal is not None:
        stop=int(np.argmin(np.linalg.norm(tail[:,:2]-goal[:2],axis=1)))
        if (math.dist(tail[stop,:2],goal[:2])>.1 or abs(wrap(tail[stop,2]-goal[2]))>.25):
            raise ValueError('retained_original_goal_mismatch')
        # The virtual terminal reference stabilizes RPP; the original-goal
        # monitor stops before driving it. Check the body up to the real goal,
        # while still validating curvature of the complete RPP reference.
        physical=np.vstack((tail[:stop+1],np.asarray(goal)))
    physical_length=float(np.linalg.norm(np.diff(physical[:,:2],axis=0),axis=1).sum())
    metrics['physical_length_m']=physical_length
    samples=[]
    for a,b in zip(physical,physical[1:]):
        angle=wrap(b[2]-a[2]);distance=np.linalg.norm(b[:2]-a[:2])
        count=max(1,math.ceil(distance/.04),math.ceil(abs(angle)/.02))
        samples.extend([a[0]+(b[0]-a[0])*t,a[1]+(b[1]-a[1])*t,a[2]+angle*t]
                       for t in np.linspace(0.,1.,count+1))
    # Check the actual initial Pure Pursuit arc as well as the reference.
    c,s=math.cos(current[2]),math.sin(current[2])
    for distance in np.linspace(0.,min(lookahead,physical_length),int(math.ceil(lookahead/.04))+1):
        x,y,yaw=trajectory_point(distance,metrics['initial_tracking_curvature'])
        samples.append([current[0]+c*x-s*y,current[1]+s*x+c*y,current[2]+yaw])
    free=(grid>=0)&(grid<=20)
    clearance=distance_transform_edt(np.pad(free,1))[1:-1,1:-1]*resolution
    if not footprint_free(clearance,resolution,origin,samples,footprint).all():
        raise ValueError('retained_route_not_observed_free')
    return start,metrics
