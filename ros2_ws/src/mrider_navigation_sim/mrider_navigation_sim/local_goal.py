"""Choose a bounded goal using only observed OccupancyGrid free space.

This is a conservative goal-progress policy, not full autonomous exploration.
Nav2 must still validate the Ackermann path to the selected point.
"""
import math
import numpy as np
from scipy.ndimage import distance_transform_edt, label


def select_goal(data,resolution,origin,current,destination,max_step=6.,radius=1.1):
    grid=np.asarray(data)
    values=(*origin,*current,*destination,resolution,max_step,radius)
    if grid.ndim!=2 or not all(math.isfinite(v) for v in values) or min(resolution,max_step,radius)<=0:
        raise ValueError('invalid_local_goal_input')
    free=(grid>=0)&(grid<=20)
    # Outside the received map is unknown, not free space.
    clearance=distance_transform_edt(np.pad(free,1,constant_values=False))[1:-1,1:-1]*resolution
    safe=clearance>radius+resolution/math.sqrt(2)
    def index(point):return (int(math.floor((point[1]-origin[1])/resolution)),int(math.floor((point[0]-origin[0])/resolution)))
    def inside(p):return 0<=p[0]<grid.shape[0] and 0<=p[1]<grid.shape[1]
    start=index(current);target=index(destination)
    if not inside(start) or not safe[start]:return None
    rows,cols=np.indices(grid.shape)
    x=origin[0]+(cols+.5)*resolution;y=origin[1]+(rows+.5)*resolution
    distance=np.hypot(x-current[0],y-current[1])
    # C-level connected components avoids blocking the ROS TF callback for a
    # Python flood fill over tens of thousands of 5 cm cells.
    components,_=label(safe&(distance<=max_step+resolution))
    component=components[start]
    if component==0:return None
    reachable=components==component
    if inside(target) and reachable[target] and math.dist(current,destination)<=max_step:
        return (*destination,True)
    error=np.hypot(x-destination[0],y-destination[1])
    # Do not leave a centimetre-scale final leg: its grid-quantized lateral
    # offset makes atan2(destination-local) demand an abrupt terminal heading.
    # Reserve at least 1 m for the final approach without extending max_step.
    step_limit=min(max_step,math.dist(current,destination)-1.)
    scores=np.where(reachable&(distance<=step_limit),error,np.inf)
    cell=np.unravel_index(np.argmin(scores),scores.shape)
    if not np.isfinite(scores[cell]) or math.dist(current,destination)-scores[cell]<1.:return None
    return float(x[cell]),float(y[cell]),False



def add_scan_evidence(data,resolution,origin,laser_pose,ranges,amin,increment,rmin,rmax,max_range_is_clear=False):
    """Fill unknown cells only within adjacent valid beams, never erase occupancy.

    A conservative sector approximation at scanner angular resolution; not a
    guarantee that objects smaller than the beam spacing are detectable.
    """
    grid=np.asarray(data).copy();ranges=np.asarray(ranges,dtype=float)
    if (grid.ndim!=2 or ranges.ndim!=1 or len(ranges)<180 or
        not all(math.isfinite(v) for v in (resolution,*origin,*laser_pose,amin,increment,rmin,rmax)) or
        resolution<=0 or increment<=0 or not 0<rmin<rmax):raise ValueError('invalid_scan_geometry')
    valid=np.isfinite(ranges)&(ranges>=rmin)&(ranges<rmax-.02)
    if max_range_is_clear:valid|=np.isfinite(ranges)&(np.abs(ranges-rmax)<1e-6)
    rows,cols=np.indices(grid.shape)
    x=origin[0]+(cols+.5)*resolution-laser_pose[0]
    y=origin[1]+(rows+.5)*resolution-laser_pose[1]
    angle=np.arctan2(y,x)-laser_pose[2]
    angle=np.arctan2(np.sin(angle),np.cos(angle))
    index=np.floor((angle-amin)/increment).astype(int)
    if abs(len(ranges)*increment-2*math.pi)<1e-4:
        # The last and first beams are adjacent on a complete 360-degree scan.
        # Omitting this sector leaves an unknown seam behind the vehicle that
        # can split its footprint from otherwise connected observed space.
        coverage=np.ones(grid.shape,dtype=bool)
        left=index%len(ranges);right=(left+1)%len(ranges)
    else:
        coverage=(index>=0)&(index<len(ranges)-1)
        left=np.clip(index,0,len(ranges)-2);right=left+1
    limit=np.minimum(ranges[left],ranges[right])-.15-resolution/math.sqrt(2)
    evidence=coverage&valid[left]&valid[right]&(np.hypot(x,y)<limit)
    grid[(grid<0)&evidence]=0
    return grid
