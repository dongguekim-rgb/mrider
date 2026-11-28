"""Choose a bounded goal using only observed OccupancyGrid free space.

This is a conservative goal-progress policy, not full autonomous exploration.
Nav2 must still validate the Ackermann path to the selected point.
"""
import math
import numpy as np
from scipy.ndimage import distance_transform_edt, label


def select_goal(data,resolution,origin,current,destination,max_step=6.,radius=1.1,diagnostics=None):
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
    d=diagnostics if diagnostics is not None else {}
    d.update(start=list(start),required_clearance=radius+resolution/math.sqrt(2),
             start_clearance=float(clearance[start]) if inside(start) else None)
    if not inside(start) or not safe[start]:
        d['reason']='start_outside' if not inside(start) else 'start_not_clear';return None
    rows,cols=np.indices(grid.shape)
    x=origin[0]+(cols+.5)*resolution;y=origin[1]+(rows+.5)*resolution
    distance=np.hypot(x-current[0],y-current[1])
    # C-level connected components avoids blocking the ROS TF callback for a
    # Python flood fill over tens of thousands of 5 cm cells.
    components,_=label(safe&(distance<=max_step+resolution))
    component=components[start]
    if component==0:d['reason']='no_component';return None
    reachable=components==component
    if inside(target) and reachable[target] and math.dist(current,destination)<=max_step:
        d['reason']='final'
        return (*destination,True)
    error=np.hypot(x-destination[0],y-destination[1])
    # Do not leave a centimetre-scale final leg: its grid-quantized lateral
    # offset makes atan2(destination-local) demand an abrupt terminal heading.
    # Reserve at least 1 m for the final approach without extending max_step.
    step_limit=min(max_step,math.dist(current,destination)-1.)
    scores=np.where(reachable&(distance<=step_limit),error,np.inf)
    cell=np.unravel_index(np.argmin(scores),scores.shape)
    if not np.isfinite(scores[cell]) or math.dist(current,destination)-scores[cell]<1.:
        d['reason']='no_progress';return None
    d['reason']='intermediate'
    return float(x[cell]),float(y[cell]),False


def select_pose_goal(data, resolution, origin, current, destination,
                     turning_radius, max_step=6., radius=1.1,diagnostics=None,footprint=None):
    """Choose a reachable forward arc end pose for every intermediate goal.

    The old XY-only goal could be directly beside the car, with no room for
    its bounded-radius turn. Candidate arcs use the planner radius and only
    observed, inflated free space. Nav2 still plans/checks the final path.
    """
    if (not all(math.isfinite(v) for v in (*current, *destination, turning_radius))
            or len(current) != 3 or len(destination) != 3 or turning_radius <= 0):
        raise ValueError('invalid_pose_goal')
    # Preserve room to align with the FINAL orientation, instead of inserting
    # a short, grid-quantized approach with a different terminal heading.
    # Only an observed connected final goal gets this extra 2 m horizon.
    d=diagnostics if diagnostics is not None else {}
    d.update(current=list(current),destination=list(destination),turning_radius=turning_radius,
             radius=radius,footprint=footprint,max_step=max_step,final_check={},spatial_check={})
    # Connectivity is only a coarse prefilter. An enclosing circle rejects
    # a safe north-facing car alongside a wall because its long front extends
    # sideways in that approximation. Check the actual oriented rectangle.
    connectivity_radius=radius
    grid=np.asarray(data)
    if (grid.ndim!=2 or not grid.size or
            not all(math.isfinite(v) for v in (*origin,resolution,max_step,radius)) or
            min(resolution,max_step,radius)<=0):
        raise ValueError('invalid_local_goal_input')
    free=(grid>=0)&(grid<=20)
    clearance=distance_transform_edt(np.pad(free,1))[1:-1,1:-1]*resolution
    if footprint is not None:
        if len(footprint)!=4 or not all(math.isfinite(v) and v>0 for v in footprint):
            raise ValueError('invalid_goal_footprint')
        connectivity_radius=min(footprint[:3])+footprint[3]
        if not footprint_free(clearance,resolution,origin,[current],footprint)[0]:
            d['reason']='start_footprint_blocked';return None
    final = select_goal(data, resolution, origin, current[:2], destination[:2], max_step+2., connectivity_radius,d['final_check'])
    final_clear=(footprint is None or footprint_free(clearance,resolution,origin,[destination],footprint)[0])
    if final is not None and final[2]:
        # A clear goal centre is not a clear final car pose. During recovery
        # a crossing actor may occupy its front/side. Turning away to another
        # subgoal here can force a full Dubins loop in the last few metres.
        # Preserve the final approach and let the bounded recovery wait for
        # fresh observations instead of replacing it with that detour.
        if not final_clear:
            d['reason']='final_footprint_blocked';return None
        d['reason']='final'
        return *destination, True
    selected = select_goal(data, resolution, origin, current[:2], destination[:2], max_step, connectivity_radius,d['spatial_check'])
    if selected is None:
        d['reason']='spatial_'+d['spatial_check']['reason']
        return None
    safe = clearance > radius+resolution/math.sqrt(2)
    best = None
    original_error = math.dist(current[:2], destination[:2])
    # Keep the subgoal inside the planner's reachable set, not exactly on its
    # minimum-radius boundary. Smac quantizes XY and heading; a boundary pose
    # can otherwise require a full Dubins loop after rounding.
    subgoal_radius = 1.05*turning_radius
    counts=dict(outside=0,clearance=0,progress=0,approach=0,accepted=0)
    d['arc_counts']=counts
    for curvature in np.linspace(-1/subgoal_radius, 1/subgoal_radius, 41):
        # Reuse a dense full arc for all candidate lengths at this curvature.
        full_travel=np.linspace(0.,max_step,int(math.ceil(max_step/(resolution/2)))+1)
        full_angle=current[2]+curvature*full_travel
        if abs(curvature)<1e-9:
            full_x=current[0]+full_travel*math.cos(current[2]);full_y=current[1]+full_travel*math.sin(current[2])
        else:
            full_x=current[0]+(np.sin(full_angle)-math.sin(current[2]))/curvature
            full_y=current[1]-(np.cos(full_angle)-math.cos(current[2]))/curvature
        body_safe=(footprint_free(clearance,resolution,origin,np.column_stack((full_x,full_y,full_angle)),footprint)
                   if footprint is not None else None)
        for length in np.arange(2., max_step+.001, .25):
            travel = np.linspace(0., length, int(math.ceil(length/(resolution/2)))+1)
            angle = current[2]+curvature*travel
            if abs(curvature) < 1e-9:
                x = current[0]+travel*math.cos(current[2]); y = current[1]+travel*math.sin(current[2])
            else:
                x = current[0]+(np.sin(angle)-math.sin(current[2]))/curvature
                y = current[1]-(np.cos(angle)-math.cos(current[2]))/curvature
            rows = np.floor((y-origin[1])/resolution).astype(int)
            cols = np.floor((x-origin[0])/resolution).astype(int)
            if (np.any(rows < 0) or np.any(rows >= grid.shape[0]) or np.any(cols < 0)
                    or np.any(cols >= grid.shape[1])):
                counts['outside']+=1;continue
            fits=(np.all(safe[rows,cols]) if body_safe is None else
                  np.all(body_safe[:np.searchsorted(full_travel,length,side='right')+1]))
            if not fits:
                counts['clearance']+=1
                continue
            error = math.hypot(x[-1]-destination[0], y[-1]-destination[1])
            # If the final pose is still unobserved/blocked, retain a full
            # turning-radius approach instead of stopping beside it.
            if original_error-error < 1.:counts['progress']+=1;continue
            if error < turning_radius:counts['approach']+=1;continue
            counts['accepted']+=1
            goal_bearing = math.atan2(destination[1]-y[-1], destination[0]-x[-1])
            mismatch = abs(math.atan2(math.sin(goal_bearing-angle[-1]), math.cos(goal_bearing-angle[-1])))
            # Account for the distance needed to align a car-like vehicle.
            # A fixed 0.25 m/rad penalty overvalued immediate XY progress:
            # a requested quarter turn ended at ~131 degrees, requiring a
            # second turn back across the corridor. R * angle expresses
            # heading mismatch in metres without changing physical limits.
            score = error+turning_radius*mismatch
            if best is None or score < best[0]:
                best = (score, float(x[-1]), float(y[-1]), float(angle[-1]), False)
    d['reason']='arc' if best else 'no_arc'
    return best[1:] if best else None


def footprint_free(clearance,resolution,origin,poses,footprint):
    """Raster cover of the whole inflated rectangle, including its interior.

    Half-cell samples plus a full cell diagonal clearance conservatively
    cover rounding and the gaps between samples. Unknown is an obstacle.
    """
    front,rear,half,margin=footprint
    poses=np.asarray(poses)
    center_cols=np.floor((poses[:,0]-origin[0])/resolution).astype(int)
    center_rows=np.floor((poses[:,1]-origin[1])/resolution).astype(int)
    inside=(center_rows>=0)&(center_cols>=0)&(center_rows<clearance.shape[0])&(center_cols<clearance.shape[1])
    center_rows=np.clip(center_rows,0,clearance.shape[0]-1);center_cols=np.clip(center_cols,0,clearance.shape[1]-1)
    enclosing=math.hypot(max(front,rear)+margin,half+margin)+resolution*math.sqrt(2)
    result=inside&(clearance[center_rows,center_cols]>enclosing)
    pending=~result
    if not np.any(pending):return result
    poses=poses[pending]
    x=np.linspace(-rear-margin,front+margin,int(math.ceil((front+rear+2*margin)/(resolution/2)))+1)
    y=np.linspace(-half-margin,half+margin,int(math.ceil(2*(half+margin)/(resolution/2)))+1)
    xx,yy=np.meshgrid(x,y);xx=xx.ravel();yy=yy.ravel()
    a=poses[:,2,None];c=np.cos(a);s=np.sin(a)
    cols=np.floor((poses[:,0,None]+c*xx-s*yy-origin[0])/resolution).astype(int)
    rows=np.floor((poses[:,1,None]+s*xx+c*yy-origin[1])/resolution).astype(int)
    inside=(rows>=0)&(cols>=0)&(rows<clearance.shape[0])&(cols<clearance.shape[1])
    rows=np.clip(rows,0,clearance.shape[0]-1);cols=np.clip(cols,0,clearance.shape[1]-1)
    result[pending]=np.all(inside&(clearance[rows,cols]>resolution*math.sqrt(2)),axis=1)
    return result



def add_scan_evidence(data,resolution,origin,laser_pose,ranges,amin,increment,rmin,rmax,max_range_is_clear=False):
    """Fill unknown cells only within adjacent valid beams, never erase occupancy.

    A conservative sector approximation at scanner angular resolution; not a
    guarantee that objects smaller than the beam spacing are detectable.
    """
    grid=np.asarray(data).copy()
    evidence=scan_free_evidence(grid.shape,resolution,origin,laser_pose,ranges,amin,increment,rmin,rmax,max_range_is_clear)
    grid[(grid<0)&evidence]=0
    return grid


def scan_free_evidence(shape,resolution,origin,laser_pose,ranges,amin,increment,rmin,rmax,max_range_is_clear=False):
    """Conservative free sector mask; invalid beams and endpoints never clear."""
    ranges=np.asarray(ranges,dtype=float)
    if (len(shape)!=2 or min(shape)<=0 or ranges.ndim!=1 or len(ranges)<180 or
        not all(math.isfinite(v) for v in (resolution,*origin,*laser_pose,amin,increment,rmin,rmax)) or
        resolution<=0 or increment<=0 or not 0<rmin<rmax):raise ValueError('invalid_scan_geometry')
    # A real return at 14.99 m is still valid for a 15 m scanner. Dropping a
    # 2 cm band near the limit left whole unknown wedges, even under the car.
    # The endpoint margin below already prevents clearing up to that return.
    valid=np.isfinite(ranges)&(ranges>=rmin)&(ranges<rmax)
    if max_range_is_clear:valid|=np.isfinite(ranges)&(ranges==rmax)
    # Cells outside the scanner's bounding square cannot be cleared. Keeping
    # all trigonometry inside that window avoids processing a growing building
    # map three times per tick, which otherwise starves scan/TF callbacks.
    evidence=np.zeros(shape,dtype=bool)
    c0=max(0,math.floor((laser_pose[0]-rmax-origin[0])/resolution))
    c1=min(shape[1],math.ceil((laser_pose[0]+rmax-origin[0])/resolution))
    r0=max(0,math.floor((laser_pose[1]-rmax-origin[1])/resolution))
    r1=min(shape[0],math.ceil((laser_pose[1]+rmax-origin[1])/resolution))
    if c0>=c1 or r0>=r1:return evidence
    x=origin[0]+(np.arange(c0,c1)[None,:]+.5)*resolution-laser_pose[0]
    y=origin[1]+(np.arange(r0,r1)[:,None]+.5)*resolution-laser_pose[1]
    angle=np.arctan2(y,x)-laser_pose[2]
    angle=(angle+math.pi)%(2*math.pi)-math.pi
    index=np.floor((angle-amin)/increment).astype(int)
    if abs(len(ranges)*increment-2*math.pi)<1e-4:
        # The last and first beams are adjacent on a complete 360-degree scan.
        # Omitting this sector leaves an unknown seam behind the vehicle that
        # can split its footprint from otherwise connected observed space.
        coverage=np.ones(index.shape,dtype=bool)
        left=index%len(ranges);right=(left+1)%len(ranges)
    else:
        coverage=(index>=0)&(index<len(ranges)-1)
        left=np.clip(index,0,len(ranges)-2);right=left+1
    limit=np.minimum(ranges[left],ranges[right])-.15-resolution/math.sqrt(2)
    evidence[r0:r1,c0:c1]=coverage&valid[left]&valid[right]&(np.hypot(x,y)<limit)
    return evidence
