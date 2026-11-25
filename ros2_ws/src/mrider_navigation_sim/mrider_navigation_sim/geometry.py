"""Deterministic 2D geometry; ideal scan has no noise or material effects."""
import math


def obstacles(scenario):
    if scenario=='large_office':
        from .large_office import walls_and_furniture
        return walls_and_furniture()
    walls = [(-2., -5., 12., -4.8), (-2., 4.8, 12., 5.),
             (-2., -5., -1.8, 5.), (11.8, -5., 12., 5.)]
    if scenario == 'sensor_turns':
        from .indoor import walls as indoor_walls, furniture
        # Compact office, within the simulated 15 m scanner range.
        walls=[(-2.,-5.,14.,-4.8),(-2.,4.8,14.,5.),(-2.,-5.,-1.8,5.),(13.8,-5.,14.,5.)]
        walls.extend(indoor_walls()+furniture()[:-1])
        walls.append((3.5,-.7,4.3,.25))
        return walls
    if scenario == 'indoor_turns':
        from .indoor import walls as indoor_walls, furniture
        walls=[(-2.,-5.,24.,-4.8),(-2.,4.8,24.,5.),(-2.,-5.,-1.8,5.),(23.8,-5.,24.,5.)]
        walls.extend((2*a,b,2*c,d) for a,b,c,d in indoor_walls()+furniture()[:-1])
        walls.append((5.,-.5,6.,.5))
        return walls
    if scenario == 'static_obstacle': walls.append((3.4, -.6, 4.4, .6))
    elif scenario == 'blocked': walls.append((3.4, -5., 3.8, 5.))
    elif scenario in ('indoor', 'indoor_dynamic'):
        from .indoor import walls as indoor_walls, furniture
        walls.extend(indoor_walls() + furniture())
    elif scenario != 'clear': raise ValueError(scenario)
    return walls


def ray_distance(x, y, angle, boxes, max_range=15.):
    dx, dy = math.cos(angle), math.sin(angle)
    best = max_range
    for xmin, ymin, xmax, ymax in boxes:
        lo, hi = 0., max_range
        for origin, direction, low, high in ((x, dx, xmin, xmax), (y, dy, ymin, ymax)):
            if abs(direction) < 1e-10:
                if not low <= origin <= high: hi = -1.; break
            else:
                a, b = (low-origin)/direction, (high-origin)/direction
                lo, hi = max(lo, min(a,b)), min(hi, max(a,b))
        if hi >= lo: best = min(best, lo)
    return best


def footprint(pose, front=.9, rear=.2, half_width=.3):
    x, y, yaw = pose; c, s = math.cos(yaw), math.sin(yaw)
    return [(x+c*px-s*py, y+s*px+c*py) for px,py in
            ((-rear,-half_width),(front,-half_width),(front,half_width),(-rear,half_width))]


def intersects(a, b):
    for polygon in (a,b):
        for i, v in enumerate(polygon):
            w = polygon[(i+1)%len(polygon)]; axis = (v[1]-w[1], w[0]-v[0])
            pa = [x*axis[0]+y*axis[1] for x,y in a]
            pb = [x*axis[0]+y*axis[1] for x,y in b]
            if max(pa) < min(pb) or max(pb) < min(pa): return False
    return True


def point_segment(p, a, b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)))
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)


def clearance(pose, boxes, front=.9, rear=.2, half_width=.3):
    car = footprint(pose,front,rear,half_width); best = math.inf
    for x0,y0,x1,y1 in boxes:
        box=[(x0,y0),(x1,y0),(x1,y1),(x0,y1)]
        if intersects(car,box): return 0.
        for a,b in ((car,box),(box,car)):
            best=min(best,*(point_segment(v,b[i],b[(i+1)%4]) for v in a for i in range(4)))
    return best


def trajectory_point(distance, curvature):
    if abs(curvature)<1e-8:return distance,0.,0.
    angle=distance*curvature
    return math.sin(angle)/curvature,(1-math.cos(angle))/curvature,angle


def swept_hit(points, curvature, length, front, rear, half_width, margin):
    """First arc distance where a beam endpoint enters an expanded footprint."""
    count=max(1,math.ceil(length/.04))
    for i in range(count+1):
        d=length*i/count; x,y,a=trajectory_point(d,curvature);c,s=math.cos(a),math.sin(a)
        for px,py in points:
            dx,dy=px-x,py-y; lx,ly=c*dx+s*dy,-s*dx+c*dy
            if -rear-margin <= lx <= front+margin and abs(ly)<=half_width+margin:return d
    return math.inf


def ray_distances(x,y,angles,boxes,max_range=15.):
    """Vectorized slab intersections, identical geometry to ray_distance."""
    import numpy as np
    angles=np.asarray(angles,dtype=float)
    if not boxes:return np.full(angles.shape,max_range)
    bounds=np.asarray(boxes,float)
    lo=np.zeros((len(angles),len(boxes)));hi=np.full_like(lo,max_range)
    for origin,direction,low,high in ((x,np.cos(angles),bounds[:,0],bounds[:,2]),
                                    (y,np.sin(angles),bounds[:,1],bounds[:,3])):
        direction=direction[:,None];parallel=np.abs(direction)<1e-10
        a=np.divide(low[None,:]-origin,direction,out=np.zeros_like(lo),where=~parallel)
        b=np.divide(high[None,:]-origin,direction,out=np.zeros_like(lo),where=~parallel)
        lo=np.maximum(lo,np.where(parallel,-np.inf,np.minimum(a,b)))
        hi=np.minimum(hi,np.where(parallel,np.inf,np.maximum(a,b)))
        hi=np.where(parallel&((origin<low[None,:])|(origin>high[None,:])),-1.,hi)
    return np.min(np.where(hi>=lo,lo,max_range),axis=1)
