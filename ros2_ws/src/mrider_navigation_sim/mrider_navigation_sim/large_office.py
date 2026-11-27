"""Original office layout inspired by multi-room navigation benchmarks.

Environment geometry only: never publish this as the controller's map.
Coordinates in metres. No HouseExpo/BARN assets are copied.
"""
BOUNDS=(-4.,-4.,56.,36.)
# 8 m circulation lanes around four room blocks; 3 m room entrances.
BLOCKS=((4.,4.,20.,12.),(28.,4.,48.,12.),(4.,20.,20.,28.),(28.,20.,48.,28.))
# Evaluation/operator stations, not a precomputed controller path.
STATIONS=((0.,0.),(12.,0.),(24.,0.),(40.,0.),(52.,0.),
          (52.,16.),(52.,32.),(24.,32.),(0.,32.),(0.,16.),(24.,16.))


def rooms():
    return [(a+i*(c-a)/2,b,a+(i+1)*(c-a)/2,d)
            for a,b,c,d in BLOCKS for i in range(2)]


def walls_and_furniture():
    a,b,c,d=BOUNDS;t=.2
    boxes=[(a,b,c,b+t),(a,d-t,c,d),(a,b,a+t,d),(c-t,b,c,d)]
    for x0,y0,x1,y1 in rooms():
        mid=(x0+x1)/2
        boxes.extend([(x0,y0,mid-1.5,y0+t),(mid+1.5,y0,x1,y0+t),
                      (x0,y1-t,x1,y1),(x0,y0,x0+t,y1),(x1-t,y0,x1,y1),
                      (x0+1.,y1-2.,x0+2.5,y1-1.),
                      (x1-2.5,y1-3.,x1-1.,y1-1.)])
    # Offset carts create local clutter without closing the circulation lanes.
    boxes.extend([(14.,1.8,15.,2.8),(35.,13.,36.,14.),(50.,25.,51.,26.)])
    return boxes


def triangle(t,span,speed):
    phase=(t*speed)%(2*span)
    return span-abs(span-phase)


class OfficeActors:
    """Two kinematic pedestrian proxies; no pedestrian intelligence assumed."""
    def __init__(self,initial_elapsed=0.):
        import math
        if not math.isfinite(initial_elapsed) or initial_elapsed<0:
            raise ValueError('nonnegative finite actor time offset required')
        self.elapsed=initial_elapsed

    def step(self,dt,vehicle_x):
        import math
        from .indoor import box
        if not math.isfinite(dt) or dt<0:raise ValueError('nonnegative finite dt required')
        self.elapsed+=dt
        return [box(24.,triangle(self.elapsed,32.,.5)),
                box(triangle(self.elapsed+25.,52.,.6),16.)]
