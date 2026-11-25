import math
from collections import deque
import numpy as np
from mrider_navigation_sim.large_office import BOUNDS,STATIONS,rooms,walls_and_furniture,OfficeActors
from mrider_navigation_sim.geometry import ray_distance,ray_distances,clearance


def test_dimensions_and_rooms():
    a,b,c,d=BOUNDS
    assert (c-a,d-b)==(60.,40.) and len(rooms())==8
    for x0,y0,x1,y1 in walls_and_furniture():
        assert a<=x0<x1<=c and b<=y0<y1<=d


def test_stations_connected_with_clearance():
    # Flood fill verifies free-space connectivity, NOT Ackermann feasibility.
    boxes=walls_and_furniture();step=.5;radius=.65
    def free(i,j):
        x,y=i*step,j*step
        return -4<x<56 and -4<y<36 and not any(
            a-radius<=x<=c+radius and b-radius<=y<=d+radius for a,b,c,d in boxes)
    seen={(0,0)};queue=deque(seen)
    while queue:
        i,j=queue.popleft()
        for key in [(i+1,j),(i-1,j),(i,j+1),(i,j-1)]:
            if key not in seen and free(*key):seen.add(key);queue.append(key)
    assert all((round(x/step),round(y/step)) in seen for x,y in STATIONS)


def test_initial_vehicle_clear():
    assert clearance((0,0,0),walls_and_furniture(),.9,.2,.3)>1.


def test_scan_range_not_scaled_with_map():
    angles=np.linspace(-math.pi,math.pi,360,endpoint=False)
    scan=ray_distances(0,0,angles,walls_and_furniture(),15.)
    assert np.isfinite(scan).all() and min(scan)>0 and max(scan)<=15.
    assert np.any(scan==15.), 'large environment includes unobserved distant space'
    expected=[ray_distance(0,0,a,walls_and_furniture(),15.) for a in angles]
    assert np.allclose(scan,expected)


def test_actors_stay_in_free_space():
    actors=OfficeActors();world=walls_and_furniture()
    for _ in range(600):
        for a,b,c,d in actors.step(1.,0.):
            assert clearance(((a+c)/2,(b+d)/2,0),world,.25,.25,.25)>0


def test_actors_move_without_vehicle_motion():
    actors=OfficeActors();before=actors.step(0.,0.)
    after=actors.step(5.,0.)
    assert all(a!=b for a,b in zip(before,after))


def test_shared_camera_renderer_matches_existing_fixture():
    from test_segmentation import scene
    from mrider_navigation_sim.synthetic_camera import render_camera
    boxes=[(5.,-5.,5.2,5.),(-1.,-3.,6.,-2.8),(-1.,2.8,6.,3.)]
    expected,_=scene()
    assert np.array_equal(render_camera((0,0,0),boxes),expected)


def test_large_office_unknown_is_not_silently_cleared():
    from mrider_navigation_sim.synthetic_camera import render_camera
    from mrider_navigation_sim.segmentation import segment,CameraGeometry
    boxes=walls_and_furniture()+OfficeActors().step(0.,0.)
    angles=-math.pi+np.arange(360)*2*math.pi/360
    scan=ray_distances(0,0,angles,boxes,15.)
    _,_,result=segment(render_camera((0,0,0),boxes),scan,-math.pi,2*math.pi/360,.05,15.,CameraGeometry(180,180,160,120,.6))
    assert not result['clear'] and result['reason']=='unknown_region'


def test_verified_simulation_scan_allows_large_office_start():
    from mrider_navigation_sim.synthetic_camera import render_camera
    from mrider_navigation_sim.segmentation import segment,CameraGeometry
    boxes=walls_and_furniture()+OfficeActors().step(0.,0.)
    scan=ray_distances(0,0,-math.pi+np.arange(360)*2*math.pi/360,boxes,15.)
    _,_,result=segment(render_camera((0,0,0),boxes),scan,-math.pi,2*math.pi/360,.05,15.,CameraGeometry(180,180,160,120,.6),True)
    assert result['clear'],result
