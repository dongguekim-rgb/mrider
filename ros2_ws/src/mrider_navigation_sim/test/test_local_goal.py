import math
import numpy as np
import pytest
from mrider_navigation_sim.local_goal import select_goal


def test_far_unknown_goal_becomes_bounded_known_goal():
    grid=np.zeros((100,100),dtype=int)
    result=select_goal(grid,.2,(0,0),(3,5),(40,5))
    assert result and not result[2]
    assert 1.<math.dist((3,5),result[:2])<=6.
    assert result[0]<20


def test_unknown_and_occupied_barriers_not_crossed():
    for value in [-1,100]:
        grid=np.zeros((100,100),dtype=int);grid[:,25:30]=value
        result=select_goal(grid,.2,(0,0),(2,5),(8,5))
        assert result is None or result[0]<5


def test_near_known_final_goal_preserved():
    result=select_goal(np.zeros((100,100)),.2,(0,0),(3,5),(7,5))
    assert result==(7,5,True)


def test_clear_goal_center_but_blocked_final_body_waits_without_detour():
    from mrider_navigation_sim.local_goal import select_pose_goal
    grid=np.zeros((200,200));grid[98:103,171:175]=100
    info={};args=(grid,.05,(-5.,-5.),(0.,0.,0.),(3.,0.,0.),2.5)
    result=select_pose_goal(*args,footprint=(.76,.23,.28,.15),diagnostics=info)
    assert info['final_check']['reason']=='final'
    assert result is None and info['reason']=='final_footprint_blocked'
    grid[:]=0
    assert select_pose_goal(*args,footprint=(.76,.23,.28,.15))==(3.,0.,0.,True)


@pytest.mark.parametrize('current,destination,resolution',[
    ((3.,5.),(9.1,5.),.2),
    ((17.9,.02),(24.,0.),.05),
    ((17.86,-.015),(24.,0.),.05),
])
def test_intermediate_goal_preserves_a_full_final_approach(current,destination,resolution):
    result=select_goal(np.zeros((140,650)),resolution,(-1.,-2.),current,destination)
    assert result and not result[2]
    assert math.dist(current,result[:2])<=6.
    assert math.dist(result[:2],destination)>=1.
    # Grid offset must not create a nearly perpendicular terminal heading.
    assert abs(math.atan2(destination[1]-result[1],destination[0]-result[0]))<.25


def test_unknown_start_and_unknown_map_edge_are_rejected():
    assert select_goal(np.full((100,100),-1),.2,(0,0),(3,5),(7,5)) is None
    assert select_goal(np.zeros((100,100)),.2,(0,0),(.1,.1),(7,5)) is None


def test_scan_fills_only_observed_unknown_and_never_erases_wall():
    from mrider_navigation_sim.local_goal import add_scan_evidence
    grid=np.full((100,100),-1);grid[50,55]=100
    args=(.2,(-10,-10),(0,0,0),[5.]*360,-math.pi,2*math.pi/360,.05,15.)
    result=add_scan_evidence(grid,*args)
    assert result[50,50]==0 and result[50,55]==100 and result[50,90]==-1
    result=add_scan_evidence(grid,.2,(-10,-10),(0,0,0),[float('nan')]*360,-math.pi,2*math.pi/360,.05,15.,True)
    assert np.array_equal(result,grid)


def test_scan_max_requires_explicit_verified_contract():
    from mrider_navigation_sim.local_goal import add_scan_evidence
    grid=np.full((100,100),-1)
    args=(.2,(-10,-10),(0,0,0),[15.]*360,-math.pi,2*math.pi/360,.05,15.)
    assert np.all(add_scan_evidence(grid,*args)==-1)
    assert add_scan_evidence(grid,*args,True)[50,50]==0


def test_full_scan_wrap_has_no_unobserved_seam_inside_free_circle():
    from mrider_navigation_sim.local_goal import add_scan_evidence
    result=add_scan_evidence(np.full((100,100),-1),.1,(-5,-5),(0,0,0),
                             [5.]*360,-math.pi,2*math.pi/360,.05,15.)
    rows,cols=np.indices(result.shape)
    interior=np.hypot(-5+(cols+.5)*.1,-5+(rows+.5)*.1)<4.75
    assert (result[interior]==0).all()


def test_oriented_body_can_progress_where_enclosing_circle_cannot():
    from mrider_navigation_sim.local_goal import select_pose_goal
    grid=np.zeros((500,700),dtype=int);grid[110:360,640:]=100
    args=(grid,.05,(-4.,-2.),(27.05,3.64,1.63),(24.,16.,math.pi/2),3.)
    assert select_pose_goal(*args,radius=.96) is None
    result=select_pose_goal(*args,radius=.96,footprint=(.733,.2,.28,.15))
    assert result is not None and result[1]>3.64


def test_oriented_check_covers_interior_and_unknown_cells():
    from scipy.ndimage import distance_transform_edt
    from mrider_navigation_sim.local_goal import footprint_free
    grid=np.ones((100,100),dtype=bool)
    grid[52,54]=False  # strictly inside, not just on the rectangle boundary
    clear=distance_transform_edt(np.pad(grid,1))[1:-1,1:-1]*.1
    assert not footprint_free(clear,.1,(-5.,-5.),[(0.,0.,0.)],(.9,.2,.3,.15))[0]
    assert not footprint_free(clear,.1,(-5.,-5.),[(4.9,0.,0.)],(.9,.2,.3,.15))[0]


@pytest.mark.parametrize('sign',[-1,1])
def test_quarter_turn_subgoal_faces_remaining_destination(sign):
    from mrider_navigation_sim.local_goal import select_pose_goal
    goal=(0.,sign*16.,sign*math.pi/2)
    result=select_pose_goal(np.zeros((500,300)),.1,(-10.,-25.),(0.,0.,0.),goal,
                            2.5,footprint=(.733,.2,.28,.15))
    assert result is not None and not result[3]
    x,y,yaw,_=result
    bearing=math.atan2(goal[1]-y,goal[0]-x)
    mismatch=abs(math.atan2(math.sin(bearing-yaw),math.cos(bearing-yaw)))
    # The old distance-dominated score over-rotated by ~0.54 rad here.
    assert mismatch<.15 and sign*yaw>1.2
