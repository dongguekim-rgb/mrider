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
