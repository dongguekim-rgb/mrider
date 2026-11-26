import copy
import math
from pathlib import Path

import numpy as np
import pytest
import yaml
from mrider_navigation_sim.path_feasibility import validate_path, require_matching_resolution
from mrider_navigation_sim.local_goal import select_pose_goal


def arc(radius, sign=1):
    theta=np.linspace(0., 1.5, 91)
    return np.column_stack((radius*np.sin(theta),sign*radius*(1-np.cos(theta)),sign*theta))


def check(points, current=(0.,0.,0.)):
    return validate_path(points,current,.533,math.atan(.533/2.1),2.)


@pytest.mark.parametrize('sign',[-1,1])
def test_measured_vehicle_accepts_forward_turns_with_margin(sign):
    result=check(arc(3.,sign))
    assert result['max_path_curvature']==pytest.approx(1/3,abs=1e-8)
    assert abs(result['initial_tracking_curvature'])<result['allowed_curvature']


@pytest.mark.parametrize('sign',[-1,1])
def test_half_scale_planner_radius_is_rejected(sign):
    with pytest.raises(ValueError,match='path_curvature'):
        check(arc(1.75,sign))


def test_heading_error_can_make_geometric_path_untrackable():
    points=np.column_stack((np.linspace(0,5,80),np.zeros(80),np.zeros(80)))
    with pytest.raises(ValueError,match='initial_tracking_curvature'):
        check(points,(0.,0.,.8))


@pytest.mark.parametrize('bad',[[],[[0,0,0]],[[0,0,0],[float('nan'),0,0]],[[0,0,0],[1,0,0]]])
def test_invalid_or_sparse_paths_cannot_hide_a_corner(bad):
    with pytest.raises(ValueError):check(bad)


def test_rotation_reverse_and_disconnected_start_rejected():
    for points,current,reason in [
        ([[0,0,0],[0,0,1],[.1,0,1]],(0,0,0),'rotation_in_place'),
        ([[0,0,0],[-.1,0,0]],(0,0,0),'reverse_segment'),
        ([[0,0,0],[.1,0,0]],(1,0,0),'start_disconnected'),
    ]:
        with pytest.raises(ValueError,match=reason):check(points,current)


def test_duplicate_pose_without_rotation_is_harmless():
    points=arc(3.)
    assert check(np.vstack((points[0],points)))['samples']==len(points)


def test_sensor_costmaps_have_the_slam_resolution_before_configure():
    base=Path(__file__).resolve().parents[1]/'config'
    nav=yaml.safe_load((base/'nav2_sensor_only.yaml').read_text())
    resolution=yaml.safe_load((base/'slam_sensor_only.yaml').read_text())['slam_toolbox']['ros__parameters']['resolution']
    require_matching_resolution(nav,resolution)
    for name in ('global_costmap','local_costmap'):
        broken=copy.deepcopy(nav);broken[name][name]['ros__parameters']['resolution']=2*resolution
        with pytest.raises(ValueError,match='resolution_must_match_slam'):
            require_matching_resolution(broken,resolution)


@pytest.mark.parametrize('sign',[-1,1])
def test_lateral_goal_uses_a_known_free_vehicle_arc(sign):
    grid=np.zeros((800,1200),dtype=int);res=.05;origin=(-4.,-20.)
    # Wall blocks east of the north/south corridor, matching a generic junction.
    y0,y1=(4.,12.) if sign==1 else (-12.,-4.)
    grid[int((y0-origin[1])/res):int((y1-origin[1])/res),640:]=100
    result=select_pose_goal(grid,res,origin,(24.,0.,0.),(24.,16.*sign,sign*math.pi/2),3.,radius=1.1)
    assert result and not result[3]
    x,y,heading,_=result
    assert x>24 and sign*y>0 and sign*heading>.6
    # End pose lies on a forward circle of radius at least the planner radius.
    curvature=2*y/((x-24)**2+y*y)
    assert abs(curvature)<=1/3.15+1e-8
    distance=heading/curvature
    assert 0<distance<=6.
    # No occupied or unknown cell is accepted along the candidate arc.
    for s in np.linspace(0,distance,300):
        px=24+math.sin(curvature*s)/curvature;py=(1-math.cos(curvature*s))/curvature
        assert grid[int((py-origin[1])/res),int((px-origin[0])/res)]==0


def test_arc_goal_cannot_use_unknown_space():
    assert select_pose_goal(np.full((100,100),-1),.1,(-5,-5),(0,0,0),(0,10,math.pi/2),3.) is None


def test_final_approach_preserves_heading_before_a_short_last_leg():
    destination=(24.,16.,math.pi/2)
    result=select_pose_goal(np.zeros((600,700)),.05,(0.,0.),
                            (25.28,9.95,1.78),destination,3.)
    assert result==(*destination,True)


def test_blocked_final_goal_cannot_create_an_incompatible_approach_heading():
    grid=np.zeros((600,700));res=.05;origin=(0.,0.)
    grid[312:328,472:488]=100
    current=(25.56,9.93,1.82)
    chosen=select_pose_goal(grid,res,origin,current,(24.,16.,math.pi/2),3.)
    assert chosen and not chosen[3]
    assert math.dist(chosen[:2],(24.,16.))>=3.
    dx,dy=chosen[0]-current[0],chosen[1]-current[1]
    forward=math.cos(current[2])*dx+math.sin(current[2])*dy
    lateral=-math.sin(current[2])*dx+math.cos(current[2])*dy
    change=chosen[2]-current[2]
    # The ending heading must be the tangent of the same reachable arc,
    # not an independent bearing from the waypoint to the blocked goal.
    assert change==pytest.approx(2*math.atan2(lateral,forward),abs=1e-8)
    assert abs(2*lateral/(dx*dx+dy*dy))<=1/3.15+1e-8
