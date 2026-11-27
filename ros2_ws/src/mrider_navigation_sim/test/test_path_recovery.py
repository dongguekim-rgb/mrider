import math
import numpy as np
import pytest

from mrider_navigation_sim.path_recovery import validated_tail


def route():
    return np.column_stack((np.arange(0.,7.,.05),np.zeros(140),np.zeros(140)))


def check(points=None,current=(3.,0.,0.),grid=None,footprint=(.76,.23,.28,.15),goal=None):
    return validated_tail(route() if points is None else points,current,
        np.zeros((100,200),dtype=int) if grid is None else grid,.05,(-1.,-2.5),
        footprint,.533,math.atan(.533/2.1),2.,goal)


def test_rejoins_remaining_reference_without_replaying_past_route():
    index,metrics=check()
    assert index==60 and metrics['length_m']<4.1
    assert metrics['max_path_steering']<1e-6


@pytest.mark.parametrize('occupied',[100,-1])
def test_new_obstacle_or_unknown_cell_blocks_reuse(occupied):
    grid=np.zeros((100,200),dtype=int);grid[50,120]=occupied
    with pytest.raises(ValueError,match='not_observed_free'):check(grid=grid)


def test_present_car_body_must_be_clear_even_when_reference_is_clear():
    grid=np.zeros((100,200),dtype=int);grid[40,80]=100
    with pytest.raises(ValueError,match='not_observed_free'):check(current=(3.,-.1,0.),grid=grid)


@pytest.mark.parametrize('current',[(3.,.5,0.),(3.,0.,math.pi)])
def test_disconnected_or_wrong_direction_start_is_rejected(current):
    with pytest.raises(ValueError):check(current=current)


def test_tight_curve_cannot_be_reused():
    a=np.linspace(0.,1.2,100);points=np.column_stack((1.5*np.sin(a),1.5*(1-np.cos(a)),a))
    with pytest.raises(ValueError,match='path_curvature'):check(points=points,current=(0.,0.,0.))


def test_existing_controller_margin_is_required_even_inside_planning_reserve():
    # A corridor may allow continuation with the configured 0.15 m tracking
    # margin but be narrower than the planner's extra 0.40 m route reserve.
    grid=np.zeros((100,200),dtype=int);grid[:36,:]=100;grid[64:,:]=100
    check(grid=grid)
    with pytest.raises(ValueError,match='not_observed_free'):
        check(grid=grid,footprint=(.76,.23,.28,.40))


def test_virtual_reference_is_not_a_driven_extension_beyond_original_goal():
    grid=np.zeros((100,200),dtype=int);grid[:,145:]=100  # wall at x=6.25
    with pytest.raises(ValueError,match='not_observed_free'):check(grid=grid)
    _,metrics=check(grid=grid,goal=(5.,0.,0.))
    assert metrics['physical_length_m']<2.1
    # A wall inside the actual stopping footprint still prevents resuming.
    grid[:,136:]=100  # x=5.80, within the real goal's front + margin
    with pytest.raises(ValueError,match='not_observed_free'):check(grid=grid,goal=(5.,0.,0.))


def test_original_goal_must_lie_on_the_retained_reference():
    with pytest.raises(ValueError,match='original_goal_mismatch'):check(goal=(5.,2.,0.))


def test_sparse_reference_uses_nearest_sample_without_false_disconnection():
    # Recorded stop: the nearest sample is 0.122 m away, but its predecessor
    # is 0.372 m away. The live controller would prune that predecessor.
    yaw=math.pi/12
    x=np.arange(3.524413522,7.,.3)
    points=np.column_stack((x,1.030758277+(x-x[0])*math.tan(yaw),np.full(len(x),yaw)))
    index,metrics=check(points=points,current=(3.895903786,1.015321996,.252244047),
                        grid=np.zeros((160,200),dtype=int))
    assert index==1 and metrics['max_path_curvature']<1e-6
