import math
from pathlib import Path
import pytest
import yaml
from mrider_navigation_sim.safety import Gate, Limits
from mrider_navigation_sim.geometry import ray_distance, clearance, swept_hit


def feed(g,t=10.,wall=None):
    wall=t if wall is None else wall
    assert g.scan_input([10.]*360,-math.pi,2*math.pi/360,.05,15.,'laser',t,t,wall)
    assert g.odom_input(0.,0.,0.,0.,0.,'odom','base_link',t,t,wall)


def active():
    g=Gate();feed(g);assert g.arm(10.,10.)[0];assert g.command_input(.4,0.,10.,10.)
    assert g.output(10.,10.)[0]>.0
    return g


def test_disarmed():
    g=Gate();feed(g);g.command_input(.4,0.,10.,10.);assert g.output(10.,10.)==(0.,0.)


def test_requires_sensors_to_arm():
    assert not Gate().arm(10.,10.)[0]


@pytest.mark.parametrize('key',['scan','odom','cmd'])
def test_independent_timeout_latches(key):
    g=active();feed(g,10.5);g.command_input(.4,0.,10.5,10.5)
    setattr(g,key,None)
    assert g.output(10.5,10.5)==(0.,0.);assert g.state=='STOPPED'
    feed(g,11.);g.command_input(.4,0.,11.,11.)
    assert g.output(11.,11.)==(0.,0.)
    assert g.arm(11.,11.)[0];g.command_input(.4,0.,11.,11.);assert g.output(11.,11.)[0]>0.


@pytest.mark.parametrize('kind',['nan','inf','empty','partial','frame','old','future','repeat','negative_range'])
def test_invalid_scan(kind):
    g=active();ranges=[10.]*360;frame='laser';stamp=10.1
    if kind=='nan':ranges[0]=math.nan
    if kind=='inf':ranges[0]=math.inf
    if kind=='empty':ranges=[]
    if kind=='partial':ranges=ranges[:100]
    if kind=='frame':frame='wrong'
    if kind=='old':stamp=9.
    if kind=='future':stamp=12.
    if kind=='repeat':stamp=10.
    if kind=='negative_range':ranges[0]=-1.
    assert not g.scan_input(ranges,-math.pi,2*math.pi/360,.05,15.,frame,stamp,10.1,10.1)
    assert g.output(10.1,10.1)==(0.,0.) and g.latched


@pytest.mark.parametrize('v,w',[(math.nan,0.),(.4,math.inf),(-.1,0.),(1.,0.),(0.,1.),(.1,1.)])
def test_invalid_command(v,w):
    g=active();assert not g.command_input(v,w,10.1,10.1);assert g.latched


def test_pause_clock():
    g=active();assert g.output(10.,10.5)==(0.,0.) and g.latched


def test_rewind_requires_rearm():
    g=active();assert g.output(5.,11.)==(0.,0.);assert g.latched
    feed(g,5.1,11.1);g.command_input(.4,0.,5.1,11.1)
    assert g.output(5.1,11.1)==(0.,0.);assert g.arm(5.1,11.1)[0]


def test_obstacle_latch_and_blocked_arm():
    g=active();g.scan=([(1.,0.)],10.,10.)
    assert g.output(10.,10.)==(0.,0.) and g.reason=='obstacle_stop'
    assert not g.arm(10.,10.)[0]


def test_slowdown():
    g=active();g.scan=([(1.6,0.)],10.,10.)
    v,s=g.output(10.,10.);assert 0<v<.4 and g.state=='SLOW'


def test_curved_swept_footprint():
    args=(1.,.9,.2,.3,.15)
    assert swept_hit([(1.3,.7)],0.,*args)==math.inf
    assert swept_hit([(1.3,.7)],.6,*args)<math.inf


def test_actual_speed_sets_stopping_distance():
    g=active();g.odom=((0.,0.,0.,.5,0.),10.,10.);g.command_input(.1,0.,10.,10.)
    g.scan=([(1.3,0.)],10.,10.)
    assert g.output(10.,10.)==(0.,0.)


def test_no_stationary_turn():
    g=active();assert not g.command_input(0.,.2,10.,10.)


def test_ray_and_collision_geometry():
    boxes=[(2.,-.5,3.,.5)]
    assert ray_distance(0.,0.,0.,boxes)==2.
    assert clearance((0.,0.,0.),boxes)==pytest.approx(1.1)
    assert clearance((1.5,0.,0.),boxes)==0.


def test_config_consistency():
    p=Path(__file__).resolve().parents[1]/'config/nav2.yaml';cfg=yaml.safe_load(p.read_text());c=Limits(**yaml.safe_load((p.parent/'simulation.yaml').read_text())['/**']['ros__parameters'])
    planner=cfg['planner_server']['ros__parameters']['GridBased']
    assert planner['minimum_turning_radius']>=c.wheelbase/math.tan(c.max_steering)
    assert planner['motion_model_for_search']=='DUBIN'
    controller=cfg['controller_server']['ros__parameters']['FollowPath']
    assert controller['desired_linear_vel']<=c.max_speed
    assert not controller['allow_reversing'] and not controller['use_rotate_to_heading']
    for key in ('local_costmap','global_costmap'):
        fp=yaml.safe_load(cfg[key][key]['ros__parameters']['footprint'])
        assert max(v[0] for v in fp)==c.front and -min(v[0] for v in fp)==c.rear
        assert max(v[1] for v in fp)==c.half_width


def test_goal_cannot_clear_fault():
    g=active();g.fault("sensor_timeout");g.goal();assert g.state=="STOPPED" and g.latched


@pytest.mark.parametrize('terminal',['FAILED','GOAL'])
def test_mission_heartbeat_does_not_cancel_manual_rearm(terminal):
    g=active();g.mission_state('FOLLOWING');g.mission_state(terminal)
    assert g.state==('STOPPED' if terminal=='FAILED' else 'GOAL')
    feed(g,10.1);assert g.arm(10.1,10.1)[0]
    g.mission_state(terminal);assert g.state=='READY'
    g.mission_state('PLANNING');g.mission_state('FAILED')
    assert g.state=='STOPPED'


def test_auto_resume_requires_clear_dwell_and_never_clears_sensor_fault():
    g=active();g.fault('obstacle_stop')
    for t in [11.,12.,13.]:
        feed(g,t);assert not g.auto_arm(t,t)[0]
    feed(g,14.);assert g.auto_arm(14.,14.)[0]
    g.fault('invalid_scan')
    for t in [15.,20.,25.]:
        feed(g,t);assert not g.auto_arm(t,t)[0]
    assert g.state=='STOPPED'


def test_sensor_dropout_during_obstacle_wait_requires_manual_rearm():
    g=active();g.fault('obstacle_stop');feed(g,11.)
    assert not g.auto_arm(11.,11.)[0]
    assert g.output(12.,12.)==(0.,0.)
    assert g.reason=='sensor_timeout'
    for t in [13.,17.,21.]:
        feed(g,t);assert not g.auto_arm(t,t)[0]


def test_obstruction_resets_dwell_and_disarm_cannot_auto_resume():
    g=active();g.fault('obstacle_stop');feed(g,11.)
    assert not g.auto_arm(11.,11.)[0]
    feed(g,13.)
    g.scan=([(1.,0.)],13.,13.)
    assert not g.auto_arm(13.,13.)[0]
    feed(g,14.);assert not g.auto_arm(14.,14.)[0]
    feed(g,16.);assert not g.auto_arm(16.,16.)[0]
    g.disarm();feed(g,20.)
    assert not g.auto_arm(20.,20.)[0] and not g.armed
