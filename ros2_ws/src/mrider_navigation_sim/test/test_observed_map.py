"""Do not retain dynamic ghosts, or invent clearance behind occlusions."""
import math
import numpy as np
import pytest
from mrider_navigation_sim.local_goal import scan_free_evidence
from mrider_navigation_sim.observed_map import reconcile


def evidence(ranges,max_clear=False):
    return scan_free_evidence((120,120),.1,(-6.,-6.),(0.,0.,0.),ranges,
                              -math.pi,2*math.pi/360,.05,15.,max_clear)


def test_three_confirmations_clear_old_obstacle_without_touching_raw_map():
    raw=np.full((120,120),-1);raw[60,70]=100;mask=evidence([4.]*360)
    assert reconcile(raw,[mask,mask])[60,70]==100
    output=reconcile(raw,[mask,mask,mask])
    assert output[60,70]==0 and output[60,110]==-1 and raw[60,70]==100


def test_present_wall_and_space_behind_it_are_never_cleared():
    raw=np.full((120,120),-1);raw[:,80]=100
    # A return at 2 m preserves the entire endpoint/behind-endpoint region.
    mask=evidence([2.]*360);output=reconcile(raw,[mask]*3)
    assert (output[:,80]==100).all() and output[60,90]==-1
    assert output[60,70]==0


def test_one_occluded_scan_vetoes_clearing():
    raw=np.zeros((120,120),dtype=int);raw[60,85]=100
    far=evidence([4.]*360);near=evidence([2.]*360)
    assert reconcile(raw,[far,near,far])[60,85]==100


@pytest.mark.parametrize('ranges',[[float('nan')]*360,[float('inf')]*360,[15.]*360,[0.]*360])
def test_invalid_or_unverified_no_hit_returns_cannot_erase_occupancy(ranges):
    raw=np.full((120,120),100);mask=evidence(ranges)
    assert np.array_equal(reconcile(raw,[mask]*3),raw)


def test_no_hit_contract_must_be_enabled_explicitly():
    raw=np.full((120,120),-1);mask=evidence([15.]*360,True)
    assert reconcile(raw,[mask]*3)[60,70]==0


def test_scan_stamp_and_frame_faults_invalidate_evidence():
    from collections import deque
    from types import SimpleNamespace
    from sensor_msgs.msg import LaserScan
    from mrider_navigation_sim.observed_map import ObservedMap
    node=SimpleNamespace(scans=deque(maxlen=5),cache={})
    scan=LaserScan();scan.header.frame_id='laser';scan.header.stamp.sec=1
    ObservedMap.scan_cb(node,scan);assert len(node.scans)==1
    ObservedMap.scan_cb(node,scan);assert not node.scans
    ObservedMap.scan_cb(node,scan)
    wrong=LaserScan();wrong.header.frame_id='wrong';wrong.header.stamp.sec=2
    ObservedMap.scan_cb(node,wrong);assert not node.scans


@pytest.mark.parametrize('latest,raw_stamp,tf_until,processing,publishes',[
    (9.8,9.9,10.,0.,True),  # two historical confirmations, latest input fresh
    (9.64,9.9,10.,0.,False),  # dropout cannot be hidden by historical evidence
    (10.1,9.9,11.,0.,False),  # future scan
    (9.8,7.,10.,0.,False),  # stale SLAM map
    (9.8,9.9,9.6,0.,False),  # only stale exact-time TF available
    (9.8,9.9,10.,.3,False),  # becoming stale DURING processing must not publish
])
def test_confirmation_history_preserves_current_input_freshness(latest,raw_stamp,tf_until,processing,publishes):
    from collections import deque
    from types import SimpleNamespace
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import TransformStamped
    from rclpy.time import Time
    from tf2_ros import TransformException
    from mrider_navigation_sim.observed_map import ObservedMap
    raw=OccupancyGrid();raw.header.frame_id='map';raw.header.stamp=Time(seconds=raw_stamp).to_msg()
    raw.info.width=raw.info.height=100;raw.info.resolution=.1
    raw.info.origin.position.x=raw.info.origin.position.y=-5.;raw.info.origin.orientation.w=1.
    raw.data=[-1]*10000
    scans=deque()
    for age in (.2,.1,0.):
        s=LaserScan();s.header.frame_id='laser';s.header.stamp=Time(seconds=latest-age).to_msg()
        s.angle_min=-math.pi;s.angle_increment=2*math.pi/360;s.range_min=.05;s.range_max=15.;s.ranges=[4.]*360
        scans.append(s)
    def lookup(parent,child,stamp):
        if stamp.nanoseconds>round(tf_until*1e9):raise TransformException('not yet available')
        tf=TransformStamped();tf.transform.rotation.w=1.;return tf
    sent=[]
    ticks=iter((10.,10.+processing))
    node=SimpleNamespace(raw=raw,scans=scans,metrics={},cache={},max_clear=False,
        get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=next(ticks))),
        tf_buffer=SimpleNamespace(lookup_transform=lookup),publisher=SimpleNamespace(publish=sent.append))
    ObservedMap.update_map(node)
    assert bool(sent)==publishes
    if publishes:
        assert sent[0].data[5050]==0 and sent[0].header.stamp==raw.header.stamp
        assert raw.data[5050]==-1


@pytest.mark.parametrize('yaw',[-3.1,0.,1.8,3.1])
def test_large_map_clearing_is_confined_to_observed_range(yaw):
    shape=(700,1000);resolution=.05;origin=(-4.,-4.);pose=(24.1,14.7,yaw)
    mask=scan_free_evidence(shape,resolution,origin,pose,[5.]*360,
                            -math.pi,2*math.pi/360,.05,15.)
    rows,cols=np.indices(shape)
    distance=np.hypot(origin[0]+(cols+.5)*resolution-pose[0],
                      origin[1]+(rows+.5)*resolution-pose[1])
    assert np.array_equal(mask,distance<5.-.15-resolution/math.sqrt(2))


@pytest.mark.parametrize('distance',[14.981,14.99,14.999])
def test_near_max_finite_returns_do_not_leave_unknown_wedges(distance):
    mask=evidence([distance]*360)  # max-range contract is deliberately OFF
    assert mask.all()  # this 12 m square is wholly before the 14.8 m margin
    # The real endpoint and beyond it must still stay unobserved.
    mask=scan_free_evidence((20,340),.1,(-1.,-1.),(0.,0.,0.),[distance]*360,
                            -math.pi,2*math.pi/360,.05,15.)
    assert mask[10,100] and not mask[10,160]
