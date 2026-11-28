"""ROS boundary: motion prediction adds stops, never clears sensor faults."""
import json
from unittest.mock import patch

import pytest
import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from mrider_navigation_sim.gate_node import SafetyNode


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID','220');rclpy.init();node=SafetyNode()
    node.require_motion=True
    try:yield node
    finally:node.destroy_node();rclpy.shutdown()


def ready(node,stamp,tracks):
    node.gate.scan=([],stamp,stamp)
    node.gate.odom=((0.,0.,0.,0.,0.),stamp,stamp)
    node.motion=(dict(frame='odom',stamp=stamp,valid=True,tracks=tracks),stamp)


def test_predicted_collision_stops_then_requires_clearance_dwell(node):
    track=dict(x=4.,y=0.,vx=-.5,vy=0.,radius=.5)
    ready(node,10.,[track]);node.gate.armed=True;node.gate.state='TRACKING'
    node.gate.cmd=((.35,0.),10.,10.)
    with patch('mrider_navigation_sim.gate_node.seconds',return_value=10.1), \
            patch('mrider_navigation_sim.gate_node.time.monotonic',return_value=10.1), \
            patch.object(node.pub,'publish') as command:
        node.tick()
        assert command.call_args.args[0].drive.speed==0.
        assert node.gate.reason=='obstacle_stop' and not node.gate.manual_fault
        assert not node.auto_arm(Trigger.Request(),Trigger.Response()).success
    ready(node,10.2,[])
    with patch('mrider_navigation_sim.gate_node.seconds',return_value=10.2), \
            patch('mrider_navigation_sim.gate_node.time.monotonic',return_value=10.2):
        assert not node.auto_arm(Trigger.Request(),Trigger.Response()).success
    ready(node,13.3,[])
    with patch('mrider_navigation_sim.gate_node.seconds',return_value=13.3), \
            patch('mrider_navigation_sim.gate_node.time.monotonic',return_value=13.3):
        assert node.auto_arm(Trigger.Request(),Trigger.Response()).success
        assert node.gate.state=='READY'


@pytest.mark.parametrize('bad',[
    None,
    dict(frame='map',stamp=10.,valid=True,tracks=[]),
    dict(frame='odom',stamp=8.,valid=True,tracks=[]),
    dict(frame='odom',stamp=11.,valid=True,tracks=[]),
])
def test_missing_or_bad_motion_input_latches_manual_fault(node,bad):
    ready(node,10.,[]);node.gate.armed=True;node.gate.state='TRACKING'
    node.gate.cmd=((.35,0.),10.,10.)
    with patch('mrider_navigation_sim.gate_node.seconds',return_value=10.1), \
            patch('mrider_navigation_sim.gate_node.time.monotonic',return_value=10.1), \
            patch.object(node.pub,'publish') as command:
        if bad is None:node.motion=None
        else:node.motion_cb(String(data=json.dumps(bad)))
        node.tick()
        assert command.call_args.args[0].drive.speed==0.
        assert node.gate.state=='STOPPED' and node.gate.manual_fault
        ready(node,10.1,[])
        assert not node.auto_arm(Trigger.Request(),Trigger.Response()).success
