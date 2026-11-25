import json
import math
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from mrider_autonomy.nodes import SENSOR_QOS
from mrider_navigation_sim.checks import Checks

rclpy.init()
node = Checks()
truth, estimator, requests = [], [], []
node.create_subscription(Odometry, '/evaluation/ground_truth', truth.append, SENSOR_QOS)
node.create_subscription(String, '/mrider_sim/estimator_status', lambda m: estimator.append(json.loads(m.data)), 10)
report = {'scenario': 'sensor_turns', 'domain': 192, 'vehicle': 1,
          'method': 'Manual launch with existing RViz process; arm/start services, no command/fault injection'}
base_service = node.service


def service(name, value=None):
    response = base_service(name, value)
    record = {'service': name, 'success': response.success, 'message': response.message}
    requests.append(record)
    print('SERVICE ' + json.dumps(record), flush=True)
    return response


node.service = service
started = time.monotonic()
try:
    node.wait(lambda: node.odom is not None and node.states and estimator, 20.)
    print('INITIAL ' + json.dumps({'mission': node.mission, 'safety': node.states[-1][1],
                                 'estimator': estimator[-1]}), flush=True)
    node.assert_single_output()
    node.nav_ready()
    node.arm()
    node.start()
    last, stops, resumed = None, [], False
    deadline = time.monotonic() + 150.
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.02)
        state = node.states[-1][1]
        key = (node.mission, state['state'], state['reason'])
        if key != last:
            last = key
            print('STATE ' + json.dumps({'elapsed_s': time.monotonic() - started,
                                        'mission': node.mission, 'safety': state,
                                        'estimated_pose': node.trace[-1] if node.trace else None}), flush=True)
            if state['state'] == 'STOPPED':
                stops.append({'reason': state['reason'], 'time': time.monotonic()})
                if state['reason'] not in ('camera_obstacle', 'obstacle_stop'):
                    raise AssertionError('nonrecoverable_stop: ' + state['reason'])
            if stops and state['state'] in ('TRACKING', 'SLOW'):
                resumed = True
        if node.mission == 'GOAL':
            node.wait(lambda: truth and abs(truth[-1].twist.twist.linear.x) < .01)
            p = truth[-1].pose.pose.position
            error = math.hypot(p.x - 12., p.y)
            steering = [m.drive.steering_angle for m in node.commands if m.drive.speed > .05]
            report.update(goal_error_m=error, collision=node.collision,
                          min_clearance_m=min(node.clearances),
                          left_steering_rad=max(steering), right_steering_rad=min(steering),
                          obstacle_stops=stops, automatic_resume=resumed)
            assert error < .5 and not node.collision, report
            assert min(steering) < -.04 and max(steering) > .04, report
            assert stops and resumed, report
            report['result'] = 'PASS'
            break
    else:
        raise AssertionError('goal_timeout')
except Exception as error:
    report.update(result='FAIL', error=str(error))
    try:
        node.service('/mrider_sim/disarm')
    except Exception:
        pass
finally:
    report.update(elapsed_s=time.monotonic() - started, requests=requests,
                  final_mission=node.mission, final_safety=node.states[-1][1] if node.states else None,
                  estimator=estimator, safety_history=node.states, mission_history=node.mission_events,
                  trajectory=node.trace, evaluation_truth_trace=[
                      [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                       m.pose.pose.position.x, m.pose.pose.position.y] for m in truth])
    Path('/tmp/mrider-live-trial.json').write_text(json.dumps(report))
    print('LIVE_RESULT ' + json.dumps({k: v for k, v in report.items() if k not in
          ('estimator', 'safety_history', 'mission_history', 'trajectory', 'evaluation_truth_trace')}), flush=True)
    node.destroy_node()
    rclpy.shutdown()
raise SystemExit(0 if report['result'] == 'PASS' else 1)
