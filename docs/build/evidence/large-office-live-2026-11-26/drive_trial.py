"""Observe a fresh, sensor-only large-office run; GT is evaluation only.

Run after the stack is ready in ROS_DOMAIN_ID=192. The only driving inputs
sent here are arm/start and subsequent operator-selected map goals. No
candidate velocity, model state, odometry, map or TF is published.
"""
import argparse
import json
import math
import os
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from mrider_autonomy.nodes import COMMAND_QOS, SENSOR_QOS
from mrider_navigation_sim.checks import Checks


parser = argparse.ArgumentParser()
parser.add_argument('--output', required=True)
parser.add_argument('--goals', default='[[24,0,0],[24,16,1.5707963267948966],[40,16,0]]')
args = parser.parse_args()
goals = json.loads(args.goals)
out = Path(args.output)
out.mkdir(parents=True, exist_ok=True)
rclpy.init()
node = Checks()
truth, estimators, local_goals, maps, map_stats, transforms = [], [], [], {}, [], []
started = time.monotonic()
report = dict(date='2026-11-26', scenario='large_office', vehicle=1,
              domain=int(os.environ['ROS_DOMAIN_ID']), goals=goals,
              ground_truth='evaluation only', rviz_restarted=False,
              command_or_fault_injection=False, requests=[], legs=[])


def elapsed():
    return time.monotonic() - started


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def yaw(q):
    return 2 * math.atan2(q.z, q.w)


def on_map(msg):
    maps['last'] = msg
    if not map_stats or elapsed() - map_stats[-1]['t'] >= 2.:
        known = sum(v >= 0 for v in msg.data)
        map_stats.append(dict(t=elapsed(), width=msg.info.width, height=msg.info.height,
                              resolution=msg.info.resolution, known_cells=known,
                              known_area_m2=known * msg.info.resolution ** 2))


def on_tf(msg):
    for tf in msg.transforms:
        if tf.header.frame_id in ('map', 'odom'):
            p = tf.transform.translation
            transforms.append([stamp(tf), tf.header.frame_id, tf.child_frame_id,
                               p.x, p.y, yaw(tf.transform.rotation)])


node.create_subscription(Odometry, '/evaluation/ground_truth', truth.append, SENSOR_QOS)
node.create_subscription(String, '/mrider_sim/estimator_status',
                         lambda m: estimators.append(json.loads(m.data)), 10)
node.create_subscription(PoseStamped, '/mrider_sim/local_goal',
                         lambda m: local_goals.append(dict(t=elapsed(), x=m.pose.position.x,
                         y=m.pose.position.y, yaw=yaw(m.pose.orientation))), COMMAND_QOS)
node.create_subscription(OccupancyGrid, '/map', on_map,
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
node.create_subscription(TFMessage, '/tf', on_tf, SENSOR_QOS)
goal_pub = node.create_publisher(PoseStamped, '/goal_pose', COMMAND_QOS)
base_service = node.service


def service(name, value=None):
    result = base_service(name, value)
    record = dict(t=elapsed(), service=name, success=result.success, message=result.message)
    report['requests'].append(record)
    print('SERVICE ' + json.dumps(record), flush=True)
    return result


node.service = service


def checkpoint():
    report.update(elapsed_s=elapsed(), final_mission=node.mission,
                  final_safety=node.states[-1][1] if node.states else None,
                  collision=node.collision,
                  min_clearance_m=min(node.clearances) if node.clearances else None,
                  local_goals=local_goals, map_stats=map_stats,
                  estimator=estimators, safety_history=node.states,
                  mission_history=node.mission_events, trajectory=node.trace,
                  evaluation_tf_trace=transforms,
                  evaluation_truth_trace=[[stamp(m), m.pose.pose.position.x,
                      m.pose.pose.position.y, yaw(m.pose.pose.orientation),
                      m.twist.twist.linear.x] for m in truth],
                  commands=[[m.drive.speed, m.drive.steering_angle] for m in node.commands])
    (out / 'drive.json').write_text(json.dumps(report))
    if 'last' in maps:
        m = maps['last']
        (out / 'map.json').write_text(json.dumps(dict(width=m.info.width, height=m.info.height,
            resolution=m.info.resolution, origin=[m.info.origin.position.x, m.info.origin.position.y],
            data=list(m.data))))


try:
    node.wait(lambda: truth and maps and node.odom is not None and estimators, 30.)
    node.assert_single_output()
    node.nav_ready()
    publishers = {t: [p.node_name for p in node.get_publishers_info_by_topic(t)] for t in
                  ('/map', '/mrider_sim/odometry', '/mrider_sim/command')}
    truth_subscribers = [p.node_name for p in node.get_subscriptions_info_by_topic('/evaluation/ground_truth')]
    assert publishers == {'/map': ['slam_toolbox'], '/mrider_sim/odometry': ['laser_odometry'],
                          '/mrider_sim/command': ['mrider_safety_gate']}, publishers
    assert truth_subscribers and set(truth_subscribers) == {'navigation_checks'}, truth_subscribers
    report['graph'] = dict(publishers=publishers, truth_subscribers=truth_subscribers)
    for index, (gx, gy, heading) in enumerate(goals):
        if index:
            assert node.mission == 'GOAL' and node.states[-1][1]['state'] == 'GOAL'
        t0, command_start, goal_start = elapsed(), len(node.commands), len(local_goals)
        node.arm()
        if index == 0:
            assert (gx, gy, heading) == (24, 0, 0), 'first goal must match launch goal'
            node.start()
        else:
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.y = float(gx), float(gy)
            msg.pose.orientation.z, msg.pose.orientation.w = math.sin(heading / 2), math.cos(heading / 2)
            event_start = len(node.mission_events)
            goal_pub.publish(msg)
            report['requests'].append(dict(t=elapsed(), topic='/goal_pose', goal=[gx, gy, heading]))
            node.wait(lambda: any(s in ('PLANNING', 'FOLLOWING') for s in node.mission_events[event_start:]), 5.)
        print('LEG_START ' + json.dumps(dict(index=index, goal=[gx, gy, heading])), flush=True)
        last, last_tick, stop_at, resumes = None, 0., None, []
        deadline = time.monotonic() + 200.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            state = node.states[-1][1]
            key = (node.mission, state['state'], state['reason'])
            if key != last or elapsed() - last_tick >= 5.:
                last, last_tick = key, elapsed()
                p = node.odom.pose.pose.position
                print('STATE ' + json.dumps(dict(t=elapsed(), leg=index, mission=node.mission,
                    safety=state['state'], reason=state['reason'], speed=state['speed'],
                    odom=[p.x, p.y], local_goal=local_goals[-1] if local_goals else None)), flush=True)
            if state['state'] == 'STOPPED':
                if state['reason'] not in ('camera_obstacle', 'obstacle_stop'):
                    raise AssertionError('nonrecoverable_stop: ' + state['reason'])
                if stop_at is None:
                    stop_at = elapsed()
            elif stop_at is not None and state['state'] in ('TRACKING', 'SLOW'):
                resumes.append(dict(stopped_at_s=stop_at, resumed_at_s=elapsed(), duration_s=elapsed()-stop_at))
                stop_at = None
            if node.collision:
                raise AssertionError('collision')
            if node.mission == 'GOAL':
                node.wait(lambda: node.states[-1][1]['state'] == 'GOAL'
                          and abs(truth[-1].twist.twist.linear.x) < .01, 5.)
                p, q = truth[-1].pose.pose.position, truth[-1].pose.pose.orientation
                error = math.hypot(p.x-gx, p.y-gy)
                yaw_error = abs(math.atan2(math.sin(yaw(q)-heading), math.cos(yaw(q)-heading)))
                steering = [m.drive.steering_angle for m in node.commands[command_start:] if m.drive.speed > .05]
                leg = dict(index=index, goal=[gx,gy,heading], elapsed_s=elapsed()-t0,
                           goal_error_m=error, yaw_error_rad=yaw_error,
                           local_goals=local_goals[goal_start:], obstacle_resumes=resumes,
                           left_steering_rad=max(steering) if steering else None,
                           right_steering_rad=min(steering) if steering else None,
                           evaluation_goal_pose=[p.x,p.y,yaw(q)], result='PASS' if error < .5 else 'FAIL')
                report['legs'].append(leg)
                print('LEG_RESULT ' + json.dumps(leg), flush=True)
                assert error < .5, leg
                checkpoint()
                break
            if node.mission == 'FAILED' and state['state'] == 'STOPPED' and stop_at is None:
                raise AssertionError('planning_or_tracking_failed')
        else:
            raise AssertionError('goal_timeout')
    report['result'] = 'PASS'
except Exception as error:
    report.update(result='FAIL', error=str(error), failed_goal_index=len(report['legs']))
    print('TRIAL_ERROR ' + str(error), flush=True)
    try:
        node.service('/mrider_sim/disarm')
        node.spin(.4)
    except Exception:
        pass
finally:
    checkpoint()
    keys = ('date','result','error','failed_goal_index','elapsed_s','collision','min_clearance_m',
            'legs','graph','requests','final_mission','final_safety')
    print('LIVE_RESULT ' + json.dumps({k:report[k] for k in keys if k in report}), flush=True)
    node.destroy_node()
    rclpy.shutdown()
raise SystemExit(0 if report['result'] == 'PASS' else 1)
