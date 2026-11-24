# Copyright 2026 Jaerock Kwon
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
The steering measurements of the numeric contract, in one scripted run.

    ros2 run mitt_bench dbw_steer_test --ros-args -p out_dir:=docs/build/records

WHEELS OFF THE GROUND. The vehicle must be armed in AUTONOMOUS: start this,
then move CH6 from KILL to AUTO - the command stream it publishes is what
lets the arm succeed. It never commands speed.

What it measures (docs/design/dbw.md 12, docs/build/04-firmware.md 4.4):
  steady-state error at -20..+20 deg        <= 1.0 deg
  10 deg steps: rise to 90 %, overshoot     <= 400 ms, <= 15 %
  a +/-20 deg triangle sweep, RMS error     <= 1.5 deg
  +/-30 deg commanded: clamped to 22.5 deg  ("한계 동작", Merge 1)
and, for Track S (sim vs real): steering range, steering rate limit, and
command latency to the setpoint echo and to motion.

Writes a CSV of every status sample and a YAML of the results. Commit both:
the team rule is that a claim without its record is not a measurement.
"""

import csv

from mitt_msgs.msg import DbwCommand, DbwStatus
import rclpy
from rclpy.node import Node

from . import analysis as an
from .common import DEG, MODE_NAMES, STATUS_QOS, now, record_path, write_yaml

ARM_TIMEOUT_S = 120.0
CMD_HZ = 50.0


def build_schedule(step_deg: float):
    """(kind, duration_s, target_deg). The order matters: each step starts from the last."""
    plan = [('hold', 2.0, 0.0)]
    for target in (step_deg, 0.0, -step_deg, 0.0):
        plan.append(('step', 3.0, target))
    for target in (-20.0, -10.0, -5.0, 5.0, 10.0, 20.0, 0.0):
        plan.append(('hold', 3.0, target))
    plan += [('hold', 3.0, 30.0), ('hold', 3.0, -30.0), ('hold', 2.0, 0.0),
             ('sweep', 20.0, 20.0), ('hold', 2.0, 0.0)]
    return plan


def triangle(t: float, period: float, amp: float) -> float:
    """0 -> +amp -> -amp -> 0 over one period."""
    x = (t / period) % 1.0
    if x < 0.25:
        return amp * 4 * x
    if x < 0.75:
        return amp * (2 - 4 * x)
    return amp * (4 * x - 4)


class SteerTest(Node):

    def __init__(self):
        super().__init__('dbw_steer_test')
        self.out_dir = self.declare_parameter('out_dir', 'bench_records').value
        self.step_deg = float(self.declare_parameter('step_deg', 10.0).value)
        self.pub = self.create_publisher(DbwCommand, '/mitt/dbw/command', 10)
        self.create_subscription(DbwStatus, '/mitt/dbw/status', self.on_status, STATUS_QOS)
        self.timer = self.create_timer(1.0 / CMD_HZ, self.on_timer)

        self.plan = build_schedule(self.step_deg)
        self.state = 'arming'
        self.t_wait = now()
        self.cmd_deg = 0.0
        self.mode = None
        self.samples = []
        self.segments = []
        self.idx = -1
        self.seg_t0 = 0.0
        self.prev_target = 0.0
        self.aborted = None
        self.done = False
        self.get_logger().info('Publishing 0 deg. Arm the vehicle: CH6 KILL -> AUTO.')

    # ------------------------------------------------------------------ io
    def on_status(self, msg: DbwStatus):
        self.mode = msg.mode
        if self.state == 'running':
            self.samples.append(an.Sample(
                t=now(), cmd=self.cmd_deg, sp=msg.steering_setpoint / DEG,
                meas=msg.steering_angle / DEG, mode=msg.mode, faults=msg.faults))
            if msg.mode != an.MODE_AUTONOMOUS and self.aborted is None:
                self.aborted = (f'vehicle left AUTONOMOUS (now {MODE_NAMES.get(msg.mode)}, '
                                f'faults 0x{msg.faults:04X}) during segment {self.idx}')

    def publish(self, deg: float):
        self.cmd_deg = deg
        m = DbwCommand()
        m.stamp = self.get_clock().now().to_msg()
        m.steering_angle = deg * DEG
        m.speed = 0.0
        self.pub.publish(m)

    # ------------------------------------------------------------------ schedule
    def start_segment(self, i: int, t: float):
        self.idx = i
        self.seg_t0 = t
        kind, dur, target = self.plan[i]
        self.segments.append(an.Segment(kind=kind, t0=t, t1=t + dur, target=target,
                                        start=self.prev_target))
        if kind != 'sweep':
            self.prev_target = target
        else:
            self.prev_target = 0.0

    def on_timer(self):
        t = now()
        if self.state == 'arming':
            self.publish(0.0)
            if self.mode == an.MODE_AUTONOMOUS:
                self.get_logger().info('AUTONOMOUS. Running the schedule (about 75 s).')
                self.state = 'running'
                self.start_segment(0, t)
            elif t - self.t_wait > ARM_TIMEOUT_S:
                self.aborted = 'never armed in AUTONOMOUS'
                self.finish()
            return
        if self.state != 'running':
            return
        if self.aborted:
            self.finish()
            return

        kind, dur, target = self.plan[self.idx]
        if t - self.seg_t0 >= dur:
            if self.idx + 1 >= len(self.plan):
                self.finish()
                return
            self.start_segment(self.idx + 1, t)
            kind, dur, target = self.plan[self.idx]
        if kind == 'sweep':
            self.publish(triangle(t - self.seg_t0, dur, target))
        else:
            self.publish(target)

    # ------------------------------------------------------------------ results
    def finish(self):
        if self.done:
            return
        self.state = 'done'
        self.timer.cancel()
        csv_path = record_path(self.out_dir, 'steer', 'csv')
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_s', 'cmd_deg', 'setpoint_deg', 'measured_deg', 'mode', 'faults'])
            t0 = self.samples[0].t if self.samples else 0.0
            for s in self.samples:
                w.writerow([f'{s.t - t0:.4f}', f'{s.cmd:.3f}', f'{s.sp:.3f}', f'{s.meas:.3f}',
                            s.mode, s.faults])

        result = an.analyze_steer(self.samples, self.segments) if self.samples else {}
        result = {'tool': 'dbw_steer_test', 'aborted': self.aborted,
                  'samples': len(self.samples), 'csv': csv_path, **result}
        yaml_path = record_path(self.out_dir, 'steer', 'yaml')
        write_yaml(yaml_path, result)

        log = self.get_logger()
        if self.aborted:
            log.error(f'ABORTED: {self.aborted}')
        for name, v in result.get('verdict', {}).items():
            log.info(f'{name:28s} {v}')
        log.info(f"for Track S: {result.get('for_simulation')}")
        log.info(f'records: {yaml_path}  {csv_path}')
        self.done = True


def main():
    rclpy.init()
    node = SteerTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.aborted = node.aborted or 'interrupted'
        node.finish()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
