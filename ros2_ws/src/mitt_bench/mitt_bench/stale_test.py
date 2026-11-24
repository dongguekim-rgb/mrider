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
Failsafe matrix row 1: the laptop stops commanding; the Teensy must notice.

    ros2 run mitt_bench dbw_stale_test --ros-args -p out_dir:=docs/build/records

WHEELS OFF. Start it, arm in AUTONOMOUS (CH6 KILL -> AUTO). It holds a 0 deg
command for 3 s, then stops publishing and times the vehicle's fall to ESTOP
with FAULT_SETPOINT_STALE set (docs/design/safety.md 2, row 1;
docs/build/04-firmware.md 4.4, "Setpoint staleness").

Expected: about 0.5 s - the firmware's threshold, plus one control tick, one
status period and the USB link. The verdict allows 0.6 s for those.
Watch the steering while it runs: centred, then de-energised.
"""

from mitt_msgs.msg import DbwCommand, DbwStatus
import rclpy
from rclpy.node import Node

from . import analysis as an
from .common import MODE_NAMES, STATUS_QOS, now, record_path, write_yaml

HOLD_S = 3.0
WAIT_S = 3.0
ARM_TIMEOUT_S = 120.0
LIMIT_S = 0.6


class StaleTest(Node):

    def __init__(self):
        super().__init__('dbw_stale_test')
        self.out_dir = self.declare_parameter('out_dir', 'bench_records').value
        self.pub = self.create_publisher(DbwCommand, '/mitt/dbw/command', 10)
        self.create_subscription(DbwStatus, '/mitt/dbw/status', self.on_status, STATUS_QOS)
        self.timer = self.create_timer(0.02, self.on_timer)
        self.state = 'arming'
        self.t_state = now()
        self.t_last_cmd = None
        self.t_estop = None
        self.faults_at_estop = None
        self.mode = None
        self.done = False
        self.get_logger().info('Publishing 0 deg. Arm the vehicle: CH6 KILL -> AUTO.')

    def on_status(self, msg: DbwStatus):
        self.mode = msg.mode
        if self.state == 'silent' and self.t_estop is None and msg.mode == an.MODE_ESTOP:
            self.t_estop = now()
            self.faults_at_estop = msg.faults

    def on_timer(self):
        t = now()
        if self.state == 'arming':
            self.send()
            if self.mode == an.MODE_AUTONOMOUS:
                self.state, self.t_state = 'holding', t
                self.get_logger().info('AUTONOMOUS. Holding 3 s, then going silent.')
            elif t - self.t_state > ARM_TIMEOUT_S:
                self.finish('never armed in AUTONOMOUS')
        elif self.state == 'holding':
            if self.mode != an.MODE_AUTONOMOUS:
                self.finish(f'left AUTONOMOUS before the test ({MODE_NAMES.get(self.mode)})')
                return
            if t - self.t_state < HOLD_S:
                self.send()
            else:
                self.state, self.t_state = 'silent', t
        elif self.state == 'silent':
            if self.t_estop is not None or t - self.t_state > WAIT_S:
                self.finish(None)

    def send(self):
        m = DbwCommand()
        m.stamp = self.get_clock().now().to_msg()
        m.steering_angle = 0.0
        m.speed = 0.0
        self.pub.publish(m)
        self.t_last_cmd = now()

    def finish(self, aborted):
        if self.done:
            return
        self.timer.cancel()
        dt = None if self.t_estop is None or self.t_last_cmd is None else self.t_estop - self.t_last_cmd
        stale_bit = (None if self.faults_at_estop is None
                     else bool(self.faults_at_estop & an.FAULT_SETPOINT_STALE))
        result = {
            'tool': 'dbw_stale_test',
            'aborted': aborted,
            'last_command_to_estop_s': None if dt is None else round(dt, 3),
            'fault_setpoint_stale_set': stale_bit,
            'faults_at_estop': self.faults_at_estop,
            'verdict': {
                'limit_s': LIMIT_S,
                'pass': (None if aborted else
                         bool(dt is not None and dt <= LIMIT_S and stale_bit)),
            },
        }
        path = record_path(self.out_dir, 'stale', 'yaml')
        write_yaml(path, result)
        log = self.get_logger()
        if aborted:
            log.error(f'ABORTED: {aborted}')
        elif dt is None:
            log.error(f'FAIL: no ESTOP within {WAIT_S} s of the last command')
        else:
            log.info(f'last command -> ESTOP: {dt:.3f} s, stale bit: {stale_bit} '
                     f"-> {'PASS' if result['verdict']['pass'] else 'FAIL'}")
        log.info(f'record: {path}')
        self.done = True


def main():
    rclpy.init()
    node = StaleTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        node.finish('interrupted')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
