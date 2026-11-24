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
The link gate: /mitt/dbw/status >= 50 Hz, zero USB dropouts over 30 minutes.

    ros2 run mitt_bench dbw_link_monitor --ros-args -p duration_s:=1800.0 \
        -p out_dir:=docs/build/records

Listens only - it commands nothing, so it is safe in any mode, and it can run
beside teleop or the steer test. docs/build/04-firmware.md 4.3, "The 30-minute
stability run", and the final acceptance gate.

A dropout is a gap of more than 250 ms between status messages: that is when
the laptop calls the link lost (docs/design/safety.md 2, row 2). A session
restart on the Teensy (a "session up" line on /mitt/dbw/event) counts too.

Also recorded: the /imu/data rate (>= 100 Hz) and the firmware's own loop
rate from its 5-second /mitt/dbw/event line (>= 200 Hz, "실측").
"""

from mitt_msgs.msg import DbwStatus
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from . import analysis as an
from .common import STATUS_QOS, now, record_path, write_yaml


class LinkMonitor(Node):

    def __init__(self):
        super().__init__('dbw_link_monitor')
        self.out_dir = self.declare_parameter('out_dir', 'bench_records').value
        self.duration = float(self.declare_parameter('duration_s', 1800.0).value)
        self.create_subscription(DbwStatus, '/mitt/dbw/status', self.on_status, STATUS_QOS)
        self.create_subscription(Imu, '/imu/data', self.on_imu, STATUS_QOS)
        self.create_subscription(String, '/mitt/dbw/event', self.on_event, 10)
        self.t0 = now()
        self.status_t = []
        self.imu_t = []
        self.loops = []
        self.session_ups = 0
        self.events = 0
        self.done = False
        self.create_timer(60.0, self.on_minute)
        self.get_logger().info(f'Listening for {self.duration:.0f} s.')

    def on_status(self, _msg):
        self.status_t.append(now())

    def on_imu(self, _msg):
        self.imu_t.append(now())

    def on_event(self, msg: String):
        self.events += 1
        p = an.parse_loop_event(msg.data)
        if p:
            self.loops.append(p)
        if msg.data.startswith('session up'):
            self.session_ups += 1
            self.get_logger().warn(f'Teensy session (re)started: {msg.data}')

    def on_minute(self):
        t = now()
        n = sum(1 for x in self.status_t if x >= t - 60.0)
        g = an.gaps(self.status_t, an.DROPOUT_GAP_S)
        self.get_logger().info(
            f'{(t - self.t0) / 60:5.1f} min: status {n / 60.0:6.1f} Hz, dropouts so far {len(g)}')
        if t - self.t0 >= self.duration:
            self.finish(None)

    def finish(self, aborted):
        if self.done:
            return
        t_end = now()
        status_rates = an.rates_per_window(self.status_t, self.t0, t_end, 60.0)
        imu_rates = an.rates_per_window(self.imu_t, self.t0, t_end, 60.0)
        # Gaps are measured between messages, so the wait before the first
        # one - the agent connecting - is never counted as a dropout.
        g = an.gaps(self.status_t, an.DROPOUT_GAP_S)
        loop_hz = [x['hz'] for x in self.loops]
        overruns = sum(int(x['overruns']) for x in self.loops)
        status_min = min(status_rates) if status_rates else None
        imu_min = min(imu_rates) if imu_rates else None
        loop_min = min(loop_hz) if loop_hz else None
        dropouts = len(g) + max(0, self.session_ups - 1)

        result = {
            'tool': 'dbw_link_monitor',
            'aborted': aborted,
            'duration_s': round(t_end - self.t0, 1),
            'status': {
                'messages': len(self.status_t),
                'hz_min_per_minute': None if status_min is None else round(status_min, 1),
                'gaps_over_250ms': len(g),
                'longest_gap_s': round(max(g), 3) if g else 0.0,
            },
            'imu': {
                'messages': len(self.imu_t),
                'hz_min_per_minute': None if imu_min is None else round(imu_min, 1),
            },
            'firmware_loop': {
                'reports': len(self.loops),
                'hz_min': loop_min,
                'period_max_us': max((x['period_max_us'] for x in self.loops), default=None),
                'exec_max_us': max((x['exec_max_us'] for x in self.loops), default=None),
                'overruns_total': overruns,
            },
            'session_starts_seen': self.session_ups,
            'verdict': {
                'status_hz_min': {'limit': an.STATUS_HZ_MIN,
                                  'pass': None if status_min is None
                                  else bool(status_min >= an.STATUS_HZ_MIN)},
                'dropouts': {'value': dropouts, 'limit': 0, 'pass': dropouts == 0},
                'imu_hz_min': {'limit': an.IMU_HZ_MIN,
                               'pass': None if imu_min is None else bool(imu_min >= an.IMU_HZ_MIN)},
                'loop_hz_min': {'limit': an.LOOP_HZ_MIN,
                                'pass': None if loop_min is None else bool(loop_min >= an.LOOP_HZ_MIN)},
            },
        }
        path = record_path(self.out_dir, 'link', 'yaml')
        write_yaml(path, result)
        log = self.get_logger()
        for k, v in result['verdict'].items():
            log.info(f'{k:16s} {v}')
        log.info(f'record: {path}')
        self.done = True


def main():
    rclpy.init()
    node = LinkMonitor()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        node.finish('interrupted')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
