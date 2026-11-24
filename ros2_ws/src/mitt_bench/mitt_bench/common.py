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
"""What every bench tool shares: QoS, where records go, how they are written."""

import datetime
import math
import os
import time

from rclpy.qos import qos_profile_sensor_data
import yaml

# The firmware publishes status and IMU best effort (firmware/mitt_dbw).
# A reliable subscription would match nothing and hear nothing.
STATUS_QOS = qos_profile_sensor_data

MODE_NAMES = {0: 'ESTOP', 1: 'MANUAL_RC', 2: 'AUTONOMOUS'}
DEG = math.pi / 180.0


def now() -> float:
    return time.monotonic()


def vehicle() -> str:
    return os.environ.get('MITT_VEHICLE', 'unknown')


def record_path(out_dir: str, tool: str, ext: str) -> str:
    """bench_<tool>_v<N>_<YYYYmmdd_HHMMSS>.<ext> in out_dir, created if need be."""
    os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    return os.path.join(out_dir, f'bench_{tool}_v{vehicle()}_{stamp}.{ext}')


def write_yaml(path: str, data: dict) -> None:
    header = {
        'tool': data.pop('tool', None),
        'vehicle': vehicle(),
        'date': datetime.datetime.now().isoformat(timespec='seconds'),
    }
    with open(path, 'w') as f:
        yaml.safe_dump({**header, **data}, f, sort_keys=False, allow_unicode=True)
