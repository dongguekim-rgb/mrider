# Copyright 2026 Donggue Kim
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
Functional tests: a launch that must not start says so and exits non-zero.

A refused launch used to end with exit status 0, which a script or a CI job
reads as success. These run the real launch files with arguments they must
refuse and check the exit status, without starting Gazebo or a vehicle.
"""

import os
import subprocess

import pytest

TIMEOUT = 60


def _launch(package, launch_file, *args, env=None):
    return subprocess.run(
        ['ros2', 'launch', package, launch_file, *args],
        capture_output=True, text=True, timeout=TIMEOUT,
        env={**os.environ, **(env or {})})


@pytest.mark.parametrize('bad', ['7', 'two'])
def test_sim_refuses_unknown_vehicle(bad):
    """sim.launch.py vehicle:=<not a vehicle> exits 1 before starting anything."""
    result = _launch('mitt_bringup', 'sim.launch.py', f'vehicle:={bad}')
    output = result.stdout + result.stderr
    assert result.returncode != 0, output
    assert 'will not start' in output
    assert 'gz sim' not in output


def test_real_names_the_wrong_ros_domain():
    """real.launch.py vehicle:=1 from outside domain 11 starts nothing and says why."""
    result = _launch('mitt_bringup', 'real.launch.py', 'vehicle:=1',
                     env={'ROS_DOMAIN_ID': '0'})
    output = result.stdout + result.stderr
    assert 'NOT STARTING THE VEHICLE' in output
    assert 'ROS domain 11' in output
    assert 'ros2_control_node' not in output


@pytest.mark.xfail(
    strict=True,
    reason='Found 2026-11-28: real.launch.py prints its refusal and then exits '
           'with status 0, where sim.launch.py and nav2.launch.py exit 1. A '
           'script or CI job reads 0 as "the vehicle started". The launch file '
           'is shared between teams, so the fix belongs upstream; remove this '
           'mark when it lands (strict: the test fails the day it passes).')
def test_real_refusal_exits_nonzero():
    """A refused real.launch.py must not report success."""
    result = _launch('mitt_bringup', 'real.launch.py', 'vehicle:=1',
                     env={'ROS_DOMAIN_ID': '0'})
    assert result.returncode != 0, result.stdout + result.stderr


def test_nav2_refuses_unknown_vehicle():
    """nav2.launch.py vehicle:=<not a vehicle> exits 1 before starting Nav2."""
    result = _launch('mitt_navigation', 'nav2.launch.py', 'vehicle:=7')
    output = result.stdout + result.stderr
    assert result.returncode != 0, output
    assert 'lifecycle_manager' not in output
