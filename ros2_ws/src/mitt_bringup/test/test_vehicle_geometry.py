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
Functional tests: every consumer of a vehicle's geometry agrees with its overlay.

The twin of vehicle N is three files that each carry the geometry:

  1. the URDF            (xacro vehicle:=N)
  2. the controllers     (vehicle_config.controllers_for)
  3. the Nav2 parameters (vehicle_config.nav2_for)

Nothing checked that the three agree with config/vehicles/vehicle<N>.yaml, or
with each other. mitt_controllers.yaml says "mitt_bringup checks the two agree
at launch"; no such check existed. These tests are that check.

They read the expected values from the YAML files rather than repeating them,
so a team that measures a new value does not have to edit a test.
"""

import importlib.util
import math
import os
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml

DESC = get_package_share_directory('mitt_description')
XACRO = os.path.join(DESC, 'urdf', 'mitt.urdf.xacro')
TOL = 1e-6


def _vehicle_config():
    """Import config/vehicle_config.py the way the launch files do."""
    path = os.path.join(DESC, 'config', 'vehicle_config.py')
    spec = importlib.util.spec_from_file_location('vehicle_config', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


VC = _vehicle_config()
ALL_VEHICLES = (0,) + tuple(VC.VEHICLES)


def _dims(vehicle):
    """Return the estimates with vehicle N's measured values laid over them."""
    with open(os.path.join(DESC, 'config', 'mitt_dimensions.yaml')) as fh:
        base = yaml.safe_load(fh)
    return {**base, **VC.load_overlay(DESC, vehicle)}


def _urdf(vehicle):
    out = subprocess.run(
        ['xacro', XACRO, f'vehicle:={vehicle}'],
        check=True, capture_output=True, text=True).stdout
    return ET.fromstring(out)


def _joint(robot, name):
    joint = robot.find(f"joint[@name='{name}']")
    assert joint is not None, f'URDF has no joint {name}'
    return joint


def _xyz(joint):
    return [float(v) for v in joint.find('origin').get('xyz').split()]


def _wheel_radius(robot, link):
    cyl = robot.find(f"link[@name='{link}']/collision/geometry/cylinder")
    if cyl is None:
        cyl = robot.find(f"link[@name='{link}']/visual/geometry/cylinder")
    assert cyl is not None, f'{link} has no cylinder geometry'
    return float(cyl.get('radius'))


@pytest.mark.parametrize('vehicle', ALL_VEHICLES)
def test_urdf_expands(vehicle):
    """The description expands for the estimates and for every vehicle."""
    robot = _urdf(vehicle)
    assert robot.find("link[@name='base_link']") is not None
    assert robot.find("link[@name='laser_link']") is not None


@pytest.mark.parametrize('vehicle', ALL_VEHICLES)
def test_urdf_wheelbase_matches_overlay(vehicle):
    """Front axle sits one wheelbase ahead of the rear axle (base_link)."""
    dims, robot = _dims(vehicle), _urdf(vehicle)
    for side in ('left', 'right'):
        x = _xyz(_joint(robot, f'front_{side}_steer_joint'))[0]
        assert x == pytest.approx(dims['wheelbase'], abs=TOL)
        assert _xyz(_joint(robot, f'rear_{side}_wheel_joint'))[0] == pytest.approx(0.0, abs=TOL)


@pytest.mark.parametrize('vehicle', ALL_VEHICLES)
def test_urdf_track_matches_overlay(vehicle):
    """Left and right wheels are one track width apart, front and rear."""
    dims, robot = _dims(vehicle), _urdf(vehicle)
    for left, right in (('front_left_steer_joint', 'front_right_steer_joint'),
                        ('rear_left_wheel_joint', 'rear_right_wheel_joint')):
        track = _xyz(_joint(robot, left))[1] - _xyz(_joint(robot, right))[1]
        assert track == pytest.approx(dims['track_width'], abs=TOL)


@pytest.mark.parametrize('vehicle', ALL_VEHICLES)
def test_urdf_wheel_radius_matches_overlay(vehicle):
    """All four wheels have the overlay's radius, and the axles sit at it."""
    dims, robot = _dims(vehicle), _urdf(vehicle)
    for wheel in ('front_left', 'front_right', 'rear_left', 'rear_right'):
        radius = _wheel_radius(robot, f'{wheel}_wheel_link')
        assert radius == pytest.approx(dims['wheel_radius'], abs=TOL)
    axle_z = _xyz(_joint(robot, 'rear_left_wheel_joint'))[2]
    assert axle_z == pytest.approx(dims['wheel_radius'], abs=TOL)


@pytest.mark.parametrize('vehicle', ALL_VEHICLES)
def test_urdf_steering_limit_and_rate(vehicle):
    """A steering joint cannot be commanded past steer_limit_deg."""
    dims, robot = _dims(vehicle), _urdf(vehicle)
    limit = math.radians(dims['steer_limit_deg'])
    for side in ('left', 'right'):
        lim = _joint(robot, f'front_{side}_steer_joint').find('limit')
        assert float(lim.get('upper')) == pytest.approx(limit, abs=TOL)
        assert float(lim.get('lower')) == pytest.approx(-limit, abs=TOL)
        assert float(lim.get('velocity')) == pytest.approx(dims['max_steer_rate'], abs=TOL)


@pytest.mark.parametrize('vehicle', ALL_VEHICLES)
def test_controller_geometry_matches_urdf(vehicle):
    """
    Give the Ackermann controller the same geometry as the URDF.

    This is the check mitt_controllers.yaml says exists at launch.
    """
    stock = os.path.join(
        get_package_share_directory('mitt_control'), 'config', 'mitt_controllers.yaml')
    with open(VC.controllers_for(vehicle, DESC, stock)) as fh:
        params = yaml.safe_load(fh)['ackermann_steering_controller']['ros__parameters']
    robot = _urdf(vehicle)
    wheelbase = _xyz(_joint(robot, 'front_left_steer_joint'))[0]
    front_track = (_xyz(_joint(robot, 'front_left_steer_joint'))[1]
                   - _xyz(_joint(robot, 'front_right_steer_joint'))[1])
    rear_track = (_xyz(_joint(robot, 'rear_left_wheel_joint'))[1]
                  - _xyz(_joint(robot, 'rear_right_wheel_joint'))[1])
    assert params['wheelbase'] == pytest.approx(wheelbase, abs=TOL)
    assert params['front_wheel_track'] == pytest.approx(front_track, abs=TOL)
    assert params['rear_wheel_track'] == pytest.approx(rear_track, abs=TOL)
    assert params['front_wheels_radius'] == pytest.approx(
        _wheel_radius(robot, 'front_left_wheel_link'), abs=TOL)
    assert params['rear_wheels_radius'] == pytest.approx(
        _wheel_radius(robot, 'rear_left_wheel_link'), abs=TOL)


def _find(node, key, found=None):
    """Collect every value stored under `key`, at any depth of a YAML tree."""
    found = [] if found is None else found
    if isinstance(node, dict):
        for k, v in node.items():
            if k == key:
                found.append(v)
            _find(v, key, found)
    elif isinstance(node, list):
        for v in node:
            _find(v, key, found)
    return found


@pytest.mark.parametrize('vehicle', VC.VEHICLES)
def test_nav2_plans_above_the_measured_turning_radius(vehicle):
    """Nav2 plans 5% above a MEASURED turning radius, never below it."""
    over = VC.load_overlay(DESC, vehicle)
    if VC.TURNING_RADIUS not in over:
        pytest.skip(f'vehicle {vehicle} has not measured {VC.TURNING_RADIUS} yet')
    stock = os.path.join(
        get_package_share_directory('mitt_navigation'), 'config', 'nav2_params.yaml')
    with open(VC.nav2_for(vehicle, DESC, stock, real=True)) as fh:
        radii = _find(yaml.safe_load(fh), VC.PLANNER_RADIUS)
    assert len(radii) == 1
    measured = float(over[VC.TURNING_RADIUS])
    assert radii[0] >= measured
    assert radii[0] == pytest.approx(measured * VC.PLANNER_MARGIN, abs=0.01)


@pytest.mark.parametrize('vehicle', VC.VEHICLES)
def test_nav2_inflation_reaches_the_footprint(vehicle):
    """
    Cover the farthest corner of the footprint with the inflation radius.

    Below that, the planner stops collision-checking the corners and plans
    paths the controller then refuses (software.md 4.5).
    """
    stock = os.path.join(
        get_package_share_directory('mitt_navigation'), 'config', 'nav2_params.yaml')
    with open(VC.nav2_for(vehicle, DESC, stock)) as fh:
        params = yaml.safe_load(fh)
    footprints = _find(params, 'footprint')
    inflations = _find(params, VC.INFLATION_RADIUS)
    assert len(footprints) == 2 and len(inflations) == 2
    for footprint, inflation in zip(footprints, inflations):
        reach = max(math.hypot(x, y) for x, y in yaml.safe_load(footprint))
        assert inflation >= reach


def test_real_vehicle_without_turning_radius_is_refused():
    """Nav2 on a real vehicle is refused until its turning radius is measured."""
    stock = os.path.join(
        get_package_share_directory('mitt_navigation'), 'config', 'nav2_params.yaml')
    unmeasured = [v for v in VC.VEHICLES
                  if VC.TURNING_RADIUS not in VC.load_overlay(DESC, v)]
    if not unmeasured:
        pytest.skip('every vehicle has a measured turning radius')
    with pytest.raises(ValueError):
        VC.nav2_for(unmeasured[0], DESC, stock, real=True)


@pytest.mark.parametrize('bad', ['7', '-1', 'two', ''])
def test_unknown_vehicle_number_is_refused(bad):
    """A vehicle number that does not exist is an error, not vehicle 0."""
    with pytest.raises(ValueError):
        VC.parse_vehicle(bad)
