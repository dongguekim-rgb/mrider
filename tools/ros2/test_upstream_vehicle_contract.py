#!/usr/bin/env python3
"""Check installed official launch contracts without starting hardware nodes."""
import importlib.util
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext

DESC = Path(get_package_share_directory('mitt_description'))
NAV = Path(get_package_share_directory('mitt_navigation'))
STOCK = NAV / 'config/nav2_params.yaml'
spec = importlib.util.spec_from_file_location('official_nav_launch', NAV / 'launch/nav2.launch.py')
NAV_LAUNCH = importlib.util.module_from_spec(spec)
spec.loader.exec_module(NAV_LAUNCH)
VC = NAV_LAUNCH.vehicle_config


class VehicleContract(unittest.TestCase):
    def test_installed_shared_module(self):
        self.assertEqual(Path(VC.__file__).resolve(), (DESC / 'config/vehicle_config.py').resolve())

    def test_twin_allows_unmeasured_overlays(self):
        for number in range(4):
            with self.subTest(vehicle=number):
                context = LaunchContext()
                context.launch_configurations.update(vehicle=str(number), use_sim_time='true')
                self.assertTrue(NAV_LAUNCH._nav2_for_vehicle(context))

    def refuse(self, vehicle, domain, message):
        env = dict(os.environ, ROS_DOMAIN_ID=domain, ROS_LOCALHOST_ONLY='1')
        result = subprocess.run(
            ['ros2', 'launch', 'mitt_navigation', 'nav2.launch.py',
             'vehicle:='+vehicle, 'use_sim_time:=false'],
            env=env, capture_output=True, text=True, timeout=15)
        output = result.stdout + result.stderr
        self.assertEqual(result.returncode, 1, output)
        self.assertIn(message, output)
        self.assertNotIn('process started with pid', output)

    def test_real_requires_vehicle(self):
        self.refuse('0', '191', 'vehicle:=0')

    def test_real_requires_matching_domain(self):
        self.refuse('1', '191', 'ROS_DOMAIN_ID')

    def test_real_requires_measured_radius(self):
        self.refuse('1', '11', 'no measured min_turning_radius')

    def test_generated_parameters_use_measurements_without_editing_stock(self):
        before = STOCK.read_bytes()
        with tempfile.TemporaryDirectory(prefix='mrider-vehicle-contract-') as temp:
            desc = Path(temp) / 'mitt_description'
            shutil.copytree(DESC / 'config', desc / 'config')
            # SYNTHETIC TEST FIXTURE ONLY. Never a vehicle calibration.
            measured = {'wheelbase': .7, 'chassis_width': .8, 'min_turning_radius': 2.01}
            (desc / 'config/vehicles/vehicle1.yaml').write_text(yaml.safe_dump(measured))
            with patch.object(VC.tempfile, 'gettempdir', return_value=temp):
                generated = Path(VC.nav2_for(1, str(desc), str(STOCK), real=True))
            self.assertNotEqual(generated, STOCK)
            params = yaml.safe_load(generated.read_text())
            values = {}
            def collect(node):
                if isinstance(node, dict):
                    for key, value in node.items():
                        if key in ('footprint', 'inflation_radius', 'minimum_turning_radius'):
                            values.setdefault(key, []).append(value)
                        collect(value)
                elif isinstance(node, list):
                    for value in node: collect(value)
            collect(params)
            base = VC._estimates(str(desc)); base.update(measured)
            self.assertEqual(values['minimum_turning_radius'], [2.12])
            self.assertGreaterEqual(values['minimum_turning_radius'][0], 2.01 * 1.05)
            self.assertEqual(values['footprint'], [VC.footprint(base)] * 2)
            self.assertEqual(values['inflation_radius'], [VC.inflation_radius(base)] * 2)
            outline = yaml.safe_load(values['footprint'][0])
            self.assertGreaterEqual(values['inflation_radius'][0], max(math.hypot(*p) for p in outline))
        self.assertEqual(STOCK.read_bytes(), before)


if __name__ == '__main__':
    unittest.main(verbosity=2)
