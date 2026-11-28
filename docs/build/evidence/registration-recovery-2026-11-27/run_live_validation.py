"""Run the same three-goal contract against an already running RViz backend.

Set ROS_DOMAIN_ID and MRIDER_SIM_REPORT_DIR before running. This script does
not create/restart a simulator or RViz. Its only driving inputs are operator
goal poses and arm/start services; truth is consumed only by the evaluator.
"""
import importlib.util
import sys
import unittest
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


root = Path(sys.argv[1] if len(sys.argv) > 1 else '/root/mrider-team')
test_file = root / 'ros2_ws/src/mrider_navigation_sim/test/test_large_office_turns_launch.py'
spec = importlib.util.spec_from_file_location('large_office_live_contract', test_file)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class LiveValidation(module.LargeOfficeTurns):
    report_name = 'live_large_office_dynamic'
    actor_offset = 227.0
    require_stop_resume = True


result = None
try:
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(LiveValidation)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
finally:
    # Leave the existing display intact, but revoke driving permission after
    # either PASS or FAIL. This cannot convert a failed assertion into PASS.
    if not rclpy.ok():
        rclpy.init()
    node = Node('live_validation_cleanup')
    client = node.create_client(Trigger, '/mrider_sim/disarm')
    try:
        if client.wait_for_service(timeout_sec=5.0):
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            print('FINAL_DISARM', future.result() if future.done() else 'timeout', flush=True)
        else:
            print('FINAL_DISARM service unavailable', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
sys.exit(0 if result is not None and result.wasSuccessful() else 1)
