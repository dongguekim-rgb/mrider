"""Delayed-start pedestrian phase: both turns AND an obstacle stop/restart.

The 227 s environment offset approximates the prior GUI's 233 s startup
relative to a normal ~6 s startup. This is not an exact wall-time replay;
the recorded scan fixture separately replays the exact registration failure.
"""
import importlib.util
from pathlib import Path
import unittest

import launch_testing
import launch_testing.asserts
import pytest

spec=importlib.util.spec_from_file_location('office_turns_contract',
    Path(__file__).with_name('test_large_office_turns_launch.py'))
turns=importlib.util.module_from_spec(spec)
spec.loader.exec_module(turns)


@pytest.mark.launch_test
def generate_test_description():
    return turns.large_office_launch('219',227.)


class LargeOfficeDynamic(turns.LargeOfficeTurns):
    report_name='large_office_dynamic'
    actor_offset=227.
    require_stop_resume=True


@launch_testing.post_shutdown_test()
class Exit(unittest.TestCase):
    def test_shutdown(self,proc_info):launch_testing.asserts.assertExitCodes(proc_info)
