import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mrider_autonomy.core import Config, Controller
from mrider_autonomy.simulation import run


class CoreTests(unittest.TestCase):
    def setUp(self):
        self.c = Controller()
        self.feed()

    def feed(self, points=((0., 0.), (10., 0.)), pose=(0., 0., 0.), now=10.):
        self.c.set_path(points, 'map', now, now, now)
        self.c.set_odometry(*pose, 'map', 'base_link', now, now, now)

    def test_missing(self):
        self.assertEqual(Controller().command(10, 10).speed, 0)

    def test_straight(self):
        cmd = self.c.command(10, 10)
        self.assertEqual(cmd.steering, 0)
        self.assertEqual(cmd.speed, self.c.config.max_speed)

    def test_left_and_right_sign(self):
        for y in (1., -1.):
            self.c = Controller()
            self.feed(((0., 0.), (4., y)))
            self.assertGreater(self.c.command(10, 10).steering*y, 0)

    def test_steering_saturation(self):
        self.c = Controller()
        self.feed(((0., 0.), (.1, 5.)))
        self.assertEqual(self.c.command(10, 10).steering, self.c.config.max_steering)

    def test_slowdown(self):
        self.feed(pose=(9.5, 0., 0.), now=11)
        cmd = self.c.command(11, 11)
        self.assertGreater(cmd.speed, 0)
        self.assertLess(cmd.speed, self.c.config.max_speed)

    def test_goal_stop(self):
        self.feed(pose=(9.95, 0., 0.), now=11)
        self.assertEqual(self.c.command(11, 11).reason, 'goal_reached')

    def test_target_behind(self):
        self.feed(pose=(0., 0., math.pi), now=11)
        self.assertEqual(self.c.command(11, 11).speed, 0)

    def test_dropout_recovery(self):
        self.assertEqual(self.c.command(10.6, 10.6).speed, 0)
        self.feed(now=11)
        self.assertGreater(self.c.command(11, 11).speed, 0)

    def test_individual_timeouts(self):
        self.c.set_odometry(0, 0, 0, 'map', 'base_link', 10.6, 10.6, 10.6)
        self.assertEqual(self.c.command(10.6, 10.6).speed, 0)
        self.c = Controller()
        self.feed()
        self.c.set_path(((0, 0), (10, 0)), 'map', 10.6, 10.6, 10.6)
        self.assertEqual(self.c.command(10.6, 10.6).speed, 0)

    def test_frozen_clock(self):
        self.assertEqual(self.c.command(10, 10.6).reason, 'stale_input')

    def test_clock_rewind(self):
        self.c.command(10, 10)
        self.assertEqual(self.c.command(5, 11).reason, 'clock_reset')
        self.feed(now=6)
        self.assertGreater(self.c.command(6, 6).speed, 0)

    def test_frames(self):
        self.c.set_path(((0, 0), (10, 0)), 'odom', 11, 11, 11)
        self.assertEqual(self.c.command(11, 11).speed, 0)
        for frame, child in [('odom', 'base_link'), ('map', 'wrong_base')]:
            self.c = Controller()
            self.feed()
            self.c.set_odometry(0, 0, 0, frame, child, 11, 11, 11)
            self.assertEqual(self.c.command(11, 11).speed, 0)

    def test_bad_timestamps(self):
        for stamp in (0, -1, 9, 11, math.nan, math.inf, 10):
            self.c = Controller()
            self.feed()
            self.assertFalse(self.c.set_path(((0, 0), (10, 0)), 'map', stamp, 10, 10))
            self.assertEqual(self.c.command(10, 10).speed, 0)

    def test_repeated_stamp_cannot_recover(self):
        for _ in range(4):
            self.assertFalse(self.c.set_path(((0, 0), (10, 0)), "map", 10, 10, 10))
            self.assertEqual(self.c.command(10, 10).speed, 0)

    def test_nonfinite_and_extreme(self):
        for value in (math.nan, math.inf, -math.inf, 1e300):
            self.c = Controller()
            self.feed()
            self.assertFalse(self.c.set_path(((0, 0), (value, 0)), 'map', 11, 11, 11))
            self.assertEqual(self.c.command(11, 11).speed, 0)
            self.assertFalse(self.c.set_odometry(value, 0, 0, 'map', 'base_link', 11, 11, 11))

    def test_empty_and_degenerate(self):
        for points in ([], [(0, 0)], [(0, 0), (0, 0)], [(0, 0), (1e-200, 0)]):
            self.assertFalse(self.c.set_path(points, 'map', 11, 11, 11))

    def test_invalid_config(self):
        for kwargs in ({'wheelbase': 0}, {'max_speed': -1}, {'lookahead': math.nan},
                       {'max_steering': 2}, {'goal_tolerance': 2}, {'frame': ''}):
            with self.assertRaises(ValueError):
                Config(**kwargs)

    def test_straight_closed_loop(self):
        self.assertLessEqual(run('straight')['goal_error_m'], Config().goal_tolerance)

    def test_curve_closed_loop(self):
        result = run('curve')
        self.assertLessEqual(result['goal_error_m'], Config().goal_tolerance)
        self.assertLessEqual(result['max_abs_steering_rad'], Config().max_steering)


if __name__ == '__main__':
    unittest.main()
