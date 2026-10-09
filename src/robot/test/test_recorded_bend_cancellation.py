"""The 200948 field run lost its bend angle to an extrapolated tangent."""
import math
import os
import unittest

from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.master.controller import Controller


class RecordedBendCancellationTests(unittest.TestCase):
    # Source captures 1790770229.7105787 and 1790770232.7724924.
    PATHS = [
        [(.637740565, .045274327), (.700710076, .047235355),
         (.771814342, .044800647), (.844382870, .037193466),
         (.931458394, .021064645), (1.013903218, -.001572259)],
        [(.504305164, .003529291), (.556885095, -.006286129),
         (.630887363, -.025809788), (.708488202, -.053938758),
         (.778264065, -.086604243), (.851663844, -.129545948),
         (.909982915, -.170981783)],
    ]

    def core(self):
        cfg = load_config(os.path.join(os.path.dirname(__file__), '../config'))
        cfg.update(wait_green=False, lidar_enabled=False, lane_curvature_preview=True,
                   steering_command_scale_rad=.03, lane_curve_speed_raw=16)
        cfg['speed_raw']['lane'] = 24
        c = Controller(cfg)
        self.addCleanup(c.close)
        return c

    def raw(self, c, path):
        c.observe_lane(path, .8, 1.)
        speed, steer = c.lane_command(1.)
        self.assertEqual(speed, 16)
        return encode_command(speed, steer, c.cfg, 0)['steering_raw']

    def test_supported_outer_bend_cannot_be_cancelled_to_one_raw_unit(self):
        for path in self.PATHS:
            c = self.core()
            raw = self.raw(c, path)
            fit = c.lane_preview
            self.assertEqual(fit['model'], 'circle')
            self.assertLess(fit['center_error_m'], -.02)
            self.assertGreater(fit['center_heading_error_rad'], .35)
            required = math.atan(c.cfg['wheelbase']*abs(fit['curvature']))
            required_raw = required/c.cfg['max_steer']*c.cfg['steering_raw_limit']
            self.assertLessEqual(raw, -required_raw+1.)

    def test_left_bend_has_the_same_radius_protection(self):
        for path in self.PATHS:
            right = self.raw(self.core(), path)
            left = self.raw(self.core(), [(x, -y) for x, y in path])
            self.assertEqual(left, -right)

    def test_clearly_left_rejoin_can_countersteer_on_a_right_shaped_arc(self):
        c = self.core()
        path = [(x, y+.22) for x, y in self.PATHS[0]]
        raw = self.raw(c, path)
        self.assertLess(c.lane_preview['curvature'], -.5)
        self.assertGreater(raw, 0)

    def test_recorded_bypass_rejoin_still_turns_toward_left_center(self):
        # Source capture 1790762858.4850428 from the earlier bypass run.
        path = [(.595376531, .190744332), (.729306277, .240745086),
                (.861479804, .273856002), (.993804334, .292148360),
                (1.126193898, .296227550)]
        c = self.core()
        self.assertGreater(self.raw(c, path), 5)
        self.assertFalse(c.lane_preview['bend_radius_floor'])

    def test_near_center_margin_does_not_switch_wheel_angle_abruptly(self):
        path = self.PATHS[0]
        before = self.raw(self.core(), [(x, y+.004) for x, y in path])
        after = self.raw(self.core(), [(x, y+.006) for x, y in path])
        self.assertLessEqual(abs(after-before), 1)


if __name__ == '__main__':
    unittest.main()
