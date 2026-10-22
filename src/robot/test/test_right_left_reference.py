"""Timed RIGHT follows only the measured left tape, 30 cm inward."""
import math
import os
import unittest

from robot.common.config import load_config
from robot.master.controller import Controller


class RightLeftReferenceTests(unittest.TestCase):
    def core(self):
        cfg = load_config(os.path.join(os.path.dirname(__file__), '../config'))
        cfg.update(wait_green=False, lidar_enabled=False,
                   steering_command_scale_rad=.03)
        c = Controller(cfg)
        self.addCleanup(c.close)
        c.action = 'RIGHT'
        c.execute('mission', 'begin_blue_action', 0.)
        c.right_lock.update(phase='WAIT_LANE', exit_started=10.)
        return c

    def left(self, c, stamp, distance=.3, heading=0.):
        slope = math.tan(heading)
        c.observe_left_boundary([(x, slope*x+distance/math.cos(heading))
                                 for x in (.25, .45, .65)], stamp)
        c.front_marker_stamp = stamp

    def test_right_curve_and_center_cannot_change_left_target(self):
        outputs = []
        for center in ([(.25, -.2), (.45, -.35), (.65, -.5)],
                       [(.25, .2), (.45, .35), (.65, .5)], []):
            c = self.core()
            self.left(c, 10.1)
            c.observe_lane(center, .99, 10.1,
                           {'RIGHT': [(.25, -.2), (.45, -.5), (.65, -.9)]})
            outputs.append(c.tick(10.1))
            self.assertEqual(c.right_lock['exit_source'], 'left_boundary')
            self.assertAlmostEqual(c.left_reference['offset_m'], .3)
        self.assertEqual(outputs, [(20, 0.)]*3)

    def test_lateral_target_is_thirty_cm_to_right_of_left_tape(self):
        c = self.core()
        self.left(c, 10.1, distance=.4)
        speed, steer = c.tick(10.1)
        self.assertEqual(speed, 20)
        self.assertGreater(steer, 0.)
        self.assertAlmostEqual(c.left_reference['lateral_m'], .1)

    def test_body_heading_follows_slanted_left_tape(self):
        c = self.core()
        self.left(c, 10.1, heading=.15)
        self.assertGreater(c.tick(10.1)[1], 0.)
        self.assertAlmostEqual(c.left_reference['heading_deg'], math.degrees(.15))

    def test_requested_offset_stays_thirty_cm_if_lane_width_changes(self):
        c = self.core()
        c.cfg['lane_width'] = .8
        self.left(c, 10.1)
        self.assertEqual(c.tick(10.1), (20, 0.))
        self.assertAlmostEqual(c.left_reference['offset_m'], .3)
        self.assertAlmostEqual(c.left_reference['lateral_m'], 0.)

    def test_no_left_tape_never_substitutes_right_or_center(self):
        c = self.core()
        c.observe_lane([(.25, .1), (.45, .2), (.65, .3)], .99, 10.1)
        c.front_marker_stamp = 10.1
        self.assertEqual(c.tick(10.1), (20, 0.))
        self.assertEqual(c.reason, 'right_timed_exit_search_straight')

    def test_left_loss_hold_expires_even_when_center_keeps_arriving(self):
        c = self.core()
        self.left(c, 10.1, heading=.15)
        previous = c.tick(10.1)
        c.observe_left_boundary([], 10.2)
        c.observe_lane([(.25, -.1), (.45, -.2), (.65, -.3)], .99, 10.2)
        self.assertEqual(c.tick(10.2), previous)
        self.assertEqual(c.reason, 'right_timed_exit_hold_reference')
        c.observe_lane([(.25, -.1), (.45, -.2), (.65, -.3)], .99, 11.2)
        self.assertEqual(c.tick(11.2), (20, 0.))
        self.left(c, 11.3)
        self.assertEqual(c.tick(11.3)[0], 20)
        self.assertEqual(c.reason, 'right_timed_exit_align')


if __name__ == '__main__':
    unittest.main()
