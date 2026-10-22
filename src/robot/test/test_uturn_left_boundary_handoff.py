"""U-turn exit ignores paint; explicit left-reference guards remain available."""
from __future__ import division

import math
import os
import unittest

from robot.common.config import load_config
from robot.master.controller import Controller
from robot.uturn.timed import TimedUturn


class UturnLeftBoundaryHandoffTests(unittest.TestCase):
    def make_done(self):
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cfg = load_config(os.path.join(root, 'config'))
        cfg.update(wait_green=False, lidar_enabled=False,
                   steering_command_scale_rad=.03)
        c = Controller(cfg)
        self.addCleanup(c.close)
        c.state = c.action = 'UTURN'
        c.action_started = 1.
        c.wait_until = 0.
        c.uturn = dict(trial_last_yaw=0., trial_turn_rad=0.)
        c.timed_uturn = TimedUturn(cfg)
        c.timed_uturn.index = len(c.timed_uturn.steps)
        c.front_marker_stamp = 10.
        return c

    def boundary(self, c, stamp, local_y=.30, slope=0.):
        points = [(x, slope*x + local_y) for x in (.25, .45, .65)]
        c.observe_left_boundary(points, stamp)

    def test_done_uturn_drives_straight_despite_a_current_left_line(self):
        c = self.make_done()
        self.boundary(c, 9.9, local_y=.40)
        command = c.tick(10.)
        self.assertEqual(command, (c.cfg['straight_speed_raw'], 0.))
        self.assertIsNone(c.left_reference)
        self.assertEqual(c.state, 'LANE')
        self.assertIsNone(c.action)
        self.assertIsNone(c.uturn)
        self.assertEqual(c.tick(10.15), command)
        self.assertFalse(c.follow_left_boundary)

    def test_missing_white_finishes_uturn_without_waiting_for_paint(self):
        c = self.make_done()
        self.boundary(c, 9.)
        straight = (c.cfg['straight_speed_raw'], 0.)
        self.assertEqual(c.tick(10.), straight)
        self.assertEqual(c.state, 'LANE')
        self.assertIsNone(c.action)
        c.observe_left_boundary([], 10.1)
        self.assertEqual(c.tick(10.1), straight)
        self.assertEqual(c.tick(10.2), straight)
        self.assertEqual(c.state, 'LANE')

    def test_white_line_reappearance_cannot_take_over_straight_exit(self):
        c = self.make_done()
        self.boundary(c, 9.9)
        self.assertGreater(c.tick(10.)[0], 0)
        self.assertEqual(c.state, 'LANE')

        self.boundary(c, 9.95, local_y=.30)
        self.assertEqual(c.tick(10.), (c.cfg['straight_speed_raw'], 0.))
        self.assertEqual(c.state, 'LANE')
        self.assertIsNone(c.uturn)

        self.boundary(c, 10.1, local_y=.30)
        self.assertEqual(c.tick(10.1), (c.cfg['straight_speed_raw'], 0.))
        self.assertFalse(c.follow_left_boundary)
        self.assertEqual(c.state, 'LANE')

    def test_slanted_line_offset_is_normal_to_the_line(self):
        c = self.make_done()
        slope = .4
        intercept = .30 * math.sqrt(1. + slope*slope)
        c.observe_left_boundary(
            [(x, slope*x + intercept) for x in (.25, .45, .65)], 10.)
        line = c.left_exit_line(10.)
        self.assertIsNotNone(line)
        self.assertAlmostEqual(line['lateral'], 0., places=7)
        for x, y in line['points']:
            self.assertAlmostEqual(y, slope*x, places=7)

    def test_curved_left_boundary_uses_the_near_tangent_for_exit_correction(self):
        c = self.make_done()
        self.boundary(c, 9.9)
        c.tick(10.)
        # The round road is coherent near the vehicle, but its distant arc
        # cannot fit a single straight line over the entire one-metre view.
        points = [(x, .30 + .5*x*x) for x in (.25, .35, .45, .65, .85, 1.05)]
        c.observe_left_boundary(points, 10.1)
        command = c.left_reference_command(c.left_exit_line(10.1), 10.1)
        self.assertGreater(command[0], 0, c.left_fit_diagnostic)
        self.assertEqual(c.state, 'LANE')
        self.assertIsNone(c.action)
        self.assertLess(c.left_fit_diagnostic['fit_points'], len(points))
        self.assertLess(abs(c.left_reference['heading_deg']), 30.)

    def test_explicit_left_reference_through_body_stops(self):
        c = self.make_done()
        self.boundary(c, 9.9)
        self.assertGreater(c.tick(10.)[0], 0)

        # The observed tape is only 11 cm from the rear-axle centre.  The
        # 12 cm body half-width plus the configured 3.5 cm margin already
        # puts the current footprint across the tape.
        self.boundary(c, 10.1, local_y=.11)
        self.assertEqual(c.left_reference_command(c.left_exit_line(10.1), 10.1)[0], 0)
        self.assertEqual(c.state, 'LANE')
        self.assertIsNone(c.action)
        self.assertEqual(c.reason, 'left_boundary_footprint_crossing')
        self.assertEqual(c.left_fit_diagnostic['guard_reason'],
                         'footprint_crosses_left_boundary')
        self.assertLess(c.left_fit_diagnostic['guard_clearance_m'], 0.)

    def test_slanted_boundary_front_corner_stops_before_it_presses_tape(self):
        c = self.make_done()
        self.boundary(c, 9.9)
        self.assertGreater(c.tick(10.)[0], 0)

        # At the rear axle the tape is clear, but its negative heading brings
        # the front-left corner onto it.  A centre-only lateral check misses
        # this case; the footprint sweep must stop it.
        slope = -1.0
        distance = .32
        intercept = distance * math.sqrt(1. + slope*slope)
        c.observe_left_boundary(
            [(x, slope*x + intercept) for x in (.25, .45, .65)], 10.1)
        self.assertEqual(c.left_reference_command(c.left_exit_line(10.1), 10.1)[0], 0)
        self.assertEqual(c.reason, 'left_boundary_footprint_crossing')
        self.assertEqual(c.left_fit_diagnostic['guard_reason'],
                         'footprint_crosses_left_boundary')
        self.assertLess(c.left_fit_diagnostic['sweep_clearance_m'], 0.)

    def test_full_left_residual_is_swept_before_next_valid_handoff(self):
        c = self.make_done()
        self.boundary(c, 9.9)
        self.assertGreater(c.tick(10.)[0], 0)

        # The current body has a small positive clearance, but a stale full
        # left steering command would rotate the front corner over the tape
        # during the next 250 ms command validity window.
        c.observe_applied_steering(c.cfg['steering_command_scale_rad'], 10.1)
        self.boundary(c, 10.1, local_y=.165)
        self.assertEqual(c.left_reference_command(c.left_exit_line(10.1), 10.1)[0], 0)
        self.assertEqual(c.reason, 'left_boundary_footprint_crossing')
        self.assertLess(c.left_fit_diagnostic['sweep_clearance_m'], 0.)
        # The old short-sweep bug (omitting abs(speed) from speed_gain) would
        # travel only about 2 mm here and incorrectly pass this frame.
        from robot.motion import calibration as chassis
        expected = (abs(c.cfg['left_reference_speed_raw']) *
                    abs(chassis.speed_gain(c.cfg, c.cfg['left_reference_speed_raw'])) * .25)
        self.assertAlmostEqual(c.left_fit_diagnostic['guard_travel_m'], expected)

    def test_zero_offset_park_line_is_not_treated_as_road_boundary(self):
        c = self.make_done()
        line = dict(points=[(.25, -.20), (.45, -.20), (.65, -.20)],
                    heading=0., lateral=-.20, offset_m=0.)
        command = c.left_reference_command(line, 10.)
        self.assertGreater(command[0], 0)
        self.assertNotEqual(c.reason, 'left_boundary_footprint_crossing')

    def test_straight_exit_ends_at_the_next_confirmed_blue_trigger(self):
        c = self.make_done()
        c.cfg['blue_timed_enabled'] = False
        self.boundary(c, 9.9)
        c.tick(10.)
        for stamp in (10.1, 10.3, 10.7):
            self.boundary(c, stamp)
            c.tick(stamp)
        self.assertTrue(c.uturn_exit_straight)
        for stamp in (10.8, 10.9):
            c.observe_sign('RIGHT', .99, stamp, stamp)
        self.assertTrue(c.uturn_exit_straight)
        for stamp in (11., 11.1, 11.2):
            c.observe_ground(dict(source='front', part='markers', slots=[],
                markers=[dict(kind='junction', x=.3, y=0., length=.6)],
                blue_lines=[dict(x=.3, y=0., yaw=0., length=.6)]), stamp)
            c.dispatch(stamp)
        self.assertEqual(c.action, 'RIGHT')
        self.assertFalse(c.uturn_exit_straight)
        self.assertFalse(c.follow_left_boundary)
        self.assertIsNone(c.left_reference)


if __name__ == '__main__':
    unittest.main()
