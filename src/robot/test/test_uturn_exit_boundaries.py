"""U-turn exit follows observed road boundaries while waiting for blue."""
import os
import json
import unittest

import robot
from robot.common.config import load_config
from robot.lane.controller import _left_boundary_sweep_clear
from robot.master.controller import Controller
from robot.uturn.timed import TimedUturn


class UturnExitBoundaryTests(unittest.TestCase):
    def core(self, left=None, right=None, stamp=10.):
        cfg = load_config(os.path.join(os.path.dirname(robot.__file__), 'config'))
        cfg.update(wait_green=False, lidar_enabled=False,
                   steering_command_scale_rad=.03, straight_speed_raw=30,
                   lane_curve_speed_raw=30, lookahead=1.)
        cfg['speed_raw']['lane'] = 40
        c = Controller(cfg)
        self.addCleanup(c.close)
        c.state = c.action = 'UTURN'
        c.action_started = 1.
        c.uturn = dict(trial_last_yaw=0., trial_turn_rad=0.)
        c.timed_uturn = TimedUturn(cfg)
        c.timed_uturn.index = len(c.timed_uturn.steps)
        c.observe_ground(dict(source='front', part='markers', slots=[],
                              markers=[], blue_lines=[]), 10.)
        edges = {}
        for side, intercept in (('LEFT', left), ('RIGHT', right)):
            if intercept is not None:
                edges[side] = [(x, intercept) for x in (.25, .45, .65, .85)]
        # A conflicting center estimate cannot override two observed edges.
        c.observe_lane([(.25, -.10), (.45, -.10), (.65, -.10)], .99, stamp, edges)
        return c

    def test_two_edges_shift_left_and_right_to_the_measured_midpoint(self):
        for left, right, sign in ((.4, -.2, 1), (.2, -.4, -1)):
            c = self.core(left, right)
            speed, steer = c.tick(10.)
            self.assertGreater(speed, 0)
            self.assertGreater(steer*sign, 0)
            self.assertEqual(c.lane_source, 'uturn_exit_center')
            self.assertIsNone(c.action)

    def test_centered_pair_keeps_steering_neutral(self):
        c = self.core(.3, -.3)
        speed, steer = c.tick(10.)
        self.assertGreater(speed, 0)
        self.assertAlmostEqual(steer, 0.)

    def test_single_observed_edge_uses_inward_half_lane_offset(self):
        for left, right, sign in ((.4, None, 1), (None, -.4, -1)):
            c = self.core(left, right)
            speed, steer = c.tick(10.)
            self.assertGreater(speed, 0)
            self.assertGreater(steer*sign, 0)

    def test_no_edges_preserve_straight_fallback(self):
        c = self.core()
        self.assertEqual(c.tick(10.), (30, 0.))

    def test_stale_edges_cannot_steer_current_exit(self):
        c = self.core(.4, -.2, stamp=8.)
        self.assertEqual(c.tick(10.), (30, 0.))

    def test_observed_flared_pair_is_not_extended_back_through_the_body(self):
        c = self.core()
        xs = (.25, .35, .45, .55)
        edges = dict(LEFT=[(x, .02+.9*x) for x in xs],
                     RIGHT=[(x, -.02-.9*x) for x in xs])
        c.observe_lane([(x, 0.) for x in xs], .99, 10.1, edges)
        speed, steer = c.tick(10.1)
        self.assertGreater(speed, 0., c.reason)
        self.assertAlmostEqual(steer, 0.)
        self.assertEqual(c.lane_source, 'uturn_exit_center')

    def test_slanted_single_edge_does_not_invent_paint_behind_its_first_point(self):
        for side, sign in (('LEFT', 1), ('RIGHT', -1)):
            c = self.core()
            xs = (.25, .45, .65, .85)
            edges = {side: [(x, sign*(.02+.9*x)) for x in xs]}
            c.observe_lane([(x, 0.) for x in xs], .99, 10.1, edges)
            speed, steer = c.tick(10.1)
            self.assertGreater(speed, 0., c.reason)
            self.assertGreater(sign*steer, 0.)

    def test_either_edge_crossing_the_vehicle_stops_before_driving(self):
        for left, right in ((.11, -.49), (.49, -.11)):
            c = self.core(left, right)
            self.assertEqual(c.tick(10.), (0, 0.))
            self.assertEqual(c.reason, 'uturn_exit_boundary_crossing')
            self.assertEqual(c.state, 'LANE')
            self.assertIsNone(c.action)

    def test_paint_between_body_corners_is_still_guarded(self):
        c = self.core()
        edges = dict(LEFT=[(x, .11) for x in (.07, .14, .21)])
        c.observe_lane([(.25, 0.), (.45, 0.)], .99, 10.1, edges)
        self.assertEqual(c.tick(10.1), (0, 0.))
        self.assertEqual(c.reason, 'uturn_exit_boundary_crossing')

    def test_a_future_sweep_into_observed_paint_is_still_blocked(self):
        for side, sign in (('LEFT', 1), ('RIGHT', -1)):
            c = self.core()
            line = dict(heading=0., lateral=sign*.16-.3, offset_m=.3,
                        side=side, boundary_distance_m=sign*.16,
                        observed_points=[(x, sign*.16) for x in (.25, .45, .65)])
            self.assertFalse(_left_boundary_sweep_clear(c, line, 30, sign*.03))
            self.assertGreater(c.left_fit_diagnostic['footprint_clearance_m'], 0.)
            self.assertLess(c.left_fit_diagnostic['sweep_clearance_m'], 0.)

    def test_paint_beyond_the_sweep_keeps_diagnostics_finite(self):
        c = self.core()
        xs = (.6, .7, .8, .9)
        edges = dict(LEFT=[(x, .02+.6*x) for x in xs],
                     RIGHT=[(x, -.02-.6*x) for x in xs])
        c.observe_lane([(x, 0.) for x in xs], .99, 10.1, edges)
        speed, steer = c.tick(10.1)
        self.assertGreater(speed, 0., c.reason)
        self.assertAlmostEqual(steer, 0.)
        self.assertIsNone(c.left_fit_diagnostic['sweep_clearance_m'])
        json.dumps(c.left_fit_diagnostic, allow_nan=False)

    def test_crossed_or_too_narrow_edges_stop_instead_of_becoming_a_center(self):
        for left, right in ((-.2, .4), (.1, -.1)):
            c = self.core(left, right)
            self.assertEqual(c.tick(10.), (0, 0.))
            self.assertEqual(c.reason, 'uturn_exit_lane_geometry_invalid')

    def test_a_new_safe_pair_recovers_after_boundary_guard_stop(self):
        c = self.core(.11, -.49)
        c.tick(10.)
        edges = dict(LEFT=[(x, .3) for x in (.25, .45, .65)],
                     RIGHT=[(x, -.3) for x in (.25, .45, .65)])
        c.observe_lane([(.25, 0.), (.45, 0.), (.65, 0.)], .99, 10.1, edges)
        c.observe_ground(dict(source='front', part='markers', slots=[],
                              markers=[], blue_lines=[]), 10.1)
        speed, steer = c.tick(10.1)
        self.assertGreater(speed, 0)
        self.assertAlmostEqual(steer, 0.)


if __name__ == '__main__':
    unittest.main()
