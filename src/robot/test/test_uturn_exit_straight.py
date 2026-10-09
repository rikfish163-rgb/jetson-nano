"""Completed U-turns drive straight until a fresh blue junction is confirmed."""
from __future__ import division

import math
import os
import unittest

from robot.common.config import load_config
from robot.master.controller import Controller
from robot.turn.planner import intersection_path
from robot.uturn.timed import TimedUturn
from test_continuous_obstacles import _SyntheticScan


class UturnExitStraightTests(unittest.TestCase):
    def done(self):
        cfg = load_config(os.path.join(os.path.dirname(__file__), '../config'))
        cfg.update(wait_green=False, lidar_enabled=False,
                   steering_command_scale_rad=.03, straight_speed_raw=30)
        c = Controller(cfg)
        self.addCleanup(c.close)
        c.state = c.action = 'UTURN'
        c.action_source = 'sign'
        c.action_started = 1.
        c.wait_until = 0.
        c.uturn = dict(trial_last_yaw=0., trial_turn_rad=0.)
        c.timed_uturn = TimedUturn(cfg)
        c.timed_uturn.index = len(c.timed_uturn.steps)
        # The U-turn has already consumed its own junction behind the car.
        c.consumed_marker = (-.5, 0.)
        self.frame(c, 10.)
        return c

    def frame(self, c, stamp, x=None, angle=0.):
        markers = [] if x is None else [dict(kind='junction', x=x, y=0., length=.8)]
        lines = [] if x is None else [dict(x=x, y=0., yaw=math.radians(angle), length=.8)]
        c.observe_ground(dict(source='front', part='markers', slots=[],
                              markers=markers, blue_lines=lines), stamp)

    def test_completion_without_paint_immediately_drives_straight(self):
        c = self.done()
        self.assertEqual(c.tick(10.), (30, 0.))
        self.assertEqual(c.state, 'LANE')
        self.assertIsNone(c.action)
        self.assertIsNone(c.uturn)
        self.assertIsNone(c.timed_uturn)
        self.assertEqual(c.last_completed_action, 'UTURN')
        self.assertTrue(c.uturn_exit_straight)
        self.assertEqual(c.reason, 'uturn_exit_straight_wait_blue')

    def test_white_line_and_curved_centers_do_not_steer_or_stop_the_exit(self):
        c = self.done()
        c.observe_left_boundary([(.25, .11), (.45, .11), (.65, .11)], 10.)
        c.observe_lane([(.3, .15), (.6, .35), (.9, .5)], .99, 10.)
        self.assertEqual(c.tick(10.), (30, 0.))
        self.assertFalse(c.follow_left_boundary)
        self.assertIsNone(c.left_reference)
        self.assertEqual(c.lane_source, 'uturn_exit_straight')
        for stamp in (10.1, 10.5, 50.):
            self.frame(c, stamp)
            c.observe_left_boundary([], stamp)
            c.observe_lane([], 0., stamp)
            self.assertEqual(c.tick(stamp), (30, 0.))
            self.assertEqual(c.state, 'LANE')
            self.assertIsNone(c.action)

    def test_queued_right_survives_completion_and_waits_for_blue(self):
        c = self.done()
        c.next_direction, c.next_direction_at = 'RIGHT', 5.
        self.assertEqual(c.tick(10.), (30, 0.))
        self.assertEqual(c.pending, 'RIGHT')
        self.frame(c, 10.1)
        self.assertEqual(c.tick(10.1), (30, 0.))
        self.assertEqual(c.pending, 'RIGHT')
        self.assertIsNone(c.action)

    def test_unconfirmed_blue_does_not_prealign_or_start_an_action(self):
        c = self.done()
        c.tick(10.)
        c.pending, c.pending_at = 'RIGHT', 10.
        cam = c.cfg['front_camera']
        x = (cam['origin_v']-.3*(cam['bev_height']-1))/cam['pixels_per_m']
        self.frame(c, 10.1, x, angle=25.)
        self.assertEqual(c.tick(10.1), (30, 0.))
        self.assertIsNone(c.action)
        self.assertIsNone(c.blue_prealign)
        self.assertEqual(c.marker_candidate['count'], 1)
        self.assertEqual(c.tick(10.15), (30, 0.))
        self.assertEqual(c.marker_candidate['count'], 1)
        self.frame(c, 10.2, x, angle=25.)
        self.assertEqual(c.tick(10.2), (30, 0.))
        self.assertIsNone(c.action)

    def test_confirmed_blue_releases_straight_and_dispatches_cached_right(self):
        c = self.done()
        c.tick(10.)
        c.pending, c.pending_at = 'RIGHT', 10.
        for stamp in (10.1, 10.2, 10.3):
            self.frame(c, stamp, .8, angle=25.)
            c.tick(stamp)
        self.assertFalse(c.uturn_exit_straight)
        self.assertEqual(c.action, 'RIGHT')
        self.assertEqual(c.state, 'BLUE_APPROACH')
        self.assertIsNone(c.pending)
        self.assertTrue(c.blue_consumed)
        self.assertIsNone(c.blue_prealign)
        self.assertFalse(c.blue_approach.get('prealigned_during_confirmation', False))
        self.assertEqual(c.last_blue_trigger['action'], 'RIGHT')

    def test_consumed_old_blue_does_not_end_straight(self):
        c = self.done()
        c.consumed_marker = (.3, 0.)
        c.blue_consumed = True
        c.tick(10.)
        c.pending, c.pending_at = 'RIGHT', 10.
        for stamp in (10.1, 10.2, 10.3):
            self.frame(c, stamp, .3)
            self.assertEqual(c.tick(stamp), (30, 0.))
        self.assertTrue(c.uturn_exit_straight)
        self.assertIsNone(c.action)

    def test_bypass_completion_returns_to_straight_until_blue(self):
        c = self.done()
        c.tick(10.)
        c.pending, c.pending_at = 'RIGHT', 10.
        c.action, c.action_source, c.state = 'BYPASS', 'obstacle', 'MANEUVER'
        c.resume_lane()
        self.frame(c, 10.1)
        self.assertEqual(c.tick(10.1), (30, 0.))
        self.assertTrue(c.uturn_exit_straight)
        self.assertEqual(c.pending, 'RIGHT')

    def test_starting_another_maneuver_clears_exit_mode(self):
        c = self.done()
        c.tick(10.)
        c.start_follow(intersection_path(c.pose, 'RIGHT', c.cfg), 'RIGHT', 10.1)
        self.assertFalse(c.uturn_exit_straight)

    def test_fresh_empty_camera_frames_keep_driving_stale_camera_stops_and_recovers(self):
        c = self.done()
        c.tick(10.)
        stale = 10.+c.cfg.get('ground_timeout', 1.25)+.01
        self.assertEqual(c.tick(stale), (0, 0.))
        self.assertEqual(c.reason, 'uturn_exit_front_stale')
        self.assertEqual(c.state, 'LANE')
        self.frame(c, stale+.1)
        self.assertEqual(c.tick(stale+.1), (30, 0.))

    def test_estop_red_and_missing_lidar_still_stop_during_straight(self):
        for fault, reason in (('estop', 'emergency_stop'), ('red', 'red_latched'),
                              ('lidar', 'scan_missing_or_stale')):
            c = self.done()
            c.tick(10.)
            if fault == 'lidar':
                c.cfg['lidar_enabled'] = True
            else:
                setattr(c, fault, True)
            self.assertEqual(c.tick(10.1), (0, 0.))
            self.assertEqual(c.reason, reason)

    def test_real_obstacle_guard_stops_and_clear_scan_restores_straight(self):
        c = self.done()
        c.tick(10.)
        c.cfg.update(lidar_enabled=True, bypass_enabled=False, timed_bypass_enabled=False)
        c.scan = _SyntheticScan(10.1, obstacles=[(.20, 0.)])
        self.frame(c, 10.1)
        self.assertEqual(c.tick(10.1), (0, 0.))
        self.assertEqual(c.obstacle_check['kind'], 'obstacle')
        self.assertTrue(c.uturn_exit_straight)
        c.scan = _SyntheticScan(10.2)
        self.frame(c, 10.2)
        self.assertEqual(c.tick(10.2), (30, 0.))


if __name__ == '__main__':
    unittest.main()
