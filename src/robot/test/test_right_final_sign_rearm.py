"""Image-driven RIGHT rearming through the timed maneuver; no hardware."""
import os
import unittest

from robot.common.config import load_config
from robot.master.controller import Controller


class RightFinalSignRearmTests(unittest.TestCase):
    def test_one_second_of_forward_right_does_not_release_visible_board(self):
        c = self.core()
        for i in range(100):
            now = 2.+i*.1
            c.observe_sign('RIGHT', .99, now-.01, now)
            c.front_marker_stamp = now
            c.tick(now)
            if c.right_lock['phase'] != 'TIMED_TURN':
                continue
            elapsed = now-c.right_lock['timed_started']
            if elapsed >= 1.05:
                self.assertIsNone(c.right_sign_rearmed_at)
                self.assertFalse(c.route_sign_rearmed)
                self.assertIsNone(c.next_direction)
                self.assertEqual(c.action, 'RIGHT')
                return
        self.fail('forward right never reached release time')

    def core(self, fast=False):
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cfg = load_config(os.path.join(root, 'config'))
        cfg.update(wait_green=False, lidar_enabled=False, steering_command_scale_rad=.03,
                   exit_frames=2, left_reference_stable_s=.1)
        if fast:
            cfg.update(right_timed_reverse_s=.1, right_timed_turn_s=.2,
                       right_timed_exit_reverse_s=.4)
        c = Controller(cfg)
        self.addCleanup(c.close)
        for t in (1., 1.1):
            c.observe_sign('RIGHT', .99, t, t)
        c.action, c.state, c.pending = 'RIGHT', 'BLUE_STOP', None
        c.action_source = 'sign'
        c.execute('mission', 'begin_blue_action', 2.)
        return c

    def released_cache(self, c):
        for i in range(100):
            now = 2.+i*.1
            c.observe_sign('RIGHT', .99, now-.01, now)
            c.front_marker_stamp = now
            c.tick(now)
            if c.right_lock['phase'] == 'TIMED_TURN':
                now += .01
                c.observe_sign('', 0., now, now)
                self.assertEqual(c.right_sign_rearmed_at, now)
                return now
            self.assertIsNone(c.next_direction)
        self.fail('current RIGHT cache was never released')

    def test_current_cache_clears_during_forward_right(self):
        c = self.core()
        now = self.released_cache(c)
        self.assertEqual(c.right_lock['phase'], 'TIMED_TURN')
        self.assertEqual(c.action, 'RIGHT')
        self.assertEqual(c.sign_label, '')
        self.assertEqual(c.sign_count, 0)
        self.assertTrue(c.route_sign_rearmed)
        self.assertGreaterEqual(c.sign_stamp, now)
        self.assertEqual(c.sign_info['decision'], 'right_sign_disappeared')

    def test_next_right_votes_while_forward_right_still_runs(self):
        c = self.core()
        now = self.released_cache(c)
        c.observe_sign('RIGHT', .99, now+.05, now+.05)
        self.assertEqual(c.sign_count, 1)
        c.front_marker_stamp = now+.1
        self.assertGreater(c.tick(now+.1)[0], 0)
        self.assertEqual(c.right_lock['phase'], 'TIMED_TURN')
        self.assertEqual(c.sign_count, 1, 'cache must be consumed only once')
        c.observe_sign('RIGHT', .99, now+.15, now+.15)
        self.assertEqual(c.next_direction, 'RIGHT')
        self.assertEqual(c.sign_info['decision'], 'stored_next_direction')

    def test_pre_release_source_frame_cannot_revote_for_the_next_right(self):
        c = self.core()
        now = self.released_cache(c)
        c.observe_sign('RIGHT', .99, now-.001, now+.01)
        self.assertEqual(c.sign_info['decision'], 'stale_or_duplicate')
        self.assertIsNone(c.next_direction)

    def test_new_right_survives_turn_reverse_and_lane_handoff(self):
        c = self.core()
        now = self.released_cache(c)
        for delta in (.05, .15):
            c.observe_sign('RIGHT', .99, now+delta, now+delta)
        self.assertEqual(c.next_direction, 'RIGHT')
        for i in range(1, 110):
            t = now+i*.1
            c.front_marker_stamp = t
            c.observe_lane([(.25, 0.), (.45, 0.), (.65, 0.)], .99, t)
            c.tick(t)
            if c.action is None:
                break
        self.assertEqual(c.state, 'LANE', c.reason)
        self.assertEqual(c.pending, 'RIGHT')

    def test_emergency_stop_still_blocks_new_route_votes(self):
        c = self.core()
        now = self.released_cache(c)
        c.estop = True
        self.assertEqual(c.tick(now+.1), (0, 0.))
        self.assertEqual(c.state, 'FAULT')
        for delta in (.15, .25):
            c.observe_sign('RIGHT', .99, now+delta, now+delta)
        self.assertIsNone(c.next_direction)

    def test_short_turn_releases_cache_without_an_extra_sign_timeout(self):
        c = self.core(fast=True)
        now = self.released_cache(c)
        self.assertLess(now, c.route_action_started+c.cfg['sign_timeout'])
        for delta in (.05, .15):
            c.observe_sign('RIGHT', .99, now+delta, now+delta)
        self.assertEqual(c.next_direction, 'RIGHT')
        for i in range(1, 60):
            t = now+i*.1
            c.front_marker_stamp = t
            c.observe_lane([(.25, 0.), (.45, 0.), (.65, 0.)], .99, t)
            c.tick(t)
            if c.action is None:
                break
        self.assertEqual(c.pending, 'RIGHT')


if __name__ == '__main__':
    unittest.main()
