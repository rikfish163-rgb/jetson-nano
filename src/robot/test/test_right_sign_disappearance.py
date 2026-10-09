"""A fresh frame without the first RIGHT rearms the next board; no hardware."""
import os
import unittest

from robot.common.config import load_config
from robot.master.controller import Controller


class RightSignDisappearanceTests(unittest.TestCase):
    def core(self, active=True):
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cfg = load_config(os.path.join(root, 'config'))
        cfg.update(wait_green=False, lidar_enabled=False)
        c = Controller(cfg)
        self.addCleanup(c.close)
        for t in (1., 1.1):
            c.observe_sign('RIGHT', .99, t, t)
        if active:
            c.action, c.state, c.pending = 'RIGHT', 'BLUE_STOP', None
            c.action_source = 'sign'
            c.execute('mission', 'begin_blue_action', 1.2)
        return c

    def test_one_fresh_empty_frame_releases_before_timeout(self):
        c = self.core()
        c.observe_sign('', 0., 1.3, 1.3)
        self.assertTrue(c.route_sign_rearmed)
        self.assertEqual(c.sign_info['decision'], 'right_sign_disappeared')
        self.assertEqual(c.action, 'RIGHT')
        for t in (1.4, 1.5):
            c.observe_sign('RIGHT', .93, t, t)
        self.assertEqual(c.next_direction, 'RIGHT')
        c.resume_lane()
        self.assertEqual(c.pending, 'RIGHT')

    def test_disappearance_preserves_pending_action(self):
        c = self.core(active=False)
        c.observe_sign('', 0., 1.2, 1.2)
        self.assertEqual(c.pending, 'RIGHT')
        self.assertEqual(c.pending_at, 1.1)
        self.assertTrue(c.route_sign_rearmed)
        c.action, c.state, c.pending = 'RIGHT', 'BLUE_STOP', None
        c.action_source = 'sign'
        c.execute('mission', 'begin_blue_action', 1.3)
        for t in (1.4, 1.5):
            c.observe_sign('RIGHT', .93, t, t)
        self.assertEqual(c.next_direction, 'RIGHT')

    def test_continuous_board_does_not_release_on_turn_timer(self):
        c = self.core()
        for i in range(70):
            t = 1.3+i*.1
            c.observe_sign('RIGHT', .93, t, t)
            c.front_marker_stamp = t
            c.tick(t)
            self.assertIsNone(c.next_direction)
            self.assertFalse(c.route_sign_rearmed)

    def test_source_gap_alone_does_not_mean_board_disappeared(self):
        c = self.core()
        for t in (3., 3.1):
            c.observe_sign('RIGHT', .93, t, t)
        self.assertIsNone(c.next_direction)
        self.assertFalse(c.route_sign_rearmed)

    def test_visible_low_confidence_right_does_not_release(self):
        c = self.core()
        c.observe_sign('', .65, 1.3, 1.3, right_visible=True)
        c.observe_sign('LEFT', .95, 1.4, 1.4, right_visible=True)
        self.assertFalse(c.route_sign_rearmed)

    def test_explicit_absence_with_another_candidate_releases(self):
        c = self.core()
        c.observe_sign('', .65, 1.3, 1.3, right_visible=False)
        self.assertTrue(c.route_sign_rearmed)

    def test_stale_empty_frame_cannot_release_or_vote(self):
        c = self.core()
        c.observe_sign('', 0., 1.05, 1.3)
        c.observe_sign('', 0., 1.2, 10.)
        self.assertFalse(c.route_sign_rearmed)
        c.observe_sign('', 0., 10., 10.)
        c.observe_sign('RIGHT', .93, 9.99, 10.01)
        self.assertIsNone(c.next_direction)
        self.assertEqual(c.sign_info['decision'], 'stale_or_duplicate')

    def test_same_board_still_visible_after_handoff_cannot_revote(self):
        c = self.core()
        c.resume_lane()
        for t in (3., 3.1):
            c.observe_sign('RIGHT', .93, t, t)
        self.assertIsNone(c.pending)
        c.observe_sign('', 0., 3.2, 3.2)
        for t in (3.3, 3.4):
            c.observe_sign('RIGHT', .93, t, t)
        self.assertEqual(c.pending, 'RIGHT')

    def test_queued_instruction_survives_its_board_disappearing(self):
        c = self.core()
        c.observe_sign('', 0., 1.3, 1.3)
        for t in (1.4, 1.5):
            c.observe_sign('RIGHT', .93, t, t)
        c.observe_sign('', 0., 1.6, 1.6)
        self.assertEqual(c.next_direction, 'RIGHT')
        c.resume_lane()
        self.assertEqual(c.pending, 'RIGHT')

    def test_red_and_estop_still_take_priority(self):
        c = self.core()
        c.observe_sign('', 0., 1.3, 1.3)
        c.observe_sign('RED', .99, 1.4, 1.4)
        self.assertTrue(c.red)
        self.assertEqual(c.tick(1.4), (0, 0.))
        c.estop = True
        c.tick(1.5)
        for t in (1.6, 1.7):
            c.observe_sign('RIGHT', .93, t, t)
        self.assertIsNone(c.next_direction)


if __name__ == '__main__':
    unittest.main()
