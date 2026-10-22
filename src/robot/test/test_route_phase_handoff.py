"""Route signs survive blue-stop/maneuver phase changes; no ROS or actuators."""
import os
import unittest

from robot.common.config import load_config
from robot.master.controller import Controller


class RoutePhaseHandoffTests(unittest.TestCase):
    def core(self):
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cfg = load_config(os.path.join(root, 'config'))
        cfg.update(wait_green=False, lidar_enabled=False)
        c = Controller(cfg)
        self.addCleanup(c.close)
        return c

    def vote(self, c, label, start):
        for stamp in (start, start + .1):
            c.observe_sign(label, .99, stamp, stamp)

    def blue(self, c, action):
        self.vote(c, action, 1.)
        c.state, c.action, c.pending = 'BLUE_STOP', action, None
        c.action_source, c.action_started = 'sign', 2.

    def test_right_queued_before_uturn_motion_survives_handoff(self):
        c = self.core()
        self.blue(c, 'UTURN')
        self.vote(c, 'RIGHT', 3.6)
        self.assertEqual(c.next_direction, 'RIGHT')
        c.execute('mission', 'begin_blue_action', 4.)
        self.assertEqual(c.action_started, 4.)
        c.resume_lane()
        self.assertEqual(c.pending, 'RIGHT')
        self.assertAlmostEqual(c.pending_at, 3.7)

    def test_vote_sequence_can_cross_blue_stop_to_motion(self):
        c = self.core()
        self.blue(c, 'LEFT')
        c.observe_sign('RIGHT', .99, 3.9, 3.9)
        c.execute('mission', 'begin_blue_action', 4.)
        c.observe_sign('RIGHT', .99, 4.1, 4.1)
        self.assertEqual(c.next_direction, 'RIGHT')
        c.resume_lane()
        self.assertEqual(c.pending, 'RIGHT')

    def test_continuously_visible_current_right_is_not_a_second_board(self):
        c = self.core()
        self.blue(c, 'RIGHT')
        for stamp in (1.2, 1.5, 2., 2.5, 3., 3.6, 3.9):
            c.observe_sign('RIGHT', .99, stamp, stamp)
        self.assertIsNone(c.next_direction)
        c.execute('mission', 'begin_blue_action', 4.)
        for stamp in (4.1, 4.5, 5., 5.6):
            c.observe_sign('RIGHT', .99, stamp, stamp)
        self.assertIsNone(c.next_direction)

    def test_second_right_after_visibility_gap_remains_accepted(self):
        c = self.core()
        self.blue(c, 'RIGHT')
        for stamp in (1.2, 1.5, 2., 2.5, 3.):
            c.observe_sign('RIGHT', .99, stamp, stamp)
        c.execute('mission', 'begin_blue_action', 4.)
        c.observe_sign('', 0., 4.1, 4.1)
        self.vote(c, 'RIGHT', 4.7)
        self.assertEqual(c.next_direction, 'RIGHT')
        c.resume_lane()
        self.assertEqual(c.pending, 'RIGHT')

    def test_visible_pending_right_reports_retained_command(self):
        c = self.core()
        self.vote(c, 'RIGHT', 1.)
        accepted_at = c.pending_at
        c.observe_sign('RIGHT', .99, 1.3, 1.3)
        self.assertEqual(c.pending, 'RIGHT')
        self.assertEqual(c.pending_at, accepted_at)
        self.assertEqual(c.sign_info['decision'], 'pending_already_stored')
        self.assertIsNone(c.next_direction)

if __name__ == '__main__':
    unittest.main()
