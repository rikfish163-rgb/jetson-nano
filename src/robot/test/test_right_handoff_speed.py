"""Every RIGHT hands off at raw 12 until the next calibrated action."""
import copy
import unittest

from test_core import CONFIG
from robot.master.controller import Controller


class RightHandoffSpeedTests(unittest.TestCase):
    def make(self):
        cfg = copy.deepcopy(CONFIG)
        cfg.update(wait_green=False, lidar_enabled=False, right_timed_enabled=True,
                   right_timed_reverse_s=.1, right_timed_turn_s=.2,
                   right_timed_exit_reverse_s=.1,
                   right_timed_reverse_speed_raw=-30,
                   right_timed_turn_speed_raw=30,
                   right_timed_exit_reverse_speed_raw=-30)
        c = Controller(cfg)
        self.addCleanup(c.close)
        return c

    def finish_right(self, c):
        c.action = 'RIGHT'
        c.resume_lane()

    def test_handoff_caps_output_and_retains_stop_and_slower_commands(self):
        c = self.make()
        c._runtime.overrides['tick'] = lambda now: (40, .01)
        self.assertEqual(c.tick(0), (40, .01))
        self.finish_right(c)
        self.assertEqual(c.tick(1), (12, .01))
        for speed in (0, 8, -8, -30):
            c._runtime.overrides['tick'] = lambda now, speed=speed: (speed, .01)
            self.assertEqual(c.tick(2), (max(-12, min(12, speed)), .01))

    def test_second_right_approach_stays_slow_until_maneuver_starts(self):
        c = self.make()
        self.finish_right(c)
        c.action, c.state = 'RIGHT', 'BLUE_APPROACH'
        c._runtime.overrides['tick'] = lambda now: (30, .01)
        self.assertEqual(c.tick(1), (12, .01))
        c.execute('mission', 'begin_blue_action', 2.)
        self.assertEqual(c.tick(2), (30, .01))
        self.finish_right(c)
        self.assertEqual(c.tick(3), (12, .01))
        self.finish_right(c)
        self.assertEqual(c.tick(4), (12, .01))

    def test_second_right_to_straight_keeps_alignment_slow(self):
        c = self.make()
        self.finish_right(c)
        self.finish_right(c)
        c._runtime.overrides['tick'] = lambda now: (30, .01)
        c.pending = 'STRAIGHT'
        self.assertEqual(c.tick(1), (12, .01))
        c.action, c.state = 'STRAIGHT', 'BLUE_APPROACH'
        c.blue_approach = dict(image_timing={})
        self.assertEqual(c.tick(2), (12, .01))
        # Once triggered, preserve the calibrated raw 20 / one-second advance.
        c.blue_approach['image_timing']['trigger'] = 3.
        c._runtime.overrides['tick'] = lambda now: (20, 0.)
        self.assertEqual(c.tick(3), (20, 0.))
        c.execute('mission', 'begin_blue_action', 4.)
        c._runtime.overrides['tick'] = lambda now: (30, 0.)
        self.assertEqual(c.tick(4), (30, 0.))
        c.resume_lane()
        self.assertEqual(c.tick(5), (30, 0.))

    def test_handoff_does_not_cap_obstacle_or_parking_actions(self):
        c = self.make()
        self.finish_right(c)
        c._runtime.overrides['tick'] = lambda now: (30, 0.)
        for state in ('MANEUVER', 'PARKING', 'TIMED_PARKING', 'UTURN'):
            c.state = state
            self.assertEqual(c.tick(1), (30, 0.))

    def test_uturn_blue_entry_clears_handoff(self):
        c = self.make()
        self.finish_right(c)
        c.action, c.state = 'UTURN', 'BLUE_STOP'
        c.execute('mission', 'begin_blue_action', 1.)
        self.assertFalse(c.right_handoff_slow)

    def test_other_action_completion_does_not_start_or_clear_handoff(self):
        c = self.make()
        c._runtime.overrides['tick'] = lambda now: (40, 0.)
        c.action = 'UTURN'
        c.resume_lane()
        self.assertEqual(c.tick(0), (40, 0.))
        self.finish_right(c)
        c.action = 'BYPASS'
        c.resume_lane()
        self.assertEqual(c.tick(1), (12, 0.))

    def test_both_right_timed_maneuvers_keep_their_normal_speeds(self):
        c = self.make()
        for start in (0., 2.):
            c.action = 'RIGHT'
            c.execute('mission', 'begin_blue_action', start)
            for offset, expected in ((0., -30), (.05, -30), (.1, 30),
                                     (.2, 30), (.31, -30), (.36, -30)):
                now = start + offset
                c.front_marker_stamp = now
                self.assertEqual(c.tick(now)[0], expected)
            self.finish_right(c)


if __name__ == '__main__':
    unittest.main()
