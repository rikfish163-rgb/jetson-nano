"""RIGHT exits retain configured road speed without a hidden raw-12 cap."""
import copy
import unittest

from test_core import CONFIG
from robot.master.controller import Controller


class RightHandoffSpeedTests(unittest.TestCase):
    def make(self):
        cfg = copy.deepcopy(CONFIG)
        cfg.update(wait_green=False, lidar_enabled=False, right_timed_enabled=True,
                   straight_speed_raw=30, lane_curve_speed_raw=30,
                   right_timed_reverse_s=.1, right_timed_turn_s=.2,
                   right_timed_exit_reverse_s=.1,
                   right_timed_reverse_speed_raw=-30,
                   right_timed_turn_speed_raw=30,
                   right_timed_exit_reverse_speed_raw=-30)
        cfg['speed_raw']['lane'] = 30
        c = Controller(cfg)
        self.addCleanup(c.close)
        return c

    def finish_right(self, c):
        c.action = 'RIGHT'
        c.resume_lane()

    def test_handoff_retains_configured_output_stop_and_slower_commands(self):
        c = self.make()
        c._runtime.overrides['tick'] = lambda now: (30, .01)
        self.assertEqual(c.tick(0), (30, .01))
        self.finish_right(c)
        self.assertEqual(c.tick(1), (30, .01))
        for speed in (0, 8, -8, -30):
            c._runtime.overrides['tick'] = lambda now, speed=speed: (speed, .01)
            self.assertEqual(c.tick(2), (speed, .01))

    def test_second_right_approach_retains_configured_speed(self):
        c = self.make()
        self.finish_right(c)
        c.action, c.state = 'RIGHT', 'BLUE_APPROACH'
        c._runtime.overrides['tick'] = lambda now: (30, .01)
        self.assertEqual(c.tick(1), (30, .01))
        c.execute('mission', 'begin_blue_action', 2.)
        self.assertEqual(c.tick(2), (30, .01))
        self.finish_right(c)
        self.assertEqual(c.tick(3), (30, .01))
        self.finish_right(c)
        self.assertEqual(c.tick(4), (30, .01))

    def test_second_right_to_straight_preserves_blue_stage_speed(self):
        c = self.make()
        self.finish_right(c)
        self.finish_right(c)
        c._runtime.overrides['tick'] = lambda now: (30, .01)
        c.pending = 'STRAIGHT'
        self.assertEqual(c.tick(1), (30, .01))
        c.action, c.state = 'STRAIGHT', 'BLUE_APPROACH'
        c.blue_approach = dict(image_timing={})
        self.assertEqual(c.tick(2), (30, .01))
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

    def test_uturn_blue_entry_retains_its_command(self):
        c = self.make()
        self.finish_right(c)
        c.action, c.state = 'UTURN', 'BLUE_STOP'
        c.execute('mission', 'begin_blue_action', 1.)
        c._runtime.overrides['tick'] = lambda now: (26, .02)
        self.assertEqual(c.tick(2), (26, .02))

    def test_other_action_completion_does_not_start_or_clear_handoff(self):
        c = self.make()
        c._runtime.overrides['tick'] = lambda now: (30, 0.)
        c.action = 'UTURN'
        c.resume_lane()
        self.assertEqual(c.tick(0), (30, 0.))
        self.finish_right(c)
        c.action = 'BYPASS'
        c.resume_lane()
        self.assertEqual(c.tick(1), (30, 0.))

    def test_first_and_second_right_resume_lane_at_thirty(self):
        for completed in (0, 1):
            c = self.make()
            c.right_completed_count = completed
            c.action = 'RIGHT'
            c.execute('mission', 'begin_blue_action', 0.)
            c.right_lock.update(phase='WAIT_LANE', exit_started=1.)
            c.front_marker_stamp = 2.
            self.assertEqual(c.tick(2.), (0, 0.))
            for stamp, points in ((2.1,[(.25,-.05),(.45,-.09),(.65,-.13)]),
                                  (2.2,[(.25,0.),(.45,0.),(.65,0.)])):
                c.observe_lane(points,.99,stamp)
                c.front_marker_stamp = stamp
                speed, steer = c.tick(stamp)
                self.assertEqual(speed,30)
                if stamp == 2.1:
                    self.assertLess(steer,0.)

    def test_left_only_edge_with_left_heading_commands_right_at_thirty(self):
        c = self.make()
        c.action = 'RIGHT'
        c.execute('mission', 'begin_blue_action', 0.)
        c.right_lock.update(phase='ALIGN_LANE', exit_started=1.)
        c.observe_left_boundary([(x,.30-.2*x) for x in (.25,.45,.65)],2.)
        line = c.left_exit_line(2.)
        self.assertIsNotNone(line)
        speed,steer = c.left_reference_command(line,2.)
        self.assertEqual(speed,30)
        self.assertLess(steer,0.)

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
