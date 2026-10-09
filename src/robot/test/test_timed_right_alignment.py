"""RIGHT exit alignment and continued search when the lane is unavailable."""
import math
import os
import unittest

from robot.common.config import load_config
from robot.master.controller import Controller


class TimedRightAlignmentTests(unittest.TestCase):
    def core(self):
        cfg = load_config(os.path.join(os.path.dirname(__file__), '../config'))
        cfg.update(wait_green=False, lidar_enabled=False, steering_command_scale_rad=.03)
        c = Controller(cfg)
        self.addCleanup(c.close)
        c.action = 'RIGHT'
        c.execute('mission', 'begin_blue_action', 0.)
        c.right_lock.update(phase='WAIT_LANE', exit_started=10.)
        return c

    def observe(self, c, stamp, heading=0., distance=.30):
        slope = math.tan(heading)
        c.observe_left_boundary([(x, slope*x + distance/math.cos(heading))
                                 for x in (.25, .45, .65)], stamp)
        c.observe_lane([(x, slope*x) for x in (.25, .45, .65)], .95, stamp)
        c.front_marker_stamp = stamp

    def test_sixty_degree_lane_gate_cannot_complete_a_misaligned_right(self):
        c = self.core()
        for stamp in (10.1, 10.2, 10.3):
            self.observe(c, stamp, heading=.35)
            command = c.tick(stamp)
        self.assertEqual(c.action, 'RIGHT')
        self.assertEqual(c.right_lock['phase'], 'ALIGN_LANE')
        self.assertGreater(command[0], 0)
        self.assertLessEqual(command[0], c.cfg['left_reference_speed_raw'])
        self.assertGreater(command[1], 0)

    def test_stable_alignment_keeps_tracking_and_accepts_the_second_right(self):
        c = self.core()
        for stamp in (10.1, 10.3):
            self.observe(c, stamp)
            c.tick(stamp)
            self.assertEqual(c.action, 'RIGHT')
        # Repeated control ticks are not additional image confirmations.
        c.tick(10.35)
        self.assertEqual(c.action, 'RIGHT')
        self.observe(c, 10.7)
        self.assertGreater(c.tick(10.7)[0], 0)
        self.assertEqual(c.action, 'RIGHT')
        self.assertFalse(c.follow_left_boundary)
        for stamp in (10.8, 10.9):
            c.observe_sign('RIGHT', .99, stamp, stamp)
        self.assertEqual(c.next_direction, 'RIGHT')
        c.cfg['blue_timed_enabled'] = False
        for stamp in (11., 11.1, 11.2):
            c.observe_ground(dict(source='front', part='markers', slots=[],
                markers=[dict(kind='junction', x=.3, y=0., length=.6)],
                blue_lines=[dict(x=.3, y=0., yaw=0., length=.6)]), stamp)
            c.tick(stamp)
        self.assertEqual(c.action, 'RIGHT')
        self.assertEqual(c.last_blue_trigger['action'], 'RIGHT')
        self.assertFalse(c.follow_left_boundary)

    def test_pre_completion_line_cannot_release_right(self):
        c = self.core()
        self.observe(c, 9.9)
        self.assertEqual(c.tick(10.1), (12,0.))
        self.assertEqual(c.action, 'RIGHT')
        self.assertEqual(c.exit_count, 0)

    def test_passed_second_sign_stays_cached_until_blue_even_without_lane(self):
        c = self.core()
        for stamp in (10.1, 10.3, 10.7):
            self.observe(c, stamp)
            c.tick(stamp)
        for stamp in (10.8, 10.9):
            c.observe_sign('RIGHT', .99, stamp, stamp)
        self.assertEqual(c.next_direction, 'RIGHT')
        # The board is behind the car well beyond the sign timeout. Blank
        # detector frames and absent lane paint must not erase its command.
        for stamp in (11., 12., 15.):
            c.observe_sign('', 0., stamp, stamp)
            c.observe_lane([], 0., stamp)
            c.observe_ground(dict(source='front', part='markers', slots=[],
                                  markers=[], blue_lines=[]), stamp)
            c.tick(stamp)
            self.assertEqual(c.next_direction, 'RIGHT')
        c.cfg['blue_timed_enabled'] = False
        for stamp in (15.1, 15.2, 15.3):
            c.observe_lane([], 0., stamp)
            c.observe_ground(dict(source='front', part='markers', slots=[],
                markers=[dict(kind='junction', x=.3, y=0., length=.6)],
                blue_lines=[dict(x=.3, y=0., yaw=0., length=.6)]), stamp)
            c.tick(stamp)
        self.assertEqual(c.action, 'RIGHT')
        self.assertEqual(c.last_blue_trigger['source'], 'cached_sign')
        self.assertNotEqual(c.reason, 'gap_limit_wait_for_lane')

    def test_aligned_curved_road_does_not_require_a_straight_exit(self):
        c = self.core()
        for stamp in (10.1, 10.3, 10.7):
            # The vehicle is tangent to the circle at x=0. A chord fitted
            # only ahead of the vehicle has a nonzero angle on this road.
            c.observe_lane([(x, .5*x*x) for x in (.25, .35, .45, .65)], .95, stamp)
            c.front_marker_stamp = stamp
            command = c.tick(stamp)
        self.assertEqual(c.action, 'RIGHT')
        self.assertEqual(c.reason, 'right_timed_exit_align')
        self.assertGreater(command[0], 0)
        self.assertFalse(c.follow_left_boundary)

    def test_left_line_alone_cannot_replace_the_normal_exit_center_path(self):
        c = self.core()
        for stamp in (10.1, 10.2, 10.7):
            c.observe_left_boundary([(.25, .3), (.45, .3), (.65, .3)], stamp)
            c.observe_lane([], 0., stamp)
            self.assertEqual(c.tick(stamp), (12,0.))
        self.assertEqual(c.action, 'RIGHT')

    def test_stale_lane_and_lidar_veto_completion(self):
        c = self.core()
        self.observe(c, 9.5)
        self.assertEqual(c.tick(10.1), (12,0.))
        self.assertEqual(c.action, 'RIGHT')
        self.assertEqual(c.exit_count, 0)
        c.checked_command = lambda *args: (0, 0)
        for stamp in (10.2, 10.4, 10.8):
            self.observe(c, stamp)
            self.assertEqual(c.tick(stamp)[0], 0)
        self.assertEqual(c.action, 'RIGHT')


if __name__ == '__main__':
    unittest.main()
