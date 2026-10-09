"""RIGHT exit uses coherent sparse paint and bridges brief reference loss."""
import os
import unittest

from robot.common.config import load_config
from robot.master.controller import Controller


class RightExitReferenceLossTests(unittest.TestCase):
    def core(self):
        cfg = load_config(os.path.join(os.path.dirname(__file__), '../config'))
        cfg.update(wait_green=False, lidar_enabled=False,
                   straight_speed_raw=30, lane_curve_speed_raw=30,
                   steering_command_scale_rad=.03)
        cfg['speed_raw']['lane'] = 30
        c = Controller(cfg)
        self.addCleanup(c.close)
        c.action = 'RIGHT'
        c.execute('mission', 'begin_blue_action', 0.)
        c.right_lock.update(phase='WAIT_LANE', exit_started=10.)
        return c

    def lane(self, c, stamp, confidence=.95, points=None):
        if points is None:
            points = [(.25, -.075), (.45, -.135), (.65, -.195)]
        c.observe_lane(points, confidence, stamp)
        c.front_marker_stamp = stamp
        command = c.tick(stamp)
        c.issued_steer = command[1]
        return command

    def test_recorded_confidence_dip_still_tracks_coherent_geometry(self):
        c = self.core()
        for stamp, confidence in ((10.1, .95), (10.2, .323), (10.3, .95),
                                  (10.4, .323)):
            speed, steer = self.lane(c, stamp, confidence)
            self.assertEqual(speed, 30)
            self.assertLess(steer, 0.)
            self.assertEqual(c.reason, 'right_timed_exit_align')

    def test_far_points_do_not_reject_a_coherent_near_reference(self):
        c = self.core()
        points = [(.25, -.075), (.45, -.135), (.65, -.195), (1.8, .5)]
        speed, steer = self.lane(c, 10.1, .323, points)
        self.assertEqual(speed, 30)
        self.assertLess(steer, 0.)
        self.assertEqual(c.reason, 'right_timed_exit_align')

    def test_brief_empty_frame_keeps_last_accepted_steering(self):
        c = self.core()
        previous = self.lane(c, 10.1)
        self.assertLess(previous[1], 0.)
        for stamp in (10.2, 10.4, 10.9):
            self.assertEqual(self.lane(c, stamp, 0., []), previous)
            self.assertEqual(c.reason, 'right_timed_exit_hold_reference')
        self.assertEqual(c.right_lock['exit_track_stamp'], 10.1)

    def test_bad_geometry_cannot_replace_recent_valid_direction(self):
        c = self.core()
        previous = self.lane(c, 10.1)
        bad = [(.25, 0.), (.45, .6), (.65, 0.)]
        self.assertEqual(self.lane(c, 10.2, .323, bad), previous)
        self.assertEqual(c.reason, 'right_timed_exit_hold_reference')

    def test_duplicate_frames_do_not_extend_reference_lifetime(self):
        c = self.core()
        self.lane(c, 10.1)
        for now in (10.2, 10.3, 10.5):
            c.front_marker_stamp = now
            c.tick(now)
        self.assertEqual(c.right_lock['exit_track_stamp'], 10.1)

    def test_hold_still_obeys_obstacle_veto(self):
        c = self.core()
        self.lane(c, 10.1)
        c.checked_command = lambda *args: (0, 0.)
        self.assertEqual(self.lane(c, 10.2, 0., []), (0, 0.))

    def test_fresh_lane_resumes_correction_after_loss(self):
        c = self.core()
        self.lane(c, 10.1)
        self.lane(c, 10.2, 0., [])
        command = self.lane(c, 10.3, .323)
        self.assertEqual(command[0], 30)
        self.assertEqual(c.reason, 'right_timed_exit_align')
        self.assertEqual(c.right_lock['exit_track_stamp'], 10.3)

    def test_long_loss_continues_configured_speed_without_old_steer(self):
        c = self.core()
        self.lane(c, 10.1)
        for stamp in (11.11, 15., 30.):
            self.assertEqual(self.lane(c, stamp, 0., []), (30, 0.))
            self.assertEqual(c.reason, 'right_timed_exit_search_straight')
            self.assertEqual(c.state, 'MANEUVER')
        self.assertEqual(c.right_lock['exit_track_stamp'], 10.1)

    def test_entry_does_not_reuse_timed_turn_or_old_reference(self):
        c = self.core()
        c.issued_steer = -.03
        self.assertEqual(self.lane(c, 10.1, 0., []), (30, 0.))
        self.assertNotIn('exit_track_stamp', c.right_lock)


if __name__ == '__main__':
    unittest.main()
