"""Command estimates must account for feedback intervals, not timer cadence."""
from __future__ import division
import unittest
from robot.common.geometry import bicycle, distance, wrap
from robot.motion.odometry import CommandOdometry


class CommandOdometryTests(unittest.TestCase):
    def test_delayed_tick_keeps_all_fresh_motion_time(self):
        odom = CommandOdometry((0.,0.,0.), .26)
        odom.observe(10., .16, 0.)
        # A busy control timer must not discard the last 0.10 s.
        self.assertAlmostEqual(odom.estimate(10.2)[0], .032)

    def test_125m_does_not_depend_on_control_timer_frequency(self):
        odom = CommandOdometry((0.,0.,0.), .26)
        for i in range(157):
            odom.observe(10.+i*.05, .16, 0.)
        self.assertAlmostEqual(odom.estimate(17.8125)[0], 1.25)

    def test_new_command_does_not_apply_to_previous_interval(self):
        odom = CommandOdometry((0.,0.,0.), .26)
        odom.observe(10., .16, 0.)
        odom.observe(10.1, 0., 0.)
        self.assertAlmostEqual(odom.estimate(10.2)[0], .016)

    def test_late_stop_corrects_previous_extrapolation(self):
        odom = CommandOdometry((0.,0.,0.), .26)
        odom.observe(10., .16, 0.)
        self.assertAlmostEqual(odom.estimate(10.15)[0], .024)
        odom.observe(10.1, 0., 0.)
        self.assertAlmostEqual(odom.estimate(10.2)[0], .016)

    def test_no_motion_before_first_feedback_and_after_timeout(self):
        odom = CommandOdometry((0.,0.,0.), .26)
        self.assertEqual(odom.estimate(10.1), (0.,0.,0.))
        odom.observe(10.1, .16, 0.)
        self.assertAlmostEqual(odom.estimate(10.2)[0], .016)
        self.assertAlmostEqual(odom.estimate(11.)[0], .04)
        odom.observe(11., .16, 0.)
        self.assertAlmostEqual(odom.estimate(11.1)[0], .056)

    def test_repeated_and_out_of_order_feedback_is_rejected(self):
        odom = CommandOdometry((0.,0.,0.), .26)
        odom.observe(10., .16, 0.)
        for stamp in (10., 9.9):
            with self.assertRaises(ValueError): odom.observe(stamp, 1., .2)
        self.assertAlmostEqual(odom.estimate(10.1)[0], .016)

    def test_invalid_feedback_at_same_stamp_can_freeze_estimate(self):
        odom = CommandOdometry((0.,0.,0.), .26)
        odom.observe(10., .16, 0.)
        odom.stop(10.)
        self.assertEqual(odom.estimate(10.2), (0.,0.,0.))

    def test_reverse_yaw_integrates_each_command_mapping(self):
        odom = CommandOdometry((0.,0.,0.), .26)
        odom.observe(10., .25, .36)
        odom.observe(10.2, -.24, -.19)
        expected = bicycle(bicycle((0.,0.,0.), .05, .36, .26), -.048, -.19, .26)
        actual = odom.estimate(10.4)
        self.assertLess(distance(actual, expected), 1e-9)
        self.assertLess(abs(wrap(actual[2]-expected[2])), 1e-9)


if __name__ == '__main__': unittest.main()
