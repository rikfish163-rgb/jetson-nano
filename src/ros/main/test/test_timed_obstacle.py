import os
import sys
import unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from timed_obstacle import TimedObstacle


class TimedObstacleTests(unittest.TestCase):
    def test_default_half_meter_trigger_and_five_second_turns(self):
        guard = TimedObstacle()
        guard.update([.51], 0, .01, .05, 8, 0)
        self.assertIsNone(guard.command(0))
        for now, steer in ((1, 22), (5.99, 22), (6, -22), (10.99, -22)):
            guard.update([.5], 0, .01, .05, 8, now)
            self.assertEqual(guard.command(now), dict(speed_raw=26, steering_raw=steer))
        guard.update([.5], 0, .01, .05, 8, 11)
        self.assertIsNone(guard.command(11))

    def test_three_second_turns_at_speed_26(self):
        guard = TimedObstacle(speed=26, turn_s=3)
        for now, steer in ((0, 22), (2.99, 22), (3, -22), (5.99, -22)):
            guard.update([.4], 0, .01, .05, 8, now)
            self.assertEqual(guard.command(now), dict(speed_raw=26, steering_raw=steer))
        guard.update([.4], 0, .01, .05, 8, 6)
        self.assertIsNone(guard.command(6))

    def test_sector_and_invalid_returns(self):
        guard = TimedObstacle()
        guard.update([2.0, 0.4], 0, 0.5, 0.05, 8, 0)
        self.assertIsNone(guard.command(0))
        guard.update([float('nan'), 0, float('inf')], 0, 0.01, 0.05, 8, 0)
        self.assertEqual(guard.command(0)['speed_raw'], 0)

    def test_left_right_resume_and_rearm(self):
        guard = TimedObstacle(turn_s=1)
        def tick(now, distance):
            guard.update([distance], 0, 0.01, 0.05, 8, now)
            return guard.command(now)
        self.assertEqual(tick(0, .4)['steering_raw'], 22)
        self.assertEqual(tick(.99, 2)['steering_raw'], 22)
        self.assertEqual(tick(1, 2)['steering_raw'], -22)
        self.assertEqual(tick(1.99, .4)['steering_raw'], -22)
        self.assertIsNone(tick(2, .4))
        self.assertIsNone(tick(3, .4))
        self.assertIsNone(tick(4, 2))
        self.assertEqual(tick(5, .4)['steering_raw'], 22)

    def test_missing_stale_scan_aborts_maneuver(self):
        guard = TimedObstacle()
        self.assertEqual(guard.command(0)['speed_raw'], 0)
        guard.update([.4], 0, .01, .05, 8, 0)
        guard.command(0)
        self.assertEqual(guard.command(.6)['speed_raw'], 0)
        guard.update([.4], 0, .01, .05, 8, .7)
        self.assertEqual(guard.command(.7)['speed_raw'], 0)

    def test_green_gate_and_lane_handoff(self):
        import test_parking_main_guard
        import legacy_controller
        controller = legacy_controller.AutoDriveController.__new__(
            legacy_controller.AutoDriveController)
        controller.maybe_latch_competition_start = lambda: None
        controller.competition_waiting_for_green = lambda: True
        controller.timed_obstacle = TimedObstacle(turn_s=1)
        controller.timed_obstacle.update([.4], 0, .01, .05, 8, 0)
        legacy_controller.rospy.get_time = lambda: 0
        self.assertEqual(controller.decide_control_mode(), 'waiting_for_green')
        self.assertIsNone(controller.timed_obstacle.started)
        controller.competition_waiting_for_green = lambda: False
        self.assertEqual(controller.decide_control_mode(), 'timed_obstacle')
        legacy_controller.rospy.get_time = lambda: 2
        controller.timed_obstacle.update([.4], 0, .01, .05, 8, 2)
        self.assertEqual(controller.decide_control_mode(), 'lane_following')


if __name__ == '__main__':
    unittest.main()
