"""Retired timer behavior is rejected; first turn uses geometry and vision."""
import unittest
from test_uturn_rear_sequence import RearUturnTests
from robot.common.contracts import validate_config


class TimedEntryTests(unittest.TestCase):
    def setUp(self):
        fixture = RearUturnTests('test_white_lane_alone_cannot_align_uturn')
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        fixture.start()
        self.c, self.ground = fixture.c, fixture.ground

    def test_elapsed_time_alone_does_not_finish_turn(self):
        for stamp in (1.4, 2, 3, 5):
            self.ground(stamp, [], 'front')
            self.c.tick(stamp)
            self.assertEqual(self.c.uturn['phase'], 'LEFT_FIRST')

    def test_planned_exit_needs_fresh_visual_confirmation(self):
        self.c.pose = self.c.exit_pose
        for stamp in (1.4, 1.5, 1.6):
            self.ground(stamp, [], 'front')
            self.c.observe_lane([(.3,0),(.5,0),(.7,0)], .9, stamp)
            self.c.tick(stamp)
        self.assertEqual(self.c.uturn['phase'], 'ALIGN_FIRST')

    def test_red_stops_first_turn(self):
        self.c.red = True
        self.ground(2, [], 'front')
        self.assertEqual(self.c.tick(2), (0,0))
        self.assertEqual(self.c.uturn['phase'], 'LEFT_FIRST')

    def test_retired_turn_duration_rejected_even_if_positive(self):
        for value in (0, -1, 2, float('nan'), 20):
            with self.assertRaises(ValueError):
                validate_config(dict(self.c.cfg, uturn_initial_turn_s=value))


if __name__ == '__main__':
    unittest.main()
