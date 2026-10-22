"""Straight completion depends on distance or lane acquisition, not three seconds."""
import os
import unittest
from robot.common.config import load_config
from robot.master.controller import Controller


class StraightDistanceRestoreTests(unittest.TestCase):
    def test_elapsed_three_seconds_does_not_finish_before_distance(self):
        cfg = load_config(os.path.join(os.path.dirname(__file__), '../config'))
        self.assertNotIn('straight_duration_s', cfg)
        self.assertEqual(cfg['straight_distance'], 1.5)
        cfg.update(wait_green=False, lidar_enabled=False, straight_distance=1.4)
        c = Controller(cfg)
        self.addCleanup(c.close)
        c.start_follow([(0, 0, 0, 1, 0), (1.4, 0, 0, 1, 0)], 'STRAIGHT', 10.)
        c.action_source = 'sign'
        c.handoff_lane_confirmed = lambda now: False
        for t, x in ((13.1, .5), (14., 1.39)):
            c.set_pose((x, 0, 0), t)
            c.front_marker_stamp = t
            output = c.execute('turn', 'straight_search_tick', t).value
            self.assertEqual(output[0], 30)
            self.assertEqual(c.action, 'STRAIGHT')
        c.set_pose((1.4, 0, 0), 14.1)
        c.front_marker_stamp = 14.1
        c.execute('turn', 'straight_search_tick', 14.1)
        self.assertIsNone(c.action)


if __name__ == '__main__':
    unittest.main()
