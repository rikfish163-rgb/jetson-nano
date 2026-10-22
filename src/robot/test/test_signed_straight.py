"""Signs select the next junction action, never a permanent lane override."""
import os
import sys
import unittest
import yaml
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))
from robot.master.controller import Controller
from test_straight_alignment import finish_blue_exit


class SignedStraightTests(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(ROOT, 'config', 'competition.yaml')) as stream:
            cfg = yaml.safe_load(stream)
        cfg.update(wait_green=False, lidar_enabled=False, sign_ttl=0,
                   straight_speed_raw=12, blue_default_straight=True)
        self.c = Controller(cfg)
        self.addCleanup(self.c.close)

    def front(self, t, x=None):
        self.c.observe_ground(dict(source='front', part='markers', slots=[],
            markers=[] if x is None else [dict(kind='junction', x=x, y=0)]), t)

    def cache(self):
        for t in (.1, .2):
            self.c.observe_sign('STRAIGHT', .99, t, t)

    def release(self):
        self.cache()
        self.front(1, .3)
        self.assertEqual(self.c.tick(1), (0, 0))
        self.assertEqual(self.c.tick(1.99), (0, 0))
        self.front(2)
        self.c.observe_lane([], 0, 2)
        self.assertEqual(self.c.tick(2), (12, 0))

    def test_cache_without_lane_stays_stopped(self):
        self.cache()
        self.front(.3, 1.2)
        self.assertEqual(self.c.tick(.3), (0, 0))
        self.assertEqual(self.c.pending, 'STRAIGHT')
        self.assertIsNone(self.c.action)

    def test_before_blue_still_follows_visible_curve(self):
        self.cache()
        self.front(.3, 1.2)
        self.c.observe_lane([(.3, .03), (.5, .08), (.7, .12)], .9, .3)
        speed, steer = self.c.tick(.3)
        self.assertEqual(speed, self.c.cfg['speed_raw']['lane'])
        self.assertGreater(steer, 0)

    def test_next_blue_releases_to_lane(self):
        self.release()
        for t in (2.1, 2.2, 2.3):
            self.c.observe_lane([(.3, 0), (.5, 0), (.7, 0)], .9, t)
            self.c.tick(t)
        self.assertEqual(self.c.state,'MANEUVER')
        self.c.set_pose((.7,0,0),2.31)
        self.front(2.31,.3)
        finish_blue_exit(self.c,2.31)
        self.assertEqual(self.c.state, 'LANE')
        self.assertIsNone(self.c.action)
        self.assertIsNone(self.c.pending)
        self.assertIsNone(self.c.follower)
        self.assertEqual(self.c.last_completed_action, 'STRAIGHT')

    def test_next_sign_is_ignored_until_exit(self):
        self.release()
        for t in (2.1, 2.2):
            self.c.observe_sign('LEFT', .99, t, t)
        self.assertIsNone(self.c.next_direction)
        for t in (2.3, 2.4, 2.5):
            self.c.observe_lane([(.3, 0), (.5, 0), (.7, 0)], .9, t)
            self.c.tick(t)
        self.c.set_pose((.7,0,0),2.51)
        self.front(2.51,.3)
        finish_blue_exit(self.c,2.51)
        self.assertIsNone(self.c.pending)
        self.assertIsNone(self.c.action)

    def test_same_blue_cannot_restart_completed_straight(self):
        self.test_next_blue_releases_to_lane()
        self.front(2.8, -.11)
        self.c.tick(2.8)
        self.assertIsNone(self.c.action)

    def test_missing_exit_has_bounded_search(self):
        self.release()
        self.c.observe_lane([], 0, 23)
        self.assertEqual(self.c.tick(23), (0, 0))
        self.assertEqual(self.c.state, 'FAULT')

    def test_camera_stale_stops_search(self):
        self.release()
        self.assertEqual(self.c.tick(3.3), (0, 0))
        self.assertEqual(self.c.reason, 'straight_front_stale')

    def test_red_and_estop_override_search(self):
        self.release()
        self.c.red = True
        self.assertEqual(self.c.tick(2.1), (0, 0))
        self.c.red = False
        self.c.estop = True
        self.assertEqual(self.c.tick(2.1), (0, 0))


if __name__ == '__main__':
    unittest.main()
