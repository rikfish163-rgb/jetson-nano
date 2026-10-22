"""Topmost-point proportional steering through the configured chassis mapping."""
import os
import unittest
from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.master.controller import Controller
from vehicle_control.pure_pursuit import PurePursuit


class LegacyLaneControlTests(unittest.TestCase):
    def setUp(self):
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cfg = load_config(os.path.join(root, 'config'))
        cfg.update(wait_green=False, lidar_enabled=False, lookahead=.45,
                   steering_command_scale_rad=.1,lane_lateral_full_scale_m=.30)
        cfg['speed_raw']['lane'] = 20
        self.c = Controller(cfg)
        self.addCleanup(self.c.close)

    def test_pure_pursuit_remains_available_for_maneuvers(self):
        from robot.lane import controller
        self.assertIs(controller.PurePursuit, PurePursuit)

    def test_paths_and_raw_steering_match_topmost_offset_both_sides(self):
        paths = [[(.5,.1),(.8,.1)],[(.717,.311),(.81,.35),(1.,.4)],
                 [(.20,.01),(.40,.02),(.60,-.03),(.80,-.1)]]
        stamp = 1.0
        for path in paths:
            for sign in (-1,1):
                points = [(x,sign*y) for x,y in path]
                target=max(points,key=lambda p:p[0])
                expected=int(round(22*max(-1.,min(1.,target[1]/.30))))
                self.c.observe_lane(points,.9,stamp)
                speed,steer = self.c.lane_command(stamp)
                self.assertEqual(speed,20)
                self.assertEqual(encode_command(speed,steer,self.c.cfg,0)['steering_raw'],expected)
                stamp += .1

    def test_lost_path_stops_when_camera_stale(self):
        self.c.observe_lane([(.5,.1),(.8,.1)],.9,1.)
        self.c.tick(1.)
        self.assertEqual(self.c.tick(2.),(0,0.))

    def test_topmost_point_reaches_recorded_right_curve(self):
        # 2026-09-29 19:03:42: the near path is slightly left, then curves right.
        # A 0.45 m setting selects the first visible point and commands left.
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.c.cfg['lookahead'] = load_config(os.path.join(root,'config'))['lookahead']
        self.c.observe_lane([(.55,.009),(.65,-.024),(.75,-.056),(.85,-.084),
                             (.95,-.116),(1.05,-.138),(1.15,-.164)],.83,1.)
        speed,steer = self.c.lane_command(1.)
        raw = encode_command(speed,steer,self.c.cfg,0)['steering_raw']
        self.assertLessEqual(raw,-10)

    def test_reacquired_path_is_not_overridden_by_previous_steering(self):
        self.c.observe_lane([(.5,-.1),(.8,-.1)],.9,1.)
        self.c.tick(1.)
        self.c.observe_lane([],0.,1.1)
        self.c.tick(1.1)
        self.c.observe_lane([(.5,.1),(.8,.1)],.9,1.15)
        speed,steer = self.c.tick(1.15)
        self.assertGreater(encode_command(speed,steer,self.c.cfg,0)['steering_raw'],0)


if __name__ == '__main__':
    unittest.main()
