"""Regressions from standalone-lane run 1790715962--1790716037."""
import os
import unittest
from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.master.controller import Controller


class LaneMessageStabilityTests(unittest.TestCase):
    def setUp(self):
        cfg = load_config(os.path.join(os.path.dirname(__file__), '../config'))
        cfg.update(wait_green=False, lidar_enabled=False, parking_enabled=False,
                   lane_curvature_preview=True, steering_command_scale_rad=.03,
                   lookahead=.8, lane_curve_speed_raw=12)
        self.c = Controller(cfg)
        self.addCleanup(self.c.close)

    def command(self, points, stamp, confidence=.9, now=None):
        self.c.observe_lane(points, confidence, stamp)
        command = self.c.tick(stamp if now is None else now)
        return command[0], encode_command(command[0], command[1], self.c.cfg, 0)['steering_raw']

    def test_inside_bend_accepts_countersteer_instead_of_full_right(self):
        self.command([(.5,0),(.7,-.04),(.9,-.12),(1.1,-.24)], 1.)
        # Fresh measured path is now to the left of the car in the same bend.
        speed, raw = self.command([(.6,.09),(.7,.08),(.8,.06),(.9,.03)], 1.1)
        self.assertGreater(speed, 0)
        self.assertGreater(raw, 0)
        self.assertEqual(self.c.state, 'LANE')

    def test_exit_reduces_angle_on_first_fresh_centered_path(self):
        self.command([(.5,0),(.7,-.04),(.9,-.12),(1.1,-.24)], 1.)
        self.assertEqual(self.command([(.5,0),(.7,0),(.9,0),(1.1,0)], 1.1)[1], 0)

    def test_near_error_is_corrected_even_when_farthest_point_is_centered(self):
        self.assertLess(self.command([(.5,-.15),(.65,-.105),(.8,-.06),(1.,0)], 1.)[1], 0)

    def test_small_confidence_dip_has_bounded_grace_without_refreshing_it(self):
        path = [(.5,.02),(.7,.02),(.9,.02),(1.1,.02)]
        self.command(path, 1.)
        origin = self.c.gap_start
        self.assertGreater(self.command(path, 1.1, .327)[0], 0)
        self.assertEqual(self.c.gap_start, origin)
        self.assertEqual(self.command(path, 1.3, .327)[0], 0)
        self.assertGreater(self.command(path, 1.4)[0], 0)

    def test_severe_confidence_drop_and_camera_loss_still_stop(self):
        path = [(.5,0),(.7,0),(.9,0),(1.1,0)]
        self.command(path, 1.)
        self.assertEqual(self.command(path, 1.1, .1)[0], 0)
        self.assertEqual(self.command(path, 1.1, now=1.7)[0], 0)

    def test_gap_cannot_hold_an_old_turn_for_seconds(self):
        self.command([(.5,0),(.7,-.04),(.9,-.12),(1.1,-.24)], 1.)
        self.assertGreater(self.command([], 1.1)[0], 0)
        self.assertEqual(self.command([], 1.4)[0], 0)

    def test_centered_circle_uses_bicycle_angle_and_mirrors(self):
        import math
        radius = .65
        points = [(x,math.sqrt(radius*radius-x*x)-radius) for x in (.25,.35,.45,.5,.55)]
        right = self.command(points, 1.)[1]
        expected = int(round(-math.atan(self.c.cfg['wheelbase']/radius)/self.c.cfg['max_steer']*22))
        self.assertEqual(right, expected)
        self.assertEqual(self.command([(x,-y) for x,y in points], 1.1)[1], -right)

    def test_obstacle_and_stale_scan_override_confidence_grace(self):
        import math
        from robot.lidar.scan import Scan
        path = [(.5,0),(.7,0),(.9,0),(1.1,0)]
        self.command(path, 1.)
        self.c.cfg['lidar_enabled'] = True
        self.c.cfg['timed_bypass_enabled'] = False
        ranges = [float('inf')]*360
        ranges[180] = .4
        self.c.scan = Scan(ranges,-math.pi,math.pi/180,.05,6.,self.c.pose,self.c.cfg['lidar'],1.1)
        self.assertEqual(self.command(path, 1.1, .327)[0], 0)
        self.assertEqual(self.c.reason,'lidar_obstacle_in_sweep')
        self.assertEqual(self.command(path, 2.)[0], 0)
        self.assertEqual(self.c.reason,'scan_missing_or_stale')

    def test_enabled_bypass_hands_back_to_continuous_center_control(self):
        from test_continuous_obstacles import _SyntheticScan
        self.c.cfg.update(lidar_enabled=True,timed_bypass_enabled=True,
            timed_bypass_trigger_distance_m=.85,timed_bypass_settle_s=.1,
            timed_bypass_left_s=.2,timed_bypass_right_s=.2)
        path = [(.5,0),(.7,0),(.9,0),(1.1,0)]
        self.c.scan = _SyntheticScan(1.,obstacles=[(.6,0.)])
        self.command(path,1.)
        self.assertEqual(self.c.state,'TIMED_BYPASS')
        phases = set([self.c.timed_bypass['phase']])
        for i in range(1,12):
            stamp = 1.+i*.1
            self.c.scan = _SyntheticScan(stamp)
            speed,raw = self.command(path,stamp)
            if self.c.timed_bypass:
                phases.add(self.c.timed_bypass['phase'])
        self.assertTrue(set(['SETTLE_LEFT','LEFT','RIGHT']).issubset(phases))
        self.assertTrue(self.c.timed_bypass_completed)
        self.assertEqual(self.c.state,'LANE')
        self.assertGreater(speed,0)
        self.assertEqual(raw,0)
