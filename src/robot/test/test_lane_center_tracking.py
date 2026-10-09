"""Measured center tracking at raw 24, without an indefinite bypass exit wait."""
import math
import os
import unittest
from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.master.controller import Controller
from test_continuous_obstacles import _SyntheticScan


class LaneCenterTrackingTests(unittest.TestCase):
    def setUp(self):
        cfg = load_config(os.path.join(os.path.dirname(__file__), '../config'))
        cfg.update(wait_green=False, lidar_enabled=False, lane_curvature_preview=True,
                   steering_command_scale_rad=.03, lane_curve_speed_raw=24)
        cfg['speed_raw'].update(lane=24, action=24)
        self.c = Controller(cfg)
        self.addCleanup(self.c.close)

    def command(self, path, stamp):
        self.c.observe_lane(path, .9, stamp)
        speed, steer = self.c.tick(stamp)
        return speed, encode_command(speed, steer, self.c.cfg, 0)['steering_raw']

    def test_centered_model_circle_does_not_oversteer_to_full_lock(self):
        radius = .65
        path = [(x, math.sqrt(radius*radius-x*x)-radius)
                for x in (.25,.35,.45,.50,.55)]
        speed, raw = self.command(path, 1.)
        self.assertEqual(speed, 24)
        self.assertTrue(-21 <= raw <= -16, raw)
        mirror = Controller(self.c.cfg)
        self.addCleanup(mirror.close)
        mirror.observe_lane([(x,-y) for x,y in path], .9, 2.)
        speed, steer = mirror.tick(2.)
        self.assertEqual(encode_command(speed,steer,mirror.cfg,0)['steering_raw'], -raw)

    def test_small_exit_correction_is_not_overridden_by_old_right_lock(self):
        self.command([(.5,0),(.7,-.04),(.9,-.12),(1.1,-.24)], 1.)
        for i in range(1,21):
            speed, raw = self.command([(.5,.05),(.7,.05),(.9,.05),(1.1,.05)], 1.+i*.05)
        self.assertEqual(speed, 24)
        self.assertGreater(raw, 0)
        self.assertIsNone(self.c.lane_curve_lock)

    def test_noisy_target_cannot_jump_between_full_lock_and_recentering(self):
        previous = self.command([(.5,0),(.7,-.08),(.9,-.16)], 1.)[1]
        for i in range(1,9):
            offset = .10 if i%2 else -.10
            raw = self.command([(.5,offset),(.7,offset),(.9,offset)], 1.+i*.05)[1]
            self.assertLessEqual(abs(raw-previous), 5)
            previous = raw

    def test_two_six_bypass_hands_to_valid_unaligned_center_without_stopping(self):
        self.c.cfg.update(lidar_enabled=True, timed_bypass_enabled=True)
        self.c.state = 'TIMED_BYPASS'
        self.c.action = 'BYPASS'
        self.c.timed_bypass = dict(phase='RIGHT', elapsed_s=5.95, last=1.,
            trigger_point=(.8,0), trigger_world=(.8,0))
        self.c.scan = _SyntheticScan(1.05)
        # The center is visible but deliberately not yet aligned with the car.
        speed, raw = self.command([(.5,.25),(.7,.30),(.9,.35)], 1.05)
        self.assertEqual(self.c.state, 'LANE')
        self.assertTrue(self.c.timed_bypass_completed)
        self.assertEqual(speed, 24)
        self.assertGreater(raw, 0)

    def test_bypass_end_with_no_visual_path_still_stops(self):
        self.c.cfg.update(lidar_enabled=True, timed_bypass_enabled=True)
        self.c.state = 'TIMED_BYPASS'
        self.c.action = 'BYPASS'
        self.c.timed_bypass = dict(phase='RIGHT', elapsed_s=5.95, last=1.)
        self.c.scan = _SyntheticScan(1.05)
        self.assertEqual(self.command([], 1.05)[0], 0)

    def test_model_circle_stays_near_center_with_small_camera_jitter(self):
        from robot.common.geometry import bicycle, local
        from robot.common.contracts import command_to_model_steering
        radius = .65
        self.c.set_pose((0.,-.03,0.), .9)
        errors = []
        for i in range(200):
            now = 1.+i*.05
            pose = self.c.pose
            angle = math.atan2(pose[0],radius+pose[1])
            points = [local(pose,(radius*math.sin(angle+a),
                       radius*(math.cos(angle+a)-1))) for a in
                      (.45,.50,.55,.60,.65,.70,.75,.80,.85,.90,.95,1.)]
            jitter = .002 if i%2 else -.002
            self.c.observe_lane([(x,y+jitter) for x,y in points],.9,now)
            speed,steer = self.c.tick(now)
            self.assertEqual(speed,24)
            self.c.set_pose(bicycle(pose,speed*.008*.05,
                command_to_model_steering(steer,self.c.cfg),.26),now)
            errors.append(abs(math.hypot(self.c.pose[0],radius+self.c.pose[1])-radius))
        # Synthetic calibrated bicycle evidence, not a measured field result.
        self.assertLess(max(errors),.08)
        self.assertLess(sum(errors[-50:])/50.,.05)


if __name__ == '__main__':
    unittest.main()
