"""Green release follows observed centers without a command-distance delay."""
import os
import unittest
from robot.common.config import load_config
from robot.common.contracts import encode_command, validate_config
from robot.master.controller import Controller


class GreenVisualStartTests(unittest.TestCase):
    def core(self):
        cfg=load_config(os.path.join(os.path.dirname(__file__),'../config'))
        cfg.update(wait_green=True,lidar_enabled=False,parking_enabled=False,
                   startup_follow_lane=True,lane_curvature_preview=True,
                   steering_command_scale_rad=.03,straight_speed_raw=24)
        cfg['speed_raw']['lane']=12
        validate_config(cfg)
        c=Controller(cfg)
        self.addCleanup(c.close)
        return c

    def release(self,c):
        for i in range(int(c.cfg['sign_votes'])):
            c.observe_sign('GREEN',.99,1.+i*.01,1.+i*.01)

    def test_green_takes_both_curves_before_estimated_distance(self):
        for sign in (-1,1):
            c=self.core()
            c.observe_lane([(.5,0),(.7,sign*.04),(.9,sign*.12),(1.1,sign*.24)],.9,1.1)
            self.assertEqual(c.tick(1.1),(0,0))
            self.release(c)
            speed,steer=c.tick(1.2)
            self.assertEqual(speed,12)
            self.assertGreater(sign*encode_command(speed,steer,c.cfg,0)['steering_raw'],0)
            self.assertEqual(c.state,'LANE')
            self.assertEqual(c.pose[0],0)
            self.assertIsNone(c.startup_started)

    def test_no_initial_or_stale_lane_cannot_drive_straight(self):
        for path,stamp,confidence in (([],1.1,.9),([(.5,0),(.9,0)],0,.9),
                                     ([(.5,0),(.9,0)],1.1,.1)):
            c=self.core()
            c.observe_lane(path,confidence,stamp)
            self.release(c)
            self.assertEqual(c.tick(1.2)[0],0)
            self.assertNotEqual(c.state,'STARTUP_STRAIGHT')

    def test_straight_then_bend_stays_slow_without_pose_progress(self):
        c=self.core()
        self.release(c)
        c.observe_lane([(.5,0),(.7,0),(.9,0),(1.1,0)],.9,1.1)
        self.assertEqual(c.tick(1.1),(12,0))
        for i in range(5):
            stamp=1.2+i*.1
            c.observe_lane([(.5,0),(.7,-.04),(.9,-.12),(1.1,-.24)],.9,stamp)
            speed,steer=c.tick(stamp)
            self.assertEqual(speed,12)
            self.assertLess(steer,0)
        c.estop=True
        self.assertEqual(c.tick(1.7),(0,0))

    def test_visual_start_setting_requires_boolean(self):
        c=self.core()
        c.cfg['startup_follow_lane']='true'
        with self.assertRaises(ValueError):validate_config(c.cfg)
