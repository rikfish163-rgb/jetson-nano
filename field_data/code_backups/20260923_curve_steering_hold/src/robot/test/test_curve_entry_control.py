import os
import unittest
from robot.common.config import load_config
from robot.master.controller import Controller


class CurveEntryControlTests(unittest.TestCase):
    def setUp(self):
        cfg=load_config(os.path.join(os.path.dirname(__file__),'..','config'))
        cfg.update(lidar_enabled=False,wait_green=False,blue_default_straight=False,
                   lookahead=.55,steering_command_scale_rad=.1,lane_curve_speed_raw=16)
        cfg['speed_raw']['lane']=20
        self.c=Controller(cfg)
        self.addCleanup(self.c.close)

    def observe(self, points, stamp, direction='RIGHT'):
        curve=dict(direction=direction,entry_side='LEFT' if direction=='RIGHT' else 'RIGHT',entry_stamp=1.)
        self.c.observe_lane(points,.9,stamp,{},curve)
        return self.c.tick(stamp)

    def test_small_steering_does_not_end_curve_slowdown(self):
        for i in range(8):
            speed,steer=self.observe([(.5,-.001),(.7,-.001),(.9,-.001)],1.+i*.1)
            self.assertEqual(speed,16)
            self.assertLess(steer,0)
        self.c.observe_lane([(.5,0),(.7,0),(.9,0)],.9,2.)
        self.assertEqual(self.c.tick(2.),(20,0.))

    def test_entry_selects_path_in_confirmed_direction_symmetrically(self):
        for sign,direction,t in ((-1,'RIGHT',1.),(1,'LEFT',2.)):
            speed,steer=self.observe([(.5,-sign*.03),(.7,sign*.08),(.9,sign*.2)],t,direction)
            self.assertEqual(speed,16)
            self.assertGreater(sign*steer,0)

    def test_direction_never_manufactures_full_lock_without_path_support(self):
        speed,steer=self.observe([(.5,.1),(.7,.1),(.9,.1)],1.)
        self.assertEqual((speed,steer),(16,0.))
        self.assertEqual(self.c.tick(2.),(0,0.))

    def test_curve_does_not_override_separate_maneuver(self):
        self.observe([(.5,.1),(.7,.1),(.9,.1)],1.)
        self.c.action='LEFT'
        speed,steer=self.c.lane_command(1.)
        self.assertEqual(speed,20)
        self.assertGreater(steer,0)

    def test_older_frame_cannot_release_current_curve(self):
        self.observe([(.5,-.001),(.7,-.001),(.9,-.001)],2.)
        self.c.observe_lane([(.5,0),(.7,0),(.9,0)],.9,1.5)
        self.assertEqual(self.c.tick(2.)[0],16)
