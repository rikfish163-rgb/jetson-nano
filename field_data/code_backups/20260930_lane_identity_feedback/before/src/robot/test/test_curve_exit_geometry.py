"""Measured exit curvature and continuously sampled geometric references."""
import os
import math
import unittest
from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.master.controller import Controller


class CurveExitGeometryTests(unittest.TestCase):
    EXIT=[(.635766,.043666),(.706168,.036871),(.792309,.020260),
          (.865871,-.001026),(.948253,-.032443)]

    def core(self):
        cfg=load_config(os.path.join(os.path.dirname(__file__),'../config'))
        cfg.update(wait_green=False,lidar_enabled=False,lane_curvature_preview=True,
                   steering_command_scale_rad=.03,lane_curve_speed_raw=16)
        cfg['speed_raw']['lane']=24
        c=Controller(cfg);self.addCleanup(c.close)
        return c

    def raw(self,c,path,stamp=1.):
        c.observe_lane(path,.8,stamp)
        speed,steer=c.lane_command(stamp)
        self.assertEqual(speed,16)
        return encode_command(speed,steer,c.cfg,0)['steering_raw']

    def test_recorded_outward_exit_does_not_cancel_measured_right_bend(self):
        c=self.core()
        # The car is outside this still-curved reference. Heading feedback
        # must not erase the radius and outward position correction.
        self.assertLessEqual(self.raw(c,self.EXIT),-14)
        self.assertLess(c.lane_preview['curvature'],-.5)
        self.assertGreater(c.lane_preview['center_heading_error_rad'],.4)
        self.assertIsNone(c.lane_curve_lock)

    def test_mirrored_exit_retains_same_geometric_angle(self):
        right=self.raw(self.core(),self.EXIT)
        left=self.raw(self.core(),[(x,-y) for x,y in self.EXIT])
        self.assertEqual(left,-right)

    def test_centered_circle_uses_ackermann_radius_and_smooth_reference(self):
        c=self.core();r=.65
        path=[(x,math.sqrt(r*r-x*x)-r) for x in (.25,.35,.45,.5,.55)]
        raw=self.raw(c,path)
        required=math.atan(c.cfg['wheelbase']/r)/c.cfg['max_steer']*22
        self.assertAlmostEqual(abs(raw),required,delta=1.)
        curve=c.lane_preview['reference_curve']
        self.assertGreaterEqual(len(curve),40)
        self.assertTrue(all(abs(math.hypot(x,y+r)-r)<.001 for x,y in curve))

    def test_tight_outward_bend_keeps_full_lock_on_each_new_frame(self):
        c=self.core();r=.55
        path=[(x,math.sqrt(r*r-x*x)-r-.08) for x in (.2,.3,.4,.45,.5)]
        for i in range(20):
            self.assertEqual(self.raw(c,path,1.+i*.1),-22)
            self.assertTrue(c.lane_preview['outer_bend_guard'])

    def test_straight_current_path_can_release_bend_and_countersteer(self):
        c=self.core();self.raw(c,self.EXIT)
        for i in range(1,15):
            raw=self.raw(c,[(.5,.06),(.7,.06),(.9,.06),(1.1,.06)],1.+i*.1)
        self.assertGreater(raw,0)


if __name__=='__main__':
    unittest.main()
