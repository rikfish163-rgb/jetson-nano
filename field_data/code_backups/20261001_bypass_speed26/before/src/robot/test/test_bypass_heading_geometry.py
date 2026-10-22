"""Full-lock bypass counterturn is bounded by the actual left arc."""
import math
import os
import unittest
from robot.common.config import load_config
from robot.common.contracts import command_to_model_steering
from robot.common.geometry import bicycle,local,wrap
from robot.master.controller import Controller
from test_continuous_obstacles import _SyntheticScan


class BypassHeadingGeometryTests(unittest.TestCase):
    def test_extended_left_arc_hands_off_without_turning_past_road_heading(self):
        cfg=load_config(os.path.join(os.path.dirname(__file__),'../config'))
        cfg.update(wait_green=False,lidar_enabled=True,lane_curvature_preview=True,
                   lane_curve_speed_raw=16,steering_command_scale_rad=.03)
        cfg['speed_raw'].update(action=24,lane=24)
        c=Controller(cfg);self.addCleanup(c.close)
        left_duration=2.6870188713
        angle=command_to_model_steering(.03,cfg)
        pose=bicycle((0.,0.,0.),.192*left_duration,angle,.26)
        c.set_pose(pose,1.)
        c.state='TIMED_BYPASS';c.action='BYPASS'
        c.timed_bypass=dict(phase='LEFT',last=1.,elapsed_s=left_duration,
            entry_heading_rad=0.,trigger_world=(.747,.154),trigger_point=(.747,.154))
        c.scan=_SyntheticScan(1.)
        self.assertEqual(c.execute('obstacle','timed_bypass_tick',1.).value,(24,-.03))
        self.assertAlmostEqual(c.timed_bypass['left_turn_angle_rad'],pose[2])
        for i in range(1,81):
            now=1.+i*.05
            pose=bicycle(pose,.192*.05,-angle,.26)
            c.set_pose(pose,now)
            # A measured straight road is visible, with lateral offset.
            path=[local(pose,(pose[0]+d,0.)) for d in (.6,.8,1.,1.2)]
            c.observe_lane(path,.8,now);c.scan=_SyntheticScan(now)
            speed,steer=c.execute('obstacle','timed_bypass_tick',now).value
            if c.timed_bypass is None:
                self.assertEqual(speed,16)
                self.assertGreaterEqual(wrap(pose[2]),-math.radians(1.))
                self.assertLessEqual(i*.05,left_duration+.05)
                self.assertEqual(c.reason,'bypass_heading_lane_handoff')
                break
            self.assertEqual((speed,steer),(24,-.03))
        else:
            self.fail('Right arc continued past the left-heading cancellation')


if __name__=='__main__':unittest.main()
