"""Estimated heading cannot override the six-second bypass right arc."""
import os
import unittest
from robot.common.config import load_config
from robot.common.geometry import bicycle
from robot.motion import calibration as chassis
from robot.master.controller import Controller
from test_continuous_obstacles import _SyntheticScan


class BypassHeadingGeometryTests(unittest.TestCase):
    def test_extended_left_arc_still_requires_six_seconds_of_right_motion(self):
        cfg=load_config(os.path.join(os.path.dirname(__file__),'../config'))
        cfg.update(wait_green=False,lidar_enabled=True,lane_curvature_preview=True,
                   lane_curve_speed_raw=16,steering_command_scale_rad=.03,
                   timed_bypass_speed_raw=26,timed_bypass_right_s=6.)
        cfg['speed_raw'].update(action=24,lane=24)
        c=Controller(cfg);self.addCleanup(c.close)
        left_duration=2.6870188713
        velocity=26*chassis.speed_gain(cfg,26)
        angle=chassis.raw_angle(cfg,26,22)
        pose=bicycle((0.,0.,0.),velocity*left_duration,angle,cfg['wheelbase'])
        c.set_pose(pose,1.)
        c.state='TIMED_BYPASS';c.action='BYPASS'
        c.timed_bypass=dict(phase='LEFT',last=1.,elapsed_s=left_duration,
            entry_heading_rad=0.,trigger_world=(.747,.154),trigger_point=(.747,.154))
        c.scan=_SyntheticScan(1.)
        self.assertEqual(c.execute('obstacle','timed_bypass_tick',1.).value,(26,-.03))
        for i in range(1,121):
            now=1.+i*.05
            pose=bicycle(pose,velocity*.05,chassis.raw_angle(cfg,26,-22),cfg['wheelbase'])
            c.set_pose(pose,now)
            path=[(d,.03) for d in (.6,.8,1.,1.2)]
            c.observe_lane(path,.8,now);c.scan=_SyntheticScan(now)
            speed,steer=c.execute('obstacle','timed_bypass_tick',now).value
            if c.timed_bypass is None:
                self.assertEqual(i,120)
                self.assertGreater(speed,0)
                break
            self.assertEqual((speed,steer),(26,-.03))
        else:
            self.fail('Right arc did not finish after six seconds')


if __name__=='__main__':unittest.main()
