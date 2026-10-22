"""Regressions from run 20260930_180622_16654; no vehicle hardware."""
import os
import unittest
from robot.common.config import load_config
from robot.master.controller import Controller
from robot.lane.preview import preview_steering


class RecordedLaneHandoffTests(unittest.TestCase):
    PATH = [(.59538,.19074),(.72931,.24075),(.86148,.27386),
            (.99380,.29215),(1.12619,.29623)]

    def setUp(self):
        cfg=load_config(os.path.join(os.path.dirname(__file__),'../config'))
        cfg.update(wait_green=False,lidar_enabled=False,lane_curvature_preview=True,
                   steering_command_scale_rad=.03,lane_curve_speed_raw=16,lookahead=.8)
        self.c=Controller(cfg)
        self.addCleanup(self.c.close)

    def test_recorded_left_path_allows_heading_correction_after_bypass(self):
        self.c.observe_lane(self.PATH,.8,1.)
        self.assertGreater(preview_steering(self.c,self.PATH,0.),0.)

    def test_sparse_samples_of_same_reference_do_not_reverse_correction(self):
        for path in (self.PATH,self.PATH[::2],self.PATH[:2]):
            self.c.observe_lane(path,.8,self.c.lane_stamp+1.)
            self.assertGreater(preview_steering(self.c,path,0.),0.)

    def test_one_bad_frame_holds_trusted_angle_but_cannot_extend_grace(self):
        self.c.observe_lane([(.5,0),(.7,-.04),(.9,-.12),(1.1,-.24)],.9,1.)
        speed,angle=self.c.lane_command(1.)
        start=self.c.gap_start
        for stamp in (1.05,1.15,1.24):
            self.c.observe_lane([(.7,0)],.1,stamp)
            held=self.c.lane_command(stamp)
            self.assertGreater(held[0],0)
            self.assertAlmostEqual(held[1],angle)
            self.assertEqual(self.c.gap_start,start)
        self.c.observe_lane([(.7,0)],.1,1.3)
        self.assertEqual(self.c.lane_command(1.3)[0],0)
        self.assertEqual(self.c.lane_command(1.9)[0],0)

    def test_collision_still_stops_during_bad_frame_grace(self):
        from test_continuous_obstacles import _SyntheticScan
        self.c.observe_lane([(.5,0),(.7,0),(.9,0)],.9,1.)
        self.c.tick(1.)
        self.c.cfg.update(lidar_enabled=True,timed_bypass_enabled=False)
        self.c.scan=_SyntheticScan(1.1,obstacles=[(.20,0)])
        self.c.observe_lane([(.7,0)],.1,1.1)
        self.assertEqual(self.c.tick(1.1)[0],0)
        self.assertEqual(self.c.reason,'lidar_obstacle_in_sweep')

    def test_bypass_lateral_offset_rejoins_center_without_overshoot(self):
        from robot.common.geometry import bicycle,local
        pose=(0.,.48,.05)
        offsets=[]
        commands=[]
        for i in range(241):
            path=[local(pose,(pose[0]+d,0.)) for d in (.5,.7,.9,1.1)]
            self.c.pose=pose
            self.c.lane_stamp=1.+i*.05
            angle=preview_steering(self.c,path,0.)
            commands.append(angle)
            offsets.append(pose[1])
            pose=bicycle(pose,.128*.05,angle/.03*.46275,.26)
        self.assertLess(commands[0],0.)
        self.assertTrue(any(angle>0. for angle in commands))
        self.assertLess(max(offsets),.49)
        self.assertGreater(min(offsets),-.04)
        self.assertLess(abs(offsets[-1]),.03)


if __name__=='__main__':unittest.main()
