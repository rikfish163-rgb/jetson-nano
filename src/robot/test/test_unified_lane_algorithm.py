"""One pursuit law, a centered model trajectory and unchanged motion handoffs."""
from __future__ import division
import math
import os
import unittest

from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.common.geometry import bicycle, local
from robot.lane.preview import preview_steering
from robot.master.controller import Controller
from vehicle_control.pure_pursuit import PurePursuit


class UnifiedLaneTests(unittest.TestCase):
    RECORDED = [(.637740565,.045274327),(.700710076,.047235355),
                (.771814342,.044800647),(.844382870,.037193466),
                (.931458394,.021064645),(1.013903218,-.001572259)]

    def core(self, preview=True):
        cfg = load_config(os.path.join(os.path.dirname(__file__),'../config'))
        cfg.update(wait_green=False,lidar_enabled=False,lane_curvature_preview=preview,
                   steering_command_scale_rad=.03,lane_curve_speed_raw=16,lookahead=.7)
        cfg['speed_raw'].update(lane=24,action=24)
        core = Controller(cfg)
        self.addCleanup(core.close)
        return core

    def command(self, core, points, stamp=1.):
        core.observe_lane(points,.9,stamp)
        return core.lane_command(stamp)

    def test_recorded_bend_uses_pursuit_without_cancelling_terms(self):
        core = self.core()
        speed, steer = self.command(core,self.RECORDED)
        reference = core.lane_preview['reference_curve']
        expected = PurePursuit(.26,core.lane_preview['lookahead_m']).compute(reference).steering_angle
        self.assertAlmostEqual(steer/.03*.46275,expected,places=8)
        self.assertEqual(core.lane_preview['controller'],'pure_pursuit')
        self.assertEqual(core.lane_target['selection'],'center_pure_pursuit')
        self.assertEqual(core.lane_target['target'],core.lane_preview['tracking_target'])
        self.assertEqual(speed,16)

    def test_retained_launch_flag_cannot_select_another_steering_algorithm(self):
        points=[(.5,-.02),(.7,-.07),(.9,-.15),(1.1,-.27)]
        enabled=self.command(self.core(True),points)
        disabled=self.command(self.core(False),points)
        self.assertEqual(enabled,disabled)

    def test_short_path_uses_the_same_pursuit_rule(self):
        core=self.core()
        speed,steer=self.command(core,[(.55,.06),(.68,.075)])
        self.assertEqual(core.lane_preview['model'],'polyline')
        expected=PurePursuit(.26,core.lane_preview['lookahead_m']).compute(core.lane_preview['reference_curve'])
        self.assertAlmostEqual(steer/.03*.46275,expected.steering_angle,places=8)
        self.assertEqual(speed,16)

    def test_mirrored_lane_has_mirrored_angle_and_same_speed(self):
        right=self.command(self.core(),[(.5,0),(.7,-.06),(.9,-.16),(1.1,-.30)])
        left=self.command(self.core(),[(.5,0),(.7,.06),(.9,.16),(1.1,.30)])
        self.assertEqual(right[0],left[0])
        self.assertAlmostEqual(right[1],-left[1],places=8)

    def test_centered_circle_uses_the_radius_without_extra_steering_gain(self):
        core=self.core();radius=.65
        points=[(radius*math.sin(t),radius*(math.cos(t)-1.))
                for t in (.45,.6,.75,.9,1.05)]
        speed,steer=self.command(core,points)
        self.assertAlmostEqual(steer/.03*.46275,-math.atan(.26/radius),places=7)
        self.assertEqual(speed,16)
        self.assertLess(abs(encode_command(speed,steer,core.cfg,0)['steering_raw']),22)

    def test_model_circle_recovers_both_offsets_with_camera_jitter(self):
        for side in (-1,1):
            for offset in (-.08,.08):
                core=self.core();radius=.8;pose=(0.,offset,0.);errors=[]
                for i in range(400):
                    angle=math.atan2(pose[1]-side*radius,pose[0])
                    points=[local(pose,(radius*math.cos(angle+side*t),
                        side*radius+radius*math.sin(angle+side*t)))
                        for t in (.5,.65,.8,.95,1.1)]
                    jitter=.002 if i%2 else -.002
                    core.lane_stamp=1.+i*.05;core.pose=pose
                    steer=preview_steering(core,[(x,y+jitter) for x,y in points],0.)
                    errors.append(abs(math.hypot(pose[0],pose[1]-side*radius)-radius))
                    pose=bicycle(pose,.128*.05,steer/.03*.46275,.26)
                self.assertLess(max(errors),.09)
                self.assertLess(sum(errors[-50:])/50.,.02)

    def test_sustained_straight_accelerates_but_bend_returns_to_slow_speed(self):
        core=self.core()
        for i in range(16):
            speed,steer=self.command(core,[(.5,0),(.7,0),(.9,0),(1.1,0)],1.+i*.1)
        self.assertEqual((speed,steer),(24,0.))
        speed,steer=self.command(core,[(.5,0),(.7,-.06),(.9,-.16),(1.1,-.30)],2.6)
        self.assertEqual(speed,16)
        self.assertLess(steer,0.)


if __name__=='__main__':unittest.main()
