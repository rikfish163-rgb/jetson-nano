"""Measured transverse stop, then a separate blue-guided straight segment."""
import math
import os
import unittest
import cv2
import numpy as np
from robot.common.config import load_config
from robot.master.controller import Controller
from robot.camera.vision import GroundDetector


class StraightStopGeometryTests(unittest.TestCase):
    def setUp(self):
        self.cfg=load_config(os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config'))
        self.cfg.update(wait_green=False,lidar_enabled=False,sign_ttl=0,
                        intersection_wait_s=1.,straight_speed_raw=12)
        self.c=Controller(self.cfg)
        self.addCleanup(self.c.close)
        self.c.pending='STRAIGHT'

    def frame(self,t,x=.9,side=None):
        self.c.observe_lane([(.5,-.3),(.8,-.3)],.9,t)
        lines=[dict(x=x,y=0.,yaw=0.,length=.6)] if x is not None else []
        if side is not None:
            lines.append(dict(x=.7,y=side,yaw=math.pi/2,length=.7))
        self.c.observe_ground(dict(source='front',part='markers',slots=[],
            markers=[dict(kind='junction',x=x,y=0.,length=.6)] if x is not None else [],
            blue_lines=lines),t)
        return self.c.tick(t)

    def arm(self,x):
        for t in (1.,1.25,1.5):self.frame(t,x)

    def test_stop_position_depends_on_measured_stripe_not_fixed_entry(self):
        for x in (.65,1.10):
            self.c.close();self.c=Controller(self.cfg);self.addCleanup(self.c.close)
            self.c.pending='STRAIGHT';self.arm(x)
            self.assertEqual(self.c.blue_approach['phase'],'STOP_LINE')
            # Rear axle is 0.33 m behind the front bumper, with 0.02 m clearance.
            stop_x=x-self.cfg['wheelbase']-self.cfg['front_overhang']-.02
            self.c.set_pose((stop_x-.03,0,0),1.6)
            self.assertEqual(self.frame(1.6,None),(12,0.))
            self.c.set_pose((stop_x,0,0),1.7)
            self.assertEqual(self.frame(1.7,None),(0,0.))
            self.assertEqual(self.c.state,'BLUE_STOP')
            self.assertEqual(self.frame(2.6,None),(0,0.))
            self.frame(2.71,None)
            self.assertEqual(self.c.action,'STRAIGHT')
            self.assertAlmostEqual(self.c.straight_search['origin'][0],stop_x)
            self.c.set_pose((stop_x+1.24,0,0),2.8)
            speed,steer=self.frame(2.8,None,-.45)
            self.assertEqual(speed,12)
            self.assertLess(steer,0)
            self.assertEqual(self.c.reason,'straight_align_right_blue')

    def test_longitudinal_blue_never_dispatches_straight(self):
        for t in (1.,1.25,1.5):self.frame(t,None,-.3)
        self.assertIsNone(self.c.action)

    def test_connected_or_fragmented_blue_retains_measured_long_lines(self):
        detector=GroundDetector(self.cfg)
        for connected in (True,False):
            bev=np.zeros((400,480,3),np.uint8)
            # Crossing and side paint share a contour, whose bounding box is
            # much thicker than the physical tape. Dark gaps also split it.
            color=(255,80,0)
            if connected:
                cv2.line(bev,(30,180),(400,180),color,8)
                cv2.line(bev,(400,180),(400,380),color,8)
            else:
                for u in (30,115,200,285):
                    cv2.line(bev,(u,180),(u+75,180),color,8)
            data,unused,unused2=detector.detect(bev,include_slots=False)
            transverse=[a for a in data['blue_lines'] if abs(a['yaw'])<.1]
            self.assertTrue(transverse,data)
            self.assertGreaterEqual(max(a['length'] for a in transverse),.7)
            if connected:
                self.assertTrue(any(abs(abs(a['yaw'])-math.pi/2)<.1 for a in data['blue_lines']),data)

    def test_wide_blue_patch_is_not_a_tape_line(self):
        bev=np.zeros((400,480,3),np.uint8)
        cv2.rectangle(bev,(40,120),(400,220),(255,80,0),-1)
        data,unused,unused2=GroundDetector(self.cfg).detect(bev,include_slots=False)
        self.assertEqual(data['blue_lines'],[])


if __name__=='__main__':unittest.main()
