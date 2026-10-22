"""Regression fixtures for a visible mouth with a cropped/missing bay bottom.

No ROS, camera, lidar or motor access. Run alongside test_parking_detection.
"""
from __future__ import division
import math
import unittest
import cv2
import numpy as np
from robot.parking.entrance import from_entrance
from robot.parking.detection import detect_bays


class EntranceDetectionTest(unittest.TestCase):
    def setUp(self):
        self.ppm = 400.
        self.cfg = dict(slots={'P4':dict(length=.45,width=.38)},
                        white={'dimension_tolerance':.08}, parking_visible_side_m=.18)
        self.mask = np.zeros((600,1200),np.uint8)
        self.valid = np.ones_like(self.mask)
        self.a,self.b = np.array([720.,304.]),np.array([820.,304.])
        self.c,self.d = np.array([720.,456.]),np.array([820.,456.])
        self.u,self.n = np.array([1.,0.]),np.array([0.,1.])

    def metric(self,u,v):
        return (600-v)/400.,(600-u)/400.

    def draw_sides(self):
        for a,b in ((self.a,self.b),(self.c,self.d)):
            cv2.line(self.mask,tuple(a.astype(int)),tuple(b.astype(int)),255,5)

    def run_pair(self):
        dilation=cv2.dilate(self.mask,np.ones((5,5),np.uint8))
        def coverage(a,b):
            pts=np.array([a+(b-a)*i/24 for i in range(25)]).astype(int)
            return sum(0<=p[0]<1200 and 0<=p[1]<600 and bool(dilation[p[1],p[0]])
                       for p in pts)/25.
        return from_entrance(self.a,self.b,self.c,self.d,self.u,self.n,
                             float(np.dot(self.a,self.n)),float(np.dot(self.c,self.n)),
                             self.metric,self.ppm,self.mask,self.valid,self.cfg,coverage)

    def test_right_mouth_without_bottom_infers_known_depth(self):
        self.draw_sides()
        slot,reason=self.run_pair()
        self.assertIsNone(reason)
        self.assertEqual(slot['evidence'],'entrance')
        self.assertTrue(slot['bottom_inferred'])
        self.assertAlmostEqual(slot['x'],.55)
        self.assertAlmostEqual(slot['y'],-.525)
        self.assertAlmostEqual(slot['yaw'],-math.pi/2)

    def test_left_mouth_is_mirrored(self):
        for p in (self.a,self.b,self.c,self.d):
            p[0]=1200-p[0]
        self.u=np.array([-1.,0.])
        self.draw_sides()
        slot,reason=self.run_pair()
        self.assertIsNone(reason)
        self.assertAlmostEqual(slot['y'],.525)
        self.assertAlmostEqual(slot['yaw'],math.pi/2)

    def test_cropped_endpoints_are_not_entrance_points(self):
        self.draw_sides()
        self.valid[:,:721]=0
        slot,unused=self.run_pair()
        self.assertIsNone(slot)

    def test_interior_fragments_of_continuing_lines_are_rejected(self):
        self.draw_sides()
        for p in (self.a,self.c):
            cv2.line(self.mask,(660,int(p[1])),tuple(p.astype(int)),255,5)
        self.assertIsNone(self.run_pair()[0])

    def test_unaligned_fragment_ends_are_rejected(self):
        self.c[0]+=40;self.d[0]+=40
        self.draw_sides()
        self.assertIsNone(self.run_pair()[0])

    def test_longitudinal_road_lines_are_not_side_facing_bays(self):
        self.a,self.b=np.array([720.,304.]),np.array([720.,404.])
        self.c,self.d=np.array([872.,304.]),np.array([872.,404.])
        self.u,self.n=np.array([0.,1.]),np.array([-1.,0.])
        self.draw_sides()
        self.assertIsNone(self.run_pair()[0])

    def test_full_detector_accepts_visible_mouth_without_back_line(self):
        self.draw_sides()
        slots,diag=detect_bays(self.mask,self.metric,self.ppm,self.cfg,self.valid)
        self.assertTrue(any(s.get('evidence')=='entrance' for s in slots))
        self.assertGreater(diag['accepted_entrance'],0)

    def test_full_detector_keeps_width_constraint(self):
        self.c[1]=544;self.d[1]=544  # 60 cm, not the 38 cm bay width.
        self.draw_sides()
        slots,unused=detect_bays(self.mask,self.metric,self.ppm,self.cfg,self.valid)
        self.assertEqual(slots,[])


if __name__=='__main__':
    unittest.main()
