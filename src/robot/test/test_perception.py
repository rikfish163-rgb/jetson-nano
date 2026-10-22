from __future__ import division
import os
import sys
import unittest
import json
import math
import numpy as np
import cv2
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.join(ROOT,'src'))
from robot.camera.vision import GroundDetector
from robot.common.contracts import ground
from robot.common.contracts import decode
from robot.common.contracts import encode_command
from robot.common.contracts import boolean
from robot.common.contracts import validate_config


class PerceptionTests(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(ROOT,'config','competition.yaml')) as f:
            self.cfg = yaml.safe_load(f)
        self.detector = GroundDetector(self.cfg)

    def test_blue_long_and_tick_are_distinct(self):
        img = np.zeros((400,480,3),np.uint8)
        cv2.rectangle(img,(20,330),(460,338),(255,0,0),-1)
        cv2.rectangle(img,(20,370),(60,378),(255,0,0),-1)
        data,_,_ = self.detector.detect(img)
        self.assertEqual(sorted(m['kind'] for m in data['markers']),['junction','tick'])

    def test_blue_sign_shaped_blob_not_floor_marker(self):
        img = np.zeros((400,480,3),np.uint8)
        cv2.rectangle(img,(100,100),(180,180),(255,0,0),-1)
        self.assertFalse(self.detector.detect(img)[0]['markers'])

    def test_front_longitudinal_blue_is_junction_without_angle_filter(self):
        for dx in (-35, 35):
            img = np.zeros((400,480,3),np.uint8)
            cv2.line(img,(350,365),(350+dx,90),(255,0,0),8)
            data,_,_ = self.detector.detect(img,include_slots=False)
            self.assertEqual([m['kind'] for m in data['markers']],['junction'])
            self.assertEqual(len(data['blue_lines']),1)
            line = data['blue_lines'][0]
            self.assertLess(line['y'],0)
            tangent = line['yaw'] % math.pi-math.pi/2
            self.assertAlmostEqual(tangent,math.atan2(-dx,275),delta=.025)

    def test_rear_blue_is_published_in_rear_axle_coordinates(self):
        img = np.zeros((400,480,3),np.uint8)
        cv2.rectangle(img,(80,360),(400,368),(255,0,0),-1)
        data,_,_=self.detector.detect(img,rear=True,include_slots=False)
        self.assertEqual(len(data['markers']),1)
        self.assertEqual(data['markers'][0]['kind'],'junction')
        self.assertAlmostEqual(data['markers'][0]['x'],-.16,places=2)
        self.assertAlmostEqual(data['markers'][0]['y'],0,places=2)
        self.assertEqual(data['slots'],[])

    def test_slanted_front_blue_keeps_normal_and_generates_junction(self):
        for sign in (-1,1):
            img=np.zeros((400,480,3),np.uint8)
            cv2.line(img,(100,200-sign*70),(380,200+sign*70),(255,0,0),8)
            data,_,_=self.detector.detect(img,include_slots=False)
            self.assertEqual([m['kind'] for m in data['markers']],['junction'])
            self.assertEqual(len(data['blue_lines']),1)
            self.assertAlmostEqual(data['blue_lines'][0]['yaw'],-sign*math.atan(.5),delta=.025)

    def test_short_slanted_front_blue_is_not_junction(self):
        img=np.zeros((400,480,3),np.uint8)
        cv2.line(img,(200,200),(260,230),(255,0,0),8)
        data,_,_=self.detector.detect(img,include_slots=False)
        self.assertEqual([m['kind'] for m in data['markers']],['tick'])
        self.assertEqual(data['blue_lines'],[])

    def test_slanted_rear_blue_still_uses_angle_filter(self):
        img=np.zeros((400,480,3),np.uint8)
        cv2.line(img,(100,130),(380,270),(255,0,0),8)
        data,_,_=self.detector.detect(img,rear=True,include_slots=False)
        self.assertEqual(data['markers'],[])
        self.assertEqual(len(data['blue_lines']),1)

    def test_blue_geometry_rejects_nonfinite_orientation(self):
        data=dict(stamp=1,frame='base_link',source='front',part='markers',markers=[],slots=[],
                  blue_lines=[dict(x=.8,y=0,length=.6,yaw=float('nan'))])
        with self.assertRaises(ValueError): ground(json.dumps(data),1,.5)

    def test_parallel_slot_u_shape_and_rear_coordinates(self):
        img = np.zeros((400,480,3),np.uint8)
        cv2.line(img,(100,50),(100,330),(255,255,255),8)
        cv2.line(img,(244,50),(244,330),(255,255,255),8)
        cv2.line(img,(100,50),(244,50),(255,255,255),8)
        front = [s for s in self.detector.detect(img)[0]['slots'] if s['kind']=='parallel']
        rear = [s for s in self.detector.detect(img,True)[0]['slots'] if s['kind']=='parallel']
        self.assertTrue(front)
        self.assertTrue(rear)
        self.assertGreater(front[0]['x'],0)
        self.assertLess(rear[0]['x'],0)

    def test_two_edges_without_end_not_a_slot(self):
        img = np.zeros((400,480,3),np.uint8)
        for x in (100,244):
            cv2.line(img,(x,50),(x,330),(255,255,255),8)
        self.assertFalse(self.detector.detect(img)[0]['slots'])

    def test_wrong_camera_resolution_is_rejected(self):
        with self.assertRaises(ValueError):
            self.detector.bev(np.zeros((480,640,3),np.uint8))

    def test_input_boundaries(self):
        for raw in ('[]','{"stamp":NaN}','{"stamp":12.0}', 'x'*65537):
            with self.assertRaises((ValueError,KeyError)):
                decode(raw,10,1)
        with self.assertRaises(ValueError):
            ground(json.dumps(dict(stamp=10,frame='camera',source='front',markers=[],slots=[])),10,1)
        with self.assertRaises(ValueError):
            ground(json.dumps(dict(stamp=10,frame='base_link',source='front',markers=[dict(x=float('nan'),y=0,kind='junction')],slots=[])),10,1)

    def test_rad_to_raw_same_forward_and_reverse(self):
        a = encode_command(16,self.cfg['max_steer'],self.cfg,256)
        b = encode_command(-16,self.cfg['max_steer'],self.cfg,257)
        self.assertEqual(a['steering_raw'],22)
        self.assertEqual(b['steering_raw'],22)
        self.assertEqual(a['seq'],0)
        self.assertEqual(b['speed_raw'],-16)
        with self.assertRaises(ValueError):
            encode_command(1,float('nan'),self.cfg,0)

    def test_split_parts_cannot_carry_the_other_observation_type(self):
        data = dict(stamp=10,frame='base_link',source='front',part='slots',
                    markers=[dict(x=.3,y=0,kind='junction')],slots=[])
        with self.assertRaises(ValueError):
            ground(json.dumps(data),10,1.25)
        data.update(part='markers',markers=[],slots=[dict(x=.3,y=-.5,yaw=0,kind='parallel')])
        with self.assertRaises(ValueError):
            ground(json.dumps(data),10,1.25)

    def test_string_false_and_nonfinite_config_are_rejected(self):
        validate_config(self.cfg)
        for bad in ('false','true',0,1,None):
            with self.assertRaises(ValueError):
                boolean(bad)
        self.cfg['max_steer'] = float('nan')
        with self.assertRaises(ValueError):
            validate_config(self.cfg)


if __name__ == '__main__':
    unittest.main()
