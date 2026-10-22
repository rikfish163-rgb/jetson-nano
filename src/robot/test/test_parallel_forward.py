"""Forward side-bay selection and main-controller regressions; no ROS output."""
import copy
import math
import os
import unittest
import yaml
import numpy as np
import cv2

from robot.common.contracts import validate_config
from robot.common.geometry import inside_slot, footprint, local, collision
from robot.master.controller import Controller
from robot.camera.vision import GroundDetector
from robot.parallel_parking.forward import select_forward_slot, plan_forward_slot

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
with open(os.path.join(ROOT, 'config', 'competition.yaml')) as stream:
    CONFIG = yaml.safe_load(stream)


class Scan(object):
    stamp = 2.0
    completed_stamp = 2.0
    valid_rays = 360
    shape_filter = False
    def __init__(self, obstacles=(), coverage=1.):
        self.obstacles = list(obstacles)
        self.value = coverage
    def coverage(self, points):
        return self.value
    def classify(self, point):
        return 'FREE'


class ForwardParallelTests(unittest.TestCase):
    def setUp(self):
        self.cfg = copy.deepcopy(CONFIG)
        self.cfg.update(parking_mode='parallel_forward', parking_slot='AUTO', wait_green=False)
        self.slots = [dict(id='P%d' % (3-i), kind='parallel', pose=(.9+.75*i,-.40,0.),
                           length=.70,width=.36,observations=3) for i in range(3)]

    def test_mode_and_auto_requirement(self):
        validate_config(self.cfg)
        self.cfg['parking_slot']='P4'
        with self.assertRaises(ValueError):validate_config(self.cfg)

    def test_each_of_three_can_be_the_only_empty_bay(self):
        for free in range(3):
            scan=Scan([s['pose'][:2] for i,s in enumerate(self.slots) if i!=free])
            target,reason,rows=select_forward_slot((0,0,0),self.slots,scan,self.cfg)
            self.assertEqual(target['id'],self.slots[free]['id'])
            self.assertEqual([r['occupancy'] for r in rows].count('FREE'),1)

    def test_unknown_multiple_empty_and_unconfirmed_do_not_lock(self):
        for scan,slots in ((Scan(coverage=0.),self.slots),(Scan(),self.slots),
                           (Scan(),[dict(self.slots[0],observations=1)])):
            self.assertIsNone(select_forward_slot((0,0,0),slots,scan,self.cfg)[0])

    def test_wrong_side_kind_far_and_duplicate_candidates(self):
        for change in (dict(pose=(.9,.4,0)),dict(kind='perpendicular'),
                       dict(pose=(3.1,-.4,0)),dict(length=.9)):
            rows=[dict(self.slots[0],**change)]
            self.assertIsNone(select_forward_slot((0,0,0),rows,Scan(),self.cfg)[0])
        self.assertIsNone(select_forward_slot((0,0,0),[self.slots[0]]*2,Scan(),self.cfg)[0])

    def test_p_triggers_without_blue_and_locks_action(self):
        c=Controller(self.cfg);self.addCleanup(c.close)
        for t in (1.,1.1,1.2):c.observe_sign('PARKING',.95,t,t)
        self.assertEqual(c.dispatch(1.2),(0,0.))
        self.assertEqual(c.state,'PARKING_SCAN')
        self.assertEqual(c.action,'PARKING')
        c.observe_sign('RIGHT',.99,1.3,1.3)
        self.assertIsNone(c.pending)

    def test_side_open_u_bays_detected_without_road_facing_long_edge(self):
        detector=GroundDetector(self.cfg)
        mask=np.zeros((1400,1200),np.uint8)
        offset=360
        def pixel(x,y):
            cam=self.cfg['front_camera']
            return (int(cam['origin_u']+offset-y*400),int(cam['origin_v']-x*400))
        # Fit two complete right-side bays inside the existing calibrated range.
        for low in (.05,.77):
            for a,b in (((low,-.22),(low,-.58)),((low+.70,-.22),(low+.70,-.58)),
                        ((low,-.58),(low+.70,-.58))):
                cv2.line(mask,pixel(*a),pixel(*b),255,5)
        rows=detector.detect_slots(mask,u_offset=offset)
        self.assertEqual(len(rows),2)
        self.assertTrue(all(r['kind']=='parallel' and r['y']<0 for r in rows))

    def test_plan_is_forward_only_and_finishes_inside(self):
        slot=self.slots[0]
        path,reason=plan_forward_slot((0,0,0),slot,Scan(),self.cfg)
        self.assertTrue(path,reason)
        self.assertTrue(all(p[3]==1 for p in path))
        self.assertTrue(inside_slot(path[-1],slot,self.cfg))
        self.assertTrue(all(abs(local(slot['pose'],p)[0])<=.35+1e-8 and
                            abs(local(slot['pose'],p)[1])<=.18+1e-8
                            for p in footprint(path[-1],self.cfg)))

    def test_blocked_or_passed_bay_never_falls_back_to_reverse(self):
        slot=self.slots[0]
        for start,scan in (((0,0,0),Scan([(0,0)])),((1.1,0,0),Scan())):
            path,reason=plan_forward_slot(start,slot,scan,self.cfg)
            self.assertFalse(path)


if __name__=='__main__':unittest.main()
