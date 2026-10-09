"""Partial, broken and oblique bay observations; no ROS or motor output."""
from __future__ import division
import copy
import math
import json
import unittest
import cv2
import numpy as np
import test_parking_entry as fixture
from robot.camera.vision import GroundDetector
from robot.common.geometry import world
from robot.common.contracts import ground


class PartialBayTest(unittest.TestCase):
    def detector(self):
        cfg = copy.deepcopy(fixture.CONFIG)
        cfg['parking_mode'] = 'forward_white'
        return GroundDetector(cfg)

    def mask(self, broken=False, partial=False, bottom=True):
        mask = np.zeros((600,1200),np.uint8)
        def draw(x1,y1,x2,y2):
            p=lambda x,y:(int(round(600-400*y)),int(round(600-400*x)))
            cv2.line(mask,p(x1,y1),p(x2,y2),255,4)
        for x in (.31,.69,1.07):
            if partial:
                draw(x,-.56,x,-.775)
            elif broken:
                draw(x,-.325,x,-.48)
                draw(x,-.535,x,-.775)
            else:
                draw(x,-.325,x,-.775)
        if bottom:
            draw(.31,-.775,1.07,-.775)
        return mask

    def assert_bays(self, d, mask):
        slots=d.detect_slots(mask,u_offset=360)
        self.assertEqual(len(slots),2,d.slot_diagnostic)
        for s,x in zip(sorted(slots,key=lambda s:s['x']),(.5,.88)):
            self.assertAlmostEqual(s['x'],x,delta=.025)
            self.assertAlmostEqual(s['y'],-.55,delta=.025)

    def test_short_sides_anchored_to_bottom_locate_both_bays(self):
        self.assert_bays(self.detector(),self.mask(partial=True))

    def test_measured_back_edge_overrides_inferred_depth(self):
        mask = self.mask(partial=True)
        detector = self.detector()
        detector.parking_valid = np.ones_like(mask)
        self.assert_bays(detector, mask)

    def test_collinear_fragments_are_joined_without_merging_adjacent_dividers(self):
        self.assert_bays(self.detector(),self.mask(broken=True))

    def test_unanchored_parallel_fragments_do_not_invent_bay_depth(self):
        d=self.detector()
        self.assertEqual(d.detect_slots(self.mask(partial=True,bottom=False),u_offset=360),[])
        self.assertGreater(d.slot_diagnostic['reject_anchor'],0)

    def test_empty_image_explains_missing_edges(self):
        d=self.detector()
        self.assertEqual(d.detect_slots(np.zeros((600,1200),np.uint8),u_offset=360),[])
        self.assertEqual(d.slot_diagnostic['raw_segments'],0)
        self.assertEqual(d.slot_diagnostic['accepted'],0)

    def test_diagnostic_counts_are_bounded_and_finite(self):
        data=dict(stamp=10,source='front',frame='base_link',part='slots',markers=[],slots=[],
                  slot_diagnostic=dict(raw_segments=12,accepted=0))
        self.assertEqual(ground(json.dumps(data),10,1)[0]['slot_diagnostic']['raw_segments'],12)
        for diag in ({'accepted':float('nan')},{'accepted':-1},{'accepted':1.5},{'surprise':1}):
            data['slot_diagnostic']=diag
            with self.assertRaises(ValueError):
                ground(json.dumps(data),10,1)


class SelectionRegressionTest(unittest.TestCase):
    def fixture(self):
        helper=fixture.WhiteParkingTest('test_near_starts_forward_full_lock_on_correct_side')
        self.addCleanup(helper.doCleanups)
        return helper,helper.core()

    def test_one_confirmed_near_bay_can_start_without_seeing_whole_far_bay(self):
        h,c=self.fixture()
        h.bays=h.bays[:1]
        h.observations(c,10.5)
        h.observations(c,11.0)
        c.tick(11.0)
        self.assertIsNotNone(c.parking_entry,c.reason)
        self.assertEqual(c.parking_entry.slot['relative_bay'],'near')

    def test_single_far_bay_is_not_mistaken_for_near_start(self):
        h,c=self.fixture()
        h.bays=h.bays[1:]
        h.observations(c,10.5)
        h.observations(c,11.0)
        self.assertEqual(c.tick(11.0),(0,0))
        self.assertIsNone(c.parking_entry)

    def test_pair_matching_uses_bay_frame_when_vehicle_is_oblique(self):
        h,c=self.fixture()
        angle=math.radians(25)
        for s in h.bays:
            s['x'],s['y']=world((0,0,angle),(s['x'],s['y']))
            s['yaw']+=angle
        h.observations(c,10.5)
        h.observations(c,11.0)
        c.tick(11.0)
        self.assertEqual(len(c.parking_candidates),2,c.parking_detection)
        self.assertIsNotNone(c.parking_entry,c.reason)

    def test_first_valid_frame_after_empty_frame_needs_confirmation(self):
        h,c=self.fixture()
        bays=h.bays
        h.bays=[]
        h.observations(c,10.5)
        h.bays=bays[:1]
        h.observations(c,11.0)
        self.assertEqual(c.tick(11.0),(0,0))
        self.assertIsNone(c.parking_entry)
        h.observations(c,11.2)
        c.tick(11.2)
        self.assertIsNotNone(c.parking_entry,c.reason)

    def test_stopped_bay_geometry_can_pair_with_latest_fresh_scan(self):
        h,c=self.fixture()
        h.observations(c,10.5)
        h.observations(c,10.6)
        c.scan.stamp=11.3
        c.tick(11.3)
        self.assertIsNotNone(c.parking_entry,c.reason)
        self.assertTrue(c.parking_detection['stationary_pairing'])

    def test_stopped_pairing_still_rejects_old_images_and_scans(self):
        h,c=self.fixture()
        h.observations(c,10.5)
        h.observations(c,10.6)
        c.scan.stamp=12
        self.assertEqual(c.tick(12),(0,0))
        self.assertIsNone(c.parking_entry)
        h.observations(c,12.1)
        c.scan.stamp=11
        self.assertEqual(c.tick(12.1),(0,0))
        self.assertIsNone(c.parking_entry)


if __name__=='__main__':
    unittest.main()
