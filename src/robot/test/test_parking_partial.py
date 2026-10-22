"""Visibility-aware template regression fixtures; no ROS or hardware access."""
from __future__ import division
import unittest
import cv2
import numpy as np
from robot.parking.partial_model import PartialBayModel


class PartialModelTest(unittest.TestCase):
    def setUp(self):
        self.cfg=dict(slots={'P4':dict(width=.38,length=.45)})
        self.metric=lambda u,v:((600-v)/400.,(600-u)/400.)
        self.model=PartialBayModel(self.cfg,self.metric,400.)
        self.pose=(.36,-.30,0.,-1.)  # mouth origin, axis pointing into right bays
        self.mask=np.zeros((600,1200),np.uint8)
        self.valid=np.ones_like(self.mask)
        for x in (.36,.74,1.12):
            v=int(round(600-x*400))
            cv2.line(self.mask,(720,v),(900,v),255,5)
        cv2.line(self.mask,(900,152),(900,456),255,5)
        for i in range(7):
            v=int(round(600-(.405+i*.11)*400))
            cv2.line(self.mask,(720,v-10),(720,v+10),255,4)

    def evaluate(self):
        return self.model.evaluate(self.mask,self.valid,self.pose)

    def test_complete_layout_has_two_supported_bays(self):
        report=self.evaluate()
        self.assertEqual(len(report['candidates']),2)
        self.assertTrue(report['pair_identity_supported'])

    def test_invisible_back_is_unknown_not_negative_evidence(self):
        self.valid[:,815:]=0
        self.mask[:,815:]=0
        report=self.evaluate()
        self.assertEqual(len(report['candidates']),2)
        self.assertFalse(report['parts']['back']['supported'])
        self.assertTrue(all(c['bottom_inferred'] for c in report['candidates']))

    def test_visible_but_missing_paint_reduces_fit(self):
        full=self.evaluate()['score']
        self.mask[:,745:]=0
        self.assertLess(self.evaluate()['score'],full)

    def test_hidden_second_bay_is_not_forced_into_confirmed_candidates(self):
        self.valid[:300,:]=0
        self.mask[:300,:]=0
        report=self.evaluate()
        self.assertEqual(len(report['candidates']),1)
        self.assertFalse(report['pair_identity_supported'])
        self.assertIsNone(report['candidates'][0].get('adjacent_group'))

    def test_blank_image_and_uniform_white_are_not_parking(self):
        for value in (0,255):
            self.mask[:]=value
            self.assertEqual(self.evaluate()['candidates'],[])

    def test_parallel_fragments_without_depth_anchor_remain_hypotheses(self):
        self.mask[:]=0
        for x in (.36,.74,1.12):
            v=int(round(600-x*400))
            cv2.line(self.mask,(650,v),(1000,v),255,5)
        self.assertEqual(self.evaluate()['candidates'],[])

    def test_detector_handles_a_cropped_back_without_old_rule_chain(self):
        self.valid[:,815:]=0
        self.mask[:,815:]=0
        slots,diag=self.model.detect(self.mask,self.valid)
        self.assertGreaterEqual(len(slots),1)
        self.assertEqual(diag['algorithm'],'partial_model_v1')
        self.assertLessEqual(diag['model']['evaluated'],320)

    def test_missing_visibility_mask_does_not_assume_black_is_observed(self):
        slots,diag=self.model.detect(self.mask,None)
        self.assertEqual(slots,[])
        self.assertEqual(diag['model']['reason'],'visibility_unavailable')


if __name__=='__main__':
    unittest.main()
