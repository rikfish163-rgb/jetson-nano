"""User-specified two-bay layout: 76 cm back, 45 cm sides, seven 5 cm dashes."""
from __future__ import division
import unittest
import cv2
import numpy as np
from robot.parking.layout import detect_layout


class LayoutTest(unittest.TestCase):
    def scene(self,left=False,indices=tuple(range(7)),span=.76,crop=False):
        mask=np.zeros((600,1200),np.uint8)
        valid=np.ones_like(mask)
        metric=lambda u,v:((600-v)/400.,(600-u)/400.)
        mouth,back=(480,300) if left else (720,900)
        groups=[]
        def line(a,b):
            a,b=np.array(a,float),np.array(b,float)
            cv2.line(mask,tuple(a.astype(int)),tuple(b.astype(int)),255,5)
            groups.append([a,b,(b-a)/np.linalg.norm(b-a)])
        lo,hi=456,600-(.36+span)*400
        line((mouth,lo),(back,lo))
        line((mouth,hi),(back,hi))
        line((back,lo),(back,hi))
        for i in indices:
            v=int(round(600-(.405+i*.11)*400))
            cv2.line(mask,(mouth,v-10),(mouth,v+10),255,4)
        if crop:
            valid[:,min(mouth,back)-8:max(mouth,back)+8]=0
        cfg=dict(slots={'P4':dict(length=.45,width=.38)},
                 white={'dimension_tolerance':.08})
        return detect_layout(groups,mask,valid,metric,400.,cfg)

    def test_common_back_and_seven_dashes_locate_two_bays(self):
        slots,diag=self.scene()
        self.assertEqual(len(slots),2)
        self.assertEqual(diag['accepted_pairs'],1)
        self.assertAlmostEqual(slots[0]['x'],.55)
        self.assertAlmostEqual(slots[1]['x'],.93)
        self.assertTrue(all(s['evidence']=='common_back_dashed_mouth' for s in slots))
        self.assertTrue(all(not s['bottom_inferred'] for s in slots))

    def test_left_layout_is_mirrored(self):
        slots,unused=self.scene(left=True)
        self.assertEqual(len(slots),2)
        self.assertTrue(all(s['y']>0 for s in slots))

    def test_dashes_may_be_partly_hidden(self):
        self.assertEqual(len(self.scene(indices=(0,3,6))[0]),2)

    def test_one_dash_does_not_prove_a_double_bay(self):
        self.assertEqual(self.scene(indices=(3,))[0],[])

    def test_wrong_overall_width_is_rejected(self):
        self.assertEqual(self.scene(span=1.10)[0],[])

    def test_missing_image_support_is_rejected(self):
        self.assertEqual(self.scene(crop=True)[0],[])


if __name__=='__main__':
    unittest.main()
