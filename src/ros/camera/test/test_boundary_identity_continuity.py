"""A current measured divider must retain identity across a search relabel."""
import imp
import os
import unittest

ROOT=os.path.abspath(os.path.join(os.path.dirname(__file__),'../../../..'))


class BoundaryIdentityTests(unittest.TestCase):
    def setUp(self):
        self.camera=imp.load_source('boundary_identity_test',os.path.join(
            ROOT,'src/ros/camera/scripts/camera_yihan_web.py'))

    def rows(self,xs):
        return [dict(x=x,y=380.-40*i,observed=True,usable=True,
                     geometry_ok=True,score=.9) for i,x in enumerate(xs)]

    def test_same_paint_relabelled_right_keeps_left_reference(self):
        previous=self.rows([268,289,313,338,362,389,413])
        remote=self.rows([54,68,84,100,116,132,148])
        current=self.rows([270,292,316,341,365,392,416])
        self.camera.previous_left_points=previous
        self.camera.previous_center_points=[dict(p,reference_side='left') for p in previous]
        left,right,matched=self.camera.associate_left_reference(remote,current,242.,True)
        self.assertEqual(matched,'right_to_left')
        self.assertEqual(left,current)
        self.assertEqual(self.camera.count_observed(right),0)
        self.assertTrue(all(p.get('observed') for p in left))

    def test_expired_or_nonmatching_history_does_not_relabel_new_paint(self):
        previous=self.rows([120]*7)
        self.camera.previous_left_points=previous
        self.camera.previous_center_points=[dict(p,reference_side='left') for p in previous]
        left,right=self.rows([50]*7),self.rows([365]*7)
        for fresh in (False,True):
            a,b,matched=self.camera.associate_left_reference(left,right,242.,fresh)
            self.assertEqual((a,b),(left,right))
            self.assertIsNone(matched)

    def test_current_left_matching_history_keeps_both_boundaries(self):
        left,right=self.rows([120]*7),self.rows([362]*7)
        self.camera.previous_left_points=left
        self.camera.previous_center_points=[dict(p,reference_side='left') for p in left]
        a,b,matched=self.camera.associate_left_reference(left,right,242.,True)
        self.assertEqual((a,b),(left,right))
        self.assertEqual(matched,'left')

    def test_hidden_reference_cannot_be_replaced_by_one_distant_line(self):
        import numpy as np
        camera=self.camera
        old=self.rows([120]*7)
        remote=self.rows([365]*7)
        current=[old]
        camera.track_boundaries_once=lambda *args:(
            current[0]+[None]*(camera.NUM_WINDOWS-len(old)),[None]*camera.NUM_WINDOWS)
        camera.find_dashed_left_boundary=lambda mask:None
        mask=np.zeros((400,480),np.uint8)
        first=camera.track_metric_lane(mask,source_stamp=1.)
        self.assertGreater(camera.count_observed(first['left_points']),0)
        current[0]=remote
        for stamp in (1.1,1.4,1.6):
            result=camera.track_metric_lane(mask,source_stamp=stamp)
            self.assertEqual(camera.count_observed(result['left_points']),0)
            self.assertEqual(result['reference_association'],'pending_reference')
        # Seeing the original line again interrupts the wrong candidate.
        current[0]=old
        for stamp in (1.7,1.8):
            result=camera.track_metric_lane(mask,source_stamp=stamp)
            self.assertEqual(camera.count_observed(result['left_points']),0)
        result=camera.track_metric_lane(mask,source_stamp=1.9)
        self.assertEqual(result['reference_association'],'confirmed_new_left')
        self.assertEqual(camera.count_observed(result['left_points']),7)


if __name__=='__main__':unittest.main()
