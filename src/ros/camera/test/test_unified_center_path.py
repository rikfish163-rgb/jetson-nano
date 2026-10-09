import imp
import os
import unittest
import numpy as np


class UnifiedCenterPathTests(unittest.TestCase):
    def setUp(self):
        self.c=imp.load_source('unified_center_test',os.path.join(
            os.path.dirname(__file__),'../scripts/camera_yihan_web.py'))

    def boundary(self,side,slope=.3):
        offset=121*np.hypot(1,slope)
        return [dict(x=240+slope*(y-260)+(-offset if side=='left' else offset),
                     y=y,score=.9,observed=True,usable=True,geometry_ok=True)
                for y in range(380,-1,-40)]

    def test_left_right_and_both_use_same_normal_geometry(self):
        left,right=self.boundary('left'),self.boundary('right')
        empty=[None]*10
        for a,b in ((empty,right),(left,right)):
            points=self.c.build_center_points(a,b,242)
            path=self.c.select_nearest_lane_path_segment(self.c.build_center_segments(points))
            self.assertGreaterEqual(len(path),3)
            for p in path:
                self.assertAlmostEqual(p['x'],240+.3*(p['y']-260),places=5)
                self.assertEqual(p['center_method'],'fitted_boundary_normal')

    def test_left_only_path_is_30cm_to_right_for_straight_and_slanted_boundary(self):
        for slope in (0.,.3,-.3):
            left=self.boundary('left',slope)
            points=self.c.build_center_points(left,[None]*10,242)
            path=self.c.select_nearest_lane_path_segment(self.c.build_center_segments(points))
            self.assertGreaterEqual(len(path),3)
            for p in path:
                boundary_x=240+slope*(p['y']-260)-121*np.hypot(1,slope)
                signed_distance=(p['x']-boundary_x)/np.hypot(1,slope)
                self.assertAlmostEqual(signed_distance,30*self.c.PX_PER_CM,places=5)

    def test_partial_opposite_boundary_never_changes_method_per_window(self):
        left,right=self.boundary('left'),self.boundary('right')
        right[1]=None
        right[3]=None
        points=self.c.build_center_points(left,right,242)
        for p in points:
            if p:
                self.assertAlmostEqual(p['x'],240+.3*(p['y']-260),places=5)
                self.assertEqual(p['reference_side'],'left')

    def test_no_observed_boundary_means_no_invented_path(self):
        points=self.boundary('left')
        for p in points:p['observed']=False
        self.assertFalse(any(self.c.build_center_points(points,[None]*10,242)))

    def test_curved_left_outer_line_is_30cm_from_axis_without_body_width(self):
        radius=360.
        for sign in (-1.,1.):
            cy=420.;cx=240.-sign*radius
            left=[dict(x=cx+sign*np.sqrt(radius**2-(y-cy)**2),
                       y=y,score=.9,observed=True,usable=True,geometry_ok=True)
                  for y in range(380,139,-40)]
            points=self.c.build_center_points(left,[None]*len(left),242,
                                             preferred_side='left')
            for p in points:
                distance=np.hypot(p['x']-cx,p['y']-cy)
                self.assertAlmostEqual(distance,radius+sign*120.,places=5)
                self.assertEqual(p['reference_side'],'left')

    def test_no_dashed_frame_uses_unified_pipeline(self):
        self.c.find_dashed_left_boundary=lambda mask:None
        def forbidden(*args,**kwargs):raise AssertionError('legacy center branch called')
        self.c.smooth_center_points_spatially=forbidden
        self.c.stabilize_center_points=forbidden
        mask=np.zeros((400,480),np.uint8)
        mask[120:400,115:125]=255
        mask[120:400,357:367]=255
        r=self.c.track_metric_lane(mask)
        self.assertGreaterEqual(len(self.c.select_nearest_lane_path_segment(r['center_segments'])),3)

    def test_recent_boundary_search_can_recover_when_bottom_histogram_is_empty(self):
        self.c.previous_tracking_time=10.
        self.c.previous_tracking_confidence=.2
        calls=[]
        def tracking(mask,width,use_previous):
            calls.append(use_previous)
            return (self.boundary('left'),[None]*10) if use_previous else ([None]*10,[None]*10)
        self.c.track_boundaries_once=tracking
        self.c.find_dashed_left_boundary=lambda mask:None
        r=self.c.track_metric_lane(np.zeros((400,480),np.uint8),source_stamp=10.1)
        self.assertEqual(calls,[False,True])
        self.assertGreaterEqual(len(self.c.select_nearest_lane_path_segment(r['center_segments'])),3)


if __name__=='__main__':unittest.main()
