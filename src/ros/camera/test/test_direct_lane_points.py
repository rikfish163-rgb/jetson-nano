import unittest
import numpy as np
from test_unified_center_path import UnifiedCenterPathTests


class DirectLanePointsTests(UnifiedCenterPathTests):
    def test_two_observed_left_points_generate_linear_normal_path(self):
        left = self.boundary('left', .3)
        left = [p if i in (2, 3) else None for i, p in enumerate(left)]
        result = self.c.build_center_points(left, [None] * 10, 242)
        path = self.c.select_nearest_lane_path_segment(
            self.c.build_center_segments(result))

        self.assertEqual(len(path), 2)
        for point in path:
            self.assertEqual(point['reference_side'], 'left')
            self.assertEqual(point['lane_mode'], self.c.LANE_MODE_LEFT_ONLY)
            self.assertEqual(point['center_method'],
                             'fitted_boundary_normal')
            boundary_x = 240 + .3 * (point['y'] - 260) - 121 * np.hypot(1, .3)
            signed_distance = (point['x'] - boundary_x) / np.hypot(1, .3)
            self.assertAlmostEqual(signed_distance, 30 * self.c.PX_PER_CM,
                                   places=5)

    def test_single_observed_left_point_does_not_invent_direction(self):
        left = self.boundary('left', .3)
        left = [p if i == 2 else None for i, p in enumerate(left)]
        result = self.c.build_center_points(left, [None] * 10, 242)

        self.assertFalse(any(result))

    def test_exit_uses_left_solid_even_with_more_irregular_right_points(self):
        left=self.boundary('left',0.)
        left=[p if i < 4 else None for i,p in enumerate(left)]
        right=self.boundary('right',0.)
        for i,p in enumerate(right):p['x']+=60 if i%2 else -40
        self.c.track_boundaries_once=lambda *args:(left,right)
        self.c.find_dashed_left_boundary=lambda mask:None
        r=self.c.track_metric_lane(np.zeros((400,480),np.uint8),source_stamp=10.)
        points=[p for p in r['center_points'] if p]
        self.assertEqual(len(points),4)
        for p in points:
            self.assertEqual(p['reference_side'],'left')
            self.assertAlmostEqual(p['x'],119.+30*self.c.PX_PER_CM)

    def test_generation_does_not_call_outlier_or_resampling_filter(self):
        def forbidden(*args):
            raise AssertionError('outlier/resampling filter called')
        self.c.fit_continuous_center_points=forbidden
        left=self.boundary('left',0.)
        left[2]['x']+=45.
        left[3]=None
        result=self.c.build_center_points(left,[None]*10,242)
        observed=[p for p in left if self.c.boundary_point_is_reliable(p,True)]
        self.assertEqual(len([p for p in result if p]),len(observed))

    def test_all_fragments_are_published_in_forward_order(self):
        points=[dict(x=x,y=y,window_index=i,usable=True)
                for i,(x,y) in enumerate(((240.,380.),(410.,340.),(200.,160.)))]
        diagnostic={}
        selected=self.c.select_nearest_lane_path_segment([[points[2]],[points[0]],[points[1]]],diagnostic)
        self.assertEqual(selected,points)
        self.assertEqual(diagnostic['reason'],'ok')
        self.assertEqual(self.c.build_center_segments(points),[points])

    def test_short_span_is_not_discarded_by_camera(self):
        points=[dict(x=240.,y=y,window_index=i,usable=True)
                for i,y in enumerate((300.,290.))]
        self.assertEqual(self.c.select_nearest_lane_path_segment([points]),points)


if __name__=='__main__':unittest.main()
