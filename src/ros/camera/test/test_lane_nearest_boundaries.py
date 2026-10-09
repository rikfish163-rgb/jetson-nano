"""Nearest ego-lane selection; imports ROS types without starting ROS nodes."""
import imp
import os
import unittest

import numpy as np

lane = imp.load_source('lane_nearest_test', os.path.join(
    os.path.dirname(__file__), '..', 'scripts', 'camera_yihan_web.py'))


class NearestBoundaryTests(unittest.TestCase):
    def setUp(self):
        lane.configure_lane_windows(20, 0.10)
        lane.previous_tracking_confidence = 0.0
        lane.previous_tracking_time = None
        lane.lane_width_est_px = 242.0
        lane.lane_width_history.clear()
        self.mask = np.zeros((400, 480), np.uint8)

    def line(self, x, width=5, y0=120, y1=400):
        self.mask[y0:y1, x-width//2:x+width//2+1] = 255

    def test_initialization_selects_nearest_not_strongest_on_each_side(self):
        self.line(120, 3)
        self.line(362, 3)
        self.line(55, 13)
        self.line(420, 13)
        left, right, _, _ = lane.initialize_lane_bases(self.mask, 242.0)
        self.assertAlmostEqual(left, 120.0, delta=2)
        self.assertAlmostEqual(right, 362.0, delta=2)

    def test_side_search_includes_line_close_to_vehicle_center(self):
        self.line(258)
        left, right, _, _ = lane.initialize_lane_bases(self.mask, 242.0)
        self.assertIsNone(left)
        self.assertAlmostEqual(right, 258.0, delta=2)

    def test_side_search_includes_line_at_outer_edge(self):
        self.line(15)
        left, right, _, _ = lane.initialize_lane_bases(self.mask, 242.0)
        self.assertAlmostEqual(left, 15.0, delta=2)
        self.assertIsNone(right)

    def test_near_band_takes_priority_over_farther_band(self):
        self.line(330, 5, 380, 400)
        self.line(285, 9, 320, 360)
        _, right, _, _ = lane.initialize_lane_bases(self.mask, 242.0)
        self.assertAlmostEqual(right, 330.0, delta=2)

    def test_empty_near_band_falls_back_for_dashed_line(self):
        self.line(330, 5, 330, 350)
        _, right, _, _ = lane.initialize_lane_bases(self.mask, 242.0)
        self.assertAlmostEqual(right, 330.0, delta=2)

    def test_initial_gap_uses_same_budget_as_an_existing_track(self):
        for height in (20, 40):
            lane.configure_lane_windows(height, .15)
            self.mask.fill(0)
            self.line(360, 5, 120, 320)  # 80 blank pixels, then a visible stripe
            result = lane.track_metric_lane(self.mask)
            self.assertEqual(result['lane_mode'], lane.LANE_MODE_RIGHT_ONLY)
            path = lane.select_nearest_lane_path_segment(result['center_segments'])
            self.assertGreaterEqual(len(path), 3)
            self.assertGreaterEqual(result['tracking_confidence'], .35)
            for point in path:
                self.assertLess(point['source_y'], 320)  # no invented near centers

    def test_initialization_does_not_jump_across_a_long_unseen_gap(self):
        for height in (20, 40):
            lane.configure_lane_windows(height, .15)
            self.mask.fill(0)
            self.line(360, 5, 120, 270)
            result = lane.track_metric_lane(self.mask)
            self.assertEqual(lane.select_nearest_lane_path_segment(result['center_segments']), [])

    def test_parallel_slanted_stripes_are_not_averaged_into_one_boundary(self):
        for y in range(360, 400):
            for base in (310, 330):
                x = base+y-380
                self.mask[y, x-2:x+3] = 255
        x, _, _ = lane.find_histogram_peak(self.mask, 360, 400, 320, 80, 80)
        if x is not None:
            self.assertLess(min(abs(x-310), abs(x-330)), 3)

    def test_far_arm_of_one_hairpin_does_not_become_opposite_boundary(self):
        lane.configure_lane_windows(40, .15)
        for y in range(220, 320):
            dx = 150*np.sqrt(1-((320-y)/100.0)**2)
            for x in (int(round(240-dx)), int(round(240+dx))):
                self.mask[y, x-2:x+3] = 255
        self.line(390, 5, 320, 400)
        result = lane.track_metric_lane(self.mask)
        self.assertEqual(result['lane_mode'], lane.LANE_MODE_RIGHT_ONLY)
        self.assertEqual(result['left_valid_count'], 0)

    def test_wide_white_patch_does_not_hide_narrow_candidate(self):
        self.mask[320:400, 320:445] = 255
        self.line(270, 5, 320, 400)
        _, right, _, _ = lane.initialize_lane_bases(self.mask, 242.0)
        self.assertAlmostEqual(right, 270.0, delta=2)

    def test_slanted_thin_stripe_survives_40_pixel_window(self):
        for slope in (-1.5, -1.0, 1.0, 1.5):
            self.mask.fill(0)
            for y in range(360, 400):
                x = int(round(320+slope*(y-380)))
                self.mask[y, x-2:x+3] = 255
            x, quality, _ = lane.find_histogram_peak(
                self.mask, 360, 400, 320, 80, 80)
            self.assertIsNotNone(x, 'slope=%s' % slope)
            self.assertAlmostEqual(x, 320, delta=2)
            self.assertGreaterEqual(quality, lane.MIN_GEOMETRY_SCORE)

    def test_wide_patch_and_horizontal_mark_are_not_slanted_boundaries(self):
        for y0, y1, x0, x1 in ((360, 400, 270, 370), (374, 384, 245, 395)):
            self.mask.fill(0)
            self.mask[y0:y1, x0:x1] = 255
            x, _, _ = lane.find_histogram_peak(
                self.mask, 360, 400, 320, 80, 80)
            self.assertIsNone(x)

    def test_cropped_slanted_stripe_requests_wider_search(self):
        for y in range(320, 360):
            x = 290+y-340
            self.mask[y, x-2:x+3] = 255
        x, _, _ = lane.find_histogram_peak(self.mask, 320, 360, 330, 36, 30)
        self.assertIsNone(x)
        x, _, _ = lane.find_histogram_peak(self.mask, 320, 360, 330, 56, 48)
        self.assertAlmostEqual(x, 290, delta=1)

    def test_left_bend_produces_center_inside_single_right_boundary(self):
        lane.configure_lane_windows(40, .15)
        lane.lane_width_est_px = 240.0
        for y in range(120, 400):
            x = 350+y-400
            self.mask[y, x-2:x+3] = 255
        result = lane.track_metric_lane(self.mask)
        self.assertEqual(result['lane_mode'], lane.LANE_MODE_RIGHT_ONLY)
        path = lane.select_nearest_lane_path_segment(result['center_segments'])
        self.assertGreaterEqual(len(path), 3)
        self.assertGreaterEqual(result['tracking_confidence'], .35)
        for point in path:
            # x = y-50 is the painted right edge; the center is 30 cm
            # (120 BEV pixels) to its left along the normal, not on the paint.
            distance = ((point['y']-50)-point['x']) / np.sqrt(2.0)
            self.assertAlmostEqual(distance, 120.0, delta=3)
        metric = [lane.bev_point_to_vehicle_m(p['x'], p['y']) for p in path]
        target = min(metric, key=lambda p: abs(np.hypot(*p)-.55))
        self.assertGreater(target[1], .05)  # positive lateral target commands LEFT

    def test_local_tracking_prefers_continuity_over_stronger_parallel_line(self):
        self.line(350, 3)
        self.line(375, 13)
        x, _, _ = lane.find_histogram_peak(self.mask, 380, 400, 350, 36, 30)
        self.assertAlmostEqual(x, 350.0, delta=2)

    def test_new_nearer_boundary_replaces_previous_outer_track(self):
        self.line(410, 9)
        old_left, old_right = lane.track_boundaries_once(self.mask, 242.0, False)
        lane.previous_left_points = old_left
        lane.previous_right_points = old_right
        self.line(305, 5)
        _, right = lane.track_boundaries_once(self.mask, 242.0, True)
        self.assertTrue(right[0]['observed'])
        self.assertAlmostEqual(right[0]['x'], 305.0, delta=2)

    def test_center_history_does_not_keep_old_outer_boundary(self):
        self.line(410, 9)
        for _ in range(3):
            lane.track_metric_lane(self.mask)
        self.line(305, 5)
        result = lane.track_metric_lane(self.mask)
        path = lane.select_nearest_lane_path_segment(result['center_segments'])
        self.assertGreaterEqual(len(path), 3)
        self.assertAlmostEqual(path[0]['x'], 184.0, delta=2)

    def test_right_only_builds_center_without_inventing_left_boundary(self):
        self.line(362)
        result = lane.track_metric_lane(self.mask)
        self.assertEqual(result['lane_mode'], lane.LANE_MODE_RIGHT_ONLY)
        self.assertEqual(result['left_valid_count'], 0)
        path = lane.select_nearest_lane_path_segment(result['center_segments'])
        self.assertGreaterEqual(len(path), 3)
        self.assertAlmostEqual(path[0]['x'], 241.0, delta=2)
        self.assertGreaterEqual(result['tracking_confidence'], .30)

    def test_left_only_builds_center_without_inventing_right_boundary(self):
        self.line(120)
        result = lane.track_metric_lane(self.mask)
        self.assertEqual(result['lane_mode'], lane.LANE_MODE_LEFT_ONLY)
        self.assertEqual(result['right_valid_count'], 0)
        path = lane.select_nearest_lane_path_segment(result['center_segments'])
        self.assertGreaterEqual(len(path), 3)
        self.assertAlmostEqual(path[0]['x'], 241.0, delta=2)

    def test_two_nearest_boundaries_build_midpoint(self):
        for x in (40, 120, 362, 440):
            self.line(x)
        result = lane.track_metric_lane(self.mask)
        self.assertEqual(result['lane_mode'], lane.LANE_MODE_BOTH_SIDES)
        path = lane.select_nearest_lane_path_segment(result['center_segments'])
        self.assertGreaterEqual(len(path), 3)
        self.assertAlmostEqual(path[0]['x'], 241.0, delta=2)

    def test_no_lines_remains_no_lane(self):
        result = lane.track_metric_lane(self.mask)
        self.assertEqual(result['lane_mode'], lane.LANE_MODE_NONE)
        self.assertEqual(lane.select_nearest_lane_path_segment(result['center_segments']), [])

    def test_one_stripe_crossing_vehicle_center_cannot_be_both_boundaries(self):
        lane.configure_lane_windows(40, .15)
        for y in range(120,400):
            x = int(round(236.0+.45*(y-380)))
            self.mask[y,x-4:x+5] = 255
        result = lane.track_metric_lane(self.mask)
        self.assertNotEqual(result['lane_mode'], lane.LANE_MODE_BOTH_SIDES)
        for left,right in zip(result['left_points'],result['right_points']):
            if lane.boundary_point_is_reliable(left) and lane.boundary_point_is_reliable(right):
                self.assertGreater(abs(left['x']-right['x']),lane.MAX_PEAK_WIDTH)
        for point in lane.select_nearest_lane_path_segment(result['center_segments']):
            # A valid center cannot lie directly on the only painted stripe.
            self.assertGreater(abs(point['x']-(236+.45*(point['y']-380))),60)
        self.assertEqual(lane.make_lane_tracking_debug(self.mask,result).shape,(400,480,3))


if __name__ == '__main__':
    unittest.main()
