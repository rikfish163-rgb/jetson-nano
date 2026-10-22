"""Center geometry regressions; imports ROS messages but starts no ROS nodes."""
import imp
import math
import os
import unittest

import numpy as np


SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'scripts',
                      'camera_yihan_web.py')
lane = imp.load_source('lane_center_geometry_test', SCRIPT)


class CenterGeometryTests(unittest.TestCase):
    def setUp(self):
        lane.configure_lane_windows(20, 0.10)

    def test_parking_boundaries_are_observed_paint_without_offset(self):
        for side, x in (('left',120.), ('right',360.)):
            raw = [None]*lane.NUM_WINDOWS
            for i in range(6):
                raw[i] = dict(x=x,y=390.-20*i,window_index=i,score=1.,
                              usable=True,observed=True,geometry_ok=True)
            tracking = dict(left_points=[None]*lane.NUM_WINDOWS,
                            right_points=[None]*lane.NUM_WINDOWS)
            tracking[side+'_points'] = raw
            result = lane.build_observed_lane_boundaries(tracking,None)
            self.assertGreaterEqual(len(result[side.upper()]),3)
            for forward,lateral in result[side.upper()]:
                self.assertAlmostEqual(lateral,(240.-x)/400.)
            for point in raw:
                if point: point['observed'] = False
            self.assertEqual(lane.build_observed_lane_boundaries(tracking,None)[side.upper()],[])

    def test_parking_export_accepts_actual_tracker_points(self):
        lane.configure_lane_windows(40,.15)
        mask = np.zeros((400,480),np.uint8)
        mask[120:400,358:363] = 255
        for stamp in (10.,10.1,10.2):
            tracking = lane.track_metric_lane(mask,source_stamp=stamp)
            raw = [p for p in tracking['right_points'] if p and p.get('observed')]
            self.assertGreaterEqual(len(raw),3)
            result = lane.build_observed_lane_boundaries(tracking,None)
            self.assertGreaterEqual(len(result['RIGHT']),3)
            self.assertTrue(all('window_index' not in p for p in raw))

    def offset_centers(self, side, slope, intercept):
        boundary = [None] * lane.NUM_WINDOWS
        for i in range(6):
            y = 390.0 - 20.0 * i
            boundary[i] = dict(x=slope*y+intercept, y=y,
                               window_index=i, score=1.0, usable=True,
                               observed=True, geometry_ok=True)
        centers = [lane.estimate_normal_center(boundary, i, side, 242.0)
                   if boundary[i] else None for i in range(lane.NUM_WINDOWS)]
        return centers

    def assert_offset_preserved(self, side, slope, intercept):
        centers = self.offset_centers(side, slope, intercept)
        mode = lane.LANE_MODE_LEFT_ONLY if side == 'left' else lane.LANE_MODE_RIGHT_ONLY
        result = lane.smooth_center_points_spatially(centers, mode)
        # The live tracker smooths twice: this must not change the geometry.
        result = lane.smooth_center_points_spatially(result, mode)
        lower = min(p['y'] for p in centers if p is not None)
        upper = max(p['y'] for p in centers if p is not None)
        self.assertGreaterEqual(sum(p is not None for p in result), 3)
        for i, point in enumerate(result):
            if point is None:
                continue
            self.assertGreaterEqual(point['y'], lower)
            self.assertLessEqual(point['y'], upper)
            self.assertAlmostEqual(point['y'], upper - (upper-lower)*i/5.0)
            distance = abs(point['x']-slope*point['y']-intercept)/math.hypot(1, slope)
            self.assertAlmostEqual(distance, 121.0, places=6)

    def test_left_only_keeps_normal_offset_and_forward_coordinate(self):
        self.assert_offset_preserved('left', 0.45, -20.0)

    def test_right_only_keeps_normal_offset_and_forward_coordinate(self):
        self.assert_offset_preserved('right', -0.45, 480.0)

    def test_normal_center_outside_image_is_not_clamped_before_fitting(self):
        self.assert_offset_preserved('right', 1.0, -50.0)
        centers = self.offset_centers('right', 1.0, -50.0)
        self.assertGreater(centers[0]['y'], lane.BEV_HEIGHT)
        result = lane.smooth_center_points_spatially(centers, lane.LANE_MODE_RIGHT_ONLY)
        path = lane.select_nearest_lane_path_segment(lane.build_center_segments(result))
        self.assertGreaterEqual(len(path), 3)
        self.assertGreater(path[0]['y'], lane.BEV_HEIGHT)
        for point in path:
            self.assertTrue(lane.point_in_trusted_control_region(point))
            self.assertGreater(lane.bev_point_to_vehicle_m(point['x'], point['y'])[0], .05)

    def test_untrusted_boundary_cannot_enter_path_through_normal_offset(self):
        points = [dict(x=240., y=200.-i*20, source_y=100.-i*20,
                       window_index=i, usable=True) for i in range(4)]
        self.assertEqual(lane.select_nearest_lane_path_segment([points]), [])

    def test_straight_single_boundary_is_unchanged(self):
        self.assert_offset_preserved('left', 0.0, 100.0)

    def test_fit_uses_center_y_not_boundary_source_y(self):
        centers = self.offset_centers('left', 0.45, -20.0)
        fit = lane.fit_center_curve_robust(centers)
        known = [p for p in centers if p is not None]
        self.assertAlmostEqual(fit['min_y'], min(p['y'] for p in known))
        self.assertAlmostEqual(fit['max_y'], max(p['y'] for p in known))
        for point in known:
            self.assertAlmostEqual(np.polyval(fit['coefficients'], point['y']),
                                   point['x'], places=6)

    def test_gap_fill_interpolates_shifted_coordinates_without_extrapolation(self):
        centers = [None] * lane.NUM_WINDOWS
        for i in (2, 3, 5, 6):
            source_y = 390.0 - 20.0*i
            y = source_y - 70.0
            centers[i] = dict(x=0.4*y+100, y=y, source_y=source_y,
                              window_index=i, score=1.0, usable=True)
        result = lane.smooth_center_points_spatially(centers, lane.LANE_MODE_LEFT_ONLY)
        # Keep the supported windows, but place them in the true y=200..280 interval.
        self.assertAlmostEqual(result[4]['y'], 240.0)
        self.assertAlmostEqual(result[4]['x'], 196.0)
        for i in (0, 1, 7, 8, 9, 10, 11, 12, 13):
            self.assertIsNone(result[i])
        ys = [p['y'] for p in result if p is not None]
        self.assertTrue(all(a > b for a, b in zip(ys, ys[1:])))

    def test_two_side_unshifted_center_is_unchanged(self):
        centers = [None] * lane.NUM_WINDOWS
        for i in range(6):
            y = 390.0 - 20.0*i
            centers[i] = dict(x=240.0, y=y, source_y=y, window_index=i,
                              score=1.0, usable=True, observed=True)
        result = lane.smooth_center_points_spatially(centers, lane.LANE_MODE_BOTH_SIDES)
        for original, point in zip(centers, result):
            if original is None:
                self.assertIsNone(point)
            else:
                self.assertAlmostEqual(point['x'], original['x'])
                self.assertAlmostEqual(point['y'], original['y'])

    def test_mixed_offsets_remain_one_forward_ordered_segment(self):
        centers = [None] * lane.NUM_WINDOWS
        for i, y in enumerate((390.0, 370.0, 285.0, 330.0, 270.0, 290.0)):
            centers[i] = dict(x=0.3*y+100, y=y, source_y=390.0-i*20,
                              window_index=i, usable=True, score=1.0)
        result = lane.smooth_center_points_spatially(centers, lane.LANE_MODE_BOTH_SIDES)
        segments = lane.build_center_segments(result)
        self.assertEqual(len(segments), 1)
        path = lane.select_nearest_lane_path_segment(segments)
        self.assertEqual(len(path), 6)
        self.assertTrue(all(a['y'] > b['y'] for a, b in zip(path, path[1:])))
        for point in path:
            self.assertAlmostEqual(point['x'], 0.3*point['y']+100, places=6)

    def test_recorded_bend_does_not_create_a_false_near_target(self):
        # Normal-offset centers from front capture at 18:27:16.64, before fitting.
        values = [(174.4465075213, 300.8491939176),
                  (153.7874525276, 280.8491939176),
                  (148.4290436308, 279.7541459446),
                  (141.3554926714, 258.6290268180),
                  (119.0192857749, 238.6290268180)]
        centers = [None] * lane.NUM_WINDOWS
        for i, (x, y) in enumerate(values):
            centers[i] = dict(x=x, y=y, source_y=390.0-i*20,
                              window_index=i, usable=True, score=1.0,
                              compensated=True)
        result = lane.smooth_center_points_spatially(centers, lane.LANE_MODE_LEFT_ONLY)
        path = lane.select_nearest_lane_path_segment(lane.build_center_segments(result))
        self.assertEqual(len(path), 5)
        nearest_x, unused_y = lane.bev_point_to_vehicle_m(path[0]['x'], path[0]['y'])
        self.assertAlmostEqual(nearest_x, (600.0-values[0][1])/400.0, places=6)
        self.assertGreater(nearest_x, 0.74)

    def test_temporal_filter_does_not_move_points_to_previous_forward_position(self):
        centers = self.offset_centers('left', .45, -20.)
        lane.previous_center_points = [dict(p, y=p['y']+60) if p else None
                                       for p in centers]
        result = lane.stabilize_center_points(centers, .3)
        for actual, expected in zip(result, centers):
            if expected:
                self.assertAlmostEqual(actual['y'], expected['y'])

    def test_temporal_filter_cannot_freeze_on_a_persistent_bend(self):
        centers = self.offset_centers('left', 0., 100.)
        lane.previous_center_points = [dict(p,x=p['x']-80) if p else None for p in centers]
        for _ in range(6):
            result = lane.stabilize_center_points(centers, .4)
            lane.previous_center_points = result
        for actual,expected in zip(result,centers):
            if expected:
                self.assertAlmostEqual(actual['x'],expected['x'],delta=3.)

    def test_current_40_pixel_windows_preserve_normal_geometry(self):
        lane.configure_lane_windows(40, .15)
        boundary = [dict(x=.45*(380-i*40)-20, y=float(380-i*40),
                         window_index=i, score=1., usable=True,
                         observed=True, geometry_ok=True)
                    for i in range(lane.NUM_WINDOWS)]
        centers = [lane.estimate_normal_center(boundary, i, 'left', 240.)
                   for i in range(lane.NUM_WINDOWS)]
        result = lane.smooth_center_points_spatially(centers, lane.LANE_MODE_LEFT_ONLY)
        path = lane.select_nearest_lane_path_segment(lane.build_center_segments(result))
        self.assertGreaterEqual(len(path), 3)
        for p in path:
            distance = abs(p['x']-.45*p['y']+20)/math.hypot(1,.45)
            self.assertAlmostEqual(distance, 120., places=5)


if __name__ == '__main__':
    unittest.main()
