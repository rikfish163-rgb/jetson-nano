"""Recorded two-white-boundary bend must keep the target inside the dark lane."""
import imp
import os
import unittest

import cv2


lane = imp.load_source('lane_curve_corridor_test', os.path.join(
    os.path.dirname(__file__), '..', 'scripts', 'camera_yihan_web.py'))
FIXTURE = os.path.join(os.path.dirname(__file__), 'data',
                       'recorded_curve_corridor.png')


class CurveCorridorTests(unittest.TestCase):
    def setUp(self):
        lane.configure_lane_windows(40, .15)
        lane.previous_tracking_time = None
        lane.previous_tracking_confidence = 0.0
        lane.previous_left_points = [None] * lane.NUM_WINDOWS
        lane.previous_right_points = [None] * lane.NUM_WINDOWS
        lane.previous_center_points = [None] * lane.NUM_WINDOWS
        lane.boundary_association = None
        lane.bend_memory = None
        lane.lane_width_est_px = 240.0
        lane.lane_width_history.clear()

    def test_outer_solid_and_inner_dashed_paint_bound_the_target_path(self):
        mask = cv2.imread(FIXTURE, cv2.IMREAD_GRAYSCALE)
        self.assertIsNotNone(mask)
        for frame in range(4):
            result = lane.track_metric_lane(mask, source_stamp=1.0+.1*frame)
            self.assertEqual(result['lane_mode'], lane.LANE_MODE_BOTH_SIDES)
            self.assertGreaterEqual(result['tracking_confidence'], .35)
            path = lane.select_nearest_lane_path_segment(result['center_segments'])
            self.assertGreaterEqual(len(path), 3)
            left_by_row = {int(p['y']): p['x'] for p in result['left_points']
                           if p.get('observed')}
            right_by_row = {int(p['y']): p['x'] for p in result['right_points']
                            if p.get('observed')}
            for point in path:
                row = int(point['source_y'])
                self.assertLess(point['x'], right_by_row[row])
                if row in left_by_row:
                    self.assertGreater(point['x'], left_by_row[row])
                x, y = int(round(point['x'])), int(round(point['y']))
                if 0 <= x < mask.shape[1] and 0 <= y < mask.shape[0]:
                    self.assertEqual(mask[y, x], 0)

    def test_recorded_turn_preserves_right_edge_through_partial_occlusion(self):
        expected = ('BOTH_SIDES', 'RIGHT_ONLY', 'NO_LANE',
                    'RIGHT_ONLY', 'BOTH_SIDES')
        for frame, mode in enumerate(expected):
            mask = cv2.imread(os.path.join(os.path.dirname(__file__), 'data',
                                           'turn_%02d.png' % frame),
                              cv2.IMREAD_GRAYSCALE)
            self.assertIsNotNone(mask)
            result = lane.track_metric_lane(mask, source_stamp=1.0+.1*frame)
            self.assertEqual(result['lane_mode'], mode)
            path = lane.select_nearest_lane_path_segment(result['center_segments'])
            if mode == 'NO_LANE':
                self.assertEqual(result['tracking_confidence'], 0.0)
                self.assertEqual(path, [])
            elif mode == 'RIGHT_ONLY':
                self.assertGreaterEqual(len(path), 3)
                right_by_row = {int(p['y']): p['x'] for p in result['right_points']
                                if p.get('observed')}
                self.assertTrue(all(point['x'] < right_by_row[int(point['source_y'])]
                                    for point in path))


if __name__ == '__main__':
    unittest.main()
