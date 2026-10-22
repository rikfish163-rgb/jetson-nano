"""The dashed line remains LEFT after the car drifts across it."""
import imp
import os
import unittest
import cv2
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..'))


class DashedLeftTests(unittest.TestCase):
    def setUp(self):
        self.camera = imp.load_source('dashed_left_test', os.path.join(
            ROOT, 'src/ros/camera/scripts/camera_yihan_web.py'))

    def test_recorded_wrong_lane_frames_recover_right_of_dashed(self):
        from vehicle_control.pure_pursuit import PurePursuit
        for sec in (71, 74, 76):
            mask = cv2.imread(os.path.join(ROOT,
                'field_data/right_drift_20260929/%d_mask.png' % sec), 0)
            result = self.camera.track_metric_lane(mask)
            self.assertTrue(result.get('dashed_left'), sec)
            self.assertEqual(result['lane_mode'], self.camera.LANE_MODE_LEFT_ONLY)
            path = self.camera.select_nearest_lane_path_segment(result['center_segments'])
            self.assertGreaterEqual(len(path), 3)
            self.assertGreater(result['tracking_confidence'], .35)
            # Vehicle y is positive left: negative steering returns toward our lane.
            points = [self.camera.bev_point_to_vehicle_m(p['x'], p['y']) for p in path]
            self.assertLess(PurePursuit(.26, .70).compute(points).steering_angle, 0.)
            left = [p for p in result['left_points'] if p.get('observed')]
            fit = np.polyfit([p['y'] for p in left], [p['x'] for p in left], 2)
            for point in path:
                self.assertGreater(point['x'], np.polyval(fit, point['y']) + 50.)

    def test_continuous_solids_and_horizontal_marker_are_not_dashes(self):
        mask = np.zeros((400, 480), np.uint8)
        mask[120:400, 115:125] = 255
        mask[120:400, 355:365] = 255
        mask[250:262, 120:360] = 255
        self.assertIsNone(self.camera.find_dashed_left_boundary(mask))

    def test_recorded_exit_wall_fragments_are_not_a_dashed_divider(self):
        mask=cv2.imread(os.path.join(os.path.dirname(__file__),
            'data/exit_wall_false_dashes.png'),0)
        self.assertIsNotNone(mask)
        self.assertIsNone(self.camera.find_dashed_left_boundary(mask))

    def test_tiny_cracks_in_solid_line_do_not_establish_dashed_identity(self):
        mask=np.zeros((400,480),np.uint8)
        for y in range(130,380,25):
            mask[y:y+23,115:125]=255
        self.assertIsNone(self.camera.find_dashed_left_boundary(mask))

    def test_two_equally_plausible_dashed_lines_are_ambiguous(self):
        mask = np.zeros((400, 480), np.uint8)
        for y in range(130, 380, 40):
            mask[y:y+18, 110:120] = 255
            mask[y:y+18, 350:360] = 255
        self.assertIsNone(self.camera.find_dashed_left_boundary(mask))

    def test_dashed_identity_is_independent_of_horizontal_position(self):
        for x in (110, 300):
            mask = np.zeros((400, 480), np.uint8)
            for y in range(130, 380, 40):
                mask[y:y+18, x:x+10] = 255
            # Outside-left solid must not become the lane's LEFT boundary.
            mask[120:400, 15:25] = 255
            result = self.camera.track_metric_lane(mask)
            self.assertTrue(result['dashed_left'])
            self.assertEqual(result['lane_mode'], self.camera.LANE_MODE_LEFT_ONLY)
            path = self.camera.select_nearest_lane_path_segment(result['center_segments'])
            self.assertGreaterEqual(len(path), 3)
            for p in path:
                self.assertAlmostEqual(p['x'], x+4.5+30*self.camera.PX_PER_CM)

    def test_visible_right_solid_pairs_with_dashed_left(self):
        mask = np.zeros((400, 480), np.uint8)
        for y in range(130, 380, 40):
            mask[y:y+18, 115:125] = 255
        mask[120:400, 357:367] = 255
        result = self.camera.track_metric_lane(mask)
        self.assertTrue(result['dashed_left'])
        self.assertEqual(result['lane_mode'], self.camera.LANE_MODE_BOTH_SIDES)
        path = self.camera.select_nearest_lane_path_segment(result['center_segments'])
        self.assertGreaterEqual(len(path), 3)
        for p in path:
            self.assertLess(abs(p['x']-240), 5.)


if __name__ == '__main__':
    unittest.main()
