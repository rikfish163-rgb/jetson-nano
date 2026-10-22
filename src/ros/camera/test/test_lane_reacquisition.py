"""Regression for recovering visible paint after a long tracking gap."""
import imp
import os
import unittest
import numpy as np


class ReacquisitionTests(unittest.TestCase):
    def test_crossing_image_center_preserves_boundary_role(self):
        lane = imp.load_source('lane_role_test', os.path.join(
            os.path.dirname(__file__), '..', 'scripts', 'camera_yihan_web.py'))
        lane.configure_lane_windows(40, .15)
        for i, x in enumerate((215, 230, 245, 260)):
            mask = np.zeros((400, 480), np.uint8)
            mask[120:400, x-2:x+3] = 255
            result = lane.track_metric_lane(mask, source_stamp=10+i*.1)
            self.assertEqual(result['lane_mode'], 'LEFT_ONLY')
            self.assertEqual(lane.make_lane_tracking_debug(mask, result).shape, (400, 480, 3))
        # Empty frames do not erase a recent association.
        lane.track_metric_lane(np.zeros_like(mask), source_stamp=10.4)
        result = lane.track_metric_lane(mask, source_stamp=10.5)
        self.assertEqual(result['lane_mode'], 'LEFT_ONLY')
        # Expired evidence cannot permanently lock out a newly visible lane.
        result = lane.track_metric_lane(mask, source_stamp=13.)
        self.assertEqual(result['lane_mode'], 'RIGHT_ONLY')

    def test_right_boundary_crossing_and_unrelated_paint_reinitialize(self):
        lane = imp.load_source('lane_right_role_test', os.path.join(
            os.path.dirname(__file__), '..', 'scripts', 'camera_yihan_web.py'))
        lane.configure_lane_windows(40, .15)
        for i, x in enumerate((265, 250, 235, 220)):
            mask = np.zeros((400, 480), np.uint8)
            mask[120:400, x-2:x+3] = 255
            result = lane.track_metric_lane(mask, source_stamp=10+i*.1)
            self.assertEqual(result['lane_mode'], 'RIGHT_ONLY')
            lane.make_lane_tracking_debug(mask, result)
        mask[:] = 0
        mask[120:400, 98:103] = 255
        result = lane.track_metric_lane(mask, source_stamp=10.4)
        self.assertEqual(result['lane_mode'], 'LEFT_ONLY')

    def test_same_lane_recovers_after_long_gap(self):
        lane = imp.load_source('lane_reacquisition_test', os.path.join(
            os.path.dirname(__file__), '..', 'scripts', 'camera_yihan_web.py'))
        lane.configure_lane_windows(40, .15)
        mask = np.zeros((400, 480), np.uint8)
        mask[120:400, 118:123] = 255
        mask[120:400, 358:363] = 255
        for i in range(4):
            lane.track_metric_lane(mask, source_stamp=10+i*.1)
        for i in range(8):
            result = lane.track_metric_lane(np.zeros_like(mask), source_stamp=10.4+i*.1)
        self.assertEqual(result['tracking_confidence'], 0.)
        result = lane.track_metric_lane(mask, source_stamp=11.2)
        self.assertEqual(result['path_diagnostic']['reason'], 'ok')
        self.assertGreaterEqual(result['path_diagnostic']['points'], 3)
        self.assertGreaterEqual(result['tracking_confidence'], .35)


if __name__ == '__main__':
    unittest.main()
