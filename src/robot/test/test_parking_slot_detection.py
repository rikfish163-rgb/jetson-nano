"""Parity checks for the side-bay Hough detector; no ROS or vehicle I/O."""
from __future__ import division

import copy
import os
import unittest

import cv2
import numpy as np
import yaml

from robot.camera.vision import GroundDetector


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
with open(os.path.join(ROOT, 'config', 'competition.yaml')) as stream:
    CONFIG = yaml.safe_load(stream)


class ParkingSlotDetectionTests(unittest.TestCase):
    def setUp(self):
        self.cfg = copy.deepcopy(CONFIG)
        self.detector = GroundDetector(self.cfg)
        cv2.setNumThreads(1)

    def mask_three_bays(self):
        """Use the measured-frame benchmark geometry from the Nano profile."""
        mask = np.zeros((600, 1200), np.uint8)
        camera = self.cfg['front_camera']
        ppm = camera['pixels_per_m']
        offset = 360

        def pixel(x, y):
            return (int(round(camera['origin_u'] + offset - y * ppm)),
                    int(round(camera['origin_v'] - x * ppm)))

        for centre_y in (-.70, 0., .70):
            corners = [pixel(x, y) for x, y in (
                (.35, centre_y - .18), (1.05, centre_y - .18),
                (1.05, centre_y + .18), (.35, centre_y + .18))]
            for first, second in zip(corners, corners[1:] + corners[:1]):
                cv2.line(mask, first, second, 255, 3)
        return mask

    def test_three_bay_output_matches_measured_geometry(self):
        rows = self.detector.detect_slots(self.mask_three_bays(), u_offset=360)
        self.assertEqual(len(rows), 3)
        rows = sorted(rows, key=lambda row: row['y'])
        for row, expected_y in zip(rows, (-.69875, 0., .70125)):
            self.assertEqual(row['kind'], 'parallel')
            self.assertAlmostEqual(row['x'], .7, delta=.002)
            self.assertAlmostEqual(row['y'], expected_y, delta=.002)
            self.assertAlmostEqual(row['yaw'], 0., delta=.002)
            self.assertAlmostEqual(row['length'], .70, delta=1e-9)
            self.assertAlmostEqual(row['width'], .36, delta=1e-9)

    def test_empty_mask_stays_empty(self):
        rows = self.detector.detect_slots(np.zeros((600, 1200), np.uint8),
                                          u_offset=360)
        self.assertEqual(rows, [])


if __name__ == '__main__':
    unittest.main()
