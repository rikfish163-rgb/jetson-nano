# -*- coding: utf-8 -*-

from __future__ import division

import os
import sys
import unittest


SCRIPTS_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts")
)
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from parking_controller import (  # noqa: E402
    STEERING_DEG_PER_RAW,
    STEERING_MAX_ANGLE_DEG,
    STEERING_MAX_ANGLE_RAD,
    STEERING_RAD_PER_RAW,
    STEERING_RAW_LIMIT,
    steering_angle_deg_to_raw,
    steering_angle_rad_to_raw,
    steering_raw_to_angle_deg,
)


class ParkingSteeringMappingTest(unittest.TestCase):
    def test_measured_scale_is_22_equal_steps(self):
        self.assertAlmostEqual(
            STEERING_DEG_PER_RAW, 26.515 / 22.0, places=12)
        self.assertAlmostEqual(
            STEERING_RAD_PER_RAW, 0.46275 / 22.0, places=12)
        self.assertEqual(STEERING_RAW_LIMIT, 22)
        self.assertAlmostEqual(STEERING_MAX_ANGLE_DEG, 26.515, places=12)
        self.assertAlmostEqual(STEERING_MAX_ANGLE_RAD, 0.46275, places=12)

    def test_degree_to_raw_rounds_magnitude_up_and_preserves_sign(self):
        self.assertEqual(steering_angle_deg_to_raw(0.0), 0)
        self.assertEqual(steering_angle_deg_to_raw(1.0), 1)
        self.assertEqual(steering_angle_deg_to_raw(12.0), 10)
        self.assertEqual(steering_angle_deg_to_raw(-12.0), -10)
        self.assertEqual(steering_angle_deg_to_raw(12.052272), 10)
        self.assertEqual(steering_angle_deg_to_raw(12.052273), 11)
        self.assertEqual(steering_angle_rad_to_raw(0.21034090), 10)
        self.assertEqual(steering_angle_rad_to_raw(-0.21034090), -10)

    def test_degree_to_raw_is_bounded(self):
        self.assertEqual(steering_angle_deg_to_raw(26.515), 22)
        self.assertEqual(steering_angle_deg_to_raw(40.0), 22)
        self.assertEqual(steering_angle_deg_to_raw(-40.0), -22)

    def test_raw_to_degree_is_exact_and_bounded(self):
        self.assertAlmostEqual(
            steering_raw_to_angle_deg(22), 26.515, places=12)
        self.assertAlmostEqual(
            steering_raw_to_angle_deg(-10), -10 * 26.515 / 22.0,
            places=12)
        with self.assertRaises(ValueError):
            steering_raw_to_angle_deg(23)
        with self.assertRaises(ValueError):
            steering_raw_to_angle_deg(1.0)


if __name__ == "__main__":
    unittest.main()
