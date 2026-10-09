# -*- coding: utf-8 -*-

from __future__ import division

import os
import sys
import unittest


PACKAGE_SRC = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "src")
)
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from vehicle_control.pure_pursuit import PurePursuit


class PurePursuitTest(unittest.TestCase):
    def setUp(self):
        self.controller = PurePursuit(
            wheelbase=0.29,
            lookahead_distance=0.60,
            target_speed=0.30,
        )

    def test_straight_path_has_zero_steering(self):
        result = self.controller.compute(
            [(0.10, 0.0), (0.40, 0.0), (0.70, 0.0), (1.00, 0.0)]
        )
        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.steering_angle, 0.0, places=12)
        self.assertAlmostEqual(result.curvature, 0.0, places=12)

    def test_left_curve_has_positive_steering(self):
        result = self.controller.compute(
            [(0.10, 0.01), (0.40, 0.08), (0.70, 0.22), (1.00, 0.45)]
        )
        self.assertTrue(result.valid)
        self.assertGreater(result.curvature, 0.0)
        self.assertGreater(result.steering_angle, 0.0)

    def test_right_curve_has_negative_steering(self):
        result = self.controller.compute(
            [(0.10, -0.01), (0.40, -0.08), (0.70, -0.22), (1.00, -0.45)]
        )
        self.assertTrue(result.valid)
        self.assertLess(result.curvature, 0.0)
        self.assertLess(result.steering_angle, 0.0)

    def test_empty_path_returns_safe_result(self):
        result = self.controller.compute([])
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "empty_path")
        self.assertEqual(result.steering_angle, 0.0)
        self.assertEqual(result.target_speed, 0.0)

    def test_nan_returns_safe_result(self):
        result = self.controller.compute([(0.10, 0.0), (float("nan"), 0.10)])
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "non_finite_point")
        self.assertEqual(result.steering_angle, 0.0)
        self.assertEqual(result.target_speed, 0.0)

    def test_inf_returns_safe_result(self):
        result = self.controller.compute([(0.10, 0.0), (0.70, float("inf"))])
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "non_finite_point")
        self.assertEqual(result.steering_angle, 0.0)
        self.assertEqual(result.target_speed, 0.0)

    def test_nearest_and_lookahead_indices_are_reported(self):
        result = self.controller.compute(
            [(-0.10, 0.0), (0.05, 0.0), (0.30, 0.0), (0.80, 0.0)]
        )
        self.assertTrue(result.valid)
        self.assertEqual(result.nearest_index, 1)
        self.assertEqual(result.target_index, 3)
        self.assertEqual(result.target_point, (0.80, 0.0))

    def test_path_without_forward_point_returns_safe_result(self):
        result = self.controller.compute([(-0.50, 0.0), (0.0, 0.20)])
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "no_forward_point")
        self.assertEqual(result.steering_angle, 0.0)
        self.assertEqual(result.target_speed, 0.0)


if __name__ == "__main__":
    unittest.main()
