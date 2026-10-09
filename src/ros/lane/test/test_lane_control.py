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

from vehicle_control.lane_control import LaneControlPolicy
from vehicle_control.pure_pursuit import PurePursuit


class LaneControlPolicyTest(unittest.TestCase):
    def make_policy(self, **overrides):
        settings = {
            "target_speed": 0.5,
            "min_confidence": 0.55,
            "path_timeout": 0.25,
            "max_steering_rad": 0.35,
            "angel_gain": 1.0,
        }
        settings.update(overrides)
        controller = PurePursuit(
            wheelbase=0.29,
            lookahead_distance=0.60,
            target_speed=settings["target_speed"],
        )
        return LaneControlPolicy(pure_pursuit=controller, **settings)

    @staticmethod
    def evaluate(policy, path, confidence=0.9, path_time=10.0, confidence_time=10.0, now=10.0):
        return policy.evaluate(
            path_points=path,
            path_update_time=path_time,
            confidence=confidence,
            confidence_update_time=confidence_time,
            now=now,
        )

    def test_straight_path_has_zero_angel(self):
        command = self.evaluate(
            self.make_policy(),
            [(0.10, 0.0), (0.40, 0.0), (0.70, 0.0), (1.0, 0.0)],
        )
        self.assertFalse(command.safe_stop)
        self.assertAlmostEqual(command.angel, 0.0, places=12)

    def test_left_path_has_positive_angel(self):
        command = self.evaluate(
            self.make_policy(),
            [(0.10, 0.01), (0.40, 0.08), (0.70, 0.22), (1.0, 0.45)],
        )
        self.assertFalse(command.safe_stop)
        self.assertGreater(command.angel, 0.0)

    def test_right_path_has_negative_angel(self):
        command = self.evaluate(
            self.make_policy(),
            [(0.10, -0.01), (0.40, -0.08), (0.70, -0.22), (1.0, -0.45)],
        )
        self.assertFalse(command.safe_stop)
        self.assertLess(command.angel, 0.0)

    def test_low_confidence_stops(self):
        command = self.evaluate(
            self.make_policy(),
            [(0.10, 0.0), (0.70, 0.0)],
            confidence=0.40,
        )
        self.assertTrue(command.safe_stop)
        self.assertEqual(command.reason, "confidence_low")
        self.assertEqual((command.speed, command.angel), (0.0, 0.0))

    def test_empty_path_stops(self):
        command = self.evaluate(self.make_policy(), [])
        self.assertTrue(command.safe_stop)
        self.assertEqual(command.reason, "path_empty")
        self.assertEqual((command.speed, command.angel), (0.0, 0.0))

    def test_stale_path_stops(self):
        command = self.evaluate(
            self.make_policy(),
            [(0.10, 0.0), (0.70, 0.0)],
            path_time=9.0,
            now=10.0,
        )
        self.assertTrue(command.safe_stop)
        self.assertEqual(command.reason, "path_stale")
        self.assertEqual((command.speed, command.angel), (0.0, 0.0))

    def test_stale_confidence_stops(self):
        command = self.evaluate(
            self.make_policy(),
            [(0.10, 0.0), (0.70, 0.0)],
            confidence_time=9.0,
            now=10.0,
        )
        self.assertTrue(command.safe_stop)
        self.assertEqual(command.reason, "confidence_stale")
        self.assertEqual((command.speed, command.angel), (0.0, 0.0))

    def test_angel_gain_scales_steering(self):
        policy = self.make_policy(angel_gain=2.5, max_steering_rad=1.0)
        command = self.evaluate(
            policy,
            [(0.10, 0.01), (0.40, 0.08), (0.70, 0.22), (1.0, 0.45)],
        )
        self.assertFalse(command.safe_stop)
        self.assertAlmostEqual(
            command.angel,
            command.raw_steering_rad * 2.5,
            places=12,
        )

    def test_max_steering_rad_clamps_output(self):
        policy = self.make_policy(max_steering_rad=0.05, angel_gain=2.0)
        command = self.evaluate(
            policy,
            [(0.10, 0.10), (0.40, 0.40), (0.70, 0.70)],
        )
        self.assertFalse(command.safe_stop)
        self.assertGreater(command.raw_steering_rad, 0.05)
        self.assertAlmostEqual(command.steering_rad_clamped, 0.05, places=12)
        self.assertAlmostEqual(command.angel, 0.10, places=12)

    def test_non_finite_path_stops(self):
        command = self.evaluate(
            self.make_policy(),
            [(0.10, 0.0), (float("nan"), 0.20)],
        )
        self.assertTrue(command.safe_stop)
        self.assertEqual(command.reason, "pure_pursuit_non_finite_point")
        self.assertEqual((command.speed, command.angel), (0.0, 0.0))

    def test_non_finite_confidence_stops(self):
        command = self.evaluate(
            self.make_policy(),
            [(0.10, 0.0), (0.70, 0.0)],
            confidence=float("inf"),
        )
        self.assertTrue(command.safe_stop)
        self.assertEqual(command.reason, "confidence_non_finite")
        self.assertEqual((command.speed, command.angel), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
