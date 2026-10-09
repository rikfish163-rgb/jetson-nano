# -*- coding: utf-8 -*-

from __future__ import division

import json
import math
import os
import sys
import unittest


SCRIPT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from parking_lidar_adapter import (  # noqa: E402
    LidarAdapterConfig,
    normalize_lidar_payload,
)


class ParkingLidarAdapterTest(unittest.TestCase):
    def test_legacy_nearest_point_becomes_strict_clear_payload(self):
        result = normalize_lidar_payload(
            {"angle": 12.0, "distance": 1.2}, now=100.0)
        self.assertEqual(result["schema"], "parking_lidar_v1")
        self.assertEqual(result["frame_id"], "base_link")
        self.assertTrue(result["valid"])
        self.assertFalse(result["obstacle_detected"])
        self.assertAlmostEqual(result["nearest_angle_rad"],
                               math.radians(12.0))
        self.assertEqual(result["source_schema"],
                         "legacy_lidar_nearest_v1")

    def test_legacy_nearest_point_crossing_threshold_sets_obstacle(self):
        result = normalize_lidar_payload(
            json.dumps({"angle": -5.0, "distance": 0.30}), now=100.0)
        self.assertTrue(result["valid"])
        self.assertTrue(result["obstacle_detected"])
        self.assertEqual(result["reason"], "obstacle_detected")

    def test_out_of_range_legacy_sentinel_is_invalid(self):
        result = normalize_lidar_payload(
            {"angle": 0.0, "distance": 999.0}, now=100.0)
        self.assertFalse(result["valid"])
        self.assertIsInstance(result["obstacle_detected"], bool)
        self.assertEqual(result["reason"], "distance_invalid")

    def test_missing_payload_is_invalid_and_never_clear(self):
        result = normalize_lidar_payload(None, now=100.0)
        self.assertFalse(result["valid"])
        self.assertFalse(result["obstacle_detected"])
        self.assertEqual(result["reason"], "payload_invalid")

    def test_angle_window_is_applied_but_unknown_bearing_is_conservative(self):
        config = LidarAdapterConfig({
            "obstacle_angle_min_deg": -30.0,
            "obstacle_angle_max_deg": 30.0,
        })
        outside = normalize_lidar_payload(
            {"angle": 90.0, "distance": 0.20}, now=100.0, config=config)
        unknown = normalize_lidar_payload(
            {"distance": 0.20}, now=100.0, config=config)
        self.assertTrue(outside["valid"])
        self.assertFalse(outside["obstacle_detected"])
        self.assertTrue(unknown["obstacle_detected"])

    def test_explicit_source_obstacle_cannot_be_overridden_by_distance(self):
        result = normalize_lidar_payload({
            "schema": "external_lidar_v1",
            "valid": True,
            "obstacle_detected": True,
            "distance_m": 1.2,
        }, now=100.0)
        self.assertTrue(result["valid"])
        self.assertTrue(result["obstacle_detected"])
        self.assertEqual(result["source_schema"], "external_lidar_v1")

    def test_malformed_boolean_field_is_invalid(self):
        result = normalize_lidar_payload({
            "angle": 0.0,
            "distance": 1.0,
            "valid": "true",
        }, now=100.0)
        self.assertFalse(result["valid"])
        self.assertEqual(result["reason"], "payload_field_type_invalid")

    def test_invalid_adapter_limits_are_rejected(self):
        with self.assertRaises(ValueError):
            LidarAdapterConfig({
                "obstacle_distance_m": 3.0,
                "max_valid_distance_m": 2.0,
            })


if __name__ == "__main__":
    unittest.main()
