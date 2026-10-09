# -*- coding: utf-8 -*-

from __future__ import division

import math
import os
import sys
import unittest


SCRIPT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from return_to_start import (  # noqa: E402
    angular_distance_rad,
    keyboard_payload,
    sector_min,
)


class ReturnToStartTest(unittest.TestCase):
    def test_angular_distance_wraps_at_pi(self):
        self.assertAlmostEqual(
            angular_distance_rad(math.radians(359.0), 0.0),
            math.radians(1.0),
        )

    def test_sector_min_supports_zero_to_two_pi_scan(self):
        # 0, 90, 180, 270 degrees.  Front is close; rear is open.
        ranges = [0.30, 1.0, 1.20, 0.65]
        front, front_count = sector_min(
            ranges, 0.0, math.pi / 2.0, 0.05, 5.0, 0.0, 45.0)
        rear, rear_count = sector_min(
            ranges, 0.0, math.pi / 2.0, 0.05, 5.0, 180.0, 45.0)
        self.assertAlmostEqual(front, 0.30)
        self.assertEqual(front_count, 1)
        self.assertAlmostEqual(rear, 1.20)
        self.assertEqual(rear_count, 1)

    def test_invalid_ranges_are_not_reported_as_clear(self):
        minimum, count = sector_min(
            [float("nan"), float("inf")],
            0.0, math.pi / 2.0, 0.05, 5.0, 0.0, 45.0)
        self.assertIsNone(minimum)
        self.assertEqual(count, 0)

    def test_keyboard_release_forces_zero_fields(self):
        payload = keyboard_payload(260, False, speed_raw=-5, steering_raw=4)
        self.assertEqual(payload, {
            "version": 1,
            "seq": 4,
            "enabled": False,
            "speed_raw": 0,
            "steering_raw": 0,
        })


if __name__ == "__main__":
    unittest.main()
