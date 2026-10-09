# -*- coding: utf-8 -*-

"""Pure synthetic LaserScan tests for P4/P5 target selection."""

from __future__ import division

import math
import os
import sys
import unittest


SCRIPT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import parking_target_selector as selector


class ScanStub(object):
    def __init__(self, points=None, count=720):
        self.angle_min = -math.pi
        self.angle_increment = 2.0 * math.pi / count
        self.range_min = 0.05
        self.ranges = [2.0] * count
        for x, y, distance in points or []:
            angle = math.atan2(y, x)
            index = int(round((angle - self.angle_min) /
                              self.angle_increment))
            if 0 <= index < count:
                self.ranges[index] = float(distance)


SETTINGS = {
    "slot_width_m": 0.38,
    "zone_start_offset_m": 0.08,
    "zone_end_offset_m": 0.53,
    "safety_margin_m": 0.03,
    "obstacle_max_distance_m": 2.0,
    "min_scan_samples": 8,
    "lidar_to_base_x_m": 0.0,
    "lidar_to_base_y_m": 0.0,
    "lidar_yaw_rad": 0.0,
    "slot_distance_bands": {
        "P5": [0.55, 0.82],
        "P4": [0.78, 1.05],
    },
    "zone_calibrated": True,
}


def candidates():
    return {
        "schema": "parking_slot_candidate_v1",
        "candidate": True,
        "slot_candidate": True,
        "candidates": [
            {"id": 1, "distance_m": 0.65, "lateral_m": 0.15,
             "angle_deg": 0.0, "length_m": 0.2,
             "thickness_m": 0.02, "area_px": 1000},
            {"id": 2, "distance_m": 0.95, "lateral_m": 0.15,
             "angle_deg": 0.0, "length_m": 0.2,
             "thickness_m": 0.02, "area_px": 900},
        ],
    }


class ParkingTargetSelectorTest(unittest.TestCase):
    @staticmethod
    def start_line(visible=True):
        return {
            "schema": "parking_start_line_v1",
            "candidate": bool(visible),
            "start_line_visible": bool(visible),
            "blue_trigger_visible": bool(visible),
            "distance_m": 0.42 if visible else None,
            "start_line_distance_m": 0.42 if visible else None,
            "lateral_m": 0.01 if visible else None,
            "angle_deg": 0.0 if visible else None,
        }

    def test_blue_start_line_is_not_a_slot_candidate(self):
        self.assertEqual(selector.extract_candidates(self.start_line()), [])

    def test_slot_selection_does_not_require_blue_start_line(self):
        result = selector.choose_target(
            candidates(), ScanStub(), requested_slot_id="P5",
            settings=SETTINGS)
        self.assertTrue(result["selection_ready"])
        self.assertEqual(result["slot_id"], "P5")
        self.assertFalse(result["blue_trigger_visible"])
        self.assertFalse(result["start_line_visible"])
        self.assertIsNone(result["turn_distance_m"])

    def test_blue_start_line_releases_only_the_already_selected_target(self):
        result = selector.choose_target(
            candidates(), ScanStub(), requested_slot_id="P5",
            settings=SETTINGS, start_line_payload=self.start_line())
        self.assertTrue(result["selection_ready"])
        self.assertTrue(result["start_line_visible"])
        self.assertTrue(result["blue_trigger_visible"])
        self.assertAlmostEqual(result["start_line_distance_m"], 0.42)
        self.assertAlmostEqual(result["turn_distance_m"], 0.42)

    def test_blue_start_line_cannot_release_when_target_is_not_ready(self):
        result = selector.choose_target(
            candidates(), ScanStub(), requested_slot_id="P5",
            settings=dict(SETTINGS, zone_calibrated=False),
            start_line_payload=self.start_line())
        self.assertFalse(result["selection_ready"])
        self.assertTrue(result["start_line_visible"])
        self.assertFalse(result["blue_trigger_visible"])

    def test_selected_slot_associates_the_near_or_far_blue_line(self):
        start_lines = {
            "schema": "parking_start_line_v1",
            "candidate": True,
            "start_line_visible": True,
            "candidates": [
                {"id": 1, "distance_m": 0.40, "lateral_m": 0.12,
                 "angle_deg": 0.0},
                {"id": 2, "distance_m": 0.80, "lateral_m": 0.12,
                 "angle_deg": 0.0},
            ],
        }
        p5 = selector.choose_target(
            candidates(), ScanStub(), requested_slot_id="P5",
            settings=SETTINGS, start_line_payload=start_lines)
        p4 = selector.choose_target(
            candidates(), ScanStub(), requested_slot_id="P4",
            settings=SETTINGS, start_line_payload=start_lines)
        self.assertAlmostEqual(p5["start_line_distance_m"], 0.40)
        self.assertAlmostEqual(p4["start_line_distance_m"], 0.80)

    def test_nearest_and_farthest_candidates_map_to_p5_and_p4(self):
        assigned = selector.assign_slot_hints(
            selector.extract_candidates(candidates()),
            slot_distance_bands=SETTINGS["slot_distance_bands"],
        )
        self.assertEqual([item["slot_id"] for item in assigned], ["P5", "P4"])

    def test_occupied_requested_p5_falls_back_to_clear_p4(self):
        scan = ScanStub(points=[(0.85, 0.15, 0.86)])
        result = selector.choose_target(
            candidates(), scan, requested_slot_id="P5", settings=SETTINGS)
        self.assertTrue(result["selection_ready"])
        self.assertEqual(result["slot_id"], "P4")
        self.assertEqual(result["selection_reason"],
                         "preferred_occupied_fallback")
        self.assertTrue(result["slots"]["P5"]["obstacle_detected"])
        self.assertFalse(result["slots"]["P4"]["obstacle_detected"])

    def test_both_bays_occupied_are_not_ready(self):
        scan = ScanStub(points=[
            (0.85, 0.15, 0.86),
            (1.15, 0.15, 1.15),
        ])
        result = selector.choose_target(
            candidates(), scan, requested_slot_id="AUTO", settings=SETTINGS)
        self.assertFalse(result["selection_ready"])
        self.assertIsNone(result["slot_id"])

    def test_missing_or_unhealthy_scan_never_unlocks_selection(self):
        result = selector.choose_target(
            candidates(), None, requested_slot_id="P5", settings=SETTINGS)
        self.assertFalse(result["valid"])
        self.assertFalse(result["selection_ready"])
        self.assertFalse(result["lidar_valid"])

    def test_target_side_is_inferred_from_signed_base_link_lateral_position(self):
        result = selector.choose_target(
            {
                "candidate": True,
                "candidates": [{
                    "id": 1, "distance_m": 0.65, "lateral_m": -0.18,
                    "angle_deg": 0.0, "length_m": 0.2,
                }],
            },
            ScanStub(),
            requested_slot_id="AUTO",
            settings=SETTINGS,
        )
        self.assertTrue(result["selection_ready"])
        self.assertEqual(result["parking_side"], "right")
        self.assertEqual(result["turn_sign"], -1)

    def test_positive_base_link_lateral_position_selects_left_turn(self):
        payload = candidates()
        result = selector.choose_target(
            payload,
            ScanStub(),
            requested_slot_id="AUTO",
            settings=SETTINGS,
        )
        self.assertTrue(result["selection_ready"])
        self.assertEqual(result["parking_side"], "left")
        self.assertEqual(result["turn_sign"], 1)

    def test_ambiguous_target_side_does_not_unlock_motion(self):
        result = selector.choose_target(
            {
                "candidate": True,
                "candidates": [{
                    "id": 1, "distance_m": 0.65, "lateral_m": 0.0,
                    "angle_deg": 0.0, "length_m": 0.2,
                }],
            },
            ScanStub(),
            requested_slot_id="AUTO",
            settings=SETTINGS,
        )
        self.assertFalse(result["selection_ready"])
        self.assertEqual(result["reason"], "parking_side_ambiguous")

    def test_lidar_yaw_and_translation_are_used_for_zone_evaluation(self):
        settings = dict(SETTINGS)
        settings.update({
            "lidar_to_base_x_m": 0.10,
            "lidar_to_base_y_m": -0.05,
            "lidar_yaw_rad": math.pi / 2.0,
        })
        # A base-link point at (0.90, 0.15) appears in laser coordinates after
        # the inverse of the configured translation/rotation.
        x_base, y_base = 0.90, 0.15
        x_lidar = math.cos(-math.pi / 2.0) * (x_base - 0.10) - math.sin(-math.pi / 2.0) * (y_base + 0.05)
        y_lidar = math.sin(-math.pi / 2.0) * (x_base - 0.10) + math.cos(-math.pi / 2.0) * (y_base + 0.05)
        distance = math.hypot(x_lidar, y_lidar)
        result = selector.evaluate_slot_zone(
            ScanStub(points=[(x_lidar, y_lidar, distance)]),
            {"distance_m": 0.65, "lateral_m": 0.15},
            settings,
        )
        self.assertTrue(result["valid"])
        self.assertTrue(result["obstacle_detected"])


if __name__ == "__main__":
    unittest.main()
