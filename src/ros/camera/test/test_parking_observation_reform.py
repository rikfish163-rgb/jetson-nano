# -*- coding: utf-8 -*-

from __future__ import division

import os
import sys
import unittest


SCRIPT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from parking_observation import ObservationBuilder


def front(slot="P5", trigger=True):
    return {
        "schema": "parking_front_metric_v1",
        "stamp": 100.0,
        "frame_id": "base_link",
        "valid": True,
        "confidence": 0.9,
        "slot_id": slot,
        "slot_visible": True,
        "parking_trigger_visible": trigger,
        "turn_distance_m": 0.70,
        "vehicle_yaw_error_rad": -0.2,
        "lateral_error_m": 0.05,
        "reverse_ready": False,
        "front_clear": None,
    }


def target(slot="P5", p5_obstacle=False, p4_obstacle=False):
    return {
        "schema": "parking_target_v1",
        "stamp": 100.0,
        "frame_id": "base_link",
        "valid": True,
        "confidence": 0.9,
        "candidate": True,
        "selection_ready": True,
        "start_line_visible": True,
        "start_line_distance_m": 0.70,
        "blue_trigger_visible": True,
        "slot_id": slot,
        "selected_slot_id": slot,
        "parking_side": "right",
        "turn_side": "right",
        "lidar_valid": True,
        "slot_obstacle_detected": (
            p5_obstacle if slot == "P5" else p4_obstacle),
        "slots": {
            "P5": {
                "visible": True, "lidar_valid": True,
                "obstacle_detected": p5_obstacle,
                "reason": "obstacle_detected" if p5_obstacle else "zone_clear",
            },
            "P4": {
                "visible": True, "lidar_valid": True,
                "obstacle_detected": p4_obstacle,
                "reason": "obstacle_detected" if p4_obstacle else "zone_clear",
            },
        },
        "distance_m": 0.65 if slot == "P5" else 0.95,
        "lateral_m": 0.10,
    }


LIDAR = {
    "schema": "parking_lidar_v1",
    "frame_id": "base_link",
    "valid": True,
    "obstacle_detected": False,
}


class ParkingObservationReformTest(unittest.TestCase):
    def make_builder(self):
        return ObservationBuilder({
            "default_slot_id": "AUTO",
            "input_timeout_s": 1.0,
            "target_timeout_s": 1.0,
            "sign_timeout_s": 1.0,
            "parking_request_hold_s": 120.0,
            "selected_slot_latch_s": 120.0,
            "min_source_confidence": 0.5,
        })

    def test_p_sign_arms_without_blue_trigger_or_selected_slot(self):
        builder = self.make_builder()
        builder.update("sign", "P", received_at=100.0)
        builder.update("front", front(trigger=True), received_at=100.0)
        builder.update("lidar", LIDAR, received_at=100.0)
        observation = builder.build(now=100.1)
        self.assertTrue(observation["parking_armed"])
        self.assertTrue(observation["parking_request"])
        self.assertFalse(observation["slot_selection_ready"])
        self.assertIsNone(observation["selected_slot_id"])
        self.assertFalse(observation["parking_trigger_visible"])

    def test_clear_target_is_latched_only_after_p_arm(self):
        builder = self.make_builder()
        builder.update("sign", "P", received_at=100.0)
        builder.update("target", target("P5"), received_at=100.0)
        builder.update("front", front("P5"), received_at=100.0)
        builder.update("lidar", LIDAR, received_at=100.0)
        observation = builder.build(now=100.1)
        self.assertEqual(observation["selected_slot_id"], "P5")
        self.assertTrue(observation["slot_selection_ready"])
        self.assertTrue(observation["parking_trigger_visible"])
        self.assertEqual(observation["parking_side"], "right")
        self.assertEqual(observation["parking_phase"], "blue_triggered")

    def test_blocked_preferred_p5_uses_clear_p4_before_latch(self):
        builder = self.make_builder()
        builder.update("sign", "P", received_at=100.0)
        builder.update("target", target("P4", p5_obstacle=True),
                       received_at=100.0)
        builder.update("front", front("P4"), received_at=100.0)
        builder.update("lidar", LIDAR, received_at=100.0)
        observation = builder.build(now=100.1)
        self.assertEqual(observation["selected_slot_id"], "P4")
        self.assertTrue(observation["slot_selection_ready"])
        self.assertFalse(observation["slot_obstacle_detected"])

    def test_latched_p5_does_not_switch_to_p4_when_p5_becomes_blocked(self):
        builder = self.make_builder()
        builder.update("sign", "P", received_at=100.0)
        builder.update("target", target("P5"), received_at=100.0)
        builder.update("front", front("P5"), received_at=100.0)
        builder.update("lidar", LIDAR, received_at=100.0)
        first = builder.build(now=100.1)
        self.assertEqual(first["selected_slot_id"], "P5")

        builder.update("target", target("P4", p5_obstacle=True),
                       received_at=100.2)
        builder.update("front", front("P5"), received_at=100.2)
        second = builder.build(now=100.3)
        self.assertEqual(second["selected_slot_id"], "P5")
        self.assertFalse(second["slot_selection_ready"])
        self.assertTrue(second["slot_obstacle_detected"])
        self.assertEqual(second["target_reason"], "latched_slot_not_current")


if __name__ == "__main__":
    unittest.main()
