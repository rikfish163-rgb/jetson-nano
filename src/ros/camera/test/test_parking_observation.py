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


class ParkingObservationBuilderTest(unittest.TestCase):
    def setUp(self):
        self.builder = ObservationBuilder({
            "default_slot_id": "P4",
            "input_timeout_s": 1.0,
            "sign_timeout_s": 1.0,
            "parking_request_hold_s": 2.0,
            "min_source_confidence": 0.5,
            "derive_clear_from_lidar": True,
        })

    @staticmethod
    def front(**overrides):
        data = {
            "schema": "parking_front_metric_v1",
            "stamp": 100.0,
            "frame_id": "base_link",
            "valid": True,
            "confidence": 0.9,
            "slot_id": "P4",
            "slot_visible": True,
            "parking_trigger_visible": True,
            "turn_distance_m": 0.7,
            "vehicle_yaw_error_rad": -0.2,
            "lateral_error_m": 0.05,
            "reverse_ready": True,
            "parking_request": False,
            "front_clear": None,
        }
        data.update(overrides)
        return data

    @staticmethod
    def rear(**overrides):
        data = {
            "schema": "parking_rear_metric_v1",
            "stamp": 100.0,
            "frame_id": "base_link",
            "valid": True,
            "confidence": 0.85,
            "slot_id": "P4",
            "rear_visible": True,
            "rear_distance_m": 0.6,
            "lateral_error_m": -0.02,
            "vehicle_yaw_error_rad": 0.03,
            "rear_clear": None,
        }
        data.update(overrides)
        return data

    @staticmethod
    def exit_metric(**overrides):
        data = {
            "schema": "parking_exit_metric_v1",
            "stamp": 100.0,
            "frame_id": "base_link",
            "valid": True,
            "confidence": 0.8,
            "slot_id": "P4",
            "exit_visible": True,
            "exit_distance_m": 0.5,
            "exit_yaw_error_rad": 0.04,
            "exit_lateral_error_m": 0.03,
            "exit_complete": False,
            "exit_clear": None,
        }
        data.update(overrides)
        return data

    @staticmethod
    def lidar():
        return {
            "schema": "parking_lidar_v1",
            "frame_id": "base_link",
            "valid": True,
            "obstacle_detected": False,
        }

    def test_no_metric_source_is_invalid_and_has_no_fake_errors(self):
        self.builder.update("lidar", self.lidar(), received_at=100.0)
        observation = self.builder.build(now=100.1)
        self.assertFalse(observation["valid"])
        self.assertIsNone(observation["turn_distance_m"])
        self.assertIsNone(observation["vehicle_yaw_error_rad"])
        self.assertFalse(observation["slot_visible"])

    def test_front_source_and_park_sign_are_joined(self):
        self.builder.update("front", self.front(), received_at=100.0)
        self.builder.update("sign", "park", received_at=100.0)
        self.builder.update("lidar", self.lidar(), received_at=100.0)
        observation = self.builder.build(now=100.1)
        self.assertTrue(observation["valid"])
        self.assertTrue(observation["parking_request"])
        self.assertEqual(observation["slot_id"], "P4")
        self.assertEqual(observation["turn_distance_m"], 0.7)
        self.assertTrue(observation["front_clear"])

    def test_front_slot_can_be_authoritative_request_when_enabled(self):
        builder = ObservationBuilder({
            "default_slot_id": "P4",
            "input_timeout_s": 1.0,
            "request_from_front_slot": True,
            "min_source_confidence": 0.5,
        })
        builder.update("front", self.front(), received_at=100.0)
        builder.update("lidar", self.lidar(), received_at=100.0)
        observation = builder.build(now=100.1)
        self.assertTrue(observation["parking_request"])

    def test_rear_pose_overrides_front_pose_during_reverse(self):
        self.builder.update("front", self.front(), received_at=100.0)
        self.builder.update("rear", self.rear(), received_at=100.0)
        self.builder.update("lidar", self.lidar(), received_at=100.0)
        observation = self.builder.build(now=100.1)
        self.assertEqual(observation["rear_distance_m"], 0.6)
        self.assertEqual(observation["lateral_error_m"], -0.02)
        self.assertEqual(observation["vehicle_yaw_error_rad"], 0.03)
        self.assertTrue(observation["rear_clear"])

    def test_front_pose_owns_common_pose_before_reverse_ready(self):
        self.builder.update("front", self.front(
            reverse_ready=False,
            vehicle_yaw_error_rad=-0.20,
            lateral_error_m=0.05), received_at=100.0)
        self.builder.update("rear", self.rear(
            vehicle_yaw_error_rad=0.40,
            lateral_error_m=-0.20), received_at=100.0)
        self.builder.update("lidar", self.lidar(), received_at=100.0)
        observation = self.builder.build(now=100.1)
        self.assertEqual(observation["pose_source"], "front")
        self.assertEqual(observation["lateral_error_m"], 0.05)
        self.assertEqual(observation["vehicle_yaw_error_rad"], -0.20)

    def test_generic_front_pose_cannot_trigger_parking_without_candidate(self):
        builder = ObservationBuilder({
            "default_slot_id": "P4",
            "input_timeout_s": 1.0,
            "request_from_front_slot": True,
            "min_source_confidence": 0.5,
        })
        generic_pose = self.front(
            parking_trigger_visible=False,
            turn_distance_m=None,
        )
        builder.update("front", generic_pose, received_at=100.0)
        observation = builder.build(now=100.1)
        self.assertTrue(observation["valid"])
        self.assertFalse(observation["parking_request"])

    def test_mixed_slot_sources_are_invalid(self):
        self.builder.update("front", self.front(slot_id="P4"),
                            received_at=100.0)
        self.builder.update("rear", self.rear(slot_id="P5"),
                            received_at=100.0)
        self.builder.update("lidar", self.lidar(), received_at=100.0)
        observation = self.builder.build(now=100.1)
        self.assertFalse(observation["valid"])
        self.assertFalse(observation["slot_consistent"])

    def test_explicit_source_clearance_false_is_not_overridden_by_lidar(self):
        self.builder.update("front", self.front(front_clear=False), received_at=100.0)
        self.builder.update("lidar", self.lidar(), received_at=100.0)
        observation = self.builder.build(now=100.1)
        self.assertFalse(observation["front_clear"])

    def test_wrong_frame_or_stale_source_is_invalid(self):
        self.builder.update("front", self.front(frame_id="map"), received_at=100.0)
        self.builder.update("lidar", self.lidar(), received_at=100.0)
        observation = self.builder.build(now=100.1)
        self.assertFalse(observation["valid"])

        self.builder.update("front", self.front(), received_at=100.0)
        stale = self.builder.build(now=101.1)
        self.assertFalse(stale["valid"])

    def test_exit_source_is_emitted_without_inventing_completion(self):
        self.builder.update("exit", self.exit_metric(), received_at=100.0)
        self.builder.update("lidar", self.lidar(), received_at=100.0)
        observation = self.builder.build(now=100.1)
        self.assertTrue(observation["valid"])
        self.assertEqual(observation["exit_distance_m"], 0.5)
        self.assertFalse(observation["exit_complete"])

    def test_unsupported_slot_cannot_trigger_parking_request(self):
        front = self.front(slot_id="P1", parking_request=True)
        self.builder.update("front", front, received_at=100.0)
        observation = self.builder.build(now=100.1)
        self.assertFalse(observation["parking_request"])
        self.assertFalse(observation["valid"])

    def test_unsupported_sign_slot_does_not_replace_supported_metric_slot(self):
        self.builder.update("front", self.front(), received_at=100.0)
        self.builder.update("sign", {
            "label": "park",
            "confidence": 0.9,
            "slot_id": "P1",
        }, received_at=100.0)
        observation = self.builder.build(now=100.1)
        self.assertEqual(observation["slot_id"], "P4")
        self.assertTrue(observation["valid"])


if __name__ == "__main__":
    unittest.main()
