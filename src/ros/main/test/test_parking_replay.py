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

from parking_controller import ParkingConfig
from parking_replay import replay


class ParkingReplayTest(unittest.TestCase):
    @staticmethod
    def calibrated_profile(**overrides):
        profile = dict(
            (field, ParkingConfig.DEFAULTS[field])
            for field in ParkingConfig.CALIBRATED_SLOT_FIELDS
        )
        profile.update(overrides)
        return profile

    @staticmethod
    def observation(stamp, **overrides):
        value = {
            "schema": "parking_observation_v1",
            "stamp": stamp,
            "frame_id": "base_link",
            "valid": True,
            "confidence": 0.95,
            "source_confidence": {
                "front": 0.95,
                "rear": 0.95,
                "exit": 0.95,
            },
            "source_status": {
                "front": {"present": True, "fresh": True, "valid": True},
                "rear": {"present": True, "fresh": True, "valid": True},
                "exit": {"present": True, "fresh": True, "valid": True},
            },
            "slot_id": "P4",
            "slot_consistent": True,
            "slot_visible": True,
            "rear_visible": True,
            "exit_visible": True,
            "front_clear": True,
            "rear_clear": True,
            "exit_clear": True,
            "turn_distance_m": 1.0,
            "vehicle_yaw_error_rad": 0.0,
            "lateral_error_m": 0.0,
            "rear_distance_m": 1.0,
            "reverse_ready": False,
            "exit_distance_m": 1.0,
            "exit_yaw_error_rad": 0.0,
            "exit_lateral_error_m": 0.0,
            "exit_complete": False,
            "parking_request": False,
        }
        value.update(overrides)
        return value

    def test_replay_follows_complete_parking_and_exit_sequence(self):
        samples = [
            (100.0, {"parking_request": True}),
            (100.1, {"turn_distance_m": 0.60}),
            (100.2, {"vehicle_yaw_error_rad": -0.60}),
            (100.3, {"vehicle_yaw_error_rad": 0.0, "reverse_ready": True}),
            (100.4, {}),
            (100.5, {"rear_distance_m": 0.50}),
            (100.6, {"rear_distance_m": 0.40, "vehicle_yaw_error_rad": 0.0}),
            (100.7, {"rear_distance_m": 0.10}),
            (100.8, {"exit_distance_m": 0.60, "exit_yaw_error_rad": 0.20}),
            (100.9, {"exit_distance_m": 0.40, "exit_yaw_error_rad": 0.05}),
            (101.0, {"exit_distance_m": 0.10}),
        ]
        records = []
        for stamp, overrides in samples:
            records.append({
                "wall_time": stamp - 0.001,
                "event": "lidar",
                "data": {"payload": {"schema": "parking_lidar_v1",
                                      "frame_id": "base_link",
                                      "valid": True,
                                      "obstacle_detected": False}},
            })
            records.append({
                "wall_time": stamp,
                "event": "observation",
                "data": {"payload": self.observation(stamp, **overrides)},
            })

        config = ParkingConfig.from_dict({
            "calibration_complete": True,
            "require_lidar_clear": True,
            "condition_debounce_frames": 1,
            "settle_time_s": 0.0,
            "parked_hold_s": 0.0,
            "slot_profiles": {
                "P4": self.calibrated_profile(),
                "P5": self.calibrated_profile(),
            },
        })
        report = replay(records, config)

        self.assertTrue(report["started"])
        self.assertTrue(report["done"])
        self.assertEqual(report["final_state"], "done")
        self.assertIsNone(report["first_fault"])
        self.assertEqual(
            [item["state"] for item in report["transitions"]],
            [
                "approach",
                "forward_right_turn",
                "forward_straighten",
                "settle",
                "reverse_steer_in",
                "reverse_straighten",
                "reverse_align",
                "parked",
                "exit_turn",
                "exit_straighten",
                "done",
            ],
        )

    def test_replay_follows_complete_p5_profile_sequence(self):
        samples = [
            (200.0, {"parking_request": True}),
            (200.1, {"turn_distance_m": 0.50}),
            (200.2, {"vehicle_yaw_error_rad": -0.60}),
            (200.3, {"vehicle_yaw_error_rad": 0.0, "reverse_ready": True}),
            (200.4, {}),
            (200.5, {"rear_distance_m": 0.50}),
            (200.6, {"rear_distance_m": 0.40,
                     "vehicle_yaw_error_rad": 0.0}),
            (200.7, {"rear_distance_m": 0.10}),
            (200.8, {"exit_distance_m": 0.60,
                     "exit_yaw_error_rad": 0.20}),
            (200.9, {"exit_distance_m": 0.40,
                     "exit_yaw_error_rad": 0.05}),
            (201.0, {"exit_distance_m": 0.10}),
        ]
        records = []
        for stamp, overrides in samples:
            records.append({
                "wall_time": stamp - 0.001,
                "event": "lidar",
                "data": {"payload": {"schema": "parking_lidar_v1",
                                      "frame_id": "base_link",
                                      "valid": True,
                                      "obstacle_detected": False}},
            })
            overrides = dict(overrides)
            overrides["slot_id"] = "P5"
            records.append({
                "wall_time": stamp,
                "event": "observation",
                "data": {"payload": self.observation(stamp, **overrides)},
            })

        config = ParkingConfig.from_dict({
            "calibration_complete": True,
            "require_lidar_clear": True,
            "condition_debounce_frames": 1,
            "settle_time_s": 0.0,
            "parked_hold_s": 0.0,
            "slot_profiles": {
                "P4": self.calibrated_profile(),
                "P5": self.calibrated_profile(turn_start_distance_m=0.60),
            },
        })
        report = replay(records, config)

        self.assertTrue(report["started"])
        self.assertTrue(report["done"])
        self.assertEqual(report["slot_id"], "P5")
        self.assertIsNone(report["first_fault"])


if __name__ == "__main__":
    unittest.main()
