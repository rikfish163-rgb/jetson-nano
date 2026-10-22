# -*- coding: utf-8 -*-

from __future__ import division

import os
import sys
import unittest


SCRIPTS_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from parking_controller import ParkingConfig, ParkingController


class ReformControllerTest(unittest.TestCase):
    def make_controller(self):
        profile = dict(
            (field, ParkingConfig.DEFAULTS[field])
            for field in ParkingConfig.CALIBRATED_SLOT_FIELDS)
        settings = {
            "calibration_complete": True,
            "condition_debounce_frames": 1,
            "observation_timeout_s": 10.0,
            "min_confidence": 0.5,
            "slot_profiles": {"P4": dict(profile), "P5": dict(profile)},
        }
        return ParkingController(ParkingConfig.from_dict(settings))

    @staticmethod
    def observation(**overrides):
        data = {
            "schema": "parking_observation_v1",
            "stamp": 100.0,
            "frame_id": "base_link",
            "valid": True,
            "confidence": 0.9,
            "source_confidence": {
                "front": 0.9, "rear": 0.9, "exit": 0.9, "target": 0.9,
            },
            "source_status": {
                "front": {"present": True, "fresh": True, "valid": True},
                "rear": {"present": True, "fresh": True, "valid": True},
                "exit": {"present": True, "fresh": True, "valid": True},
                "target": {"present": True, "fresh": True, "valid": True},
            },
            "parking_armed": True,
            "parking_request": True,
            "selected_slot_id": "P5",
            "slot_selection_ready": True,
            "slot_obstacle_detected": False,
            "parking_side": "right",
            "turn_side": "right",
            "slot_id": "P5",
            "slot_consistent": True,
            "slot_visible": True,
            "parking_trigger_visible": False,
            "blue_trigger_visible": False,
            "front_clear": True,
            "rear_clear": True,
            "exit_clear": True,
            "turn_distance_m": 0.9,
            "vehicle_yaw_error_rad": 0.0,
            "lateral_error_m": 0.0,
            "rear_visible": True,
            "rear_distance_m": 0.8,
            "reverse_ready": False,
            "exit_visible": True,
            "exit_distance_m": 1.0,
            "exit_yaw_error_rad": 0.0,
            "exit_lateral_error_m": 0.0,
            "exit_complete": False,
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

    def test_armed_only_waits_for_blue_without_motion(self):
        controller = self.make_controller()
        self.assertTrue(controller.start("P5", now=100.0, armed_only=True))
        self.assertEqual(controller.state, "wait_blue")
        command = controller.step(self.observation(), self.lidar(), now=100.0)
        self.assertEqual(command["state"], "wait_blue")
        self.assertEqual(command["reason"], "waiting_for_blue_trigger")
        self.assertEqual((command["speed_raw"], command["steering_raw"]), (0, 0))

    def test_blue_trigger_releases_approach_then_turn_gate(self):
        controller = self.make_controller()
        controller.start("P5", now=100.0, armed_only=True)
        command = controller.step(
            self.observation(parking_trigger_visible=True),
            self.lidar(), now=100.0)
        self.assertEqual(command["state"], "approach")
        self.assertEqual(command["speed_raw"], 12)
        command = controller.step(
            self.observation(parking_trigger_visible=True, turn_distance_m=0.6),
            self.lidar(), now=100.1)
        self.assertEqual(command["state"], "forward_right_turn")
        self.assertEqual(command["steering_raw"], -22)

    def test_latched_slot_mismatch_or_occupancy_faults_closed_loop(self):
        controller = self.make_controller()
        controller.start("P5", now=100.0, armed_only=True)
        command = controller.step(
            self.observation(selected_slot_id="P4", slot_id="P4"),
            self.lidar(), now=100.0)
        self.assertEqual(command["state"], "fault")
        self.assertEqual(command["reason"], "selected_slot_mismatch")

        controller = self.make_controller()
        controller.start("P5", now=100.0, armed_only=True)
        command = controller.step(
            self.observation(slot_obstacle_detected=True),
            self.lidar(), now=100.0)
        self.assertEqual(command["state"], "fault")
        self.assertEqual(command["reason"], "selected_slot_obstructed")


if __name__ == "__main__":
    unittest.main()
