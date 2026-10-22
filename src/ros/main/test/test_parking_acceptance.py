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

from parking_acceptance import evaluate
from parking_controller import ParkingConfig, STATE_REQUIRED_SOURCES


class ParkingAcceptanceTest(unittest.TestCase):
    @staticmethod
    def calibrated_profile(**overrides):
        profile = dict(
            (field, ParkingConfig.DEFAULTS[field])
            for field in ParkingConfig.CALIBRATED_SLOT_FIELDS
        )
        profile.update(overrides)
        return profile

    @staticmethod
    def record(stamp, event, payload):
        return {
            "wall_time": float(stamp),
            "run_id": "acceptance-run",
            "event": event,
            "data": {"payload": payload},
        }

    @staticmethod
    def direct_record(stamp, event, data):
        return {
            "wall_time": float(stamp),
            "run_id": "acceptance-run",
            "event": event,
            "data": data,
        }

    @classmethod
    def status(cls, stamp, state, speed_raw, steering_raw):
        required_source = STATE_REQUIRED_SOURCES.get(state)
        payload = {
            "schema": "parking_status_v1",
            "state": state,
            "phase_index": 0,
            "state_age_s": 0.0,
            "state_reason": "test",
            "command_reason": "test",
            "transition_seq": 1,
            "slot_id": "P4",
            "holding": state != "done",
            "active": state not in ("done", "idle"),
            "done": state == "done",
            "fault": False,
            "calibration_complete": True,
            "required_visual_source": required_source,
            "observation_valid": True,
            "observation_source_confidence": {
                "front": 0.95,
                "rear": 0.95,
                "exit": 0.95,
            },
            "observation_slot": "P4",
            "observation_slot_consistent": True,
            "speed_raw": speed_raw,
            "steering_raw": steering_raw,
            "safe_stop": speed_raw == 0 and steering_raw == 0,
        }
        return cls.record(stamp, "status", payload)

    @classmethod
    def actuator_command(cls, stamp, speed_raw, steering_raw):
        # The typed Ackermann callback writes this event directly under data,
        # unlike String-topic callbacks which use data.payload.
        return cls.direct_record(stamp, "actuator_command", {
            "schema": "ackermann_command_observation_v1",
            "speed_raw": float(speed_raw),
            "steering_raw": float(steering_raw),
            "valid": True,
        })

    @classmethod
    def base_controller_status(cls, stamp, speed_raw, steering_raw):
        return cls.record(stamp, "base_controller_status", {
            "schema": "base_controller_status_v1",
            "speed_raw": float(speed_raw),
            "steering_raw": float(steering_raw),
            "serial_bytes": 11,
            "serial_open": True,
        })

    def successful_records(self):
        states = [
            ("approach", 12, 0),
            ("forward_right_turn", 12, -10),
            ("forward_straighten", 12, 8),
            ("settle", 0, 0),
            ("reverse_steer_in", -10, -10),
            ("reverse_straighten", -10, 8),
            ("reverse_align", -10, 2),
            ("parked", 0, 0),
            ("exit_turn", 10, 10),
            ("exit_straighten", 10, -6),
            ("done", 0, 0),
        ]
        records = [self.direct_record(0.0, "logger_start", {
            "config_file": "parking.yaml",
            "config_sha256": "test-hash",
        })]
        for index, (state, speed, steering) in enumerate(states, 1):
            records.append(self.status(index, state, speed, steering))
            records.append(self.record(index + 0.001, "control", {
                "version": 1,
                "seq": index,
                "speed_raw": speed,
                "steering_raw": steering,
            }))
            records.append(self.record(index + 0.0015, "lidar", {
                "schema": "parking_lidar_v1",
                "frame_id": "base_link",
                "valid": True,
                "obstacle_detected": False,
                "emergency_stop": False,
            }))
            records.append(self.actuator_command(
                index + 0.002, speed, steering))
            records.append(self.base_controller_status(
                index + 0.003, speed, steering))
        return records

    @classmethod
    def odometry(cls, stamp, x, y, linear_x):
        return cls.direct_record(stamp, "odometry", {
            "position_x_m": float(x),
            "position_y_m": float(y),
            "position_z_m": 0.0,
            "orientation_x": 0.0,
            "orientation_y": 0.0,
            "orientation_z": 0.0,
            "orientation_w": 1.0,
            "linear_x_mps": float(linear_x),
            "linear_y_mps": 0.0,
            "angular_z_radps": 0.0,
            "frame_id": "odom",
            "child_frame_id": "base_link",
            "valid": True,
        })

    def config(self, calibration_complete=True):
        return ParkingConfig.from_dict({
            "calibration_complete": calibration_complete,
            "condition_debounce_frames": 1,
            "slot_profiles": {
                "P4": self.calibrated_profile(),
                "P5": self.calibrated_profile(),
            },
        })

    def test_successful_run_passes_all_evidence_checks(self):
        report = evaluate(self.successful_records(), self.config())
        self.assertTrue(report["passed"])
        self.assertTrue(report["checks"]["state_sequence"]["passed"])
        self.assertTrue(report["checks"]["visual_contract"]["passed"])
        self.assertEqual(
            report["checks"]["commands"]["missing_motion_states"], [])
        self.assertTrue(report["checks"]["actuator_command_path"]["passed"])
        self.assertTrue(
            report["checks"]["base_controller_write_path"]["passed"])
        self.assertTrue(report["checks"]["lidar_contract"]["passed"])

    def test_missing_lidar_contract_cannot_be_accepted(self):
        records = [record for record in self.successful_records()
                   if record["event"] != "lidar"]
        report = evaluate(records, self.config())
        self.assertFalse(report["passed"])
        self.assertEqual(
            report["checks"]["lidar_contract"]["reason"],
            "lidar_records_missing",
        )

    def test_physical_feedback_is_explicitly_separate(self):
        report = evaluate(
            self.successful_records(),
            self.config(),
            require_physical_feedback=True,
        )
        self.assertTrue(report["software_passed"])
        self.assertFalse(report["passed"])
        self.assertEqual(
            report["physical_feedback"]["reason"], "odometry_missing")
        self.assertFalse(report["checks"]["physical_feedback"]["passed"])

    def test_physical_feedback_mode_accepts_both_motion_directions(self):
        records = self.successful_records()
        records.extend([
            self.odometry(2.5, 0.00, 0.00, 0.15),
            self.odometry(5.5, 0.10, 0.00, -0.15),
            self.odometry(9.5, 0.20, 0.00, 0.15),
        ])
        report = evaluate(
            records,
            self.config(),
            require_physical_feedback=True,
        )
        self.assertTrue(report["passed"])
        self.assertEqual(
            report["physical_feedback"]["forward_motion_samples"], 2)
        self.assertEqual(
            report["physical_feedback"]["reverse_motion_samples"], 1)

    def test_locked_configuration_cannot_be_accepted_with_motion(self):
        report = evaluate(
            self.successful_records(),
            self.config(calibration_complete=False),
        )
        self.assertFalse(report["passed"])
        self.assertEqual(
            report["checks"]["commands"]["reason"],
            "config_calibration_incomplete",
        )

    def test_settle_and_parked_states_must_hold_zero_command(self):
        records = self.successful_records()
        for record in records:
            if (record["event"] == "status" and
                    record["data"].get("payload", {}).get("state") == "parked"):
                record["data"]["payload"]["speed_raw"] = 1
                break
        report = evaluate(records, self.config())
        self.assertFalse(report["passed"])
        self.assertEqual(
            report["checks"]["commands"]["reason"],
            "nonzero_command_in_zero_state",
        )

    def test_all_recorded_config_hashes_are_checked(self):
        records = self.successful_records()
        records[0]["data"]["config_hashes"] = {
            "/definitely/missing/parking.yaml": "deadbeef",
        }
        report = evaluate(records, self.config())
        self.assertFalse(report["checks"]["config_provenance"]["passed"])
        self.assertEqual(
            report["checks"]["config_provenance"]["reason"],
            "config_hashes_mismatch",
        )


if __name__ == "__main__":
    unittest.main()
