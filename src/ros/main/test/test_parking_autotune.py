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

from parking_autotune import build_candidate


class ParkingAutotuneTest(unittest.TestCase):
    @staticmethod
    def observation(stamp, **overrides):
        value = {
            "schema": "parking_observation_v1",
            "stamp": stamp,
            "frame_id": "base_link",
            "valid": True,
            "confidence": 0.95,
            "slot_id": "P4",
            "slot_visible": True,
            "rear_visible": True,
            "exit_visible": True,
            "front_clear": True,
            "rear_clear": True,
            "exit_clear": True,
            "turn_distance_m": 0.65,
            "vehicle_yaw_error_rad": -0.60,
            "lateral_error_m": 0.04,
            "rear_distance_m": 0.12,
            "reverse_ready": True,
            "exit_distance_m": 0.08,
            "exit_yaw_error_rad": 0.05,
            "exit_lateral_error_m": 0.08,
            "exit_complete": False,
            "parking_request": True,
        }
        value.update(overrides)
        return value

    @staticmethod
    def record(stamp, event, payload, run_id="run-1"):
        return {
            "wall_time": stamp,
            "run_id": run_id,
            "event": event,
            "data": {"payload": payload},
        }

    def make_successful_run(self, run_id="run-1"):
        samples = [
            (1.0, "forward_right_turn", {"turn_distance_m": 0.65}),
            (2.0, "forward_straighten", {"vehicle_yaw_error_rad": -0.60}),
            (3.0, "settle", {"vehicle_yaw_error_rad": 0.04}),
            (4.0, "reverse_steer_in", {"rear_distance_m": 0.80}),
            (5.0, "reverse_straighten", {"rear_distance_m": 0.50}),
            (6.0, "reverse_align", {"vehicle_yaw_error_rad": 0.05}),
            (7.0, "parked", {"rear_distance_m": 0.12}),
            (8.0, "exit_turn", {"exit_distance_m": 0.60}),
            (9.0, "exit_straighten", {"exit_distance_m": 0.40}),
            (10.0, "done", {"exit_distance_m": 0.08}),
        ]
        records = [{
            "wall_time": 0.0,
            "run_id": run_id,
            "event": "logger_start",
            "data": {
            "config_file": "/tmp/parking.yaml",
            "config_sha256": "abc123",
            "config_hashes": {"/tmp/parking.yaml": "abc123"},
            },
        }, self.record(0.0, "status", {
            "state": "approach", "slot_id": "P4",
            "calibration_complete": True}, run_id)]
        for stamp, state, overrides in samples:
            records.append(self.record(
                stamp - 0.01,
                "observation",
                self.observation(stamp - 0.01, **overrides),
                run_id,
            ))
            records.append(self.record(stamp, "status", {
                "state": state, "slot_id": "P4",
                "calibration_complete": True}, run_id))
        return records

    def test_builds_profile_from_successful_transition_observations(self):
        report = build_candidate(
            self.make_successful_run(),
            min_runs=1,
            margin_m=0.02,
            margin_rad=0.03,
            observation_age_s=0.1,
        )

        self.assertEqual(report["successful_run_count"], 1)
        self.assertEqual(report["used_run_ids"], ["run-1"])
        self.assertEqual(report["run_metadata"][0]["config_sha256"], "abc123")
        self.assertEqual(
            report["run_metadata"][0]["config_hashes"]["/tmp/parking.yaml"],
            "abc123",
        )
        profile = report["slot_profiles"]["P4"]
        self.assertAlmostEqual(profile["turn_start_distance_m"], 0.67)
        self.assertAlmostEqual(
            profile["forward_turn_target_yaw_error_rad"], -0.60)
        self.assertAlmostEqual(profile["reverse_steer_switch_distance_m"], 0.52)
        self.assertAlmostEqual(profile["park_stop_distance_m"], 0.14)
        # Completion is declared when distance <= threshold, so the
        # conservative margin must move this threshold inward, not outward.
        self.assertAlmostEqual(profile["exit_complete_distance_m"], 0.06)

    def test_failed_run_is_not_used_by_default(self):
        records = self.make_successful_run("failed")
        records.append(self.record(9.0, "status", {
            "state": "fault", "slot_id": "P4"}, "failed"))
        report = build_candidate(records, min_runs=1)
        self.assertEqual(report["successful_run_count"], 0)
        self.assertEqual(report["slot_profiles"], {})

    def test_failed_run_is_diagnostic_only_when_explicitly_allowed(self):
        records = self.make_successful_run("failed")
        records.append(self.record(9.0, "status", {
            "state": "fault", "slot_id": "P4"}, "failed"))
        report = build_candidate(records, min_runs=1, allow_failed=True)
        self.assertEqual(report["diagnostic_run_count"], 1)
        self.assertEqual(report["diagnostic_run_ids"], ["failed"])
        self.assertEqual(report["slot_profiles"], {})

    def test_incomplete_state_sequence_cannot_generate_a_profile(self):
        records = self.make_successful_run("incomplete")
        records = [
            record for record in records
            if not (record["event"] == "status" and
                    record["data"].get("payload", {}).get("state") == "done")
        ]
        report = build_candidate(records, min_runs=1)
        self.assertEqual(report["successful_run_count"], 0)
        self.assertEqual(report["slot_profiles"], {})


if __name__ == "__main__":
    unittest.main()
