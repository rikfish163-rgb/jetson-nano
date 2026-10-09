# -*- coding: utf-8 -*-

from __future__ import print_function

import os
import sys
import unittest


SCRIPTS_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

import reverse_parking


class ReverseParkingEntrypointTest(unittest.TestCase):
    def test_slot_is_normalized_and_visual_mode_is_safe_by_default(self):
        args = reverse_parking.parse_args(["p5"])
        command = reverse_parking.build_command(args)
        self.assertEqual(args.slot, "P5")
        self.assertIn("slot_id:=AUTO", command)
        self.assertIn("requested_slot_id:=P5", command)
        self.assertIn("parking_only:=true", command)
        self.assertIn("allow_preparking_approach:=false", command)
        self.assertIn("allow_experimental_motion:=false", command)
        self.assertIn("require_calibrated_parking:=true", command)
        self.assertNotIn("start_actuators:=true", command)

    def test_motion_mode_propagates_target_and_all_closed_loop_inputs(self):
        args = reverse_parking.parse_args([
            "P4", "--enable-motion", "--start-actuators", "--start-cameras",
            "--speed-limit-raw", "18",
            "--front-marker-input-mode", "metric_white",
        ])
        command = reverse_parking.build_command(args)
        self.assertIn("slot_id:=AUTO", command)
        self.assertIn("requested_slot_id:=P4", command)
        self.assertIn("start_actuators:=true", command)
        self.assertIn("start_cameras:=true", command)
        self.assertIn("start_lidar_adapter:=true", command)
        self.assertIn("start_lidar_source:=true", command)
        self.assertIn("parking_only:=false", command)
        self.assertIn("allow_preparking_approach:=true", command)
        self.assertIn("start_preparking_lane_adapter:=false", command)
        self.assertIn("preparking_speed_raw:=20", command)
        self.assertIn("allow_legacy_parking_trigger:=false", command)
        self.assertIn("experimental_speed_limit_raw:=18", command)
        self.assertIn("front_marker_input_mode:=metric_white", command)
        self.assertIn("experimental_steering_limit_raw:=22", command)
        self.assertIn("enable_exit_pose:=true", command)
        self.assertIn("allow_experimental_motion:=true", command)
        self.assertIn("require_calibrated_parking:=false", command)

    def test_missing_slot_uses_the_explicit_prompt_function(self):
        args = reverse_parking.parse_args([], input_func=lambda _prompt: "p4")
        self.assertEqual(args.slot, "P4")

    def test_auto_is_a_valid_dynamic_selection_preference(self):
        args = reverse_parking.parse_args(["auto"])
        command = reverse_parking.build_command(args)
        self.assertEqual(args.slot, "AUTO")
        self.assertIn("slot_id:=AUTO", command)
        self.assertIn("requested_slot_id:=AUTO", command)


if __name__ == "__main__":
    unittest.main()
