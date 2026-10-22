# -*- coding: utf-8 -*-

"""Regression tests for the non-ROS command-policy safety gate."""

from __future__ import division

import os
import sys
import types
import unittest


SCRIPTS_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)


def _install_import_stubs():
    if "rospy" not in sys.modules:
        sys.modules["rospy"] = types.ModuleType("rospy")
    if "std_msgs" not in sys.modules:
        sys.modules["std_msgs"] = types.ModuleType("std_msgs")
    if "std_msgs.msg" not in sys.modules:
        message_module = types.ModuleType("std_msgs.msg")

        class Bool(object):
            pass

        class String(object):
            pass

        message_module.Bool = Bool
        message_module.String = String
        sys.modules["std_msgs.msg"] = message_module
    rospy_module = sys.modules["rospy"]
    if not hasattr(rospy_module, "loginfo"):
        rospy_module.loginfo = lambda *args, **kwargs: None


_install_import_stubs()

from legacy_controller import (  # noqa: E402
    AutoDriveController,
    green_start_command,
    is_green_start_label,
    is_confirmed_sign_label,
    observation_parking_trigger_visible,
    preparking_lidar_block_reason,
    observation_target_ready,
    resolve_parking_only,
    should_wait_before_parking,
    update_pending_action,
)


class ParkingMainGuardTest(unittest.TestCase):
    @staticmethod
    def parking_observation(trigger=False):
        return {
            "schema": "parking_observation_v1",
            "parking_armed": True,
            "slot_selection_ready": True,
            "selected_slot_id": "P5",
            "slot_obstacle_detected": False,
            "parking_side": "right",
            "parking_trigger_visible": bool(trigger),
        }

    def test_white_target_does_not_release_fsm_without_blue_start_line(self):
        observation = self.parking_observation(trigger=False)
        self.assertTrue(observation_target_ready(observation))
        self.assertFalse(observation_parking_trigger_visible(observation))

    def test_selected_target_and_blue_start_line_release_fsm_gate(self):
        observation = self.parking_observation(trigger=True)
        self.assertTrue(observation_target_ready(observation))
        self.assertTrue(observation_parking_trigger_visible(observation))

    def test_decide_control_mode_keeps_straight_until_blue_start_line(self):
        controller = AutoDriveController.__new__(AutoDriveController)

        class Config(object):
            calibration_complete = True
            require_lidar_clear = False
            forward_speed_raw = 20

        class ParkingControllerStub(object):
            done = False
            holding = False
            config = Config()

        controller.parking_controller = ParkingControllerStub()
        controller.image_data = self.parking_observation(trigger=False)
        controller.sign_label = None
        controller.lidar_data = None
        controller.allow_legacy_parking_trigger = False
        controller.parking_only = False
        controller.require_calibrated_parking = False
        controller.allow_experimental_motion = False
        controller.allow_preparking_approach = False
        controller.max_speed_raw = 100
        controller.green_start_speed_raw = 20
        controller.last_preparking_lidar_reason = None
        self.assertEqual(
            controller.decide_control_mode(), "parking_preparking_approach")

        controller.image_data = self.parking_observation(trigger=True)
        self.assertEqual(controller.decide_control_mode(), "reverse_parking")

    def test_green_start_accepts_confirmed_case_insensitive_label(self):
        self.assertTrue(is_green_start_label({
            "label": "GREEN",
            "valid": True,
            "confirmed": True,
        }))

    def test_green_start_rejects_unconfirmed_structured_label(self):
        self.assertFalse(is_green_start_label({
            "label": "green",
            "valid": True,
            "confirmed": False,
        }))

    def test_green_start_command_uses_initial_speed_and_straight_heading(self):
        self.assertEqual(
            green_start_command(20, 100),
            {"speed_raw": 20, "steering_raw": 0})

    def test_confirmed_red_sign_stops_after_competition_start(self):
        controller = AutoDriveController.__new__(AutoDriveController)

        class Config(object):
            calibration_complete = True
            require_lidar_clear = False

        class ParkingControllerStub(object):
            done = False
            holding = False
            config = Config()

        controller.parking_controller = ParkingControllerStub()
        controller.competition_mode = True
        controller.competition_started = True
        controller.competition_state = "RUNNING"
        controller.sign_label = {
            "label": "RED",
            "valid": True,
            "confirmed": True,
        }
        controller.image_data = None
        controller.lidar_data = None
        controller.allow_legacy_parking_trigger = False
        controller.parking_only = False
        controller.require_calibrated_parking = False
        controller.allow_experimental_motion = False
        controller.allow_preparking_approach = False
        controller.last_preparking_lidar_reason = None

        self.assertEqual(
            controller.decide_control_mode(), "traffic_sign_detection")

    def test_unconfirmed_red_sign_does_not_stop_the_running_lane_controller(self):
        self.assertFalse(is_confirmed_sign_label({
            "label": "RED",
            "valid": True,
            "confirmed": False,
        }, ("red", "stop", "yield")))
        self.assertTrue(is_confirmed_sign_label({
            "label": "RED",
            "valid": True,
            "confirmed": True,
        }, ("red", "stop", "yield")))

    def test_camera_sign_payload_can_latch_each_post_start_action_class(self):
        for label in ("STRAIGHT", "LEFT", "RIGHT", "UTURN", "PARKING"):
            self.assertEqual(
                update_pending_action("NONE", {
                    "label": label,
                    "valid": True,
                    "confirmed": True,
                }), label)

    def test_green_label_selects_green_start_mode(self):
        controller = AutoDriveController.__new__(AutoDriveController)

        class Config(object):
            calibration_complete = True
            require_lidar_clear = False

        class ParkingControllerStub(object):
            done = False
            holding = False
            config = Config()

        controller.parking_controller = ParkingControllerStub()
        controller.image_data = None
        controller.sign_label = {
            "label": "GREEN",
            "valid": True,
            "confirmed": True,
        }
        controller.lidar_data = None
        controller.allow_legacy_parking_trigger = False
        controller.parking_only = False
        controller.require_calibrated_parking = False
        controller.allow_experimental_motion = False
        controller.allow_preparking_approach = False
        controller.max_speed_raw = 100
        controller.green_start_speed_raw = 20

        self.assertEqual(controller.decide_control_mode(), "green_start")
        self.assertEqual(
            controller.command_for_mode("green_start"),
            {"speed_raw": 20, "steering_raw": 0})

    def test_competition_waits_for_green_even_with_lane_command(self):
        controller = AutoDriveController.__new__(AutoDriveController)
        controller.competition_mode = True
        controller.competition_started = False
        controller.competition_state = "WAIT_GREEN"
        controller.sign_label = None
        controller.lane_data = {"speed_raw": 20, "steering_raw": 0}
        controller.lidar_data = None

        self.assertEqual(
            controller.decide_control_mode(), "waiting_for_green")

    def test_competition_green_latches_and_missing_green_does_not_wait_again(self):
        controller = AutoDriveController.__new__(AutoDriveController)

        class Config(object):
            calibration_complete = True
            require_lidar_clear = False

        class ParkingControllerStub(object):
            done = False
            holding = False
            config = Config()

        controller.parking_controller = ParkingControllerStub()
        controller.competition_mode = True
        controller.competition_started = False
        controller.competition_state = "WAIT_GREEN"
        controller.green_start_hold_sec = 0.50
        controller.green_start_until = 0.0
        controller.green_start_speed_raw = 20
        controller.max_speed_raw = 100
        controller.max_steering_raw = 22
        controller.image_data = None
        controller.lidar_data = None
        controller.allow_legacy_parking_trigger = False
        controller.parking_only = False
        controller.require_calibrated_parking = False
        controller.allow_experimental_motion = False
        controller.allow_preparking_approach = False
        controller.last_preparking_lidar_reason = None
        controller.lane_data = None
        controller.sign_label = {
            "label": "green",
            "valid": True,
            "confirmed": True,
        }

        self.assertEqual(controller.decide_control_mode(), "lane_following")
        self.assertTrue(controller.competition_started)
        self.assertEqual(controller.competition_state, "RUNNING")

        # Simulate the confirmed label expiring after the one-shot start
        # event.  The latched RUNNING state must keep control with the lane
        # source instead of returning to the green wait gate.
        controller.sign_label = None
        controller.green_start_until = 0.0
        controller.lane_data = {"speed_raw": 18, "steering_raw": -2}
        self.assertEqual(controller.decide_control_mode(), "lane_following")
        self.assertEqual(
            controller.command_for_mode("lane_following"),
            {"speed_raw": 18, "steering_raw": -2})

        # A later green result is perception-only; it cannot reopen the
        # start phase or switch the already-running competition mode.
        controller.sign_label = {
            "label": "green",
            "valid": True,
            "confirmed": True,
        }
        self.assertEqual(controller.decide_control_mode(), "lane_following")
        self.assertEqual(controller.competition_state, "RUNNING")

    def test_valid_lane_command_takes_over_during_initial_green_boost(self):
        controller = AutoDriveController.__new__(AutoDriveController)

        class Config(object):
            calibration_complete = True
            require_lidar_clear = False

        class ParkingControllerStub(object):
            done = False
            holding = False
            config = Config()

        controller.parking_controller = ParkingControllerStub()
        controller.competition_mode = True
        controller.competition_started = False
        controller.competition_state = "WAIT_GREEN"
        controller.green_start_hold_sec = 0.50
        controller.green_start_until = 0.0
        controller.green_start_speed_raw = 20
        controller.max_speed_raw = 100
        controller.max_steering_raw = 22
        controller.image_data = None
        controller.lidar_data = None
        controller.lane_data = {"speed_raw": 20, "steering_raw": 7}
        controller.allow_legacy_parking_trigger = False
        controller.parking_only = False
        controller.require_calibrated_parking = False
        controller.allow_experimental_motion = False
        controller.allow_preparking_approach = False
        controller.last_preparking_lidar_reason = None
        controller.sign_label = {
            "label": "green",
            "valid": True,
            "confirmed": True,
        }

        self.assertEqual(controller.decide_control_mode(), "lane_following")
        self.assertEqual(
            controller.command_for_mode("lane_following"),
            {"speed_raw": 20, "steering_raw": 7})
        self.assertEqual(controller.competition_state, "RUNNING")

    def test_unconfirmed_green_does_not_latch_competition_start(self):
        controller = AutoDriveController.__new__(AutoDriveController)
        controller.competition_mode = True
        controller.competition_started = False
        controller.competition_state = "WAIT_GREEN"
        controller.sign_label = {
            "label": "green",
            "valid": True,
            "confirmed": False,
        }
        self.assertEqual(
            controller.decide_control_mode(), "waiting_for_green")
        self.assertFalse(controller.competition_started)

    def test_preparking_approach_ignores_lane_adapter_steering(self):
        controller = AutoDriveController.__new__(AutoDriveController)

        class Config(object):
            forward_speed_raw = 20

        class ParkingControllerStub(object):
            config = Config()
            experimental_speed_limit_raw = 20

        controller.parking_controller = ParkingControllerStub()
        # The field failure was a lane adapter command of +18 while the
        # parking route was supposed to remain straight before the right turn.
        controller.lane_data = {"speed_raw": 20, "steering_raw": 18}

        self.assertEqual(
            controller.command_for_mode("parking_preparking_approach"),
            {"speed_raw": 20, "steering_raw": 0})

    def test_preparking_approach_mode_owns_heading_before_trigger(self):
        controller = AutoDriveController.__new__(AutoDriveController)

        class Config(object):
            calibration_complete = False
            require_lidar_clear = True
            forward_speed_raw = 20

        class ParkingControllerStub(object):
            done = False
            holding = False
            config = Config()
            experimental_speed_limit_raw = 20

        controller.parking_controller = ParkingControllerStub()
        controller.image_data = None
        controller.sign_label = None
        controller.lidar_data = {
            "schema": "parking_lidar_v1",
            "frame_id": "base_link",
            "valid": True,
            "obstacle_detected": False,
        }
        controller.allow_legacy_parking_trigger = False
        controller.parking_only = False
        controller.require_calibrated_parking = False
        controller.allow_experimental_motion = True
        controller.allow_preparking_approach = True

        self.assertEqual(
            controller.decide_control_mode(), "parking_preparking_approach")

    def test_invalid_config_forces_zero_command_policy(self):
        self.assertTrue(resolve_parking_only(False, False))

    def test_valid_config_keeps_explicit_requested_policy(self):
        self.assertFalse(resolve_parking_only(False, True))
        self.assertTrue(resolve_parking_only(True, True))

    def test_parking_launch_waits_until_calibration_when_preparking_is_enabled(self):
        self.assertTrue(should_wait_before_parking(
            parking_only=False,
            require_calibrated_parking=True,
            calibration_complete=False,
        ))

    def test_calibrated_parking_launch_can_follow_lane_before_trigger(self):
        self.assertFalse(should_wait_before_parking(
            parking_only=False,
            require_calibrated_parking=True,
            calibration_complete=True,
        ))

    def test_experimental_motion_explicitly_opens_only_calibration_wait_gate(self):
        self.assertFalse(should_wait_before_parking(
            parking_only=False,
            require_calibrated_parking=True,
            calibration_complete=False,
            allow_experimental_motion=True,
        ))

    def test_parking_only_still_wins_over_experimental_motion(self):
        self.assertTrue(should_wait_before_parking(
            parking_only=True,
            require_calibrated_parking=True,
            calibration_complete=False,
            allow_experimental_motion=True,
        ))

    def test_explicit_experimental_preparking_approach_opens_lane_gate(self):
        self.assertFalse(should_wait_before_parking(
            parking_only=True,
            require_calibrated_parking=True,
            calibration_complete=False,
            allow_experimental_motion=True,
            allow_preparking_approach=True,
        ))

    def test_legacy_launch_policy_remains_explicit(self):
        self.assertFalse(should_wait_before_parking(
            parking_only=False,
            require_calibrated_parking=False,
            calibration_complete=False,
        ))

    def test_preparking_lidar_gate_blocks_missing_payload(self):
        self.assertEqual(
            preparking_lidar_block_reason(None), "lidar_missing")

    def test_preparking_lidar_gate_accepts_only_valid_clear_base_link(self):
        self.assertIsNone(preparking_lidar_block_reason({
            "schema": "parking_lidar_v1",
            "frame_id": "base_link",
            "valid": True,
            "obstacle_detected": False,
        }))

    def test_preparking_lidar_gate_blocks_bad_frame_and_obstacle(self):
        payload = {
            "schema": "parking_lidar_v1",
            "frame_id": "laser",
            "valid": True,
            "obstacle_detected": False,
        }
        self.assertEqual(
            preparking_lidar_block_reason(payload), "lidar_frame_invalid")
        payload["frame_id"] = "base_link"
        payload["obstacle_detected"] = True
        self.assertEqual(
            preparking_lidar_block_reason(payload), "obstacle_detected")

    def test_preparking_lidar_gate_can_be_disabled_for_legacy_launch(self):
        self.assertIsNone(preparking_lidar_block_reason(
            None, require_lidar_clear=False))


if __name__ == "__main__":
    unittest.main()
