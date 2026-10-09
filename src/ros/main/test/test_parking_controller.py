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

from parking_controller import ParkingConfig, ParkingController


class ParkingControllerTest(unittest.TestCase):
    @staticmethod
    def calibrated_profile(**overrides):
        profile = dict(
            (field, ParkingConfig.DEFAULTS[field])
            for field in ParkingConfig.CALIBRATED_SLOT_FIELDS
        )
        profile.update(overrides)
        return profile

    def make_controller(self, **overrides):
        profile = self.calibrated_profile()
        settings = {
            "calibration_complete": True,
            "require_lidar_clear": True,
            "condition_debounce_frames": 1,
            "observation_timeout_s": 10.0,
            "settle_time_s": 0.0,
            "parked_hold_s": 0.0,
            "forward_speed_raw": 12,
            "reverse_speed_raw": -10,
            "exit_speed_raw": 10,
            "entry_steering_raw": -22,
            "straighten_steering_raw": 8,
            "exit_steering_raw": 10,
            "exit_straighten_steering_raw": -6,
            "turn_start_distance_m": 0.70,
            "forward_turn_target_yaw_error_rad": -0.55,
            "setup_yaw_tolerance_rad": 0.10,
            "reverse_steer_switch_distance_m": 0.55,
            "reverse_yaw_tolerance_rad": 0.10,
            "park_stop_distance_m": 0.15,
            "park_lateral_tolerance_m": 0.08,
            "park_yaw_tolerance_rad": 0.08,
            "exit_straighten_distance_m": 0.45,
            "exit_complete_distance_m": 0.12,
            "exit_yaw_tolerance_rad": 0.12,
            "exit_lateral_tolerance_m": 0.15,
            "slot_profiles": {"P4": dict(profile), "P5": dict(profile)},
        }
        settings.update(overrides)
        if "slot_profiles" in overrides:
            supplied_profiles = overrides["slot_profiles"]
            settings["slot_profiles"] = dict(
                (slot_id, dict(self.calibrated_profile(**slot_profile)))
                for slot_id, slot_profile in supplied_profiles.items()
            )
        return ParkingController(ParkingConfig.from_dict(settings))

    @staticmethod
    def observation(**overrides):
        data = {
            "schema": "parking_observation_v1",
            "stamp": 100.0,
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

    def step(self, controller, observation, now):
        return controller.step(observation, self.lidar(), now=now)

    def test_requires_calibration_before_motion(self):
        controller = ParkingController(ParkingConfig())
        self.assertFalse(controller.start("P4", now=100.0))
        command = controller.step(self.observation(), self.lidar(), now=100.0)
        self.assertEqual(command["state"], "fault")
        self.assertEqual(command["reason"], "calibration_incomplete")
        self.assertEqual((command["speed_raw"], command["steering_raw"]), (0, 0))

    def test_measured_chassis_speed_sign_convention_is_enforced(self):
        config = ParkingConfig.from_dict({
            "forward_speed_raw": 12,
            "reverse_speed_raw": -10,
            "exit_speed_raw": 10,
        })
        self.assertEqual(config.forward_speed_raw, 12)
        self.assertEqual(config.reverse_speed_raw, -10)
        self.assertEqual(config.exit_speed_raw, 10)

        for invalid in (
                {"forward_speed_raw": -12},
                {"reverse_speed_raw": 10},
                {"exit_speed_raw": -10}):
            with self.assertRaises(ValueError):
                ParkingConfig.from_dict(invalid)

    def test_experimental_motion_is_explicit_and_hard_bounded(self):
        controller = ParkingController(
            ParkingConfig({
                "calibration_complete": False,
                "condition_debounce_frames": 1,
                "observation_timeout_s": 10.0,
            }),
            allow_experimental_motion=True,
            experimental_speed_limit_raw=1,
            experimental_steering_limit_raw=4,
        )
        self.assertTrue(controller.start("P5", now=100.0))
        command = self.step(
            controller,
            self.observation(
                slot_id="P5",
                turn_distance_m=0.40,
            ),
            100.0,
        )
        self.assertEqual(command["state"], "forward_right_turn")
        self.assertEqual(command["speed_raw"], 1)
        self.assertEqual(command["steering_raw"], -4)
        self.assertTrue(controller.status()["experimental_motion"])

    def test_experimental_motion_accepts_measured_full_steering_limit(self):
        controller = ParkingController(
            ParkingConfig({
                "entry_steering_raw": -22,
                "condition_debounce_frames": 1,
            }),
            allow_experimental_motion=True,
            experimental_speed_limit_raw=20,
            experimental_steering_limit_raw=22,
        )
        self.assertTrue(controller.start("P5", now=100.0))
        command = self.step(
            controller,
            self.observation(slot_id="P5", turn_distance_m=0.40),
            100.0,
        )
        self.assertEqual(command["state"], "forward_right_turn")
        self.assertEqual(command["speed_raw"], 12)
        self.assertEqual(command["steering_raw"], -22)

    def test_approach_allows_straight_motion_without_front_pose(self):
        controller = self.make_controller()
        self.assertTrue(controller.start("P4", now=100.0))
        command = self.step(
            controller,
            self.observation(
                vehicle_yaw_error_rad=None,
                lateral_error_m=None,
                turn_distance_m=1.0,
            ),
            100.0,
        )
        self.assertEqual(command["state"], "approach")
        self.assertEqual(command["reason"], "approach")
        self.assertEqual((command["speed_raw"], command["steering_raw"]),
                         (12, 0))

    def test_approach_stops_at_turn_trigger_without_front_pose(self):
        controller = self.make_controller()
        self.assertTrue(controller.start("P4", now=100.0))
        command = self.step(
            controller,
            self.observation(
                vehicle_yaw_error_rad=None,
                lateral_error_m=None,
                turn_distance_m=0.60,
            ),
            100.0,
        )
        self.assertEqual(command["state"], "approach")
        self.assertEqual(command["reason"], "yaw_error_missing")
        self.assertEqual((command["speed_raw"], command["steering_raw"]),
                         (0, 0))

    def test_reverse_parking_and_exit_state_sequence(self):
        controller = self.make_controller()
        self.assertTrue(controller.start("P4", now=100.0))

        command = self.step(controller, self.observation(), 100.0)
        self.assertEqual(command["state"], "approach")
        self.assertEqual(command["speed_raw"], 12)
        self.assertEqual(command["steering_raw"], 0)

        command = self.step(
            controller, self.observation(turn_distance_m=0.60), 100.1
        )
        self.assertEqual(command["state"], "forward_right_turn")
        self.assertEqual(command["steering_raw"], -22)

        command = self.step(
            controller,
            self.observation(vehicle_yaw_error_rad=-0.60),
            100.2,
        )
        self.assertEqual(command["state"], "forward_straighten")
        self.assertEqual(command["steering_raw"], 8)

        command = self.step(
            controller,
            self.observation(
                vehicle_yaw_error_rad=0.0,
                reverse_ready=True,
            ),
            100.3,
        )
        self.assertEqual(command["state"], "settle")
        self.assertEqual((command["speed_raw"], command["steering_raw"]), (0, 0))

        command = self.step(controller, self.observation(), 100.4)
        self.assertEqual(command["state"], "reverse_steer_in")
        self.assertEqual(command["speed_raw"], -10)
        self.assertEqual(command["steering_raw"], -22)

        command = self.step(
            controller,
            self.observation(rear_distance_m=0.50),
            100.5,
        )
        self.assertEqual(command["state"], "reverse_straighten")
        self.assertEqual(command["steering_raw"], 8)

        command = self.step(
            controller,
            self.observation(
                rear_distance_m=0.40,
                vehicle_yaw_error_rad=0.0,
            ),
            100.6,
        )
        self.assertEqual(command["state"], "reverse_align")
        self.assertEqual(command["speed_raw"], -10)

        command = self.step(
            controller,
            self.observation(
                rear_distance_m=0.10,
                lateral_error_m=0.0,
                vehicle_yaw_error_rad=0.0,
            ),
            100.7,
        )
        self.assertEqual(command["state"], "parked")
        self.assertEqual((command["speed_raw"], command["steering_raw"]), (0, 0))

        command = self.step(
            controller,
            self.observation(
                exit_distance_m=0.60,
                exit_yaw_error_rad=0.20,
            ),
            100.8,
        )
        self.assertEqual(command["state"], "exit_turn")
        self.assertEqual(command["speed_raw"], 10)
        self.assertEqual(command["steering_raw"], 10)

        command = self.step(
            controller,
            self.observation(
                exit_distance_m=0.40,
                exit_yaw_error_rad=0.05,
            ),
            100.9,
        )
        self.assertEqual(command["state"], "exit_straighten")
        self.assertEqual(command["steering_raw"], -6)

        command = self.step(
            controller,
            self.observation(
                exit_distance_m=0.10,
                exit_yaw_error_rad=0.0,
                exit_lateral_error_m=0.0,
            ),
            101.0,
        )
        self.assertEqual(command["state"], "done")
        self.assertTrue(command["done"])
        self.assertEqual((command["speed_raw"], command["steering_raw"]), (0, 0))

    def test_parked_route_clears_slot_before_left_exit_turn(self):
        profile = self.calibrated_profile(
            exit_forward_clearance_distance_m=0.40,
            exit_forward_yaw_tolerance_rad=0.12,
            exit_forward_lateral_tolerance_m=0.15,
            exit_forward_steering_raw=0,
        )
        config = ParkingConfig.from_dict({
            "calibration_complete": True,
            "require_lidar_clear": True,
            "condition_debounce_frames": 1,
            "observation_timeout_s": 10.0,
            "settle_time_s": 0.0,
            "parked_hold_s": 0.0,
            "slot_profiles": {
                "P4": dict(profile),
                "P5": dict(profile),
            },
        })
        controller = ParkingController(config)
        self.assertTrue(controller.start("P4", now=100.0))
        controller.state = "parked"
        controller.state_entered_at = 100.0

        command = self.step(
            controller,
            self.observation(rear_distance_m=0.20),
            100.0,
        )
        self.assertEqual(command["state"], "exit_forward_clearance")
        self.assertEqual(command["speed_raw"], 10)
        self.assertEqual(command["steering_raw"], 0)

        command = self.step(
            controller,
            self.observation(rear_distance_m=0.45),
            100.1,
        )
        self.assertEqual(command["state"], "exit_turn")
        self.assertEqual(command["speed_raw"], 10)
        self.assertEqual(command["steering_raw"], 10)

    def test_stale_or_wrong_frame_observation_stops_without_transition(self):
        controller = self.make_controller()
        controller.start("P4", now=100.0)

        stale = self.observation(stamp=80.0)
        command = self.step(controller, stale, 100.0)
        self.assertEqual(command["state"], "approach")
        self.assertEqual(command["reason"], "observation_stale")
        self.assertTrue(command["safe_stop"])

        wrong_frame = self.observation(frame_id="map")
        command = self.step(controller, wrong_frame, 100.0)
        self.assertEqual(command["reason"], "observation_frame_invalid")
        self.assertTrue(command["safe_stop"])

        wrong_slot = self.observation(slot_id="P5")
        command = self.step(controller, wrong_slot, 100.0)
        self.assertEqual(command["reason"], "observation_slot_mismatch")
        self.assertTrue(command["safe_stop"])

    def test_lidar_obstacle_latches_fault(self):
        controller = self.make_controller()
        controller.start("P4", now=100.0)
        command = controller.step(
            self.observation(),
            {"schema": "parking_lidar_v1", "frame_id": "base_link",
             "valid": True, "obstacle_detected": True},
            now=100.0,
        )
        self.assertEqual(command["state"], "fault")
        self.assertEqual(command["reason"], "obstacle_detected")
        self.assertEqual((command["speed_raw"], command["steering_raw"]), (0, 0))

    def test_lidar_emergency_stop_latches_fault(self):
        controller = self.make_controller()
        controller.start("P4", now=100.0)
        command = controller.step(
            self.observation(),
            {"schema": "parking_lidar_v1", "frame_id": "base_link",
             "valid": True, "obstacle_detected": False,
             "emergency_stop": True},
            now=100.0,
        )
        self.assertEqual(command["state"], "fault")
        self.assertEqual(command["reason"], "emergency_stop")
        self.assertEqual((command["speed_raw"], command["steering_raw"]),
                         (0, 0))

    def test_reverse_align_feedback_is_bounded(self):
        controller = self.make_controller(
            steering_lateral_gain_raw_per_m=100.0,
            steering_yaw_gain_raw_per_rad=100.0,
        )
        controller.start("P4", now=100.0)
        controller.state = "reverse_align"
        command = self.step(
            controller,
            self.observation(
                rear_distance_m=0.50,
                lateral_error_m=1.0,
                vehicle_yaw_error_rad=1.0,
            ),
            100.0,
        )
        self.assertEqual(command["speed_raw"], -10)
        self.assertEqual(command["steering_raw"], 22)

    def test_forward_turn_feedback_uses_target_and_is_bounded(self):
        controller = self.make_controller(
            forward_turn_yaw_gain_raw_per_rad=8.0,
        )
        controller.start("P4", now=100.0)
        controller.state = "forward_right_turn"
        command = self.step(
            controller,
            self.observation(vehicle_yaw_error_rad=-0.20),
            100.0,
        )
        # Nominal -22 plus 8 * (target - current) is below the raw limit.
        self.assertEqual(command["speed_raw"], 12)
        self.assertEqual(command["steering_raw"], -22)

    def test_forward_lateral_feedback_is_optional_and_requires_fresh_error(self):
        controller = self.make_controller(
            forward_turn_lateral_gain_raw_per_m=10.0,
        )
        controller.start("P4", now=100.0)
        controller.state = "forward_right_turn"
        command = self.step(
            controller,
            self.observation(lateral_error_m=0.20),
            100.0,
        )
        self.assertEqual(command["steering_raw"], -20)

        command = self.step(
            controller,
            self.observation(lateral_error_m=None),
            100.0,
        )
        self.assertEqual(command["reason"], "lateral_error_m_missing")
        self.assertEqual((command["speed_raw"], command["steering_raw"]),
                         (0, 0))

    def test_exit_lateral_feedback_is_bounded_and_requires_error(self):
        controller = self.make_controller(
            exit_turn_lateral_gain_raw_per_m=100.0,
        )
        controller.start("P4", now=100.0)
        controller.state = "exit_turn"
        command = self.step(
            controller,
            self.observation(exit_distance_m=0.60, exit_lateral_error_m=0.20),
            100.0,
        )
        self.assertEqual(command["speed_raw"], 10)
        self.assertEqual(command["steering_raw"], 22)

        command = self.step(
            controller,
            self.observation(exit_distance_m=0.60,
                             exit_lateral_error_m=None),
            100.0,
        )
        self.assertEqual(command["reason"], "exit_lateral_error_m_missing")
        self.assertEqual((command["speed_raw"], command["steering_raw"]),
                         (0, 0))

    def test_exit_turn_waits_for_white_line_parallel_yaw(self):
        controller = self.make_controller()
        controller.start("P4", now=100.0)
        controller.state = "exit_turn"
        command = self.step(
            controller,
            self.observation(
                exit_distance_m=0.10,
                exit_yaw_error_rad=0.30,
            ),
            100.0,
        )
        self.assertEqual(command["state"], "exit_turn")
        self.assertEqual(command["speed_raw"], 10)
        self.assertEqual(command["steering_raw"], 10)

    def test_status_separates_state_entry_and_last_command_reason(self):
        controller = self.make_controller()
        controller.start("P4", now=100.0)
        observation = self.observation()
        command = self.step(controller, observation, 100.0)
        status = controller.status(
            now=100.0, command=command, observation=observation)
        self.assertEqual(status["schema"], "parking_status_v1")
        self.assertEqual(status["state"], "approach")
        self.assertEqual(status["phase_index"], 1)
        self.assertEqual(status["state_reason"], "started")
        self.assertEqual(status["command_reason"], "approach")
        self.assertEqual(status["observation_valid"], True)

    def test_phase_uses_required_visual_source_confidence(self):
        controller = self.make_controller()
        controller.start("P4", now=100.0)
        command = self.step(
            controller,
            self.observation(source_confidence={
                "front": 0.60,
                "rear": 0.95,
                "exit": 0.95,
            }),
            100.0,
        )
        self.assertEqual(command["reason"], "source_confidence_low")
        self.assertEqual((command["speed_raw"], command["steering_raw"]),
                         (0, 0))

    def test_phase_rejects_missing_required_source_confidence(self):
        controller = self.make_controller()
        controller.start("P4", now=100.0)
        observation = self.observation()
        del observation["source_confidence"]
        command = self.step(controller, observation, 100.0)
        self.assertEqual(command["reason"], "source_confidence_missing")
        self.assertEqual((command["speed_raw"], command["steering_raw"]),
                         (0, 0))

    def test_phase_rejects_missing_required_source_status(self):
        controller = self.make_controller()
        controller.start("P4", now=100.0)
        observation = self.observation()
        del observation["source_status"]
        command = self.step(controller, observation, 100.0)
        self.assertEqual(command["reason"], "source_status_missing")
        self.assertEqual((command["speed_raw"], command["steering_raw"]),
                         (0, 0))

    def test_phase_rejects_stale_required_source_status(self):
        controller = self.make_controller()
        controller.start("P4", now=100.0)
        observation = self.observation()
        observation["source_status"]["front"]["fresh"] = False
        command = self.step(controller, observation, 100.0)
        self.assertEqual(command["reason"], "source_status_stale")
        self.assertEqual((command["speed_raw"], command["steering_raw"]),
                         (0, 0))

    def test_forward_turn_holds_right_turn_through_short_visual_dropout(self):
        controller = self.make_controller(visual_dropout_hold_s=0.40)
        controller.start("P4", now=100.0)
        command = self.step(
            controller,
            self.observation(turn_distance_m=0.60),
            100.0,
        )
        self.assertEqual(command["state"], "forward_right_turn")
        self.assertEqual((command["speed_raw"], command["steering_raw"]),
                         (12, -22))

        dropout = self.observation(
            stamp=100.1,
            valid=False,
            confidence=0.0,
            source_confidence={"front": 0.0, "rear": 0.95, "exit": 0.95},
            source_status={
                "front": {"present": True, "fresh": True, "valid": False},
                "rear": {"present": True, "fresh": True, "valid": True},
                "exit": {"present": True, "fresh": True, "valid": True},
            },
        )
        command = self.step(controller, dropout, 100.30)
        self.assertEqual(command["state"], "forward_right_turn")
        self.assertFalse(command["safe_stop"])
        self.assertEqual((command["speed_raw"], command["steering_raw"]),
                         (12, -22))
        self.assertEqual(command["reason"],
                         "forward_right_turn_visual_hold:observation_invalid")

    def test_forward_turn_stops_after_visual_dropout_hold_expires(self):
        controller = self.make_controller(visual_dropout_hold_s=0.40)
        controller.start("P4", now=100.0)
        self.step(controller, self.observation(turn_distance_m=0.60), 100.0)
        dropout = self.observation(
            stamp=100.1,
            valid=False,
            confidence=0.0,
            source_confidence={"front": 0.0, "rear": 0.95, "exit": 0.95},
            source_status={
                "front": {"present": True, "fresh": True, "valid": False},
                "rear": {"present": True, "fresh": True, "valid": True},
                "exit": {"present": True, "fresh": True, "valid": True},
            },
        )
        command = self.step(controller, dropout, 100.45)
        self.assertEqual(command["state"], "forward_right_turn")
        self.assertTrue(command["safe_stop"])
        self.assertEqual(command["reason"], "observation_invalid")

    def test_invalid_transition_latches_fault_without_motion(self):
        controller = self.make_controller()
        controller.start("P4", now=100.0)
        self.assertFalse(
            controller._transition("reverse_align", 100.0, "bad_skip"))
        command = self.step(controller, self.observation(), 100.0)
        self.assertEqual(command["state"], "fault")
        self.assertIn("invalid_transition", command["reason"])
        self.assertEqual((command["speed_raw"], command["steering_raw"]),
                         (0, 0))

    def test_motion_state_timeout_latches_fault(self):
        controller = self.make_controller(approach_timeout_s=0.50)
        controller.start("P4", now=100.0)
        command = self.step(controller, self.observation(), 100.6)
        self.assertEqual(command["state"], "fault")
        self.assertEqual(command["reason"], "state_timeout_approach")
        self.assertEqual((command["speed_raw"], command["steering_raw"]), (0, 0))

    def test_slot_profile_is_validated_before_motion(self):
        controller = self.make_controller(
            slot_profiles={
                "P4": {"park_stop_distance_m": 0.0},
                "P5": {},
            },
        )
        self.assertFalse(controller.start("P4", now=100.0))
        self.assertEqual(controller.state, "fault")
        self.assertIn("park_stop_distance_m", controller.reason)

    def test_slot_profile_cannot_override_global_calibration_lock(self):
        controller = self.make_controller(
            slot_profiles={
                "P4": {"calibration_complete": False},
                "P5": {},
            },
        )
        self.assertFalse(controller.start("P4", now=100.0))
        self.assertEqual(controller.state, "fault")
        self.assertIn("calibration_complete", controller.reason)

    def test_p5_is_supported_and_side_parking_slots_are_rejected(self):
        controller = self.make_controller(
            slot_profiles={
                "P4": {},
                "P5": {"turn_start_distance_m": 0.60},
            },
        )
        self.assertTrue(controller.start("P5", now=100.0))
        self.assertEqual(controller.slot_id, "P5")
        self.assertEqual(controller.profile["turn_start_distance_m"], 0.60)

        rejected = self.make_controller()
        self.assertFalse(rejected.start("P1", now=100.0))
        self.assertEqual(rejected.state, "fault")
        self.assertIn("slot_id_not_supported", rejected.reason)

    def test_calibrated_config_requires_all_supported_slot_profiles(self):
        with self.assertRaises(ValueError):
            ParkingConfig.from_dict({
                "calibration_complete": True,
                "slot_profiles": {"P4": {}},
            })

    def test_calibrated_config_rejects_empty_measured_profile(self):
        with self.assertRaises(ValueError):
            ParkingConfig.from_dict({
                "calibration_complete": True,
                "slot_profiles": {"P4": {}, "P5": {}},
            })

    def test_left_side_profile_mirrors_nominal_entry_and_exit_commands(self):
        controller = self.make_controller(
            parking_side="left",
            slot_profiles={
                "P4": {"parking_side": "left"},
                "P5": {"parking_side": "left"},
            },
        )
        self.assertTrue(controller.start("P4", now=100.0))
        self.assertEqual(controller.parking_side, "left")
        self.assertEqual(controller.profile["forward_turn_target_yaw_error_rad"],
                         0.55)
        self.assertEqual(controller.profile["entry_steering_raw"], 22)
        self.assertEqual(controller.profile["straighten_steering_raw"], -8)
        self.assertEqual(controller.profile["exit_steering_raw"], -10)
        self.assertEqual(controller.profile["exit_straighten_steering_raw"], 6)

    def test_parking_side_mismatch_faults_a_reform_mission(self):
        controller = self.make_controller()
        controller.start("P4", now=100.0, parking_side="right")
        observation = self.observation(
            parking_armed=True,
            slot_selection_ready=True,
            selected_slot_id="P4",
            slot_obstacle_detected=False,
            slot_id="P4",
            parking_side="left",
        )
        command = self.step(controller, observation, 100.0)
        self.assertEqual(command["state"], "fault")
        self.assertEqual(command["reason"], "parking_side_mismatch")

    def test_parking_boundary_waits_for_stable_alignment(self):
        controller = self.make_controller(condition_debounce_frames=3)
        controller.start("P4", now=100.0)
        controller.state = "reverse_align"
        boundary_observation = self.observation(
            rear_distance_m=0.10,
            lateral_error_m=0.0,
            vehicle_yaw_error_rad=0.0,
        )

        command = self.step(controller, boundary_observation, 100.0)
        self.assertEqual(command["state"], "reverse_align")
        self.assertEqual(command["reason"], "parking_settling")
        self.assertEqual(command["speed_raw"], 0)

        command = self.step(controller, boundary_observation, 100.1)
        self.assertEqual(command["state"], "reverse_align")
        self.assertEqual(command["speed_raw"], 0)

        command = self.step(controller, boundary_observation, 100.2)
        self.assertEqual(command["state"], "parked")
        self.assertEqual(command["speed_raw"], 0)


if __name__ == "__main__":
    unittest.main()
