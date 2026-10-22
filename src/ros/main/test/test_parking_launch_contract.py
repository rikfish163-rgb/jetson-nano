#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Static contract checks for the parking launch topic graph.

This test deliberately uses only the standard-library XML parser.  It catches
an accidentally disconnected node or a missing launch parameter before a
bring-up session, without requiring roscore, cameras, lidar, or actuators.
"""

from __future__ import print_function

import os
import unittest
import xml.etree.ElementTree as ElementTree


TEST_DIR = os.path.dirname(os.path.abspath(__file__))
MAIN_DIR = os.path.abspath(os.path.join(TEST_DIR, ".."))
LAUNCH_PATH = os.path.join(MAIN_DIR, "launch", "parking_closed_loop.launch")
START_ALL_PATH = os.path.join(MAIN_DIR, "launch", "start_all.launch")
VISION_LAUNCH_PATH = os.path.join(
    MAIN_DIR, "launch", "vision_competition.launch")


class ParkingLaunchContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = ElementTree.parse(LAUNCH_PATH).getroot()
        cls.start_all_root = ElementTree.parse(START_ALL_PATH).getroot()
        cls.vision_root = ElementTree.parse(VISION_LAUNCH_PATH).getroot()

    def _node(self, name):
        for node in self.root.findall(".//node"):
            if node.get("name") == name:
                return node
        self.fail("parking launch node is missing: %s" % name)

    @staticmethod
    def _param_names(node):
        return set(param.get("name") for param in node.findall("param"))

    def test_topic_args_are_declared_once_at_launch_boundary(self):
        names = set(arg.get("name") for arg in self.root.findall("arg"))
        expected = {
            "front_image_topic",
            "rear_image_topic",
            "sign_image_topic",
            "slot_candidate_topic",
            "start_line_topic",
            "front_candidate_topic",
            "target_topic",
            "front_target_topic",
            "target_scan_topic",
            "target_zone_calibrated",
            "start_target_selector",
            "requested_slot_id",
            "front_mask_topic",
            "front_overlay_topic",
            "slot_mask_topic",
            "slot_overlay_topic",
            "start_line_mask_topic",
            "start_line_overlay_topic",
            "front_metric_bev_topic",
            "front_marker_input_mode",
            "slot_input_mode",
            "start_line_input_mode",
            "front_marker_max_abs_lateral_m",
            "front_marker_horizontal_open_width",
            "visual_profile_file",
            "front_metric_topic",
            "front_pose_topic",
            "rear_bev_topic",
            "rear_metric_topic",
            "exit_pose_topic",
            "exit_metric_topic",
            "lane_path_topic",
            "lane_confidence_topic",
            "front_pose_min_confidence",
            "parking_min_source_confidence",
            "sign_topic",
            "parking_lidar_topic",
            "sign_send_topic",
            "sign_hz",
            "sign_post_green_hz",
            "parking_only",
            "require_calibrated_parking",
            "allow_preparking_approach",
            "allow_legacy_parking_trigger",
            "start_preparking_lane_adapter",
            "preparking_speed_raw",
            "preparking_lane_state",
            "green_start_speed_raw",
            "green_start_hold_s",
            "competition_mode",
            "start_lane_controller",
            "lane_normal_state",
            "lane_speed_raw",
            "lane_max_steering_raw",
            "lane_command_topic",
        }
        self.assertTrue(expected.issubset(names))

    def test_competition_gate_is_enabled_by_default_and_forwarded(self):
        start_all_args = dict(
            (arg.get("name"), arg.get("default"))
            for arg in self.start_all_root.findall("arg"))
        self.assertEqual(start_all_args["competition_mode"], "true")
        self.assertEqual(start_all_args["green_start_speed_raw"], "20")
        self.assertEqual(start_all_args["green_start_hold_s"], "0.50")

        include = self.root.find("include")
        include_args = dict(
            (arg.get("name"), arg.get("value"))
            for arg in include.findall("arg"))
        self.assertEqual(
            include_args["competition_mode"], "$(arg competition_mode)")
        self.assertEqual(
            include_args["green_start_hold_s"], "$(arg green_start_hold_s)")

    def test_sign_node_throttles_after_confirmed_green(self):
        node = self._node("sign_string_node")
        params = dict(
            (param.get("name"), param.get("value"))
            for param in node.findall("param"))
        self.assertEqual(
            params["post_green_hz"], "$(arg sign_post_green_hz)")

    def test_sign_node_is_cpu_limited_and_uses_bounded_rate(self):
        node = self._node("sign_string_node")
        self.assertIn("--hz $(arg sign_hz)", node.get("args"))
        self.assertEqual(
            node.get("launch-prefix"), "nice -n 15 taskset -c 3")

    def test_start_all_owns_the_white_line_command_adapter(self):
        args = dict(
            (arg.get("name"), arg.get("default"))
            for arg in self.start_all_root.findall("arg"))
        self.assertEqual(args["start_lane_controller"], "true")
        self.assertEqual(args["lane_normal_state"], "LANE_FOLLOWING")
        self.assertEqual(args["lane_command_topic"],
                         "/camera_yihan/receive")
        adapter = None
        for node in self.start_all_root.findall(".//node"):
            if node.get("name") == "lane_main_adapter":
                adapter = node
                break
        self.assertIsNotNone(adapter)
        param_names = set(param.get("name")
                          for param in adapter.findall("param"))
        self.assertTrue({
            "normal_lane_state", "speed_raw", "max_steering_raw",
            "lane_path_topic", "lane_confidence_topic", "output_topic",
        }.issubset(param_names))

    def test_lightweight_vision_launch_isolated_and_defaults_to_speed_40(self):
        args = dict(
            (arg.get("name"), arg.get("default"))
            for arg in self.vision_root.findall("arg"))
        self.assertEqual(args["lane_speed_raw"], "40")
        self.assertEqual(args["green_start_speed_raw"], "40")
        self.assertEqual(args["sign_hz"], "2.0")
        self.assertEqual(args["sign_post_green_hz"], "1.0")

        lane = None
        sign = None
        for node in self.vision_root.findall(".//node"):
            if node.get("name") == "front_lane_metric":
                lane = node
            if node.get("name") == "sign_string_node":
                sign = node
        self.assertIsNotNone(lane)
        self.assertIsNotNone(sign)
        self.assertEqual(lane.get("launch-prefix"), "taskset -c 0-2")
        self.assertEqual(
            sign.get("launch-prefix"), "nice -n 15 taskset -c 3")
        self.assertIn("/camera_hts/send", sign.get("args"))

    def test_metric_nodes_receive_the_declared_topics(self):
        expected_params = {
            "parking_start_line_metric": {
                "role", "image_topic", "input_mode",
                "max_abs_lateral_m",
                "candidate_topic", "mask_topic", "overlay_topic",
            },
            "parking_slot_metric": {
                "role", "image_topic", "metric_mask_topic", "input_mode",
                "max_abs_lateral_m", "horizontal_open_kernel_width",
                "candidate_topic", "mask_topic", "overlay_topic", "slot_id",
            },
            "parking_front_metric": {
                "candidate_topic", "pose_topic", "output_topic",
            },
            "parking_target_selector": {
                "slot_candidate_topic", "start_line_topic", "scan_topic", "sign_topic",
                "output_topic", "preferred_slot_id", "zone_calibrated",
            },
            "parking_rear_metric": {
                "image_topic", "output_topic", "slot_id", "target_topic",
            },
            "parking_exit_metric": {
                "pose_topic", "candidate_topic", "output_topic",
            },
            "parking_observation_node": {
                "front_topic", "rear_topic", "exit_topic", "sign_topic",
                "target_topic", "lidar_topic", "min_source_confidence",
            },
            "front_lane_metric": {
                "image_topic", "metric_bev_topic", "lane_path_topic",
                "lane_confidence_topic",
            },
            "parking_lane_pose": {
                "target_topic", "path_topic", "confidence_topic", "front_output_topic",
                "exit_output_topic", "min_confidence",
            },
            "rear_lane_debug": {"image_topic"},
            "lane_main_adapter": {
                "normal_lane_state", "speed_raw", "max_steering_raw",
                "lane_path_topic", "lane_confidence_topic", "output_topic",
            },
            "parking_session_logger": {
                "front_image_topic", "rear_image_topic", "sign_image_topic",
                "rear_bev_topic", "front_mask_topic", "front_overlay_topic",
                "front_candidate_topic", "front_pose_topic",
                "slot_candidate_topic", "start_line_topic",
                "slot_mask_topic", "slot_overlay_topic",
                "start_line_mask_topic", "start_line_overlay_topic",
                "front_metric_topic", "rear_metric_topic", "exit_pose_topic",
                "exit_metric_topic", "lane_path_topic",
                "lane_confidence_topic",
            },
        }
        for node_name, required in expected_params.items():
            actual = self._param_names(self._node(node_name))
            self.assertTrue(
                required.issubset(actual),
                "%s missing params: %s" % (
                    node_name, sorted(required - actual)),
            )

    def test_camera_topics_match_metric_input_defaults(self):
        args = dict((arg.get("name"), arg.get("default"))
                    for arg in self.root.findall("arg"))
        self.assertEqual(args["front_image_topic"],
                         "/front/usb_cam/image_raw")
        self.assertEqual(args["rear_image_topic"],
                         "/rear/usb_cam/image_raw")
        self.assertEqual(args["sign_image_topic"],
                         "$(arg front_image_topic)")

    def test_sign_node_uses_the_hts_bridge_send_topic(self):
        node = self._node("sign_string_node")
        self.assertIn(
            "--publish-topic $(arg sign_send_topic)",
            node.get("args"),
        )

    def test_rear_metric_receives_shared_visual_profile(self):
        node = self._node("parking_rear_metric")
        profile_files = set(
            item.get("file") for item in node.findall("rosparam"))
        self.assertIn("$(arg visual_profile_file)", profile_files)

    def test_slot_detector_receives_shared_visual_profile_and_slot(self):
        node = self._node("parking_slot_metric")
        profile_files = set(
            item.get("file") for item in node.findall("rosparam"))
        self.assertIn("$(arg visual_profile_file)", profile_files)
        self.assertIn("slot_id", self._param_names(node))

    def test_start_line_detector_cannot_be_used_as_a_slot_detector(self):
        node = self._node("parking_start_line_metric")
        params = self._param_names(node)
        self.assertIn("role", params)
        self.assertNotIn("slot_id", params)
        values = dict((param.get("name"), param.get("value"))
                      for param in node.findall("param"))
        self.assertEqual(values["role"], "start_line")

    def test_lane_pose_has_explicit_live_exit_pose_switch(self):
        names = set(arg.get("name") for arg in self.root.findall("arg"))
        self.assertIn("enable_exit_pose", names)
        self.assertIn("exit_distance_offset_m", names)
        node = self._node("parking_lane_pose")
        params = self._param_names(node)
        self.assertIn("enable_exit_pose", params)
        self.assertIn("exit_distance_offset_m", params)


if __name__ == "__main__":
    unittest.main()
