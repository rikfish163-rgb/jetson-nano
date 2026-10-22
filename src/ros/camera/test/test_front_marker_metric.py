#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Regression tests for the metric-white P4/P5 approach marker."""

from __future__ import print_function

import os
import sys
import types
import unittest

import cv2
import numpy as np


SCRIPT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)


def _install_ros_stubs():
    """Keep the pure detector tests runnable without a ROS installation."""
    rospy = types.ModuleType("rospy")
    sys.modules["rospy"] = rospy

    cv_bridge = types.ModuleType("cv_bridge")
    cv_bridge.CvBridge = type("CvBridge", (object,), {})
    cv_bridge.CvBridgeError = type("CvBridgeError", (Exception,), {})
    sys.modules["cv_bridge"] = cv_bridge

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.Image = type("Image", (object,), {})
    sensor_msgs.msg = sensor_msgs_msg
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = type("String", (object,), {})
    std_msgs.msg = std_msgs_msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg


from vision_test_support import preserve_modules
_restore_modules = preserve_modules()
_install_ros_stubs()

import blue_marker_metric as marker
_restore_modules()


class FrontMarkerMetricTest(unittest.TestCase):
    @staticmethod
    def _component(distance, area):
        return {
            "distance_m": float(distance),
            "area_px": int(area),
        }

    def test_requested_slot_selects_near_or_far_candidate(self):
        candidates = [
            self._component(0.65, 2200),
            self._component(0.90, 1200),
        ]
        profiles = {
            "P5": {
                "front_candidate_order": "nearest",
                "front_min_candidate_count": 1,
            },
            "P4": {
                "front_candidate_order": "farthest",
                "front_min_candidate_count": 2,
            },
        }

        selected, mode, ready = marker.select_target_component(
            candidates, "P5", profiles)
        self.assertEqual(mode, "nearest")
        self.assertTrue(ready)
        self.assertAlmostEqual(selected["distance_m"], 0.65)

        selected, mode, ready = marker.select_target_component(
            candidates, "P4", profiles)
        self.assertEqual(mode, "farthest")
        self.assertTrue(ready)
        self.assertAlmostEqual(selected["distance_m"], 0.90)

    def test_far_slot_requires_both_visible_candidates(self):
        profiles = {
            "P4": {
                "front_candidate_order": "farthest",
                "front_min_candidate_count": 2,
            },
        }
        selected, mode, ready = marker.select_target_component(
            [self._component(0.90, 1200)], "P4", profiles)
        self.assertIsNone(selected)
        self.assertEqual(mode, "farthest")
        self.assertFalse(ready)

    def test_horizontal_white_slot_edge_is_a_metric_candidate(self):
        metric_mask = np.zeros(
            (marker.BEV_HEIGHT, marker.BEV_WIDTH), dtype=np.uint8)
        # Representative P5 near edge from the 2026-09-05 field frame:
        # a horizontal 0.15 m segment around x=0.40 m left of base_link.
        cv2.rectangle(metric_mask, (47, 236), (107, 246), 255, -1)
        # A longitudinal lane boundary must not merge into the slot edge.
        cv2.rectangle(metric_mask, (125, 80), (134, 350), 255, -1)

        horizontal = marker.build_horizontal_metric_mask(
            metric_mask, kernel_width=21, kernel_height=3)
        legal, selected = marker.analyze_components(
            horizontal, max_abs_lateral_m=0.50)

        self.assertIsNotNone(selected)
        self.assertEqual(len(legal), 1)
        self.assertAlmostEqual(selected["distance_m"], 0.8975, places=2)
        self.assertAlmostEqual(selected["lateral_m"], 0.4075, places=2)
        self.assertLess(abs(selected["angle_deg"]), 1.0)

    def test_longitudinal_lane_boundary_is_removed(self):
        metric_mask = np.zeros(
            (marker.BEV_HEIGHT, marker.BEV_WIDTH), dtype=np.uint8)
        cv2.rectangle(metric_mask, (125, 80), (134, 350), 255, -1)

        horizontal = marker.build_horizontal_metric_mask(
            metric_mask, kernel_width=21, kernel_height=3)
        legal, selected = marker.analyze_components(
            horizontal, max_abs_lateral_m=0.50)

        self.assertEqual(legal, [])
        self.assertIsNone(selected)


if __name__ == "__main__":
    unittest.main()
