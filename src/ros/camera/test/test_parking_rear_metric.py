# -*- coding: utf-8 -*-

"""Regression tests for the explicit P5 single-side rear geometry mode."""

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
    rospy = types.ModuleType("rospy")
    rospy.get_param = lambda _name, default=None: default
    rospy.has_param = lambda _name: False
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


class RearParkingMetricTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from vision_test_support import preserve_modules
        restore = preserve_modules()
        try:
            _install_ros_stubs()
            import parking_rear_metric
            cls.module = parking_rear_metric
        finally:
            restore()

    @staticmethod
    def p5_t_shape():
        image = np.zeros((400, 480, 3), dtype=np.uint8)
        blue = (255, 0, 0)  # BGR, hue 120
        # One stable left boundary and the transverse stop line.  The right
        # boundary is intentionally absent, matching the P5 field frame.
        cv2.line(image, (100, 20), (100, 260), blue, 8)
        cv2.line(image, (110, 260), (350, 260), blue, 8)
        return image

    def test_default_pair_mode_rejects_single_side_geometry(self):
        settings = dict(self.module.RearParkingDetector.DEFAULTS)
        settings["line_color"] = "blue"
        detector = self.module.RearParkingDetector(settings)
        result, _overlay = detector.detect(self.p5_t_shape(), stamp=1.0)

        self.assertFalse(result["valid"])
        self.assertFalse(result["rear_visible"])

    def test_explicit_single_side_mode_recovers_p5_pose(self):
        settings = dict(self.module.RearParkingDetector.DEFAULTS)
        settings.update({
            "line_color": "blue",
            "single_side_mode": True,
            "single_side_lateral_sign": 1.0,
        })
        detector = self.module.RearParkingDetector(settings)
        result, overlay = detector.detect(self.p5_t_shape(), stamp=1.0)

        self.assertIsNotNone(overlay)
        self.assertTrue(result["valid"])
        self.assertTrue(result["rear_visible"])
        self.assertEqual(result["geometry_mode"], "single_side")
        self.assertAlmostEqual(result["rear_distance_m"], 0.35, delta=0.04)
        self.assertAlmostEqual(result["lateral_error_m"], 0.16, delta=0.06)
        self.assertAlmostEqual(result["vehicle_yaw_error_rad"], 0.0, delta=0.08)
        self.assertGreaterEqual(result["confidence"], 0.55)


if __name__ == "__main__":
    unittest.main()
