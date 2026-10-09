# -*- coding: utf-8 -*-

"""Regression tests for the motion-free field-capture payload boundary."""

from __future__ import print_function

import os
import sys
import types
import unittest

try:
    import importlib.util
except ImportError:
    import imp


SCRIPT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts"))


def _load_module():
    rospy = types.ModuleType("rospy")
    sys.modules["rospy"] = rospy
    cv2 = types.ModuleType("cv2")
    sys.modules["cv2"] = cv2
    yaml = types.ModuleType("yaml")
    sys.modules["yaml"] = yaml
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

    rear_path = os.path.join(SCRIPT_DIR, "parking_rear_metric.py")
    if sys.version_info[0] >= 3:
        rear_spec = importlib.util.spec_from_file_location(
            "parking_rear_metric_capture_test", rear_path)
        rear_module = importlib.util.module_from_spec(rear_spec)
        rear_spec.loader.exec_module(rear_module)
    else:
        rear_module = imp.load_source(
            "parking_rear_metric_capture_test", rear_path)
    sys.modules["parking_rear_metric"] = rear_module

    path = os.path.join(SCRIPT_DIR, "parking_field_capture.py")
    if sys.version_info[0] >= 3:
        spec = importlib.util.spec_from_file_location(
            "parking_field_capture_test", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    else:
        module = imp.load_source("parking_field_capture_test", path)
    return module


class ParkingFieldCaptureTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from vision_test_support import preserve_modules
        restore = preserve_modules()
        try:
            cls.module = _load_module()
        finally:
            restore()

    def test_decode_payload_accepts_mapping_only(self):
        self.assertEqual(
            self.module.decode_payload('{"schema":"parking_observation_v1"}')
            ["schema"],
            "parking_observation_v1")
        self.assertIsNone(self.module.decode_payload("not-json"))
        self.assertIsNone(self.module.decode_payload("[]"))

    def test_capture_has_no_command_topic(self):
        self.assertEqual(self.module.IMAGE_TOPIC_DEFAULTS["rear_bev"],
                         "/debug/rear_bev")
        self.assertEqual(
            self.module.IMAGE_TOPIC_DEFAULTS["front_overlay"],
            "/debug/parking_slot_overlay")
        self.assertEqual(
            self.module.IMAGE_TOPIC_DEFAULTS["front_mask"],
            "/debug/parking_slot_mask")
        self.assertEqual(self.module.IMAGE_TOPIC_DEFAULTS['start_line_mask'],
                         '/debug/parking_start_line_mask')
        self.assertNotIn("command", self.module.STRING_TOPIC_DEFAULTS)
        self.assertNotIn("control", self.module.STRING_TOPIC_DEFAULTS)


if __name__ == "__main__":
    unittest.main()
