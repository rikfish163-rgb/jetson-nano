# -*- coding: utf-8 -*-

"""Regression tests for per-slot visual calibration selection.

The production nodes are ROS wrappers, so this test supplies tiny module
stubs and instantiates their configuration path without starting ROS, cameras,
or a vehicle.  It protects the P4/P5 mapping boundary from silently falling
back to one global affine transform.
"""

from __future__ import print_function

import os
import sys
import types
import unittest

import yaml

try:
    import importlib.util
except ImportError:
    import imp


SCRIPT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts"))
PROFILE_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "config",
                 "parking_visual_profiles.yaml"))


class _Publisher(object):
    def publish(self, _message):
        pass


class _RospyStub(types.ModuleType):
    def __init__(self, parameters):
        super(_RospyStub, self).__init__("rospy")
        self.parameters = parameters

    def get_param(self, name, default=None):
        return self.parameters.get(name, default)

    def Publisher(self, *_args, **_kwargs):
        return _Publisher()

    def Subscriber(self, *_args, **_kwargs):
        return object()

    def Timer(self, *_args, **_kwargs):
        return object()

    def Duration(self, value):
        return value

    def is_shutdown(self):
        return False

    def loginfo(self, *_args, **_kwargs):
        pass


def _install_message_stubs():
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = type("String", (object,), {})
    std_msgs_msg.Float32 = type("Float32", (object,), {})
    std_msgs.msg = std_msgs_msg
    nav_msgs = types.ModuleType("nav_msgs")
    nav_msgs_msg = types.ModuleType("nav_msgs.msg")
    nav_msgs_msg.Path = type("Path", (object,), {})
    nav_msgs.msg = nav_msgs_msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg
    sys.modules["nav_msgs"] = nav_msgs
    sys.modules["nav_msgs.msg"] = nav_msgs_msg


def _load_module(name, filename):
    path = os.path.join(SCRIPT_DIR, filename)
    if sys.version_info[0] >= 3:
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    return imp.load_source(name, path)


class ParkingVisualProfilesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profiles = {
            "P4": {
                "lane_yaw_sign": -1.0,
                "lane_slot_yaw_bias_rad": 0.11,
                "lane_slot_lateral_offset_m": 0.21,
                "candidate_yaw_gain": 1.2,
                "candidate_yaw_bias_rad": 0.12,
                "candidate_lateral_gain": 0.8,
                "candidate_lateral_bias_m": 0.03,
                "exit_candidate_yaw_gain": 0.7,
                "exit_candidate_yaw_bias_rad": -0.08,
                "exit_candidate_lateral_gain": 1.1,
                "exit_candidate_lateral_bias_m": -0.02,
                "rear_expected_slot_width_m": 0.36,
                "rear_vehicle_center_x_m": 0.02,
                "rear_reference_distance_m": 0.15,
                "rear_stop_line_bias_m": -0.03,
                "rear_yaw_sign": -1.0,
                "rear_line_color": "white",
            },
            "P5": {
                "lane_yaw_sign": -1.0,
                "lane_slot_yaw_bias_rad": -0.22,
                "lane_slot_lateral_offset_m": -0.31,
                "candidate_yaw_gain": 1.4,
                "candidate_yaw_bias_rad": -0.19,
                "candidate_lateral_gain": 0.6,
                "candidate_lateral_bias_m": -0.04,
                "exit_candidate_yaw_gain": 0.9,
                "exit_candidate_yaw_bias_rad": 0.16,
                "exit_candidate_lateral_gain": 1.3,
                "exit_candidate_lateral_bias_m": 0.05,
                "rear_expected_slot_width_m": 0.39,
                "rear_vehicle_center_x_m": -0.01,
                "rear_reference_distance_m": 0.10,
                "rear_stop_line_bias_m": 0.02,
                "rear_yaw_sign": 1.0,
                "rear_line_color": "blue",
                "rear_single_side_mode": True,
                "rear_single_side_lateral_sign": 1.0,
            },
        }

    def setUp(self):
        from vision_test_support import preserve_modules
        self.addCleanup(preserve_modules())
        _install_message_stubs()

    def _params(self, slot_id):
        return {
            "~slot_id": slot_id,
            "~slot_profiles": self.profiles,
        }

    def test_lane_pose_selects_requested_slot(self):
        sys.modules["rospy"] = _RospyStub(self._params("P5"))
        module = _load_module("parking_lane_pose_profile_test", "parking_lane_pose.py")
        node = module.ParkingLanePoseNode()
        self.assertEqual(node.yaw_sign, -1.0)
        self.assertEqual(node.slot_yaw_bias_rad, -0.22)
        self.assertEqual(node.slot_lateral_offset_m, -0.31)

    def test_deployed_p5_forward_right_yaw_sign_is_negative(self):
        with open(PROFILE_PATH, "r") as stream:
            profiles = yaml.safe_load(stream)["slot_profiles"]
        self.assertEqual(profiles["P5"]["lane_yaw_sign"], -1.0)
        # The parking bay is defined by white bay lines.  Blue is a separate
        # start-line trigger and must never be used as the rear slot geometry.
        self.assertEqual(profiles["P5"]["rear_line_color"], "white")
        self.assertEqual(profiles["P4"]["front_candidate_order"], "farthest")
        self.assertEqual(profiles["P4"]["front_min_candidate_count"], 2)
        self.assertEqual(profiles["P5"]["front_candidate_order"], "nearest")
        self.assertEqual(profiles["P5"]["front_min_candidate_count"], 1)

    def test_front_affine_selects_requested_slot(self):
        sys.modules["rospy"] = _RospyStub(self._params("P5"))
        module = _load_module(
            "parking_front_metric_profile_test", "parking_front_metric.py")
        node = module.ParkingFrontMetricNode()
        self.assertEqual(node.pose_yaw_gain, 1.4)
        self.assertEqual(node.pose_yaw_bias_rad, -0.19)
        self.assertEqual(node.pose_lateral_gain, 0.6)
        self.assertEqual(node.pose_lateral_bias_m, -0.04)

    def test_exit_affine_selects_requested_slot(self):
        sys.modules["rospy"] = _RospyStub(self._params("P5"))
        module = _load_module(
            "parking_exit_metric_profile_test", "parking_exit_metric.py")
        node = module.ParkingExitMetricNode()
        self.assertEqual(node.candidate_yaw_gain, 0.9)
        self.assertEqual(node.candidate_yaw_bias_rad, 0.16)
        self.assertEqual(node.candidate_lateral_gain, 1.3)
        self.assertEqual(node.candidate_lateral_bias_m, 0.05)

    def test_rear_bev_selects_requested_slot(self):
        # Profile selection is pure; keep this test runnable without ROS or
        # OpenCV by supplying import-time stubs for the wrapper dependencies.
        sys.modules["cv2"] = types.ModuleType("cv2")
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
        sys.modules["rospy"] = _RospyStub(self._params("P5"))
        module = _load_module(
            "parking_rear_metric_profile_test", "parking_rear_metric.py")
        settings = module.apply_rear_visual_profile(
            module.RearParkingDetector.DEFAULTS,
            self.profiles["P5"],
        )
        self.assertEqual(settings["expected_slot_width_m"], 0.39)
        self.assertEqual(settings["vehicle_center_x_m"], -0.01)
        self.assertEqual(settings["rear_reference_distance_m"], 0.10)
        self.assertEqual(settings["stop_line_bias_m"], 0.02)
        self.assertEqual(settings["yaw_sign"], 1.0)
        self.assertEqual(settings["line_color"], "blue")
        self.assertTrue(settings["single_side_mode"])
        self.assertEqual(settings["single_side_lateral_sign"], 1.0)


if __name__ == "__main__":
    unittest.main()
