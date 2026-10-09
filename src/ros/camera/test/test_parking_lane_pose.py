# -*- coding: utf-8 -*-

from __future__ import print_function

import json
import os
import sys
import time
import types
import unittest


SCRIPT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)


class _Publisher(object):
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _RospyStub(types.ModuleType):
    def __init__(self, parameters):
        super(_RospyStub, self).__init__("rospy")
        self.parameters = parameters
        self.publishers = []

    def get_param(self, name, default=None):
        return self.parameters.get(name, default)

    def Publisher(self, *_args, **_kwargs):
        publisher = _Publisher()
        self.publishers.append(publisher)
        return publisher

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
    std_msgs_msg.Float32 = type("Float32", (object,), {})
    class String(object):
        def __init__(self, data=None):
            self.data = data
    std_msgs_msg.String = String
    std_msgs.msg = std_msgs_msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg

    nav_msgs = types.ModuleType("nav_msgs")
    nav_msgs_msg = types.ModuleType("nav_msgs.msg")
    nav_msgs_msg.Path = type("Path", (object,), {})
    nav_msgs.msg = nav_msgs_msg
    sys.modules["nav_msgs"] = nav_msgs
    sys.modules["nav_msgs.msg"] = nav_msgs_msg


class ParkingLanePoseTest(unittest.TestCase):
    def setUp(self):
        from vision_test_support import preserve_modules
        self.addCleanup(preserve_modules())

    def test_exit_distance_comes_from_live_lane_path_when_not_fixed(self):
        _install_message_stubs()
        rospy = _RospyStub({
            "~slot_id": "P5",
            "~slot_profiles": {"P5": {}},
            "~enable_front_pose": True,
            "~enable_exit_pose": True,
            "~min_confidence": 0.45,
            "~input_timeout_s": 1.0,
            "~exit_distance_offset_m": 0.0,
        })
        sys.modules["rospy"] = rospy

        import parking_lane_pose

        node = parking_lane_pose.ParkingLanePoseNode()
        now = time.time()
        node.path = [(0.30, 0.02), (0.60, 0.08), (0.90, 0.14)]
        node.path_received_at = now
        node.confidence = 0.90
        node.confidence_received_at = now
        node._publish(None)

        self.assertEqual(len(node.exit_pub.messages), 1)
        result = json.loads(node.exit_pub.messages[-1].data)
        self.assertTrue(result["valid"])
        self.assertEqual(result["exit_distance_source"], "lane_path_min_x")
        self.assertAlmostEqual(result["exit_distance_m"], 0.30, places=2)
        self.assertEqual(result["slot_id"], "P5")


if __name__ == "__main__":
    unittest.main()
