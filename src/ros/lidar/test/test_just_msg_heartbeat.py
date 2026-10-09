#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Regression tests for the legacy LS01B String heartbeat publisher.

The production module imports rospy at module load time, so these tests load
it with small ROS message/publisher stubs.  The callback itself only needs a
LaserScan-shaped object and numpy; no ROS master or serial device is used.
"""

from __future__ import print_function

import json
import math
import os
import sys
import types
import unittest


SCRIPT = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "scripts", "just_msg.py"))


class FakeString(object):
    def __init__(self):
        self.data = ""


class FakePublisher(object):
    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.messages = []

    def publish(self, message):
        self.messages.append(message.data)


class Scan(object):
    angle_min = -math.pi
    angle_max = math.pi
    range_min = 0.01
    range_max = 25.0

    def __init__(self, ranges):
        self.ranges = list(ranges)


class JustMessageHeartbeatTest(unittest.TestCase):
    def _load(self):
        saved_modules = {}
        module_names = ("rospy", "sensor_msgs", "sensor_msgs.msg",
                        "std_msgs", "std_msgs.msg")
        for name in module_names:
            saved_modules[name] = sys.modules.get(name)

        publishers = []
        fake_rospy = types.ModuleType("rospy")

        def fake_publisher(topic_name, *args, **kwargs):
            publisher = FakePublisher(topic_name)
            publishers.append(publisher)
            return publisher

        fake_rospy.Publisher = fake_publisher
        fake_rospy.loginfo_throttle = lambda *args, **kwargs: None

        fake_sensor = types.ModuleType("sensor_msgs")
        fake_sensor_msg = types.ModuleType("sensor_msgs.msg")
        fake_sensor_msg.LaserScan = Scan
        fake_sensor.msg = fake_sensor_msg

        fake_std = types.ModuleType("std_msgs")
        fake_std_msg = types.ModuleType("std_msgs.msg")
        fake_std_msg.String = FakeString
        fake_std.msg = fake_std_msg

        sys.modules["rospy"] = fake_rospy
        sys.modules["sensor_msgs"] = fake_sensor
        sys.modules["sensor_msgs.msg"] = fake_sensor_msg
        sys.modules["std_msgs"] = fake_std
        sys.modules["std_msgs.msg"] = fake_std_msg
        try:
            # Execute the file directly instead of using Python 2 runpy.  The
            # latter may leave a successfully imported ``numpy`` binding as
            # None when the script is loaded outside a real package context.
            namespace = {
                "__name__": "just_msg_test",
                "__file__": SCRIPT,
            }
            with open(SCRIPT, "r") as stream:
                source = stream.read()
            exec(compile(source, SCRIPT, "exec"), namespace)
            return namespace, publishers[0]
        finally:
            for name, old_module in saved_modules.items():
                if old_module is None:
                    sys.modules.pop(name, None)
                else:
                    sys.modules[name] = old_module

    @staticmethod
    def _payload(publisher):
        assert publisher.messages
        return json.loads(publisher.messages[-1])

    def test_far_valid_scan_publishes_clear_heartbeat(self):
        namespace, publisher = self._load()
        namespace["lidar_callback"](Scan([3.0, 8.0, 12.0]))
        payload = self._payload(publisher)
        self.assertTrue(payload["valid"])
        self.assertEqual(payload["distance"], 2.0)

    def test_near_valid_scan_keeps_nearest_distance(self):
        namespace, publisher = self._load()
        namespace["lidar_callback"](Scan([1.0, 0.20, 0.8]))
        payload = self._payload(publisher)
        self.assertTrue(payload["valid"])
        self.assertEqual(payload["distance"], 0.2)

    def test_all_invalid_scan_stays_invalid(self):
        namespace, publisher = self._load()
        namespace["lidar_callback"](Scan([float("inf"), float("nan"), 0.01]))
        payload = self._payload(publisher)
        self.assertFalse(payload["valid"])
        self.assertIsNone(payload["distance"])


if __name__ == "__main__":
    unittest.main()
