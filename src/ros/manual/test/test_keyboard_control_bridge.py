# -*- coding: utf-8 -*-

from __future__ import division

import imp
import json
import os
import sys
import time
import types
import unittest


class _FakePublisher(object):
    def __init__(self, *args, **kwargs):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _FakeSubscriber(object):
    pass


class _Namespace(object):
    def __init__(self, **values):
        self.__dict__.update(values)


class _FakeDrive(object):
    def __init__(self):
        self.speed = 0.0
        self.steering_angle = 0.0


class _FakeAckermannDriveStamped(object):
    def __init__(self):
        self.header = _Namespace(stamp=None)
        self.drive = _FakeDrive()


class KeyboardControlBridgeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy = types.ModuleType("rospy")
        rospy.get_param = lambda name, default=None: default
        rospy.Publisher = _FakePublisher
        rospy.Subscriber = lambda *args, **kwargs: _FakeSubscriber()
        rospy.on_shutdown = lambda callback: None
        rospy.loginfo = lambda *args, **kwargs: None
        rospy.logwarn = lambda *args, **kwargs: None
        rospy.logwarn_throttle = lambda *args, **kwargs: None
        rospy.Time = _Namespace(now=lambda: _Namespace(to_sec=lambda: 10.0))

        ackermann_msgs = types.ModuleType("ackermann_msgs")
        ackermann_msg = types.ModuleType("ackermann_msgs.msg")
        ackermann_msg.AckermannDriveStamped = _FakeAckermannDriveStamped
        ackermann_msgs.msg = ackermann_msg

        std_msgs = types.ModuleType("std_msgs")
        std_msg = types.ModuleType("std_msgs.msg")
        std_msg.String = lambda data="": _Namespace(data=data)
        std_msgs.msg = std_msg

        cls._saved_modules = {
            name: sys.modules.get(name)
            for name in ("rospy", "ackermann_msgs", "ackermann_msgs.msg",
                         "std_msgs", "std_msgs.msg")
        }
        sys.modules["rospy"] = rospy
        sys.modules["ackermann_msgs"] = ackermann_msgs
        sys.modules["ackermann_msgs.msg"] = ackermann_msg
        sys.modules["std_msgs"] = std_msgs
        sys.modules["std_msgs.msg"] = std_msg

        path = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "scripts", "testloop.py"))
        cls.module = imp.load_source("parking_testloop_under_test", path)

    @classmethod
    def tearDownClass(cls):
        for name, module in cls._saved_modules.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module

    @staticmethod
    def message(enabled, seq, speed, steering):
        return _Namespace(data=json.dumps({
            "version": 1,
            "seq": seq,
            "enabled": enabled,
            "speed_raw": speed,
            "steering_raw": steering,
        }))

    def setUp(self):
        self.bridge = self.module.AckermannControlBridge()

    def test_joystick_uses_bridge_and_keyboard_has_priority(self):
        self.bridge.control_callback(self.message(True, 1, 4, 2))
        self.bridge.joystick_control_callback(self.message(True, 1, -5, 6))
        self.assertEqual(self.bridge.current_command(), (-5, 6))
        self.bridge.keyboard_control_callback(self.message(True, 1, 0, 0))
        self.assertEqual(self.bridge.current_command(), (0, 0))

    def test_joystick_disconnect_latches_stop(self):
        self.bridge.control_callback(self.message(True, 1, 4, 2))
        self.bridge.joystick_control_callback(self.message(True, 1, 5, 6))
        self.bridge.joystick_last_valid_time = time.time() - 1
        self.assertEqual(self.bridge.current_command(), (0, 0))
        self.assertEqual(self.bridge.current_command(), (0, 0))
        self.bridge.joystick_control_callback(self.message(False, 2, 0, 0))
        self.assertEqual(self.bridge.current_command(), (4, 2))

    def test_malformed_joystick_stops_automatic_output(self):
        self.bridge.control_callback(self.message(True, 1, 4, 2))
        self.bridge.joystick_control_callback(_Namespace(data='{}'))
        self.assertEqual(self.bridge.current_command(), (0, 0))

    def test_production_requires_fresh_source_stamp(self):
        self.bridge.require_stamp = True
        msg = self.message(True, 1, 12, 0)
        for stamp in (None, 9.0, 11.0, float('nan')):
            data = json.loads(msg.data)
            data['stamp'] = stamp
            self.bridge.control_callback(_Namespace(data=json.dumps(data)))
            self.assertEqual(self.bridge.current_command(), (0,0))

    def test_replayed_source_stamp_cannot_refresh_watchdog(self):
        self.bridge.require_stamp = True
        data = json.loads(self.message(True, 1, 12, 0).data)
        data['stamp'] = 9.9
        msg = _Namespace(data=json.dumps(data))
        self.bridge.control_callback(msg)
        self.assertEqual(self.bridge.current_command(), (12,0))
        self.bridge.control_callback(msg)
        self.assertEqual(self.bridge.current_command(), (0,0))

    def test_manual_channels_have_independent_stamp_order(self):
        self.bridge.require_stamp = True
        data = json.loads(self.message(True, 1, 12, 0).data)
        data['stamp'] = 9.9
        msg = _Namespace(data=json.dumps(data))
        self.bridge.joystick_control_callback(msg)
        self.bridge.keyboard_control_callback(msg)
        self.assertEqual(self.bridge.current_command(), (12,0))

    def test_keyboard_overrides_auto_command(self):
        self.bridge.control_callback(self.message(
            True, 1, 4, 2))
        self.assertEqual(self.bridge.current_command(), (4, 2))

        self.bridge.keyboard_control_callback(self.message(
            True, 2, -3, 7))
        self.assertEqual(self.bridge.current_command(), (-3, 7))

    def test_keyboard_timeout_latches_stop_until_explicit_release(self):
        self.bridge.control_callback(self.message(
            True, 1, 4, 2))
        self.bridge.keyboard_control_callback(self.message(
            True, 2, 3, -5))
        self.bridge.keyboard_last_valid_time = time.time() - 1.0

        self.assertEqual(self.bridge.current_command(), (0, 0))
        self.assertTrue(self.bridge.keyboard_fault)
        self.assertEqual(self.bridge.current_command(), (0, 0))

        self.bridge.keyboard_control_callback(self.message(
            False, 3, 0, 0))
        self.assertEqual(self.bridge.current_command(), (4, 2))

    def test_invalid_keyboard_payload_stops(self):
        self.bridge.control_callback(self.message(
            True, 1, 4, 2))
        self.bridge.keyboard_control_callback(
            _Namespace(data=json.dumps({
                "version": 1,
                "seq": 2,
                "speed_raw": 3,
                "steering_raw": 1,
            })))
        self.assertEqual(self.bridge.current_command(), (0, 0))


if __name__ == "__main__":
    unittest.main()
