#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import json
import math
import threading
import time

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String


try:
    integer_types = (int, long)
except NameError:
    integer_types = (int,)


class AckermannControlBridge(object):
    """Validate central-control JSON and publish commands for the chassis."""

    def __init__(self):
        self.control_topic = rospy.get_param("~control_topic", "/control/cmd")
        self.keyboard_control_topic = rospy.get_param(
            "~keyboard_control_topic", "/keyboard/control_cmd")
        self.command_topic = rospy.get_param("~command_topic", "/ackermann_cmd")
        self.publish_rate = float(rospy.get_param("~publish_rate", 20.0))
        self.command_timeout = float(rospy.get_param("~command_timeout", 0.25))
        self.require_stamp = rospy.get_param('~require_stamp', False)
        if type(self.require_stamp) is not bool:
            raise ValueError('require_stamp must be boolean')
        self.keyboard_timeout = float(
            rospy.get_param("~keyboard_timeout", 0.30))
        self.max_speed_raw = int(rospy.get_param("~max_speed_raw", 100))
        self.max_steering_raw = int(rospy.get_param("~max_steering_raw", 22))

        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate must be greater than zero")
        if self.command_timeout <= 0.0:
            raise ValueError("command_timeout must be greater than zero")
        if self.keyboard_timeout <= 0.0:
            raise ValueError("keyboard_timeout must be greater than zero")
        if self.max_speed_raw < 0 or self.max_steering_raw < 0:
            raise ValueError("raw limits must not be negative")

        self.lock = threading.Lock()
        self.source_stamps = {}
        self.speed_raw = 0
        self.steering_raw = 0
        self.last_sequence = None
        self.last_valid_time = None
        self.timeout_active = True
        self.keyboard_speed_raw = 0
        self.keyboard_steering_raw = 0
        self.keyboard_last_valid_time = None
        self.keyboard_override = False
        self.keyboard_fault = False
        self.keyboard_timeout_active = False

        self.joystick_command = (0, 0)
        self.joystick_last_valid_time = None
        self.joystick_override = False
        self.joystick_fault = False

        self.command_pub = rospy.Publisher(
            self.command_topic, AckermannDriveStamped, queue_size=1)
        self.control_sub = rospy.Subscriber(
            self.control_topic, String, self.control_callback, queue_size=1)
        self.keyboard_control_sub = rospy.Subscriber(
            self.keyboard_control_topic,
            String,
            self.keyboard_control_callback,
            queue_size=1)
        rospy.on_shutdown(self.publish_stop)
        self.joystick_sub = rospy.Subscriber('/joystick/control_cmd', String,
            self.joystick_control_callback, queue_size=1)

        rospy.loginfo(
            "Control bridge ready: %s -> %s, keyboard=%s, rate=%.1f Hz, "
            "timeout=%.3f s, keyboard_timeout=%.3f s",
            self.control_topic,
            self.command_topic,
            self.keyboard_control_topic,
            self.publish_rate,
            self.command_timeout,
            self.keyboard_timeout)

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    @staticmethod
    def require_integer(data, field):
        value = data.get(field)
        if isinstance(value, bool) or not isinstance(value, integer_types):
            raise ValueError("%s must be an integer" % field)
        return int(value)

    def decode_command(self, raw_message, source=None):
        if len(raw_message) > 4096:
            raise ValueError('command message too large')
        try:
            data = json.loads(raw_message)
        except (TypeError, ValueError) as exc:
            raise ValueError("invalid JSON: %s" % exc)

        if not isinstance(data, dict):
            raise ValueError("JSON root must be an object")

        if self.require_stamp or 'stamp' in data:
            stamp = data.get('stamp')
            if (isinstance(stamp, bool) or not isinstance(stamp, integer_types+(float,)) or
                    math.isnan(stamp) or math.isinf(stamp) or
                    not 0 <= rospy.Time.now().to_sec()-stamp <= self.command_timeout):
                raise ValueError('command source stamp stale, future or missing')

        version = self.require_integer(data, "version")
        sequence = self.require_integer(data, "seq")
        speed_raw = self.require_integer(data, "speed_raw")
        steering_raw = self.require_integer(data, "steering_raw")

        if version != 1:
            raise ValueError("unsupported protocol version: %d" % version)
        if sequence < 0 or sequence > 255:
            raise ValueError("seq must be in the range 0..255")

        limited_speed = self.clamp(
            speed_raw, -self.max_speed_raw, self.max_speed_raw)
        limited_steering = self.clamp(
            steering_raw, -self.max_steering_raw, self.max_steering_raw)

        if limited_speed != speed_raw or limited_steering != steering_raw:
            rospy.logwarn_throttle(
                1.0,
                "Control command was limited: speed %d->%d, steering %d->%d" %
                (speed_raw, limited_speed, steering_raw, limited_steering))

        if source is not None and 'stamp' in data:
            with self.lock:
                if data['stamp'] <= self.source_stamps.get(source, -1):
                    raise ValueError('duplicate or out-of-order source stamp')
                self.source_stamps[source] = data['stamp']
        return sequence, limited_speed, limited_steering

    def decode_keyboard_command(self, raw_message, source='keyboard'):
        """Decode a keyboard command with an explicit enable bit."""
        try:
            data = json.loads(raw_message)
        except (TypeError, ValueError) as exc:
            raise ValueError("invalid keyboard JSON: %s" % exc)
        if not isinstance(data, dict):
            raise ValueError("keyboard JSON root must be an object")
        enabled = data.get("enabled")
        if not isinstance(enabled, bool):
            raise ValueError("keyboard enabled must be boolean")
        sequence, speed_raw, steering_raw = self.decode_command(raw_message, source)
        return enabled, sequence, speed_raw, steering_raw

    def control_callback(self, msg):
        try:
            sequence, speed_raw, steering_raw = self.decode_command(msg.data, 'auto')
        except (KeyError, TypeError, ValueError) as exc:
            with self.lock:
                self.speed_raw = 0
                self.steering_raw = 0
                self.last_valid_time = None
                self.timeout_active = True
            rospy.logwarn_throttle(1.0, "Rejected control command: %s" % exc)
            return

        with self.lock:
            self.speed_raw = speed_raw
            self.steering_raw = steering_raw
            self.last_sequence = sequence
            self.last_valid_time = time.time()
            self.timeout_active = False

    def keyboard_control_callback(self, msg):
        try:
            enabled, sequence, speed_raw, steering_raw = \
                self.decode_keyboard_command(msg.data)
        except (KeyError, TypeError, ValueError) as exc:
            with self.lock:
                self.keyboard_speed_raw = 0
                self.keyboard_steering_raw = 0
                self.keyboard_last_valid_time = None
                self.keyboard_override = False
                self.keyboard_fault = True
                self.keyboard_timeout_active = True
            rospy.logwarn_throttle(
                1.0, "Rejected keyboard control command: %s" % exc)
            return

        with self.lock:
            if not enabled:
                # Explicit release is required after a keyboard timeout.  It
                # is safer than silently returning to autonomous motion after
                # the terminal or SSH session disappears.
                self.keyboard_speed_raw = 0
                self.keyboard_steering_raw = 0
                self.keyboard_last_valid_time = None
                self.keyboard_override = False
                self.keyboard_fault = False
                self.keyboard_timeout_active = False
                return
            self.keyboard_speed_raw = speed_raw
            self.keyboard_steering_raw = steering_raw
            self.keyboard_last_valid_time = time.time()
            self.keyboard_override = True
            self.keyboard_fault = False
            self.keyboard_timeout_active = False

    def joystick_control_callback(self, msg):
        try:
            enabled, sequence, speed, steer = self.decode_keyboard_command(msg.data, 'joystick')
        except (KeyError, TypeError, ValueError):
            with self.lock:
                self.joystick_fault = True
                self.joystick_command = (0, 0)
            return
        with self.lock:
            self.joystick_override = enabled
            self.joystick_fault = False
            self.joystick_command = (speed, steer) if enabled else (0, 0)
            self.joystick_last_valid_time = time.time() if enabled else None

    def current_command(self):
        now = time.time()
        with self.lock:
            if self.keyboard_fault:
                return 0, 0
            if self.keyboard_override:
                if (self.keyboard_last_valid_time is None or
                        not 0 <= now - self.keyboard_last_valid_time <= self.keyboard_timeout):
                    self.keyboard_speed_raw = 0
                    self.keyboard_steering_raw = 0
                    self.keyboard_last_valid_time = None
                    self.keyboard_override = False
                    self.keyboard_fault = True
                    if not self.keyboard_timeout_active:
                        rospy.logwarn(
                            "Keyboard command timed out after %.3f s; "
                            "stopping vehicle",
                            self.keyboard_timeout)
                    self.keyboard_timeout_active = True
                    return 0, 0
                return self.keyboard_speed_raw, self.keyboard_steering_raw
            if self.joystick_override and (self.joystick_last_valid_time is None or
                    not 0 <= now-self.joystick_last_valid_time <= self.keyboard_timeout):
                self.joystick_fault = True
            if self.joystick_fault:
                return 0, 0
            if self.joystick_override:
                return self.joystick_command
            if (self.last_valid_time is None or
                    not 0 <= now - self.last_valid_time <= self.command_timeout):
                self.speed_raw = 0
                self.steering_raw = 0
                if not self.timeout_active:
                    rospy.logwarn(
                        "Central command timed out after %.3f s; stopping vehicle",
                        self.command_timeout)
                self.timeout_active = True
            return self.speed_raw, self.steering_raw

    def publish_command(self, speed_raw, steering_raw):
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.drive.speed = float(speed_raw)
        msg.drive.steering_angle = float(steering_raw)
        self.command_pub.publish(msg)

    def publish_stop(self):
        self.publish_command(0, 0)

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            speed_raw, steering_raw = self.current_command()
            self.publish_command(speed_raw, steering_raw)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("ackermann_control_bridge")
    import rosgraph
    publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
    topic = rospy.get_param('~command_topic', '/ackermann_cmd')
    if dict(publishers).get(topic):
        raise RuntimeError('another chassis command publisher already exists: '+topic)
    bridge = AckermannControlBridge()
    try:
        bridge.run()
    except rospy.ROSInterruptException:
        pass
