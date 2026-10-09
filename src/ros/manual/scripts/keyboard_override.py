#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Safe keyboard override publisher for the central control bridge.

This node intentionally publishes ``std_msgs/String`` JSON on a dedicated
keyboard topic.  It never publishes ``/ackermann_cmd`` directly, so the
Ackermann control bridge remains the only chassis command publisher.
"""

from __future__ import print_function

import json
import select
import sys
import termios
import tty

import rospy
from std_msgs.msg import String


class KeyboardOverride(object):
    def __init__(self):
        self.topic = rospy.get_param(
            "~keyboard_control_topic", "/keyboard/control_cmd")
        self.publish_rate = float(rospy.get_param("~publish_rate", 20.0))
        self.speed_step_raw = int(rospy.get_param("~speed_step_raw", 1))
        self.steering_step_raw = int(
            rospy.get_param("~steering_step_raw", 1))
        # Conservative manual default.  It can be raised explicitly after the
        # speed raw calibration, but never exceeds the central bridge limit.
        self.max_speed_raw = int(rospy.get_param("~max_speed_raw", 12))
        self.max_steering_raw = int(
            rospy.get_param("~max_steering_raw", 22))
        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate must be greater than zero")
        if self.speed_step_raw <= 0 or self.steering_step_raw <= 0:
            raise ValueError("keyboard steps must be positive")
        if self.max_speed_raw < 0 or self.max_steering_raw < 0:
            raise ValueError("keyboard raw limits must not be negative")

        self.enabled = False
        self.speed_raw = 0
        self.steering_raw = 0
        self.sequence = 0
        self.publisher = rospy.Publisher(self.topic, String, queue_size=1)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self.publish_callback)
        rospy.loginfo(
            "Keyboard override ready on %s; w/s speed, a/d steering, "
            "space/x stop, r centre, q release and exit",
            self.topic)

    def publish_callback(self, _event):
        data = {
            "stamp": rospy.Time.now().to_sec(),
            "version": 1,
            "seq": self.sequence,
            "enabled": bool(self.enabled),
            "speed_raw": int(self.speed_raw if self.enabled else 0),
            "steering_raw": int(self.steering_raw if self.enabled else 0),
        }
        self.publisher.publish(String(data=json.dumps(data)))
        self.sequence = (self.sequence + 1) % 256

    def publish_release(self):
        self.enabled = False
        self.speed_raw = 0
        self.steering_raw = 0
        for _ in range(3):
            self.publish_callback(None)
            rospy.sleep(0.05)

    def read_key(self, settings):
        tty.setraw(sys.stdin.fileno())
        try:
            ready, _, _ = select.select([sys.stdin], [], [], 0.10)
            if ready:
                return sys.stdin.read(1)
            return None
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def apply_key(self, key):
        if key is None:
            return True
        if key in ("q", "\x03"):
            return False
        key = key.lower()
        if key == "w":
            self.enabled = True
            self.speed_raw = min(
                self.max_speed_raw, self.speed_raw + self.speed_step_raw)
        elif key == "s":
            self.enabled = True
            self.speed_raw = max(
                -self.max_speed_raw, self.speed_raw - self.speed_step_raw)
        elif key == "a":
            self.enabled = True
            self.steering_raw = min(
                self.max_steering_raw,
                self.steering_raw + self.steering_step_raw)
        elif key == "d":
            self.enabled = True
            self.steering_raw = max(
                -self.max_steering_raw,
                self.steering_raw - self.steering_step_raw)
        elif key in (" ", "x"):
            self.enabled = True
            self.speed_raw = 0
            self.steering_raw = 0
        elif key == "r":
            self.enabled = True
            self.steering_raw = 0
        return True

    def run(self):
        if not sys.stdin.isatty():
            raise RuntimeError(
                "keyboard_override must run from an interactive terminal")
        settings = termios.tcgetattr(sys.stdin)
        try:
            while not rospy.is_shutdown():
                if not self.apply_key(self.read_key(settings)):
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            self.publish_release()


if __name__ == "__main__":
    rospy.init_node("keyboard_override")
    KeyboardOverride().run()
