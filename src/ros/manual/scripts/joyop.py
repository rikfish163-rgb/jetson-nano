#!/usr/bin/env python
"""Deadman joystick input for testloop; never publish chassis commands directly."""
from __future__ import division
import json
import math
import threading
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class JoystickCommand(object):
    def __init__(self, speed_axis=1, steer_axis=3, deadman_button=4,
                 release_button=7, speed_raw=12, steer_raw=22, timeout=.25):
        if min(speed_axis, steer_axis, deadman_button, release_button) < 0:
            raise ValueError('joystick indices must be nonnegative')
        if not 0 < speed_raw <= 100 or not 0 < steer_raw <= 22 or not 0 < timeout <= 1:
            raise ValueError('invalid joystick limits')
        self.indices = speed_axis, steer_axis, deadman_button, release_button
        self.limits = speed_raw, steer_raw
        self.timeout = timeout
        self.enabled, self.command, self.stamp = False, (0, 0), None

    def observe(self, axes, buttons, now):
        a, b, deadman, release = self.indices
        try:
            speed, steer = float(axes[a]), float(axes[b])
            if any(math.isnan(v) or math.isinf(v) or abs(v) > 1 for v in (speed, steer)):
                raise ValueError('invalid axes')
            held, released = bool(buttons[deadman]), bool(buttons[release])
        except (IndexError, TypeError, ValueError):
            self.enabled, self.command, self.stamp = True, (0, 0), now
            return
        self.stamp = now
        if held:
            self.enabled = True
            self.command = tuple(int(round(v * limit)) for v, limit in zip((speed, steer), self.limits))
        else:
            self.command = (0, 0)
            if released:
                self.enabled = False

    def output(self, now, sequence):
        fresh = self.stamp is not None and 0 <= now-self.stamp <= self.timeout
        speed, steer = self.command if fresh else (0, 0)
        return dict(version=1, seq=sequence, stamp=now, enabled=self.enabled,
                    speed_raw=speed, steering_raw=steer)


class AckermannDriveJoyop(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.command = JoystickCommand(**{name: rospy.get_param('~'+name, default)
            for name, default in (('speed_axis',1), ('steer_axis',3), ('deadman_button',4),
                                  ('release_button',7), ('speed_raw',12), ('steer_raw',22), ('timeout',.25))})
        self.sequence = 0
        self.publisher = rospy.Publisher('/joystick/control_cmd', String, queue_size=1)
        self.subscriber = rospy.Subscriber('/joy', Joy, self.observe, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(.05), self.publish)
        rospy.on_shutdown(self.stop)

    def observe(self, msg):
        with self.lock:
            now = rospy.Time.now().to_sec()
            stamp = msg.header.stamp.to_sec()
            if not 0 <= now-stamp <= self.command.timeout:
                self.command.enabled, self.command.command = True, (0,0)
                return
            self.command.observe(msg.axes, msg.buttons, stamp)

    def publish(self, event):
        with self.lock:
            self.publisher.publish(String(data=json.dumps(self.command.output(rospy.Time.now().to_sec(), self.sequence))))
            self.sequence = (self.sequence + 1) % 256

    def stop(self):
        with self.lock:
            self.command.enabled, self.command.command = True, (0, 0)
        self.publish(None)


if __name__ == '__main__':
    rospy.init_node('ackermann_drive_joyop')
    node = AckermannDriveJoyop()
    rospy.spin()
