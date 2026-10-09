#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Run a fixed raw-command test scenario with keyboard start/e-stop control.

Controls:
    w       Start the complete scenario from step 1 while idle.
    Space   Abort the scenario immediately and command a full stop.

All other keys are ignored. The node publishes continuously so that the
base-controller watchdog remains fed while the scenario is active or idle.
"""

from __future__ import print_function

import select
import sys
import termios
import time
import tty

import rospy
from ackermann_msgs.msg import AckermannDriveStamped


PUBLISH_RATE_HZ = 20.0

# Each item is: (description, speed_raw, steering_raw, duration_seconds).
SCENARIO = (
    ("stopped, steering raw +1", 0, 1, 10.0),
    ("stopped, steering raw -1", 0, -1, 10.0),
    ("stopped, steering raw +22", 0, 22, 80.0),
    ("stopped, steering raw -22", 0, -22, 80.0),
    ("forward, speed raw +20", 50, 0, 5.0),
    ("stopped", 0, 0, 5.0),
    ("reverse, speed raw -20", -50, 0, 5.0),
)


class AckermannAutoLoop(object):
    def __init__(self):
        self.command_topic = rospy.get_param(
            "~command_topic", "/ackermann_cmd")
        self.publish_rate_hz = float(rospy.get_param(
            "~publish_rate", PUBLISH_RATE_HZ))
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate must be greater than zero")

        self.publisher = rospy.Publisher(
            self.command_topic, AckermannDriveStamped, queue_size=1)

        self.speed_raw = 0
        self.steering_raw = 0
        self.running = False
        self.step_index = -1
        self.step_started_at = 0.0

        self.terminal_fd = None
        self.original_terminal_settings = None

        rospy.on_shutdown(self.publish_stop)

    def print_instructions(self):
        total_duration = sum(step[3] for step in SCENARIO)
        rospy.loginfo("Raw scenario controller ready")
        rospy.loginfo("Publishing: %s at %.1f Hz",
                      self.command_topic, self.publish_rate_hz)
        rospy.loginfo("Press [w] to start the %.1f-second scenario",
                      total_duration)
        rospy.loginfo("Press [Space] for emergency stop")
        rospy.loginfo("All other keys are ignored; use Ctrl-C to exit")

    def configure_terminal(self):
        if not sys.stdin.isatty():
            raise RuntimeError(
                "autoloop.py must be run from an interactive terminal")
        self.terminal_fd = sys.stdin.fileno()
        self.original_terminal_settings = termios.tcgetattr(self.terminal_fd)
        tty.setcbreak(self.terminal_fd)

    def restore_terminal(self):
        if self.terminal_fd is not None and self.original_terminal_settings:
            termios.tcsetattr(
                self.terminal_fd,
                termios.TCSADRAIN,
                self.original_terminal_settings)
            self.original_terminal_settings = None

    @staticmethod
    def read_key_nonblocking():
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return None
        return sys.stdin.read(1)

    def set_command(self, speed_raw, steering_raw):
        self.speed_raw = int(speed_raw)
        self.steering_raw = int(steering_raw)

    def publish_command(self):
        message = AckermannDriveStamped()
        message.header.stamp = rospy.Time.now()
        message.drive.speed = float(self.speed_raw)
        message.drive.steering_angle = float(self.steering_raw)
        self.publisher.publish(message)

    def publish_stop(self):
        self.set_command(0, 0)
        self.publish_command()

    def enter_step(self, step_index, started_at):
        self.step_index = step_index
        self.step_started_at = started_at

        description, speed_raw, steering_raw, duration = SCENARIO[step_index]
        self.set_command(speed_raw, steering_raw)
        rospy.loginfo(
            "Step %d/%d: %s; speed_raw=%d steering_raw=%d duration=%.1fs",
            step_index + 1,
            len(SCENARIO),
            description,
            speed_raw,
            steering_raw,
            duration)

    def start_scenario(self):
        if self.running:
            return
        self.running = True
        rospy.loginfo("Scenario started")
        self.enter_step(0, time.time())

    def emergency_stop(self):
        was_running = self.running
        self.running = False
        self.step_index = -1
        self.set_command(0, 0)
        self.publish_command()
        if was_running:
            rospy.logwarn("Emergency stop: scenario aborted; output is 0/0")
        else:
            rospy.loginfo("Stop command confirmed: output is 0/0")

    def finish_scenario(self):
        self.running = False
        self.step_index = -1
        self.set_command(0, 0)
        self.publish_command()
        rospy.loginfo(
            "Scenario complete; vehicle stopped. Press [w] to run again")

    def update_scenario(self, now):
        if not self.running:
            return

        # A loop delay may cross more than one boundary. Advance using the
        # scheduled boundary time so individual delays do not accumulate.
        while self.running:
            duration = SCENARIO[self.step_index][3]
            if now - self.step_started_at < duration:
                return

            next_started_at = self.step_started_at + duration
            next_step = self.step_index + 1
            if next_step >= len(SCENARIO):
                self.finish_scenario()
                return
            self.enter_step(next_step, next_started_at)

    def handle_key(self, key):
        if key == "w":
            self.start_scenario()
        elif key == " ":
            self.emergency_stop()
        # Ctrl-C is handled by the terminal/ROS. Every other key is ignored.

    def run(self):
        self.configure_terminal()
        self.print_instructions()
        self.publish_stop()

        rate = rospy.Rate(self.publish_rate_hz)
        try:
            while not rospy.is_shutdown():
                key = self.read_key_nonblocking()
                if key is not None:
                    self.handle_key(key)

                self.update_scenario(time.time())
                self.publish_command()
                rate.sleep()
        finally:
            self.publish_stop()
            self.restore_terminal()


if __name__ == "__main__":
    rospy.init_node("ackermann_auto_loop_node")
    controller = AckermannAutoLoop()
    try:
        controller.run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
    finally:
        controller.restore_terminal()
