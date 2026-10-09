#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Read-only ROS preflight check for the parking stack.

This node subscribes to the contracts and never publishes a vehicle command.
It is intended to be run after ``parking_closed_loop.launch`` while the
calibration lock is still false.  It reports which sources are alive and
whether any central command was non-zero during the observation window.
"""

from __future__ import print_function

import argparse
import json
import math
import sys
import time

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String


TOPICS = {
    "observation": "/parking/observation",
    "control": "/control/cmd",
    "status": "/parking/status",
    "lidar": "/parking/lidar",
    "front_metric": "/perception/front_parking_metric",
    "rear_metric": "/perception/rear_parking_metric",
    "exit_metric": "/perception/exit_metric",
    "actuator_command": "/ackermann_cmd",
    "base_controller_status": "/base_controller/status",
}


class ParkingStackCheck(object):
    def __init__(self):
        self.topics = dict(TOPICS)
        self.topics["observation"] = rospy.get_param(
            "~observation_topic", TOPICS["observation"])
        self.topics["lidar"] = rospy.get_param(
            "~lidar_topic", TOPICS["lidar"])
        self.topics["actuator_command"] = rospy.get_param(
            "~actuator_command_topic", TOPICS["actuator_command"])
        self.topics["base_controller_status"] = rospy.get_param(
            "~base_controller_status_topic", TOPICS["base_controller_status"])
        self.last = {}
        self.received = dict((name, 0) for name in self.topics)
        self.control_nonzero = []
        self.actuator_nonzero = []
        for name, topic in self.topics.items():
            if name == "actuator_command":
                rospy.Subscriber(
                    topic, AckermannDriveStamped, self._actuator_callback,
                    callback_args=name, queue_size=10)
            else:
                rospy.Subscriber(
                    topic, String, self._callback,
                    callback_args=name, queue_size=10)

    def _callback(self, message, name):
        received_at = time.time()
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = message.data
        self.last[name] = {
            "received_at": received_at,
            "payload": payload,
        }
        self.received[name] += 1
        if name == "control" and isinstance(payload, dict):
            try:
                speed = int(payload.get("speed_raw", 0))
                steering = int(payload.get("steering_raw", 0))
            except (TypeError, ValueError):
                return
            if speed != 0 or steering != 0:
                self.control_nonzero.append({
                    "wall_time": received_at,
                    "speed_raw": speed,
                    "steering_raw": steering,
                })

    @staticmethod
    def _finite(value):
        try:
            value = float(value)
        except (TypeError, ValueError):
            return None
        if math.isnan(value) or math.isinf(value):
            return None
        return value

    def _actuator_callback(self, message, name):
        received_at = time.time()
        speed = self._finite(message.drive.speed)
        steering = self._finite(message.drive.steering_angle)
        payload = {
            "schema": "ackermann_command_observation_v1",
            "speed_raw": speed,
            "steering_raw": steering,
            "valid": speed is not None and steering is not None,
        }
        self.last[name] = {
            "received_at": received_at,
            "payload": payload,
        }
        self.received[name] += 1
        if (payload["valid"] and
                (speed != 0.0 or steering != 0.0)):
            self.actuator_nonzero.append({
                "wall_time": received_at,
                "speed_raw": speed,
                "steering_raw": steering,
            })

    def report(self):
        calibration_complete = rospy.get_param(
            "/main_node/parking/calibration_complete", None)
        parking_only = rospy.get_param("/main_node/parking_only", None)
        sources = {}
        for name in self.topics:
            entry = self.last.get(name)
            if entry is None:
                sources[name] = {
                    "topic": self.topics[name],
                    "received": 0,
                    "last": None,
                }
                continue
            payload = entry["payload"]
            summary = {
                "topic": self.topics[name],
                "received": self.received[name],
            }
            if isinstance(payload, dict):
                summary.update({
                    "schema": payload.get("schema"),
                    "valid": payload.get("valid"),
                    "state": payload.get("state"),
                    "required_visual_source": payload.get(
                        "required_visual_source"),
                    "slot_id": payload.get("slot_id"),
                    "slot_consistent": payload.get("slot_consistent",
                                                   payload.get(
                                                       "observation_slot_consistent")),
                    "source_confidence": payload.get(
                        "source_confidence",
                        payload.get("observation_source_confidence")),
                    "speed_raw": payload.get("speed_raw"),
                    "steering_raw": payload.get("steering_raw"),
                    "obstacle_detected": payload.get("obstacle_detected"),
                    "emergency_stop": payload.get("emergency_stop"),
                })
            else:
                summary["type"] = type(payload).__name__
            sources[name] = summary
        return {
            "calibration_complete": calibration_complete,
            "parking_only": parking_only,
            "sources": sources,
            "control_nonzero_count": len(self.control_nonzero),
            "control_nonzero": self.control_nonzero[-10:],
            "actuator_nonzero_count": len(self.actuator_nonzero),
            "actuator_nonzero": self.actuator_nonzero[-10:],
        }


def main(argv=None):
    parser = argparse.ArgumentParser(description="read-only parking ROS preflight")
    parser.add_argument("--duration", type=float, default=5.0)
    args = parser.parse_args(rospy.myargv()[1:] if argv is None else argv)
    if args.duration <= 0.0:
        parser.error("--duration must be positive")

    rospy.init_node("parking_stack_check", anonymous=True)
    checker = ParkingStackCheck()
    deadline = time.time() + args.duration
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown() and time.time() < deadline:
        rate.sleep()

    report = checker.report()
    print(json.dumps(report, indent=2, sort_keys=True))
    # A locked stack must not emit any non-zero command.  If the parameter is
    # unavailable, keep the result informational rather than guessing.
    if (report["calibration_complete"] is False and
            (report["control_nonzero_count"] or
             report["actuator_nonzero_count"])):
        return 1
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(2)
