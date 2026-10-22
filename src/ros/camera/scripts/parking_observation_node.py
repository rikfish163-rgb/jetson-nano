#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS adapter for the strict reverse-parking observation contract.

The node has one job: join fresh, metric perception outputs and publish one
``parking_observation_v1`` JSON message.  It intentionally does not infer
geometry from the existing pixel-only lane topic.  Until a front/rear metric
source is valid, ``valid`` stays false and the control side remains stopped.
"""

from __future__ import print_function

import json
import os
import sys
import time

import rospy
from std_msgs.msg import String


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from parking_observation import ObservationBuilder


class ParkingObservationNode(object):
    def __init__(self):
        self.output_topic = rospy.get_param(
            "~output_topic", "/parking/observation")
        self.publish_hz = float(rospy.get_param("~publish_hz", 20.0))
        if self.publish_hz <= 0.0:
            raise ValueError("publish_hz must be positive")

        builder_config = {
            "default_slot_id": rospy.get_param("~default_slot_id", "P4"),
            "supported_slot_ids": rospy.get_param(
                "~supported_slot_ids", ["P4", "P5"]),
            "input_timeout_s": float(
                rospy.get_param("~input_timeout_s", 0.50)),
            "target_timeout_s": float(
                rospy.get_param("~target_timeout_s", 0.50)),
            "sign_timeout_s": float(
                rospy.get_param("~sign_timeout_s", 1.50)),
            "parking_request_hold_s": float(
                rospy.get_param("~parking_request_hold_s", 120.0)),
            "selected_slot_latch_s": float(
                rospy.get_param("~selected_slot_latch_s", 120.0)),
            "parking_sign_labels": rospy.get_param(
                "~parking_sign_labels", [
                    "p", "park", "parking", "parking_sign", "p_sign",
                    "reverse_parking"]),
            "request_from_front_slot": bool(
                rospy.get_param("~request_from_front_slot", False)),
            "min_source_confidence": float(
                rospy.get_param("~min_source_confidence", 0.55)),
            "derive_clear_from_lidar": bool(
                rospy.get_param("~derive_clear_from_lidar", True)),
        }
        self.builder = ObservationBuilder(builder_config)
        self.publisher = rospy.Publisher(
            self.output_topic, String, queue_size=1)

        self._subscribe("front", "~front_topic",
                        "/perception/front_parking_metric")
        self._subscribe("rear", "~rear_topic",
                        "/perception/rear_parking_metric")
        self._subscribe("exit", "~exit_topic",
                        "/perception/exit_metric")
        self._subscribe("target", "~target_topic",
                        "/perception/parking_target")
        self._subscribe("sign", "~sign_topic", "/camera_hts/receive")
        self._subscribe("lidar", "~lidar_topic", "/parking/lidar")

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_hz), self._publish)
        rospy.loginfo(
            "parking_observation_node ready: output=%s rate=%.1fHz",
            self.output_topic, self.publish_hz)

    def _subscribe(self, source_name, parameter_name, default_topic):
        topic = rospy.get_param(parameter_name, default_topic)
        rospy.Subscriber(
            topic,
            String,
            lambda message, name=source_name: self._callback(name, message),
            queue_size=1,
        )
        rospy.loginfo("parking observation input %s: %s", source_name, topic)

    def _callback(self, source_name, message):
        if not self.builder.update(source_name, message.data, time.time()):
            rospy.logwarn_throttle(
                2.0,
                "parking observation rejected malformed %s payload",
                source_name,
            )

    def _publish(self, _event):
        if rospy.is_shutdown():
            return
        observation = self.builder.build(time.time())
        message = String()
        message.data = json.dumps(
            observation, separators=(",", ":"), allow_nan=False)
        self.publisher.publish(message)


def main():
    rospy.init_node("parking_observation_node", anonymous=False)
    try:
        ParkingObservationNode()
    except Exception as exc:
        rospy.logfatal("parking_observation_node cannot start: %s", exc)
        return 2
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
