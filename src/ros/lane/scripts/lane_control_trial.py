#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
"""TEST/TRIAL lane-path controller. This is not the final vehicle FSM."""

from __future__ import division

import json
import threading

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from std_msgs.msg import String

from vehicle_control.lane_control import LaneControlPolicy
from vehicle_control.pure_pursuit import PurePursuit


PATH_TOPIC = "/vision/lane_path"
CONFIDENCE_TOPIC = "/vision/lane_confidence"
CONTROL_TOPIC = "/control/send"
PUBLISH_HZ = 20.0


class LaneControlTrialNode(object):
    def __init__(self):
        wheelbase = rospy.get_param("~wheelbase", 0.29)
        lookahead_distance = rospy.get_param("~lookahead_distance", 0.60)
        target_speed = rospy.get_param("~target_speed", 0.0)
        min_confidence = rospy.get_param("~min_confidence", 0.55)
        path_timeout = rospy.get_param("~path_timeout", 0.25)
        max_steering_rad = rospy.get_param("~max_steering_rad", 0.35)
        angel_gain = rospy.get_param("~angel_gain", 1.0)

        pure_pursuit = PurePursuit(
            wheelbase=wheelbase,
            lookahead_distance=lookahead_distance,
            target_speed=target_speed,
        )
        self.policy = LaneControlPolicy(
            pure_pursuit=pure_pursuit,
            target_speed=target_speed,
            min_confidence=min_confidence,
            path_timeout=path_timeout,
            max_steering_rad=max_steering_rad,
            angel_gain=angel_gain,
        )

        self.lock = threading.Lock()
        self.path_points = None
        self.path_update_time = None
        self.confidence = None
        self.confidence_update_time = None

        self.control_pub = rospy.Publisher(CONTROL_TOPIC, String, queue_size=1)
        rospy.Subscriber(PATH_TOPIC, Path, self.path_callback, queue_size=1)
        rospy.Subscriber(
            CONFIDENCE_TOPIC,
            Float32,
            self.confidence_callback,
            queue_size=1,
        )
        rospy.on_shutdown(self.publish_safe_stop)

        rospy.loginfo(
            "lane_control_trial TEST/TRIAL started: target_speed=%.3f "
            "lookahead=%.3f wheelbase=%.3f max_steering_rad=%.3f "
            "angel_gain=%.3f min_confidence=%.3f timeout=%.3f",
            target_speed,
            lookahead_distance,
            wheelbase,
            max_steering_rad,
            angel_gain,
            min_confidence,
            path_timeout,
        )
        if float(target_speed) == 0.0:
            rospy.logwarn("target_speed is 0.0: node will only publish stop-speed trials")

    def path_callback(self, message):
        points = []
        if message.header.frame_id and message.header.frame_id != "base_link":
            rospy.logwarn_throttle(
                2.0,
                "lane path frame_id is '%s', expected 'base_link'" %
                message.header.frame_id,
            )
            points.append((float("nan"), float("nan")))
        else:
            for pose_stamped in message.poses:
                points.append(
                    (
                        pose_stamped.pose.position.x,
                        pose_stamped.pose.position.y,
                    )
                )

        receive_time = rospy.Time.now().to_sec()
        with self.lock:
            self.path_points = points
            self.path_update_time = receive_time

    def confidence_callback(self, message):
        receive_time = rospy.Time.now().to_sec()
        with self.lock:
            self.confidence = message.data
            self.confidence_update_time = receive_time

    def current_inputs(self):
        with self.lock:
            path_points = None
            if self.path_points is not None:
                path_points = list(self.path_points)
            return (
                path_points,
                self.path_update_time,
                self.confidence,
                self.confidence_update_time,
            )

    def publish_command(self, speed, angel):
        payload = {
            "speed": float(speed),
            "angel": float(angel),
        }
        self.control_pub.publish(
            String(data=json.dumps(payload, allow_nan=False, separators=(",", ":")))
        )

    def publish_safe_stop(self):
        try:
            self.publish_command(0.0, 0.0)
        except Exception:
            pass

    def run(self):
        rate = rospy.Rate(PUBLISH_HZ)
        while not rospy.is_shutdown():
            inputs = self.current_inputs()
            command = self.policy.evaluate(
                path_points=inputs[0],
                path_update_time=inputs[1],
                confidence=inputs[2],
                confidence_update_time=inputs[3],
                now=rospy.Time.now().to_sec(),
            )
            self.publish_command(command.speed, command.angel)

            confidence_text = "None"
            if command.confidence is not None:
                confidence_text = str(command.confidence)
            if command.safe_stop:
                rospy.logwarn_throttle(
                    1.0,
                    "SAFE STOP reason=%s confidence=%s path_points=%d "
                    "raw_steering_rad=%.4f final_angel=%.4f speed=%.3f" %
                    (
                        command.reason,
                        confidence_text,
                        command.path_point_count,
                        command.raw_steering_rad,
                        command.angel,
                        command.speed,
                    ),
                )
            else:
                rospy.loginfo_throttle(
                    1.0,
                    "control reason=ok confidence=%.3f path_points=%d "
                    "raw_steering_rad=%.4f final_angel=%.4f speed=%.3f" %
                    (
                        command.confidence,
                        command.path_point_count,
                        command.raw_steering_rad,
                        command.angel,
                        command.speed,
                    ),
                )
            rate.sleep()


def main():
    rospy.init_node("lane_control_trial", anonymous=False)
    try:
        node = LaneControlTrialNode()
    except (TypeError, ValueError) as error:
        rospy.logfatal("invalid lane_control_trial parameter: %s" % str(error))
        return
    node.run()


if __name__ == "__main__":
    main()
