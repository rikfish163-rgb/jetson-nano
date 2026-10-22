#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
"""Minimal direct Ackermann trial for the frozen lane-perception pipeline."""

from __future__ import division

import math
import threading

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32

from vehicle_control.pure_pursuit import PurePursuit


PATH_TOPIC = "/vision/lane_path"
CONFIDENCE_TOPIC = "/vision/lane_confidence"
COMMAND_TOPIC = "/ackermann_cmd"
PUBLISH_HZ = 20.0
MAX_STEERING_RAW = 22
MAX_STEERING_RAD = 0.46275


def _is_finite(value):
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return False
    return not math.isnan(value) and not math.isinf(value)


class LaneAckermannTrial(object):
    def __init__(self):
        wheelbase = float(rospy.get_param("~wheelbase", 0.29))
        lookahead_distance = float(
            rospy.get_param("~lookahead_distance", 0.60)
        )
        self.min_confidence = float(
            rospy.get_param("~min_confidence", 0.55)
        )
        self.input_timeout = float(rospy.get_param("~input_timeout", 0.25))
        self.speed_raw = float(rospy.get_param("~speed_raw", 0.0))
        steering_limit = float(rospy.get_param("~max_steering_raw", 8))
        if not _is_finite(self.speed_raw) or not 0.0 <= self.speed_raw <= 100.0:
            raise ValueError("speed_raw must be finite and in [0, 100]")
        if not _is_finite(steering_limit):
            raise ValueError("max_steering_raw must be finite")
        self.max_steering_raw = int(max(0.0, min(MAX_STEERING_RAW, steering_limit)))

        if not _is_finite(self.min_confidence):
            raise ValueError("min_confidence must be finite")
        if self.min_confidence < 0.0 or self.min_confidence > 1.0:
            raise ValueError("min_confidence must be in [0, 1]")
        if not _is_finite(self.input_timeout) or self.input_timeout <= 0.0:
            raise ValueError("input_timeout must be positive and finite")

        self.pure_pursuit = PurePursuit(
            wheelbase=wheelbase,
            lookahead_distance=lookahead_distance,
            target_speed=0.0,
        )

        self.lock = threading.Lock()
        self.path_points = None
        self.path_time = None
        self.path_frame_valid = False
        self.confidence = None
        self.confidence_time = None

        self.publisher = rospy.Publisher(
            COMMAND_TOPIC,
            AckermannDriveStamped,
            queue_size=1,
        )
        rospy.Subscriber(PATH_TOPIC, Path, self.path_callback, queue_size=1)
        rospy.Subscriber(
            CONFIDENCE_TOPIC,
            Float32,
            self.confidence_callback,
            queue_size=1,
        )
        rospy.on_shutdown(self.publish_stop)

        rospy.logwarn(
            "lane_ackermann_trial is a direct raw-like trial: speed=%.1f, "
            "steering_raw_limit=+-%d, min_confidence=%.3f, timeout=%.3f",
            self.speed_raw,
            self.max_steering_raw,
            self.min_confidence,
            self.input_timeout,
        )

    def path_callback(self, message):
        points = []
        frame_valid = message.header.frame_id == "base_link"
        if frame_valid:
            for pose_stamped in message.poses:
                points.append(
                    (
                        pose_stamped.pose.position.x,
                        pose_stamped.pose.position.y,
                    )
                )

        now = rospy.Time.now().to_sec()
        with self.lock:
            self.path_points = points
            self.path_time = now
            self.path_frame_valid = frame_valid

    def confidence_callback(self, message):
        now = rospy.Time.now().to_sec()
        with self.lock:
            self.confidence = message.data
            self.confidence_time = now

    def snapshot(self):
        with self.lock:
            points = None
            if self.path_points is not None:
                points = list(self.path_points)
            return (
                points,
                self.path_time,
                self.path_frame_valid,
                self.confidence,
                self.confidence_time,
            )

    def evaluate(self, now):
        snapshot = self.snapshot()
        points = snapshot[0]
        path_time = snapshot[1]
        frame_valid = snapshot[2]
        confidence = snapshot[3]
        confidence_time = snapshot[4]

        if points is None:
            return 0.0, 0.0, "path_missing", 0, confidence, 0.0
        point_count = len(points)
        if not frame_valid:
            return 0.0, 0.0, "path_frame_invalid", point_count, confidence, 0.0
        if not points:
            return 0.0, 0.0, "path_empty", point_count, confidence, 0.0
        if not _is_finite(now):
            return 0.0, 0.0, "time_invalid", point_count, confidence, 0.0
        if not _is_finite(path_time):
            return 0.0, 0.0, "path_time_missing", point_count, confidence, 0.0
        if now - float(path_time) > self.input_timeout:
            return 0.0, 0.0, "path_stale", point_count, confidence, 0.0
        if float(path_time) - now > self.input_timeout:
            return 0.0, 0.0, "path_time_future", point_count, confidence, 0.0
        if not _is_finite(confidence):
            return 0.0, 0.0, "confidence_invalid", point_count, confidence, 0.0
        if not _is_finite(confidence_time):
            return 0.0, 0.0, "confidence_time_missing", point_count, confidence, 0.0
        if now - float(confidence_time) > self.input_timeout:
            return 0.0, 0.0, "confidence_stale", point_count, confidence, 0.0
        if float(confidence_time) - now > self.input_timeout:
            return 0.0, 0.0, "confidence_time_future", point_count, confidence, 0.0
        if float(confidence) < self.min_confidence:
            return 0.0, 0.0, "confidence_low", point_count, confidence, 0.0

        result = self.pure_pursuit.compute(points)
        if not result.valid:
            return (
                0.0,
                0.0,
                "pure_pursuit_%s" % result.reason,
                point_count,
                confidence,
                0.0,
            )
        if not _is_finite(result.steering_angle):
            return 0.0, 0.0, "steering_invalid", point_count, confidence, 0.0

        steering_rad = float(result.steering_angle)
        # Endpoint-calibrated linear approximation, not a full servo calibration.
        steering_raw_float = steering_rad / MAX_STEERING_RAD * MAX_STEERING_RAW
        steering_raw = int(round(steering_raw_float))
        steering_raw = max(
            -self.max_steering_raw,
            min(self.max_steering_raw, steering_raw),
        )
        return (
            self.speed_raw,
            float(steering_raw),
            "ok",
            point_count,
            float(confidence),
            steering_rad,
        )

    def publish(self, speed, steering_raw):
        message = AckermannDriveStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "base_link"
        message.drive.speed = float(speed)
        message.drive.steering_angle = float(steering_raw)
        self.publisher.publish(message)

    def publish_stop(self):
        try:
            self.publish(0.0, 0.0)
        except Exception:
            pass

    def run(self):
        rate = rospy.Rate(PUBLISH_HZ)
        while not rospy.is_shutdown():
            command = self.evaluate(rospy.Time.now().to_sec())
            self.publish(command[0], command[1])
            if command[2] == "ok":
                rospy.loginfo_throttle(
                    1.0,
                    "lane trial confidence=%.3f points=%d steering_rad=%.4f "
                    "steering_raw=%.1f speed_raw=%.1f" %
                    (
                        command[4],
                        command[3],
                        command[5],
                        command[1],
                        command[0],
                    ),
                )
            else:
                rospy.logwarn_throttle(
                    1.0,
                    "lane trial SAFE STOP reason=%s confidence=%s points=%d" %
                    (command[2], str(command[4]), command[3]),
                )
            rate.sleep()


def main():
    rospy.init_node("lane_ackermann_trial", anonymous=False)
    try:
        node = LaneAckermannTrial()
    except (TypeError, ValueError) as error:
        rospy.logfatal("invalid lane_ackermann_trial parameter: %s" % str(error))
        return
    node.run()


if __name__ == "__main__":
    main()
