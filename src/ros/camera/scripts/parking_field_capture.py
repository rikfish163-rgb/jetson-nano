#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

"""Capture a motion-free visual calibration bundle for P4/P5.

This node subscribes to the existing camera/metric topics and writes a small
set of timestamped images plus the decoded JSON payloads.  It never publishes
vehicle commands.  For each rear BEV sample it also runs the rear detector
with both white and blue line masks, so a field capture can identify the
usable line-colour hypothesis before motion is enabled.
"""

from __future__ import print_function

import json
import math
import os
import sys
import time

import cv2
import rospy
import yaml

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

from parking_rear_metric import (  # noqa: E402
    RearParkingDetector,
    apply_rear_visual_profile,
)


try:
    string_types = (basestring,)
except NameError:
    string_types = (str,)


IMAGE_TOPIC_DEFAULTS = {
    "front_raw": "/front/usb_cam/image_raw",
    "front_bev": "/debug/metric_bev",
    # Compatibility aliases point at the white slot detector.  The reform
    # launch also records the independent blue start-line debug images.
    "front_mask": "/debug/parking_slot_mask",
    "front_overlay": "/debug/parking_slot_overlay",
    "slot_mask": "/debug/parking_slot_mask",
    "slot_overlay": "/debug/parking_slot_overlay",
    "start_line_mask": "/debug/parking_start_line_mask",
    "start_line_overlay": "/debug/parking_start_line_overlay",
    "rear_raw": "/rear/usb_cam/image_raw",
    "rear_bev": "/debug/rear_bev",
}

STRING_TOPIC_DEFAULTS = {
    "front_candidate": "/perception/parking_slot_candidate",
    "slot_candidate": "/perception/parking_slot_candidate",
    "start_line": "/perception/parking_start_line",
    "front_pose": "/perception/front_slot_pose",
    "rear_metric": "/perception/rear_parking_metric",
    "exit_metric": "/perception/exit_metric",
    "observation": "/parking/observation",
}


def decode_payload(value):
    """Decode one std_msgs/String payload, returning a mapping or None."""
    if not isinstance(value, string_types):
        return None
    try:
        data = json.loads(value)
    except (TypeError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def finite(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return False
    return not math.isnan(value) and not math.isinf(value)


class ParkingFieldCapture(object):
    """Short-lived ROS subscriber that produces a replayable visual bundle."""

    def __init__(self):
        self.slot_id = str(rospy.get_param("~slot_id", "P4"))
        self.output_dir = os.path.abspath(rospy.get_param(
            "~output_dir", "/tmp/parking_field_captures"))
        self.sample_count = int(rospy.get_param("~sample_count", 8))
        self.period_s = float(rospy.get_param("~period_s", 0.25))
        self.max_age_s = float(rospy.get_param("~max_age_s", 0.75))
        if self.sample_count <= 0:
            raise ValueError("sample_count must be positive")
        if self.period_s <= 0.0 or not finite(self.period_s):
            raise ValueError("period_s must be positive and finite")
        if self.max_age_s <= 0.0 or not finite(self.max_age_s):
            raise ValueError("max_age_s must be positive and finite")

        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)
        base_run_id = "capture_%s_%s" % (
            self.slot_id, time.strftime("%Y%m%d_%H%M%S"))
        self.run_id = base_run_id
        self.run_dir = os.path.join(self.output_dir, self.run_id)
        suffix = 1
        while os.path.exists(self.run_dir):
            self.run_id = "%s_%02d" % (base_run_id, suffix)
            self.run_dir = os.path.join(self.output_dir, self.run_id)
            suffix += 1
        os.makedirs(self.run_dir)
        self.manifest_path = os.path.join(self.run_dir, "manifest.jsonl")

        self.bridge = CvBridge()
        self.images = {}
        self.image_received_at = {}
        self.image_stamps = {}
        self.payloads = {}
        self.payload_received_at = {}

        for key, default_topic in IMAGE_TOPIC_DEFAULTS.items():
            topic = rospy.get_param("~%s_topic" % key, default_topic)
            rospy.Subscriber(
                topic, Image, self._make_image_callback(key), queue_size=1,
                buff_size=2 ** 24)
        for key, default_topic in STRING_TOPIC_DEFAULTS.items():
            topic = rospy.get_param("~%s_topic" % key, default_topic)
            rospy.Subscriber(
                topic, String, self._make_string_callback(key), queue_size=1)

        self.rear_settings = self._load_rear_settings()
        self.sample_index = 0
        self.timer = rospy.Timer(
            rospy.Duration(self.period_s), self._sample_callback)
        rospy.loginfo(
            "parking_field_capture ready: slot=%s samples=%d output=%s "
            "motion_publishers=none",
            self.slot_id, self.sample_count, self.run_dir)

    def _make_image_callback(self, key):
        def callback(message):
            # The closure is intentionally tiny so the latest frame is kept
            # without retaining a ROS message queue or publishing anything.
            try:
                image = callback.owner.bridge.imgmsg_to_cv2(message, "bgr8")
            except CvBridgeError as exc:
                rospy.logwarn_throttle(
                    2.0, "capture image conversion failed (%s): %s" %
                    (key, str(exc)))
                return
            # Store only the latest frame; the capture timer decides whether
            # it is fresh enough for the next bundle sample.
            callback.owner.images[key] = image.copy()
            callback.owner.image_received_at[key] = time.time()
            try:
                callback.owner.image_stamps[key] = message.header.stamp.to_sec()
            except (AttributeError, TypeError, ValueError):
                callback.owner.image_stamps[key] = None
        callback.owner = self
        return callback

    def _make_string_callback(self, key):
        def callback(message):
            callback.owner.payloads[key] = decode_payload(message.data)
            callback.owner.payload_received_at[key] = time.time()
        callback.owner = self
        return callback

    def _load_rear_settings(self):
        settings = dict(RearParkingDetector.DEFAULTS)
        for key in RearParkingDetector.DEFAULTS:
            parameter = "~" + key
            if rospy.has_param(parameter):
                settings[key] = rospy.get_param(parameter)
        profiles = rospy.get_param("~slot_profiles", {})
        if not isinstance(profiles, dict):
            raise ValueError("slot_profiles must be an object")
        profile = profiles.get(self.slot_id, {})
        return apply_rear_visual_profile(settings, profile)

    def _fresh(self, received_at, now):
        return (received_at is not None and
                now - float(received_at) <= self.max_age_s)

    def _write_image(self, name, image):
        if image is None:
            return None
        path = os.path.join(self.run_dir, name)
        if not cv2.imwrite(path, image):
            raise IOError("cannot write capture image: %s" % path)
        return os.path.basename(path)

    def _rear_color_probe(self, image, stamp):
        result = {}
        for color in ("white", "blue"):
            settings = dict(self.rear_settings)
            settings["line_color"] = color
            detector = RearParkingDetector(settings)
            detected, overlay = detector.detect(image, stamp)
            detected["slot_id"] = self.slot_id
            entry = {
                "valid": bool(detected.get("valid")),
                "confidence": detected.get("confidence"),
                "rear_distance_m": detected.get("rear_distance_m"),
                "lateral_error_m": detected.get("lateral_error_m"),
                "vehicle_yaw_error_rad": detected.get(
                    "vehicle_yaw_error_rad"),
            }
            overlay_name = self._write_image(
                "sample_%03d_rear_%s_overlay.png" %
                (self.sample_index, color), overlay)
            entry["overlay"] = overlay_name
            result[color] = entry
        return result

    def _sample_callback(self, _event):
        if rospy.is_shutdown():
            return
        now = time.time()
        sample = {
            "schema": "parking_field_capture_v1",
            "run_id": self.run_id,
            "slot_id": self.slot_id,
            "sample_index": self.sample_index,
            "wall_time": now,
            "motion_publishers": [],
            "images": {},
            "image_stamps": {},
            "payloads": {},
            "rear_color_probe": {},
        }

        for key in IMAGE_TOPIC_DEFAULTS:
            if self._fresh(self.image_received_at.get(key), now):
                image = self.images.get(key)
                sample["images"][key] = self._write_image(
                    "sample_%03d_%s.png" % (self.sample_index, key), image)
                sample["image_stamps"][key] = self.image_stamps.get(key)
            else:
                sample["images"][key] = None
                sample["image_stamps"][key] = None

        for key in STRING_TOPIC_DEFAULTS:
            if self._fresh(self.payload_received_at.get(key), now):
                sample["payloads"][key] = self.payloads.get(key)
            else:
                sample["payloads"][key] = None

        rear_image = (self.images.get("rear_bev")
                       if self._fresh(self.image_received_at.get("rear_bev"), now)
                       else None)
        if rear_image is not None:
            try:
                sample["rear_color_probe"] = self._rear_color_probe(
                    rear_image, now)
            except Exception as exc:
                sample["rear_color_probe"] = {"error": str(exc)}

        with open(self.manifest_path, "a") as stream:
            stream.write(json.dumps(
                sample, separators=(",", ":"), allow_nan=False) + "\n")
        rospy.loginfo(
            "parking_field_capture sample %d/%d: rear_bev=%s rear_probe=%s",
            self.sample_index + 1, self.sample_count,
            str(sample["images"].get("rear_bev")),
            json.dumps(sample["rear_color_probe"], sort_keys=True))
        self.sample_index += 1
        if self.sample_index >= self.sample_count:
            rospy.loginfo("parking_field_capture complete: %s", self.manifest_path)
            rospy.signal_shutdown("capture_complete")


def main():
    rospy.init_node("parking_field_capture", anonymous=False)
    try:
        ParkingFieldCapture()
    except Exception as exc:
        rospy.logfatal("parking_field_capture cannot start: %s", exc)
        return 2
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
