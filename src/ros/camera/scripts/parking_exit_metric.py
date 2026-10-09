#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Normalize a calibrated exit-pose source for the parking observer.

The ordinary front lane detector can tell us where a road lane is in
``base_link``, but it cannot by itself tell us the map-specific exit line or
when the vehicle is clear of the bay.  This node therefore accepts an
explicit ``parking_exit_pose_v1`` source and publishes the common metric
contract.  Missing exit-line geometry stays invalid instead of being
replaced by a timer or a guessed distance.
"""

from __future__ import print_function

import json
import math
import sys
import time

import rospy
from std_msgs.msg import String


try:
    string_types = (basestring,)
except NameError:
    string_types = (str,)


def _number(data, key, default=None):
    value = data.get(key) if isinstance(data, dict) else None
    if isinstance(value, bool):
        return default
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    if math.isnan(value) or math.isinf(value):
        return default
    return value


def _bool(data, key, default=None):
    value = data.get(key) if isinstance(data, dict) else None
    return value if isinstance(value, bool) else default


def _profile_number(profile, key, default):
    """Read one optional per-slot exit-pose calibration value."""
    value = profile.get(key, default) if isinstance(profile, dict) else default
    if isinstance(value, bool):
        raise ValueError("visual profile %s must be numeric" % key)
    try:
        value = float(value)
    except (TypeError, ValueError):
        raise ValueError("visual profile %s must be numeric" % key)
    if math.isnan(value) or math.isinf(value):
        raise ValueError("visual profile %s must be finite" % key)
    return value


def _decode(value):
    if not isinstance(value, string_types):
        return None
    try:
        decoded = json.loads(value)
    except (TypeError, ValueError):
        return None
    return decoded if isinstance(decoded, dict) else None


def _normalise_slot_id(value):
    if not isinstance(value, string_types):
        return None
    value = value.strip().upper()
    if value in ("", "AUTO", "ANY", "NONE", "NULL"):
        return None
    return value if value in ("P4", "P5") else None


class ParkingExitMetricNode(object):
    def __init__(self):
        self.pose_topic = rospy.get_param(
            "~pose_topic", "/perception/exit_pose")
        self.candidate_topic = rospy.get_param(
            "~candidate_topic", "/perception/parking_target")
        self.output_topic = rospy.get_param(
            "~output_topic", "/perception/exit_metric")
        raw_slot_id = str(rospy.get_param("~slot_id", "P4"))
        self.slot_id = _normalise_slot_id(raw_slot_id)
        self.input_timeout_s = float(
            rospy.get_param("~input_timeout_s", 0.50))
        self.use_candidate = bool(rospy.get_param("~use_candidate", False))
        self.candidate_yaw_gain = float(
            rospy.get_param("~candidate_yaw_gain", 1.0))
        self.candidate_yaw_bias_rad = float(
            rospy.get_param("~candidate_yaw_bias_rad", 0.0))
        self.candidate_lateral_gain = float(
            rospy.get_param("~candidate_lateral_gain", 1.0))
        self.candidate_lateral_bias_m = float(
            rospy.get_param("~candidate_lateral_bias_m", 0.0))
        self.candidate_max_yaw_rad = float(
            rospy.get_param("~candidate_max_yaw_rad", 1.50))
        self.candidate_max_lateral_m = float(
            rospy.get_param("~candidate_max_lateral_m", 0.80))
        self.exit_trigger_offset_m = float(
            rospy.get_param("~exit_trigger_offset_m", 0.0))
        self.candidate_confidence = float(
            rospy.get_param("~candidate_confidence", 0.75))
        self.slot_profiles = rospy.get_param("~slot_profiles", {})
        if not isinstance(self.slot_profiles, dict):
            raise ValueError("slot_profiles must be an object")
        slot_profile = (self.slot_profiles.get(self.slot_id, {})
                        if self.slot_id is not None else {})
        if not isinstance(slot_profile, dict):
            raise ValueError("visual profile %s must be an object" % self.slot_id)
        self.candidate_yaw_gain = _profile_number(
            slot_profile, "exit_candidate_yaw_gain",
            self.candidate_yaw_gain)
        self.candidate_yaw_bias_rad = _profile_number(
            slot_profile, "exit_candidate_yaw_bias_rad",
            self.candidate_yaw_bias_rad)
        self.candidate_lateral_gain = _profile_number(
            slot_profile, "exit_candidate_lateral_gain",
            self.candidate_lateral_gain)
        self.candidate_lateral_bias_m = _profile_number(
            slot_profile, "exit_candidate_lateral_bias_m",
            self.candidate_lateral_bias_m)
        if self.input_timeout_s <= 0.0:
            raise ValueError("input_timeout_s must be positive")
        if self.candidate_max_yaw_rad <= 0.0 or self.candidate_max_lateral_m <= 0.0:
            raise ValueError("candidate pose limits must be positive")
        if self.candidate_confidence < 0.0 or self.candidate_confidence > 1.0:
            raise ValueError("candidate_confidence must be in the range 0..1")
        self.pose = None
        self.received_at = None
        self.candidate = None
        self.candidate_received_at = None
        self.publisher = rospy.Publisher(self.output_topic, String, queue_size=1)
        rospy.Subscriber(self.pose_topic, String, self._callback, queue_size=1)
        rospy.Subscriber(
            self.candidate_topic, String, self._candidate_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.05), self._publish)
        rospy.loginfo(
            "parking_exit_metric ready: pose=%s candidate=%s use_candidate=%s output=%s",
            self.pose_topic, self.candidate_topic, str(self.use_candidate),
            self.output_topic)

    def _callback(self, message):
        decoded = _decode(message.data)
        self.pose = decoded
        self.received_at = time.time() if decoded is not None else None

    def _candidate_callback(self, message):
        decoded = _decode(message.data)
        self.candidate = decoded
        self.candidate_received_at = time.time() if decoded is not None else None

    def _candidate_pose(self, candidate):
        if not self.use_candidate or not isinstance(candidate, dict):
            return None
        if candidate.get("candidate") is not True:
            return None
        candidate_slot = _normalise_slot_id(candidate.get("slot_id"))
        if self.slot_id is None and candidate_slot is None:
            return None
        if self.slot_id is not None and candidate_slot not in (None, self.slot_id):
            return None
        distance = _number(candidate, "distance_m")
        angle_deg = _number(candidate, "angle_deg")
        lateral_m = _number(candidate, "lateral_m")
        if distance is None or distance < 0.0 or angle_deg is None or lateral_m is None:
            return None
        profile = (self.slot_profiles.get(candidate_slot, {})
                   if candidate_slot is not None else {})
        yaw_gain = _profile_number(
            profile, "exit_candidate_yaw_gain", self.candidate_yaw_gain)
        yaw_bias = _profile_number(
            profile, "exit_candidate_yaw_bias_rad",
            self.candidate_yaw_bias_rad)
        lateral_gain = _profile_number(
            profile, "exit_candidate_lateral_gain",
            self.candidate_lateral_gain)
        lateral_bias = _profile_number(
            profile, "exit_candidate_lateral_bias_m",
            self.candidate_lateral_bias_m)
        yaw_error = yaw_gain * math.radians(angle_deg) + yaw_bias
        lateral_error = lateral_gain * lateral_m + lateral_bias
        if abs(yaw_error) > self.candidate_max_yaw_rad:
            return None
        if abs(lateral_error) > self.candidate_max_lateral_m:
            return None
        return {
            "schema": "parking_exit_pose_v1",
            "stamp": time.time(),
            "frame_id": "base_link",
            "valid": True,
            "confidence": self.candidate_confidence,
            "slot_id": candidate_slot or self.slot_id,
            "exit_visible": True,
            "exit_distance_m": max(0.0, distance - self.exit_trigger_offset_m),
            "exit_yaw_error_rad": yaw_error,
            "exit_lateral_error_m": lateral_error,
            "exit_complete": False,
            "exit_clear": None,
        }

    def _publish(self, _event):
        if rospy.is_shutdown():
            return
        now = time.time()
        fresh = (isinstance(self.pose, dict) and self.received_at is not None and
                 now - self.received_at <= self.input_timeout_s)
        candidate_fresh = (
            isinstance(self.candidate, dict) and
            self.candidate_received_at is not None and
            now - self.candidate_received_at <= self.input_timeout_s)
        pose = self.pose if fresh else None
        source_name = "explicit_pose"
        if pose is None and candidate_fresh:
            pose = self._candidate_pose(self.candidate)
            source_name = "candidate_affine"
        valid = bool(
            isinstance(pose, dict) and
            pose.get("schema") in ("parking_exit_pose_v1", "parking_exit_metric_v1") and
            pose.get("valid") is True and
            pose.get("frame_id", pose.get("coordinate_frame")) == "base_link" and
            (_normalise_slot_id(pose.get("slot_id")) is not None and
             (self.slot_id is None or
              _normalise_slot_id(pose.get("slot_id")) == self.slot_id)) and
            _number(pose, "confidence") is not None and
            _number(pose, "exit_distance_m") is not None and
            _number(pose, "exit_yaw_error_rad") is not None and
            _number(pose, "exit_lateral_error_m") is not None and
            _bool(pose, "exit_visible") is True
        )
        confidence = _number(pose, "confidence", 0.0) if valid else 0.0
        result = {
            "schema": "parking_exit_metric_v1",
            "stamp": now,
            "frame_id": "base_link",
            "valid": valid,
            "confidence": max(0.0, min(1.0, confidence)),
            "slot_id": (_normalise_slot_id(pose.get("slot_id"))
                        if valid else self.slot_id),
            "exit_visible": bool(valid),
            "exit_distance_m": _number(pose, "exit_distance_m") if valid else None,
            "exit_yaw_error_rad": (_number(pose, "exit_yaw_error_rad")
                                    if valid else None),
            "exit_lateral_error_m": (_number(pose, "exit_lateral_error_m")
                                      if valid else None),
            "exit_complete": (_bool(pose, "exit_complete", False)
                               if valid else False),
            "exit_clear": _bool(pose, "exit_clear") if valid else None,
            "parking_request": False,
            "pose_source": source_name if valid else "none",
        }
        self.publisher.publish(String(
            data=json.dumps(result, separators=(",", ":"), allow_nan=False)))


def main():
    rospy.init_node("parking_exit_metric", anonymous=False)
    try:
        ParkingExitMetricNode()
    except Exception as exc:
        rospy.logfatal("parking_exit_metric cannot start: %s", exc)
        return 2
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
