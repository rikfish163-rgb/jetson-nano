#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Turn the existing metric front Path into an optional parking pose source.

The existing front node already publishes a ``nav_msgs/Path`` in
``base_link``.  This adapter fits its near, trusted points and exposes the
result as a parking pose.  Because an ordinary road lane is not automatically
the P4/P5 slot axis, the yaw/lateral biases are explicit parameters and the
node is opt-in in the launch file.
"""

from __future__ import division, print_function

import json
import math
import sys
import time

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Float32, String


def _normalise_slot_id(value):
    """Return P4/P5 or None for the dynamic AUTO configuration."""
    try:
        slot_id = str(value).strip().upper()
    except (TypeError, ValueError):
        slot_id = ""
    if slot_id in ("", "AUTO", "ANY", "NONE", "UNSELECTED"):
        return None
    if slot_id not in ("P4", "P5"):
        raise ValueError("slot_id must be P4, P5, or AUTO")
    return slot_id


def fit_path(points):
    """Fit ``y = slope*x + intercept`` to finite base_link points."""
    if not isinstance(points, list) or len(points) < 3:
        return None
    xs = []
    ys = []
    for point in points:
        try:
            x = float(point[0])
            y = float(point[1])
        except (TypeError, ValueError, IndexError):
            continue
        if math.isnan(x) or math.isnan(y) or math.isinf(x) or math.isinf(y):
            continue
        xs.append(x)
        ys.append(y)
    if len(xs) < 3:
        return None
    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    denominator = sum((x - mean_x) ** 2 for x in xs)
    if denominator <= 1.0e-6:
        return None
    slope = sum((x - mean_x) * (y - mean_y)
                for x, y in zip(xs, ys)) / denominator
    return {
        "slope": float(slope),
        "intercept": float(mean_y - slope * mean_x),
        "min_x": float(min(xs)),
        "max_x": float(max(xs)),
        "point_count": len(xs),
    }


def _profile_number(profile, key, default):
    """Read one optional per-slot lane-pose calibration value."""
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


class ParkingLanePoseNode(object):
    def __init__(self):
        self.path_topic = rospy.get_param("~path_topic", "/vision/lane_path")
        self.confidence_topic = rospy.get_param(
            "~confidence_topic", "/vision/lane_confidence")
        self.front_output_topic = rospy.get_param(
            "~front_output_topic", "/perception/front_slot_pose")
        self.exit_output_topic = rospy.get_param(
            "~exit_output_topic", "/perception/exit_pose")
        self.configured_slot_id = _normalise_slot_id(
            rospy.get_param("~slot_id", "P4"))
        self.slot_id = self.configured_slot_id
        self.target_topic = rospy.get_param(
            "~target_topic", "/perception/parking_target")
        self.input_timeout_s = float(
            rospy.get_param("~input_timeout_s", 0.50))
        self.min_confidence = float(
            rospy.get_param("~min_confidence", 0.60))
        self.min_x_m = float(rospy.get_param("~min_x_m", 0.05))
        self.max_x_m = float(rospy.get_param("~max_x_m", 1.20))
        self.yaw_sign = float(rospy.get_param("~yaw_sign", -1.0))
        self.slot_yaw_bias_rad = float(
            rospy.get_param("~slot_yaw_bias_rad", 0.0))
        self.slot_lateral_offset_m = float(
            rospy.get_param("~slot_lateral_offset_m", 0.0))
        self.default_yaw_sign = self.yaw_sign
        self.default_slot_yaw_bias_rad = self.slot_yaw_bias_rad
        self.default_slot_lateral_offset_m = self.slot_lateral_offset_m
        self.reverse_ready_yaw_tolerance_rad = float(
            rospy.get_param("~reverse_ready_yaw_tolerance_rad", 0.10))
        self.reverse_ready_lateral_tolerance_m = float(
            rospy.get_param("~reverse_ready_lateral_tolerance_m", 0.10))
        self.enable_front_pose = bool(
            rospy.get_param("~enable_front_pose", False))
        self.enable_exit_pose = bool(
            rospy.get_param("~enable_exit_pose", False))
        self.exit_distance_m = rospy.get_param("~exit_distance_m", None)
        # When no fixed exit distance is supplied, use the nearest point of
        # the fitted white lane path as a live remaining-distance signal.
        # The offset is a field-tunable safety point measured in base_link.
        self.exit_distance_offset_m = float(
            rospy.get_param("~exit_distance_offset_m", 0.0))
        self.slot_profiles = rospy.get_param("~slot_profiles", {})
        if not isinstance(self.slot_profiles, dict):
            raise ValueError("slot_profiles must be an object")
        self._apply_slot_profile(self.slot_id)
        if self.input_timeout_s <= 0.0:
            raise ValueError("input_timeout_s must be positive")
        if self.min_confidence < 0.0 or self.min_confidence > 1.0:
            raise ValueError("min_confidence must be in the range 0..1")
        if not _finite(self.exit_distance_offset_m):
            raise ValueError("exit_distance_offset_m must be finite")
        if (_finite(self.exit_distance_m) and
                float(self.exit_distance_m) < 0.0):
            raise ValueError("exit_distance_m must be non-negative")

        self.path = None
        self.path_received_at = None
        self.confidence = 0.0
        self.confidence_received_at = None
        self.selected_slot_id = self.slot_id
        self.front_pub = rospy.Publisher(
            self.front_output_topic, String, queue_size=1)
        self.exit_pub = rospy.Publisher(
            self.exit_output_topic, String, queue_size=1)
        rospy.Subscriber(self.path_topic, Path, self._path_callback, queue_size=1)
        rospy.Subscriber(
            self.confidence_topic, Float32, self._confidence_callback,
            queue_size=1)
        if self.configured_slot_id is None:
            rospy.Subscriber(
                self.target_topic, String, self._target_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.05), self._publish)
        rospy.loginfo(
            "parking_lane_pose ready: path=%s front=%s exit=%s front_enabled=%s",
            self.path_topic, self.front_output_topic, self.exit_output_topic,
            str(self.enable_front_pose))

    def _apply_slot_profile(self, slot_id):
        """Apply the visual profile for a concrete, selected slot."""
        slot_profile = self.slot_profiles.get(slot_id, {}) if slot_id else {}
        if not isinstance(slot_profile, dict):
            raise ValueError("visual profile %s must be an object" % slot_id)
        self.slot_id = slot_id
        self.yaw_sign = _profile_number(
            slot_profile, "lane_yaw_sign", self.default_yaw_sign)
        self.slot_yaw_bias_rad = _profile_number(
            slot_profile, "lane_slot_yaw_bias_rad",
            self.default_slot_yaw_bias_rad)
        self.slot_lateral_offset_m = _profile_number(
            slot_profile, "lane_slot_lateral_offset_m",
            self.default_slot_lateral_offset_m)

    def _target_callback(self, message):
        """Latch the target selector's concrete P4/P5 choice in AUTO mode."""
        try:
            data = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(data, dict) or not data.get("selection_ready", False):
            return
        try:
            slot_id = _normalise_slot_id(
                data.get("selected_slot_id", data.get("slot_id")))
        except ValueError:
            return
        if slot_id is None:
            return
        if (self.selected_slot_id is not None and
                slot_id != self.selected_slot_id):
            return
        try:
            self._apply_slot_profile(slot_id)
            self.selected_slot_id = slot_id
        except ValueError as exc:
            rospy.logwarn_throttle(
                2.0, "parking_lane_pose ignored slot profile: %s", exc)

    def _path_callback(self, message):
        if message.header.frame_id != "base_link":
            self.path = None
            self.path_received_at = None
            return
        points = []
        for pose in message.poses:
            if pose.header.frame_id not in ("", "base_link"):
                continue
            x = float(pose.pose.position.x)
            y = float(pose.pose.position.y)
            if x < self.min_x_m or x > self.max_x_m:
                continue
            points.append((x, y))
        self.path = points
        self.path_received_at = time.time()

    def _confidence_callback(self, message):
        self.confidence = max(0.0, min(1.0, float(message.data)))
        self.confidence_received_at = time.time()

    def _fit(self, now):
        if (self.path_received_at is None or self.path is None or
                now - self.path_received_at > self.input_timeout_s):
            return None
        if (self.confidence_received_at is None or
                now - self.confidence_received_at > self.input_timeout_s or
                self.confidence < self.min_confidence):
            return None
        return fit_path(self.path)

    def _base(self, fit):
        if fit is None:
            return {"valid": False, "confidence": 0.0}
        lane_angle = math.atan(fit["slope"])
        yaw_error = self.yaw_sign * (lane_angle + self.slot_yaw_bias_rad)
        # For a target line at y_target, the vehicle at y=0 is to its right
        # when y_target is positive-left; the controller's error is vehicle
        # lateral position relative to that target.
        lateral_error = -(fit["intercept"] + self.slot_lateral_offset_m)
        return {
            "valid": True,
            "confidence": float(self.confidence),
            "vehicle_yaw_error_rad": float(yaw_error),
            "lateral_error_m": float(lateral_error),
            "path_point_count": int(fit["point_count"]),
            "path_min_x_m": float(fit["min_x"]),
            "path_max_x_m": float(fit["max_x"]),
        }

    def _publish(self, _event):
        if rospy.is_shutdown():
            return
        now = time.time()
        fit = self._fit(now)
        base = self._base(fit)
        front = {
            "schema": "parking_front_pose_v1",
            "stamp": now,
            "frame_id": "base_link",
            "valid": bool(self.enable_front_pose and fit is not None and
                           self.slot_id is not None),
            "confidence": base["confidence"]
            if fit is not None and self.slot_id is not None else 0.0,
            "slot_id": self.slot_id,
            "vehicle_yaw_error_rad": base.get("vehicle_yaw_error_rad"),
            "lateral_error_m": base.get("lateral_error_m"),
            "reverse_ready": bool(
                fit is not None and self.slot_id is not None and
                abs(base["vehicle_yaw_error_rad"]) <=
                self.reverse_ready_yaw_tolerance_rad and
                abs(base["lateral_error_m"]) <=
                self.reverse_ready_lateral_tolerance_m),
            "parking_request": False,
            "slot_visible": fit is not None,
            "front_clear": None,
        }
        self.front_pub.publish(String(
            data=json.dumps(front, separators=(",", ":"), allow_nan=False)))

        if _finite(self.exit_distance_m):
            exit_distance = float(self.exit_distance_m)
            exit_distance_source = "fixed"
        elif fit is not None:
            exit_distance = max(
                0.0, float(fit["min_x"]) - self.exit_distance_offset_m)
            exit_distance_source = "lane_path_min_x"
        else:
            exit_distance = None
            exit_distance_source = "none"
        exit_valid = bool(
            self.enable_exit_pose and fit is not None and
            self.slot_id is not None and
            _finite(exit_distance))
        exit_data = {
            "schema": "parking_exit_pose_v1",
            "stamp": now,
            "frame_id": "base_link",
            "valid": exit_valid,
            "confidence": base["confidence"] if exit_valid else 0.0,
            "slot_id": self.slot_id,
            "exit_visible": exit_valid,
            "exit_distance_m": float(exit_distance)
            if exit_valid else None,
            "exit_yaw_error_rad": base.get("vehicle_yaw_error_rad")
            if exit_valid else None,
            "exit_lateral_error_m": base.get("lateral_error_m")
            if exit_valid else None,
            "exit_complete": False,
            "exit_clear": None,
            "exit_distance_source": exit_distance_source,
            "path_min_x_m": base.get("path_min_x_m") if fit is not None else None,
            "path_max_x_m": base.get("path_max_x_m") if fit is not None else None,
        }
        self.exit_pub.publish(String(
            data=json.dumps(exit_data, separators=(",", ":"), allow_nan=False)))


def _finite(value):
    if value is None or isinstance(value, bool):
        return False
    try:
        value = float(value)
    except (TypeError, ValueError):
        return False
    return not math.isnan(value) and not math.isinf(value)


def main():
    rospy.init_node("parking_lane_pose", anonymous=False)
    try:
        ParkingLanePoseNode()
    except Exception as exc:
        rospy.logfatal("parking_lane_pose cannot start: %s", exc)
        return 2
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
