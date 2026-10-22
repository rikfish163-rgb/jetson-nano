#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Fuse the selected white bay target with the independent blue start line.

The target selector supplies white bay geometry, a concrete P4/P5 identity,
and the lidar-clear selection gate.  It also forwards the independent blue
start-line visibility and distance.  The blue line is only an approach/ FSM
trigger; it is never used as the bay identity or slot pose.  A separate calibrated
``parking_front_pose_v1`` source may therefore be supplied on ``pose_topic``
for the slot heading, lateral error and ``reverse_ready`` gate.
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


def _profile_number(profile, key, default):
    """Read one optional per-slot visual calibration value strictly."""
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


def _payload(message_data):
    if not isinstance(message_data, string_types):
        return None
    try:
        data = json.loads(message_data)
    except (TypeError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def _normalise_slot_id(value):
    if not isinstance(value, string_types):
        return None
    value = value.strip().upper()
    if value in ("", "AUTO", "ANY", "NONE", "NULL"):
        return None
    return value if value in ("P4", "P5") else None


class ParkingFrontMetricNode(object):
    def __init__(self):
        self.candidate_topic = rospy.get_param(
            "~candidate_topic", "/perception/parking_target")
        self.pose_topic = rospy.get_param(
            "~pose_topic", "/perception/front_slot_pose")
        self.output_topic = rospy.get_param(
            "~output_topic", "/perception/front_parking_metric")
        raw_slot_id = str(rospy.get_param("~slot_id", "P4"))
        # AUTO is a launch sentinel.  The target selector supplies the
        # concrete P4/P5 id in each candidate payload.
        self.slot_id = _normalise_slot_id(raw_slot_id)
        self.input_timeout_s = float(
            rospy.get_param("~input_timeout_s", 0.50))
        self.trigger_offset_m = float(
            rospy.get_param("~trigger_offset_m", 0.0))
        self.candidate_confidence = float(
            rospy.get_param("~candidate_confidence", 0.75))
        self.slot_profiles = rospy.get_param("~slot_profiles", {})
        if not isinstance(self.slot_profiles, dict):
            raise ValueError("slot_profiles must be an object")
        slot_profile = (self.slot_profiles.get(self.slot_id, {})
                        if self.slot_id is not None else {})
        if not isinstance(slot_profile, dict):
            raise ValueError("visual profile %s must be an object" % self.slot_id)
        # A white bay candidate's line angle is not intrinsically a slot
        # heading, so deriving a pose from it is opt-in and calibrated by an explicit
        # affine mapping.  This keeps the node useful for the approach trigger
        # even before the P4/P5 marker role has been measured.
        self.derive_pose_from_candidate = bool(
            rospy.get_param("~derive_pose_from_candidate", False))
        self.pose_yaw_gain = float(
            rospy.get_param("~candidate_yaw_gain", 1.0))
        self.pose_yaw_bias_rad = float(
            rospy.get_param("~candidate_yaw_bias_rad", 0.0))
        self.pose_lateral_gain = float(
            rospy.get_param("~candidate_lateral_gain", 1.0))
        self.pose_lateral_bias_m = float(
            rospy.get_param("~candidate_lateral_bias_m", 0.0))
        self.pose_max_yaw_rad = float(
            rospy.get_param("~candidate_max_yaw_rad", 1.50))
        self.pose_max_lateral_m = float(
            rospy.get_param("~candidate_max_lateral_m", 0.80))
        self.pose_yaw_gain = _profile_number(
            slot_profile, "candidate_yaw_gain", self.pose_yaw_gain)
        self.pose_yaw_bias_rad = _profile_number(
            slot_profile, "candidate_yaw_bias_rad", self.pose_yaw_bias_rad)
        self.pose_lateral_gain = _profile_number(
            slot_profile, "candidate_lateral_gain", self.pose_lateral_gain)
        self.pose_lateral_bias_m = _profile_number(
            slot_profile, "candidate_lateral_bias_m", self.pose_lateral_bias_m)
        if self.input_timeout_s <= 0.0:
            raise ValueError("input_timeout_s must be positive")
        if self.candidate_confidence < 0.0 or self.candidate_confidence > 1.0:
            raise ValueError("candidate_confidence must be in the range 0..1")
        if self.pose_max_yaw_rad <= 0.0 or self.pose_max_lateral_m <= 0.0:
            raise ValueError("candidate pose limits must be positive")

        self.candidate = None
        self.candidate_received_at = None
        self.pose = None
        self.pose_received_at = None
        self.publisher = rospy.Publisher(
            self.output_topic, String, queue_size=1)
        rospy.Subscriber(
            self.candidate_topic, String, self._candidate_callback,
            queue_size=1)
        rospy.Subscriber(
            self.pose_topic, String, self._pose_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.05), self._publish)
        rospy.loginfo(
            "parking_front_metric ready: candidate=%s pose=%s output=%s",
            self.candidate_topic, self.pose_topic, self.output_topic)

    def _candidate_callback(self, message):
        data = _payload(message.data)
        if data is None:
            self.candidate = None
            self.candidate_received_at = None
            return
        self.candidate = data
        self.candidate_received_at = time.time()

    def _pose_callback(self, message):
        data = _payload(message.data)
        if data is None:
            self.pose = None
            self.pose_received_at = None
            return
        self.pose = data
        self.pose_received_at = time.time()

    def _fresh(self, value, received_at, now):
        return (isinstance(value, dict) and received_at is not None and
                now - received_at <= self.input_timeout_s)

    def _candidate_pose(self, candidate):
        """Return a calibrated pose derived from the marker, or ``None``.

        ``blue_marker_metric`` fits ``v=a*u+b`` in the front metric BEV and
        publishes ``angle_deg=atan(a)``.  The mapping below intentionally does
        not claim that this image angle is already the slot yaw.  For a marker
        perpendicular to the bay axis, for example, the calibrated bias is
        approximately +/-pi/2; the exact value and sign must be measured from
        a P4/P5 BEV frame.
        """
        if not self.derive_pose_from_candidate:
            return None
        angle_deg = _number(candidate, "angle_deg")
        lateral_m = _number(candidate, "lateral_m")
        if angle_deg is None or lateral_m is None:
            return None
        candidate_slot = _normalise_slot_id(candidate.get("slot_id"))
        profile = (self.slot_profiles.get(candidate_slot, {})
                   if candidate_slot is not None else {})
        yaw_gain = _profile_number(
            profile, "candidate_yaw_gain", self.pose_yaw_gain)
        yaw_bias = _profile_number(
            profile, "candidate_yaw_bias_rad", self.pose_yaw_bias_rad)
        lateral_gain = _profile_number(
            profile, "candidate_lateral_gain", self.pose_lateral_gain)
        lateral_bias = _profile_number(
            profile, "candidate_lateral_bias_m", self.pose_lateral_bias_m)
        yaw_error = yaw_gain * math.radians(angle_deg) + yaw_bias
        lateral_error = lateral_gain * lateral_m + lateral_bias
        if abs(yaw_error) > self.pose_max_yaw_rad:
            return None
        if abs(lateral_error) > self.pose_max_lateral_m:
            return None
        return {
            "vehicle_yaw_error_rad": float(yaw_error),
            "lateral_error_m": float(lateral_error),
            "reverse_ready": bool(
                abs(yaw_error) <= float(
                    rospy.get_param("~reverse_ready_yaw_tolerance_rad", 0.10)) and
                abs(lateral_error) <= float(
                    rospy.get_param("~reverse_ready_lateral_tolerance_m", 0.10))),
            "slot_id": candidate_slot or self.slot_id,
        }

    def _publish(self, _event):
        if rospy.is_shutdown():
            return
        now = time.time()
        candidate = (self.candidate if self._fresh(
            self.candidate, self.candidate_received_at, now) else None)
        pose = (self.pose if self._fresh(
            self.pose, self.pose_received_at, now) else None)

        target_payload = bool(
            isinstance(candidate, dict) and
            candidate.get("schema") == "parking_target_v1")
        candidate_slot = (_normalise_slot_id(
            candidate.get("selected_slot_id", candidate.get("slot_id")))
            if isinstance(candidate, dict) else None)
        candidate_slot_valid = (
            (self.slot_id is None and candidate_slot is not None) or
            (self.slot_id is not None and
             candidate_slot in (None, self.slot_id)))
        # The reform target is accepted only after white bay geometry, lidar,
        # and the concrete P4/P5 selection are all ready.  Legacy blue-marker
        # payloads remain accepted for old replay/launch compositions, but
        # they are never produced by the reform launch.
        candidate_selection_ready = (candidate.get("selection_ready")
                                     if isinstance(candidate, dict) else None)
        if target_payload:
            candidate_selection_valid = bool(
                candidate.get("valid") is True and
                candidate_selection_ready is True and
                candidate.get("lidar_valid") is True and
                candidate.get("slot_obstacle_detected") is False)
        else:
            candidate_selection_valid = candidate_selection_ready is not False
        candidate_valid = bool(
            candidate and candidate.get("candidate") is True and
            candidate_slot_valid and candidate_selection_valid)
        if target_payload:
            start_line_distance = _number(
                candidate, "start_line_distance_m")
            if start_line_distance is None:
                start_line_distance = _number(candidate, "turn_distance_m")
            blue_trigger_visible = bool(
                candidate_valid and
                candidate.get("blue_trigger_visible") is True)
        else:
            start_line_distance = None
            blue_trigger_visible = bool(
                candidate_valid and
                (candidate.get("blue_trigger_visible", True)
                 if isinstance(candidate, dict) else False))
        # Keep raw start-line visibility as a diagnostic field.  The actual
        # trigger remains ``blue_trigger_visible`` above, which is gated by a
        # valid, selected, lidar-clear target.
        start_line_visible = bool(
            target_payload and
            isinstance(candidate, dict) and
            candidate.get("start_line_visible") is True)
        distance = (start_line_distance if target_payload else
                    _number(candidate, "distance_m"))
        if distance is not None:
            distance = max(0.0, distance - self.trigger_offset_m)

        candidate_pose = self._candidate_pose(candidate) if candidate_valid else None

        pose_valid = bool(
            isinstance(pose, dict) and
            pose.get("schema") == "parking_front_pose_v1" and
            pose.get("valid") is True and
            pose.get("frame_id", pose.get("coordinate_frame")) == "base_link"
        )
        # The blue start line is the approach trigger, not the lifetime of the
        # front pose.  Once the car starts turning it may leave the camera FOV
        # while an independently calibrated pose source remains fresh.  Keep
        # that pose alive and let the controller require turn_distance_m only
        # in the approach state.
        confidence = self.candidate_confidence if candidate_valid else 0.0
        if pose_valid:
            pose_confidence = _number(pose, "confidence", 0.0)
            pose_confidence = max(0.0, min(1.0, pose_confidence))
            confidence = (min(confidence, pose_confidence)
                          if candidate_valid else pose_confidence)

        pose_source = pose if pose_valid else candidate_pose
        derived_pose_valid = candidate_pose is not None
        if derived_pose_valid:
            confidence = min(confidence, self.candidate_confidence)

        result = {
            "schema": "parking_front_metric_v1",
            "stamp": now,
            "frame_id": "base_link",
            "valid": bool(candidate_valid or pose_valid or derived_pose_valid),
            "confidence": float(confidence),
            "slot_id": (pose.get("slot_id", self.slot_id)
                         if pose_valid else
                         pose_source.get("slot_id", self.slot_id)
                         if isinstance(pose_source, dict) else self.slot_id),
            "parking_side": (candidate.get("parking_side")
                              if isinstance(candidate, dict) else None),
            "turn_side": (candidate.get("turn_side", candidate.get("parking_side"))
                          if isinstance(candidate, dict) else None),
            "slot_visible": bool(
                (candidate_valid or pose_valid or derived_pose_valid) and
                (pose.get("slot_visible", True)
                 if pose_valid else True)),
            # This field is intentionally narrower than slot_visible.  A
            # white bay pose can keep setup control alive, but only the
            # independent blue start-line result can release the FSM.
            "parking_trigger_visible": blue_trigger_visible,
            "turn_distance_m": distance,
            "start_line_visible": start_line_visible,
            "start_line_distance_m": start_line_distance,
            # Do not use blue_marker_metric.angle_deg here.  It is the marker
            # line angle, not a calibrated slot-axis angle.
            "vehicle_yaw_error_rad": (_number(
                pose, "vehicle_yaw_error_rad") if pose_valid else
                pose_source.get("vehicle_yaw_error_rad")
                if derived_pose_valid else None),
            "lateral_error_m": (_number(
                pose, "lateral_error_m") if pose_valid else
                pose_source.get("lateral_error_m")
                if derived_pose_valid else None),
            "reverse_ready": (pose.get("reverse_ready") is True
                               if pose_valid else
                               pose_source.get("reverse_ready") is True
                               if derived_pose_valid else False),
            "parking_request": (pose.get("parking_request") is True
                                if pose_valid else False),
            "front_clear": (_bool(pose, "front_clear") if pose_valid else None),
            "candidate_distance_m": _number(candidate, "distance_m"),
            "candidate_lateral_m": _number(candidate, "lateral_m"),
            "candidate_angle_deg": _number(candidate, "angle_deg"),
            "candidate_count": int(_number(candidate, "candidate_count", 0)),
            "candidate_selection_mode": (candidate.get("selection_mode")
                                          if isinstance(candidate, dict)
                                          else None),
            "candidate_selection_ready": bool(
                candidate_valid and candidate_selection_valid),
            "blue_trigger_visible": blue_trigger_visible,
            "slot_selection_ready": bool(
                candidate_valid and candidate_selection_valid),
            "selected_slot_id": candidate_slot if candidate_valid else None,
            "slot_obstacle_detected": (
                candidate.get("slot_obstacle_detected")
                if candidate_valid and isinstance(candidate, dict)
                else None),
            "target_reason": (candidate.get("target_reason")
                              if isinstance(candidate, dict) else None),
            "pose_source": ("explicit_pose" if pose_valid else
                            "candidate_affine" if derived_pose_valid else
                            "none"),
        }
        message = String()
        message.data = json.dumps(result, separators=(",", ":"), allow_nan=False)
        self.publisher.publish(message)


def _bool(data, key):
    value = data.get(key) if isinstance(data, dict) else None
    return value if isinstance(value, bool) else None


def main():
    rospy.init_node("parking_front_metric", anonymous=False)
    try:
        ParkingFrontMetricNode()
    except Exception as exc:
        rospy.logfatal("parking_front_metric cannot start: %s", exc)
        return 2
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
