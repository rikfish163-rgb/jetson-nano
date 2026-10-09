#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Normalize the legacy LS01B nearest-point stream for reverse parking.

The existing ``lidar_broadcast`` nodes publish a JSON object containing only
``angle`` (degrees) and ``distance`` (metres) on ``/lidar/send``.  The bridge
forwards that object to ``/lidar/receive``; it does not add validity or an
obstacle decision.  The parking controller must not consume that ambiguous
payload directly, so this module provides one explicit safety contract:

    parking_lidar_v1

The pure ``normalize_lidar_payload`` function is intentionally usable without
ROS.  The ROS node below is only an adapter between String topics; it does not
open the sensor and does not command the vehicle.
"""

from __future__ import division, print_function

import json
import math
import time


try:
    string_types = (basestring,)
except NameError:
    string_types = (str,)


SCHEMA = "parking_lidar_v1"
LEGACY_SCHEMA = "legacy_lidar_nearest_v1"


def _finite(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(value) or math.isinf(value):
        return None
    return value


def _bool_or_none(data, key):
    value = data.get(key) if isinstance(data, dict) else None
    return value if isinstance(value, bool) else None


def _parse_payload(payload):
    if isinstance(payload, dict):
        return dict(payload)
    if not isinstance(payload, string_types):
        return None
    try:
        decoded = json.loads(payload.strip())
    except (TypeError, ValueError):
        return None
    return dict(decoded) if isinstance(decoded, dict) else None


def _normalise_angle_deg(value):
    value = _finite(value)
    if value is None:
        return None
    while value > 180.0:
        value -= 360.0
    while value < -180.0:
        value += 360.0
    return value


def _angle_in_window(angle_deg, minimum, maximum):
    """Return whether an angle is in a possibly wrapping degree interval."""
    if angle_deg is None:
        # An unknown bearing must not be treated as a confirmed clear sector.
        return True
    angle_deg = _normalise_angle_deg(angle_deg)
    minimum = _normalise_angle_deg(minimum)
    maximum = _normalise_angle_deg(maximum)
    if minimum <= maximum:
        return minimum <= angle_deg <= maximum
    return angle_deg >= minimum or angle_deg <= maximum


class LidarAdapterConfig(object):
    """Validated parameters for the legacy-to-parking lidar adapter."""

    DEFAULTS = {
        # The source nodes currently publish ``angle`` in degrees.
        "input_angle_unit": "deg",
        # Obstacle status is a global clearance gate.  The frame label means
        # the result is consumed as a vehicle-level safety fact; no point
        # coordinate is exposed to the controller.
        "output_frame_id": "base_link",
        "min_valid_distance_m": 0.05,
        "max_valid_distance_m": 2.00,
        # Provisional bring-up value.  It must be measured with the actual
        # LS01B mounting, chassis envelope and competition clearance before
        # calibration_complete is enabled.
        "obstacle_distance_m": 0.35,
        "obstacle_angle_min_deg": -180.0,
        "obstacle_angle_max_deg": 180.0,
    }

    def __init__(self, values=None):
        values = {} if values is None else dict(values)
        unknown = sorted(set(values.keys()) - set(self.DEFAULTS.keys()))
        if unknown:
            raise ValueError("unknown lidar adapter parameters: %s" %
                             ", ".join(unknown))
        settings = dict(self.DEFAULTS)
        settings.update(values)
        self.input_angle_unit = settings["input_angle_unit"]
        self.output_frame_id = settings["output_frame_id"]
        self.min_valid_distance_m = _finite(settings["min_valid_distance_m"])
        self.max_valid_distance_m = _finite(settings["max_valid_distance_m"])
        self.obstacle_distance_m = _finite(settings["obstacle_distance_m"])
        self.obstacle_angle_min_deg = _finite(
            settings["obstacle_angle_min_deg"])
        self.obstacle_angle_max_deg = _finite(
            settings["obstacle_angle_max_deg"])
        if self.input_angle_unit not in ("deg", "rad"):
            raise ValueError("input_angle_unit must be deg or rad")
        if (not isinstance(self.output_frame_id, string_types) or
                not self.output_frame_id):
            raise ValueError("output_frame_id must be a non-empty string")
        if (self.min_valid_distance_m is None or
                self.max_valid_distance_m is None or
                self.obstacle_distance_m is None or
                self.min_valid_distance_m < 0.0 or
                self.max_valid_distance_m <= self.min_valid_distance_m or
                self.obstacle_distance_m < self.min_valid_distance_m or
                self.obstacle_distance_m > self.max_valid_distance_m):
            raise ValueError("invalid lidar distance limits")
        if (self.obstacle_angle_min_deg is None or
                self.obstacle_angle_max_deg is None or
                self.obstacle_angle_min_deg < -180.0 or
                self.obstacle_angle_min_deg > 180.0 or
                self.obstacle_angle_max_deg < -180.0 or
                self.obstacle_angle_max_deg > 180.0):
            raise ValueError("lidar obstacle angle limits must be in -180..180")


def _extract_distance(data):
    for key in ("nearest_distance_m", "distance_m", "distance"):
        if key in data:
            return _finite(data.get(key))
    return None


def _extract_angle_deg(data, input_angle_unit):
    if "nearest_angle_deg" in data:
        return _normalise_angle_deg(data.get("nearest_angle_deg"))
    if "angle_deg" in data:
        return _normalise_angle_deg(data.get("angle_deg"))
    if "nearest_angle_rad" in data:
        angle = _finite(data.get("nearest_angle_rad"))
        return _normalise_angle_deg(math.degrees(angle)) if angle is not None else None
    if "angle_rad" in data:
        angle = _finite(data.get("angle_rad"))
        return _normalise_angle_deg(math.degrees(angle)) if angle is not None else None
    if "angle" in data:
        angle = _finite(data.get("angle"))
        if angle is None:
            return None
        if input_angle_unit == "rad":
            angle = math.degrees(angle)
        return _normalise_angle_deg(angle)
    return None


def normalize_lidar_payload(payload, now=None, config=None):
    """Return a strict ``parking_lidar_v1`` payload.

    ``valid`` means the source supplied a finite distance inside the sensor's
    configured usable range and did not explicitly mark itself invalid.
    Missing legacy validity is allowed because the old source has no such
    field; missing distance, malformed validity, or an out-of-range sentinel
    (the old visualizer uses ``999``) is invalid and therefore blocks motion.
    """
    if now is None:
        now = time.time()
    now = float(now)
    settings = config if isinstance(config, LidarAdapterConfig) else \
        LidarAdapterConfig(config)
    data = _parse_payload(payload)
    source_schema = (data.get("schema") if isinstance(data, dict) else None)
    if not source_schema:
        source_schema = LEGACY_SCHEMA

    distance = _extract_distance(data) if isinstance(data, dict) else None
    angle_deg = (_extract_angle_deg(data, settings.input_angle_unit)
                 if isinstance(data, dict) else None)
    source_valid = (_bool_or_none(data, "valid")
                    if isinstance(data, dict) else None)
    source_obstacle = (_bool_or_none(data, "obstacle_detected")
                       if isinstance(data, dict) else None)
    source_emergency = (_bool_or_none(data, "emergency_stop")
                        if isinstance(data, dict) else None)

    malformed_flags = bool(
        isinstance(data, dict) and
        (("valid" in data and source_valid is None) or
         ("obstacle_detected" in data and source_obstacle is None) or
         ("emergency_stop" in data and source_emergency is None)))
    range_valid = bool(
        distance is not None and
        settings.min_valid_distance_m <= distance <=
        settings.max_valid_distance_m)
    valid = bool(data is not None and not malformed_flags and range_valid and
                 source_valid is not False)

    threshold_hit = bool(
        range_valid and distance <= settings.obstacle_distance_m and
        _angle_in_window(
            angle_deg,
            settings.obstacle_angle_min_deg,
            settings.obstacle_angle_max_deg))
    obstacle_detected = bool(
        source_obstacle is True or threshold_hit)
    emergency_stop = bool(source_emergency is True)

    if data is None:
        reason = "payload_invalid"
    elif malformed_flags:
        reason = "payload_field_type_invalid"
    elif not range_valid:
        reason = "distance_invalid"
    elif emergency_stop:
        reason = "emergency_stop"
    elif obstacle_detected:
        reason = "obstacle_detected"
    elif source_obstacle is False:
        reason = "source_clear"
    else:
        reason = "clear"

    result = {
        "schema": SCHEMA,
        "stamp": now,
        "frame_id": settings.output_frame_id,
        "coordinate_frame": settings.output_frame_id,
        "valid": valid,
        "obstacle_detected": obstacle_detected,
        "emergency_stop": emergency_stop,
        "nearest_distance_m": distance,
        "nearest_angle_deg": angle_deg,
        "nearest_angle_rad": (math.radians(angle_deg)
                               if angle_deg is not None else None),
        "source_schema": source_schema,
        "source_valid": source_valid,
        "source_obstacle_detected": source_obstacle,
        "threshold_distance_m": settings.obstacle_distance_m,
        "obstacle_angle_min_deg": settings.obstacle_angle_min_deg,
        "obstacle_angle_max_deg": settings.obstacle_angle_max_deg,
        "reason": reason,
    }
    if isinstance(data, dict):
        source_stamp = _finite(data.get("stamp"))
        if source_stamp is not None:
            result["source_stamp"] = source_stamp
    return result


try:
    import rospy
    from std_msgs.msg import String
except ImportError:  # Keep the pure normalizer importable on a workstation.
    rospy = None
    String = None


class ParkingLidarAdapterNode(object):
    """ROS String-to-String adapter; it owns no motion command topic."""

    def __init__(self):
        if rospy is None or String is None:
            raise RuntimeError("ROS std_msgs is required for the adapter node")
        values = {
            "input_angle_unit": rospy.get_param("~input_angle_unit", "deg"),
            "output_frame_id": rospy.get_param(
                "~output_frame_id", "base_link"),
            "min_valid_distance_m": rospy.get_param(
                "~min_valid_distance_m", 0.05),
            "max_valid_distance_m": rospy.get_param(
                "~max_valid_distance_m", 2.00),
            "obstacle_distance_m": rospy.get_param(
                "~obstacle_distance_m", 0.35),
            "obstacle_angle_min_deg": rospy.get_param(
                "~obstacle_angle_min_deg", -180.0),
            "obstacle_angle_max_deg": rospy.get_param(
                "~obstacle_angle_max_deg", 180.0),
        }
        self.config = LidarAdapterConfig(values)
        self.input_topic = rospy.get_param("~input_topic", "/lidar/receive")
        self.output_topic = rospy.get_param("~output_topic", "/parking/lidar")
        self.publisher = rospy.Publisher(self.output_topic, String, queue_size=1)
        self.subscriber = rospy.Subscriber(
            self.input_topic, String, self._callback, queue_size=1)
        rospy.loginfo(
            "parking_lidar_adapter ready: %s -> %s, threshold=%.3fm",
            self.input_topic, self.output_topic,
            self.config.obstacle_distance_m)

    def _callback(self, message):
        normalized = normalize_lidar_payload(
            message.data, now=time.time(), config=self.config)
        output = String()
        output.data = json.dumps(
            normalized, separators=(",", ":"), allow_nan=False)
        self.publisher.publish(output)


def main():
    if rospy is None:
        raise RuntimeError("ROS is required to run parking_lidar_adapter")
    rospy.init_node("parking_lidar_adapter", anonymous=False)
    try:
        ParkingLidarAdapterNode()
    except Exception as exc:
        rospy.logfatal("parking_lidar_adapter cannot start: %s", exc)
        return 2
    rospy.spin()
    return 0


if __name__ == "__main__":
    main()
