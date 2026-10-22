#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

"""Detect a reverse-slot centreline and stop line in rear metric BEV.

This is a deliberately conservative geometric detector.  It looks for two
approximately parallel depth lines and a transverse stop line in the already
calibrated rear BEV image.  The output is valid only when all three are
visible and mutually consistent.  It does not use the pixel-only
``/vision/rear_lane`` topic and it does not silently fall back to an
uncalibrated image.  P5 may explicitly opt into a single-side mode because
the field layout can expose one boundary and the transverse stop line only;
that mode uses the measured slot width and never changes the default pair
mode used by other slots.
"""

from __future__ import print_function

import json
import math
import os
import sys
import time

import cv2
import numpy as np
import rospy
import yaml

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CALIBRATION = os.path.abspath(os.path.join(
    SCRIPT_DIR, "..", "calibration", "rear_bev.yaml"))


def _normalise_slot_id(value):
    if not isinstance(value, (str,)):
        try:
            if not isinstance(value, basestring):
                return None
        except NameError:
            return None
    value = value.strip().upper()
    if value in ("", "AUTO", "ANY", "NONE", "NULL"):
        return None
    return value if value in ("P4", "P5") else None


def _finite(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return False
    return not math.isnan(value) and not math.isinf(value)


REAR_PROFILE_TO_SETTING = {
    "rear_expected_slot_width_m": "expected_slot_width_m",
    "rear_vehicle_center_x_m": "vehicle_center_x_m",
    "rear_reference_distance_m": "rear_reference_distance_m",
    "rear_stop_line_bias_m": "stop_line_bias_m",
    "rear_yaw_sign": "yaw_sign",
    "rear_line_color": "line_color",
    "rear_single_side_mode": "single_side_mode",
    "rear_single_side_lateral_sign": "single_side_lateral_sign",
}


def apply_rear_visual_profile(settings, profile):
    """Merge the selected slot's rear-BEV visual calibration.

    The rear camera intrinsics and homography are vehicle-wide calibration
    files.  The values below are deliberately separate per-slot values for
    field paint/line alignment.  Ignore the front/exit keys in the shared
    visual profile, but validate every rear key that is present.
    """
    if profile is None:
        profile = {}
    if not isinstance(profile, dict):
        raise ValueError("rear visual profile must be an object")

    merged = dict(settings)
    for profile_key, setting_key in REAR_PROFILE_TO_SETTING.items():
        if profile_key not in profile:
            continue
        value = profile[profile_key]
        if setting_key == "line_color":
            value = str(value).lower()
            if value not in ("white", "blue"):
                raise ValueError(
                    "visual profile %s must be white or blue" % profile_key)
            merged[setting_key] = value
            continue
        if setting_key == "single_side_mode":
            if not isinstance(value, bool):
                raise ValueError(
                    "visual profile %s must be boolean" % profile_key)
            merged[setting_key] = value
            continue
        if isinstance(value, bool):
            raise ValueError("visual profile %s must be numeric" % profile_key)
        try:
            value = float(value)
        except (TypeError, ValueError):
            raise ValueError("visual profile %s must be numeric" % profile_key)
        if not _finite(value):
            raise ValueError("visual profile %s must be finite" % profile_key)
        merged[setting_key] = value
    return merged


class RearParkingDetector(object):
    """Pure detector core; ROS is only used by :class:`RearParkingNode`."""

    DEFAULTS = {
        "bev_width": 480,
        "bev_height": 400,
        "pixels_per_m": 400.0,
        "bev_x_min_m": -0.60,
        "bev_y_max_m": 1.00,
        "depth_order": "top_row_far_to_bottom_row_near",
        "vehicle_center_x_m": 0.0,
        "camera_to_rear_axle_m": 0.0,
        "stop_line_bias_m": 0.0,
        "rear_reference_distance_m": 0.0,
        "yaw_sign": 1.0,
        "line_color": "white",
        "white_min": 150,
        "white_delta_max": 65,
        "blue_h_min": 90,
        "blue_h_max": 130,
        "blue_s_min": 45,
        "blue_v_min": 35,
        "morph_kernel": 3,
        "hough_threshold": 24,
        "hough_min_line_length_px": 28,
        "hough_max_line_gap_px": 12,
        "side_y_min_m": 0.05,
        "side_y_max_m": 1.00,
        "stop_y_min_m": 0.08,
        "stop_y_max_m": 1.20,
        "side_max_slope": 0.55,
        "stop_max_slope": 0.16,
        "min_slot_width_m": 0.20,
        "max_slot_width_m": 0.50,
        "expected_slot_width_m": 0.38,
        "max_parallel_slope_delta": 0.20,
        "min_stop_span_m": 0.28,
        "stop_center_tolerance_m": 0.22,
        "max_lateral_m": 0.65,
        "max_yaw_rad": 0.75,
        # P5's field view can contain one boundary plus its transverse stop
        # line.  This is disabled by default and must be enabled per slot.
        "single_side_mode": False,
        "single_side_lateral_sign": 1.0,
        "single_side_min_span_m": 0.20,
    }

    def __init__(self, values=None):
        settings = dict(self.DEFAULTS)
        if values:
            settings.update(values)
        self.settings = settings
        self.width = int(settings["bev_width"])
        self.height = int(settings["bev_height"])
        self.px_per_m = float(settings["pixels_per_m"])
        self.x_min_m = float(settings["bev_x_min_m"])
        self.y_max_m = float(settings["bev_y_max_m"])
        self.center_u = self.width / 2.0
        self.validate()

    def validate(self):
        if self.width <= 0 or self.height <= 0 or self.px_per_m <= 0.0:
            raise ValueError("BEV dimensions and pixels_per_m must be positive")
        for key in (
                "bev_x_min_m", "bev_y_max_m", "vehicle_center_x_m",
                "camera_to_rear_axle_m", "stop_line_bias_m",
                "rear_reference_distance_m", "yaw_sign", "side_y_min_m",
                "side_y_max_m", "stop_y_min_m", "stop_y_max_m",
                "side_max_slope", "stop_max_slope", "min_slot_width_m",
                "max_slot_width_m", "expected_slot_width_m",
                "max_parallel_slope_delta", "min_stop_span_m",
                "stop_center_tolerance_m", "max_lateral_m", "max_yaw_rad",
                "single_side_lateral_sign", "single_side_min_span_m"):
            if not _finite(self.settings[key]):
                raise ValueError("%s must be finite" % key)
        if self.settings["line_color"] not in ("white", "blue"):
            raise ValueError("line_color must be white or blue")
        if self.settings["min_slot_width_m"] <= 0.0:
            raise ValueError("min_slot_width_m must be positive")
        if self.settings["max_slot_width_m"] < self.settings["min_slot_width_m"]:
            raise ValueError("max_slot_width_m must not be smaller than min")
        if abs(abs(float(self.settings["single_side_lateral_sign"])) - 1.0) > 1e-6:
            raise ValueError("single_side_lateral_sign must be -1 or +1")
        if float(self.settings["single_side_min_span_m"]) <= 0.0:
            raise ValueError("single_side_min_span_m must be positive")

    def pixel_to_ground(self, u, v):
        """Return ``(distance_behind_camera, lateral_left)`` in metres."""
        right_x = (float(u) - self.center_u) / self.px_per_m
        right_x += float(self.settings["vehicle_center_x_m"])
        lateral_left = -right_x
        if "top_row_near" in str(self.settings["depth_order"]):
            distance = float(v) / self.px_per_m
        else:
            distance = self.y_max_m - float(v) / self.px_per_m
        return distance, lateral_left

    def ground_to_pixel(self, distance, lateral_left):
        right_x = -float(lateral_left) - float(self.settings["vehicle_center_x_m"])
        u = self.center_u + right_x * self.px_per_m
        if "top_row_near" in str(self.settings["depth_order"]):
            v = float(distance) * self.px_per_m
        else:
            v = (self.y_max_m - float(distance)) * self.px_per_m
        return int(round(u)), int(round(v))

    def _line_mask(self, image):
        line_color = str(self.settings["line_color"]).lower()
        if line_color == "blue":
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            lower = np.array([
                int(self.settings["blue_h_min"]),
                int(self.settings["blue_s_min"]),
                int(self.settings["blue_v_min"]),
            ], dtype=np.uint8)
            upper = np.array([
                int(self.settings["blue_h_max"]), 255, 255
            ], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
        else:
            minimum = np.min(image, axis=2)
            maximum = np.max(image, axis=2)
            bright = cv2.inRange(
                minimum, int(self.settings["white_min"]), 255)
            neutral = cv2.inRange(
                maximum - minimum, 0, int(self.settings["white_delta_max"]))
            mask = cv2.bitwise_and(bright, neutral)

        kernel_size = int(self.settings["morph_kernel"])
        if kernel_size > 1:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_RECT, (kernel_size, kernel_size))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _segment(self, line):
        x1, y1, x2, y2 = [float(value) for value in line]
        d1, l1 = self.pixel_to_ground(x1, y1)
        d2, l2 = self.pixel_to_ground(x2, y2)
        return d1, l1, d2, l2

    def _side_candidates(self, lines):
        candidates = []
        for line in lines:
            d1, l1, d2, l2 = self._segment(line)
            delta_d = d2 - d1
            delta_l = l2 - l1
            pixel_dx = abs(float(line[2]) - float(line[0]))
            pixel_dy = abs(float(line[3]) - float(line[1]))
            if pixel_dy < pixel_dx or pixel_dy < 1.0:
                continue
            if abs(delta_d) < 0.04:
                continue
            slope = delta_l / delta_d
            if abs(slope) > float(self.settings["side_max_slope"]):
                continue
            d_min = min(d1, d2)
            d_max = max(d1, d2)
            if d_max < float(self.settings["side_y_min_m"]):
                continue
            if d_min > float(self.settings["side_y_max_m"]):
                continue
            length = math.sqrt(delta_d * delta_d + delta_l * delta_l)
            if length <= 0.05:
                continue
            intercept = l1 - slope * d1
            candidates.append({
                "slope": slope,
                "intercept": intercept,
                "d_min": d_min,
                "d_max": d_max,
                "length": length,
            })
        return candidates

    def _horizontal_candidates(self, lines):
        candidates = []
        for line in lines:
            d1, l1, d2, l2 = self._segment(line)
            delta_d = d2 - d1
            delta_l = l2 - l1
            if abs(delta_l) < abs(delta_d) or abs(delta_l) < 0.05:
                continue
            slope = delta_d / delta_l
            if abs(slope) > float(self.settings["stop_max_slope"]):
                continue
            distance = (d1 + d2) / 2.0
            if distance < float(self.settings["stop_y_min_m"]):
                continue
            if distance > float(self.settings["stop_y_max_m"]):
                continue
            candidates.append({
                "distance": distance,
                "l_min": min(l1, l2),
                "l_max": max(l1, l2),
                "span": abs(delta_l),
            })
        return candidates

    def _best_pair(self, candidates):
        reference = float(self.settings["rear_reference_distance_m"])
        best = None
        for first_index in range(len(candidates)):
            for second_index in range(first_index + 1, len(candidates)):
                first = candidates[first_index]
                second = candidates[second_index]
                if (abs(first["slope"] - second["slope"]) >
                        float(self.settings["max_parallel_slope_delta"])):
                    continue
                first_l = first["slope"] * reference + first["intercept"]
                second_l = second["slope"] * reference + second["intercept"]
                separation = abs(first_l - second_l)
                if separation < float(self.settings["min_slot_width_m"]):
                    continue
                if separation > float(self.settings["max_slot_width_m"]):
                    continue
                width_error = abs(
                    separation - float(self.settings["expected_slot_width_m"]))
                length = first["length"] + second["length"]
                score = length - 2.0 * width_error
                if best is None or score > best["score"]:
                    best = {
                        "first": first,
                        "second": second,
                        "score": score,
                        "separation": separation,
                    }
        return best

    def _best_single_side(self, candidates):
        """Select one boundary for an explicitly configured slot.

        ``single_side_lateral_sign`` identifies the visible boundary in the
        vehicle ground frame: +1 is the left boundary and -1 is the right
        boundary.  The slot centre is inferred by moving half the known slot
        width inward from that boundary.
        """
        if not candidates:
            return None
        minimum_span = float(self.settings["single_side_min_span_m"])
        legal = [candidate for candidate in candidates
                 if candidate["length"] >= minimum_span]
        if not legal:
            return None
        return max(legal, key=lambda candidate: candidate["length"])

    def _single_side_center(self, side, distance):
        side_lateral = side["slope"] * distance + side["intercept"]
        inward_sign = float(self.settings["single_side_lateral_sign"])
        return side_lateral - inward_sign * float(
            self.settings["expected_slot_width_m"]) / 2.0

    def _best_stop(self, stops, pair=None, single_side=None):
        if pair is None and single_side is None:
            return None
        best = None
        for stop in stops:
            centre = (stop["l_min"] + stop["l_max"]) / 2.0
            if pair is not None:
                first = pair["first"]
                second = pair["second"]
                expected = (
                    (first["slope"] * stop["distance"] + first["intercept"]) +
                    (second["slope"] * stop["distance"] + second["intercept"])) / 2.0
            else:
                expected = self._single_side_center(
                    single_side, stop["distance"])
            if abs(centre - expected) > float(
                    self.settings["stop_center_tolerance_m"]):
                continue
            if stop["span"] < float(self.settings["min_stop_span_m"]):
                continue
            score = stop["span"] - abs(centre - expected)
            if best is None or score > best["score"]:
                best = dict(stop)
                best["score"] = score
        return best

    def detect(self, image, stamp=None):
        if stamp is None:
            stamp = time.time()
        empty = {
            "schema": "parking_rear_metric_v1",
            "stamp": float(stamp),
            "frame_id": "base_link",
            "valid": False,
            "confidence": 0.0,
            "slot_id": None,
            "rear_visible": False,
            "rear_distance_m": None,
            "lateral_error_m": None,
            "vehicle_yaw_error_rad": None,
            "rear_clear": None,
            "parking_request": False,
            "geometry_mode": None,
        }
        if image is None or len(image.shape) < 2:
            return empty, None
        if image.shape[1] != self.width or image.shape[0] != self.height:
            return empty, None

        mask = self._line_mask(image)
        lines = cv2.HoughLinesP(
            mask,
            1.0,
            np.pi / 180.0,
            int(self.settings["hough_threshold"]),
            minLineLength=int(self.settings["hough_min_line_length_px"]),
            maxLineGap=int(self.settings["hough_max_line_gap_px"]),
        )
        line_list = [] if lines is None else [line[0] for line in lines]
        sides = self._side_candidates(line_list)
        pair = self._best_pair(sides)
        single_side = None
        if pair is None and bool(self.settings.get("single_side_mode", False)):
            single_side = self._best_single_side(sides)
        stops = self._horizontal_candidates(line_list)
        stop = self._best_stop(stops, pair, single_side)

        overlay = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        if (pair is None and single_side is None) or stop is None:
            return empty, overlay

        ref = float(self.settings["rear_reference_distance_m"])
        if pair is not None:
            first = pair["first"]
            second = pair["second"]
            centre_lateral = (
                (first["slope"] * ref + first["intercept"]) +
                (second["slope"] * ref + second["intercept"])) / 2.0
            mean_slope = (first["slope"] + second["slope"]) / 2.0
            geometry_mode = "parallel_pair"
            geometry_length = (first["length"] + second["length"]) / 2.0
        else:
            centre_lateral = self._single_side_center(single_side, ref)
            mean_slope = single_side["slope"]
            geometry_mode = "single_side"
            geometry_length = single_side["length"]
        yaw = float(self.settings["yaw_sign"]) * math.atan2(
            mean_slope, 1.0)
        distance = (stop["distance"] +
                    float(self.settings["camera_to_rear_axle_m"]) +
                    float(self.settings["stop_line_bias_m"])
                    )
        if pair is not None:
            geometry_score = min(0.35, geometry_length)
            width_score = max(
                0.0, 0.10 - abs(pair["separation"] -
                                 float(self.settings["expected_slot_width_m"])))
        else:
            # The width is a configured field prior in this explicit mode;
            # the lower base score reflects that one boundary is not observed.
            geometry_score = min(0.35, geometry_length * 0.50)
            width_score = 0.10
        confidence = min(1.0, max(0.0,
            0.30 + geometry_score + min(0.20, stop["span"] / 2.0) +
            width_score
        ))
        valid = (distance >= 0.0 and
                 abs(centre_lateral) <= float(self.settings["max_lateral_m"]) and
                 abs(yaw) <= float(self.settings["max_yaw_rad"]))
        result = dict(empty)
        result.update({
            "valid": bool(valid),
            "confidence": float(confidence if valid else 0.0),
            "rear_visible": True,
            "rear_distance_m": float(distance) if valid else None,
            "lateral_error_m": float(centre_lateral) if valid else None,
            "vehicle_yaw_error_rad": float(yaw) if valid else None,
            "geometry_mode": geometry_mode if valid else None,
        })

        draw_candidates = ((first, (0, 255, 0)),
                           (second, (0, 255, 0))) if pair is not None else (
                               (single_side, (255, 255, 0)),)
        for candidate, color in draw_candidates:
            p1 = self.ground_to_pixel(
                candidate["d_min"],
                candidate["slope"] * candidate["d_min"] + candidate["intercept"])
            p2 = self.ground_to_pixel(
                candidate["d_max"],
                candidate["slope"] * candidate["d_max"] + candidate["intercept"])
            cv2.line(overlay, p1, p2, color, 2)
        p1 = self.ground_to_pixel(stop["distance"], stop["l_min"])
        p2 = self.ground_to_pixel(stop["distance"], stop["l_max"])
        cv2.line(overlay, p1, p2, (0, 255, 255), 2)
        cv2.putText(
            overlay,
            "valid=%s mode=%s d=%.2f lat=%.2f yaw=%.2f conf=%.2f" %
            (str(valid), geometry_mode, distance, centre_lateral, yaw,
             confidence),
            (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
            (0, 255, 0) if valid else (0, 0, 255), 1, cv2.LINE_AA)
        return result, overlay


class RearParkingNode(object):
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/debug/rear_bev")
        self.output_topic = rospy.get_param(
            "~output_topic", "/perception/rear_parking_metric")
        self.debug_topic = rospy.get_param(
            "~debug_topic", "/debug/rear_parking_metric")
        self.calibration_file = rospy.get_param(
            "~calibration_file", DEFAULT_CALIBRATION)
        settings = self._load_settings(self.calibration_file)
        for key in RearParkingDetector.DEFAULTS:
            parameter = "~" + key
            if rospy.has_param(parameter):
                settings[key] = rospy.get_param(parameter)
        raw_slot_id = str(rospy.get_param("~slot_id", "P4"))
        self.slot_id = _normalise_slot_id(raw_slot_id)
        self.configured_slot_id = self.slot_id
        slot_profiles = rospy.get_param("~slot_profiles", {})
        if not isinstance(slot_profiles, dict):
            raise ValueError("slot_profiles must be an object")
        self.slot_profiles = slot_profiles
        slot_profile = (slot_profiles.get(self.slot_id, {})
                        if self.slot_id is not None else {})
        settings = apply_rear_visual_profile(settings, slot_profile)
        settings["slot_id"] = self.slot_id
        self.base_settings = dict(settings)
        self.detector = RearParkingDetector(settings)
        self.bridge = CvBridge()
        self.result_pub = rospy.Publisher(self.output_topic, String, queue_size=1)
        self.debug_pub = rospy.Publisher(self.debug_topic, Image, queue_size=1)
        self.target_topic = rospy.get_param(
            "~target_topic", "/perception/parking_target")
        self.target = None
        self.target_received_at = None
        self.selected_slot_id = self.slot_id
        rospy.Subscriber(
            self.image_topic, Image, self._callback, queue_size=1,
            buff_size=2 ** 24)
        if self.configured_slot_id is None:
            rospy.Subscriber(
                self.target_topic, String, self._target_callback, queue_size=1)
        rospy.loginfo(
            "parking_rear_metric ready: image=%s output=%s calibration=%s "
            "slot=%s line_color=%s expected_width=%.3f",
            self.image_topic, self.output_topic, self.calibration_file,
            self.slot_id or "AUTO", str(self.detector.settings["line_color"]),
            float(self.detector.settings["expected_slot_width_m"]))

    def _target_callback(self, message):
        try:
            data = json.loads(message.data)
        except (TypeError, ValueError):
            self.target = None
            self.target_received_at = None
            return
        if not isinstance(data, dict):
            self.target = None
            self.target_received_at = None
            return
        slot_id = _normalise_slot_id(data.get("slot_id"))
        if (data.get("selection_ready") is not True or
                data.get("valid") is not True or slot_id is None):
            self.target = data
            self.target_received_at = time.time()
            return
        if (self.selected_slot_id is not None and
                slot_id != self.selected_slot_id):
            return
        self.target = data
        self.target_received_at = time.time()
        self._activate_slot(slot_id)

    def _activate_slot(self, slot_id):
        if self.configured_slot_id is not None:
            return
        profile = self.slot_profiles.get(slot_id, {})
        settings = apply_rear_visual_profile(self.base_settings, profile)
        settings["slot_id"] = slot_id
        self.detector = RearParkingDetector(settings)
        self.slot_id = slot_id
        self.selected_slot_id = slot_id

    @staticmethod
    def _load_settings(path):
        settings = {}
        if not os.path.isfile(path):
            rospy.logwarn("rear BEV calibration not found: %s", path)
            return settings
        try:
            with open(path, "r") as stream:
                data = yaml.safe_load(stream)
            if isinstance(data, dict):
                settings.update(data)
            else:
                rospy.logwarn("rear BEV calibration is not a mapping: %s", path)
        except Exception as exc:
            rospy.logwarn("cannot read rear BEV calibration %s: %s", path, exc)
        if "pixels_per_metre" in settings and "pixels_per_m" not in settings:
            settings["pixels_per_m"] = settings["pixels_per_metre"]
        return settings

    def _callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, "bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "rear parking BEV conversion failed: %s", exc)
            return
        if self.configured_slot_id is None and self.slot_id is None:
            result = {
                "schema": "parking_rear_metric_v1",
                "stamp": time.time(),
                "frame_id": "base_link",
                "valid": False,
                "confidence": 0.0,
                "slot_id": None,
                "rear_visible": False,
                "rear_distance_m": None,
                "lateral_error_m": None,
                "vehicle_yaw_error_rad": None,
                "rear_clear": None,
                "reason": "waiting_for_selected_slot",
            }
            overlay = None
        else:
            result, overlay = self.detector.detect(image, time.time())
        result["slot_id"] = self.slot_id
        self.result_pub.publish(String(
            data=json.dumps(result, separators=(",", ":"), allow_nan=False)))
        if overlay is not None:
            try:
                debug = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
                debug.header = message.header
                self.debug_pub.publish(debug)
            except CvBridgeError as exc:
                rospy.logwarn_throttle(2.0, "rear parking debug conversion failed: %s", exc)


def main():
    rospy.init_node("parking_rear_metric", anonymous=False)
    try:
        RearParkingNode()
    except Exception as exc:
        rospy.logfatal("parking_rear_metric cannot start: %s", exc)
        return 2
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
