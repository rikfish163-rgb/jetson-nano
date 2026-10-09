#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
from __future__ import print_function

import json
import math
import hashlib

import cv2
import numpy as np
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String


try:
    string_types = (basestring,)
except NameError:
    string_types = (str,)


FRONT_CAMERA_TOPIC = "/front/usb_cam/image_raw"
METRIC_MASK_TOPIC = "/debug/metric_bev"
CANDIDATE_TOPIC = "/perception/blue_marker_candidate"
DEBUG_MASK_TOPIC = "/debug/blue_metric_mask"
DEBUG_OVERLAY_TOPIC = "/debug/blue_marker_overlay"
INPUT_MODE = "blue_raw"
OUTPUT_ROLE = "legacy"

ROLE_SCHEMAS = {
    "legacy": "blue_marker_candidate_v1",
    "start_line": "parking_start_line_v1",
    "slot": "parking_slot_candidate_v1",
}


# Copied element-for-element from the active front-camera implementation below.
CALIBRATION_SOURCE = "/home/nano/robocup_ws/src/ros/camera/scripts/camera_yihan_web.py"
# Pin the provenance to that source so a normal parking bring-up reports any
# accidental divergence between its calibration and the active lane node.
CALIBRATION_SOURCE_SHA256 = "2cc4edf69c148525e265cba97935f6d60b22fd908afd88084bb0ba40f0eb5232"

CAMERA_SIZE = (640, 360)
BEV_SIZE = (480, 400)

K = np.array([
    [338.724360, 0.0, 342.299347],
    [0.0, 338.759483, 169.956657],
    [0.0, 0.0, 1.0]
], dtype=np.float64)

D = np.array([
    -0.313909,
     0.080347,
     0.001009,
    -0.000759,
     0.0
], dtype=np.float64)

H_METRIC = np.array([
    [-0.455307634051, -1.851847127629, 416.759846959926],
    [ 0.072741868805, -3.467660494260, 575.376931923200],
    [ 0.000124867696, -0.007523542869,   1.000000000000]
], dtype=np.float64)

BEV_WIDTH = 480
BEV_HEIGHT = 400
PX_PER_CM = 4.0
PX_PER_M = PX_PER_CM * 100.0
VEHICLE_CENTER_U = 240.0
VEHICLE_REAR_V = 600.0


# Color candidate parameters retained from camera_blue_debug.py.
FLOOR_Y_RATIO = 0.2
BLUE_H_MIN = 90
BLUE_H_MAX = 130
BLUE_S_MIN = 30
BLUE_V_MIN = 15
MEDIAN_KERNEL_SIZE = 3
OPEN_KERNEL_SIZE = 3
CLOSE_KERNEL_SIZE = 7


# Single-frame Metric BEV geometry gates.
MIN_COMPONENT_AREA_PX = 100
MIN_MAJOR_LENGTH_M = 0.08
MAX_MAJOR_LENGTH_M = 0.70
MIN_THICKNESS_M = 0.015
MAX_THICKNESS_M = 0.055
MIN_ASPECT_RATIO = 2.2
MAX_ABS_ANGLE_DEG = 30.0
MAX_ABS_LATERAL_M = 0.35
MIN_DISTANCE_M = 0.55
MAX_DISTANCE_M = 1.00

# In metric-white mode the already calibrated front-lane BEV is the input.
# A horizontal opening removes the longitudinal lane boundaries and keeps the
# near transverse edge of P4/P5 as the approach trigger.
HORIZONTAL_OPEN_KERNEL_WIDTH = 21
HORIZONTAL_OPEN_KERNEL_HEIGHT = 3


bridge = CvBridge()
candidate_pub = None
mask_pub = None
overlay_pub = None
undistort_map_x = None
undistort_map_y = None
FRONT_SLOT_ID = None
SLOT_PROFILES = {}


def init_undistort_maps():
    """Precompute the fixed 640x360 nearest-neighbor undistortion maps."""
    global undistort_map_x
    global undistort_map_y

    new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
        K,
        D,
        CAMERA_SIZE,
        1.0,
        CAMERA_SIZE
    )

    float_map_x, float_map_y = cv2.initUndistortRectifyMap(
        K,
        D,
        None,
        new_camera_matrix,
        CAMERA_SIZE,
        cv2.CV_32FC1
    )

    undistort_map_x, undistort_map_y = cv2.convertMaps(
        float_map_x,
        float_map_y,
        cv2.CV_16SC2,
        nninterpolation=True
    )


def calibration_source_sha256():
    try:
        digest = hashlib.sha256()
        with open(CALIBRATION_SOURCE, "rb") as source_file:
            while True:
                block = source_file.read(65536)
                if not block:
                    break
                digest.update(block)
        return digest.hexdigest()
    except IOError:
        return None


def build_blue_mask(frame_bgr):
    """Build the tested HSV blue candidate mask without a global pixel gate."""
    height, width = frame_bgr.shape[:2]
    floor_mask = np.zeros((height, width), dtype=np.uint8)
    floor_mask[int(height * FLOOR_Y_RATIO):height, :] = 255

    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    lower_blue = np.array(
        [BLUE_H_MIN, BLUE_S_MIN, BLUE_V_MIN],
        dtype=np.uint8
    )
    upper_blue = np.array(
        [BLUE_H_MAX, 255, 255],
        dtype=np.uint8
    )

    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    blue_mask = cv2.bitwise_and(blue_mask, floor_mask)
    blue_mask = cv2.medianBlur(blue_mask, MEDIAN_KERNEL_SIZE)

    open_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (OPEN_KERNEL_SIZE, OPEN_KERNEL_SIZE)
    )
    close_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (CLOSE_KERNEL_SIZE, CLOSE_KERNEL_SIZE)
    )
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, open_kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, close_kernel)
    return blue_mask


def make_metric_bev(blue_mask):
    """Undistort and project a binary blue mask into the frozen Metric BEV."""
    height, width = blue_mask.shape[:2]
    if (width, height) != CAMERA_SIZE:
        return None

    undistorted_mask = cv2.remap(
        blue_mask,
        undistort_map_x,
        undistort_map_y,
        interpolation=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0
    )
    return cv2.warpPerspective(
        undistorted_mask,
        H_METRIC,
        BEV_SIZE,
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0
    )


def build_horizontal_metric_mask(metric_mask, kernel_width=21,
                                 kernel_height=3):
    """Keep transverse white slot edges in an existing metric BEV mask."""
    if metric_mask is None or len(metric_mask.shape) != 2:
        return None
    height, width = metric_mask.shape[:2]
    if (width, height) != BEV_SIZE:
        return None
    kernel_width = int(kernel_width)
    kernel_height = int(kernel_height)
    if kernel_width <= 0 or kernel_height <= 0:
        raise ValueError("horizontal marker kernel dimensions must be positive")
    _, binary = cv2.threshold(metric_mask, 127, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT, (kernel_width, kernel_height))
    return cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)


def normalise_role(value):
    """Return the detector role without allowing semantic mixing."""
    value = str(value or "legacy").strip().lower()
    if value not in ROLE_SCHEMAS:
        raise ValueError("role must be legacy, start_line or slot")
    return value


def role_schema(role=None):
    role = normalise_role(OUTPUT_ROLE if role is None else role)
    return ROLE_SCHEMAS[role]


def empty_result(stamp, candidate_count=0, selection_mode=None,
                 selection_ready=False, slot_id=None, role=None):
    role = normalise_role(OUTPUT_ROLE if role is None else role)
    result = {
        "schema": role_schema(role),
        "role": role,
        "candidate": False,
        "slot_candidate": False,
        "start_line_visible": False,
        "start_line_distance_m": None,
        "distance_m": None,
        "lateral_m": None,
        "angle_deg": None,
        "length_m": None,
        "thickness_m": None,
        "aspect_ratio": None,
        "area_px": None,
        "candidate_count": int(candidate_count),
        "candidates": [],
        "selection_mode": selection_mode,
        "selection_ready": bool(selection_ready),
        "blue_trigger_visible": False,
        "stamp": float(stamp)
    }
    if slot_id is None:
        slot_id = FRONT_SLOT_ID
    if role != "start_line" and slot_id is not None:
        result["slot_id"] = slot_id
    return result


def public_candidate(component, candidate_id=None, slot_hint=None):
    """Return only JSON-safe metric fields for one connected component."""
    if not isinstance(component, dict):
        return None
    if candidate_id is None:
        candidate_id = component.get("label", 0)
    result = {
        "id": int(candidate_id),
        "distance_m": float(component["distance_m"]),
        "lateral_m": float(component["lateral_m"]),
        "angle_deg": float(component["angle_deg"]),
        "length_m": float(component["length_m"]),
        "thickness_m": float(component["thickness_m"]),
        "area_px": int(component["area_px"]),
    }
    if isinstance(slot_hint, string_types):
        slot_hint = slot_hint.strip().upper()
        if slot_hint in ("P4", "P5"):
            result["slot_hint"] = slot_hint
    return result


def analyze_components(metric_mask, max_abs_lateral_m=None,
                       min_distance_m=None, max_distance_m=None):
    """Return every legal component and select the largest by connected area."""
    if max_abs_lateral_m is None:
        max_abs_lateral_m = MAX_ABS_LATERAL_M
    if min_distance_m is None:
        min_distance_m = MIN_DISTANCE_M
    if max_distance_m is None:
        max_distance_m = MAX_DISTANCE_M
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        metric_mask,
        connectivity=8
    )
    legal_components = []

    for label_index in range(1, num_labels):
        area_px = int(stats[label_index, cv2.CC_STAT_AREA])
        if area_px < MIN_COMPONENT_AREA_PX:
            continue

        ys, xs = np.where(labels == label_index)
        if xs.size < MIN_COMPONENT_AREA_PX:
            continue

        points = np.column_stack((xs, ys)).astype(np.float32)
        rect = cv2.minAreaRect(points)
        axis_a_px = float(rect[1][0])
        axis_b_px = float(rect[1][1])
        major_px = max(axis_a_px, axis_b_px)
        minor_px = min(axis_a_px, axis_b_px)
        if minor_px <= 0.0:
            continue

        length_m = major_px / PX_PER_M
        thickness_m = minor_px / PX_PER_M
        aspect_ratio = major_px / minor_px

        if length_m < MIN_MAJOR_LENGTH_M or length_m > MAX_MAJOR_LENGTH_M:
            continue
        if thickness_m < MIN_THICKNESS_M or thickness_m > MAX_THICKNESS_M:
            continue
        if aspect_ratio < MIN_ASPECT_RATIO:
            continue

        # Required centerline distance fit: v = a*u + b.
        u_values = xs.astype(np.float64)
        v_values = ys.astype(np.float64)
        if float(np.max(u_values) - np.min(u_values)) < 1.0:
            continue
        line_a, line_b = np.polyfit(u_values, v_values, 1)
        if not np.isfinite(line_a) or not np.isfinite(line_b):
            continue

        angle_deg = math.degrees(math.atan(float(line_a)))
        if abs(angle_deg) > MAX_ABS_ANGLE_DEG:
            continue

        centroid_u = float(centroids[label_index][0])
        lateral_m = (VEHICLE_CENTER_U - centroid_u) / PX_PER_M
        if abs(lateral_m) > float(max_abs_lateral_m):
            continue

        v_centerline = float(line_a) * VEHICLE_CENTER_U + float(line_b)
        distance_m = (VEHICLE_REAR_V - v_centerline) / PX_PER_M
        if (distance_m < float(min_distance_m) or
                distance_m > float(max_distance_m)):
            continue

        component_mask = np.zeros_like(metric_mask)
        component_mask[labels == label_index] = 255
        contour_result = cv2.findContours(
            component_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )
        contours = contour_result[-2]

        legal_components.append({
            "label": label_index,
            "area_px": area_px,
            "distance_m": float(distance_m),
            "lateral_m": float(lateral_m),
            "angle_deg": float(angle_deg),
            "length_m": float(length_m),
            "thickness_m": float(thickness_m),
            "aspect_ratio": float(aspect_ratio),
            "line_a": float(line_a),
            "line_b": float(line_b),
            "u_min": int(np.min(xs)),
            "u_max": int(np.max(xs)),
            "rect": rect,
            "contours": contours
        })

    legal_components.sort(key=lambda item: item["area_px"], reverse=True)
    selected = legal_components[0] if legal_components else None
    return legal_components, selected


def target_selection_config(slot_id=None, slot_profiles=None):
    """Return the explicit P4/P5 candidate ordering and count requirement.

    The detector sees geometry, not a semantic P4/P5 label.  The map gives
    that geometry a deterministic meaning: P5 is the near candidate and P4
    is the far candidate.  ``front_min_candidate_count`` prevents a P4 run
    from treating a lone, ambiguous marker as the far bay.  A missing profile
    keeps the historical largest-area behaviour for standalone detector use.
    """
    mode = "largest"
    minimum_count = 1
    if slot_id is None or str(slot_id).strip() == "":
        return mode, minimum_count
    if slot_profiles is None:
        slot_profiles = {}
    if not isinstance(slot_profiles, dict):
        raise ValueError("slot_profiles must be an object")
    profile = slot_profiles.get(slot_id, {})
    if not isinstance(profile, dict):
        raise ValueError("visual profile %s must be an object" % slot_id)
    raw_mode = profile.get("front_candidate_order", mode)
    if not isinstance(raw_mode, string_types):
        raise ValueError("front_candidate_order must be a string")
    mode = raw_mode.strip().lower()
    if mode == "far":
        mode = "farthest"
    if mode not in ("largest", "nearest", "farthest"):
        raise ValueError(
            "front_candidate_order must be largest, nearest or farthest")

    raw_count = profile.get("front_min_candidate_count", minimum_count)
    if isinstance(raw_count, bool):
        raise ValueError("front_min_candidate_count must be an integer")
    try:
        numeric_count = float(raw_count)
    except (TypeError, ValueError):
        raise ValueError("front_min_candidate_count must be an integer")
    if (not np.isfinite(numeric_count) or
            numeric_count != int(numeric_count)):
        raise ValueError("front_min_candidate_count must be an integer")
    minimum_count = int(numeric_count)
    if minimum_count < 1:
        raise ValueError("front_min_candidate_count must be positive")
    return mode, minimum_count


def select_target_component(legal_components, slot_id=None,
                            slot_profiles=None):
    """Select the requested bay from all legal front candidates.

    Returns ``(selected, selection_mode, selection_ready)``.  ``selected``
    is ``None`` when the configured minimum number of candidates is not
    visible, so the front metric node cannot turn an ambiguous frame into a
    parking trigger.
    """
    components = list(legal_components or [])
    mode, minimum_count = target_selection_config(slot_id, slot_profiles)
    if len(components) < minimum_count:
        return None, mode, False
    if mode == "nearest":
        ordered = sorted(components, key=lambda item: item["distance_m"])
    elif mode == "farthest":
        ordered = sorted(
            components, key=lambda item: item["distance_m"], reverse=True)
    else:
        ordered = sorted(
            components, key=lambda item: item["area_px"], reverse=True)
    return ordered[0], mode, True


def result_from_candidate(selected, candidate_count, stamp,
                          selection_mode=None, selection_ready=None,
                          slot_id=None, legal_components=None, role=None):
    role = normalise_role(OUTPUT_ROLE if role is None else role)
    if selected is None:
        result = empty_result(
            stamp,
            candidate_count=candidate_count,
            selection_mode=selection_mode,
            selection_ready=bool(selection_ready),
            slot_id=slot_id,
            role=role,
        )
        result["candidates"] = [
            public_candidate(item, index, slot_id)
            for index, item in enumerate(legal_components or [])
            if public_candidate(item, index, slot_id) is not None
        ]
        return result

    result = {
        "schema": role_schema(role),
        "role": role,
        "candidate": True,
        "slot_candidate": bool(role == "slot"),
        "start_line_visible": bool(role == "start_line"),
        "start_line_distance_m": (selected["distance_m"]
                                   if role == "start_line" else None),
        "distance_m": selected["distance_m"],
        "lateral_m": selected["lateral_m"],
        "angle_deg": selected["angle_deg"],
        "length_m": selected["length_m"],
        "thickness_m": selected["thickness_m"],
        "aspect_ratio": selected["aspect_ratio"],
        "area_px": selected["area_px"],
        "candidate_count": int(candidate_count),
        "candidates": [
            public_candidate(item, index, slot_id)
            for index, item in enumerate(legal_components or [selected])
            if public_candidate(item, index, slot_id) is not None
        ],
        "selection_mode": selection_mode,
        "selection_ready": bool(
            True if selection_ready is None else selection_ready),
        # Only the start-line role may assert the blue trigger.  A valid
        # white bay candidate must leave this false.
        "blue_trigger_visible": bool(role == "start_line"),
        "stamp": float(stamp)
    }
    if slot_id is None:
        slot_id = FRONT_SLOT_ID
    if role != "start_line" and slot_id is not None:
        result["slot_id"] = slot_id
    return result


def draw_overlay(metric_mask, legal_components, selected,
                 selection_mode=None):
    overlay = cv2.cvtColor(metric_mask, cv2.COLOR_GRAY2BGR)
    cv2.line(
        overlay,
        (int(VEHICLE_CENTER_U), 0),
        (int(VEHICLE_CENTER_U), BEV_HEIGHT - 1),
        (255, 0, 255),
        1
    )

    for component in legal_components:
        is_selected = component is selected
        color = (0, 255, 0) if is_selected else (0, 255, 255)
        thickness = 2 if is_selected else 1
        if component["contours"]:
            cv2.drawContours(
                overlay,
                component["contours"],
                -1,
                color,
                thickness
            )

        u0 = component["u_min"]
        u1 = component["u_max"]
        v0 = int(round(component["line_a"] * u0 + component["line_b"]))
        v1 = int(round(component["line_a"] * u1 + component["line_b"]))
        cv2.line(overlay, (u0, v0), (u1, v1), color, thickness)

        center_v = int(round(
            component["line_a"] * VEHICLE_CENTER_U + component["line_b"]
        ))
        cv2.circle(
            overlay,
            (int(VEHICLE_CENTER_U), center_v),
            4,
            color,
            -1
        )

    if selected is None:
        cv2.putText(
            overlay,
            "candidate=false count=%d mode=%s" %
            (len(legal_components), selection_mode or "largest"),
            (8, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 0, 255),
            1,
            cv2.LINE_AA
        )
    else:
        lines = [
            "candidate=true count=%d mode=%s area=%d" %
            (len(legal_components), selection_mode or "largest",
             selected["area_px"]),
            "distance=%.3fm lateral=%.3fm angle=%.1fdeg" %
            (selected["distance_m"], selected["lateral_m"], selected["angle_deg"]),
            "length=%.3fm thickness=%.3fm aspect=%.2f" %
            (selected["length_m"], selected["thickness_m"], selected["aspect_ratio"])
        ]
        for line_index, text_value in enumerate(lines):
            cv2.putText(
                overlay,
                text_value,
                (8, 22 + line_index * 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 0),
                1,
                cv2.LINE_AA
            )

    return overlay


def process_frame(frame_bgr, stamp=0.0, role=None):
    role = normalise_role(OUTPUT_ROLE if role is None else role)
    height, width = frame_bgr.shape[:2]
    if (width, height) != CAMERA_SIZE:
        return None, None, empty_result(stamp, role=role), []

    blue_mask = build_blue_mask(frame_bgr)
    metric_mask = make_metric_bev(blue_mask)
    legal_components, _ = analyze_components(metric_mask)
    selected, selection_mode, selection_ready = select_target_component(
        legal_components, FRONT_SLOT_ID, SLOT_PROFILES)
    result = result_from_candidate(
        selected, len(legal_components), stamp, selection_mode,
        selection_ready, FRONT_SLOT_ID, legal_components, role)
    overlay = draw_overlay(
        metric_mask, legal_components, selected, selection_mode)
    return metric_mask, overlay, result, legal_components


def process_metric_mask(metric_mask, stamp=0.0, role=None):
    role = normalise_role(OUTPUT_ROLE if role is None else role)
    horizontal_mask = build_horizontal_metric_mask(
        metric_mask,
        kernel_width=HORIZONTAL_OPEN_KERNEL_WIDTH,
        kernel_height=HORIZONTAL_OPEN_KERNEL_HEIGHT,
    )
    if horizontal_mask is None:
        return None, None, empty_result(stamp, role=role), []
    legal_components, _ = analyze_components(horizontal_mask)
    selected, selection_mode, selection_ready = select_target_component(
        legal_components, FRONT_SLOT_ID, SLOT_PROFILES)
    result = result_from_candidate(
        selected, len(legal_components), stamp, selection_mode,
        selection_ready, FRONT_SLOT_ID, legal_components, role)
    overlay = draw_overlay(
        horizontal_mask, legal_components, selected, selection_mode)
    return horizontal_mask, overlay, result, legal_components


def publish_debug_image(publisher, image, encoding, stamp):
    if publisher is None or image is None:
        return
    try:
        message = bridge.cv2_to_imgmsg(image, encoding=encoding)
        message.header.stamp = stamp
        message.header.frame_id = "base_link"
        publisher.publish(message)
    except CvBridgeError as error:
        rospy.logwarn("debug image conversion failed: %s" % str(error))


def image_callback(message):
    try:
        frame_bgr = bridge.imgmsg_to_cv2(message, "bgr8")
    except CvBridgeError as error:
        rospy.logerr("cv_bridge failed: %s" % str(error))
        return

    height, width = frame_bgr.shape[:2]
    if (width, height) != CAMERA_SIZE:
        rospy.logwarn_throttle(
            5.0,
            "blue marker expects 640x360, got %dx%d" % (width, height)
        )
        result = empty_result(message.header.stamp.to_sec(), role=OUTPUT_ROLE)
        candidate_pub.publish(String(data=json.dumps(result, allow_nan=False)))
        return

    stamp_sec = message.header.stamp.to_sec()
    if stamp_sec <= 0.0:
        stamp_sec = rospy.Time.now().to_sec()

    metric_mask, overlay, result, _ = process_frame(
        frame_bgr, stamp_sec, role=OUTPUT_ROLE)
    candidate_pub.publish(String(data=json.dumps(result, allow_nan=False)))
    publish_debug_image(mask_pub, metric_mask, "mono8", message.header.stamp)
    publish_debug_image(overlay_pub, overlay, "bgr8", message.header.stamp)


def metric_mask_callback(message):
    try:
        metric_mask = bridge.imgmsg_to_cv2(message, "mono8")
    except CvBridgeError as error:
        rospy.logerr("metric mask cv_bridge failed: %s" % str(error))
        return

    height, width = metric_mask.shape[:2]
    if (width, height) != BEV_SIZE:
        rospy.logwarn_throttle(
            5.0,
            "metric-white marker expects %dx%d, got %dx%d" %
            (BEV_WIDTH, BEV_HEIGHT, width, height)
        )
        result = empty_result(message.header.stamp.to_sec(), role=OUTPUT_ROLE)
        candidate_pub.publish(String(data=json.dumps(result, allow_nan=False)))
        return

    stamp_sec = message.header.stamp.to_sec()
    if stamp_sec <= 0.0:
        stamp_sec = rospy.Time.now().to_sec()
    marker_mask, overlay, result, _ = process_metric_mask(
        metric_mask, stamp_sec, role=OUTPUT_ROLE)
    candidate_pub.publish(String(data=json.dumps(result, allow_nan=False)))
    publish_debug_image(mask_pub, marker_mask, "mono8", message.header.stamp)
    publish_debug_image(overlay_pub, overlay, "bgr8", message.header.stamp)


def main():
    global candidate_pub
    global mask_pub
    global overlay_pub
    global FRONT_CAMERA_TOPIC
    global METRIC_MASK_TOPIC
    global CANDIDATE_TOPIC
    global DEBUG_MASK_TOPIC
    global DEBUG_OVERLAY_TOPIC
    global INPUT_MODE
    global OUTPUT_ROLE
    global MAX_ABS_LATERAL_M
    global MIN_DISTANCE_M
    global MAX_DISTANCE_M
    global HORIZONTAL_OPEN_KERNEL_WIDTH
    global HORIZONTAL_OPEN_KERNEL_HEIGHT
    global FRONT_SLOT_ID
    global SLOT_PROFILES

    rospy.init_node("blue_marker_metric", anonymous=False)
    # Keep the historical defaults for existing launch files, but allow the
    # parking composition to declare the complete topic graph in one place.
    # The detector's geometry and calibration remain unchanged; this only
    # prevents a camera/replay remap from silently leaving the node
    # disconnected from the rest of the parking pipeline.
    FRONT_CAMERA_TOPIC = rospy.get_param(
        "~image_topic", FRONT_CAMERA_TOPIC)
    METRIC_MASK_TOPIC = rospy.get_param(
        "~metric_mask_topic", METRIC_MASK_TOPIC)
    CANDIDATE_TOPIC = rospy.get_param(
        "~candidate_topic", CANDIDATE_TOPIC)
    DEBUG_MASK_TOPIC = rospy.get_param(
        "~mask_topic", DEBUG_MASK_TOPIC)
    DEBUG_OVERLAY_TOPIC = rospy.get_param(
        "~overlay_topic", DEBUG_OVERLAY_TOPIC)
    INPUT_MODE = str(rospy.get_param(
        "~input_mode", INPUT_MODE)).strip().lower()
    OUTPUT_ROLE = normalise_role(rospy.get_param("~role", OUTPUT_ROLE))
    raw_slot_id = rospy.get_param("~slot_id", "")
    normalized_slot_id = str(raw_slot_id).strip().upper()
    FRONT_SLOT_ID = (normalized_slot_id
                     if normalized_slot_id in ("P4", "P5") else None)
    SLOT_PROFILES = rospy.get_param("~slot_profiles", {})
    # Validate the target-selection contract at startup.  A malformed map
    # profile must stop the detector rather than silently selecting the largest
    # component and sending the vehicle toward the wrong bay.
    selection_mode, minimum_count = target_selection_config(
        FRONT_SLOT_ID, SLOT_PROFILES)
    MAX_ABS_LATERAL_M = float(rospy.get_param(
        "~max_abs_lateral_m", MAX_ABS_LATERAL_M))
    MIN_DISTANCE_M = float(rospy.get_param(
        "~min_distance_m", MIN_DISTANCE_M))
    MAX_DISTANCE_M = float(rospy.get_param(
        "~max_distance_m", MAX_DISTANCE_M))
    HORIZONTAL_OPEN_KERNEL_WIDTH = int(rospy.get_param(
        "~horizontal_open_kernel_width",
        HORIZONTAL_OPEN_KERNEL_WIDTH))
    HORIZONTAL_OPEN_KERNEL_HEIGHT = int(rospy.get_param(
        "~horizontal_open_kernel_height",
        HORIZONTAL_OPEN_KERNEL_HEIGHT))
    if INPUT_MODE not in ("blue_raw", "metric_white"):
        raise ValueError("input_mode must be blue_raw or metric_white")
    if MAX_ABS_LATERAL_M <= 0.0:
        raise ValueError("max_abs_lateral_m must be positive")
    if MIN_DISTANCE_M < 0.0 or MAX_DISTANCE_M <= MIN_DISTANCE_M:
        raise ValueError("marker distance limits are invalid")
    init_undistort_maps()

    actual_sha = calibration_source_sha256()
    rospy.loginfo("Metric calibration copied from active source: %s", CALIBRATION_SOURCE)
    rospy.loginfo("Expected calibration source SHA256: %s", CALIBRATION_SOURCE_SHA256)
    if actual_sha is None:
        rospy.logwarn("Unable to read frozen calibration source for SHA verification")
    elif actual_sha != CALIBRATION_SOURCE_SHA256:
        rospy.logwarn(
            "Frozen calibration source SHA mismatch: actual=%s expected=%s",
            actual_sha,
            CALIBRATION_SOURCE_SHA256
        )
    else:
        rospy.loginfo("Frozen calibration source SHA256 verified")

    rospy.loginfo(
        "Metric BEV: %dx%d, %.1f px/cm; HSV H[%d,%d] S>=%d V>=%d",
        BEV_WIDTH,
        BEV_HEIGHT,
        PX_PER_CM,
        BLUE_H_MIN,
        BLUE_H_MAX,
        BLUE_S_MIN,
        BLUE_V_MIN
    )
    rospy.loginfo(
        "front detector role=%s slot=%s candidate_order=%s min_candidates=%d",
        OUTPUT_ROLE, FRONT_SLOT_ID or "unset", selection_mode, minimum_count)

    candidate_pub = rospy.Publisher(CANDIDATE_TOPIC, String, queue_size=1)
    mask_pub = rospy.Publisher(DEBUG_MASK_TOPIC, Image, queue_size=1)
    overlay_pub = rospy.Publisher(DEBUG_OVERLAY_TOPIC, Image, queue_size=1)
    if INPUT_MODE == "metric_white":
        rospy.Subscriber(
            METRIC_MASK_TOPIC,
            Image,
            metric_mask_callback,
            queue_size=1,
            buff_size=2 ** 24
        )
        rospy.loginfo(
            "marker input mode=metric_white subscribe=%s horizontal_kernel=%dx%d",
            METRIC_MASK_TOPIC,
            HORIZONTAL_OPEN_KERNEL_WIDTH,
            HORIZONTAL_OPEN_KERNEL_HEIGHT)
    else:
        rospy.Subscriber(
            FRONT_CAMERA_TOPIC,
            Image,
            image_callback,
            queue_size=1,
            buff_size=2 ** 24
        )
        rospy.loginfo("marker input mode=blue_raw subscribe=%s", FRONT_CAMERA_TOPIC)

    rospy.loginfo("publish candidate: %s", CANDIDATE_TOPIC)
    rospy.loginfo("publish mask: %s", DEBUG_MASK_TOPIC)
    rospy.loginfo("publish overlay: %s", DEBUG_OVERLAY_TOPIC)
    rospy.spin()


if __name__ == "__main__":
    main()
