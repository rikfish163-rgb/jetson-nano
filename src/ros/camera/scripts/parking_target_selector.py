#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Select a free P4/P5 reverse-parking bay from white bay geometry and lidar.

The slot detector supplies metric candidates from the white bay lines in
``base_link``.  This module assigns the near/far candidates to P5/P4,
projects the raw LaserScan into the same frame, and evaluates a small
rectangular zone in front of each bay.  A separate blue start-line detector is
accepted only as a trigger signal; it is never used to identify a bay or to
measure bay occupancy.  The geometry remains in pure functions so the safety
decision can be replayed without ROS and tested with synthetic scans.

Coordinate contract:

* ``base_link.x`` is vehicle-forward and ``base_link.y`` is vehicle-left;
* slot candidate ``distance_m`` is the forward distance to the selected white
  bay geometry;
* candidate ``lateral_m`` is positive to vehicle-left;
* LaserScan points are transformed by the calibrated laser-to-base yaw and
  translation before slot occupancy is evaluated.

The numeric zone offsets are field-tunable.  ``zone_calibrated`` must be true
before ``selection_ready`` can become true; a provisional zone never unlocks
vehicle motion.
"""

from __future__ import division, print_function

import json
import math
import sys
import time

try:
    import rospy
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
except ImportError:  # Keep the pure helpers importable on a non-ROS laptop.
    rospy = None
    LaserScan = None
    String = None


try:
    string_types = (basestring,)
except NameError:
    string_types = (str,)


SUPPORTED_SLOTS = ("P4", "P5")
SCHEMA = "parking_target_v1"


def finite(value, default=None):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    if math.isnan(value) or math.isinf(value):
        return default
    return value


def _normalise_slot(value):
    if not isinstance(value, string_types):
        return None
    value = value.strip().upper()
    if value in ("", "AUTO", "ANY", "NONE", "NULL"):
        return None
    return value if value in SUPPORTED_SLOTS else None


def _normalise_side(value):
    """Return a concrete vehicle-relative parking side or None."""
    if not isinstance(value, string_types):
        return None
    value = value.strip().lower()
    if value in ("left", "l", "+1", "positive"):
        return "left"
    if value in ("right", "r", "-1", "negative"):
        return "right"
    return None


def _side_from_candidate(candidate, slot_id, settings):
    """Resolve side from explicit map/vision data, then lateral position."""
    if not isinstance(candidate, dict):
        return None, "missing"
    for key in ("parking_side", "turn_side", "side"):
        side = _normalise_side(candidate.get(key))
        if side is not None:
            return side, "candidate_%s" % key

    profiles = settings.get("slot_profiles", {})
    profile = profiles.get(slot_id, {}) if isinstance(profiles, dict) else {}
    if isinstance(profile, dict):
        side = _normalise_side(profile.get("parking_side"))
        if side is not None:
            return side, "slot_profile"

    slot_sides = settings.get("slot_parking_sides", {})
    if isinstance(slot_sides, dict):
        side = _normalise_side(slot_sides.get(slot_id))
        if side is not None:
            return side, "slot_map"

    side = _normalise_side(settings.get("parking_side"))
    if side is not None:
        return side, "global_map"

    lateral = finite(candidate.get("lateral_m"))
    deadband = finite(settings.get("side_inference_deadband_m"), 0.05)
    if lateral is not None and deadband is not None:
        if lateral > abs(deadband):
            return "left", "candidate_lateral"
        if lateral < -abs(deadband):
            return "right", "candidate_lateral"
    return None, "ambiguous_lateral"


def _decode(payload):
    if isinstance(payload, dict):
        return dict(payload)
    if not isinstance(payload, string_types):
        return None
    text = payload.strip()
    if not text:
        return None
    try:
        value = json.loads(text)
    except (TypeError, ValueError):
        return {"label": text}
    return dict(value) if isinstance(value, dict) else None


def _start_line_summary(payload, selected_slot=None,
                        slot_candidate=None, settings=None):
    """Return the independent blue start-line observation.

    ``parking_slot_candidate_v1`` is deliberately rejected here.  The
    legacy schema is accepted only for old replay callers; the reform launch
    uses ``parking_start_line_v1`` so a white bay candidate can never assert a
    blue trigger by accident.
    """
    data = _decode(payload)
    empty = {
        "visible": False,
        "distance_m": None,
        "lateral_m": None,
        "angle_deg": None,
        "schema": (data.get("schema") if isinstance(data, dict) else None),
    }
    if not isinstance(data, dict):
        return empty
    schema = data.get("schema")
    if schema == "parking_slot_candidate_v1":
        return empty
    if schema not in (None, "parking_start_line_v1",
                      "blue_marker_candidate_v1"):
        return empty
    if data.get("candidate") is not True:
        return empty

    candidates = []
    raw_candidates = data.get("candidates")
    if isinstance(raw_candidates, list):
        for index, item in enumerate(raw_candidates):
            candidate = _candidate_from_item(item, index)
            if candidate is not None:
                candidates.append(candidate)
    if not candidates:
        visible = data.get("start_line_visible")
        if visible is None:
            visible = data.get("blue_trigger_visible")
        if visible is True:
            distance = finite(data.get("start_line_distance_m"))
            if distance is None:
                distance = finite(data.get("distance_m"))
            empty.update({
                "visible": True,
                "distance_m": distance,
                "lateral_m": finite(data.get("lateral_m")),
                "angle_deg": finite(data.get("angle_deg")),
            })
        return empty

    # Prefer a blue candidate on the same vehicle-relative side when that
    # distinction is observable.  Do not reject a line whose centroid is near
    # the vehicle centre; a transverse line can legitimately have a centred
    # centroid even though the bay is on one side.
    settings = dict(settings or {})
    target_lateral = (finite(slot_candidate.get("lateral_m"))
                      if isinstance(slot_candidate, dict) else None)
    deadband = abs(finite(
        settings.get("side_inference_deadband_m"), 0.05) or 0.05)
    if (target_lateral is not None and abs(target_lateral) > deadband):
        same_side = [item for item in candidates
                     if finite(item.get("lateral_m")) is not None and
                     item["lateral_m"] * target_lateral > 0.0]
        if same_side:
            candidates = same_side

    # The field map defines P5 as the near bay and P4 as the far bay.  If both
    # blue start lines are in view, choose the line belonging to the selected
    # bay by this order; this is only trigger association, never slot identity.
    order = str(settings.get("start_line_selection_mode", "near_far"))
    if len(candidates) > 1 and order == "near_far":
        if selected_slot == "P4":
            candidates = sorted(candidates,
                                key=lambda item: item["distance_m"],
                                reverse=True)
        elif selected_slot == "P5":
            candidates = sorted(candidates,
                                key=lambda item: item["distance_m"])
    chosen = candidates[0]
    distance = finite(chosen.get("distance_m"))
    empty.update({
        "visible": True,
        "distance_m": distance,
        "lateral_m": finite(chosen.get("lateral_m")),
        "angle_deg": finite(chosen.get("angle_deg")),
    })
    return empty


def _candidate_from_item(item, index):
    if not isinstance(item, dict):
        return None
    distance = finite(item.get("distance_m"))
    lateral = finite(item.get("lateral_m"))
    if distance is None or lateral is None or distance < 0.0:
        return None
    result = {
        "id": str(item.get("id", item.get("label", index))),
        "distance_m": float(distance),
        "lateral_m": float(lateral),
        "angle_deg": finite(item.get("angle_deg")),
        "length_m": finite(item.get("length_m")),
        "thickness_m": finite(item.get("thickness_m")),
        "area_px": int(finite(item.get("area_px"), 0.0) or 0),
    }
    for key in ("parking_side", "turn_side", "side"):
        side = _normalise_side(item.get(key))
        if side is not None:
            result["parking_side"] = side
            break
    slot = _normalise_slot(item.get("slot_id", item.get("slot_hint")))
    if slot is not None:
        result["slot_id"] = slot
    return result


def extract_candidates(payload):
    """Extract white bay candidates, never blue start-line geometry."""
    data = _decode(payload)
    if not isinstance(data, dict):
        return []
    schema = data.get("schema")
    if schema == "parking_start_line_v1":
        return []
    if (schema == "parking_slot_candidate_v1" and
            data.get("slot_candidate") is not True):
        return []
    raw_candidates = data.get("candidates")
    candidates = []
    if isinstance(raw_candidates, list):
        for index, item in enumerate(raw_candidates):
            candidate = _candidate_from_item(item, index)
            if candidate is not None:
                candidates.append(candidate)
    if not candidates and data.get("candidate") is True:
        candidate = _candidate_from_item(data, 0)
        if candidate is not None:
            candidates.append(candidate)
    return candidates


def _band_for_slot(slot_id, slot_profiles, slot_distance_bands):
    bands = slot_distance_bands if isinstance(slot_distance_bands, dict) else {}
    profile = (slot_profiles.get(slot_id, {})
               if isinstance(slot_profiles, dict) else {})
    value = bands.get(slot_id)
    if value is None:
        value = profile.get("front_distance_band_m")
    if value is None:
        minimum = profile.get("front_distance_min_m")
        maximum = profile.get("front_distance_max_m")
        if minimum is not None or maximum is not None:
            value = [minimum, maximum]
    if isinstance(value, dict):
        value = [value.get("min_m", value.get("min")),
                 value.get("max_m", value.get("max"))]
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        return None
    minimum = finite(value[0])
    maximum = finite(value[1])
    if minimum is None or maximum is None or maximum < minimum:
        return None
    return minimum, maximum


def assign_slot_hints(candidates, slot_profiles=None,
                      slot_distance_bands=None):
    """Assign P4/P5 identities using explicit hints, depth, or distance bands."""
    copied = []
    for index, item in enumerate(candidates or []):
        candidate = _candidate_from_item(item, index)
        if candidate is not None:
            copied.append(candidate)
    ordered = sorted(copied, key=lambda item: item["distance_m"])

    # An explicit slot hint wins.  This is useful when a classifier has
    # already identified the bay; it is never overwritten by near/far order.
    unassigned = [item for item in ordered if "slot_id" not in item]
    if len(ordered) >= 2:
        used = set(item.get("slot_id") for item in ordered
                   if item.get("slot_id") in SUPPORTED_SLOTS)
        available = [slot for slot in ("P5", "P4") if slot not in used]
        for item in unassigned:
            if not available:
                break
            # The nearest visible trigger is P5; the far trigger is P4.
            item["slot_id"] = available.pop(0)
    elif len(ordered) == 1:
        distance = ordered[0]["distance_m"]
        matches = []
        for slot in SUPPORTED_SLOTS:
            band = _band_for_slot(slot, slot_profiles,
                                  slot_distance_bands)
            if band is not None and band[0] <= distance <= band[1]:
                matches.append(slot)
        # Overlapping or absent bands are intentionally ambiguous.
        if len(matches) == 1 and "slot_id" not in ordered[0]:
            ordered[0]["slot_id"] = matches[0]
    return ordered


def scan_health(scan, min_valid_samples=8, max_range_m=3.0):
    """Return a small, JSON-safe validity summary for a LaserScan-like object."""
    ranges = getattr(scan, "ranges", None)
    if not isinstance(ranges, (list, tuple)):
        try:
            ranges = list(ranges)
        except (TypeError, ValueError):
            ranges = None
    if not ranges:
        return {"valid": False, "valid_sample_count": 0,
                "reason": "scan_missing"}
    max_range = finite(max_range_m)
    if max_range is None or max_range <= 0.0:
        max_range = 3.0
    minimum = finite(getattr(scan, "range_min", 0.0), 0.0)
    valid_count = 0
    for value in ranges:
        value = finite(value)
        if value is not None and value >= max(0.0, minimum) and value <= max_range:
            valid_count += 1
    required = max(1, int(min_valid_samples))
    return {
        "valid": bool(valid_count >= required),
        "valid_sample_count": int(valid_count),
        "sample_count": int(len(ranges)),
        "reason": "ok" if valid_count >= required else "scan_too_few_valid_samples",
    }


def _scan_points_base_link(scan, settings):
    ranges = getattr(scan, "ranges", None)
    if ranges is None:
        return []
    angle_min = finite(getattr(scan, "angle_min", 0.0), 0.0)
    increment = finite(getattr(scan, "angle_increment", 0.0), 0.0)
    if increment is None or abs(increment) <= 1.0e-12:
        return []
    tx = finite(settings.get("lidar_to_base_x_m"), 0.0)
    ty = finite(settings.get("lidar_to_base_y_m"), 0.0)
    yaw = finite(settings.get("lidar_yaw_rad"), 0.0)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    max_range = finite(settings.get("obstacle_max_distance_m"), 3.0)
    range_min = finite(getattr(scan, "range_min", 0.0), 0.0)
    points = []
    for index, raw_range in enumerate(ranges):
        distance = finite(raw_range)
        if (distance is None or distance < max(0.0, range_min) or
                distance > max_range):
            continue
        angle = angle_min + index * increment
        x_lidar = distance * math.cos(angle)
        y_lidar = distance * math.sin(angle)
        x_base = cos_yaw * x_lidar - sin_yaw * y_lidar + tx
        y_base = sin_yaw * x_lidar + cos_yaw * y_lidar + ty
        points.append((x_base, y_base, distance))
    return points


def evaluate_slot_zone(scan, candidate, settings=None):
    """Classify one rectangular bay zone from a raw LaserScan."""
    settings = dict(settings or {})
    health = scan_health(
        scan,
        settings.get("min_scan_samples", 8),
        settings.get("obstacle_max_distance_m", 3.0),
    )
    if not health["valid"]:
        return {
            "valid": False,
            "obstacle_detected": None,
            "min_distance_m": None,
            "point_count": 0,
            "reason": health["reason"],
        }
    distance = finite(candidate.get("distance_m"))
    lateral = finite(candidate.get("lateral_m"))
    if distance is None or lateral is None:
        return {
            "valid": False,
            "obstacle_detected": None,
            "min_distance_m": None,
            "point_count": 0,
            "reason": "candidate_geometry_missing",
        }
    width = finite(settings.get("slot_width_m"), 0.38)
    margin = finite(settings.get("safety_margin_m"), 0.05)
    start_offset = finite(settings.get("zone_start_offset_m"), 0.08)
    end_offset = finite(settings.get("zone_end_offset_m"), 0.53)
    if width is None or margin is None or start_offset is None or end_offset is None:
        return {
            "valid": False, "obstacle_detected": None,
            "min_distance_m": None, "point_count": 0,
            "reason": "zone_parameters_invalid",
        }
    x_min = distance + min(start_offset, end_offset)
    x_max = distance + max(start_offset, end_offset)
    y_min = lateral - width / 2.0 - margin
    y_max = lateral + width / 2.0 + margin
    in_zone = []
    for x_base, y_base, range_m in _scan_points_base_link(scan, settings):
        if x_min <= x_base <= x_max and y_min <= y_base <= y_max:
            in_zone.append(range_m)
    return {
        "valid": True,
        "obstacle_detected": bool(in_zone),
        "min_distance_m": float(min(in_zone)) if in_zone else None,
        "point_count": int(len(in_zone)),
        "reason": "obstacle_detected" if in_zone else "zone_clear",
        "zone_x_min_m": float(x_min),
        "zone_x_max_m": float(x_max),
        "zone_y_min_m": float(y_min),
        "zone_y_max_m": float(y_max),
    }


def _empty_slot_status(reason):
    return {
        "visible": False,
        "lidar_valid": False,
        "obstacle_detected": None,
        "min_distance_m": None,
        "point_count": 0,
        "reason": reason,
    }


def choose_target(candidate_payload, scan, requested_slot_id=None,
                  settings=None, start_line_payload=None):
    """Return a deterministic target-selection contract.

    A requested slot is preferred only if it is visible and clear.  If it is
    occupied, the other clear slot is selected before the vehicle moves.  No
    caller may change the selected slot after the reverse manoeuvre starts;
    that latch is enforced by ``ObservationBuilder``.
    """
    settings = dict(settings or {})
    profiles = settings.get("slot_profiles", {})
    bands = settings.get("slot_distance_bands", {})
    candidates = assign_slot_hints(
        extract_candidates(candidate_payload), profiles, bands)
    health = scan_health(
        scan,
        settings.get("min_scan_samples", 8),
        settings.get("obstacle_max_distance_m", 3.0),
    )
    slots = dict((slot, _empty_slot_status("slot_not_visible"))
                 for slot in SUPPORTED_SLOTS)
    enriched = []
    for candidate in candidates:
        slot = candidate.get("slot_id")
        item = dict(candidate)
        if slot not in SUPPORTED_SLOTS:
            item["lidar"] = _empty_slot_status("slot_identity_ambiguous")
            enriched.append(item)
            continue
        # If more than one component maps to the same slot, the nearest one is
        # the conservative representative; the map should normally produce
        # exactly one candidate per slot.
        lidar_result = evaluate_slot_zone(scan, candidate, settings)
        item["slot_id"] = slot
        item["lidar"] = dict(lidar_result)
        previous = slots[slot]
        if (not previous.get("visible") or
                candidate["distance_m"] <
                float(previous.get("distance_m", float("inf")))):
            slots[slot] = {
                "visible": True,
                "lidar_valid": bool(lidar_result.get("valid")),
                "obstacle_detected": lidar_result.get("obstacle_detected"),
                "min_distance_m": lidar_result.get("min_distance_m"),
                "point_count": int(lidar_result.get("point_count", 0)),
                "reason": lidar_result.get("reason"),
            }
        enriched.append(item)

    preferred = _normalise_slot(requested_slot_id)
    if preferred is None:
        preferred = _normalise_slot(settings.get("preferred_slot_id"))
    clear = []
    for slot in SUPPORTED_SLOTS:
        status = slots[slot]
        if (status.get("visible") and status.get("lidar_valid") and
                status.get("obstacle_detected") is False):
            clear.append(slot)

    selected_slot = None
    selection_reason = "no_clear_slot"
    if preferred in clear:
        selected_slot = preferred
        selection_reason = "preferred_clear"
    elif preferred in SUPPORTED_SLOTS and preferred in slots and slots[preferred].get("visible"):
        if preferred not in clear:
            selection_reason = "preferred_occupied_fallback"
    if selected_slot is None and clear:
        order = settings.get("fallback_order", "nearest_clear")
        selected_slot = ("P4" if order == "farthest_clear" else "P5")
        if selected_slot not in clear:
            selected_slot = clear[0]
        if selection_reason != "preferred_occupied_fallback":
            selection_reason = "fallback_clear"

    selected = None
    for item in enriched:
        if item.get("slot_id") == selected_slot:
            selected = item
            break
    parking_side, side_source = _side_from_candidate(
        selected, selected_slot, settings)
    start_line = _start_line_summary(
        start_line_payload,
        selected_slot=selected_slot,
        slot_candidate=selected,
        settings=settings,
    )
    zone_calibrated = bool(settings.get("zone_calibrated", False))
    ready = bool(selected_slot is not None and health["valid"] and
                 zone_calibrated and parking_side is not None)
    candidate_data = _decode(candidate_payload)
    candidate_confidence = finite(
        candidate_data.get("confidence")
        if isinstance(candidate_data, dict) else None,
        0.75)
    if candidate_confidence is None:
        candidate_confidence = 0.75
    if not ready:
        if not health["valid"]:
            reason = "lidar_" + health["reason"]
        elif not zone_calibrated:
            reason = "zone_not_calibrated"
        elif parking_side is None:
            reason = "parking_side_ambiguous"
        else:
            reason = selection_reason
    else:
        reason = "selected_" + str(selected_slot)
    result = {
        "schema": SCHEMA,
        "stamp": time.time(),
        "frame_id": "base_link",
        "coordinate_frame": "base_link",
        "valid": bool(ready),
        "candidate": bool(selected is not None),
        "selection_ready": bool(ready),
        # This is the raw blue-line visibility for diagnostics.  The final
        # trigger is gated by the selected, lidar-clear, calibrated target;
        # slot geometry itself never sets it.
        "start_line_visible": bool(start_line["visible"]),
        "start_line_distance_m": start_line["distance_m"],
        "start_line_lateral_m": start_line["lateral_m"],
        "start_line_angle_deg": start_line["angle_deg"],
        "start_line_schema": start_line["schema"],
        "blue_trigger_visible": bool(
            ready and selected_slot is not None and start_line["visible"]),
        "turn_distance_m": (start_line["distance_m"]
                            if start_line["visible"] else None),
        "slot_id": selected_slot,
        "selected_slot_id": selected_slot,
        "preferred_slot_id": preferred,
        "parking_side": parking_side,
        "turn_side": parking_side,
        "turn_sign": (1 if parking_side == "left" else
                       -1 if parking_side == "right" else None),
        "side_source": side_source,
        "candidate_count": int(len(candidates)),
        "candidates": enriched,
        "slots": slots,
        "lidar_valid": bool(health["valid"]),
        "lidar_valid_sample_count": int(health.get("valid_sample_count", 0)),
        "slot_obstacle_detected": (slots[selected_slot].get("obstacle_detected")
                                    if selected_slot in slots else None),
        "reason": reason,
        "selection_reason": selection_reason,
        "target_reason": reason,
        "confidence": float(max(0.0, min(1.0, candidate_confidence if ready else 0.0))),
    }
    if selected is not None:
        for key in ("distance_m", "lateral_m", "angle_deg", "length_m",
                    "thickness_m", "area_px"):
            result[key] = selected.get(key)
    else:
        for key in ("distance_m", "lateral_m", "angle_deg", "length_m",
                    "thickness_m", "area_px"):
            result[key] = None
    return result


class ParkingTargetSelectorNode(object):
    """ROS wrapper that adds freshness to the pure selection decision."""

    def __init__(self):
        if rospy is None:
            raise RuntimeError("ROS is required for ParkingTargetSelectorNode")
        self.slot_candidate_topic = rospy.get_param(
            "~slot_candidate_topic",
            rospy.get_param("~candidate_topic",
                            "/perception/parking_slot_candidate"))
        # Keep the alias for old diagnostic code and tests; it always means
        # the white bay-candidate stream in the reform composition.
        self.candidate_topic = self.slot_candidate_topic
        self.start_line_topic = rospy.get_param(
            "~start_line_topic", "/perception/parking_start_line")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.sign_topic = rospy.get_param("~sign_topic", "/camera_hts/receive")
        self.output_topic = rospy.get_param(
            "~output_topic", "/perception/parking_target")
        self.input_timeout_s = float(rospy.get_param("~input_timeout_s", 0.50))
        self.scan_timeout_s = float(rospy.get_param("~scan_timeout_s", 0.35))
        self.settings = {
            "slot_width_m": float(rospy.get_param("~slot_width_m", 0.38)),
            "slot_length_m": float(rospy.get_param("~slot_length_m", 0.45)),
            "zone_start_offset_m": float(rospy.get_param("~zone_start_offset_m", 0.08)),
            "zone_end_offset_m": float(rospy.get_param("~zone_end_offset_m", 0.53)),
            "safety_margin_m": float(rospy.get_param("~safety_margin_m", 0.05)),
            "obstacle_max_distance_m": float(rospy.get_param("~obstacle_max_distance_m", 3.0)),
            "lidar_to_base_x_m": float(rospy.get_param("~lidar_to_base_x_m", 0.0)),
            "lidar_to_base_y_m": float(rospy.get_param("~lidar_to_base_y_m", 0.0)),
            "lidar_yaw_rad": float(rospy.get_param("~lidar_yaw_rad", 0.0)),
            "min_scan_samples": int(rospy.get_param("~min_scan_samples", 8)),
            "zone_calibrated": bool(rospy.get_param("~zone_calibrated", False)),
            "preferred_slot_id": rospy.get_param("~preferred_slot_id", "AUTO"),
            "slot_profiles": rospy.get_param("~slot_profiles", {}),
            "slot_distance_bands": rospy.get_param("~slot_distance_bands", {}),
            "slot_parking_sides": rospy.get_param("~slot_parking_sides", {}),
            "parking_side": rospy.get_param("~parking_side", "AUTO"),
            "start_line_selection_mode": rospy.get_param(
                "~start_line_selection_mode", "near_far"),
            "side_inference_deadband_m": float(
                rospy.get_param("~side_inference_deadband_m", 0.05)),
            "fallback_order": rospy.get_param("~fallback_order", "nearest_clear"),
        }
        self.candidate = None
        self.candidate_received_at = None
        self.start_line = None
        self.start_line_received_at = None
        self.scan = None
        self.scan_received_at = None
        self.requested_slot_id = _normalise_slot(
            self.settings.get("preferred_slot_id"))
        self.publisher = rospy.Publisher(self.output_topic, String, queue_size=1)
        rospy.Subscriber(self.slot_candidate_topic, String,
                         self._candidate_callback, queue_size=1)
        rospy.Subscriber(self.start_line_topic, String,
                         self._start_line_callback, queue_size=1)
        rospy.Subscriber(self.scan_topic, LaserScan,
                         self._scan_callback, queue_size=1)
        rospy.Subscriber(self.sign_topic, String,
                         self._sign_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.05), self._publish)
        rospy.loginfo(
            "parking_target_selector ready: slot_candidate=%s start_line=%s "
            "scan=%s output=%s "
            "zone_calibrated=%s preferred=%s",
            self.slot_candidate_topic, self.start_line_topic,
            self.scan_topic, self.output_topic,
            str(self.settings["zone_calibrated"]),
            self.requested_slot_id or "AUTO")

    def _candidate_callback(self, message):
        decoded = _decode(message.data)
        self.candidate = decoded
        self.candidate_received_at = time.time() if decoded is not None else None

    def _start_line_callback(self, message):
        decoded = _decode(message.data)
        self.start_line = decoded
        self.start_line_received_at = time.time() if decoded is not None else None

    def _scan_callback(self, message):
        self.scan = message
        self.scan_received_at = time.time()

    def _sign_callback(self, message):
        data = _decode(message.data)
        if not isinstance(data, dict):
            return
        slot = _normalise_slot(data.get("slot_id"))
        label = data.get("label", data.get("class", data.get("sign")))
        if slot is None and isinstance(label, string_types):
            slot = _normalise_slot(label)
        if slot is not None:
            self.requested_slot_id = slot

    def _publish(self, _event):
        if rospy.is_shutdown():
            return
        now = time.time()
        candidate = (self.candidate if self.candidate_received_at is not None and
                     now - self.candidate_received_at <= self.input_timeout_s
                     else None)
        start_line = (self.start_line
                      if self.start_line_received_at is not None and
                      now - self.start_line_received_at <= self.input_timeout_s
                      else None)
        scan = (self.scan if self.scan_received_at is not None and
                now - self.scan_received_at <= self.scan_timeout_s else None)
        result = choose_target(
            candidate,
            scan,
            self.requested_slot_id,
            self.settings,
            start_line_payload=start_line,
        )
        result["input_fresh"] = bool(candidate is not None)
        result["start_line_fresh"] = bool(start_line is not None)
        result["scan_fresh"] = bool(scan is not None)
        if candidate is None:
            result["reason"] = "candidate_stale_or_missing"
            result["target_reason"] = result["reason"]
            result["valid"] = False
            result["selection_ready"] = False
            result["confidence"] = 0.0
        elif scan is None:
            result["reason"] = "scan_stale_or_missing"
            result["target_reason"] = result["reason"]
            result["valid"] = False
            result["selection_ready"] = False
            result["confidence"] = 0.0
        self.publisher.publish(String(
            data=json.dumps(result, separators=(",", ":"), allow_nan=False)))


def main():
    if rospy is None:
        print("parking_target_selector requires ROS", file=sys.stderr)
        return 2
    rospy.init_node("parking_target_selector", anonymous=False)
    try:
        ParkingTargetSelectorNode()
    except Exception as exc:
        rospy.logfatal("parking_target_selector cannot start: %s", exc)
        return 2
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
