#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Fuse P-sign arming, P4/P5 target selection, and metric pose sources.

The controller consumes one strict ``parking_observation_v1`` payload.  This
module is the boundary between several ROS String topics and that contract;
it never converts pixels to metres and it never chooses a different bay after
the first selected bay has been latched.

The important mission facts are intentionally separate:

* ``parking_armed``: a P sign has been seen and the mission is armed;
* ``selected_slot_id``/``slot_selection_ready``: the target selector found a
  specific clear P4/P5 bay from white bay geometry and the raw LaserScan;
* ``parking_trigger_visible``: the independent blue start line is visible in
  the front metric source and the P sign has already armed the mission.

All geometric errors and lidar safety fields are in ``base_link``:
``x`` forward, ``y`` left.  The old sign and lidar String formats remain
accepted only at this adapter boundary.
"""

from __future__ import division

import json
import math
import time


try:
    string_types = (basestring,)
except NameError:
    string_types = (str,)


SUPPORTED_SLOT_IDS = ("P4", "P5")
SOURCE_SCHEMAS = {
    "front": "parking_front_metric_v1",
    "rear": "parking_rear_metric_v1",
    "exit": "parking_exit_metric_v1",
    "target": "parking_target_v1",
}
PARKING_SIGN_LABELS = (
    "p", "park", "parking", "parking_sign", "p_sign",
    "reverse_parking",
)


def is_finite(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return False
    return not math.isnan(number) and not math.isinf(number)


def number(data, key, default=None):
    if not isinstance(data, dict):
        return default
    value = data.get(key)
    if isinstance(value, bool) or not is_finite(value):
        return default
    return float(value)


def normalise_slot_id(value, supported=SUPPORTED_SLOT_IDS):
    if not isinstance(value, string_types):
        return None
    value = value.strip().upper()
    if value in ("", "AUTO", "ANY", "NONE", "NULL"):
        return None
    return value if value in tuple(supported) else None


def normalise_parking_side(value):
    """Return the vehicle-relative entry side used by the mirrored FSM."""
    if not isinstance(value, string_types):
        return None
    value = value.strip().lower()
    if value in ("left", "l", "+1", "positive"):
        return "left"
    if value in ("right", "r", "-1", "negative"):
        return "right"
    return None


def parse_payload(payload):
    """Parse a String payload without accepting malformed sensor data."""
    if isinstance(payload, dict):
        return dict(payload)
    if not isinstance(payload, string_types):
        return None
    text = payload.strip()
    if not text:
        return None
    try:
        decoded = json.loads(text)
    except (TypeError, ValueError):
        # The existing sign classifier publishes plain ``P``/``park``.
        return {"label": text}
    return dict(decoded) if isinstance(decoded, dict) else None


def _bool_or_none(data, key):
    value = data.get(key) if isinstance(data, dict) else None
    return value if isinstance(value, bool) else None


class ObservationBuilder(object):
    """Build the single strict observation used by the parking FSM."""

    DEFAULTS = {
        "default_slot_id": "P4",
        "supported_slot_ids": ("P4", "P5"),
        "input_timeout_s": 0.50,
        "target_timeout_s": 0.50,
        "sign_timeout_s": 1.50,
        # A P sign is an arm event, not a one-frame trigger.  The value is
        # long enough to reach either visible bay and remains field-tunable.
        "parking_request_hold_s": 120.0,
        "selected_slot_latch_s": 120.0,
        "parking_sign_labels": PARKING_SIGN_LABELS,
        "request_from_front_slot": False,
        "min_source_confidence": 0.55,
        "derive_clear_from_lidar": True,
    }

    def __init__(self, values=None):
        settings = dict(self.DEFAULTS)
        if values:
            settings.update(values)
        supported = settings["supported_slot_ids"]
        if not isinstance(supported, (list, tuple)) or not supported:
            raise ValueError("supported_slot_ids must be a non-empty list")
        supported = tuple(str(slot).strip().upper() for slot in supported)
        if any(not slot for slot in supported):
            raise ValueError("supported_slot_ids must contain non-empty strings")
        if len(set(supported)) != len(supported):
            raise ValueError("supported_slot_ids must not contain duplicates")
        self.supported_slot_ids = supported
        raw_default = settings["default_slot_id"]
        if not isinstance(raw_default, string_types):
            raise ValueError("default_slot_id must be a string")
        raw_default = raw_default.strip().upper()
        self.default_slot_id = normalise_slot_id(
            raw_default, self.supported_slot_ids)
        if self.default_slot_id is None and raw_default not in (
                "AUTO", "ANY", "NONE", "NULL", ""):
            raise ValueError("default_slot_id must be supported or AUTO")
        self.input_timeout_s = float(settings["input_timeout_s"])
        self.target_timeout_s = float(settings["target_timeout_s"])
        self.sign_timeout_s = float(settings["sign_timeout_s"])
        self.parking_request_hold_s = float(
            settings["parking_request_hold_s"])
        self.selected_slot_latch_s = float(
            settings["selected_slot_latch_s"])
        labels = settings["parking_sign_labels"]
        if not isinstance(labels, (list, tuple)):
            raise ValueError("parking_sign_labels must be a list")
        self.parking_sign_labels = tuple(
            str(label).strip().lower() for label in labels)
        self.request_from_front_slot = bool(
            settings["request_from_front_slot"])
        self.min_source_confidence = float(settings["min_source_confidence"])
        self.derive_clear_from_lidar = bool(settings["derive_clear_from_lidar"])
        if (self.input_timeout_s <= 0.0 or self.target_timeout_s <= 0.0 or
                self.sign_timeout_s <= 0.0):
            raise ValueError("source timeouts must be positive")
        if self.parking_request_hold_s < 0.0 or self.selected_slot_latch_s < 0.0:
            raise ValueError("parking holds must be non-negative")
        if (self.min_source_confidence < 0.0 or
                self.min_source_confidence > 1.0):
            raise ValueError("min_source_confidence must be in the range 0..1")

        self._sources = {}
        self._parking_armed_until = 0.0
        # Keep the historical private name for tools that inspect it.
        self._parking_request_until = 0.0
        self._last_sign = None
        self._selected_slot_id = None
        self._selected_slot_at = None

    def update(self, source_name, payload, received_at=None):
        """Store one callback payload; return true when it is parseable."""
        if source_name not in SOURCE_SCHEMAS and source_name not in (
                "sign", "lidar"):
            raise ValueError("unknown observation source: %s" % source_name)
        if received_at is None:
            received_at = time.time()
        decoded = parse_payload(payload)
        if decoded is None:
            return False
        self._sources[source_name] = {
            "data": decoded,
            "received_at": float(received_at),
            "source_stamp": number(decoded, "stamp"),
        }
        if source_name == "sign":
            self._remember_sign(decoded, float(received_at))
        return True

    def clear(self):
        self._sources = {}
        self._parking_armed_until = 0.0
        self._parking_request_until = 0.0
        self._last_sign = None
        self._selected_slot_id = None
        self._selected_slot_at = None

    def build(self, now=None):
        if now is None:
            now = time.time()
        now = float(now)

        front, front_meta = self._fresh_source("front", now)
        rear, rear_meta = self._fresh_source("rear", now)
        exit_data, exit_meta = self._fresh_source("exit", now)
        target, target_meta = self._fresh_source(
            "target", now, self.target_timeout_s)
        lidar, lidar_meta = self._fresh_source("lidar", now)
        sign, sign_meta = self._fresh_source(
            "sign", now, self.sign_timeout_s)
        if sign is not None:
            self._remember_sign(sign, sign_meta["received_at"])

        front_ok = self._metric_source_ok("front", front)
        rear_ok = self._metric_source_ok("rear", rear)
        exit_ok = self._metric_source_ok("exit", exit_data)
        target_ok = self._target_source_ok(target)
        lidar_ok = self._lidar_ok(lidar)
        parking_armed = self._parking_armed(now)

        # First target selection after the P arm is the mission latch.  Before
        # the arm, white bay candidates are diagnostics only and cannot start
        # the parking FSM.  The blue start line is an independent trigger and
        # cannot select a bay.  After the latch, a target-selector fallback to
        # the other bay is deliberately rejected by _active_target_status().
        if (parking_armed and self._selected_slot_id is None and target_ok):
            self._selected_slot_id = normalise_slot_id(
                target.get("slot_id"), self.supported_slot_ids)
            self._selected_slot_at = now
        if (not parking_armed and self._selected_slot_id is not None and
                self._selected_slot_at is not None and
                now - self._selected_slot_at > self.selected_slot_latch_s):
            self._selected_slot_id = None
            self._selected_slot_at = None

        selected_slot_id = self._selected_slot_id
        if selected_slot_id is None and target_ok:
            selected_slot_id = normalise_slot_id(
                target.get("slot_id"), self.supported_slot_ids)
        active_target, target_reason, target_slot_status = (
            self._active_target_status(
                target, target_ok, parking_armed, selected_slot_id))
        parking_side = (normalise_parking_side(
            target.get("parking_side", target.get("turn_side")))
            if isinstance(target, dict) and active_target else None)
        lidar_clear = bool(lidar_ok and
                           lidar.get("obstacle_detected") is not True)

        # Every fresh valid metric source must agree on slot identity.  The
        # sign's P4/P5 value is only a preference; it is not a geometry source
        # and therefore must not veto a clear-bay fallback.
        source_slot_ids = []
        for source, source_ok in (
                (front, front_ok), (rear, rear_ok), (exit_data, exit_ok)):
            if source_ok and isinstance(source, dict):
                source_slot = normalise_slot_id(
                    source.get("slot_id"), self.supported_slot_ids)
                if source_slot is not None:
                    source_slot_ids.append(source_slot)
        if target_ok:
            target_slot = normalise_slot_id(
                target.get("slot_id"), self.supported_slot_ids)
            if target_slot is not None:
                source_slot_ids.append(target_slot)
        slot_consistent = len(set(source_slot_ids)) <= 1

        slot_id = selected_slot_id
        if slot_id is None:
            for candidate in (target, front, rear, exit_data):
                if isinstance(candidate, dict):
                    slot_id = normalise_slot_id(
                        candidate.get("slot_id"), self.supported_slot_ids)
                    if slot_id is not None:
                        break
        if slot_id is None:
            slot_id = self.default_slot_id

        source_confidence = {}
        confidence_values = []
        for source_name, source, source_ok in (
                ("front", front, front_ok),
                ("rear", rear, rear_ok),
                ("exit", exit_data, exit_ok),
                ("target", target, target_ok)):
            if source_ok:
                value = max(0.0, min(1.0,
                    number(source, "confidence", 0.0)))
                source_confidence[source_name] = float(value)
                confidence_values.append(value)
            else:
                source_confidence[source_name] = 0.0
        if isinstance(sign, dict):
            sign_confidence = number(sign, "confidence")
            if sign_confidence is not None:
                confidence_values.append(max(0.0, min(1.0, sign_confidence)))
        confidence = min(confidence_values) if confidence_values else 0.0

        front_clear = self._clear(front, "front_clear", lidar_clear)
        rear_clear = self._clear(rear, "rear_clear", lidar_clear)
        exit_clear = self._clear(exit_data, "exit_clear", lidar_clear)

        # Front owns setup pose; rear owns reverse pose once front explicitly
        # reports reverse_ready or disappears.  Both are already base_link.
        front_ready = (_bool_or_none(front, "reverse_ready")
                       if front_ok else None)
        if front_ok and front_ready is not True:
            pose_source = front
            pose_source_name = "front"
        elif rear_ok:
            pose_source = rear
            pose_source_name = "rear"
        elif front_ok:
            pose_source = front
            pose_source_name = "front"
        else:
            pose_source = {}
            pose_source_name = None

        parking_trigger_visible = bool(
            parking_armed and active_target and front_ok and
            front.get("parking_trigger_visible") is True)
        blue_trigger_visible = bool(
            parking_armed and active_target and
            target.get("blue_trigger_visible") is True
            if isinstance(target, dict) else False)
        legacy_request = self._legacy_parking_request(
            now, front, front_ok)
        parking_request = bool(parking_armed or legacy_request)
        if not parking_armed:
            # A raw blue start line may be visible before a P sign.  It is
            # reported for diagnostics but cannot become the mission trigger.
            parking_trigger_visible = False
            blue_trigger_visible = False

        if not parking_armed:
            parking_phase = "disarmed"
        elif not active_target:
            parking_phase = "armed_wait_target"
        elif not parking_trigger_visible:
            parking_phase = "armed_wait_blue"
        else:
            parking_phase = "blue_triggered"

        target_obstacle = None
        if isinstance(target_slot_status, dict):
            value = target_slot_status.get("obstacle_detected")
            if isinstance(value, bool):
                target_obstacle = value
        observation = {
            "schema": "parking_observation_v1",
            "stamp": now,
            "frame_id": "base_link",
            "coordinate_frame": "base_link",
            # Keep source validity separate from mission readiness.  Main.py
            # uses parking_armed/slot_selection_ready before starting the FSM;
            # once started, ParkingController enforces the same target latch.
            "valid": bool((front_ok or rear_ok or exit_ok or target_ok) and
                          slot_consistent),
            "confidence": float(confidence),
            "source_confidence": source_confidence,
            "parking_armed": bool(parking_armed),
            "parking_request": parking_request,
            "slot_id": slot_id,
            "selected_slot_id": self._selected_slot_id,
            "slot_selection_ready": bool(active_target),
            "slot_obstacle_detected": target_obstacle,
            "parking_side": parking_side,
            "turn_side": parking_side,
            "turn_sign": (1 if parking_side == "left" else
                           -1 if parking_side == "right" else None),
            "target_valid": bool(target_ok),
            "target_reason": target_reason,
            "parking_phase": parking_phase,
            "slot_consistent": bool(slot_consistent),
            "slot_visible": bool(front_ok and
                                  front.get("slot_visible") is True),
            "parking_trigger_visible": parking_trigger_visible,
            "blue_trigger_visible": blue_trigger_visible,
            "start_line_visible": bool(
                front_ok and front.get("start_line_visible") is True),
            "start_line_distance_m": number(
                front, "start_line_distance_m") if front_ok else None,
            "rear_visible": bool(rear_ok and
                                  rear.get("rear_visible") is True),
            "exit_visible": bool(exit_ok and
                                  exit_data.get("exit_visible") is True),
            "front_clear": bool(front_clear),
            "rear_clear": bool(rear_clear),
            "exit_clear": bool(exit_clear),
            "turn_distance_m": number(front, "turn_distance_m") if front_ok else None,
            "vehicle_yaw_error_rad": number(
                pose_source, "vehicle_yaw_error_rad"),
            "lateral_error_m": number(pose_source, "lateral_error_m"),
            "pose_source": pose_source_name,
            "rear_distance_m": number(rear, "rear_distance_m") if rear_ok else None,
            "reverse_ready": (_bool_or_none(front, "reverse_ready")
                              if front_ok else False),
            "exit_distance_m": number(
                exit_data, "exit_distance_m") if exit_ok else None,
            "exit_yaw_error_rad": number(
                exit_data, "exit_yaw_error_rad") if exit_ok else None,
            "exit_lateral_error_m": number(
                exit_data, "exit_lateral_error_m") if exit_ok else None,
            "exit_complete": (_bool_or_none(exit_data, "exit_complete")
                               if exit_ok else False),
            "emergency_stop": bool(
                (isinstance(lidar, dict) and lidar.get("emergency_stop") is True) or
                (isinstance(front, dict) and front.get("emergency_stop") is True) or
                (isinstance(rear, dict) and rear.get("emergency_stop") is True) or
                (isinstance(exit_data, dict) and
                 exit_data.get("emergency_stop") is True) or
                (isinstance(target, dict) and
                 target.get("emergency_stop") is True)
            ),
            "lidar_valid": bool(lidar_ok),
            "lidar_obstacle_detected": (
                lidar.get("obstacle_detected")
                if isinstance(lidar, dict) and
                isinstance(lidar.get("obstacle_detected"), bool)
                else None),
            "target": target if isinstance(target, dict) else None,
            "source_status": {
                "front": self._status(front_meta, front_ok),
                "rear": self._status(rear_meta, rear_ok),
                "exit": self._status(exit_meta, exit_ok),
                "target": self._status(target_meta, target_ok),
                "lidar": self._status(lidar_meta, lidar_ok),
                "sign": self._status(sign_meta, sign is not None),
            },
        }
        return observation

    def _fresh_source(self, source_name, now, timeout=None):
        entry = self._sources.get(source_name)
        if entry is None:
            return None, {"present": False, "fresh": False, "age_s": None}
        if timeout is None:
            timeout = self.input_timeout_s
        age = max(0.0, now - float(entry["received_at"]))
        fresh = age <= float(timeout)
        metadata = {
            "present": True,
            "fresh": bool(fresh),
            "age_s": age,
            "received_at": float(entry["received_at"]),
            "source_stamp": entry.get("source_stamp"),
        }
        return (entry["data"] if fresh else None), metadata

    def _metric_source_ok(self, source_name, data):
        if not isinstance(data, dict):
            return False
        if data.get("schema") != SOURCE_SCHEMAS[source_name]:
            return False
        if data.get("valid") is not True:
            return False
        if data.get("frame_id", data.get("coordinate_frame")) != "base_link":
            return False
        slot_id = normalise_slot_id(data.get("slot_id"), self.supported_slot_ids)
        if slot_id is None:
            return False
        confidence = number(data, "confidence")
        return (confidence is not None and
                confidence >= self.min_source_confidence)

    def _target_source_ok(self, data):
        """Accept only a ready, clear, concrete target selector result."""
        if not isinstance(data, dict):
            return False
        if data.get("schema") != SOURCE_SCHEMAS["target"]:
            return False
        if data.get("valid") is not True or data.get("selection_ready") is not True:
            return False
        if data.get("frame_id", data.get("coordinate_frame")) != "base_link":
            return False
        if normalise_slot_id(data.get("slot_id"), self.supported_slot_ids) is None:
            return False
        if data.get("candidate") is not True:
            return False
        if data.get("lidar_valid") is not True:
            return False
        if data.get("slot_obstacle_detected") is not False:
            return False
        if normalise_parking_side(
                data.get("parking_side", data.get("turn_side"))) is None:
            return False
        confidence = number(data, "confidence")
        return confidence is not None and confidence >= self.min_source_confidence

    def _active_target_status(self, target, target_ok, parking_armed,
                              selected_slot_id):
        if not parking_armed:
            return False, "parking_not_armed", None
        if selected_slot_id is None:
            return False, "slot_not_selected", None
        if not target_ok:
            reason = (target.get("target_reason", target.get("reason", "target_not_ready"))
                      if isinstance(target, dict) else "target_not_ready")
            status = self._slot_status_from_target(target, selected_slot_id)
            return False, str(reason), status
        target_slot = normalise_slot_id(
            target.get("slot_id"), self.supported_slot_ids)
        status = self._slot_status_from_target(target, selected_slot_id)
        if target_slot != selected_slot_id:
            return False, "latched_slot_not_current", status
        if normalise_parking_side(
                target.get("parking_side", target.get("turn_side"))) is None:
            return False, "parking_side_missing", status
        if not isinstance(status, dict):
            return False, "selected_slot_status_missing", status
        if status.get("obstacle_detected") is True:
            return False, "selected_slot_obstructed", status
        if status.get("lidar_valid") is not True:
            return False, "selected_slot_lidar_invalid", status
        if status.get("obstacle_detected") is not False:
            return False, "selected_slot_obstacle_status_missing", status
        return True, "selected_slot_clear", status

    @staticmethod
    def _slot_status_from_target(target, slot_id):
        if not isinstance(target, dict) or slot_id is None:
            return None
        slots = target.get("slots")
        if isinstance(slots, dict) and isinstance(slots.get(slot_id), dict):
            return dict(slots[slot_id])
        if target.get("slot_id") == slot_id:
            return {
                "visible": target.get("candidate") is True,
                "lidar_valid": target.get("lidar_valid"),
                "obstacle_detected": target.get("slot_obstacle_detected"),
                "min_distance_m": target.get("min_distance_m"),
                "point_count": target.get("point_count", 0),
                "reason": target.get("reason"),
            }
        return None

    @staticmethod
    def _lidar_ok(data):
        if not isinstance(data, dict):
            return False
        if data.get("schema") != "parking_lidar_v1":
            return False
        if data.get("frame_id", data.get("coordinate_frame")) != "base_link":
            return False
        if data.get("valid") is not True:
            return False
        return isinstance(data.get("obstacle_detected"), bool)

    def _clear(self, source, field, lidar_clear):
        explicit = _bool_or_none(source, field)
        if explicit is not None:
            return explicit
        if self.derive_clear_from_lidar:
            return lidar_clear
        return False

    @staticmethod
    def _status(metadata, valid):
        status = dict(metadata or {})
        status["valid"] = bool(valid)
        return status

    def _remember_sign(self, sign, received_at):
        if not isinstance(sign, dict):
            return
        label = sign.get("label", sign.get("class", sign.get("sign")))
        if not isinstance(label, string_types):
            return
        label = label.strip().lower()
        confidence = number(sign, "confidence", 1.0)
        if confidence is None or confidence < self.min_source_confidence:
            return
        slot_id = normalise_slot_id(sign.get("slot_id"), self.supported_slot_ids)
        label_slot = normalise_slot_id(label, self.supported_slot_ids)
        if slot_id is None:
            slot_id = label_slot
        explicit_slot_value = sign.get("slot_id")
        if (explicit_slot_value is not None and slot_id is None and
                str(explicit_slot_value).strip().upper() not in (
                    "AUTO", "ANY", "NONE", "NULL", "")):
            return
        self._last_sign = {
            "label": label,
            "confidence": confidence,
            "received_at": float(received_at),
            "slot_id": slot_id,
        }
        parking_label = label in self.parking_sign_labels or label in (
            "p4", "p5")
        if parking_label:
            until = float(received_at) + self.parking_request_hold_s
            self._parking_armed_until = max(self._parking_armed_until, until)
            self._parking_request_until = max(self._parking_request_until, until)

    def _parking_armed(self, now):
        return self._parking_armed_until >= float(now)

    def _legacy_parking_request(self, now, front=None, front_ok=False):
        if (self.request_from_front_slot and front_ok and
                isinstance(front, dict) and
                front.get("parking_trigger_visible") is True):
            return True
        # Explicit legacy metric requests remain bounded by their source
        # freshness.  A target blue line alone is never included here.
        for source_name in ("front", "rear", "exit"):
            data, _ = self._fresh_source(source_name, now)
            source_slot = (normalise_slot_id(
                data.get("slot_id"), self.supported_slot_ids)
                if isinstance(data, dict) else None)
            if (isinstance(data, dict) and
                    data.get("parking_request") is True and
                    source_slot is not None):
                return True
        return False

    def _parking_request(self, now, front=None, front_ok=False):
        """Compatibility helper retained for older replay callers."""
        return bool(self._parking_armed(now) or
                    self._legacy_parking_request(now, front, front_ok))
