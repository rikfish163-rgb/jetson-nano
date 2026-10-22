#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Closed-loop reverse-parking state machine.

The controller is deliberately ROS-independent.  A vision/odometry node
publishes one JSON observation per frame and this module returns a bounded raw
speed/steering command.  All geometric errors are expressed in ``base_link``:

* x is vehicle-forward, y is vehicle-left;
* ``vehicle_yaw_error_rad`` is vehicle yaw relative to the slot centreline;
  positive means the vehicle points to the left of the slot;
* ``lateral_error_m`` is the rear-axle lateral error; positive means left of
  the slot centreline;
* distances are positive remaining distances along the relevant line.

The default configuration is intentionally uncalibrated.  It will only emit
zero commands until ``calibration_complete`` is explicitly enabled.
"""

from __future__ import division

import math
import time

try:
    string_types = (basestring,)
except NameError:
    string_types = (str,)


STATE_IDLE = "idle"
STATE_WAIT_BLUE = "wait_blue"
STATE_APPROACH = "approach"
STATE_FORWARD_RIGHT_TURN = "forward_right_turn"
STATE_FORWARD_STRAIGHTEN = "forward_straighten"
STATE_SETTLE = "settle"
STATE_REVERSE_STEER_IN = "reverse_steer_in"
STATE_REVERSE_STRAIGHTEN = "reverse_straighten"
STATE_REVERSE_ALIGN = "reverse_align"
STATE_PARKED = "parked"
STATE_EXIT_FORWARD_CLEARANCE = "exit_forward_clearance"
STATE_EXIT_TURN = "exit_turn"
STATE_EXIT_STRAIGHTEN = "exit_straighten"
STATE_DONE = "done"
STATE_FAULT = "fault"


# The nominal route is deliberately explicit.  Perception supplies the
# measured gate/error values; it never selects a new motion state.  Keeping
# this table next to the state constants makes an accidental phase skip a
# controller fault instead of an unbounded command.
STATE_SEQUENCE = (
    STATE_IDLE,
    STATE_APPROACH,
    STATE_FORWARD_RIGHT_TURN,
    STATE_FORWARD_STRAIGHTEN,
    STATE_SETTLE,
    STATE_REVERSE_STEER_IN,
    STATE_REVERSE_STRAIGHTEN,
    STATE_REVERSE_ALIGN,
    STATE_PARKED,
    STATE_EXIT_FORWARD_CLEARANCE,
    STATE_EXIT_TURN,
    STATE_EXIT_STRAIGHTEN,
    STATE_DONE,
    # Appended to preserve the historical phase indices used by replay/UI
    # consumers; the actual route enters this gate immediately after arming.
    STATE_WAIT_BLUE,
)
STATE_INDEX = dict((state, index) for index, state in enumerate(STATE_SEQUENCE))
ALLOWED_TRANSITIONS = {
    STATE_IDLE: (STATE_WAIT_BLUE, STATE_APPROACH),
    STATE_WAIT_BLUE: (STATE_APPROACH,),
    STATE_APPROACH: (STATE_FORWARD_RIGHT_TURN,),
    STATE_FORWARD_RIGHT_TURN: (STATE_FORWARD_STRAIGHTEN,),
    STATE_FORWARD_STRAIGHTEN: (STATE_SETTLE,),
    STATE_SETTLE: (STATE_REVERSE_STEER_IN,),
    STATE_REVERSE_STEER_IN: (STATE_REVERSE_STRAIGHTEN,),
    STATE_REVERSE_STRAIGHTEN: (STATE_REVERSE_ALIGN,),
    STATE_REVERSE_ALIGN: (STATE_PARKED,),
    STATE_PARKED: (STATE_EXIT_FORWARD_CLEARANCE, STATE_EXIT_TURN),
    STATE_EXIT_FORWARD_CLEARANCE: (STATE_EXIT_TURN,),
    STATE_EXIT_TURN: (STATE_EXIT_STRAIGHTEN,),
    STATE_EXIT_STRAIGHTEN: (STATE_DONE,),
    STATE_DONE: (),
    STATE_FAULT: (),
}

STATE_TIMEOUT_FIELDS = {
    STATE_WAIT_BLUE: "wait_blue_timeout_s",
    STATE_APPROACH: "approach_timeout_s",
    STATE_FORWARD_RIGHT_TURN: "forward_turn_timeout_s",
    STATE_FORWARD_STRAIGHTEN: "forward_straighten_timeout_s",
    STATE_SETTLE: "settle_timeout_s",
    STATE_REVERSE_STEER_IN: "reverse_steer_in_timeout_s",
    STATE_REVERSE_STRAIGHTEN: "reverse_straighten_timeout_s",
    STATE_REVERSE_ALIGN: "reverse_align_timeout_s",
    STATE_PARKED: "parked_timeout_s",
    STATE_EXIT_FORWARD_CLEARANCE: "exit_forward_clearance_timeout_s",
    STATE_EXIT_TURN: "exit_turn_timeout_s",
    STATE_EXIT_STRAIGHTEN: "exit_straighten_timeout_s",
}

STATE_REQUIRED_SOURCES = {
    STATE_WAIT_BLUE: "target",
    STATE_APPROACH: "front",
    STATE_FORWARD_RIGHT_TURN: "front",
    STATE_FORWARD_STRAIGHTEN: "front",
    STATE_REVERSE_STEER_IN: "rear",
    STATE_REVERSE_STRAIGHTEN: "rear",
    STATE_REVERSE_ALIGN: "rear",
    # Parking is confirmed by the rear slot geometry.  Exit perception is
    # required only once the vehicle is about to leave the bay; making parked
    # depend on exit data would make a successful reverse insertion impossible
    # to report while the front camera is still acquiring the exit line.
    STATE_PARKED: "rear",
    STATE_EXIT_FORWARD_CLEARANCE: "rear",
    STATE_EXIT_TURN: "exit",
    STATE_EXIT_STRAIGHTEN: "exit",
}

# A camera can lose a fitted line for a few frames while the vehicle is
# turning.  The forward setup may bridge only this short visual dropout; all
# other phases remain strict so a stale camera cannot drive the car into the
# bay or out of it.  Lidar/emergency safety checks run before this bridge.
VISUAL_HOLD_STATES = (
    STATE_FORWARD_RIGHT_TURN,
    STATE_FORWARD_STRAIGHTEN,
)
VISUAL_HOLD_VALIDATION_REASONS = (
    "observation_missing",
    "observation_invalid",
    "source_confidence_low",
    "source_status_invalid",
    "source_status_stale",
    "confidence_low",
    "observation_stale",
)
VISUAL_HOLD_HANDLER_REASONS = (
    "slot_not_visible",
    "yaw_error_missing",
    "lateral_error_missing",
)


def _is_finite(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return False
    return not math.isnan(value) and not math.isinf(value)


def normalise_parking_side(value):
    """Return a concrete vehicle-relative side or None for AUTO."""
    if not isinstance(value, string_types):
        return None
    value = value.strip().lower()
    if value in ("left", "l", "+1", "positive"):
        return "left"
    if value in ("right", "r", "-1", "negative"):
        return "right"
    return None


def parking_side_sign(parking_side):
    """Return +1 for left steering and -1 for right steering."""
    side = normalise_parking_side(parking_side)
    if side == "left":
        return 1
    if side == "right":
        return -1
    return None


# Measured steering actuator scale.  The parking controller's public command
# contract remains integer raw units; these helpers are the only place where
# a requested physical steering angle is quantised to that contract.
STEERING_RAW_LIMIT = 22
STEERING_MAX_ANGLE_DEG = 26.515
STEERING_MAX_ANGLE_RAD = 0.46275
STEERING_DEG_PER_RAW = STEERING_MAX_ANGLE_DEG / STEERING_RAW_LIMIT
STEERING_RAD_PER_RAW = STEERING_MAX_ANGLE_RAD / STEERING_RAW_LIMIT


def steering_angle_deg_to_raw(angle_deg, raw_limit=STEERING_RAW_LIMIT):
    """Convert a physical steering angle to bounded integer raw units.

    The magnitude is rounded upward so a requested physical angle is never
    under-commanded by quantisation.  Sign is restored after rounding; using
    ``math.ceil(angle_deg)`` directly would be wrong for negative angles.
    """
    if not _is_finite(angle_deg):
        raise ValueError("steering angle must be finite")
    if not isinstance(raw_limit, int) or isinstance(raw_limit, bool):
        raise ValueError("raw_limit must be an integer")
    if raw_limit <= 0:
        raise ValueError("raw_limit must be positive")
    angle_deg = float(angle_deg)
    if angle_deg == 0.0:
        return 0
    raw = int(math.ceil(
        abs(angle_deg) * raw_limit / STEERING_MAX_ANGLE_DEG))
    raw = min(raw_limit, raw)
    return raw if angle_deg > 0.0 else -raw


def steering_angle_rad_to_raw(angle_rad, raw_limit=STEERING_RAW_LIMIT):
    """Convert a physical steering angle in radians to integer raw units."""
    if not _is_finite(angle_rad):
        raise ValueError("steering angle must be finite")
    if not isinstance(raw_limit, int) or isinstance(raw_limit, bool):
        raise ValueError("raw_limit must be an integer")
    if raw_limit <= 0:
        raise ValueError("raw_limit must be positive")
    angle_rad = float(angle_rad)
    if angle_rad == 0.0:
        return 0
    raw = int(math.ceil(
        abs(angle_rad) * raw_limit / STEERING_MAX_ANGLE_RAD))
    raw = min(raw_limit, raw)
    return raw if angle_rad > 0.0 else -raw


def steering_raw_to_angle_deg(steering_raw):
    """Return the exact physical degree value represented by one raw unit."""
    if (isinstance(steering_raw, bool) or
            not isinstance(steering_raw, int)):
        raise ValueError("steering_raw must be an integer")
    if steering_raw < -STEERING_RAW_LIMIT or steering_raw > STEERING_RAW_LIMIT:
        raise ValueError("steering_raw must be in the range -22..22")
    return float(steering_raw) * STEERING_DEG_PER_RAW


class ParkingConfig(object):
    """Validated parameters for one generic perpendicular parking manoeuvre."""

    # Slot profiles describe map-specific geometry and phase tuning only.
    # Vehicle/chassis limits, sensor quality policy and the calibration lock
    # are global; accepting them in a P4/P5 override would make the effective
    # safety contract depend on an unreviewed slot selection.
    SLOT_PROFILE_FIELDS = (
        "parking_side",
        "slot_length_m",
        "slot_width_m",
        "safety_margin_m",
        "turn_start_distance_m",
        "forward_turn_target_yaw_error_rad",
        "setup_yaw_tolerance_rad",
        "reverse_steer_switch_distance_m",
        "reverse_yaw_tolerance_rad",
        "park_stop_distance_m",
        "park_lateral_tolerance_m",
        "park_yaw_tolerance_rad",
        "exit_straighten_distance_m",
        "exit_complete_distance_m",
        "exit_yaw_tolerance_rad",
        "exit_lateral_tolerance_m",
        "exit_turn_require_yaw_alignment",
        "exit_forward_clearance_distance_m",
        "exit_forward_yaw_tolerance_rad",
        "exit_forward_lateral_tolerance_m",
        "exit_forward_steering_raw",
        "entry_steering_raw",
        "straighten_steering_raw",
        "exit_steering_raw",
        "exit_straighten_steering_raw",
        "forward_turn_lateral_gain_raw_per_m",
        "forward_turn_yaw_gain_raw_per_rad",
        "forward_straighten_lateral_gain_raw_per_m",
        "forward_straighten_yaw_gain_raw_per_rad",
        "reverse_entry_lateral_gain_raw_per_m",
        "reverse_entry_yaw_gain_raw_per_rad",
        "reverse_straighten_lateral_gain_raw_per_m",
        "reverse_straighten_yaw_gain_raw_per_rad",
        "exit_turn_lateral_gain_raw_per_m",
        "exit_turn_yaw_gain_raw_per_rad",
        "exit_straighten_lateral_gain_raw_per_m",
        "exit_straighten_yaw_gain_raw_per_rad",
    )

    # A calibrated runtime must not silently inherit the generic placeholder
    # route from DEFAULTS.  These are the per-slot geometry/route values that
    # have to be written explicitly after measuring the actual P4/P5 layout.
    # Feedback gains remain optional: zero is a deliberate fixed-angle mode,
    # while non-zero gains are separately guarded by their observation fields.
    CALIBRATED_SLOT_FIELDS = (
        "parking_side",
        "slot_length_m",
        "slot_width_m",
        "safety_margin_m",
        "turn_start_distance_m",
        "forward_turn_target_yaw_error_rad",
        "setup_yaw_tolerance_rad",
        "reverse_steer_switch_distance_m",
        "reverse_yaw_tolerance_rad",
        "park_stop_distance_m",
        "park_lateral_tolerance_m",
        "park_yaw_tolerance_rad",
        "exit_straighten_distance_m",
        "exit_complete_distance_m",
        "exit_yaw_tolerance_rad",
        "exit_lateral_tolerance_m",
        "exit_turn_require_yaw_alignment",
        "exit_forward_clearance_distance_m",
        "exit_forward_yaw_tolerance_rad",
        "exit_forward_lateral_tolerance_m",
        "exit_forward_steering_raw",
        "entry_steering_raw",
        "straighten_steering_raw",
        "exit_steering_raw",
        "exit_straighten_steering_raw",
    )

    DEFAULTS = {
        # The vehicle and slot dimensions are metadata for calibration and
        # future geometric checks.  They must be replaced with measured data.
        "wheelbase_m": 0.29,
        "vehicle_length_m": 0.45,
        "vehicle_width_m": 0.30,
        "front_overhang_m": 0.08,
        "rear_overhang_m": 0.08,
        "slot_length_m": 0.45,
        "slot_width_m": 0.38,
        "safety_margin_m": 0.05,

        # Raw values are the existing byte-oriented chassis contract, not SI
        # units.  The latest field check established that positive speed is
        # forward and negative speed is reverse.
        "forward_speed_raw": 12,
        "reverse_speed_raw": -10,
        "exit_speed_raw": 10,
        "entry_steering_raw": -22,
        "straighten_steering_raw": 8,
        "exit_steering_raw": 10,
        "exit_straighten_steering_raw": -6,
        "max_speed_raw": 100,
        "max_steering_raw": 22,

        # Safety and timing.
        "calibration_complete": False,
        "require_lidar_clear": True,
        "observation_timeout_s": 0.50,
        "min_confidence": 0.70,
        # P-sign arm to blue-line trigger is a bounded waiting phase.  It
        # never emits motion; the preparking lane policy lives in main.py.
        "wait_blue_timeout_s": 30.0,
        # Keep a short, bounded last-command bridge for line dropouts during
        # the forward turn/straighten setup.  Zero disables the bridge; the
        # actual left/right sign is selected from parking_side at start().
        "visual_dropout_hold_s": 0.35,
        "condition_debounce_frames": 3,
        "settle_time_s": 0.50,
        "parked_hold_s": 1.00,
        "auto_exit": True,

        # Map/slot selection.  Slot-specific overrides can be placed in
        # ``slot_profiles`` without changing the state machine.
        "default_slot_id": "P4",
        "supported_slot_ids": ("P4", "P5"),
        # Keep the historical static-fixture/default profile deterministic.
        # The deployed reform launch overrides this with parking.yaml=auto and
        # passes the concrete side selected by the target selector.
        "parking_side": "right",
        "slot_profiles": {},

        # Forward setup thresholds.
        "turn_start_distance_m": 0.70,
        "forward_turn_target_yaw_error_rad": -0.55,
        "setup_yaw_tolerance_rad": 0.10,

        # Reverse insertion thresholds.
        "reverse_steer_switch_distance_m": 0.55,
        "reverse_yaw_tolerance_rad": 0.10,
        "park_stop_distance_m": 0.15,
        "park_lateral_tolerance_m": 0.08,
        "park_yaw_tolerance_rad": 0.08,

        # Exit thresholds.
        "exit_straighten_distance_m": 0.45,
        "exit_complete_distance_m": 0.12,
        "exit_yaw_tolerance_rad": 0.12,
        "exit_lateral_tolerance_m": 0.15,
        # The mirrored exit turn ends on a live white-line yaw estimate.  The
        # distance threshold remains a fallback only when this is false.
        "exit_turn_require_yaw_alignment": True,
        # A positive value inserts a rear-camera-verified forward clearance
        # segment after parking.  Zero preserves the old direct exit path for
        # synthetic/replay configurations that do not provide this geometry.
        "exit_forward_clearance_distance_m": 0.0,
        "exit_forward_yaw_tolerance_rad": 0.12,
        "exit_forward_lateral_tolerance_m": 0.15,
        "exit_forward_steering_raw": 0,

        # Reverse visual feedback gains.  Output is raw steering units.
        "steering_lateral_gain_raw_per_m": 18.0,
        "steering_yaw_gain_raw_per_rad": 8.0,

        # Optional bounded visual feedback around the nominal steering values.
        # Zero keeps the original fixed-angle behaviour until the signs and
        # gains are measured on the real chassis.
        "forward_turn_lateral_gain_raw_per_m": 0.0,
        "forward_turn_yaw_gain_raw_per_rad": 0.0,
        "forward_straighten_lateral_gain_raw_per_m": 0.0,
        "forward_straighten_yaw_gain_raw_per_rad": 0.0,
        "reverse_entry_lateral_gain_raw_per_m": 0.0,
        "reverse_entry_yaw_gain_raw_per_rad": 0.0,
        "reverse_straighten_lateral_gain_raw_per_m": 0.0,
        "reverse_straighten_yaw_gain_raw_per_rad": 0.0,
        "exit_turn_lateral_gain_raw_per_m": 0.0,
        "exit_turn_yaw_gain_raw_per_rad": 0.0,
        "exit_straighten_lateral_gain_raw_per_m": 0.0,
        "exit_straighten_yaw_gain_raw_per_rad": 0.0,

        # A lost visual source must not leave a motion state alive forever.
        "approach_timeout_s": 10.0,
        "forward_turn_timeout_s": 10.0,
        "forward_straighten_timeout_s": 8.0,
        "settle_timeout_s": 4.0,
        "reverse_steer_in_timeout_s": 10.0,
        "reverse_straighten_timeout_s": 8.0,
        "reverse_align_timeout_s": 12.0,
        "parked_timeout_s": 120.0,
        "exit_turn_timeout_s": 10.0,
        "exit_straighten_timeout_s": 8.0,
        "exit_forward_clearance_timeout_s": 8.0,
    }

    _POSITIVE_FIELDS = (
        "wheelbase_m",
        "vehicle_length_m",
        "vehicle_width_m",
        "front_overhang_m",
        "rear_overhang_m",
        "slot_length_m",
        "slot_width_m",
        "safety_margin_m",
        "max_speed_raw",
        "max_steering_raw",
        "observation_timeout_s",
        "condition_debounce_frames",
        "wait_blue_timeout_s",
        "turn_start_distance_m",
        "setup_yaw_tolerance_rad",
        "reverse_steer_switch_distance_m",
        "reverse_yaw_tolerance_rad",
        "park_stop_distance_m",
        "park_lateral_tolerance_m",
        "park_yaw_tolerance_rad",
        "exit_straighten_distance_m",
        "exit_complete_distance_m",
        "exit_yaw_tolerance_rad",
        "exit_lateral_tolerance_m",
        "exit_forward_yaw_tolerance_rad",
        "exit_forward_lateral_tolerance_m",
        "approach_timeout_s",
        "forward_turn_timeout_s",
        "forward_straighten_timeout_s",
        "settle_timeout_s",
        "reverse_steer_in_timeout_s",
        "reverse_straighten_timeout_s",
        "reverse_align_timeout_s",
        "parked_timeout_s",
        "exit_turn_timeout_s",
        "exit_straighten_timeout_s",
        "exit_forward_clearance_timeout_s",
    )

    _NON_NEGATIVE_FIELDS = (
        "settle_time_s",
        "parked_hold_s",
        "exit_forward_clearance_distance_m",
        "visual_dropout_hold_s",
    )

    @classmethod
    def from_dict(cls, values=None, _require_slot_profiles=None):
        if values is None:
            values = {}
        if not isinstance(values, dict):
            raise ValueError("parking configuration must be a JSON/YAML object")
        return cls(values, _require_slot_profiles=_require_slot_profiles)

    def __init__(self, values=None, _require_slot_profiles=None):
        values = {} if values is None else dict(values)
        unknown = sorted(set(values.keys()) - set(self.DEFAULTS.keys()))
        if unknown:
            raise ValueError("unknown parking parameters: %s" % ", ".join(unknown))

        self.values = dict(self.DEFAULTS)
        self.values.update(values)
        for key, value in self.values.items():
            setattr(self, key, value)
        # A calibrated runtime configuration must describe every supported
        # slot.  ``profile_for`` validates an already merged profile below;
        # that internal pass explicitly disables this presence check because
        # the merged object intentionally no longer contains slot_profiles.
        if _require_slot_profiles is None:
            _require_slot_profiles = self.calibration_complete
        self.validate(require_slot_profiles=_require_slot_profiles)

    def validate(self, require_slot_profiles=None):
        if not isinstance(self.calibration_complete, bool):
            raise ValueError("calibration_complete must be boolean")
        if not isinstance(self.require_lidar_clear, bool):
            raise ValueError("require_lidar_clear must be boolean")
        if not isinstance(self.auto_exit, bool):
            raise ValueError("auto_exit must be boolean")
        if not isinstance(self.exit_turn_require_yaw_alignment, bool):
            raise ValueError("exit_turn_require_yaw_alignment must be boolean")
        if (not isinstance(self.default_slot_id, string_types) or
                not self.default_slot_id):
            raise ValueError("default_slot_id must be a non-empty string")
        if (not isinstance(self.supported_slot_ids, (list, tuple)) or
                not self.supported_slot_ids):
            raise ValueError("supported_slot_ids must be a non-empty list")
        for slot_id in self.supported_slot_ids:
            if not isinstance(slot_id, string_types) or not slot_id:
                raise ValueError(
                    "supported_slot_ids must contain non-empty strings")
        if len(set(self.supported_slot_ids)) != len(self.supported_slot_ids):
            raise ValueError("supported_slot_ids must not contain duplicates")
        default_slot_auto = (isinstance(self.default_slot_id, string_types) and
                             self.default_slot_id.strip().upper() in
                             ("AUTO", "ANY", "NONE"))
        if self.default_slot_id not in self.supported_slot_ids and not default_slot_auto:
            raise ValueError(
                "default_slot_id must be included in supported_slot_ids")
        # The P4/P5 FSM is mirrored at runtime from the selected bay side.
        # AUTO is allowed at launch, but a concrete side is required before
        # the controller owns a motion command.
        if (not isinstance(self.parking_side, string_types) or
                self.parking_side.strip().lower() not in
                ("left", "right", "auto", "any", "none")):
            raise ValueError(
                "parking_side must be left, right, or auto for the P4/P5 FSM")
        if not isinstance(self.slot_profiles, dict):
            raise ValueError("slot_profiles must be an object")
        for profile_slot, profile in self.slot_profiles.items():
            if not isinstance(profile_slot, string_types) or not profile_slot:
                raise ValueError(
                    "slot_profiles keys must be non-empty strings")
            if not isinstance(profile, dict):
                raise ValueError(
                    "slot profile %s must be an object" % profile_slot)
        profile_slots = set(self.slot_profiles.keys())
        unsupported_profiles = sorted(
            profile_slots - set(self.supported_slot_ids))
        if unsupported_profiles:
            raise ValueError(
                "slot_profiles contain unsupported slots: %s" %
                ", ".join(unsupported_profiles))
        if require_slot_profiles is None:
            require_slot_profiles = self.calibration_complete
        if not isinstance(require_slot_profiles, bool):
            raise ValueError("require_slot_profiles must be boolean")
        if require_slot_profiles:
            missing_profiles = sorted(
                set(self.supported_slot_ids) - profile_slots)
            if missing_profiles:
                raise ValueError(
                    "calibrated config requires slot_profiles for: %s" %
                    ", ".join(missing_profiles))
            incomplete_profiles = {}
            for profile_slot in self.supported_slot_ids:
                profile = self.slot_profiles.get(profile_slot, {})
                missing_fields = [
                    field for field in self.CALIBRATED_SLOT_FIELDS
                    if field not in profile
                ]
                if missing_fields:
                    incomplete_profiles[profile_slot] = missing_fields
            if incomplete_profiles:
                details = "; ".join(
                    "%s: %s" % (slot_id, ", ".join(fields))
                    for slot_id, fields in sorted(incomplete_profiles.items())
                )
                raise ValueError(
                    "calibrated slot profiles missing measured fields: %s" %
                    details)

        for field in self._POSITIVE_FIELDS:
            value = getattr(self, field)
            if not _is_finite(value) or float(value) <= 0.0:
                raise ValueError("%s must be positive and finite" % field)
        for field in self._NON_NEGATIVE_FIELDS:
            value = getattr(self, field)
            if not _is_finite(value) or float(value) < 0.0:
                raise ValueError("%s must be non-negative and finite" % field)

        for field in (
                "forward_speed_raw",
                "reverse_speed_raw",
                "exit_speed_raw",
                "entry_steering_raw",
                "straighten_steering_raw",
                "exit_steering_raw",
                "exit_straighten_steering_raw",
                "exit_forward_steering_raw",
                "forward_turn_target_yaw_error_rad",
                "steering_lateral_gain_raw_per_m",
                "steering_yaw_gain_raw_per_rad",
                "forward_turn_lateral_gain_raw_per_m",
                "forward_turn_yaw_gain_raw_per_rad",
                "forward_straighten_lateral_gain_raw_per_m",
                "forward_straighten_yaw_gain_raw_per_rad",
                "reverse_entry_lateral_gain_raw_per_m",
                "reverse_entry_yaw_gain_raw_per_rad",
                "reverse_straighten_lateral_gain_raw_per_m",
                "reverse_straighten_yaw_gain_raw_per_rad",
                "exit_turn_lateral_gain_raw_per_m",
                "exit_turn_yaw_gain_raw_per_rad",
                "exit_straighten_lateral_gain_raw_per_m",
                "exit_straighten_yaw_gain_raw_per_rad"):
            if not _is_finite(getattr(self, field)):
                raise ValueError("%s must be finite" % field)

        if self.forward_speed_raw <= 0:
            raise ValueError("forward_speed_raw must be positive")
        if self.reverse_speed_raw >= 0:
            raise ValueError("reverse_speed_raw must be negative")
        if self.exit_speed_raw <= 0:
            raise ValueError("exit_speed_raw must be positive")
        if (not _is_finite(self.min_confidence) or
                self.min_confidence < 0.0 or self.min_confidence > 1.0):
            raise ValueError("min_confidence must be in the range 0..1")
        if self.visual_dropout_hold_s > 1.0:
            raise ValueError("visual_dropout_hold_s must not exceed 1 second")
        if self.condition_debounce_frames != int(self.condition_debounce_frames):
            raise ValueError("condition_debounce_frames must be an integer")
        if self.steering_lateral_gain_raw_per_m < 0 or self.steering_yaw_gain_raw_per_rad < 0:
            raise ValueError("reverse steering gains must not be negative")
        if self.reverse_steer_switch_distance_m <= self.park_stop_distance_m:
            raise ValueError(
                "reverse_steer_switch_distance_m must exceed park_stop_distance_m")
        if self.exit_straighten_distance_m <= self.exit_complete_distance_m:
            raise ValueError(
                "exit_straighten_distance_m must exceed exit_complete_distance_m")
        if (self.exit_forward_clearance_distance_m > 0.0 and
                self.exit_forward_clearance_distance_m <=
                self.park_stop_distance_m):
            raise ValueError(
                "exit_forward_clearance_distance_m must exceed park_stop_distance_m")

    def profile_for(self, slot_id):
        """Return global settings merged with an optional slot profile."""
        if slot_id not in self.supported_slot_ids:
            raise ValueError("slot_id_not_supported: %s" % slot_id)
        profile = dict(self.values)
        profiles = profile.pop("slot_profiles")
        override = profiles.get(slot_id, {})
        if not isinstance(override, dict):
            raise ValueError("slot profile %s must be an object" % slot_id)
        unknown = sorted(set(override.keys()) - set(self.DEFAULTS.keys()))
        if unknown:
            raise ValueError(
                "unknown parameters in slot profile %s: %s" %
                (slot_id, ", ".join(unknown)))
        forbidden = sorted(
            set(override.keys()) - set(self.SLOT_PROFILE_FIELDS))
        if forbidden:
            raise ValueError(
                "global-only parameters in slot profile %s: %s" %
                (slot_id, ", ".join(forbidden)))
        profile.update(override)
        # Validate the merged P4/P5 profile before motion starts.  Without
        # this second validation a bad slot-specific threshold could survive
        # the global config check and fail only after the vehicle moved.
        profile.pop("slot_profiles", None)
        return ParkingConfig(
            profile, _require_slot_profiles=False).values


class ParkingController(object):
    """Finite-state controller for forward setup, reverse insertion and exit."""

    # Experimental motion is an explicit bring-up lane, not a replacement for
    # the calibrated competition configuration.  Keep these hard ceilings in
    # code so a launch typo cannot turn the first field test into unbounded
    # motion.  The steering ceiling now matches the measured actuator range.
    EXPERIMENTAL_SPEED_LIMIT_RAW = 20
    EXPERIMENTAL_STEERING_LIMIT_RAW = 22

    def __init__(self, config=None, allow_experimental_motion=False,
                 experimental_speed_limit_raw=1,
                 experimental_steering_limit_raw=4):
        self.config = config if config is not None else ParkingConfig()
        self.experimental_motion = bool(allow_experimental_motion)
        self.experimental_speed_limit_raw = self._experimental_limit(
            experimental_speed_limit_raw,
            self.EXPERIMENTAL_SPEED_LIMIT_RAW,
            "experimental_speed_limit_raw",
        )
        self.experimental_steering_limit_raw = self._experimental_limit(
            experimental_steering_limit_raw,
            self.EXPERIMENTAL_STEERING_LIMIT_RAW,
            "experimental_steering_limit_raw",
        )
        self.state = STATE_IDLE
        self.reason = "idle"
        # ``state_reason`` is the reason for entering the current state;
        # ``reason``/``last_command_reason`` remain the last command result
        # for compatibility with the existing replay and debug tools.
        self.state_reason = "idle"
        self.last_command_reason = "idle"
        self.transition_seq = 0
        self.slot_id = None
        self.parking_side = None
        self.turn_sign = None
        self.profile = None
        self.state_entered_at = None
        self._condition_name = None
        self._condition_count = 0
        self._last_valid_observation = None
        self._last_valid_observation_at = None
        self._state_handlers = {
            STATE_WAIT_BLUE: self._step_wait_blue,
            STATE_APPROACH: self._step_approach,
            STATE_FORWARD_RIGHT_TURN: self._step_forward_right_turn,
            STATE_FORWARD_STRAIGHTEN: self._step_forward_straighten,
            STATE_SETTLE: self._step_settle,
            STATE_REVERSE_STEER_IN: self._step_reverse_steer_in,
            STATE_REVERSE_STRAIGHTEN: self._step_reverse_straighten,
            STATE_REVERSE_ALIGN: self._step_reverse_align,
            STATE_PARKED: self._step_parked,
            STATE_EXIT_FORWARD_CLEARANCE: self._step_exit_forward_clearance,
            STATE_EXIT_TURN: self._step_exit_turn,
            STATE_EXIT_STRAIGHTEN: self._step_exit_straighten,
        }

    @staticmethod
    def _experimental_limit(value, maximum, field):
        if isinstance(value, bool):
            raise ValueError("%s must be an integer" % field)
        try:
            value = int(value)
        except (TypeError, ValueError):
            raise ValueError("%s must be an integer" % field)
        if value <= 0 or value > maximum:
            raise ValueError(
                "%s must be in the range 1..%d" % (field, maximum))
        return value

    @property
    def active(self):
        return self.state not in (STATE_IDLE, STATE_DONE)

    @property
    def holding(self):
        """Whether main.py must retain parking ownership of the command."""
        return self.state != STATE_IDLE

    @property
    def done(self):
        return self.state == STATE_DONE

    @property
    def fault(self):
        return self.state == STATE_FAULT

    def start(self, slot_id=None, now=None, armed_only=False,
              parking_side=None):
        if self.state != STATE_IDLE:
            return False
        if now is None:
            now = time.time()
        if (not self.config.calibration_complete and
                not self.experimental_motion):
            self._fault("calibration_incomplete", now)
            return False

        slot_id = slot_id or self.config.default_slot_id
        # AUTO is a launch-time selection policy, not a usable controller
        # profile.  Main.py must bind a concrete P4/P5 target before this
        # controller can own the command stream.
        if isinstance(slot_id, string_types):
            slot_id = slot_id.strip().upper()
        if slot_id in ("AUTO", "ANY", "NONE", ""):
            self._fault("slot_id_required", now)
            return False
        if not isinstance(slot_id, string_types) or not slot_id:
            self._fault("slot_id_invalid", now)
            return False
        try:
            profile = self.config.profile_for(slot_id)
        except ValueError as exc:
            self._fault(str(exc), now)
            return False
        resolved_side = (normalise_parking_side(parking_side) or
                         normalise_parking_side(profile.get("parking_side")) or
                         normalise_parking_side(self.config.parking_side))
        turn_sign = parking_side_sign(resolved_side)
        if turn_sign is None:
            self._fault("parking_side_required", now)
            return False
        # Mirror the nominal route at the last safe boundary.  The feedback
        # terms remain profile-specific, but the fixed command values and
        # signed yaw target must follow the observed bay side.
        profile = dict(profile)
        profile["parking_side"] = resolved_side
        profile["forward_turn_target_yaw_error_rad"] = (
            turn_sign * abs(profile["forward_turn_target_yaw_error_rad"]))
        profile["entry_steering_raw"] = (
            turn_sign * abs(profile["entry_steering_raw"]))
        profile["straighten_steering_raw"] = (
            -turn_sign * abs(profile["straighten_steering_raw"]))
        profile["exit_steering_raw"] = (
            -turn_sign * abs(profile["exit_steering_raw"]))
        profile["exit_straighten_steering_raw"] = (
            turn_sign * abs(profile["exit_straighten_steering_raw"]))
        self.slot_id = slot_id
        self.parking_side = resolved_side
        self.turn_sign = turn_sign
        self.profile = profile
        initial_state = STATE_WAIT_BLUE if armed_only else STATE_APPROACH
        start_reason = "armed_waiting_for_blue" if armed_only else "started"
        if not self._transition(initial_state, now, start_reason):
            return False
        return True

    def reset(self):
        self.state = STATE_IDLE
        self.reason = "reset"
        self.state_reason = "reset"
        self.last_command_reason = "reset"
        self.transition_seq += 1
        self.slot_id = None
        self.parking_side = None
        self.turn_sign = None
        self.profile = None
        self.state_entered_at = None
        self._condition_name = None
        self._condition_count = 0
        self._last_valid_observation = None
        self._last_valid_observation_at = None

    def status(self, now=None, command=None, observation=None):
        """Return the stable status contract for logs, UI and field tuning."""
        if now is None:
            now = time.time()
        status = {
            "schema": "parking_status_v1",
            "state": self.state,
            "phase_index": STATE_INDEX.get(self.state, -1),
            "required_visual_source": STATE_REQUIRED_SOURCES.get(self.state),
            "state_age_s": float(self._state_age(now)),
            "state_reason": self.state_reason,
            "last_command_reason": self.last_command_reason,
            "reason": self.last_command_reason,
            "transition_seq": int(self.transition_seq),
            "slot_id": self.slot_id,
            "parking_side": self.parking_side,
            "turn_side": self.parking_side,
            "turn_sign": self.turn_sign,
            "holding": bool(self.holding),
            "active": bool(self.active),
            "done": bool(self.done),
            "fault": bool(self.fault),
            "calibration_complete": bool(
                self.config.calibration_complete),
            "experimental_motion": bool(self.experimental_motion),
            "experimental_speed_limit_raw": int(
                self.experimental_speed_limit_raw),
            "experimental_steering_limit_raw": int(
                self.experimental_steering_limit_raw),
            "next_states": list(ALLOWED_TRANSITIONS.get(self.state, ())),
        }
        if command is not None:
            status.update({
                "speed_raw": int(command.get("speed_raw", 0)),
                "steering_raw": int(command.get("steering_raw", 0)),
                "safe_stop": bool(command.get("safe_stop", True)),
                "command_reason": command.get(
                    "reason", self.last_command_reason),
            })
        if isinstance(observation, dict):
            status.update({
                "observation_valid": observation.get("valid"),
                "observation_confidence": observation.get("confidence"),
                "observation_source_confidence": observation.get(
                    "source_confidence"),
                "observation_slot": observation.get("slot_id"),
                "observation_slot_consistent": observation.get(
                    "slot_consistent"),
                "observation_source_status": observation.get("source_status"),
                "parking_armed": observation.get("parking_armed"),
                "slot_selection_ready": observation.get(
                    "slot_selection_ready"),
                "selected_slot_id": observation.get("selected_slot_id"),
                "observation_parking_side": observation.get(
                    "parking_side", observation.get("turn_side")),
                "parking_trigger_visible": observation.get(
                    "parking_trigger_visible"),
                "blue_trigger_visible": observation.get(
                    "blue_trigger_visible"),
                "slot_obstacle_detected": observation.get(
                    "slot_obstacle_detected"),
                "parking_phase": observation.get("parking_phase"),
            })
        return status

    def step(self, observation, lidar_data=None, now=None):
        if now is None:
            now = time.time()

        if self.state == STATE_IDLE:
            return self._stop("not_started")
        if self.state == STATE_DONE:
            return self._stop("done", done=True)
        if self.state == STATE_FAULT:
            return self._stop(self.reason)

        timeout_reason = self._state_timeout_reason(now)
        if timeout_reason is not None:
            self._fault(timeout_reason, now)
            return self._stop(timeout_reason)

        safety_reason = self._safety_reason(observation, lidar_data)
        if safety_reason is not None:
            if safety_reason in ("emergency_stop", "obstacle_detected"):
                self._fault(safety_reason, now)
            return self._stop(safety_reason)

        target_reason = self._mission_target_reason(observation)
        if target_reason is not None:
            if target_reason in (
                    "selected_slot_obstructed",
                    "selected_slot_mismatch",
                    "parking_side_mismatch"):
                self._fault(target_reason, now)
            return self._stop(target_reason)

        valid, reason = self._valid_observation(observation, now)
        if not valid:
            held = self._visual_dropout_command(observation, reason, now)
            if held is not None:
                return held
            return self._stop(reason)

        self._last_valid_observation = dict(observation)
        self._last_valid_observation_at = float(now)

        handler = self._state_handlers.get(self.state)
        if handler is None:
            self._fault("unknown_state", now)
            return self._stop(self.reason)
        command = handler(observation, now)
        if (isinstance(command, dict) and command.get("safe_stop") and
                command.get("reason") in VISUAL_HOLD_HANDLER_REASONS):
            held = self._visual_dropout_command(
                observation, command.get("reason"), now)
            if held is not None:
                return held
        return command

    def _mission_target_reason(self, observation):
        """Keep the latched target valid for the entire manoeuvre.

        Legacy direct-controller tests and replay profiles do not carry the
        reform fields; they retain the old API.  A reform observation that
        has ``parking_armed`` must keep its selected target concrete and
        clear, otherwise a changing target selector could silently redirect
        the vehicle to a different bay during reverse motion.
        """
        if not isinstance(observation, dict):
            return None
        if "parking_armed" not in observation:
            return None
        if observation.get("parking_armed") is not True:
            return "parking_not_armed"
        if observation.get("slot_selection_ready") is not True:
            return "slot_selection_not_ready"
        if observation.get("selected_slot_id") != self.slot_id:
            return "selected_slot_mismatch"
        if observation.get("slot_obstacle_detected") is True:
            return "selected_slot_obstructed"
        if ("parking_side" in observation or "turn_side" in observation):
            observed_side = normalise_parking_side(
                observation.get("parking_side", observation.get("turn_side")))
            if observed_side is None:
                return "parking_side_missing"
            if observed_side != self.parking_side:
                return "parking_side_mismatch"
        return None

    def _step_wait_blue(self, observation, now):
        """Hold after a P sign until the selected blue trigger is visible.

        This state owns no motion.  ``main.py`` may keep the vehicle on its
        pre-parking lane policy while the state is still idle; once a target
        has been selected it starts this state and the controller becomes the
        sole command owner.  A selected bay cannot be changed here: a blocked
        or mismatched latched target is a stop/fault condition.
        """
        if observation.get("parking_armed") is not True:
            return self._stop("parking_not_armed")
        if observation.get("slot_selection_ready") is not True:
            return self._stop("slot_selection_not_ready")
        selected_slot = observation.get("selected_slot_id")
        if selected_slot != self.slot_id:
            return self._stop("selected_slot_mismatch")
        if observation.get("slot_obstacle_detected") is True:
            self._fault("selected_slot_obstructed", now)
            return self._stop(self.reason)
        if observation.get("parking_trigger_visible") is not True:
            return self._stop("waiting_for_blue_trigger")
        if not self._transition(STATE_APPROACH, now, "blue_trigger_visible"):
            return self._stop(self.reason)
        return self._step_approach(observation, now)

    def _visual_dropout_command(self, observation, reason, now):
        """Bridge a short front-vision dropout without bypassing safety.

        The bridge is deliberately limited to the two forward setup states.
        It reuses the last valid front pose only for a bounded interval and
        only when the current wrapper still has the expected slot/frame
        identity.  Current lidar safety has already been checked by ``step``.
        A persistent dropout returns ``None`` and the normal safe-stop path
        remains in force.
        """
        if self.state not in VISUAL_HOLD_STATES:
            return None
        if (reason not in VISUAL_HOLD_VALIDATION_REASONS and
                reason not in VISUAL_HOLD_HANDLER_REASONS):
            return None
        if not isinstance(self.profile, dict):
            return None
        hold_s = self.profile.get("visual_dropout_hold_s", 0.0)
        if not _is_finite(hold_s) or float(hold_s) <= 0.0:
            return None
        if (self._last_valid_observation is None or
                self._last_valid_observation_at is None):
            return None
        if isinstance(observation, dict):
            frame_id = observation.get(
                "frame_id", observation.get("coordinate_frame"))
            if frame_id not in (None, "base_link"):
                return None
            if observation.get("slot_id") not in (None, self.slot_id):
                return None
            if observation.get("slot_consistent") is False:
                return None
            if observation.get("emergency_stop") is True:
                return None
            if observation.get("front_clear") is False:
                return None
        age = max(0.0, float(now) - float(self._last_valid_observation_at))
        if age > float(hold_s):
            return None

        last = self._last_valid_observation
        if self.state == STATE_FORWARD_RIGHT_TURN:
            steering = self._forward_turn_steering(last)
            hold_reason = "forward_right_turn_visual_hold:%s" % reason
        else:
            steering = self._forward_straighten_steering(last)
            hold_reason = "forward_straighten_visual_hold:%s" % reason
        return self._move(
            self.profile["forward_speed_raw"], steering, hold_reason)

    def _step_approach(self, observation, now):
        missing = self._missing_clearance(observation, "front_clear")
        if missing:
            return self._stop(missing)
        if observation.get("slot_visible") is not True:
            return self._stop("slot_not_visible")
        # The approach leg is deliberately a fixed-heading, fixed-speed leg:
        # the selected front marker is the distance trigger, while pose
        # feedback is required before the first steering command.  Requiring
        # yaw/lateral feedback here would deadlock the vehicle before it can
        # reach the calibrated turn point when the front lane pose is not
        # visible yet.  Once the trigger is stable, _feedback_missing below
        # still prevents entering the turn open-loop.
        distance = self._number(observation, "turn_distance_m")
        if distance is None or distance < 0.0:
            return self._stop("turn_distance_missing")
        if self._stable(
                "turn_start",
                distance <= self.profile["turn_start_distance_m"]):
            if self._number(observation, "vehicle_yaw_error_rad") is None:
                return self._stop("yaw_error_missing")
            missing = self._feedback_missing(
                observation,
                "lateral_error_m",
                "forward_turn_lateral_gain_raw_per_m",
            )
            if missing:
                return self._stop(missing)
            if not self._transition(
                    STATE_FORWARD_RIGHT_TURN, now, "turn_start"):
                return self._stop(self.reason)
            return self._move(
                self.profile["forward_speed_raw"],
                self.profile["entry_steering_raw"],
                "forward_right_turn")
        return self._move(
            self.profile["forward_speed_raw"], 0, "approach")

    def _step_forward_right_turn(self, observation, now):
        missing = self._missing_clearance(observation, "front_clear")
        if missing:
            return self._stop(missing)
        if observation.get("slot_visible") is not True:
            return self._stop("slot_not_visible")
        missing = self._feedback_missing(
            observation,
            "lateral_error_m",
            "forward_turn_lateral_gain_raw_per_m",
        )
        if missing:
            return self._stop(missing)
        yaw_error = self._number(observation, "vehicle_yaw_error_rad")
        if yaw_error is None:
            return self._stop("yaw_error_missing")
        target = self.profile["forward_turn_target_yaw_error_rad"]
        reached = yaw_error <= target if target < 0.0 else yaw_error >= target
        if self._stable("forward_turn_reached", reached):
            if not isinstance(observation.get("reverse_ready"), bool):
                return self._stop("reverse_ready_missing")
            missing = self._feedback_missing(
                observation,
                "lateral_error_m",
                "forward_straighten_lateral_gain_raw_per_m",
            )
            if missing:
                return self._stop(missing)
            if not self._transition(
                    STATE_FORWARD_STRAIGHTEN, now, "forward_turn_reached"):
                return self._stop(self.reason)
            return self._move(
                self.profile["forward_speed_raw"],
                self.profile["straighten_steering_raw"],
                "forward_straighten")
        return self._move(
            self.profile["forward_speed_raw"],
            self._forward_turn_steering(observation),
            "forward_right_turn")

    def _step_forward_straighten(self, observation, now):
        missing = self._missing_clearance(observation, "front_clear")
        if missing:
            return self._stop(missing)
        if observation.get("slot_visible") is not True:
            return self._stop("slot_not_visible")
        missing = self._feedback_missing(
            observation,
            "lateral_error_m",
            "forward_straighten_lateral_gain_raw_per_m",
        )
        if missing:
            return self._stop(missing)
        yaw_error = self._number(observation, "vehicle_yaw_error_rad")
        if yaw_error is None:
            return self._stop("yaw_error_missing")
        if not isinstance(observation.get("reverse_ready"), bool):
            return self._stop("reverse_ready_missing")
        ready = (
            observation["reverse_ready"] and
            abs(yaw_error) <= self.profile["setup_yaw_tolerance_rad"])
        if self._stable("reverse_ready", ready):
            if not self._transition(STATE_SETTLE, now, "reverse_ready"):
                return self._stop(self.reason)
            return self._stop("reverse_settle")
        return self._move(
            self.profile["forward_speed_raw"],
            self._forward_straighten_steering(observation),
            "forward_straighten")

    def _step_settle(self, observation, now):
        for field in ("front_clear", "rear_clear"):
            missing = self._missing_clearance(observation, field)
            if missing:
                return self._stop(missing)
        if self._state_age(now) < self.profile["settle_time_s"]:
            return self._stop("settling")
        if not self._transition(
                STATE_REVERSE_STEER_IN, now, "reverse_start"):
            return self._stop(self.reason)
        return self._step_reverse_steer_in(observation, now)

    def _reverse_inputs(self, observation):
        missing = self._missing_clearance(observation, "rear_clear")
        if missing:
            return None, missing
        if observation.get("rear_visible") is not True:
            return None, "rear_not_visible"
        distance = self._number(observation, "rear_distance_m")
        lateral = self._number(observation, "lateral_error_m")
        yaw_error = self._number(observation, "vehicle_yaw_error_rad")
        if distance is None or distance < 0.0:
            return None, "rear_distance_missing"
        if lateral is None:
            return None, "lateral_error_missing"
        if yaw_error is None:
            return None, "yaw_error_missing"
        return (distance, lateral, yaw_error), None

    def _step_reverse_steer_in(self, observation, now):
        values, reason = self._reverse_inputs(observation)
        if values is None:
            return self._stop(reason)
        distance, _, _ = values
        if self._stable(
                "reverse_steer_switch",
                distance <= self.profile["reverse_steer_switch_distance_m"]):
            if not self._transition(
                    STATE_REVERSE_STRAIGHTEN, now, "reverse_steer_switch"):
                return self._stop(self.reason)
            return self._move(
                self.profile["reverse_speed_raw"],
                self.profile["straighten_steering_raw"],
                "reverse_straighten")
        return self._move(
            self.profile["reverse_speed_raw"],
            self._reverse_entry_steering(observation),
            "reverse_steer_in")

    def _step_reverse_straighten(self, observation, now):
        values, reason = self._reverse_inputs(observation)
        if values is None:
            return self._stop(reason)
        _, _, yaw_error = values
        if self._stable(
                "reverse_yaw_aligned",
                abs(yaw_error) <= self.profile["reverse_yaw_tolerance_rad"]):
            if not self._transition(
                    STATE_REVERSE_ALIGN, now, "reverse_yaw_aligned"):
                return self._stop(self.reason)
            return self._reverse_align_command(observation)
        return self._move(
            self.profile["reverse_speed_raw"],
            self._reverse_straighten_steering(observation),
            "reverse_straighten")

    def _step_reverse_align(self, observation, now):
        values, reason = self._reverse_inputs(observation)
        if values is None:
            return self._stop(reason)
        distance, lateral, yaw_error = values
        parked = (
            distance <= self.profile["park_stop_distance_m"] and
            abs(lateral) <= self.profile["park_lateral_tolerance_m"] and
            abs(yaw_error) <= self.profile["park_yaw_tolerance_rad"])
        if distance <= self.profile["park_stop_distance_m"]:
            if not parked:
                self._fault("park_boundary_reached_not_aligned", now)
                return self._stop(self.reason)
            if self._stable("parked", parked):
                if not self._transition(STATE_PARKED, now, "parked"):
                    return self._stop(self.reason)
                return self._stop("parked")
            return self._stop("parking_settling")
        return self._reverse_align_command(observation)

    def _reverse_align_command(self, observation):
        lateral = self._number(observation, "lateral_error_m")
        yaw_error = self._number(observation, "vehicle_yaw_error_rad")
        if lateral is None or yaw_error is None:
            return self._stop("align_error_missing")
        steering = (
            self.profile["steering_lateral_gain_raw_per_m"] * lateral +
            self.profile["steering_yaw_gain_raw_per_rad"] * yaw_error)
        return self._move(
            self.profile["reverse_speed_raw"],
            steering,
            "reverse_align")

    def _forward_turn_steering(self, observation):
        lateral = self._number(observation, "lateral_error_m")
        yaw_error = self._number(observation, "vehicle_yaw_error_rad")
        if yaw_error is None:
            return self.profile["entry_steering_raw"]
        target = self.profile["forward_turn_target_yaw_error_rad"]
        yaw_gain = self.profile["forward_turn_yaw_gain_raw_per_rad"]
        lateral_gain = self.profile["forward_turn_lateral_gain_raw_per_m"]
        return (
            self.profile["entry_steering_raw"] +
            lateral_gain * (lateral if lateral is not None else 0.0) +
            yaw_gain * (target - yaw_error))

    def _forward_straighten_steering(self, observation):
        lateral = self._number(observation, "lateral_error_m")
        yaw_error = self._number(observation, "vehicle_yaw_error_rad")
        if yaw_error is None:
            return self.profile["straighten_steering_raw"]
        yaw_gain = self.profile["forward_straighten_yaw_gain_raw_per_rad"]
        lateral_gain = self.profile[
            "forward_straighten_lateral_gain_raw_per_m"]
        return (
            self.profile["straighten_steering_raw"] +
            lateral_gain * (lateral if lateral is not None else 0.0) -
            yaw_gain * yaw_error)

    def _reverse_entry_steering(self, observation):
        lateral = self._number(observation, "lateral_error_m")
        yaw_error = self._number(observation, "vehicle_yaw_error_rad")
        if lateral is None or yaw_error is None:
            return self.profile["entry_steering_raw"]
        return (
            self.profile["entry_steering_raw"] +
            self.profile["reverse_entry_lateral_gain_raw_per_m"] * lateral +
            self.profile["reverse_entry_yaw_gain_raw_per_rad"] * yaw_error)

    def _reverse_straighten_steering(self, observation):
        lateral = self._number(observation, "lateral_error_m")
        yaw_error = self._number(observation, "vehicle_yaw_error_rad")
        if lateral is None or yaw_error is None:
            return self.profile["straighten_steering_raw"]
        return (
            self.profile["straighten_steering_raw"] +
            self.profile["reverse_straighten_lateral_gain_raw_per_m"] * lateral +
            self.profile["reverse_straighten_yaw_gain_raw_per_rad"] * yaw_error)

    def _step_parked(self, observation, now):
        if self._state_age(now) < self.profile["parked_hold_s"]:
            return self._stop("parked_hold")
        if not self.profile["auto_exit"]:
            return self._stop("parked_waiting_exit")

        # The field route first moves forward until the rear camera confirms
        # that the rear axle has cleared the slot boundary.  This keeps the
        # following mirrored turn tied to the measured current pose instead of a
        # timer.  A zero threshold is retained only for older replay profiles
        # that model the historical direct exit path.
        if self.profile.get("exit_forward_clearance_distance_m", 0.0) > 0.0:
            for field in ("front_clear", "rear_clear"):
                missing = self._missing_clearance(observation, field)
                if missing:
                    return self._stop(missing)
            if not self._transition(
                    STATE_EXIT_FORWARD_CLEARANCE, now, "exit_clearance_start"):
                return self._stop(self.reason)
            return self._step_exit_forward_clearance(observation, now)

        missing = self._missing_clearance(observation, "front_clear")
        if missing:
            return self._stop(missing)
        missing = self._missing_clearance(observation, "exit_clear")
        if missing:
            return self._stop(missing)
        if not self._transition(STATE_EXIT_TURN, now, "exit_start"):
            return self._stop(self.reason)
        return self._step_exit_turn(observation, now)

    def _step_exit_forward_clearance(self, observation, now):
        """Drive straight until the rear camera confirms the bay is cleared."""
        for field in ("front_clear", "rear_clear"):
            missing = self._missing_clearance(observation, field)
            if missing:
                return self._stop(missing)
        values, reason = self._reverse_inputs(observation)
        if values is None:
            return self._stop(reason)
        distance, lateral, yaw_error = values
        if (abs(lateral) > self.profile["exit_forward_lateral_tolerance_m"] or
                abs(yaw_error) > self.profile["exit_forward_yaw_tolerance_rad"]):
            return self._stop("exit_forward_pose_not_safe")

        target = self.profile["exit_forward_clearance_distance_m"]
        reached = distance >= target
        if self._stable("exit_clearance_reached", reached):
            if not self._transition(STATE_EXIT_TURN, now, "exit_clearance_reached"):
                return self._stop(self.reason)
            return self._step_exit_turn(observation, now)
        if reached:
            return self._stop("exit_clearance_settling")
        return self._move(
            self.profile["exit_speed_raw"],
            self.profile["exit_forward_steering_raw"],
            "exit_forward_clearance")

    def _exit_inputs(self, observation):
        for field in ("front_clear", "exit_clear"):
            missing = self._missing_clearance(observation, field)
            if missing:
                return None, missing
        if observation.get("exit_visible") is not True:
            return None, "exit_not_visible"
        distance = self._number(observation, "exit_distance_m")
        yaw_error = self._number(observation, "exit_yaw_error_rad")
        if distance is None or distance < 0.0:
            return None, "exit_distance_missing"
        if yaw_error is None:
            return None, "exit_yaw_error_missing"
        return (distance, yaw_error), None

    def _step_exit_turn(self, observation, now):
        missing = self._feedback_missing(
            observation,
            "exit_lateral_error_m",
            "exit_turn_lateral_gain_raw_per_m",
        )
        if missing:
            return self._stop(missing)
        values, reason = self._exit_inputs(observation)
        if values is None:
            return self._stop(reason)
        distance, yaw_error = values
        if self.profile.get("exit_turn_require_yaw_alignment", False):
            straighten_reached = (
                distance <= self.profile["exit_straighten_distance_m"] and
                abs(yaw_error) <= self.profile["exit_yaw_tolerance_rad"])
        else:
            straighten_reached = (
                distance <= self.profile["exit_straighten_distance_m"])
        if self._stable(
                "exit_straighten",
                straighten_reached):
            if self._number(observation, "exit_lateral_error_m") is None:
                return self._stop("exit_lateral_error_missing")
            if not isinstance(observation.get("exit_complete"), bool):
                return self._stop("exit_complete_invalid")
            if not self._transition(
                    STATE_EXIT_STRAIGHTEN, now, "exit_straighten"):
                return self._stop(self.reason)
            return self._exit_straighten_command(observation)
        return self._move(
            self.profile["exit_speed_raw"],
            self._exit_turn_steering(observation),
            "exit_turn")

    def _exit_straighten_command(self, observation):
        if self._number(observation, "exit_lateral_error_m") is None:
            return self._stop("exit_lateral_error_missing")
        return self._move(
            self.profile["exit_speed_raw"],
            self._exit_straighten_steering(observation),
            "exit_straighten")

    def _step_exit_straighten(self, observation, now):
        values, reason = self._exit_inputs(observation)
        if values is None:
            return self._stop(reason)
        distance, yaw_error = values
        lateral = self._number(observation, "exit_lateral_error_m")
        if lateral is None:
            return self._stop("exit_lateral_error_missing")
        explicit_complete = observation.get("exit_complete")
        geometric_complete = (
            distance <= self.profile["exit_complete_distance_m"] and
            abs(yaw_error) <= self.profile["exit_yaw_tolerance_rad"] and
            abs(lateral) <= self.profile["exit_lateral_tolerance_m"])
        if explicit_complete is True or self._stable(
                "exit_complete", geometric_complete):
            if not self._transition(STATE_DONE, now, "exit_complete"):
                return self._stop(self.reason)
            return self._stop("exit_complete", done=True)
        if explicit_complete is not False and explicit_complete is not None:
            return self._stop("exit_complete_invalid")
        return self._move(
            self.profile["exit_speed_raw"],
            self._exit_straighten_steering(observation),
            "exit_straighten")

    def _exit_turn_steering(self, observation):
        lateral = self._number(observation, "exit_lateral_error_m")
        yaw_error = self._number(observation, "exit_yaw_error_rad")
        if yaw_error is None:
            return self.profile["exit_steering_raw"]
        lateral_gain = self.profile["exit_turn_lateral_gain_raw_per_m"]
        yaw_gain = self.profile["exit_turn_yaw_gain_raw_per_rad"]
        return (
            self.profile["exit_steering_raw"] +
            lateral_gain * (lateral if lateral is not None else 0.0) +
            yaw_gain * yaw_error)

    def _exit_straighten_steering(self, observation):
        lateral = self._number(observation, "exit_lateral_error_m")
        yaw_error = self._number(observation, "exit_yaw_error_rad")
        if yaw_error is None:
            return self.profile["exit_straighten_steering_raw"]
        lateral_gain = self.profile[
            "exit_straighten_lateral_gain_raw_per_m"]
        yaw_gain = self.profile["exit_straighten_yaw_gain_raw_per_rad"]
        return (
            self.profile["exit_straighten_steering_raw"] +
            lateral_gain * (lateral if lateral is not None else 0.0) +
            yaw_gain * yaw_error)

    def _valid_observation(self, observation, now):
        if not isinstance(observation, dict):
            return False, "observation_missing"
        if observation.get("schema") != "parking_observation_v1":
            return False, "observation_schema_invalid"
        if observation.get("valid") is not True:
            return False, "observation_invalid"
        frame_id = observation.get("frame_id", observation.get("coordinate_frame"))
        if frame_id != "base_link":
            return False, "observation_frame_invalid"
        observation_slot = observation.get("slot_id")
        if not isinstance(observation_slot, string_types) or not observation_slot:
            return False, "observation_slot_missing"
        if observation_slot != self.slot_id:
            return False, "observation_slot_mismatch"
        if observation.get("slot_consistent") is not True:
            return False, "observation_slot_inconsistent"
        confidence = self._number(observation, "confidence")
        if confidence is None:
            return False, "confidence_missing"
        if confidence < 0.0 or confidence > 1.0:
            return False, "confidence_invalid"
        required_source = STATE_REQUIRED_SOURCES.get(self.state)
        source_confidence = observation.get("source_confidence")
        if required_source is not None:
            if not isinstance(source_confidence, dict):
                return False, "source_confidence_missing"
            confidence = self._number(source_confidence, required_source)
            if confidence is None:
                return False, "source_confidence_missing"
            if confidence < 0.0 or confidence > 1.0:
                return False, "source_confidence_invalid"
            if confidence < self.profile["min_confidence"]:
                return False, "source_confidence_low"
            # ``source_confidence`` is not a freshness guarantee.  Require
            # the observation builder's per-source status as well, so a
            # producer cannot accidentally keep a stale/invalid metric alive
            # merely by copying its last numeric errors into a fresh wrapper.
            source_statuses = observation.get("source_status")
            if not isinstance(source_statuses, dict):
                return False, "source_status_missing"
            required_status = source_statuses.get(required_source)
            if not isinstance(required_status, dict):
                return False, "source_status_missing"
            if (required_status.get("present") is not True or
                    required_status.get("fresh") is not True):
                return False, "source_status_stale"
            if required_status.get("valid") is not True:
                return False, "source_status_invalid"
        elif confidence < self.profile["min_confidence"]:
            return False, "confidence_low"
        stamp = self._number(observation, "stamp")
        if stamp is None:
            return False, "observation_stamp_missing"
        age = float(now) - stamp
        if age > self.profile["observation_timeout_s"]:
            return False, "observation_stale"
        if age < -self.profile["observation_timeout_s"]:
            return False, "observation_from_future"
        return True, None

    def _safety_reason(self, observation, lidar_data):
        if isinstance(observation, dict) and observation.get("emergency_stop") is True:
            return "emergency_stop"
        if isinstance(lidar_data, dict) and lidar_data.get("emergency_stop") is True:
            return "emergency_stop"
        if not isinstance(lidar_data, dict):
            return "lidar_missing" if self.config.require_lidar_clear else None
        if lidar_data.get("schema") != "parking_lidar_v1":
            return "lidar_schema_invalid"
        lidar_frame = lidar_data.get(
            "frame_id", lidar_data.get("coordinate_frame"))
        if lidar_frame != "base_link":
            return "lidar_frame_invalid"
        if not isinstance(lidar_data.get("valid"), bool):
            return "lidar_valid_missing"
        if self.config.require_lidar_clear and lidar_data.get("valid") is not True:
            return "lidar_invalid"
        if not isinstance(lidar_data.get("obstacle_detected"), bool):
            return "lidar_obstacle_status_missing"
        if lidar_data.get("obstacle_detected") is True:
            return "obstacle_detected"
        return None

    @staticmethod
    def _number(data, key):
        if not isinstance(data, dict):
            return None
        value = data.get(key)
        if isinstance(value, bool) or not _is_finite(value):
            return None
        return float(value)

    def _feedback_missing(self, observation, error_field, gain_field):
        """Require an error only when its feedback path is enabled.

        This lets the nominal fixed-angle route remain backward compatible
        while making an enabled visual feedback term fail safe if its source
        disappears.  A missing zero-gain term is intentionally irrelevant.
        """
        gain = float(self.profile.get(gain_field, 0.0))
        if abs(gain) <= 1.0e-12:
            return None
        if self._number(observation, error_field) is None:
            return error_field + "_missing"
        return None

    @staticmethod
    def _missing_clearance(data, key):
        if not isinstance(data, dict) or data.get(key) is not True:
            return key + "_not_clear"
        return None

    def _stable(self, name, condition):
        if name != self._condition_name:
            self._condition_name = name
            self._condition_count = 0
        if condition:
            self._condition_count += 1
        else:
            self._condition_count = 0
        return self._condition_count >= int(self.profile["condition_debounce_frames"])

    def _state_age(self, now):
        if self.state_entered_at is None:
            return 0.0
        return max(0.0, float(now) - float(self.state_entered_at))

    def _state_timeout_reason(self, now):
        if not isinstance(self.profile, dict):
            return None
        field = STATE_TIMEOUT_FIELDS.get(self.state)
        if field is None:
            return None
        timeout = self.profile.get(field)
        if timeout is None or not _is_finite(timeout) or float(timeout) <= 0.0:
            return None
        if self._state_age(now) > float(timeout):
            return "state_timeout_%s" % self.state
        return None

    def _transition(self, state, now, reason):
        if state not in ALLOWED_TRANSITIONS.get(self.state, ()):
            self._fault(
                "invalid_transition_%s_to_%s" % (self.state, state), now)
            return False
        self.state = state
        self.reason = reason
        self.state_reason = reason
        self.last_command_reason = reason
        self.transition_seq += 1
        self.state_entered_at = float(now)
        self._condition_name = None
        self._condition_count = 0
        return True

    def _fault(self, reason, now):
        self.state = STATE_FAULT
        self.reason = reason
        self.state_reason = reason
        self.last_command_reason = reason
        self.transition_seq += 1
        self.state_entered_at = float(now)
        self._condition_name = None
        self._condition_count = 0

    def _move(self, speed_raw, steering_raw, reason):
        if self.state == STATE_FAULT:
            return self._stop(self.reason)
        self.reason = reason
        self.last_command_reason = reason
        speed_limit = self.profile["max_speed_raw"]
        steering_limit = self.profile["max_steering_raw"]
        if self.experimental_motion:
            speed_limit = min(
                speed_limit, self.experimental_speed_limit_raw)
            steering_limit = min(
                steering_limit, self.experimental_steering_limit_raw)
        speed = int(round(max(-speed_limit, min(
            speed_limit, float(speed_raw)))))
        steering = int(round(max(-steering_limit, min(
            steering_limit, float(steering_raw)))))
        return {
            "speed_raw": speed,
            "steering_raw": steering,
            "state": self.state,
            "reason": reason,
            "slot_id": self.slot_id,
            "parking_side": self.parking_side,
            "turn_sign": self.turn_sign,
            "safe_stop": False,
            "done": self.state == STATE_DONE,
        }

    def _stop(self, reason, done=False):
        self.reason = reason
        self.last_command_reason = reason
        return {
            "speed_raw": 0,
            "steering_raw": 0,
            "state": self.state,
            "reason": reason,
            "slot_id": self.slot_id,
            "parking_side": self.parking_side,
            "turn_sign": self.turn_sign,
            "safe_stop": True,
            "done": bool(done or self.state == STATE_DONE),
        }
