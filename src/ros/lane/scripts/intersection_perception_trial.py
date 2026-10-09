#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Safety-gated perception trigger for the intersection turn trial.

This node never publishes Ackermann commands.  It latches a confirmed LEFT or
RIGHT traffic sign, confirms a metric blue-marker candidate over consecutive
frames, and then sends exactly one command to intersection_turn_trial.py.
"""

from __future__ import print_function

import json
import math
import threading


LEFT = "LEFT"
RIGHT = "RIGHT"
STOP = "STOP"
RESET = "RESET"
ARM = "ARM"
DISARM = "DISARM"
CLEAR = "CLEAR"

TURN_IDLE = "IDLE"
TURN_ACTIVE_STATES = frozenset(
    ("ENTRY_STRAIGHT", "TURN_ARC", "EXIT_ALIGN", "REACQUIRE"))

WAIT_OPERATOR_ARM = "WAIT_OPERATOR_ARM"
WAIT_SIGN = "WAIT_SIGN"
WAIT_MARKER = "WAIT_MARKER"
COMMAND_SENT = "COMMAND_SENT"
TURN_ACTIVE = "TURN_ACTIVE"
DONE = "DONE"
STOPPED = "STOPPED"
FAULT = "FAULT"
DISABLED = "DISABLED"

SUPPORTED_MARKER_SCHEMAS = frozenset(
    ("blue_marker_candidate_v1", "parking_start_line_v1"))


def _finite_float(value):
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    if math.isnan(number) or math.isinf(number):
        return None
    return number


def _positive_integer(value, name):
    number = _finite_float(value)
    if number is None or number != round(number) or number < 1:
        raise ValueError("%s must be a positive integer" % name)
    return int(round(number))


def default_config():
    return {
        # Two independent locks are required: this launch parameter and ARM.
        "enable_auto_trigger": False,
        "sign_confidence_threshold": 0.60,
        "marker_required_frames": 3,
        "marker_min_distance_m": 0.55,
        "marker_max_distance_m": 0.70,
        "marker_max_abs_lateral_m": 0.25,
        "marker_max_abs_angle_deg": 15.0,
        "marker_min_length_m": 0.20,
        "marker_message_timeout_s": 0.30,
        "turn_status_timeout_s": 0.50,
        "publish_hz": 20.0,
    }


def validate_config(config):
    config = dict(config)
    if not isinstance(config.get("enable_auto_trigger"), bool):
        raise ValueError("enable_auto_trigger must be a boolean")
    config["marker_required_frames"] = _positive_integer(
        config.get("marker_required_frames"), "marker_required_frames")
    float_names = (
        "sign_confidence_threshold",
        "marker_min_distance_m",
        "marker_max_distance_m",
        "marker_max_abs_lateral_m",
        "marker_max_abs_angle_deg",
        "marker_min_length_m",
        "marker_message_timeout_s",
        "turn_status_timeout_s",
        "publish_hz",
    )
    for name in float_names:
        number = _finite_float(config.get(name))
        if number is None:
            raise ValueError("%s must be finite" % name)
        config[name] = number
    if not 0.0 <= config["sign_confidence_threshold"] <= 1.0:
        raise ValueError("sign_confidence_threshold must be in [0, 1]")
    if config["marker_min_distance_m"] < 0.0:
        raise ValueError("marker_min_distance_m must be non-negative")
    if (config["marker_max_distance_m"] <=
            config["marker_min_distance_m"]):
        raise ValueError("marker distance window is invalid")
    for name in (
            "marker_max_abs_lateral_m", "marker_max_abs_angle_deg",
            "marker_min_length_m", "marker_message_timeout_s",
            "turn_status_timeout_s", "publish_hz"):
        if config[name] <= 0.0:
            raise ValueError("%s must be positive" % name)
    return config


def decode_json_object(raw):
    try:
        value = json.loads(raw)
    except (TypeError, ValueError):
        return None
    return value if isinstance(value, dict) else None


def validate_marker_payload(payload, config):
    """Return a normalized metric marker candidate or an error reason."""
    if not isinstance(payload, dict):
        return None, "marker_json_invalid"
    schema = str(payload.get("schema", "")).strip()
    if schema not in SUPPORTED_MARKER_SCHEMAS:
        return None, "marker_schema_invalid"

    if schema == "parking_start_line_v1":
        visible = (payload.get("candidate") is True and
                   payload.get("start_line_visible") is True and
                   payload.get("blue_trigger_visible") is True)
        distance = payload.get("start_line_distance_m")
    else:
        visible = payload.get("candidate") is True
        distance = payload.get("distance_m")
    if not visible:
        return None, "marker_not_visible"

    normalized = {
        "schema": schema,
        "stamp": _finite_float(payload.get("stamp")),
        "distance_m": _finite_float(distance),
        "lateral_m": _finite_float(payload.get("lateral_m")),
        "angle_deg": _finite_float(payload.get("angle_deg")),
        "length_m": _finite_float(payload.get("length_m")),
    }
    for name in ("stamp", "distance_m", "lateral_m", "angle_deg",
                 "length_m"):
        if normalized[name] is None:
            return None, "marker_%s_invalid" % name
    if not (config["marker_min_distance_m"] <= normalized["distance_m"] <=
            config["marker_max_distance_m"]):
        return None, "marker_distance_out_of_window"
    if (abs(normalized["lateral_m"]) >
            config["marker_max_abs_lateral_m"]):
        return None, "marker_lateral_out_of_window"
    if abs(normalized["angle_deg"]) > config["marker_max_abs_angle_deg"]:
        return None, "marker_angle_out_of_window"
    if normalized["length_m"] < config["marker_min_length_m"]:
        return None, "marker_too_short"
    return normalized, "marker_valid"


class IntersectionPerceptionGate(object):
    """Pure state holder used by the ROS wrapper and offline tests."""

    def __init__(self, config):
        self.config = validate_config(config)
        self.pending_action = None
        self.pending_confidence = None
        self.armed = False
        self.dispatched = False
        self.dispatched_action = None
        self.turn_active_seen = False
        self.marker_count = 0
        self.last_marker = None
        self.last_marker_stamp = None
        self.last_marker_received_at = None
        self.turn_status = None
        self.turn_status_received_at = None
        self.state = (WAIT_OPERATOR_ARM if
                      self.config["enable_auto_trigger"] else DISABLED)
        self.reason = ("waiting_for_operator_arm" if
                       self.config["enable_auto_trigger"] else
                       "auto_trigger_disabled")

    def reset(self):
        self.pending_action = None
        self.pending_confidence = None
        self.armed = False
        self.dispatched = False
        self.dispatched_action = None
        self.turn_active_seen = False
        self.marker_count = 0
        self.last_marker = None
        self.last_marker_stamp = None
        self.last_marker_received_at = None
        self.state = (WAIT_OPERATOR_ARM if
                      self.config["enable_auto_trigger"] else DISABLED)
        self.reason = ("waiting_for_operator_arm" if
                       self.config["enable_auto_trigger"] else
                       "auto_trigger_disabled")

    def handle_operator_command(self, command):
        command = str(command).strip().upper()
        if command == STOP:
            self.armed = False
            self.dispatched = False
            self.marker_count = 0
            self.state = STOPPED
            self.reason = "stopped_by_operator"
            return STOP
        if command == RESET:
            if self.dispatched or self.state in (COMMAND_SENT, TURN_ACTIVE):
                self.armed = False
                self.state = STOPPED
                self.reason = "reset_during_turn_forced_stop"
                return STOP
            self.reset()
            return RESET
        if command == CLEAR:
            if self.dispatched:
                self.reason = "clear_ignored_while_turn_active"
                return None
            self.pending_action = None
            self.pending_confidence = None
            self.marker_count = 0
            self.state = WAIT_SIGN if self.armed else WAIT_OPERATOR_ARM
            self.reason = "pending_action_cleared"
            return None
        if command == DISARM:
            self.armed = False
            self.marker_count = 0
            self.state = STOPPED
            self.reason = "disarmed_by_operator"
            return STOP
        if command == ARM:
            if not self.config["enable_auto_trigger"]:
                self.state = DISABLED
                self.reason = "arm_rejected_auto_trigger_disabled"
                return None
            if self.dispatched or self.state in (TURN_ACTIVE, COMMAND_SENT):
                self.reason = "arm_rejected_turn_active"
                return None
            if self.state in (DONE, FAULT, STOPPED):
                self.reason = "arm_rejected_reset_required"
                return None
            self.armed = True
            self.marker_count = 0
            self.last_marker_stamp = None
            self.last_marker_received_at = None
            self.state = WAIT_MARKER if self.pending_action else WAIT_SIGN
            self.reason = "armed_waiting_for_%s" % (
                "marker" if self.pending_action else "sign")
            return None
        self.state = FAULT
        self.armed = False
        self.reason = "invalid_operator_command"
        return STOP

    def ingest_sign(self, payload):
        if not isinstance(payload, dict):
            self.reason = "sign_json_invalid"
            return False
        confidence = _finite_float(payload.get("confidence"))
        label = str(payload.get("label", "")).strip().upper()
        valid = (payload.get("valid") is True and
                 payload.get("confirmed") is True and
                 label in (LEFT, RIGHT) and confidence is not None and
                 confidence >= self.config["sign_confidence_threshold"])
        if not valid:
            return False
        if self.pending_action is None:
            self.pending_action = label
            self.pending_confidence = confidence
            if self.armed:
                self.state = WAIT_MARKER
                self.reason = "pending_%s_waiting_for_marker" % label.lower()
            return True
        if self.pending_action == label:
            self.pending_confidence = confidence
            return True
        self.reason = "different_sign_ignored_pending_%s" % (
            self.pending_action.lower())
        return False

    def ingest_marker(self, payload, received_at):
        received_at = _finite_float(received_at)
        if received_at is None:
            self.marker_count = 0
            self.reason = "marker_receive_time_invalid"
            return False
        if not self.armed or self.pending_action is None or self.dispatched:
            self.marker_count = 0
            return False
        marker, reason = validate_marker_payload(payload, self.config)
        if marker is None:
            self.marker_count = 0
            self.last_marker = None
            self.reason = reason
            return False
        payload_age = received_at - marker["stamp"]
        if (payload_age < -self.config["marker_message_timeout_s"] or
                payload_age > self.config["marker_message_timeout_s"]):
            self.marker_count = 0
            self.last_marker = None
            self.reason = "marker_payload_stale"
            return False
        if (self.last_marker_stamp is not None and
                marker["stamp"] <= self.last_marker_stamp):
            self.reason = "marker_stamp_not_increasing"
            return False
        if (self.last_marker_received_at is not None and
                received_at - self.last_marker_received_at >
                self.config["marker_message_timeout_s"]):
            self.marker_count = 0
        self.last_marker = marker
        self.last_marker_stamp = marker["stamp"]
        self.last_marker_received_at = received_at
        self.marker_count += 1
        self.state = WAIT_MARKER
        self.reason = "marker_confirmation_%d_of_%d" % (
            self.marker_count, self.config["marker_required_frames"])
        return True

    def ingest_turn_status(self, payload, received_at):
        if not isinstance(payload, dict):
            self.reason = "turn_status_json_invalid"
            return False
        state = str(payload.get("state", "")).strip().upper()
        if not state:
            self.reason = "turn_status_state_missing"
            return False
        received_at = _finite_float(received_at)
        if received_at is None:
            self.reason = "turn_status_receive_time_invalid"
            return False
        self.turn_status = payload
        self.turn_status_received_at = received_at
        if self.dispatched and state in TURN_ACTIVE_STATES:
            self.turn_active_seen = True
            self.state = TURN_ACTIVE
            self.reason = "turn_%s" % state.lower()
        elif self.dispatched and state == DONE:
            self.state = DONE
            self.reason = "turn_done"
            self.armed = False
            self.dispatched = False
            self.pending_action = None
            self.pending_confidence = None
            self.marker_count = 0
        elif self.dispatched and state == FAULT:
            self.state = FAULT
            self.reason = "turn_fault:%s" % str(
                payload.get("reason", "unknown"))
            self.armed = False
            self.dispatched = False
            self.marker_count = 0
        elif (self.dispatched and self.turn_active_seen and
              state == TURN_IDLE):
            self.state = STOPPED
            self.reason = "turn_stopped_before_done"
            self.armed = False
            self.dispatched = False
            self.marker_count = 0
        return True

    def evaluate(self, now):
        now = _finite_float(now)
        if now is None:
            self.state = FAULT
            self.armed = False
            self.reason = "evaluation_time_invalid"
            return None
        if not self.config["enable_auto_trigger"]:
            self.state = DISABLED
            self.reason = "auto_trigger_disabled"
            return None
        if self.dispatched:
            return None
        if not self.armed:
            if self.state not in (DONE, FAULT, STOPPED):
                self.state = WAIT_OPERATOR_ARM
                self.reason = "waiting_for_operator_arm"
            return None
        if self.pending_action not in (LEFT, RIGHT):
            self.state = WAIT_SIGN
            self.reason = "waiting_for_confirmed_turn_sign"
            return None
        if self.marker_count < self.config["marker_required_frames"]:
            self.state = WAIT_MARKER
            return None
        if (self.last_marker_received_at is None or
                now - self.last_marker_received_at < 0.0 or
                now - self.last_marker_received_at >
                self.config["marker_message_timeout_s"]):
            self.marker_count = 0
            self.reason = "marker_timeout"
            return None
        if self.turn_status is None or self.turn_status_received_at is None:
            self.reason = "turn_status_missing"
            return None
        turn_status_age = now - self.turn_status_received_at
        if (turn_status_age < 0.0 or
                turn_status_age > self.config["turn_status_timeout_s"]):
            self.reason = "turn_status_timeout"
            return None
        turn_state = str(self.turn_status.get("state", "")).strip().upper()
        if turn_state != TURN_IDLE:
            self.reason = "turn_not_idle:%s" % (turn_state or "unknown")
            return None
        action = self.pending_action
        self.dispatched = True
        self.dispatched_action = action
        self.turn_active_seen = False
        self.state = COMMAND_SENT
        self.reason = "%s_dispatched_once" % action.lower()
        return action

    def status(self, now):
        now = _finite_float(now)
        turn_status_age = None
        marker_age = None
        if now is not None and self.turn_status_received_at is not None:
            turn_status_age = max(0.0, now - self.turn_status_received_at)
        if now is not None and self.last_marker_received_at is not None:
            marker_age = max(0.0, now - self.last_marker_received_at)
        marker = self.last_marker or {}
        return {
            "state": self.state,
            "reason": self.reason,
            "auto_trigger_enabled": self.config["enable_auto_trigger"],
            "armed": bool(self.armed),
            "pending_action": self.pending_action or "NONE",
            "pending_confidence": self.pending_confidence,
            "marker_count": int(self.marker_count),
            "marker_required_frames": int(
                self.config["marker_required_frames"]),
            "marker_distance_m": marker.get("distance_m"),
            "marker_lateral_m": marker.get("lateral_m"),
            "marker_angle_deg": marker.get("angle_deg"),
            "marker_age_s": marker_age,
            "turn_state": (str(self.turn_status.get("state"))
                           if isinstance(self.turn_status, dict) else None),
            "turn_status_age_s": turn_status_age,
            "dispatched": bool(self.dispatched),
            "dispatched_action": self.dispatched_action or "NONE",
        }


class IntersectionPerceptionNode(object):
    def __init__(self):
        import rospy
        from std_msgs.msg import String

        self.rospy = rospy
        self.String = String
        config = default_config()
        for name, default_value in list(config.items()):
            config[name] = rospy.get_param("~" + name, default_value)
        self.gate = IntersectionPerceptionGate(config)
        self.lock = threading.RLock()

        sign_topic = rospy.get_param(
            "~traffic_sign_topic", "/perception/traffic_sign")
        marker_topic = rospy.get_param(
            "~blue_marker_topic", "/perception/blue_marker_candidate")
        turn_status_topic = rospy.get_param(
            "~turn_status_topic", "/trial/intersection_turn_status")
        operator_topic = rospy.get_param(
            "~operator_topic", "/trial/intersection_perception_cmd")
        turn_command_topic = rospy.get_param(
            "~turn_command_topic", "/trial/intersection_turn_cmd")
        status_topic = rospy.get_param(
            "~status_topic", "/trial/intersection_perception_status")

        self.turn_command_pub = rospy.Publisher(
            turn_command_topic, String, queue_size=1)
        self.status_pub = rospy.Publisher(status_topic, String, queue_size=1)
        rospy.Subscriber(
            sign_topic, String, self.sign_callback, queue_size=1)
        rospy.Subscriber(
            marker_topic, String, self.marker_callback, queue_size=1)
        rospy.Subscriber(
            turn_status_topic, String, self.turn_status_callback, queue_size=1)
        rospy.Subscriber(
            operator_topic, String, self.operator_callback, queue_size=1)
        rospy.on_shutdown(self.shutdown)

    def _decode(self, message, source):
        payload = decode_json_object(message.data)
        if payload is None:
            self.rospy.logwarn_throttle(
                1.0, "%s JSON is invalid" % source)
        return payload

    def sign_callback(self, message):
        payload = self._decode(message, "traffic sign")
        with self.lock:
            self.gate.ingest_sign(payload)

    def marker_callback(self, message):
        payload = self._decode(message, "blue marker")
        now = self.rospy.get_time()
        with self.lock:
            self.gate.ingest_marker(payload, now)

    def turn_status_callback(self, message):
        payload = self._decode(message, "intersection turn status")
        now = self.rospy.get_time()
        with self.lock:
            self.gate.ingest_turn_status(payload, now)

    def operator_callback(self, message):
        with self.lock:
            relay = self.gate.handle_operator_command(message.data)
            if relay is not None:
                self.turn_command_pub.publish(self.String(data=relay))

    def shutdown(self):
        try:
            self.turn_command_pub.publish(self.String(data=STOP))
        except Exception:
            pass

    def run(self):
        rate = self.rospy.Rate(self.gate.config["publish_hz"])
        while not self.rospy.is_shutdown():
            now = self.rospy.get_time()
            with self.lock:
                action = self.gate.evaluate(now)
                if action is not None:
                    self.turn_command_pub.publish(self.String(data=action))
                    self.rospy.logwarn(
                        "Perception gate dispatched one %s command", action)
                payload = self.gate.status(now)
                self.status_pub.publish(self.String(data=json.dumps(
                    payload, sort_keys=True, allow_nan=False)))
                self.rospy.loginfo_throttle(
                    1.0,
                    "intersection perception state=%s pending=%s marker=%d/%d "
                    "turn=%s reason=%s" % (
                        payload["state"], payload["pending_action"],
                        payload["marker_count"],
                        payload["marker_required_frames"],
                        payload["turn_state"], payload["reason"]))
            rate.sleep()


def main():
    import rospy
    rospy.init_node("intersection_perception_gate", anonymous=False)
    try:
        node = IntersectionPerceptionNode()
    except Exception as error:
        rospy.logfatal(
            "intersection_perception_gate configuration failed: %s", error)
        return
    node.run()


if __name__ == "__main__":
    main()
