#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Independent, safety-gated phased LEFT/RIGHT intersection turn trial.

Ackermann fields in this project carry legacy raw-like values:
signed speed raw and integer steering raw in [-22, 22].

Each direction has independent entry-straight, turn-arc and exit-align
parameters.  Timed phases shape the trial path, while only consecutive lane
observations may declare the trial DONE.
"""

from __future__ import print_function

import json
import math
import threading


IDLE = "IDLE"
ENTRY_STRAIGHT = "ENTRY_STRAIGHT"
TURN_ARC = "TURN_ARC"
EXIT_ALIGN = "EXIT_ALIGN"
REACQUIRE = "REACQUIRE"
DONE = "DONE"
FAULT = "FAULT"

LEFT = "LEFT"
RIGHT = "RIGHT"
STOP = "STOP"
RESET = "RESET"

HARD_MAX_ABS_SPEED = 100
HARD_MAX_ABS_STEERING = 22


def _finite(value):
    try:
        value = float(value)
        return not (math.isnan(value) or math.isinf(value))
    except (TypeError, ValueError):
        return False


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


def _integer_parameter(value, name):
    numeric = float(value)
    if not _finite(numeric) or numeric != round(numeric):
        raise ValueError("%s must be a finite integer" % name)
    return int(round(numeric))


def default_config():
    """Conservative raw defaults; motion is disabled by default."""
    return {
        "left_entry_speed": 10,
        "left_entry_time_s": 0.60,
        "left_turn_speed": 10,
        "left_turn_steering": 8,
        "left_arc_time_s": 1.60,
        "left_exit_speed": 10,
        "left_exit_steering": 0,
        "left_exit_min_time_s": 0.30,
        "right_entry_speed": 10,
        "right_entry_time_s": 0.10,
        "right_turn_speed": 10,
        "right_turn_steering": -12,
        "right_arc_time_s": 1.00,
        "right_exit_speed": 10,
        "right_exit_steering": 0,
        "right_exit_min_time_s": 0.30,
        "min_turn_hold_s": 0.80,
        "turn_progress_heading": 0.40,
        "turn_progress_frames": 2,
        "reacquire_confidence": 0.55,
        "reacquire_candidate_frames": 2,
        "reacquire_frames": 4,
        "reacquire_max_miss_frames": 2,
        "reacquire_heading_tolerance": 0.35,
        "reacquire_lateral_tolerance": 0.25,
        "reacquire_min_points": 4,
        "reacquire_min_forward_span": 0.30,
        "lane_timeout_s": 0.30,
        "turn_timeout_s": 5.0,
        "enable_motion": False,
        "max_abs_speed": 20,
        "max_abs_steering": 12,
        "publish_hz": 20.0,
    }


def validate_config(config):
    integer_names = (
        "left_entry_speed", "left_exit_speed", "left_exit_steering",
        "right_entry_speed", "right_exit_speed", "right_exit_steering",
        "left_turn_speed", "left_turn_steering",
        "right_turn_speed", "right_turn_steering",
        "turn_progress_frames", "reacquire_candidate_frames",
        "reacquire_frames", "reacquire_max_miss_frames",
        "reacquire_min_points", "max_abs_speed", "max_abs_steering",
    )
    for name in integer_names:
        config[name] = _integer_parameter(config[name], name)

    float_names = (
        "left_entry_time_s", "left_arc_time_s",
        "left_exit_min_time_s",
        "right_entry_time_s", "right_arc_time_s",
        "right_exit_min_time_s",
        "min_turn_hold_s", "turn_progress_heading",
        "reacquire_confidence", "reacquire_heading_tolerance",
        "reacquire_lateral_tolerance", "reacquire_min_forward_span",
        "lane_timeout_s", "turn_timeout_s", "publish_hz",
    )
    for name in float_names:
        config[name] = float(config[name])
        if not _finite(config[name]):
            raise ValueError("%s must be finite" % name)

    if not isinstance(config["enable_motion"], bool):
        raise ValueError("enable_motion must be a boolean")
    speed_names = (
        "left_entry_speed", "left_turn_speed", "left_exit_speed",
        "right_entry_speed", "right_turn_speed", "right_exit_speed",
    )
    if any(config[name] < 0 for name in speed_names):
        raise ValueError("phase speeds must be non-negative")
    if config["left_turn_steering"] <= 0:
        raise ValueError("left_turn_steering must be positive (LEFT)")
    if config["right_turn_steering"] >= 0:
        raise ValueError("right_turn_steering must be negative (RIGHT)")
    if config["max_abs_speed"] < 0 or config["max_abs_speed"] > HARD_MAX_ABS_SPEED:
        raise ValueError("max_abs_speed must be in [0, 100]")
    if (config["max_abs_steering"] < 0 or
            config["max_abs_steering"] > HARD_MAX_ABS_STEERING):
        raise ValueError("max_abs_steering must be in [0, 22]")
    for name in speed_names:
        if abs(config[name]) > config["max_abs_speed"]:
            raise ValueError("%s exceeds max_abs_speed" % name)
    steering_names = (
        "left_turn_steering", "left_exit_steering",
        "right_turn_steering", "right_exit_steering")
    for name in steering_names:
        if abs(config[name]) > config["max_abs_steering"]:
            raise ValueError("%s exceeds max_abs_steering" % name)
    for name in ("left_entry_time_s", "right_entry_time_s",
                 "left_exit_min_time_s", "right_exit_min_time_s"):
        if config[name] < 0.0:
            raise ValueError("%s must be non-negative" % name)
    for name in ("left_arc_time_s", "right_arc_time_s"):
        if config[name] <= 0.0:
            raise ValueError("%s must be positive" % name)
    if config["min_turn_hold_s"] < 0.0:
        raise ValueError("min_turn_hold_s must be non-negative")
    if config["turn_progress_heading"] <= 0.0:
        raise ValueError("turn_progress_heading must be positive")
    if config["turn_progress_frames"] < 1:
        raise ValueError("turn_progress_frames must be >= 1")
    if not 0.0 <= config["reacquire_confidence"] <= 1.0:
        raise ValueError("reacquire_confidence must be in [0, 1]")
    if config["reacquire_candidate_frames"] < 1:
        raise ValueError("reacquire_candidate_frames must be >= 1")
    if config["reacquire_frames"] < config["reacquire_candidate_frames"]:
        raise ValueError("reacquire_frames must be >= candidate frames")
    if config["reacquire_max_miss_frames"] < 0:
        raise ValueError("reacquire_max_miss_frames must be non-negative")
    if config["reacquire_min_points"] < 2:
        raise ValueError("reacquire_min_points must be >= 2")
    if config["reacquire_heading_tolerance"] <= 0.0:
        raise ValueError("heading tolerance must be positive")
    if config["reacquire_lateral_tolerance"] < 0.0:
        raise ValueError("lateral tolerance must be non-negative")
    if config["reacquire_min_forward_span"] <= 0.0:
        raise ValueError("minimum forward span must be positive")
    if config["lane_timeout_s"] <= 0.0:
        raise ValueError("lane timeout must be positive")
    if config["turn_timeout_s"] <= config["min_turn_hold_s"]:
        raise ValueError("turn timeout must exceed minimum hold")
    for prefix in ("left", "right"):
        planned_phase_time = (
            config[prefix + "_entry_time_s"] +
            config[prefix + "_arc_time_s"] +
            config[prefix + "_exit_min_time_s"])
        if config["turn_timeout_s"] <= planned_phase_time:
            raise ValueError(
                "turn_timeout_s must exceed %s planned phase time" % prefix)
    if config["publish_hz"] <= 0.0:
        raise ValueError("publish_hz must be positive")
    return config


def empty_lane_assessment(reason):
    return {
        "valid": False,
        "candidate": False,
        "fatal": False,
        "reason": reason,
        "point_count": 0,
        "forward_point_count": 0,
        "heading_error": None,
        "lateral_error": None,
    }


def is_lane_reacquired(points, confidence, config):
    """Check whether a base_link path plausibly describes an exit lane.

    points are (x_forward_m, y_left_m), ordered near-to-far.
    """
    result = empty_lane_assessment("no_path")
    if points is None:
        return result
    try:
        points = list(points)
    except (TypeError, ValueError):
        result["fatal"] = True
        result["reason"] = "path_malformed"
        return result

    result["point_count"] = len(points)
    if not _finite(confidence):
        result["fatal"] = True
        result["reason"] = "confidence_nonfinite"
        return result
    confidence = float(confidence)
    if confidence < 0.0 or confidence > 1.0:
        result["fatal"] = True
        result["reason"] = "confidence_out_of_range"
        return result

    clean = []
    for point in points:
        if not isinstance(point, (tuple, list)) or len(point) != 2:
            result["fatal"] = True
            result["reason"] = "path_point_malformed"
            return result
        if not _finite(point[0]) or not _finite(point[1]):
            result["fatal"] = True
            result["reason"] = "path_point_nonfinite"
            return result
        clean.append((float(point[0]), float(point[1])))

    forward = [point for point in clean if point[0] > 0.0]
    result["forward_point_count"] = len(forward)
    if len(forward) < config["reacquire_min_points"]:
        result["reason"] = "insufficient_forward_points"
        return result

    for index in range(1, len(forward)):
        if forward[index][0] < forward[index - 1][0]:
            result["fatal"] = True
            result["reason"] = "path_not_near_to_far"
            return result

    forward_span = forward[-1][0] - forward[0][0]
    if forward_span < config["reacquire_min_forward_span"]:
        result["reason"] = "forward_span_too_short"
        return result

    dx = forward[-1][0] - forward[0][0]
    dy = forward[-1][1] - forward[0][1]
    heading_error = math.atan2(dy, dx)
    near_count = min(3, len(forward))
    lateral_error = sum(point[1] for point in forward[:near_count]) / float(near_count)
    result["heading_error"] = heading_error
    result["lateral_error"] = lateral_error

    if confidence < config["reacquire_confidence"]:
        result["reason"] = "confidence_low"
        return result
    if abs(heading_error) > config["reacquire_heading_tolerance"]:
        result["reason"] = "heading_out_of_tolerance"
        return result
    if abs(lateral_error) > config["reacquire_lateral_tolerance"]:
        result["reason"] = "lateral_out_of_tolerance"
        return result

    result["valid"] = True
    result["candidate"] = True
    result["reason"] = "exit_lane_candidate"
    return result


class IntersectionTurnFSM(object):
    """Pure state machine; ROS I/O stays outside for offline testing."""

    def __init__(self, config):
        self.config = validate_config(dict(config))
        self.state = IDLE
        self.action = None
        self.reason = "waiting_for_command"
        self.turn_started_at = None
        self.phase_started_at = None
        self.last_path_sequence = None
        self.turn_progress_armed = False
        self.turn_progress_source = "none"
        self.turn_progress_count = 0
        self.reacquire_count = 0
        self.reacquire_miss_count = 0
        self.last_assessment = empty_lane_assessment("no_lane_observation")

    def _reset_trial(self, state, reason):
        self.state = state
        self.action = None
        self.reason = reason
        self.turn_started_at = None
        self.phase_started_at = None
        self.last_path_sequence = None
        self.turn_progress_armed = False
        self.turn_progress_source = "none"
        self.turn_progress_count = 0
        self.reacquire_count = 0
        self.reacquire_miss_count = 0
        self.last_assessment = empty_lane_assessment("no_lane_observation")

    def handle_command(self, command, now):
        command = str(command).strip().upper()
        if command == STOP:
            self._reset_trial(IDLE, "stopped_by_operator")
            return True
        if command == RESET:
            if self.state in (DONE, FAULT):
                self._reset_trial(IDLE, "reset_by_operator")
                return True
            self.reason = "reset_ignored_in_%s" % self.state.lower()
            return False
        if command in (LEFT, RIGHT):
            if self.state != IDLE:
                return False
            if not _finite(now):
                self._reset_trial(FAULT, "command_time_nonfinite")
                return False
            self.state = ENTRY_STRAIGHT
            self.action = command
            self.reason = "entry_straight_started"
            self.turn_started_at = float(now)
            self.phase_started_at = float(now)
            self.last_path_sequence = None
            self.turn_progress_armed = False
            self.turn_progress_source = "none"
            self.turn_progress_count = 0
            self.reacquire_count = 0
            self.reacquire_miss_count = 0
            return True
        self._reset_trial(FAULT, "invalid_control_command")
        return False

    def force_fault(self, reason):
        self.state = FAULT
        self.reason = reason
        self.turn_progress_armed = False
        self.turn_progress_source = "none"
        self.turn_progress_count = 0
        self.reacquire_count = 0
        self.reacquire_miss_count = 0

    def requested_command(self):
        if self.state not in (
                ENTRY_STRAIGHT, TURN_ARC, EXIT_ALIGN, REACQUIRE):
            return 0, 0
        if self.action not in (LEFT, RIGHT):
            self.force_fault("active_state_without_action")
            return 0, 0
        prefix = "left" if self.action == LEFT else "right"
        if self.state == ENTRY_STRAIGHT:
            speed = self.config[prefix + "_entry_speed"]
            steering = 0
        elif self.state == TURN_ARC:
            speed = self.config[prefix + "_turn_speed"]
            steering = self.config[prefix + "_turn_steering"]
        else:
            speed = self.config[prefix + "_exit_speed"]
            steering = self.config[prefix + "_exit_steering"]
        speed = int(_clamp(speed, -self.config["max_abs_speed"],
                           self.config["max_abs_speed"]))
        steering = int(_clamp(steering, -self.config["max_abs_steering"],
                              self.config["max_abs_steering"]))
        return speed, steering

    def output_command(self):
        speed, steering = self.requested_command()
        if not self.config["enable_motion"]:
            return 0, 0
        if self.state not in (
                ENTRY_STRAIGHT, TURN_ARC, EXIT_ALIGN, REACQUIRE):
            return 0, 0
        if not _finite(speed) or not _finite(steering):
            self.force_fault("control_nonfinite")
            return 0, 0
        return speed, steering

    def _observation_error(self, observation):
        if observation is None:
            return "lane_observation_missing"
        if observation.get("frame_id") != "base_link":
            return "lane_frame_invalid"
        for name in ("path_age", "confidence_age"):
            if not _finite(observation.get(name)):
                return "%s_nonfinite" % name
            if float(observation[name]) < 0.0:
                return "%s_negative" % name
            if float(observation[name]) > self.config["lane_timeout_s"]:
                return "%s_timeout" % name
        if not _finite(observation.get("confidence")):
            return "lane_confidence_nonfinite"
        if not isinstance(observation.get("path_sequence"), int):
            return "lane_sequence_invalid"
        return None

    def step(self, now, observation):
        active_states = (ENTRY_STRAIGHT, TURN_ARC, EXIT_ALIGN, REACQUIRE)
        if self.state not in active_states:
            return self.output_command()
        if (not _finite(now) or not _finite(self.turn_started_at) or
                not _finite(self.phase_started_at)):
            self.force_fault("turn_time_nonfinite")
            return 0, 0
        elapsed = float(now) - self.turn_started_at
        phase_elapsed = float(now) - self.phase_started_at
        if elapsed < 0.0 or phase_elapsed < 0.0:
            self.force_fault("turn_time_reversed")
            return 0, 0
        if elapsed > self.config["turn_timeout_s"]:
            self.force_fault("turn_timeout")
            return 0, 0

        error = self._observation_error(observation)
        if error is not None:
            self.force_fault(error)
            return 0, 0
        if self.action not in (LEFT, RIGHT):
            self.force_fault("active_state_without_action")
            return 0, 0

        sequence = observation["path_sequence"]
        new_observation = sequence != self.last_path_sequence
        if new_observation:
            self.last_path_sequence = sequence
            assessment = is_lane_reacquired(
                observation.get("points"), observation.get("confidence"),
                self.config)
            assessment["lane_confidence"] = float(observation["confidence"])
            self.last_assessment = assessment
            if assessment["fatal"]:
                self.force_fault(assessment["reason"])
                return 0, 0
        else:
            assessment = self.last_assessment

        heading_error = assessment.get("heading_error")
        if (self.state == TURN_ARC and new_observation and
                not self.turn_progress_armed):
            if heading_error is not None and _finite(heading_error):
                if abs(heading_error) >= self.config["turn_progress_heading"]:
                    self.turn_progress_count += 1
                    if (self.turn_progress_count >=
                            self.config["turn_progress_frames"]):
                        self.turn_progress_armed = True
                        self.turn_progress_source = "visual_heading"
                else:
                    self.turn_progress_count = 0
            else:
                self.turn_progress_count = 0

        prefix = "left" if self.action == LEFT else "right"

        if self.state == ENTRY_STRAIGHT:
            self.reacquire_count = 0
            self.reacquire_miss_count = 0
            if phase_elapsed < self.config[prefix + "_entry_time_s"]:
                self.reason = "entry_straight"
                return self.output_command()
            self.state = TURN_ARC
            self.phase_started_at = float(now)
            self.reason = "turn_arc_started"
            return self.output_command()

        if self.state == TURN_ARC:
            self.reacquire_count = 0
            self.reacquire_miss_count = 0
            if phase_elapsed < self.config[prefix + "_arc_time_s"]:
                if self.turn_progress_armed:
                    self.reason = "turn_arc_visual_progress"
                else:
                    self.reason = "turn_arc"
                return self.output_command()

            # Completing the calibrated arc only arms exit-lane recognition.
            # It never declares the turn successful; DONE still requires
            # consecutive metric lane observations below.
            self.state = EXIT_ALIGN
            self.phase_started_at = float(now)
            if not self.turn_progress_armed:
                self.turn_progress_armed = True
                self.turn_progress_source = "arc_phase_complete"
            self.reason = "exit_align_started"
            return self.output_command()

        if self.state == EXIT_ALIGN:
            if (elapsed < self.config["min_turn_hold_s"] or
                    phase_elapsed <
                    self.config[prefix + "_exit_min_time_s"]):
                self.reacquire_count = 0
                self.reacquire_miss_count = 0
                self.reason = "exit_align_minimum"
                return self.output_command()

        if not new_observation:
            return self.output_command()

        if assessment["candidate"]:
            self.reacquire_count += 1
            self.reacquire_miss_count = 0
            self.reason = "exit_lane_candidate"
            if (self.state == EXIT_ALIGN and
                    self.reacquire_count >=
                    self.config["reacquire_candidate_frames"]):
                self.state = REACQUIRE
                self.reacquire_miss_count = 0
                self.reason = "confirming_exit_lane"
            if (self.state == REACQUIRE and
                    self.reacquire_count >= self.config["reacquire_frames"]):
                self.state = DONE
                self.reason = "exit_lane_reacquired"
                self.turn_progress_armed = False
                self.turn_progress_source = "none"
                self.turn_progress_count = 0
                self.reacquire_count = 0
                self.reacquire_miss_count = 0
                return 0, 0
        else:
            if self.state == REACQUIRE:
                self.reacquire_miss_count += 1
                if (self.reacquire_miss_count <=
                        self.config["reacquire_max_miss_frames"]):
                    self.reason = "reacquire_candidate_miss:%s" % (
                        assessment["reason"])
                    return self.output_command()
                self.state = EXIT_ALIGN
                self.phase_started_at = float(now)
                self.reacquire_count = 0
                self.reacquire_miss_count = 0
                self.reason = "reacquire_miss_limit:%s" % assessment["reason"]
                return self.output_command()
            self.reacquire_count = 0
            self.reacquire_miss_count = 0
            self.reason = assessment["reason"]
        return self.output_command()


class IntersectionTurnNode(object):
    def __init__(self):
        import rospy
        from ackermann_msgs.msg import AckermannDriveStamped
        from nav_msgs.msg import Path
        from std_msgs.msg import Float32, String

        self.rospy = rospy
        self.AckermannDriveStamped = AckermannDriveStamped
        self.String = String
        config = default_config()
        for name, default_value in list(config.items()):
            config[name] = rospy.get_param("~" + name, default_value)
        self.fsm = IntersectionTurnFSM(config)
        self.lock = threading.RLock()
        self.path_points = None
        self.path_frame = None
        self.path_received_at = None
        self.path_sequence = 0
        self.confidence = None
        self.confidence_received_at = None

        self.command_sub = rospy.Subscriber(
            "/trial/intersection_turn_cmd", String,
            self.command_callback, queue_size=1)
        self.path_sub = rospy.Subscriber(
            "/vision/lane_path", Path, self.path_callback, queue_size=1)
        self.confidence_sub = rospy.Subscriber(
            "/vision/lane_confidence", Float32,
            self.confidence_callback, queue_size=1)
        self.command_pub = rospy.Publisher(
            "/ackermann_cmd", AckermannDriveStamped, queue_size=1)
        self.status_pub = rospy.Publisher(
            "/trial/intersection_turn_status", String, queue_size=1)
        rospy.on_shutdown(self.shutdown)

    def path_callback(self, message):
        points = []
        frame_id = str(message.header.frame_id)
        frame_valid = frame_id == "base_link"
        for pose in message.poses:
            pose_frame = str(pose.header.frame_id)
            if pose_frame and pose_frame != "base_link":
                frame_valid = False
            points.append((pose.pose.position.x, pose.pose.position.y))
        with self.lock:
            self.path_points = points
            self.path_frame = frame_id if frame_valid else "invalid"
            self.path_received_at = self.rospy.get_time()
            self.path_sequence += 1

    def confidence_callback(self, message):
        with self.lock:
            self.confidence = message.data
            self.confidence_received_at = self.rospy.get_time()

    def command_callback(self, message):
        now = self.rospy.get_time()
        with self.lock:
            accepted = self.fsm.handle_command(message.data, now)
            if not accepted:
                self.rospy.logwarn_throttle(
                    1.0, "intersection_turn_trial: %s" % self.fsm.reason)
            # A command callback never starts motion directly.  The 20 Hz loop
            # must first validate fresh lane/path data.  STOP is therefore also
            # immediate, while LEFT/RIGHT cannot emit one unchecked frame.
            self.publish_control(0, 0)
            self.publish_status(now, 0, 0)

    def snapshot(self, now):
        with self.lock:
            path_age = (float("inf") if self.path_received_at is None else
                        now - self.path_received_at)
            confidence_age = (
                float("inf") if self.confidence_received_at is None else
                now - self.confidence_received_at)
            return {
                "points": self.path_points,
                "frame_id": self.path_frame,
                "path_age": path_age,
                "confidence": self.confidence,
                "confidence_age": confidence_age,
                "path_sequence": self.path_sequence,
            }

    def publish_control(self, speed, steering):
        message = self.AckermannDriveStamped()
        message.header.stamp = self.rospy.Time.now()
        message.header.frame_id = "base_link"
        message.drive.speed = float(speed)
        message.drive.steering_angle = float(steering)
        self.command_pub.publish(message)

    def publish_status(self, now, output_speed, output_steering):
        requested_speed, requested_steering = self.fsm.requested_command()
        assessment = self.fsm.last_assessment
        turn_elapsed = (None if self.fsm.turn_started_at is None else
                        max(0.0, now - self.fsm.turn_started_at))
        phase_elapsed = (None if self.fsm.phase_started_at is None else
                         max(0.0, now - self.fsm.phase_started_at))
        payload = {
            "state": self.fsm.state,
            "action": self.fsm.action or "NONE",
            "turn_elapsed": turn_elapsed,
            "phase_elapsed": phase_elapsed,
            "motion_enabled": self.fsm.config["enable_motion"],
            "lane_valid": bool(assessment.get("valid", False)),
            "lane_confidence": assessment.get("lane_confidence"),
            "lane_point_count": assessment.get("point_count", 0),
            "forward_point_count": assessment.get("forward_point_count", 0),
            "turn_progress_armed": self.fsm.turn_progress_armed,
            "turn_progress_source": self.fsm.turn_progress_source,
            "turn_progress_count": self.fsm.turn_progress_count,
            "reacquire_candidate": bool(
                assessment.get("candidate", False)),
            "reacquire_count": self.fsm.reacquire_count,
            "reacquire_miss_count": self.fsm.reacquire_miss_count,
            "heading_error": assessment.get("heading_error"),
            "lateral_error": assessment.get("lateral_error"),
            "requested_speed": requested_speed,
            "requested_steering": requested_steering,
            "output_speed": output_speed,
            "output_steering": output_steering,
            "reason": self.fsm.reason,
            "timestamp": now,
        }
        self.status_pub.publish(
            self.String(data=json.dumps(payload, sort_keys=True)))

    def shutdown(self):
        try:
            self.publish_control(0, 0)
        except Exception:
            pass

    def run(self):
        rate = self.rospy.Rate(self.fsm.config["publish_hz"])
        while not self.rospy.is_shutdown():
            now = self.rospy.get_time()
            observation = self.snapshot(now)
            with self.lock:
                try:
                    speed, steering = self.fsm.step(now, observation)
                except Exception as error:
                    self.fsm.force_fault(
                        "internal_exception:%s" % type(error).__name__)
                    speed, steering = 0, 0
                    self.rospy.logerr_throttle(
                        1.0, "intersection_turn_trial exception: %s" % error)
                self.publish_control(speed, steering)
                self.publish_status(now, speed, steering)
                self.rospy.loginfo_throttle(
                    1.0,
                    "intersection turn state=%s action=%s output=(%d,%d) "
                    "progress=%s candidate=%s count=%d reason=%s" %
                    (self.fsm.state, self.fsm.action or "NONE",
                     speed, steering,
                     self.fsm.turn_progress_source,
                     self.fsm.last_assessment.get("candidate", False),
                     self.fsm.reacquire_count, self.fsm.reason))
            rate.sleep()


def main():
    import rospy
    rospy.init_node("intersection_turn_trial", anonymous=False)
    try:
        node = IntersectionTurnNode()
    except Exception as error:
        rospy.logfatal("intersection_turn_trial configuration failed: %s", error)
        return
    node.run()


if __name__ == "__main__":
    main()
