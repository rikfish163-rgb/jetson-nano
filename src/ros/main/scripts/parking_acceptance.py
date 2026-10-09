#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Offline acceptance report for one reverse-parking JSONL session.

This tool never imports ROS and never publishes a command.  It checks the
evidence written by ``parking_session_logger.py`` against the same state and
profile contracts used by the runtime controller.  A successful report is
necessary evidence for a run, but it is not a substitute for an operator's
physical safety inspection.  By default the report describes software and
command-path acceptance.  ``--require-physical-feedback`` additionally gates
the result on valid odometry showing both forward and reverse motion.
"""

from __future__ import print_function

import argparse
import hashlib
import json
import math
import os
import sys


try:
    string_types = (basestring,)
except NameError:
    string_types = (str,)


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from parking_controller import (  # noqa: E402
    STATE_APPROACH,
    STATE_DONE,
    STATE_EXIT_STRAIGHTEN,
    STATE_EXIT_TURN,
    STATE_EXIT_FORWARD_CLEARANCE,
    STATE_FAULT,
    STATE_FORWARD_RIGHT_TURN,
    STATE_FORWARD_STRAIGHTEN,
    STATE_IDLE,
    STATE_PARKED,
    STATE_REVERSE_ALIGN,
    STATE_REVERSE_STEER_IN,
    STATE_REVERSE_STRAIGHTEN,
    STATE_REQUIRED_SOURCES,
    STATE_SETTLE,
    STATE_TIMEOUT_FIELDS,
    ParkingConfig,
)


EXPECTED_STATES = (
    STATE_APPROACH,
    STATE_FORWARD_RIGHT_TURN,
    STATE_FORWARD_STRAIGHTEN,
    STATE_SETTLE,
    STATE_REVERSE_STEER_IN,
    STATE_REVERSE_STRAIGHTEN,
    STATE_REVERSE_ALIGN,
    STATE_PARKED,
    STATE_EXIT_TURN,
    STATE_EXIT_STRAIGHTEN,
    STATE_DONE,
)

EXPECTED_STATES_WITH_FORWARD_CLEARANCE = (
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
)

MOTION_STATES = (
    STATE_APPROACH,
    STATE_FORWARD_RIGHT_TURN,
    STATE_FORWARD_STRAIGHTEN,
    STATE_REVERSE_STEER_IN,
    STATE_REVERSE_STRAIGHTEN,
    STATE_REVERSE_ALIGN,
    STATE_EXIT_FORWARD_CLEARANCE,
    STATE_EXIT_TURN,
    STATE_EXIT_STRAIGHTEN,
)

FORWARD_MOTION_STATES = (
    STATE_APPROACH,
    STATE_FORWARD_RIGHT_TURN,
    STATE_FORWARD_STRAIGHTEN,
    STATE_EXIT_FORWARD_CLEARANCE,
    STATE_EXIT_TURN,
    STATE_EXIT_STRAIGHTEN,
)

REVERSE_MOTION_STATES = (
    STATE_REVERSE_STEER_IN,
    STATE_REVERSE_STRAIGHTEN,
    STATE_REVERSE_ALIGN,
)

ZERO_COMMAND_STATES = (
    STATE_IDLE,
    STATE_SETTLE,
    STATE_PARKED,
    STATE_FAULT,
    STATE_DONE,
)


def _finite(value):
    if isinstance(value, bool):
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(value) or math.isinf(value):
        return None
    return value


def _payload(record):
    if not isinstance(record, dict):
        return None
    data = record.get("data")
    if not isinstance(data, dict):
        return None
    if "payload" in data:
        value = data.get("payload")
        return value if isinstance(value, dict) else None
    # String topics are wrapped as {topic, payload}; typed actuator and
    # odometry callbacks are written directly as {topic, ...}.  Both are
    # produced by parking_session_logger and must be accepted here.
    return data


def load_records(path):
    records = []
    with open(path, "r") as stream:
        for line_number, line in enumerate(stream, 1):
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except (TypeError, ValueError) as exc:
                raise ValueError("invalid JSONL at line %d: %s" %
                                 (line_number, exc))
            if not isinstance(record, dict):
                continue
            stamp = _finite(record.get("wall_time"))
            if stamp is None:
                continue
            record["wall_time"] = stamp
            records.append(record)
    records.sort(key=lambda item: item["wall_time"])
    return records


def _status_entries(records):
    entries = []
    previous_state = None
    for record in records:
        if record.get("event") != "status":
            continue
        payload = _payload(record)
        if not isinstance(payload, dict):
            continue
        state = payload.get("state")
        if not isinstance(state, string_types) or not state:
            continue
        if state == previous_state:
            continue
        entries.append({
            "wall_time": record["wall_time"],
            "state": state,
            "payload": payload,
        })
        previous_state = state
    return entries


def _logger_start(records):
    for record in records:
        if record.get("event") != "logger_start":
            continue
        data = record.get("data")
        if not isinstance(data, dict):
            continue
        # parking_session_logger writes logger_start metadata directly under
        # data, while some early fixtures nested it under data.payload.  Keep
        # both forms readable so the acceptance tool can validate real logs
        # as well as legacy captures.
        payload = data.get("payload")
        if isinstance(payload, dict):
            return payload
        if "payload" not in data:
            return data
    return None


def _check_state_sequence(entries):
    observed = []
    for entry in entries:
        state = entry["state"]
        if not observed and state == "idle":
            continue
        if state == "idle" and STATE_DONE in observed:
            break
        observed.append(state)
        if state == STATE_FAULT:
            return {
                "passed": False,
                "reason": "fault_state_seen",
                "observed": observed,
                "expected": list(EXPECTED_STATES),
            }
        if state == STATE_DONE:
            break
    expected = (EXPECTED_STATES_WITH_FORWARD_CLEARANCE
                if STATE_EXIT_FORWARD_CLEARANCE in observed
                else EXPECTED_STATES)
    passed = tuple(observed) == expected
    return {
        "passed": bool(passed),
        "reason": "ok" if passed else "state_sequence_mismatch",
        "observed": observed,
        "expected": list(expected),
    }


def _sha256(path):
    if not path or not os.path.isfile(path):
        return None
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        while True:
            block = stream.read(65536)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def _check_config_provenance(records, config_path):
    start = _logger_start(records)
    if start is None:
        return {"passed": False, "reason": "logger_start_missing"}
    recorded_hash = start.get("config_sha256")
    if not isinstance(recorded_hash, string_types) or not recorded_hash:
        return {"passed": False, "reason": "config_hash_missing"}
    result = {
        "passed": True,
        "reason": "hash_recorded",
        "recorded_config_file": start.get("config_file"),
        "recorded_config_sha256": recorded_hash,
    }
    recorded_hashes = start.get("config_hashes")
    if isinstance(recorded_hashes, dict) and recorded_hashes:
        checked_hashes = {}
        hash_errors = []
        for path in sorted(recorded_hashes):
            expected = recorded_hashes.get(path)
            current = _sha256(path)
            checked_hashes[path] = {
                "recorded": expected,
                "current": current,
                "passed": bool(
                    isinstance(expected, string_types) and
                    current is not None and current == expected),
            }
            if not checked_hashes[path]["passed"]:
                hash_errors.append(path)
        result["recorded_config_hashes"] = dict(recorded_hashes)
        result["checked_config_hashes"] = checked_hashes
        if hash_errors:
            result.update({
                "passed": False,
                "reason": "config_hashes_mismatch",
                "config_hash_errors": hash_errors,
            })
            return result
    if config_path:
        current_hash = _sha256(config_path)
        result["checked_config_file"] = config_path
        result["checked_config_sha256"] = current_hash
        if current_hash is None:
            result.update({"passed": False, "reason": "config_file_missing"})
        elif current_hash != recorded_hash:
            result.update({"passed": False, "reason": "config_hash_mismatch"})
        else:
            result["reason"] = "hash_matches"
    return result


def _check_commands(records, config):
    nonzero = 0
    control_count = 0
    limit_violations = []
    terminal_nonzero = []
    recorded_states = set()
    for record in records:
        if record.get("event") != "status":
            continue
        payload = _payload(record)
        if isinstance(payload, dict) and isinstance(payload.get("state"),
                                                    string_types):
            recorded_states.add(payload.get("state"))
    # The forward-clearance segment is a configured extension: old logs from
    # the direct parked->exit route remain valid, while a log that records the
    # new state must prove that it carried a non-zero command there.
    required_motion_states = tuple(
        state for state in MOTION_STATES
        if state != STATE_EXIT_FORWARD_CLEARANCE or
        state in recorded_states)
    motion_state_nonzero = dict(
        (state, False) for state in required_motion_states)
    for record in records:
        if record.get("event") not in ("control", "status"):
            continue
        payload = _payload(record)
        if not isinstance(payload, dict):
            continue
        speed = _finite(payload.get("speed_raw"))
        steering = _finite(payload.get("steering_raw"))
        if speed is None or steering is None:
            continue
        if record.get("event") == "control":
            control_count += 1
            if speed != 0.0 or steering != 0.0:
                nonzero += 1
        elif record.get("event") == "status":
            state = payload.get("state")
            if (state in motion_state_nonzero and
                    (speed != 0.0 or steering != 0.0)):
                motion_state_nonzero[state] = True
            if (state in ZERO_COMMAND_STATES and
                    (speed != 0.0 or steering != 0.0)):
                terminal_nonzero.append({
                    "wall_time": record["wall_time"],
                    "state": state,
                    "speed_raw": speed,
                    "steering_raw": steering,
                })
        if (abs(speed) > float(config.max_speed_raw) or
                abs(steering) > float(config.max_steering_raw)):
            limit_violations.append({
                "wall_time": record["wall_time"],
                "event": record.get("event"),
                "speed_raw": speed,
                "steering_raw": steering,
            })

    calibration_values = []
    for record in records:
        if record.get("event") != "status":
            continue
        payload = _payload(record)
        if isinstance(payload, dict) and isinstance(
                payload.get("calibration_complete"), bool):
            calibration_values.append(payload["calibration_complete"])
    calibration_enabled = bool(calibration_values and any(calibration_values))
    locked_motion = bool(
        calibration_values and any(not value for value in calibration_values) and
        nonzero > 0)
    missing_motion_states = [state for state, seen in
                             motion_state_nonzero.items() if not seen]
    passed = bool(
        control_count > 0 and bool(config.calibration_complete) and
        calibration_enabled and not locked_motion and not limit_violations and
        not missing_motion_states and not terminal_nonzero)
    reason = "ok"
    if control_count == 0:
        reason = "control_records_missing"
    elif not bool(config.calibration_complete):
        reason = "config_calibration_incomplete"
    elif not calibration_enabled:
        reason = "calibration_not_enabled"
    elif locked_motion:
        reason = "nonzero_command_while_locked"
    elif limit_violations:
        reason = "command_limit_exceeded"
    elif missing_motion_states:
        reason = "motion_command_missing_for_state"
    elif terminal_nonzero:
        reason = "nonzero_command_in_zero_state"
    return {
        "passed": passed,
        "reason": reason,
        "control_record_count": control_count,
        "nonzero_control_count": nonzero,
        "calibration_values": calibration_values,
        "limit_violations": limit_violations,
        "terminal_nonzero": terminal_nonzero,
        "motion_state_nonzero": motion_state_nonzero,
        "missing_motion_states": missing_motion_states,
    }


def _check_actuator_commands(records, config):
    """Check that both drive directions reached the Ackermann ROS topic.

    ``/ackermann_cmd`` is still a command observation, not a wheel encoder or
    chassis-motion measurement.  Keeping this check separate from
    ``_check_commands`` makes that evidence boundary explicit.
    """
    record_count = 0
    valid_count = 0
    forward_count = 0
    reverse_count = 0
    invalid_records = []
    limit_violations = []
    for record in records:
        if record.get("event") != "actuator_command":
            continue
        record_count += 1
        payload = _payload(record)
        if not isinstance(payload, dict):
            invalid_records.append({
                "wall_time": record["wall_time"],
                "reason": "payload_missing",
            })
            continue
        speed = _finite(payload.get("speed_raw", payload.get("speed")))
        steering = _finite(payload.get(
            "steering_raw", payload.get("steering_angle")))
        if speed is None or steering is None or payload.get("valid") is False:
            invalid_records.append({
                "wall_time": record["wall_time"],
                "reason": "command_invalid",
            })
            continue
        valid_count += 1
        if speed > 0.0:
            forward_count += 1
        elif speed < 0.0:
            reverse_count += 1
        if (abs(speed) > float(config.max_speed_raw) or
                abs(steering) > float(config.max_steering_raw)):
            limit_violations.append({
                "wall_time": record["wall_time"],
                "speed_raw": speed,
                "steering_raw": steering,
            })

    passed = bool(
        record_count > 0 and valid_count > 0 and forward_count > 0 and
        reverse_count > 0 and not invalid_records and
        not limit_violations)
    reason = "ok"
    if record_count == 0:
        reason = "actuator_records_missing"
    elif invalid_records:
        reason = "actuator_record_invalid"
    elif limit_violations:
        reason = "actuator_command_limit_exceeded"
    elif forward_count == 0 or reverse_count == 0:
        reason = "actuator_direction_missing"
    return {
        "passed": passed,
        "reason": reason,
        "record_count": record_count,
        "valid_count": valid_count,
        "forward_count": forward_count,
        "reverse_count": reverse_count,
        "invalid_records": invalid_records,
        "limit_violations": limit_violations,
        "motion_measurement": False,
    }


def _check_base_controller_status(records, config):
    """Check the local base-controller serial-write status stream."""
    record_count = 0
    valid_count = 0
    forward_count = 0
    reverse_count = 0
    write_failures = []
    invalid_records = []
    for record in records:
        if record.get("event") != "base_controller_status":
            continue
        record_count += 1
        payload = _payload(record)
        if not isinstance(payload, dict):
            invalid_records.append({
                "wall_time": record["wall_time"],
                "reason": "payload_missing",
            })
            continue
        speed = _finite(payload.get("speed_raw"))
        steering = _finite(payload.get("steering_raw"))
        serial_bytes = _finite(payload.get("serial_bytes"))
        if speed is None or steering is None or serial_bytes is None:
            invalid_records.append({
                "wall_time": record["wall_time"],
                "reason": "status_invalid",
            })
            continue
        valid_count += 1
        if speed > 0.0:
            forward_count += 1
        elif speed < 0.0:
            reverse_count += 1
        if ((speed != 0.0 or steering != 0.0) and
                (serial_bytes <= 0.0 or payload.get("serial_open") is False)):
            write_failures.append({
                "wall_time": record["wall_time"],
                "speed_raw": speed,
                "steering_raw": steering,
                "serial_bytes": serial_bytes,
                "serial_open": payload.get("serial_open"),
            })
        if (abs(speed) > float(config.max_speed_raw) or
                abs(steering) > float(config.max_steering_raw)):
            write_failures.append({
                "wall_time": record["wall_time"],
                "reason": "command_limit_exceeded",
                "speed_raw": speed,
                "steering_raw": steering,
            })

    passed = bool(
        record_count > 0 and valid_count > 0 and forward_count > 0 and
        reverse_count > 0 and not invalid_records and not write_failures)
    reason = "ok"
    if record_count == 0:
        reason = "base_controller_status_missing"
    elif invalid_records:
        reason = "base_controller_status_invalid"
    elif write_failures:
        reason = "base_controller_serial_write_failure"
    elif forward_count == 0 or reverse_count == 0:
        reason = "base_controller_direction_missing"
    return {
        "passed": passed,
        "reason": reason,
        "record_count": record_count,
        "valid_count": valid_count,
        "forward_count": forward_count,
        "reverse_count": reverse_count,
        "invalid_records": invalid_records,
        "write_failures": write_failures,
        "motion_measurement": False,
    }


def _summarize_odometry(records):
    """Summarize optional vehicle feedback without making it a fake requirement."""
    entries, invalid_count = _odometry_entries(records)
    frame_ids = sorted(set(
        entry["frame_id"] for entry in entries if entry["frame_id"]))
    child_frame_ids = sorted(set(
        entry["child_frame_id"] for entry in entries
        if entry["child_frame_id"]))
    frame_contract_ok = bool(entries) and all(
        entry["frame_id"] and entry["child_frame_id"] for entry in entries)
    return {
        "available": bool(entries),
        "quality_ok": bool(entries) and invalid_count == 0,
        "record_count": len(entries),
        "invalid_count": invalid_count,
        "reason": "recorded" if entries else "not_configured",
        "motion_measurement": True,
        "frame_ids": frame_ids,
        "child_frame_ids": child_frame_ids,
        "frame_contract_ok": frame_contract_ok,
    }


def _odometry_entries(records):
    entries = []
    invalid_count = 0
    for record in records:
        if record.get("event") != "odometry":
            continue
        payload = _payload(record)
        if not isinstance(payload, dict):
            invalid_count += 1
            continue
        x = _finite(payload.get("position_x_m"))
        y = _finite(payload.get("position_y_m"))
        if (x is None or y is None or
                payload.get("valid") is not True):
            invalid_count += 1
            continue
        entries.append({
            "wall_time": record["wall_time"],
            "position_x_m": x,
            "position_y_m": y,
            "linear_x_mps": _finite(payload.get("linear_x_mps")),
            "frame_id": payload.get("frame_id"),
            "child_frame_id": payload.get("child_frame_id"),
        })
    entries.sort(key=lambda item: item["wall_time"])
    return entries, invalid_count


def _state_at(status_entries, stamp):
    state = None
    for entry in status_entries:
        if entry["wall_time"] > stamp:
            break
        state = entry["state"]
    return state


def _check_physical_feedback(records):
    """Require actual odometry motion when the caller asks for it.

    This is deliberately stronger than seeing an ``/ackermann_cmd`` or a
    serial-write status.  It still proves motion evidence only; the visual
    state/slot checks remain responsible for proving the manoeuvre geometry.
    """
    summary = _summarize_odometry(records)
    result = dict(summary)
    result["passed"] = False
    result["min_motion_speed_mps"] = 0.01
    result["min_path_length_m"] = 0.02
    if not summary["available"]:
        result["reason"] = "odometry_missing"
        result["forward_motion_samples"] = 0
        result["reverse_motion_samples"] = 0
        result["path_length_m"] = 0.0
        return result
    if not summary["quality_ok"]:
        result["reason"] = "odometry_invalid"
        result["forward_motion_samples"] = 0
        result["reverse_motion_samples"] = 0
        result["path_length_m"] = 0.0
        return result

    if not summary["frame_contract_ok"]:
        result["reason"] = "odometry_frame_missing"
        result["forward_motion_samples"] = 0
        result["reverse_motion_samples"] = 0
        result["path_length_m"] = 0.0
        return result

    status_entries = _status_entries(records)
    approach_times = [
        entry["wall_time"] for entry in status_entries
        if entry["state"] == STATE_APPROACH
    ]
    done_times = [
        entry["wall_time"] for entry in status_entries
        if entry["state"] == STATE_DONE
    ]
    if not approach_times or not done_times:
        result["reason"] = "odometry_state_window_missing"
        result["forward_motion_samples"] = 0
        result["reverse_motion_samples"] = 0
        result["path_length_m"] = 0.0
        return result

    window_start = min(approach_times)
    window_end = max(done_times)
    entries, _ = _odometry_entries(records)
    entries = [entry for entry in entries if
               window_start <= entry["wall_time"] <= window_end]
    if not entries:
        result["reason"] = "odometry_motion_window_empty"
        result["forward_motion_samples"] = 0
        result["reverse_motion_samples"] = 0
        result["path_length_m"] = 0.0
        result["run_window"] = {
            "start_wall_time": window_start,
            "end_wall_time": window_end,
        }
        return result

    forward_samples = 0
    reverse_samples = 0
    forward_matching_samples = 0
    reverse_matching_samples = 0
    path_length = 0.0
    previous = None

    for entry in entries:
        speed = entry.get("linear_x_mps")
        if speed is not None:
            if speed >= result["min_motion_speed_mps"]:
                forward_samples += 1
            elif speed <= -result["min_motion_speed_mps"]:
                reverse_samples += 1
            state = _state_at(status_entries, entry["wall_time"])
            if (state in FORWARD_MOTION_STATES and
                    speed >= result["min_motion_speed_mps"]):
                forward_matching_samples += 1
            elif (state in REVERSE_MOTION_STATES and
                  speed <= -result["min_motion_speed_mps"]):
                reverse_matching_samples += 1
        if previous is not None:
            path_length += math.hypot(
                entry["position_x_m"] - previous["position_x_m"],
                entry["position_y_m"] - previous["position_y_m"])
        previous = entry

    result["forward_motion_samples"] = forward_samples
    result["reverse_motion_samples"] = reverse_samples
    result["forward_matching_samples"] = forward_matching_samples
    result["reverse_matching_samples"] = reverse_matching_samples
    result["path_length_m"] = path_length
    result["run_window"] = {
        "start_wall_time": window_start,
        "end_wall_time": window_end,
    }
    result["passed"] = bool(
        forward_matching_samples > 0 and reverse_matching_samples > 0 and
        path_length >= result["min_path_length_m"])
    if result["passed"]:
        result["reason"] = "odometry_motion_recorded"
    elif forward_matching_samples == 0 or reverse_matching_samples == 0:
        result["reason"] = "odometry_state_direction_missing"
    else:
        result["reason"] = "odometry_path_too_short"
    return result


def _check_visual_contract(records, config):
    selected_slot = None
    observed_states = set()
    for record in records:
        if record.get("event") != "status":
            continue
        payload = _payload(record)
        if isinstance(payload, dict):
            observed_states.add(payload.get("state"))
    # ``exit_forward_clearance`` is an optional route extension.  Only states
    # that actually occurred in the log are required to prove a visual
    # source; otherwise a legacy direct-exit log would fail retroactively.
    valid_by_state = dict(
        (state, False) for state in STATE_REQUIRED_SOURCES
        if state in observed_states)
    errors = []
    invalid_samples = []
    for record in records:
        if record.get("event") != "status":
            continue
        payload = _payload(record)
        if not isinstance(payload, dict):
            continue
        state = payload.get("state")
        if state == "idle":
            continue
        if selected_slot is None and isinstance(
                payload.get("slot_id"), string_types):
            selected_slot = payload.get("slot_id")
        observation_slot = payload.get("observation_slot")
        if observation_slot is not None and selected_slot is not None and \
                observation_slot != selected_slot:
            errors.append({
                "state": state,
                "reason": "observation_slot_mismatch",
                "observation_slot": observation_slot,
                "selected_slot": selected_slot,
            })
        if ("observation_slot_consistent" in payload and
                payload.get("observation_slot_consistent") is False):
            errors.append({
                "state": state,
                "reason": "observation_slot_inconsistent",
            })

        required_source = STATE_REQUIRED_SOURCES.get(state)
        if required_source is None:
            continue
        source_name = payload.get("required_visual_source")
        if source_name != required_source:
            errors.append({
                "state": state,
                "reason": "required_source_mismatch",
                "expected": required_source,
                "actual": source_name,
            })
            continue
        source_confidence = payload.get("observation_source_confidence")
        confidence = (source_confidence.get(required_source)
                      if isinstance(source_confidence, dict) else None)
        confidence = _finite(confidence)
        if payload.get("observation_valid") is True and confidence is not None and \
                confidence >= float(config.min_confidence):
            if not isinstance(observation_slot, string_types) or not observation_slot:
                errors.append({
                    "state": state,
                    "reason": "observation_slot_missing",
                })
                continue
            if state in valid_by_state:
                valid_by_state[state] = True
        else:
            invalid_samples.append({
                "state": state,
                "reason": "required_visual_source_invalid",
                "source": required_source,
                "confidence": confidence,
            })

    missing_states = [state for state, valid in valid_by_state.items()
                      if not valid]
    if selected_slot is None:
        errors.append({"reason": "selected_slot_missing"})
    if missing_states:
        errors.append({
            "reason": "required_visual_states_missing",
            "states": missing_states,
        })
    return {
        "passed": not errors,
        "reason": "ok" if not errors else "visual_contract_failure",
        "selected_slot": selected_slot,
        "valid_by_state": valid_by_state,
        "invalid_samples": invalid_samples,
        "errors": errors,
    }


def _check_lidar_contract(records):
    """Require the normalized safety stream for an accepted run."""
    record_count = 0
    valid_count = 0
    errors = []
    obstacle_records = []
    for record in records:
        if record.get("event") != "lidar":
            continue
        record_count += 1
        payload = _payload(record)
        if not isinstance(payload, dict):
            errors.append({
                "wall_time": record["wall_time"],
                "reason": "payload_missing",
            })
            continue
        frame_id = payload.get(
            "frame_id", payload.get("coordinate_frame"))
        if (payload.get("schema") != "parking_lidar_v1" or
                frame_id != "base_link" or
                payload.get("valid") is not True or
                not isinstance(payload.get("obstacle_detected"), bool) or
                not isinstance(payload.get("emergency_stop"), bool)):
            errors.append({
                "wall_time": record["wall_time"],
                "reason": "lidar_contract_invalid",
                "schema": payload.get("schema"),
                "frame_id": frame_id,
                "valid": payload.get("valid"),
            })
            continue
        valid_count += 1
        if (payload.get("obstacle_detected") is True or
                payload.get("emergency_stop") is True):
            obstacle_records.append({
                "wall_time": record["wall_time"],
                "obstacle_detected": payload.get("obstacle_detected"),
                "emergency_stop": payload.get("emergency_stop"),
            })

    passed = bool(record_count > 0 and valid_count > 0 and
                  not errors and not obstacle_records)
    reason = "ok"
    if record_count == 0:
        reason = "lidar_records_missing"
    elif errors:
        reason = "lidar_contract_failure"
    elif obstacle_records:
        reason = "lidar_obstacle_seen"
    return {
        "passed": passed,
        "reason": reason,
        "record_count": record_count,
        "valid_count": valid_count,
        "errors": errors,
        "obstacle_records": obstacle_records,
    }


def _check_timeouts(records, config):
    violations = []
    for record in records:
        if record.get("event") != "status":
            continue
        payload = _payload(record)
        if not isinstance(payload, dict):
            continue
        state = payload.get("state")
        age = _finite(payload.get("state_age_s"))
        if age is None:
            violations.append({
                "state": state,
                "reason": "state_age_missing",
            })
            continue
        if age < 0.0:
            violations.append({
                "state": state,
                "reason": "state_age_negative",
                "state_age_s": age,
            })
            continue
        field = STATE_TIMEOUT_FIELDS.get(state)
        if field is not None and age > float(getattr(config, field)) + 1.0e-6:
            violations.append({
                "state": state,
                "reason": "state_timeout_exceeded",
                "state_age_s": age,
                "timeout_s": float(getattr(config, field)),
            })
    return {
        "passed": not violations,
        "reason": "ok" if not violations else "timeout_evidence_failure",
        "violations": violations,
    }


def evaluate(records, config=None, config_path="", require_physical_feedback=False):
    """Return a deterministic acceptance report without changing any file."""
    config = config if config is not None else ParkingConfig()
    entries = _status_entries(records)
    state_check = _check_state_sequence(entries)
    provenance_check = _check_config_provenance(records, config_path)
    command_check = _check_commands(records, config)
    actuator_check = _check_actuator_commands(records, config)
    base_controller_check = _check_base_controller_status(records, config)
    lidar_check = _check_lidar_contract(records)
    visual_check = _check_visual_contract(records, config)
    timeout_check = _check_timeouts(records, config)
    physical_check = _check_physical_feedback(records)
    checks = {
        "config_provenance": provenance_check,
        "state_sequence": state_check,
        "commands": command_check,
        "actuator_command_path": actuator_check,
        "base_controller_write_path": base_controller_check,
        "lidar_contract": lidar_check,
        "visual_contract": visual_check,
        "state_timeouts": timeout_check,
    }
    software_passed = bool(all(check["passed"] for check in checks.values()))
    if require_physical_feedback:
        checks["physical_feedback"] = physical_check
    return {
        "schema": "parking_acceptance_v1",
        "passed": bool(software_passed and
                        (not require_physical_feedback or
                         physical_check["passed"])),
        "software_passed": software_passed,
        "acceptance_mode": ("physical_closed_loop" if
                             require_physical_feedback else
                             "software_and_command_path"),
        "physical_feedback_required": bool(require_physical_feedback),
        "run_id": records[0].get("run_id") if records else None,
        "state_transitions": [
            {"wall_time": entry["wall_time"], "state": entry["state"]}
            for entry in entries
        ],
        "physical_feedback": physical_check,
        "checks": checks,
    }


def load_config(path):
    values = {}
    if path:
        try:
            import yaml
        except ImportError:
            raise RuntimeError("PyYAML is required when --config is supplied")
        with open(path, "r") as stream:
            values = yaml.safe_load(stream)
        if values is None:
            values = {}
        if not isinstance(values, dict):
            raise ValueError("parking config must be a YAML mapping")
    return ParkingConfig.from_dict(values)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="offline acceptance report for one parking JSONL run")
    parser.add_argument("log_file", help="parking_session_logger JSONL file")
    parser.add_argument("--config", default="",
                        help="parking.yaml used for the recorded run")
    parser.add_argument(
        "--require-physical-feedback", action="store_true",
        help=("gate acceptance on valid odometry showing both forward and "
              "reverse motion"))
    args = parser.parse_args(argv)
    config = load_config(args.config)
    report = evaluate(
        load_records(args.log_file), config, args.config,
        require_physical_feedback=args.require_physical_feedback)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (IOError, OSError, ValueError, RuntimeError) as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        sys.exit(2)
