#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Build measured P4/P5 slot profiles from successful parking JSONL runs.

This tool is deliberately offline-only.  It reads the same observation and
status records written by ``parking_session_logger.py`` and proposes the
distance/yaw thresholds that were actually reached at each state transition.
It never imports ROS, publishes a command, or changes ``parking.yaml``.

The output is a YAML fragment containing ``slot_profiles``.  A human should
review that fragment against the field geometry before copying it into the
runtime configuration and setting ``calibration_complete: true``.  Failed
runs may be included for diagnosis, but never contribute to the candidate
profile.
"""

from __future__ import print_function

import argparse
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

from parking_controller import ParkingConfig  # noqa: E402


SUPPORTED_SLOT_IDS = ("P4", "P5")
EXPECTED_STATES = (
    "approach",
    "forward_right_turn",
    "forward_straighten",
    "settle",
    "reverse_steer_in",
    "reverse_straighten",
    "reverse_align",
    "parked",
    "exit_turn",
    "exit_straighten",
    "done",
)


TRANSITION_RULES = (
    # state, output key, observation field, transform, margin family,
    # margin direction.  Most thresholds are conservative when enlarged;
    # exit_complete_distance_m is conservative when reduced because the FSM
    # declares success once distance is <= the threshold.
    ("forward_right_turn", "turn_start_distance_m",
     "turn_distance_m", "distance", "m", "upper"),
    ("forward_straighten", "forward_turn_target_yaw_error_rad",
     "vehicle_yaw_error_rad", "signed", "rad", "none"),
    ("settle", "setup_yaw_tolerance_rad",
     "vehicle_yaw_error_rad", "absolute", "rad", "upper"),
    ("reverse_straighten", "reverse_steer_switch_distance_m",
     "rear_distance_m", "distance", "m", "upper"),
    ("reverse_align", "reverse_yaw_tolerance_rad",
     "vehicle_yaw_error_rad", "absolute", "rad", "upper"),
    ("parked", "park_stop_distance_m",
     "rear_distance_m", "distance", "m", "upper"),
    ("parked", "park_lateral_tolerance_m",
     "lateral_error_m", "absolute", "m", "upper"),
    ("parked", "park_yaw_tolerance_rad",
     "vehicle_yaw_error_rad", "absolute", "rad", "upper"),
    ("exit_straighten", "exit_straighten_distance_m",
     "exit_distance_m", "distance", "m", "upper"),
    ("done", "exit_complete_distance_m",
     "exit_distance_m", "distance", "m", "lower"),
    ("done", "exit_yaw_tolerance_rad",
     "exit_yaw_error_rad", "absolute", "rad", "upper"),
    ("done", "exit_lateral_tolerance_m",
     "exit_lateral_error_m", "absolute", "m", "upper"),
)


def _finite_number(value):
    if isinstance(value, bool):
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(value) or math.isinf(value):
        return None
    return value


def _number(data, key):
    if not isinstance(data, dict):
        return None
    return _finite_number(data.get(key))


def _payload(record):
    if not isinstance(record, dict):
        return None
    data = record.get("data")
    if not isinstance(data, dict):
        return None
    if "payload" in data:
        payload = data.get("payload")
        return payload if isinstance(payload, dict) else None
    # Typed logger callbacks write their fields directly under data; String
    # callbacks wrap the decoded JSON under data.payload.
    return data


def _logger_start_payload(record):
    """Read logger_start from current direct or legacy nested JSONL data."""
    if not isinstance(record, dict) or record.get("event") != "logger_start":
        return None
    data = record.get("data")
    if not isinstance(data, dict):
        return None
    payload = data.get("payload")
    if isinstance(payload, dict):
        return payload
    if "payload" not in data:
        return data
    return None


def load_records(paths):
    records = []
    for path in paths:
        with open(path, "r") as stream:
            for line_number, line in enumerate(stream, 1):
                line = line.strip()
                if not line:
                    continue
                try:
                    record = json.loads(line)
                except (TypeError, ValueError) as exc:
                    raise ValueError(
                        "invalid JSONL at %s:%d: %s" %
                        (path, line_number, exc))
                if not isinstance(record, dict):
                    continue
                stamp = _finite_number(record.get("wall_time"))
                if stamp is None:
                    continue
                record["wall_time"] = stamp
                record["run_id"] = str(record.get("run_id", os.path.basename(path)))
                records.append(record)
    records.sort(key=lambda item: item["wall_time"])
    return records


def _group_runs(records):
    grouped = {}
    for record in records:
        grouped.setdefault(record["run_id"], []).append(record)
    return grouped


def _latest_observation(observations, stamp, max_age_s):
    for record in reversed(observations):
        age = float(stamp) - float(record["wall_time"])
        if age < 0.0:
            continue
        if age <= float(max_age_s):
            return _payload(record)
        break
    return None


def _run_summary(records, observation_age_s):
    observations = []
    transitions = []
    previous_state = None
    saw_fault = False
    calibration_values = []
    slot_id = None
    config_file = None
    config_sha256 = None
    config_hashes = {}

    for record in records:
        if record.get("event") == "logger_start":
            payload = _logger_start_payload(record)
            if isinstance(payload, dict):
                config_file = payload.get("config_file")
                config_sha256 = payload.get("config_sha256")
                if isinstance(payload.get("config_hashes"), dict):
                    config_hashes = dict(payload.get("config_hashes"))
            continue
        payload = _payload(record)
        if payload is None:
            continue
        if record.get("event") == "observation":
            observations.append(record)
            candidate_slot = payload.get("slot_id")
            if isinstance(candidate_slot, string_types) and candidate_slot:
                slot_id = candidate_slot
            continue
        if record.get("event") != "status":
            continue

        state = payload.get("state")
        if not isinstance(state, string_types) or not state:
            continue
        calibration_value = payload.get("calibration_complete")
        if isinstance(calibration_value, bool):
            calibration_values.append(calibration_value)
        if state == "fault":
            saw_fault = True
        if state == previous_state:
            continue
        candidate_slot = payload.get("slot_id")
        if isinstance(candidate_slot, string_types) and candidate_slot:
            slot_id = candidate_slot
        transition = {
            "state": state,
            "from_state": previous_state,
            "wall_time": record["wall_time"],
            "slot_id": candidate_slot or slot_id,
            "observation": _latest_observation(
                observations, record["wall_time"], observation_age_s),
        }
        transitions.append(transition)
        previous_state = state

    states = [item["state"] for item in transitions]
    normalized_states = list(states)
    if normalized_states and normalized_states[0] == "idle":
        normalized_states = normalized_states[1:]
    if "done" in normalized_states:
        normalized_states = normalized_states[:
                                             normalized_states.index("done") + 1]
    sequence_ok = tuple(normalized_states) == EXPECTED_STATES
    slot_supported = slot_id in SUPPORTED_SLOT_IDS
    calibration_ok = bool(
        calibration_values and all(calibration_values))
    return {
        "run_id": records[0]["run_id"] if records else None,
        "slot_id": slot_id,
        "successful": bool(
            sequence_ok and slot_supported and calibration_ok and not saw_fault),
        "saw_fault": bool(saw_fault),
        "state_sequence_ok": bool(sequence_ok),
        "slot_supported": bool(slot_supported),
        "calibration_ok": bool(calibration_ok),
        "success_reason": (
            "ok" if (sequence_ok and slot_supported and calibration_ok and
                      not saw_fault) else
            "fault_seen" if saw_fault else
            "calibration_incomplete" if not calibration_ok else
            "slot_not_supported" if not slot_supported else
            "state_sequence_mismatch"),
        "config_file": config_file,
        "config_sha256": config_sha256,
        "config_hashes": config_hashes,
        "states": normalized_states,
        "transitions": transitions,
    }


def _percentile(values, fraction):
    values = sorted(float(value) for value in values)
    if not values:
        return None
    if len(values) == 1:
        return values[0]
    position = (len(values) - 1) * float(fraction)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return values[lower]
    weight = position - lower
    return values[lower] + (values[upper] - values[lower]) * weight


def _candidate(values, transform, margin_family, margin_m, margin_rad,
               margin_direction):
    if transform == "signed":
        return _percentile(values, 0.5)
    if transform == "absolute":
        values = [abs(value) for value in values]
    value = _percentile(values, 0.8)
    margin = margin_m if margin_family == "m" else margin_rad
    if margin_direction == "lower":
        return max(0.0, value - margin)
    return max(0.0, value + margin)


def build_candidate(records, min_runs=1, margin_m=0.02, margin_rad=0.03,
                    observation_age_s=0.75, allow_failed=False):
    """Return a report and slot profile without modifying any file."""
    if int(min_runs) < 1:
        raise ValueError("min_runs must be >= 1")
    if float(margin_m) < 0.0 or float(margin_rad) < 0.0:
        raise ValueError("margins must be non-negative")

    all_summaries = []
    successful_summaries = []
    diagnostic_summaries = []
    for run_id, run_records in sorted(_group_runs(records).items()):
        summary = _run_summary(run_records, observation_age_s)
        summary["run_id"] = run_id
        all_summaries.append(summary)
        if summary["successful"]:
            successful_summaries.append(summary)
        elif allow_failed:
            diagnostic_summaries.append(summary)

    samples = {}
    sample_runs = {}
    for summary in successful_summaries:
        default_slot = summary.get("slot_id")
        for transition in summary["transitions"]:
            observation = transition.get("observation")
            if not isinstance(observation, dict):
                continue
            slot_id = transition.get("slot_id") or default_slot
            if not isinstance(slot_id, string_types) or not slot_id:
                continue
            for (state, key, field, transform, margin_family,
                 margin_direction) in TRANSITION_RULES:
                if transition["state"] != state:
                    continue
                value = _number(observation, field)
                if value is None:
                    continue
                sample_key = (
                    slot_id, key, transform, margin_family, margin_direction)
                samples.setdefault(sample_key, []).append(value)
                sample_runs.setdefault(sample_key, set()).add(summary["run_id"])

    profiles = {}
    sample_report = {}
    for (slot_id, key, transform, margin_family, margin_direction), values in sorted(samples.items()):
        sample_key = (slot_id, key, transform, margin_family, margin_direction)
        run_count = len(sample_runs[sample_key])
        candidate = None
        if run_count >= int(min_runs):
            candidate = _candidate(
                values, transform, margin_family, margin_m, margin_rad,
                margin_direction)
            profiles.setdefault(slot_id, {})[key] = float(candidate)
        slot_report = sample_report.setdefault(slot_id, {})
        slot_report[key] = {
            "run_count": run_count,
            "values": [float(value) for value in values],
            "candidate": candidate,
        }

    profile_errors = {}
    validated_profiles = {}
    for slot_id, profile in sorted(profiles.items()):
        try:
            # Reuse the runtime validator so an automatically proposed
            # profile cannot bypass threshold ordering or the P4/P5/right-turn
            # safety contract.
            candidate_config = ParkingConfig.from_dict({
                "slot_profiles": {slot_id: profile},
            })
            candidate_config.profile_for(slot_id)
            validated_profiles[slot_id] = profile
        except ValueError as exc:
            profile_errors[slot_id] = str(exc)

    return {
        "successful_or_allowed_run_count": (
            len(successful_summaries) + len(diagnostic_summaries)),
        "successful_run_count": len(successful_summaries),
        "diagnostic_run_count": len(diagnostic_summaries),
        "used_run_ids": [summary["run_id"]
                         for summary in successful_summaries],
        "diagnostic_run_ids": [summary["run_id"]
                               for summary in diagnostic_summaries],
        "run_metadata": [{
            "run_id": summary["run_id"],
            "slot_id": summary.get("slot_id"),
            "successful": bool(summary["successful"]),
            "saw_fault": bool(summary["saw_fault"]),
            "state_sequence_ok": bool(summary["state_sequence_ok"]),
            "slot_supported": bool(summary["slot_supported"]),
            "calibration_ok": bool(summary["calibration_ok"]),
            "success_reason": summary["success_reason"],
            "config_file": summary.get("config_file"),
            "config_sha256": summary.get("config_sha256"),
            "config_hashes": summary.get("config_hashes", {}),
        } for summary in all_summaries],
        "profile_errors": profile_errors,
        "slot_profiles": validated_profiles,
        "samples": sample_report,
    }


def write_profile(path, report):
    fragment = {
        "slot_profiles": report["slot_profiles"],
        "autotune_report": {
            "used_run_ids": report["used_run_ids"],
            "diagnostic_run_ids": report.get("diagnostic_run_ids", []),
            "successful_run_count": report["successful_run_count"],
            "profile_errors": report.get("profile_errors", {}),
            "run_metadata": report.get("run_metadata", []),
        },
    }
    try:
        import yaml
    except ImportError:
        with open(path, "w") as stream:
            json.dump(fragment, stream, indent=2, sort_keys=True)
            stream.write("\n")
        return
    with open(path, "w") as stream:
        yaml.safe_dump(fragment, stream, default_flow_style=False)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="propose measured parking slot profiles from successful JSONL runs")
    parser.add_argument("log_files", nargs="+", help="parking_session_logger JSONL files")
    parser.add_argument("--out", default="",
                        help="optional YAML fragment output path")
    parser.add_argument("--min-runs", type=int, default=1,
                        help="minimum independent runs per proposed key")
    parser.add_argument("--margin-m", type=float, default=0.02,
                        help="conservative distance margin in metres")
    parser.add_argument("--margin-rad", type=float, default=0.03,
                        help="conservative angular margin in radians")
    parser.add_argument("--observation-age-s", type=float, default=0.75)
    parser.add_argument("--allow-failed", action="store_true",
                        help="include failed runs for diagnosis; never use to enable motion")
    args = parser.parse_args(argv)
    if args.observation_age_s <= 0.0:
        parser.error("--observation-age-s must be positive")

    report = build_candidate(
        load_records(args.log_files),
        min_runs=args.min_runs,
        margin_m=args.margin_m,
        margin_rad=args.margin_rad,
        observation_age_s=args.observation_age_s,
        allow_failed=args.allow_failed,
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    if args.out:
        write_profile(args.out, report)
    return 0 if report["slot_profiles"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (IOError, OSError, ValueError, RuntimeError) as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        sys.exit(2)
