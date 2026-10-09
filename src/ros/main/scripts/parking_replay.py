#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Replay a logged parking session without publishing any vehicle command.

The session logger stores the same observation, lidar, status and metric
contracts used at runtime.  This tool runs the controller against that
chronological evidence and reports the first fault, state transitions and the
last safe command.  It is intentionally offline-only: it never imports ROS or
publishes to the chassis.
"""

from __future__ import print_function

import argparse
import json
import os
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from parking_controller import ParkingConfig, ParkingController


def _payload(record):
    if not isinstance(record, dict):
        return None
    data = record.get("data")
    if not isinstance(data, dict):
        return None
    value = data.get("payload")
    return value if isinstance(value, dict) else None


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
            try:
                record["wall_time"] = float(record.get("wall_time"))
            except (TypeError, ValueError):
                continue
            records.append(record)
    records.sort(key=lambda item: item["wall_time"])
    return records


def load_config(path, allow_uncalibrated=False):
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
        values = dict(values)
    if allow_uncalibrated:
        values["calibration_complete"] = True
        # This flag is an offline-only replay convenience.  It must not be
        # confused with a field unlock and therefore explicitly bypasses the
        # runtime requirement for measured per-slot profile fields.
        return ParkingConfig(values, _require_slot_profiles=False)
    return ParkingConfig.from_dict(values)


def replay(records, config):
    controller = ParkingController(config)
    latest_lidar = None
    started = False
    previous_state = controller.state
    transitions = []
    commands = 0
    safe_stops = 0
    first_fault = None
    first_time = None
    last_time = None
    last_command = controller.step(None, None, now=0.0)

    for record in records:
        now = record["wall_time"]
        last_time = now
        event = record.get("event")
        payload = _payload(record)
        if event == "lidar" and payload is not None:
            latest_lidar = payload
            continue
        if event != "observation" or payload is None:
            continue

        if not started:
            if payload.get("parking_request") is not True:
                continue
            slot_id = payload.get("slot_id")
            if not controller.start(slot_id, now=now):
                first_fault = {
                    "state": controller.state,
                    "reason": controller.reason,
                    "wall_time": now,
                }
                started = True
                continue
            started = True
            first_time = now

        last_command = controller.step(payload, latest_lidar, now=now)
        commands += 1
        if last_command.get("safe_stop"):
            safe_stops += 1
        state = controller.state
        if state != previous_state:
            transitions.append({
                "wall_time": now,
                "state": state,
                "state_reason": controller.state_reason,
                "command_reason": last_command.get("reason"),
                "transition_seq": controller.transition_seq,
            })
            previous_state = state
        if first_fault is None and state == "fault":
            first_fault = {
                "state": state,
                "reason": last_command.get("reason"),
                "wall_time": now,
            }

    duration = None
    if first_time is not None and last_time is not None:
        duration = max(0.0, last_time - first_time)
    final_status = controller.status(
        now=last_time if last_time is not None else 0.0,
        command=last_command,
    )
    return {
        "started": bool(started),
        "final_state": controller.state,
        "final_reason": controller.reason,
        "slot_id": controller.slot_id,
        "done": bool(controller.done),
        "fault": bool(controller.fault),
        "first_fault": first_fault,
        "transition_count": len(transitions),
        "transitions": transitions,
        "observation_steps": commands,
        "safe_stop_steps": safe_stops,
        "duration_s": duration,
        "last_command": last_command,
        "final_status": final_status,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Replay a parking JSONL session without ROS or motion")
    parser.add_argument("log_file", help="parking_session_logger JSONL file")
    parser.add_argument("--config", default="", help="parking.yaml used for the run")
    parser.add_argument(
        "--allow-uncalibrated-replay",
        action="store_true",
        help="override calibration_complete only for offline replay")
    args = parser.parse_args(argv)

    records = load_records(args.log_file)
    config = load_config(args.config, args.allow_uncalibrated_replay)
    report = replay(records, config)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["done"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (IOError, OSError, ValueError, RuntimeError) as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        sys.exit(2)
