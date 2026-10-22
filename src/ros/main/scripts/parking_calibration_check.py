#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Offline readiness check before arming the P4/P5 parking controller.

This command is intentionally stronger than a YAML syntax check and weaker
than a field test.  It verifies that the runtime configuration, slot profiles,
camera/BEV files, and declared visual pose paths exist and are internally
consistent.  It never imports rospy, opens a device, publishes a command, or
claims that a camera currently sees a valid line.
"""

from __future__ import print_function

import argparse
import json
import math
import os
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MAIN_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
SRC_DIR = os.path.abspath(os.path.join(MAIN_DIR, ".."))
CAMERA_SCRIPTS_DIR = os.path.join(SRC_DIR, "camera", "scripts")
DEFAULT_CONFIG = os.path.join(MAIN_DIR, "config", "parking.yaml")
DEFAULT_FRONT_CONFIG = os.path.join(
    SRC_DIR, "camera", "config", "parking_front.yaml")
DEFAULT_REAR_CONFIG = os.path.join(
    SRC_DIR, "camera", "config", "rear_parking.yaml")
DEFAULT_EXIT_CONFIG = os.path.join(
    SRC_DIR, "camera", "config", "parking_exit.yaml")
DEFAULT_LIDAR_CONFIG = os.path.join(
    SRC_DIR, "camera", "config", "parking_lidar.yaml")
DEFAULT_REAR_BEV = os.path.join(
    SRC_DIR, "camera", "calibration", "rear_bev.yaml")
DEFAULT_FRONT_CALIBRATION = os.path.join(
    SRC_DIR, "camera", "calibration", "front_640x360.yaml")
DEFAULT_REAR_CALIBRATION = os.path.join(
    SRC_DIR, "camera", "calibration", "rear_640x480.yaml")

if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
if CAMERA_SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, CAMERA_SCRIPTS_DIR)

from parking_controller import ParkingConfig  # noqa: E402
from parking_lidar_adapter import LidarAdapterConfig  # noqa: E402


try:
    string_types = (basestring,)
except NameError:
    string_types = (str,)


def _finite(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return False
    return not math.isnan(value) and not math.isinf(value)


def _load_yaml(path):
    try:
        import yaml
    except ImportError:
        raise RuntimeError("PyYAML is required for the offline calibration check")
    if not os.path.isfile(path):
        raise IOError("file not found: %s" % path)
    with open(path, "r") as stream:
        data = yaml.safe_load(stream)
    if data is None:
        data = {}
    if not isinstance(data, dict):
        raise ValueError("YAML root must be a mapping: %s" % path)
    return data


def _flat_numeric(value):
    """Flatten a YAML numeric vector/matrix and reject non-finite values."""
    if isinstance(value, dict):
        value = value.get("data")
    if not isinstance(value, (list, tuple)):
        return None
    flattened = []
    for item in value:
        nested = _flat_numeric(item) if isinstance(item, (list, tuple)) else [item]
        if nested is None:
            return None
        flattened.extend(nested)
    result = []
    for item in flattened:
        if not _finite(item):
            return None
        result.append(float(item))
    return result


def _same_numeric_vector(first, second, tolerance=1.0e-6):
    first = _flat_numeric(first)
    second = _flat_numeric(second)
    if first is None or second is None or len(first) != len(second):
        return False
    return all(abs(a - b) <= tolerance for a, b in zip(first, second))


def _rear_bev_intrinsics_match(rear_bev_data, rear_calibration_data):
    """Ensure the BEV H is paired with the camera's current K and D.

    ``rectified_camera_matrix`` is intentionally not compared with the
    camera YAML projection matrix: the BEV calibration may use a different,
    explicitly stored optimal rectification matrix, as the current H does.
    """
    if not isinstance(rear_bev_data, dict) or not isinstance(
            rear_calibration_data, dict):
        return False
    if (rear_bev_data.get("image_width") !=
            rear_calibration_data.get("image_width") or
            rear_bev_data.get("image_height") !=
            rear_calibration_data.get("image_height")):
        return False
    return (
        _same_numeric_vector(
            rear_bev_data.get("camera_matrix"),
            rear_calibration_data.get("camera_matrix")) and
        _same_numeric_vector(
            rear_bev_data.get("distortion_coefficients"),
            rear_calibration_data.get("distortion_coefficients"))
    )


def _check(checks, name, passed, reason, required=True, path=None):
    item = {
        "name": name,
        "passed": bool(passed),
        "required": bool(required),
        "reason": str(reason),
    }
    if path:
        item["path"] = path
    checks.append(item)
    return bool(passed)


def _file_check(checks, name, path, required=True):
    return _check(
        checks,
        name,
        os.path.isfile(path),
        "present" if os.path.isfile(path) else "missing",
        required=required,
        path=path,
    )


def _visual_mode_check(checks, name, mode, allowed, topic=None):
    if mode not in allowed:
        return _check(
            checks,
            name,
            False,
            "mode must be one of: %s" % ", ".join(allowed),
        )
    if mode == "external" and not topic:
        return _check(checks, name, False, "external topic is missing")
    return _check(checks, name, True, "declared:%s" % mode)


def check_readiness(
        config_path=DEFAULT_CONFIG,
        front_config_path=DEFAULT_FRONT_CONFIG,
        rear_config_path=DEFAULT_REAR_CONFIG,
        exit_config_path=DEFAULT_EXIT_CONFIG,
        lidar_config_path=DEFAULT_LIDAR_CONFIG,
        rear_bev_path=DEFAULT_REAR_BEV,
        front_calibration_path=DEFAULT_FRONT_CALIBRATION,
        rear_calibration_path=DEFAULT_REAR_CALIBRATION,
        front_pose_mode="none",
        exit_pose_mode="none",
        external_front_topic="",
        external_exit_topic="",
        start_lane_nodes=False,
        start_lane_pose=False,
        front_device="",
        rear_device=""):
    """Return a JSON-serializable static readiness report.

    ``front_pose_mode`` and ``exit_pose_mode`` describe the source that the
    caller intends to launch.  They are explicit because a static checker
    cannot discover whether an external topic will actually be published.
    ``lane_pose`` is accepted as the exit source when both lane nodes are
    enabled.  The lane adapter now derives ``exit_distance_m`` from the
    nearest fresh point of the fitted base_link path when no fixed override is
    supplied; the live controller still requires that source to be fresh.
    """
    checks = []
    config = None
    config_data = None
    config_error = None

    try:
        config_data = _load_yaml(config_path)
        try:
            config = ParkingConfig.from_dict(config_data)
            _check(checks, "parking_config", True, "valid", path=config_path)
        except ValueError as exc:
            # Keep a structurally valid but incomplete calibrated config
            # available for diagnostics below.  The normal runtime parser
            # still rejects it; this fallback only makes the checker explain
            # which measured profile fields are absent.
            config_error = str(exc)
            config = ParkingConfig(
                config_data, _require_slot_profiles=False)
            _check(checks, "parking_config", False, config_error,
                   path=config_path)
    except (IOError, OSError, RuntimeError, ValueError) as exc:
        config_error = str(exc)
        _check(checks, "parking_config", False, str(exc), path=config_path)

    profile_complete = False
    if config is not None:
        supported = tuple(config.supported_slot_ids)
        profile_keys = set(config.slot_profiles.keys())
        missing = sorted(set(supported) - profile_keys)
        profile_complete = not missing
        _check(
            checks,
            "slot_profiles_present",
            profile_complete,
            "all supported slots have profiles" if profile_complete else
            "missing profiles: %s" % ", ".join(missing),
        )
        measured_fields_ok = True
        measured_field_errors = {}
        for slot_id in supported:
            profile = config.slot_profiles.get(slot_id, {})
            missing_fields = [
                field for field in config.CALIBRATED_SLOT_FIELDS
                if field not in profile
            ]
            if missing_fields:
                measured_fields_ok = False
                measured_field_errors[slot_id] = missing_fields
        _check(
            checks,
            "slot_profile_measured_fields",
            measured_fields_ok,
            "all measured fields are explicit" if measured_fields_ok else
            json.dumps(measured_field_errors),
        )
        profile_valid = True
        profile_errors = {}
        for slot_id in supported:
            try:
                config.profile_for(slot_id)
            except ValueError as exc:
                profile_valid = False
                profile_errors[slot_id] = str(exc)
        _check(
            checks,
            "slot_profiles_valid",
            profile_valid,
            "valid" if profile_valid else json.dumps(profile_errors),
        )
        _check(
            checks,
            "calibration_complete",
            config.calibration_complete is True,
            "enabled" if config.calibration_complete else
            "calibration_complete is false",
        )
    else:
        _check(checks, "slot_profiles_present", False, "parking config unavailable")
        _check(checks, "slot_profile_measured_fields", False,
               "parking config unavailable")
        _check(checks, "slot_profiles_valid", False, "parking config unavailable")
        _check(checks, "calibration_complete", False, "parking config unavailable")

    front_data = None
    try:
        front_data = _load_yaml(front_config_path)
        _check(checks, "front_config", True, "valid", path=front_config_path)
    except (IOError, OSError, RuntimeError, ValueError) as exc:
        _check(checks, "front_config", False, str(exc), path=front_config_path)

    rear_data = None
    try:
        rear_data = _load_yaml(rear_config_path)
        _check(checks, "rear_config", True, "valid", path=rear_config_path)
    except (IOError, OSError, RuntimeError, ValueError) as exc:
        _check(checks, "rear_config", False, str(exc), path=rear_config_path)

    exit_data = None
    try:
        exit_data = _load_yaml(exit_config_path)
        _check(checks, "exit_config", True, "valid", path=exit_config_path)
    except (IOError, OSError, RuntimeError, ValueError) as exc:
        _check(checks, "exit_config", False, str(exc), path=exit_config_path)

    lidar_data = None
    lidar_config_ok = False
    try:
        lidar_data = _load_yaml(lidar_config_path)
        LidarAdapterConfig(lidar_data)
        lidar_config_ok = _check(
            checks, "lidar_config", True, "valid", path=lidar_config_path)
    except (IOError, OSError, RuntimeError, ValueError) as exc:
        _check(checks, "lidar_config", False, str(exc), path=lidar_config_path)

    rear_bev_data = None
    try:
        rear_bev_data = _load_yaml(rear_bev_path)
        _check(checks, "rear_bev_calibration", True, "valid", path=rear_bev_path)
    except (IOError, OSError, RuntimeError, ValueError) as exc:
        _check(checks, "rear_bev_calibration", False, str(exc), path=rear_bev_path)

    front_calibration_data = None
    try:
        front_calibration_data = _load_yaml(front_calibration_path)
        _check(checks, "front_camera_calibration", True, "valid",
               path=front_calibration_path)
    except (IOError, OSError, RuntimeError, ValueError) as exc:
        _check(checks, "front_camera_calibration", False, str(exc),
               path=front_calibration_path)

    rear_calibration_data = None
    try:
        rear_calibration_data = _load_yaml(rear_calibration_path)
        _check(checks, "rear_camera_calibration", True, "valid",
               path=rear_calibration_path)
    except (IOError, OSError, RuntimeError, ValueError) as exc:
        _check(checks, "rear_camera_calibration", False, str(exc),
               path=rear_calibration_path)

    def _calibration_shape_ok(data, width, height):
        return bool(
            isinstance(data, dict) and
            data.get("image_width") == width and
            data.get("image_height") == height and
            isinstance(data.get("camera_matrix"), dict) and
            isinstance(data.get("distortion_coefficients"), dict)
        )

    _check(
        checks,
        "front_camera_calibration_shape",
        _calibration_shape_ok(front_calibration_data, 640, 360),
        "640x360 calibration present" if _calibration_shape_ok(
            front_calibration_data, 640, 360) else
        "expected 640x360 camera calibration fields are missing",
    )
    _check(
        checks,
        "rear_camera_calibration_shape",
        _calibration_shape_ok(rear_calibration_data, 640, 480),
        "640x480 calibration present" if _calibration_shape_ok(
            rear_calibration_data, 640, 480) else
        "expected 640x480 camera calibration fields are missing",
    )

    rear_bev_fields = (
        "image_width",
        "image_height",
        "bev_width",
        "bev_height",
        "camera_matrix",
        "distortion_coefficients",
        "rectified_camera_matrix",
        "homography_rectified_to_bev",
    )
    rear_bev_ok = bool(
        isinstance(rear_bev_data, dict) and
        all(field in rear_bev_data for field in rear_bev_fields) and
        ("pixels_per_m" in rear_bev_data or
         "pixels_per_metre" in rear_bev_data)
    )
    _check(
        checks,
        "rear_bev_calibration_fields",
        rear_bev_ok,
        "required BEV matrices and scale present" if rear_bev_ok else
        "BEV matrices or pixel scale are missing",
    )
    rear_intrinsics_match = _rear_bev_intrinsics_match(
        rear_bev_data, rear_calibration_data)
    _check(
        checks,
        "rear_bev_intrinsics_consistency",
        rear_intrinsics_match,
        "BEV K/D match rear camera calibration" if rear_intrinsics_match else
        "BEV K/D do not match rear camera calibration",
    )

    front_mode_ok = _visual_mode_check(
        checks,
        "front_pose_source",
        front_pose_mode,
        ("candidate_affine", "lane_pose", "external"),
        external_front_topic,
    )
    if front_pose_mode == "candidate_affine":
        front_mode_ok = _check(
            checks,
            "front_candidate_affine_enabled",
            isinstance(front_data, dict) and
            front_data.get("derive_pose_from_candidate") is True,
            "enabled" if isinstance(front_data, dict) and
            front_data.get("derive_pose_from_candidate") is True else
            "parking_front.yaml derive_pose_from_candidate is false",
        ) and front_mode_ok
    elif front_pose_mode == "lane_pose":
        front_mode_ok = _check(
            checks,
            "front_lane_pose_launch",
            bool(start_lane_nodes and start_lane_pose),
            "lane nodes enabled" if start_lane_nodes and start_lane_pose else
            "start_lane_nodes and start_lane_pose must both be true",
        ) and front_mode_ok

    rear_fields = (
        "line_color",
        "expected_slot_width_m",
        "camera_to_rear_axle_m",
    )
    rear_fields_ok = bool(
        isinstance(rear_data, dict) and
        rear_data.get("line_color") in ("white", "blue") and
        all(key in rear_data and _finite(rear_data.get(key))
            for key in ("expected_slot_width_m", "camera_to_rear_axle_m")) and
        rear_data.get("camera_to_rear_axle_calibrated") is True and
        rear_data.get("line_color_calibrated") is True
    )
    _check(
        checks,
        "rear_metric_parameters",
        rear_fields_ok,
        "required fields present" if rear_fields_ok else
        "rear metric fields or calibration certificates are incomplete",
    )

    exit_mode_ok = _visual_mode_check(
        checks,
        "exit_pose_source",
        exit_pose_mode,
        ("candidate_affine", "lane_pose", "external"),
        external_exit_topic,
    )
    if exit_pose_mode == "candidate_affine":
        exit_mode_ok = _check(
            checks,
            "exit_candidate_affine_enabled",
            isinstance(exit_data, dict) and
            exit_data.get("use_candidate") is True,
            "enabled" if isinstance(exit_data, dict) and
            exit_data.get("use_candidate") is True else
            "parking_exit.yaml use_candidate is false",
        ) and exit_mode_ok

    if exit_pose_mode == "lane_pose":
        # The lane adapter publishes a dynamic exit distance from the nearest
        # fresh point in the base_link path.  Static checking cannot prove a
        # camera currently sees that line, so runtime freshness remains the
        # final gate in parking_exit_metric and the FSM.
        _check(
            checks,
            "exit_lane_pose_launch",
            bool(start_lane_nodes and start_lane_pose),
            "lane nodes enabled" if start_lane_nodes and start_lane_pose else
            "start_lane_nodes and start_lane_pose must both be true",
        )

    devices_checked = False
    if front_device or rear_device:
        devices_checked = True
        if front_device:
            _file_check(checks, "front_device", front_device)
        if rear_device:
            _file_check(checks, "rear_device", rear_device)

    required_passed = all(
        item["passed"] for item in checks if item.get("required"))
    measured_profiles_ready = bool(
        config is not None and
        any(item["name"] == "slot_profile_measured_fields" and
            item["passed"] for item in checks)
    )
    static_visual_ready = bool(
        front_mode_ok and exit_mode_ok and rear_fields_ok and
        lidar_config_ok and
        measured_profiles_ready and
        rear_bev_ok and
        rear_intrinsics_match and
        _calibration_shape_ok(front_calibration_data, 640, 360) and
        _calibration_shape_ok(rear_calibration_data, 640, 480)
    )
    motion_ready = bool(
        config is not None and config.calibration_complete is True and
        profile_complete and required_passed and static_visual_ready
    )
    blocking = [
        item["name"] for item in checks
        if item.get("required") and not item.get("passed")
    ]

    return {
        "schema": "parking_calibration_check_v1",
        "passed": bool(motion_ready),
        "motion_ready": bool(motion_ready),
        "static_visual_ready": bool(static_visual_ready),
        "devices_checked": bool(devices_checked),
        "config_file": config_path,
        "config_calibration_complete": bool(
            config is not None and config.calibration_complete is True),
        "supported_slot_ids": list(config.supported_slot_ids)
        if config is not None else [],
        "blocking": blocking,
        "checks": checks,
        "note": (
            "motion_ready only means static configuration is complete; it is "
            "not evidence that the vehicle moved or that the field geometry "
            "has been visually verified"
        ),
    }


def _print_human(report):
    print("parking calibration check: %s" %
          ("READY_FOR_MOTION_CONFIG" if report["motion_ready"] else "NOT_READY"))
    print("static_visual_ready: %s" % report["static_visual_ready"])
    if report["blocking"]:
        print("blocking:")
        for name in report["blocking"]:
            print("  - %s" % name)
    else:
        print("blocking: none")
    for item in report["checks"]:
        status = "OK" if item["passed"] else "FAIL"
        print("[%s] %s: %s" % (status, item["name"], item["reason"]))
    print(report["note"])


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="offline static readiness check for P4/P5 parking")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--front-config", default=DEFAULT_FRONT_CONFIG)
    parser.add_argument("--rear-config", default=DEFAULT_REAR_CONFIG)
    parser.add_argument("--exit-config", default=DEFAULT_EXIT_CONFIG)
    parser.add_argument("--lidar-config", default=DEFAULT_LIDAR_CONFIG)
    parser.add_argument("--rear-bev", default=DEFAULT_REAR_BEV)
    parser.add_argument("--front-calibration", default=DEFAULT_FRONT_CALIBRATION)
    parser.add_argument("--rear-calibration", default=DEFAULT_REAR_CALIBRATION)
    parser.add_argument(
        "--front-pose-mode",
        choices=("none", "candidate_affine", "lane_pose", "external"),
        default="none",
    )
    parser.add_argument(
        "--exit-pose-mode",
        choices=("none", "candidate_affine", "lane_pose", "external"),
        default="none",
    )
    parser.add_argument("--external-front-topic", default="")
    parser.add_argument("--external-exit-topic", default="")
    parser.add_argument("--front-device", default="")
    parser.add_argument("--rear-device", default="")
    parser.add_argument("--start-lane-nodes", action="store_true")
    parser.add_argument("--start-lane-pose", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    report = check_readiness(
        config_path=args.config,
        front_config_path=args.front_config,
        rear_config_path=args.rear_config,
        exit_config_path=args.exit_config,
        lidar_config_path=args.lidar_config,
        rear_bev_path=args.rear_bev,
        front_calibration_path=args.front_calibration,
        rear_calibration_path=args.rear_calibration,
        front_pose_mode=args.front_pose_mode,
        exit_pose_mode=args.exit_pose_mode,
        external_front_topic=args.external_front_topic,
        external_exit_topic=args.external_exit_topic,
        start_lane_nodes=args.start_lane_nodes,
        start_lane_pose=args.start_lane_pose,
        front_device=args.front_device,
        rear_device=args.rear_device,
    )
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        _print_human(report)
    return 0 if report["motion_ready"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (IOError, OSError, RuntimeError, ValueError) as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        sys.exit(2)
