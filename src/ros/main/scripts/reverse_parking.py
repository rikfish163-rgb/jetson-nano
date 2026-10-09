#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

"""Start the P4/P5 reverse-parking composition with an explicit target.

The entrypoint is intentionally conservative: without ``--enable-motion`` it
only starts the perception/observation graph and keeps the controller behind
its calibration lock.  Physical actuator and camera drivers are opt-in so an
existing keyboard/base-camera session is not duplicated accidentally.
"""

from __future__ import print_function

import argparse
import os
import sys

try:
    from shlex import quote as shell_quote
except ImportError:
    from pipes import quote as shell_quote


SUPPORTED_SLOTS = ("AUTO", "P4", "P5")
DEFAULT_SPEED_LIMIT_RAW = 20
DEFAULT_STEERING_LIMIT_RAW = 22


def _positive_bounded(value, maximum, name):
    try:
        value = int(value)
    except (TypeError, ValueError):
        raise argparse.ArgumentTypeError("%s must be an integer" % name)
    if value <= 0 or value > maximum:
        raise argparse.ArgumentTypeError(
            "%s must be in the range 1..%d" % (name, maximum))
    return value


def parse_args(argv=None, input_func=None):
    parser = argparse.ArgumentParser(
        description="start the P4/P5 closed-loop reverse-parking stack")
    parser.add_argument(
        "slot", nargs="?",
        help="preferred bay (AUTO/P4/P5); radar may fall back to the other clear bay")
    parser.add_argument(
        "--enable-motion", action="store_true",
        help="allow the bounded supervised experimental motion path")
    parser.add_argument(
        "--start-actuators", action="store_true",
        help="start ackermann/base_controller from this launch")
    parser.add_argument(
        "--start-cameras", action="store_true",
        help="start the configured front/rear USB cameras")
    parser.add_argument(
        "--no-lidar", action="store_true",
        help="do not start the lidar source/adapter (visual dry-run only)")
    parser.add_argument(
        "--enable-exit-pose", action="store_true",
        help="publish the lane-path-based live exit pose")
    parser.add_argument(
        "--front-marker-input-mode", choices=("blue_raw", "metric_white"),
        default="metric_white",
        help="front bay-geometry source; keep metric_white for white bay lines")
    parser.add_argument(
        "--speed-limit-raw", type=lambda value: _positive_bounded(
            value, DEFAULT_SPEED_LIMIT_RAW, "--speed-limit-raw"),
        default=DEFAULT_SPEED_LIMIT_RAW,
        help="experimental speed ceiling, 1..20 raw units")
    parser.add_argument(
        "--steering-limit-raw", type=lambda value: _positive_bounded(
            value, DEFAULT_STEERING_LIMIT_RAW, "--steering-limit-raw"),
        default=DEFAULT_STEERING_LIMIT_RAW,
        help="experimental steering ceiling, 1..22 raw units")
    parser.add_argument(
        "--dry-run", action="store_true",
        help="print the exact roslaunch command without starting it")
    args = parser.parse_args(argv)

    slot = args.slot
    if slot is None:
        if input_func is None:
            try:
                input_func = raw_input
            except NameError:
                input_func = input
        try:
            slot = input_func("选择目标库(P4/P5): ")
        except EOFError:
            parser.error("未提供目标库，请输入 AUTO、P4 或 P5")
    slot = str(slot).strip().upper()
    if slot not in SUPPORTED_SLOTS:
        parser.error("目标库只能是 AUTO、P4 或 P5")
    args.slot = slot
    return args


def build_command(args):
    """Build argv for the existing parking_closed_loop.launch."""
    preparking = bool(args.enable_motion)
    command = [
        "roslaunch",
        "main",
        "parking_closed_loop.launch",
        # AUTO is the runtime mode.  A positional P4/P5 value is a radar
        # selection preference, not a hard assignment: an occupied preferred
        # bay must be allowed to fall back to the other clear bay.
        "slot_id:=AUTO",
        "requested_slot_id:=" + args.slot,
        "parking_only:=" + ("false" if preparking else "true"),
        "start_lane_nodes:=true",
        "start_lane_pose:=true",
        # The central parking controller owns the pre-trigger heading.  Keep
        # the legacy lane command adapter off so track curvature cannot make
        # the vehicle turn before the selected bay trigger.
        "start_preparking_lane_adapter:=false",
        "preparking_speed_raw:=20",
        "request_from_front_slot:=false",
        "allow_legacy_parking_trigger:=false",
        "allow_preparking_approach:=" +
        ("true" if preparking else "false"),
        "front_marker_input_mode:=" + args.front_marker_input_mode,
        "start_cameras:=" + ("true" if args.start_cameras else "false"),
        "start_actuators:=" + ("true" if args.start_actuators else "false"),
        "start_lidar_adapter:=" + ("false" if args.no_lidar else "true"),
        "start_lidar_source:=" + ("false" if args.no_lidar else "true"),
        "experimental_speed_limit_raw:=%d" % args.speed_limit_raw,
        "experimental_steering_limit_raw:=%d" % args.steering_limit_raw,
    ]
    enable_exit_pose = args.enable_exit_pose or args.enable_motion
    command.append(
        "enable_exit_pose:=" + ("true" if enable_exit_pose else "false"))
    if args.enable_motion:
        command.extend([
            "allow_experimental_motion:=true",
            "require_calibrated_parking:=false",
        ])
    else:
        command.extend([
            "allow_experimental_motion:=false",
            "require_calibrated_parking:=true",
        ])
    return command


def main(argv=None):
    args = parse_args(argv)
    command = build_command(args)
    printable = " ".join(shell_quote(item) for item in command)
    print("目标库: %s" % args.slot)
    print("启动命令: %s" % printable)
    if not args.enable_motion:
        print("当前为视觉/状态机预览，未启用车辆运动；需要实车低速试验时再加 --enable-motion。")
    if args.dry_run:
        return 0
    os.execvp(command[0], command)
    return 0


if __name__ == "__main__":
    sys.exit(main())
