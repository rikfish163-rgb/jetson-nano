#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
"""Run the verified 2026-09-05 lane controller with its historical PP core.

This is an isolated compatibility entry point.  It does not copy, overwrite,
or modify the current lane controller or Pure Pursuit implementation.
"""

from __future__ import print_function

import imp
import os
import runpy
import sys


HISTORICAL_PURE_PURSUIT = (
    "/home/nano/robocup_cleanup_backups/20260929/front_preview_line/"
    "src/ros/lane/src/vehicle_control/pure_pursuit.py"
)
HISTORICAL_LANE_NODE = (
    "/home/nano/robocup_ws/src/ros/lane/scripts/lane_ackermann_trial.py"
)


def _require_file(path):
    if not os.path.isfile(path):
        raise RuntimeError("required historical file is missing: %s" % path)


def main():
    _require_file(HISTORICAL_PURE_PURSUIT)
    _require_file(HISTORICAL_LANE_NODE)

    # Keep the historical backup tree read-only during trial runs.
    sys.dont_write_bytecode = True

    # Import the unchanged vehicle model, then bind the verified historical
    # Pure Pursuit module under the package name expected by the 09-05 node.
    import vehicle_control.vehicle_model  # noqa: F401

    historical_module = imp.load_source(
        "vehicle_control.pure_pursuit",
        HISTORICAL_PURE_PURSUIT,
    )
    sys.modules["vehicle_control.pure_pursuit"] = historical_module

    runpy.run_path(HISTORICAL_LANE_NODE, run_name="__main__")


if __name__ == "__main__":
    main()
