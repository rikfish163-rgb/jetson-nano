#!/usr/bin/env bash
set -e
cd /home/nano/robocup_ws
# The shared entry validates the engine and prevents duplicate vehicle nodes.
# Retain green release and collision/unknown-space protection with no parking.
# Mandatory 1.25 m straight at 12; lane bends and uncertain exits use 16.
# Lane control admits 24 only after sustained centered straight observations.
exec bash tools/sign_detector_20260916/run_yolo_vehicle.sh \
  parking_enabled:=false start_rear:=false wait_green:=true lidar_enabled:=true \
  startup_follow_lane:=false straight_distance:=1.25 \
  lane_speed_raw:=24 action_speed_raw:=24 straight_speed_raw:=12 \
  lane_curvature_preview:=true lookahead:=0.8 lane_curve_speed_raw:=16 \
  debug_view:=false record_lane:=false sign_hz:=2.0 ground_hz:=4.0 "$@"
