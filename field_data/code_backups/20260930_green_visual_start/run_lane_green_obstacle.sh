#!/usr/bin/env bash
set -e
cd /home/nano/robocup_ws
# The shared entry validates the engine and prevents duplicate vehicle nodes.
# Retain green release and collision/unknown-space protection with no parking.
exec bash tools/sign_detector_20260916/run_yolo_vehicle.sh \
  parking_enabled:=false start_rear:=false wait_green:=true lidar_enabled:=true \
  lane_speed_raw:=12 action_speed_raw:=24 straight_speed_raw:=24 \
  lane_curvature_preview:=true lookahead:=0.8 lane_curve_speed_raw:=12 \
  debug_view:=false record_lane:=false sign_hz:=2.0 ground_hz:=4.0 "$@"
