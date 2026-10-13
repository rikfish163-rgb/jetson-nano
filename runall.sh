#!/usr/bin/env bash
set -e
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source /home/nano/robocup_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
unset ROS_HOSTNAME
export ROS_IP=127.0.0.1
# Vehicle settings live in src/robot/config/vehicle_tuning.yaml.
# Explicit name:=value arguments override that file for this run only.
exec python2 tools/vehicle_launch.py "$@"
