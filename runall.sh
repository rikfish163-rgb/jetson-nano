#!/usr/bin/env bash
set -e
# Manage future boots only; never start or stop the vehicle here.
case "${1-}" in
  --enable|--enable=*)
    if [[ "$1" == --enable ]]; then
      [[ $# == 2 ]] || { echo 'Usage: bash runall.sh --enable true|false' >&2; exit 2; }
      boot_enable="$2"
    else
      [[ $# == 1 ]] || { echo 'Do not combine --enable with driving parameters' >&2; exit 2; }
      boot_enable="${1#--enable=}"
    fi
    case "$boot_enable" in
      true)
        script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
        sudo install -m 0644 "$script_dir/tools/robocup-vehicle.service" /etc/systemd/system/robocup-vehicle.service
        sudo systemctl daemon-reload
        sudo systemctl enable robocup-vehicle.service
        echo 'Boot autostart enabled. Next boot will wait for green. Vehicle not started now.'
        ;;
      false)
        sudo systemctl disable robocup-vehicle.service
        echo 'Boot autostart disabled. Any currently running vehicle is unchanged.'
        ;;
      *) echo '--enable requires true or false' >&2; exit 2 ;;
    esac
    exit 0
    ;;
esac
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source /home/nano/robocup_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
unset ROS_HOSTNAME
export ROS_IP=127.0.0.1
# Vehicle settings live in src/robot/config/vehicle_tuning.yaml.
# Explicit name:=value arguments override that file for this run only.
exec python2 tools/vehicle_launch.py "$@"
