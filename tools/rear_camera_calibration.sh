#!/usr/bin/env bash
# Camera-only ROS calibration GUI, intended for an ssh -X terminal.
set -e
source /opt/ros/melodic/setup.bash
source /home/nano/robocup_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
unset ROS_HOSTNAME
export ROS_IP=127.0.0.1
export QT_X11_NO_MITSHM=1

if [[ -z "${DISPLAY:-}" ]]; then
  echo 'No X11 display. Connect with ssh -X -t nano@100.86.37.124.' >&2
  exit 1
fi
rear_device=/dev/v4l/by-path/platform-70090000.xusb-usb-0:2.4:1.0-video-index0
if fuser "$rear_device" >/dev/null 2>&1; then
  echo 'Rear camera is busy. Stop its previous process with Ctrl+C first.' >&2
  exit 1
fi
rear_calibration_log=$(mktemp /tmp/rear-calibration-launch.XXXXXX.log)
roslaunch robocup_competition rear_camera_capture.launch \
  rear_device:="$rear_device" rear_fps:=8 start_ground:=false \
  >"$rear_calibration_log" 2>&1 &
rear_calibration_pid=$!
cleanup() {
  kill -INT "$rear_calibration_pid" 2>/dev/null || true
  wait "$rear_calibration_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

rear_calibration_ready=false
for attempt in {1..40}; do
  if ! kill -0 "$rear_calibration_pid" 2>/dev/null; then
    cat "$rear_calibration_log"
    exit 1
  fi
  if rosparam get /run_id >/dev/null 2>&1; then
    rear_calibration_ready=true
    break
  fi
  sleep .5
done
if [[ "$rear_calibration_ready" != true ]]; then
  cat "$rear_calibration_log"
  exit 1
fi
echo "Camera launch log: $rear_calibration_log"
rosrun camera_calibration cameracalibrator.py \
  --size 8x11 --square 0.02 --camera_name rear_camera \
  --no-service-check --disable_calib_cb_fast_check \
  image:=/rear/usb_cam/image_raw camera:=/rear/rear_camera
