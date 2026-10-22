#!/usr/bin/env bash
# Same vehicle route; an independent subscriber records the diagnostic run.
set -eo pipefail
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
unset ROS_HOSTNAME
export ROS_IP=127.0.0.1
if pgrep -f '/usb_cam_node|/robot/master/main.py|/robot/master/controller_node.py|__name:=parking_diagnostic_record' >/dev/null; then
  echo '已有车辆、摄像头或诊断录制在运行，请先在原终端 Ctrl+C。' >&2
  exit 1
fi
available_kb=$(df -Pk . | awk 'END {print $4}')
if (( available_kb < 650 * 1024 )); then
  echo '录制需要至少 650 MiB 可用空间，未启动车辆。' >&2
  exit 1
fi
session="/home/nano/robocup_ws/field_data/parking_diagnostics/$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$session"
cp src/robot/config/vehicle_tuning.yaml "$session/vehicle_tuning.yaml"
# Preserve the exact vision/control sources used by this recorded run.
mkdir -p "$session/source"
source_paths=(src/robot/parking/p_line_reference.py src/robot/parking/timed_core.py
  src/robot/parking/timed_vision.py src/robot/parking/timed_vision_node.py
  src/robot/lidar/side_bays.py src/robot/lidar/side_bay_node.py
  src/robot/parallel_parking/reference_camera.yaml src/robot/config/competition.yaml)
for source_path in "${source_paths[@]}"; do
  cp --parents "$source_path" "$session/source"
done
sha256sum "${source_paths[@]}" > "$session/source_sha256.txt"
printf '%s\n' "$@" > "$session/launch_arguments.txt"
printf '录制目录：%s\n连续录制摄像头已发布帧和诊断数据；上限 120 秒或 300 MiB，先到即停止录制，不改变车辆控制。\n' "$session"
roslaunch tools/parking_diagnostic_record.launch "output:=$session/run.bag" seconds:=120 max_mb:=300 > "$session/recorder.log" 2>&1 &
recorder_pid=$!
cleanup() {
  kill -INT "$recorder_pid" 2>/dev/null || true
  wait "$recorder_pid" 2>/dev/null || true
  printf '\n录制文件已收尾：%s\n' "$session"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
ready=false
for attempt in {1..20}; do
  if ! kill -0 "$recorder_pid" 2>/dev/null; then break; fi
  if rosnode ping -c 1 /parking_diagnostic_record >/dev/null 2>&1; then ready=true; break; fi
  sleep 0.5
done
if [[ "$ready" != true ]]; then
  cat "$session/recorder.log" >&2
  echo '录制器未就绪，未启动车辆。' >&2
  exit 1
fi
# Avoid duplicate PNG snapshots/processed-frame recording during this full run.
bash runall.sh "$@" sign_capture:=false record_lane:=false \
  2>&1 | tee "$session/vehicle.log"
