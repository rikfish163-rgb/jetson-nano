#!/usr/bin/env bash
# Supervised stationary wheel test; never publishes a nonzero drive speed.
set -eo pipefail
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
set -u
export ROS_MASTER_URI=http://localhost:11311
unset ROS_HOSTNAME
export ROS_IP=127.0.0.1
if pgrep -f '/base_controller|/testloop.py|/robot/master/main.py|/robot/master/controller_node.py' >/dev/null; then
  echo '请先在原车辆运行终端 Ctrl+C，当前有底盘或控制节点运行，测试未启动。' >&2
  exit 1
fi
probe_dir="$(mktemp -d /home/nano/robocup_ws/field_data/steering_probe_XXXXXXXX)"
launch_pid=''
record_pid=''
cleanup() {
  if [ -n "$launch_pid" ]; then
    kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
  if [ -n "$record_pid" ]; then
    kill -TERM "$record_pid" 2>/dev/null || true
    wait "$record_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM
echo "静止转向测试：速度始终为 0；请观察前轮。日志：$probe_dir"
roslaunch robocup_competition stack.launch \
  start_controller:=false start_actuators:=true \
  start_cameras:=false start_rear:=false start_lane:=false \
  start_lidar:=false start_sign:=false start_ground:=false \
  > "$probe_dir/launch.log" 2>&1 &
launch_pid=$!
ready=false
for attempt in $(seq 1 20); do
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    cat "$probe_dir/launch.log" >&2
    exit 1
  fi
  if rosnode ping -c 1 /ackermann_control_bridge >/dev/null 2>&1 && \
     rosnode ping -c 1 /base_controller >/dev/null 2>&1; then
    ready=true
    break
  fi
  sleep .5
done
if [ "$ready" != true ]; then
  echo '底盘节点未就绪，测试未发送。' >&2
  cat "$probe_dir/launch.log" >&2
  exit 1
fi
rostopic echo /base_controller/status > "$probe_dir/base_status.yaml" 2>&1 &
record_pid=$!
for steering in -3 -8 -15 -22 3 8 15 22; do
  echo "当前请求舵量 $steering，保持 2 秒，然后回正；速度 0。"
  python2 src/robot/motion/bench_command.py --speed 0 \
    --steering "$steering" --seconds 2 --confirm-wheels-safe
  sleep .3
done
echo "测试完成，已请求回正。记录在 $probe_dir"
