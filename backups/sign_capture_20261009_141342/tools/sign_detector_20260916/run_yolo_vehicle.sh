#!/usr/bin/env bash
set -e
parking_slot=P4
parking_style=
requested_mode=
launch_args=()
for arg in "$@"; do
  case "$arg" in
    --S|--T) style="${arg#--}" ;;
    parking_entry_style:=*) style="${arg#parking_entry_style:=}" ;;
    parking_slot:=*) parking_slot="${arg#parking_slot:=}"; continue ;;
    parking_mode:=*) requested_mode="${arg#parking_mode:=}"; continue ;;
    *) launch_args+=("$arg"); continue ;;
  esac
  if [[ "$style" != S && "$style" != T ]] ||
     [[ -n "$parking_style" && "$parking_style" != "$style" ]]; then
    echo '入库方式只能选 --S 或 --T，不能同时选择。' >&2
    exit 2
  fi
  parking_style="$style"
done
case "$parking_slot" in
  P1|P2|P3)
    if [[ -n "$parking_style" ]]; then
      echo '--S/--T 仅用于 P4/P5；P1/P2/P3 按显式车位执行侧方。' >&2
      exit 2
    fi
    parking_mode=timed_sequence
    parking_description=侧方
    ;;
  P4|P5)
    parking_mode=forward_center
    parking_style="${parking_style:-S}"
    if [[ "$parking_style" == S ]]; then
      parking_description='识别P后直入库'
    else
      parking_description='识别P后到蓝线，前进右满舵1秒后停车'
    fi
    ;;
  *) echo '必须用 parking_slot:=P1/P2/P3/P4/P5 显式选择车位。' >&2; exit 2 ;;
esac
if [[ -n "$requested_mode" && "$requested_mode" != "$parking_mode" ]]; then
  echo "车位 $parking_slot 与 parking_mode:=$requested_mode 不匹配；请删除旧的 parking_mode 参数。" >&2
  exit 2
fi
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source /home/nano/robocup_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
unset ROS_HOSTNAME
export ROS_IP=127.0.0.1
if [ ! -s /home/nano/robodata/models/20260916_yolov5s/yolov5s_640_fp16.engine ]; then
  echo 'YOLO TensorRT 引擎尚未就绪，未启动车辆。' >&2
  exit 1
fi
if pgrep -f '/usb_cam_node|/robot/master/main.py|/robot/master/controller_node.py' >/dev/null; then
  echo '已有摄像头或控制器正在运行。请先在原运行终端 Ctrl+C，避免重复占用摄像头，再运行此命令。' >&2
  exit 1
fi
printf '泊车：%s，%s\n' "$parking_slot" "$parking_description"
exec roslaunch robocup_competition full.launch \
  live:=true enabled:=true start_actuators:=true start_rear:=false \
  lidar_enabled:=true parking_enabled:=true wait_green:=true \
  "parking_slot:=$parking_slot" "parking_mode:=$parking_mode" "parking_entry_style:=$parking_style" \
  lane_hz:=12 lane_window_height:=40 lane_min_span:=0.15 record_lane:=false \
  lane_speed_raw:=40 action_speed_raw:=20 straight_speed_raw:=30 \
  lane_curvature_preview:=true lane_curve_speed_raw:=30 \
  startup_steering_raw:=-3 \
  lookahead:=1.0 action_lookahead:=0.30 steering_command_scale_rad:=0.03 \
  sign_backend:=yolo_trt sign_capture:=false \
  sign_model:=/home/nano/robodata/models/20260916_yolov5s/yolov5s_640_fp16.engine \
  sign_ttl:=0 blue_default_straight:=false marker_trigger_x:=0.36 \
  blue_align_duration_s:=1.0 blue_aligned_advance_m:=0.36 intersection_wait_s:=1.0 straight_distance:=1.25 \
  left_turn_full_lock:=true left_turn_entry:=0.12 left_turn_radius:=0.65 left_turn_exit:=0.30 \
  right_turn_full_lock:=true right_exit_on_blue:=true right_lock_min_angle_deg:=30 \
  right_turn_entry:=0.0 right_reverse_entry_m:=0.25 \
  right_turn_radius:=0.55 right_turn_exit:=0.25 "${launch_args[@]}"
