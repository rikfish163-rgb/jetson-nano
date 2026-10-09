#!/usr/bin/env bash
set -e
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
exec roslaunch robocup_competition full.launch \
  live:=true enabled:=true start_actuators:=true start_rear:=false \
  lidar_enabled:=true parking_enabled:=true parking_mode:=forward_plan wait_green:=true \
  lane_hz:=12 lane_window_height:=40 lane_min_span:=0.15 record_lane:=true \
  lane_speed_raw:=30 action_speed_raw:=20 straight_speed_raw:=20 \
  lane_curvature_preview:=true lane_curve_speed_raw:=30 \
  startup_steering_raw:=0 \
  lookahead:=1.0 action_lookahead:=0.30 steering_command_scale_rad:=0.03 \
  sign_backend:=yolo_trt \
  sign_model:=/home/nano/robodata/models/20260916_yolov5s/yolov5s_640_fp16.engine \
  sign_ttl:=0 blue_default_straight:=false marker_trigger_x:=0.36 \
  blue_align_duration_s:=1.0 blue_aligned_advance_m:=0.36 intersection_wait_s:=1.0 straight_distance:=1.25 \
  left_turn_full_lock:=false left_turn_entry:=0.12 left_turn_radius:=0.65 left_turn_exit:=0.30 \
  right_turn_full_lock:=true right_exit_on_blue:=true right_lock_min_angle_deg:=30 \
  right_turn_entry:=0.0 right_reverse_entry_m:=0.25 \
  right_turn_radius:=0.55 right_turn_exit:=0.25 "$@"
