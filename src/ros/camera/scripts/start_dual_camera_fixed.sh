#!/bin/bash

WORKSPACE=/home/nano/robocup_ws
SCRIPT_DIR=/home/nano/robocup_ws/src/ros/camera/scripts

# Stable physical USB paths observed on the current Jetson.  Override with
# PARKING_FRONT_DEV/PARKING_REAR_DEV after a deliberate camera swap.
FRONT_DEV="${PARKING_FRONT_DEV:-/dev/v4l/by-path/platform-70090000.xusb-usb-0:2.3:1.0-video-index0}"
REAR_DEV="${PARKING_REAR_DEV:-/dev/v4l/by-path/platform-70090000.xusb-usb-0:2.4:1.0-video-index0}"
FRONT_CALIB=/home/nano/robocup_ws/src/ros/camera/calibration/front_640x360.yaml
REAR_CALIB=/home/nano/robocup_ws/src/ros/camera/calibration/rear_640x480.yaml

source $WORKSPACE/devel/setup.bash
cd $SCRIPT_DIR

echo "========== Stop old processes =========="
pkill -f usb_cam_node
pkill -f camera_yihan_web.py
pkill -f rear_camera_lane_web.py
pkill -f lane_fusion_node.py
pkill -f ros_flask_viewer.py
# Do not kill msg_bridge's universal_bridge_node: main.launch owns the
# /camera_hts/send -> /camera_hts/receive and /control/send -> /control/receive
# channels.  Only stop the legacy yihan bridge if it was started separately.
pkill -f "$SCRIPT_DIR/bridge_node.py"
sleep 1

echo "========== Start front camera: $FRONT_DEV =========="
nohup bash -c "source $WORKSPACE/devel/setup.bash; rosrun usb_cam usb_cam_node __ns:=front _video_device:=$FRONT_DEV _image_width:=640 _image_height:=360 _pixel_format:=mjpeg _framerate:=30 _camera_frame_id:=front_camera _camera_name:=front_camera _camera_info_url:=file://$FRONT_CALIB" > /tmp/front_usb_cam.log 2>&1 &
sleep 2

echo "========== Start rear camera: $REAR_DEV =========="
nohup bash -c "source $WORKSPACE/devel/setup.bash; rosrun usb_cam usb_cam_node __ns:=rear _video_device:=$REAR_DEV _image_width:=640 _image_height:=480 _pixel_format:=mjpeg _framerate:=30 _camera_frame_id:=rear_camera _camera_name:=rear_camera _camera_info_url:=file://$REAR_CALIB" > /tmp/rear_usb_cam.log 2>&1 &
sleep 2

echo "========== Start front lane node =========="
nohup bash -c "source $WORKSPACE/devel/setup.bash; cd $SCRIPT_DIR; python2 camera_yihan_web.py" > /tmp/front_lane.log 2>&1 &
sleep 1

echo "========== Start rear lane node =========="
nohup bash -c "source $WORKSPACE/devel/setup.bash; cd $SCRIPT_DIR; python2 rear_camera_lane_web.py" > /tmp/rear_lane.log 2>&1 &
sleep 1

echo "========== Start fusion node =========="
nohup bash -c "source $WORKSPACE/devel/setup.bash; cd $SCRIPT_DIR; python2 lane_fusion_node.py" > /tmp/lane_fusion.log 2>&1 &
sleep 1

echo "========== Start web viewer =========="
nohup bash -c "source $WORKSPACE/devel/setup.bash; cd $SCRIPT_DIR; python2 ros_flask_viewer.py" > /tmp/ros_flask_viewer.log 2>&1 &

echo "========== Done =========="
echo "Open viewer: http://192.168.1.103:8080/"
echo "Check topics: rostopic list | grep -E 'front|rear|vision|lane|debug'"
echo "Check fused output: rostopic echo /lane/send"
echo "Check fusion log: tail -f /tmp/lane_fusion.log"
