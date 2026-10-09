#!/bin/bash

WORKSPACE=/home/nano/robocup_ws
SCRIPT_DIR=/home/nano/robocup_ws/src/ros/camera/scripts

source $WORKSPACE/devel/setup.bash
cd $SCRIPT_DIR

rm -f /tmp/roscore.log
rm -f /tmp/usb_cam.log
rm -f /tmp/bridge_node.log
rm -f /tmp/camera_yihan_web.log
rm -f /tmp/lane_receiver_node.log

echo "Starting white lane debug..."

nohup bash -c "source $WORKSPACE/devel/setup.bash; roscore" > /tmp/roscore.log 2>&1 &
sleep 4

nohup bash -c "source $WORKSPACE/devel/setup.bash; rosrun usb_cam usb_cam_node __ns:=front _video_device:=/dev/video0 _image_width:=640 _image_height:=360 _framerate:=30 _pixel_format:=mjpeg" > /tmp/usb_cam.log 2>&1 &
sleep 3

nohup bash -c "source $WORKSPACE/devel/setup.bash; cd $SCRIPT_DIR; python2 bridge_node.py" > /tmp/bridge_node.log 2>&1 &
sleep 1

nohup bash -c "source $WORKSPACE/devel/setup.bash; cd $SCRIPT_DIR; python2 camera_yihan_web.py" > /tmp/camera_yihan_web.log 2>&1 &
sleep 1

nohup bash -c "source $WORKSPACE/devel/setup.bash; cd $SCRIPT_DIR; python2 lane_receiver_node.py" > /tmp/lane_receiver_node.log 2>&1 &

echo "White debug started."
echo "Use: tail -f /tmp/lane_receiver_node.log"
echo "Use: tail -f /tmp/camera_yihan_web.log"
