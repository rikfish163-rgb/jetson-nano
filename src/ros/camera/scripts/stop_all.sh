#!/bin/bash

pkill -f camera_blue_debug.py
pkill -f camera_yihan_web.py
pkill -f lane_receiver_node.py
pkill -f bridge_node.py
pkill -f ros_flask_viewer
pkill -f usb_cam_node
pkill -f base_controller
pkill -f roscore
pkill -f rosmaster

echo "All smartcar debug processes stopped."
