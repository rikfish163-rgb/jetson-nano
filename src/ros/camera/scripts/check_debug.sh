#!/bin/bash

source /home/nano/robocup_ws/devel/setup.bash

echo "========== Process Check =========="
ps -ef | grep -E "roscore|usb_cam|camera_yihan|camera_blue|bridge_node|lane_receiver" | grep -v grep

echo ""
echo "========== Topic Check =========="
rostopic list | grep -E "/front/usb_cam/image_raw|/lane/send|/all/receive|/debug/front_raw|/debug/warped_image"

echo ""
echo "========== /lane/send one message =========="
timeout 3 rostopic echo -n 1 /lane/send

echo ""
echo "========== /all/receive one message =========="
timeout 3 rostopic echo -n 1 /all/receive

echo ""
echo "========== Latest receiver log =========="
tail -n 10 /tmp/lane_receiver_node.log

echo ""
echo "========== Check Done =========="
