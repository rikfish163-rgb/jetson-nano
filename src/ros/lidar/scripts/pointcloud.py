#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import LaserScan
from flask import Flask, Response
from std_msgs.msg import String
import json
import math

# ====================== 精简配置 ======================
FRAME_WIDTH = 900
FRAME_HEIGHT = 900
MAX_DISPLAY_RANGE = 2.0
SERVER_PORT = 5000

latest_frame = None
app = Flask(__name__)

# ====================== 发送者 ======================
class DataSender:
    def __init__(self, topic_name):
        self.pub = rospy.Publisher(topic_name, String, queue_size=1)

    def send(self, data):
        msg = String()
        msg.data = json.dumps(data)
        self.pub.publish(msg)

sender = DataSender("/lidar/send")

# ====================== 雷达回调 ======================
def lidar_callback(scan_data):
    global latest_frame
    img = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
    cx = FRAME_WIDTH // 2
    cy = FRAME_HEIGHT // 2

    angles = np.linspace(scan_data.angle_min, scan_data.angle_max, len(scan_data.ranges))

    min_dist = 999.0
    closest_angle = 0.0
    closest_pixel = (cx, cy)

    for i, r in enumerate(scan_data.ranges):
        if r < 0.05 or r > MAX_DISPLAY_RANGE:
            continue
        angle = angles[i]
        x_pix = cx + int((r / MAX_DISPLAY_RANGE) * cx * np.cos(angle))
        y_pix = cy - int((r / MAX_DISPLAY_RANGE) * cy * np.sin(angle))
        cv2.circle(img, (x_pix, y_pix), 1, (0, 0, 255), -1)

        if r < min_dist:
            min_dist = r
            closest_angle = angle
            closest_pixel = (x_pix, y_pix)

    # 小车
    left_ratio   = 0.30 / MAX_DISPLAY_RANGE
    right_ratio  = 0.18 / MAX_DISPLAY_RANGE
    vert_ratio   = 0.18 / MAX_DISPLAY_RANGE
    car_left   = cx - int(left_ratio * cx)
    car_right  = cx + int(right_ratio * cx)
    car_top    = cy - int(vert_ratio * cy)
    car_bottom = cy + int(vert_ratio * cy)
    cv2.rectangle(img, (car_left, car_top), (car_right, car_bottom), (0,255,255), 2)

    # 最近点
    if min_dist < 999.0:
        cv2.line(img, (cx, cy), closest_pixel, (0,255,0), 2)
        cv2.putText(img, "%.3f m" % min_dist, (closest_pixel[0]+10, closest_pixel[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

    cv2.circle(img, (cx, cy), 3, (0,255,255), -1)

    latest_frame = img.copy()

    # 发送消息
    msg_data = {
        "angle": round(math.degrees(closest_angle), 2),
        "distance": round(min_dist, 3)
    }
    sender.send(msg_data)

# ====================== 视频流 ======================
def gen():
    while not rospy.is_shutdown():
        if latest_frame is None:
            continue
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 60]
        ret, jpg = cv2.imencode('.jpg', latest_frame, encode_param)
        yield b'--frame\r\nContent-Type:image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n'

@app.route('/video_feed')
def video_feed():
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    rospy.init_node('lidar_only_points')
    rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)
    app.run(host='0.0.0.0', port=SERVER_PORT, debug=False, threaded=True, use_reloader=False)