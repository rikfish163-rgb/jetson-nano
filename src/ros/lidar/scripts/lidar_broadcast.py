#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import LaserScan
from flask import Flask, Response

# ====================== ���ò��� ======================
FRAME_WIDTH = 600
FRAME_HEIGHT = 600
MAX_DISPLAY_RANGE = 1.0  # �״������ʾ���루�ף�
SERVER_PORT = 5000       # ��ҳ���ʶ˿�

app = Flask(__name__)
frame = None

def lidar_callback(scan_data):
    global frame
    # ������ɫ����ͼ
    img = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
    cx = FRAME_WIDTH // 2
    cy = FRAME_HEIGHT // 2

    # ����ÿ����ĽǶȣ�����LS01B�ĽǶȷ�Χ��
    angles = np.linspace(
        scan_data.angle_min,
        scan_data.angle_max,
        len(scan_data.ranges)
    )

    # �����״��
    for i, r in enumerate(scan_data.ranges):
        # ������Ч����
        if r < scan_data.range_min or r > MAX_DISPLAY_RANGE:
            continue

        angle = angles[i]
        # ������ת�������꣨ROS��������ϵ��
        x = cx + int((r / MAX_DISPLAY_RANGE) * cx * np.cos(angle))
        y = cy - int((r / MAX_DISPLAY_RANGE) * cy * np.sin(angle))
        cv2.circle(img, (x, y), 2, (0, 255, 0), -1)

    # �������ĵ㣨��ɫ��
    cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
    # ���Ʋο�Ȧ����ɫ��1.25m/2.5m/3.75m��
    cv2.circle(img, (cx, cy), int(cx * 0.25), (80, 80, 80), 1)
    cv2.circle(img, (cx, cy), int(cx * 0.5), (80, 80, 80), 1)
    cv2.circle(img, (cx, cy), int(cx * 0.75), (80, 80, 80), 1)

    frame = img

def generate_frames():
    global frame
    while not rospy.is_shutdown():
        if frame is None:
            continue
        # ����ΪJPG����
        ret, jpeg = cv2.imencode('.jpg', frame)
        if not ret:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n'
               + jpeg.tobytes()
               + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    try:
        # ��ʼ��ROS�ڵ�
        rospy.init_node('lidar_live_stream_node', anonymous=True)
        # ���ļ����״ﻰ�⣨LS01BĬ�Ϸ���/scan��
        rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=5)
        rospy.loginfo("? �����״������ڵ������ɹ����ȴ��״�����...")
        # ����Flask��������0.0.0.0���������������豸����
        app.run(host='0.0.0.0', port=SERVER_PORT, debug=False, threaded=True)
    except rospy.ROSInterruptException:
        rospy.logerr("? �ڵ㱻�ж�")
    except Exception as e:
        rospy.logerr("? �ڵ��쳣: %s", str(e))