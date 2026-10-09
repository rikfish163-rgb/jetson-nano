#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import LaserScan
from flask import Flask, Response
import threading

# ====================== 低延迟配置 ======================
FRAME_WIDTH = 400
FRAME_HEIGHT = 400
MAX_DISPLAY_RANGE = 1.0
SAFE_DISTANCE = 0.05
SERVER_PORT = 5000

CLUSTER_EPS = 0.2
CLUSTER_MIN_POINTS = 5
MIN_CLUSTER_SIZE = 6

# 全局最新帧（低延迟关键）
latest_frame = None
frame_lock = threading.Lock()

app = Flask(__name__)

# ====================== 雷达回调 ======================
def lidar_callback(scan_data):
    global latest_frame
    img = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
    cx = FRAME_WIDTH // 2
    cy = FRAME_HEIGHT // 2

    valid_points = []
    valid_pixel_points = []

    angles = np.linspace(scan_data.angle_min, scan_data.angle_max, len(scan_data.ranges))

    for i, r in enumerate(scan_data.ranges):
        if r < 0.1 or r > MAX_DISPLAY_RANGE:
            continue

        angle = angles[i]
        x_world = r * np.cos(angle)
        y_world = r * np.sin(angle)

        x_pix = cx + int((r / MAX_DISPLAY_RANGE) * cx * np.cos(angle))
        y_pix = cy - int((r / MAX_DISPLAY_RANGE) * cy * np.sin(angle))

        valid_points.append([x_world, y_world])
        valid_pixel_points.append((x_pix, y_pix))

    # DBSCAN
    cluster_labels = np.array([])
    if len(valid_points) >= CLUSTER_MIN_POINTS:
        db = DBSCAN(eps=CLUSTER_EPS, min_samples=CLUSTER_MIN_POINTS, algorithm="ball_tree", metric="euclidean")
        cluster_labels = db.fit_predict(np.array(valid_points))

    # 画点
    for idx, (x_p, y_p) in enumerate(valid_pixel_points):
        if len(cluster_labels) > 0 and cluster_labels[idx] == -1:
            continue
        cv2.circle(img, (x_p, y_p), 1, (0, 0, 255), -1)

    # 安全点
    safe_world = []
    safe_pixel = []
    for idx, (xw, yw) in enumerate(valid_points):
        if len(cluster_labels) > 0 and cluster_labels[idx] == -1:
            continue
        d = np.sqrt(xw**2 + yw**2)
        if d < SAFE_DISTANCE:
            continue
        ratio = (d - SAFE_DISTANCE) / d
        sx = xw * ratio
        sy = yw * ratio
        sd = np.sqrt(sx**2 + sy**2)
        spx = cx + int((sd / MAX_DISPLAY_RANGE) * cx * (sx/sd))
        spy = cy - int((sd / MAX_DISPLAY_RANGE) * cy * (sy/sd))
        safe_world.append([sx, sy])
        safe_pixel.append([spx, spy])

    for (x, y) in safe_pixel:
        cv2.circle(img, (x, y), 1, (0, 255, 0), -1)

    # 最近点
    if len(safe_world) > 0:
        dists = [np.sqrt(x**2 + y**2) for x, y in safe_world]
        idx = np.argmin(dists)
        mx, my = safe_pixel[idx]
        cv2.line(img, (cx, cy), (mx, my), (0,255,0), 1)
        cv2.putText(img, "%.2f" % dists[idx], (mx+10, my-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)

    cv2.circle(img, (cx, cy), 3, (0,0,255), -1)

    # 只保存最新一帧（不堆积）
    with frame_lock:
        latest_frame = img.copy()

# ====================== 超低延迟推流 ======================
def gen():
    while not rospy.is_shutdown():
        frame = None
        with frame_lock:
            frame = latest_frame

        if frame is None:
            continue

        # 低延迟编码关键：质量调低、速度最快
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
        ret, jpg = cv2.imencode('.jpg', frame, encode_param)
        if not ret:
            continue

        yield b'--frame\r\nContent-Type:image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n'

@app.route('/video_feed')
def video_feed():
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    rospy.init_node('lidar_low_latency')
    rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)  # 队列=1，低延迟
    app.run(host='0.0.0.0', port=SERVER_PORT, debug=False, threaded=True, use_reloader=False)