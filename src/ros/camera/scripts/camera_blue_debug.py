#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
from __future__ import print_function

import time
import cv2
import numpy as np
import rospy
import json

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError


# ============================================================
# ROS 话题配置
# ============================================================

# 前摄像头图像
FRONT_CAMERA_TOPIC = "/front/usb_cam/image_raw"

# 发布原图，网页端可看：/stream/blue_raw
DEBUG_RAW_TOPIC = "/debug/front_raw"

# 发布蓝线二值图，网页端可看：/stream/blue_mask
DEBUG_BLUE_MASK_TOPIC = "/debug/warped_image"
LANE_SEND_TOPIC = "/lane/send"

# 发布蓝线叠加图，网页端可看：/stream/blue_overlay
DEBUG_BLUE_OVERLAY_TOPIC = "/debug/blue_overlay"


# ============================================================
# 蓝线检测参数
# ============================================================

# 只检测地面区域
# 蓝线如果在画面比较靠前，可以设 0.35
# 如果天花板/围挡误检多，可以设 0.42
FLOOR_Y_RATIO = 0.2

# HSV 蓝色阈值
# OpenCV 里 H 范围是 0~179，不是 0~360
# 蓝色一般在 90~130 附近
BLUE_H_MIN = 90
BLUE_H_MAX = 130

# 饱和度下限
# 蓝线识别太少：降低到 50
# 误检灰色/反光太多：提高到 90 / 100
BLUE_S_MIN = 30

# 亮度下限
# 蓝线偏暗：降低到 30
# 暗噪声太多：提高到 60
BLUE_V_MIN = 15

# 亮度上限，通常保持 255
BLUE_V_MAX = 255

# 开运算：去小噪点
OPEN_KERNEL_SIZE = 3

# 闭运算：连接断裂蓝线
CLOSE_KERNEL_SIZE = 7

# 最小蓝色连通块面积
# 蓝线太碎被删：降低到 30
# 小噪点太多：提高到 100 / 200
MIN_BLUE_AREA = 60

# 最大蓝色连通块面积
# 防止大面积误检
MAX_BLUE_AREA = 80000

# 蓝色像素数量阈值
# 注意：countNonZero 最大也就 640*480=307200 左右
# 所以原来那种 80000000 是不合理的
BLUE_POINT_THRESHOLD = 1500

# 打印间隔
PRINT_INTERVAL = 0.5


# ============================================================
# 全局变量
# ============================================================

bridge = CvBridge()
raw_pub = None
blue_mask_pub = None
lane_pub = None
blue_overlay_pub = None
last_print_time = 0.0


# ============================================================
# 工具函数
# ============================================================

def publish_image(pub, img, encoding=None):
    """
    发布 OpenCV 图像到 ROS 话题。
    """
    if pub is None or img is None:
        return

    try:
        if img.dtype != np.uint8:
            img = np.clip(img, 0, 255).astype(np.uint8)

        if encoding is None:
            if len(img.shape) == 2:
                encoding = "mono8"
            else:
                encoding = "bgr8"

        msg = bridge.cv2_to_imgmsg(img, encoding=encoding)
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)

    except Exception as e:
        rospy.logwarn("publish_image failed: %s" % str(e))


def build_floor_mask(frame_bgr):
    """
    构建地面 ROI，只检测画面下方区域。
    """
    h, w = frame_bgr.shape[:2]

    y0 = int(h * FLOOR_Y_RATIO)

    mask = np.zeros((h, w), dtype=np.uint8)
    mask[y0:h, :] = 255

    return mask


def filter_blue_components(binary_img):
    """
    连通域过滤：
    去掉小噪点，保留面积合理的蓝色区域。
    """

    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        binary_img,
        connectivity=8
    )

    cleaned = np.zeros_like(binary_img)

    for i in range(1, num_labels):
        area = stats[i, cv2.CC_STAT_AREA]
        x = stats[i, cv2.CC_STAT_LEFT]
        y = stats[i, cv2.CC_STAT_TOP]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]

        if area < MIN_BLUE_AREA:
            continue

        if area > MAX_BLUE_AREA:
            continue

        if w <= 1 or h <= 1:
            continue

        cleaned[labels == i] = 255

    return cleaned


# ============================================================
# 蓝线检测主函数
# ============================================================

def detect_blue_line(frame_bgr):
    """
    蓝线检测流程：

    原图
    -> 地面 ROI
    -> HSV 蓝色阈值
    -> 开运算去噪
    -> 闭运算连接
    -> 连通域过滤
    -> 输出蓝线二值图
    """

    # 1. 地面 ROI
    roi_mask = build_floor_mask(frame_bgr)

    # 2. 转 HSV
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    # 3. 蓝色阈值
    lower_blue = np.array([BLUE_H_MIN, BLUE_S_MIN, BLUE_V_MIN], dtype=np.uint8)
    upper_blue = np.array([BLUE_H_MAX, 255, BLUE_V_MAX], dtype=np.uint8)

    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # 4. 只保留地面区域
    blue_mask = cv2.bitwise_and(blue_mask, roi_mask)

    # 5. 中值滤波，去孤立点
    blue_mask = cv2.medianBlur(blue_mask, 3)

    # 6. 开运算去噪
    open_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (OPEN_KERNEL_SIZE, OPEN_KERNEL_SIZE)
    )

    blue_mask = cv2.morphologyEx(
        blue_mask,
        cv2.MORPH_OPEN,
        open_kernel
    )

    # 7. 闭运算连接断裂区域
    close_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (CLOSE_KERNEL_SIZE, CLOSE_KERNEL_SIZE)
    )

    blue_mask = cv2.morphologyEx(
        blue_mask,
        cv2.MORPH_CLOSE,
        close_kernel
    )

    # 8. 连通域过滤
    blue_mask = filter_blue_components(blue_mask)

    return blue_mask


def make_overlay(frame_bgr, blue_mask):
    """
    生成蓝线叠加图。
    为了兼容 Python2 + OpenCV，这里不用 cv2.addWeighted。
    检测到的蓝线区域直接标成红色，方便观察。
    """
    overlay = frame_bgr.copy()

    mask_bool = blue_mask > 0

    # BGR 格式，所以红色是 (0, 0, 255)
    overlay[mask_bool] = (0, 0, 255)

    return overlay

# ============================================================
# 图像回调
# ============================================================

def image_callback(msg):
    global last_print_time

    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("cv_bridge failed: %s" % str(e))
        return

    blue_mask = detect_blue_line(frame)
    overlay = make_overlay(frame, blue_mask)

    publish_image(raw_pub, frame, encoding="bgr8")
    publish_image(blue_mask_pub, blue_mask, encoding="mono8")
    publish_image(blue_overlay_pub, overlay, encoding="bgr8")

    now = time.time()

    if now - last_print_time >= PRINT_INTERVAL:
        blue_points = cv2.countNonZero(blue_mask)
        detected = blue_points > BLUE_POINT_THRESHOLD

        print("blue_points = %d, blue_detected = %s" % (blue_points, str(detected)))
        if lane_pub is not None:
            lane_data = {
                "source": "lane",
                "blue_points": int(blue_points),
                "blue_detected": bool(detected),
                "stop_line_trigger": bool(detected)
            }
            msg = String()
            msg.data = json.dumps(lane_data)
            lane_pub.publish(msg)

        last_print_time = now


# ============================================================
# 主函数
# ============================================================

def main():
    global raw_pub
    global blue_mask_pub
    global blue_overlay_pub
    global lane_pub

    rospy.init_node("camera_blue_debug", anonymous=True)

    raw_pub = rospy.Publisher(
        DEBUG_RAW_TOPIC,
        Image,
        queue_size=1
    )

    blue_mask_pub = rospy.Publisher(
        DEBUG_BLUE_MASK_TOPIC,
        Image,
        queue_size=1
    )

    blue_overlay_pub = rospy.Publisher(
        DEBUG_BLUE_OVERLAY_TOPIC,
        Image,
        queue_size=1
    )

    lane_pub = rospy.Publisher(LANE_SEND_TOPIC, String, queue_size=1)
    rospy.Subscriber(
        FRONT_CAMERA_TOPIC,
        Image,
        image_callback,
        queue_size=1,
        buff_size=2 ** 24
    )

    print("camera_blue_debug started")
    print("subscribe: %s" % FRONT_CAMERA_TOPIC)
    print("publish raw: %s" % DEBUG_RAW_TOPIC)
    print("publish blue mask: %s" % DEBUG_BLUE_MASK_TOPIC)
    print("publish blue overlay: %s" % DEBUG_BLUE_OVERLAY_TOPIC)
    print("HSV blue range: H[%d,%d], S>=%d, V>=%d" %
          (BLUE_H_MIN, BLUE_H_MAX, BLUE_S_MIN, BLUE_V_MIN))
    print("BLUE_POINT_THRESHOLD = %d" % BLUE_POINT_THRESHOLD)

    rospy.spin()


if __name__ == "__main__":
    main()