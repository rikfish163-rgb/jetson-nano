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

# 前摄像头图像话题
FRONT_CAMERA_TOPIC = "/rear/usb_cam/image_raw"

# 原图发布话题，网页端看这个：/stream/front_raw
DEBUG_FRONT_TOPIC = "/debug/rear_raw"

# 白线检测结果发布话题，网页端看这个：/stream/warped_image
DEBUG_WHITE_TOPIC = "/debug/rear_warped_image"
LANE_SEND_TOPIC = "/vision/rear_lane"


# ============================================================
# 当前版本目标
# ============================================================
# 1. 优先保证左右两侧白线都能显示出来
# 2. 不再单纯依赖 WHITE_L_MIN
# 3. 用 HLS + BGR + 局部对比度综合判断白线
# 4. 尽量减少地面灰色反光误检


# ============================================================
# 主要调试参数
# ============================================================

# 只检测图像下方地面区域
# 数值越大，检测区域越靠下
FLOOR_Y_RATIO = 0.48

# HLS 亮度阈值
# 不要再调到 80 那么低，否则地面会大面积进来
# 白线太少：降到 115 / 110
# 反光太多：升到 130 / 140
WHITE_L_MIN = 130

# HLS 饱和度上限
# 白色通常饱和度较低
# 白线太少：升到 190 / 200
# 灰色地面太多：降到 150 / 140
WHITE_S_MAX = 130

# BGR 三通道最低亮度
# 三个通道都要比较亮，才更像真正白色
# 右侧白线少：降到 135
# 反光/灰地太多：升到 155 / 165
WHITE_RGB_MIN = 168

# BGR 三通道最大差值
# 白色的 B/G/R 差距不应太大
# 太严格会漏白线，太宽松会进反光
WHITE_RGB_DELTA_MAX = 70

# 局部对比度阈值
# 白线应该比周围地面更亮
# 白线太少：降到 5
# 反光太多：升到 12 / 15
LOCAL_CONTRAST_MIN = 18

# 强白线亮度阈值
# 如果某些区域本身非常亮，即使局部对比度不强，也允许保留
STRONG_L_MIN = 195

# 开运算：去小噪点
OPEN_KERNEL_SIZE = 3

# 闭运算：连接断裂白线
# 白线断裂：CLOSE_KERNEL_W 增大到 9 / 11
# 反光连成大片：CLOSE_KERNEL_W 减小到 5 / 7
CLOSE_KERNEL_W = 7
CLOSE_KERNEL_H = 3

# 最小连通块面积
# 远处虚线被删：减小到 10
# 小噪点太多：增大到 25 / 40
MIN_COMPONENT_AREA = 12

# 最大连通块面积
# 大块反光太多：减小到 15000 / 20000
# 大弯道边线被删：增大到 40000
MAX_COMPONENT_AREA = 30000

# 小碎片填充率过滤
MIN_FILL_RATIO = 0.08

# 是否过滤图像中间偏下的竖向反光
# 如果中间真实白线被删，可以改成 False
FILTER_CENTER_VERTICAL_REFLECTION = True

# 终端打印间隔
PRINT_INTERVAL = 0.5


# ============================================================
# 全局变量
# ============================================================

bridge = CvBridge()
front_pub = None
white_pub = None
lane_pub = None
last_print_time = 0.0


# ============================================================
# ROS 图像发布函数
# ============================================================

def publish_image(pub, img, encoding=None):
    """
    将 OpenCV 图像发布为 ROS Image。
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


# ============================================================
# 构建地面 ROI
# ============================================================

def build_floor_mask(frame_bgr):
    """
    只保留图像下方地面区域。
    避免天花板、墙面、围挡进入检测。
    """
    h, w = frame_bgr.shape[:2]

    y0 = int(h * FLOOR_Y_RATIO)

    mask = np.zeros((h, w), dtype=np.uint8)
    mask[y0:h, :] = 255

    return mask


# ============================================================
# 连通域过滤函数
# ============================================================

def filter_white_components(binary_img):
    """
    连通域过滤。

    重点：
    1. 不能全局删除竖向区域，否则右侧白线会被误删；
    2. 只过滤中间偏下的竖向反光；
    3. 左右两侧尽量保留，因为白线经常出现在左右两侧；
    4. 过滤面积过大、很稀碎的小噪声。
    """

    img_h, img_w = binary_img.shape[:2]

    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        binary_img,
        connectivity=8
    )

    cleaned = np.zeros_like(binary_img)

    for i in range(1, num_labels):
        x = stats[i, cv2.CC_STAT_LEFT]
        y = stats[i, cv2.CC_STAT_TOP]
        bw = stats[i, cv2.CC_STAT_WIDTH]
        bh = stats[i, cv2.CC_STAT_HEIGHT]
        area = stats[i, cv2.CC_STAT_AREA]

        if area < MIN_COMPONENT_AREA:
            continue

        if bw <= 1 or bh <= 1:
            continue

        bbox_area = float(bw * bh)
        fill_ratio = float(area) / bbox_area

        cx = x + bw / 2.0
        cy = y + bh / 2.0

        in_left_side = cx < img_w * 0.35
        in_center = (cx >= img_w * 0.35 and cx <= img_w * 0.65)
        in_right_side = cx > img_w * 0.65

        # 面积过大，一般是整片反光或地面被误识别
        if area > MAX_COMPONENT_AREA:
            continue

        # 小而稀碎的噪点删掉
        if area < 80 and fill_ratio < MIN_FILL_RATIO:
            continue

        # 只过滤中间偏下的竖向反光
        # 不过滤左右两侧，否则右侧白线会消失
        if FILTER_CENTER_VERTICAL_REFLECTION:
            if in_center and cy > img_h * 0.45:
                if bh > 35 and bh > bw * 1.7:
                    continue

        # 左右两侧尽量保留
        # 因为左右两侧的竖向/斜向白色区域很可能就是白线
        if in_left_side or in_right_side:
            cleaned[labels == i] = 255
            continue

        # 中间区域做更保守的过滤
        if in_center:
            # 中间特别细长的小线段，多数是地面反光/划痕
            if area < 120 and bh > bw * 3.0:
                continue

            # 中间非常稀碎的纹理
            if fill_ratio < 0.08 and area < 300:
                continue

        cleaned[labels == i] = 255

    return cleaned


# ============================================================
# 白线检测主函数
# ============================================================

def detect_white_line(frame_bgr):
    """
    白线检测流程：

    原图
    -> 地面 ROI
    -> HLS 白色筛选
    -> BGR 真白色筛选
    -> 局部对比度筛选
    -> 形态学处理
    -> 连通域过滤
    -> 输出白线二值图
    """

    # 1. 地面 ROI
    roi_mask = build_floor_mask(frame_bgr)

    # 2. HLS 白色粗筛
    hls = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HLS)
    h_channel, l_channel, s_channel = cv2.split(hls)

    mask_l = cv2.inRange(l_channel, WHITE_L_MIN, 255)
    mask_s = cv2.inRange(s_channel, 0, WHITE_S_MAX)
    mask_hls = cv2.bitwise_and(mask_l, mask_s)

    # 3. BGR 真白色判断
    b_channel, g_channel, r_channel = cv2.split(frame_bgr)

    max_rgb = cv2.max(cv2.max(r_channel, g_channel), b_channel)
    min_rgb = cv2.min(cv2.min(r_channel, g_channel), b_channel)

    delta_rgb = cv2.subtract(max_rgb, min_rgb)

    # 三个通道都比较亮
    mask_rgb_bright = cv2.inRange(min_rgb, WHITE_RGB_MIN, 255)

    # 三个通道差距不能太大
    mask_rgb_neutral = cv2.inRange(delta_rgb, 0, WHITE_RGB_DELTA_MAX)

    mask_rgb_white = cv2.bitwise_and(mask_rgb_bright, mask_rgb_neutral)

    # 4. 局部对比度判断
    # 白线应该比附近地面亮，而不是整片地面一起亮
    l_blur = cv2.GaussianBlur(l_channel, (31, 31), 0)
    local_contrast = cv2.subtract(l_channel, l_blur)

    mask_contrast = cv2.inRange(local_contrast, LOCAL_CONTRAST_MIN, 255)

    # 5. 强白线判断
    # 对非常亮的白线放宽局部对比度限制
    mask_strong_l = cv2.inRange(l_channel, STRONG_L_MIN, 255)

    # 6. 综合白线判断
    # 条件 A：HLS 像白色 + RGB 像白色 + 比周围亮
    mask_normal_white = cv2.bitwise_and(mask_hls, mask_rgb_white)
    mask_normal_white = cv2.bitwise_and(mask_normal_white, mask_contrast)

    # 条件 B：RGB 像白色 + 亮度非常高
    mask_strong_white = cv2.bitwise_and(mask_rgb_white, mask_strong_l)

    white_mask = cv2.bitwise_or(mask_normal_white, mask_strong_white)

    # 7. 只保留地面区域
    white_mask = cv2.bitwise_and(white_mask, roi_mask)

    # 8. 中值滤波，减少孤立噪点
    white_mask = cv2.medianBlur(white_mask, 3)

    # 9. 开运算：去小白点
    open_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (OPEN_KERNEL_SIZE, OPEN_KERNEL_SIZE)
    )

    white_mask = cv2.morphologyEx(
        white_mask,
        cv2.MORPH_OPEN,
        open_kernel
    )

    # 10. 闭运算：连接断裂虚线/边线
    close_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (CLOSE_KERNEL_W, CLOSE_KERNEL_H)
    )

    white_mask = cv2.morphologyEx(
        white_mask,
        cv2.MORPH_CLOSE,
        close_kernel
    )

    # 11. 连通域过滤
    white_mask = filter_white_components(white_mask)

    # 12. 再轻微闭运算一次，让白线块更完整
    white_mask = cv2.morphologyEx(
        white_mask,
        cv2.MORPH_CLOSE,
        close_kernel
    )

    return white_mask


# ============================================================
# 车道中心/分层 offset 计算
# ============================================================

def calc_lane_offset_in_band(white_mask, y0_ratio, y1_ratio):
    """
    在指定纵向区域内计算车道中心和 offset。
    y0_ratio/y1_ratio: 0~1，越大越靠近车身。
    返回 dict，供直道和弯道共同使用。
    """
    h, w = white_mask.shape[:2]
    y0 = int(h * y0_ratio)
    y1 = int(h * y1_ratio)

    if y1 <= y0:
        return {
            "offset": 9999,
            "lane_center_x": -1,
            "left_x": -1,
            "right_x": -1,
            "white_sum": 0
        }

    roi = white_mask[y0:y1, :]
    hist = cv2.reduce(roi, 0, cv2.REDUCE_SUM, dtype=cv2.CV_32S).flatten()

    mid_x = int(w / 2)
    left_hist = hist[:mid_x]
    right_hist = hist[mid_x:]

    left_sum = int(left_hist.sum())
    right_sum = int(right_hist.sum())
    white_sum = left_sum + right_sum

    # 注意：这里的 sum 是 255 累加值，不是像素个数。
    # 2500 大约等价于 10 个白色像素，适合分层 ROI。
    MIN_SIDE_SUM = 2500

    left_x = -1
    right_x = -1

    if left_sum > MIN_SIDE_SUM:
        left_x = int((left_hist * np.arange(0, mid_x)).sum() / float(left_sum))

    if right_sum > MIN_SIDE_SUM:
        right_x = int((right_hist * np.arange(mid_x, w)).sum() / float(right_sum))

    LANE_HALF_WIDTH_PIXELS = 120

    if left_x >= 0 and right_x >= 0:
        lane_center_x = int((left_x + right_x) / 2)
    elif left_x >= 0:
        lane_center_x = int(left_x + LANE_HALF_WIDTH_PIXELS)
    elif right_x >= 0:
        lane_center_x = int(right_x - LANE_HALF_WIDTH_PIXELS)
    else:
        lane_center_x = -1

    image_center_x = int(w / 2)

    if lane_center_x >= 0:
        OFFSET_BIAS = 17
        raw_offset_error = int(lane_center_x - image_center_x)
        offset = int(raw_offset_error - OFFSET_BIAS)
    else:
        offset = 9999

    return {
        "offset": int(offset),
        "lane_center_x": int(lane_center_x),
        "left_x": int(left_x),
        "right_x": int(right_x),
        "white_sum": int(white_sum)
    }


def judge_curve(offset_near, offset_mid):
    """
    根据近处 offset 和中距离 offset 的差值，给出弯道趋势。
    只作为视觉提示，不直接控制车辆。
    """
    if offset_near == 9999 or offset_mid == 9999:
        return "UNKNOWN"

    diff = int(offset_mid - offset_near)
    CURVE_THRESHOLD = 25

    if diff > CURVE_THRESHOLD:
        return "RIGHT_CURVE"
    elif diff < -CURVE_THRESHOLD:
        return "LEFT_CURVE"
    else:
        return "STRAIGHT"


# ============================================================
# 图像回调函数
# ============================================================

def image_callback(msg):
    global last_print_time

    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("cv_bridge failed: %s" % str(e))
        return

    white_mask = detect_white_line(frame)

    publish_image(front_pub, frame, encoding="bgr8")
    publish_image(white_pub, white_mask, encoding="mono8")

    now = time.time()

    if now - last_print_time >= PRINT_INTERVAL:
        white_points = cv2.countNonZero(white_mask)

        # near: 近处车道中心，最适合判断当前是否压线/偏离
        # mid:  中距离车道中心，适合观察入弯趋势
        # old:  兼容旧版本控制组，保持 offset_error 的意义不突然改变
        near_info = calc_lane_offset_in_band(white_mask, 0.78, 0.95)
        mid_info = calc_lane_offset_in_band(white_mask, 0.60, 0.78)
        old_info = calc_lane_offset_in_band(white_mask, 0.60, 0.95)

        offset_error = old_info["offset"]
        offset_near = near_info["offset"]
        offset_mid = mid_info["offset"]
        curve_hint = judge_curve(offset_near, offset_mid)

        # 粗略反光提示：白色面积异常偏大时，提示可能有地面反光。
        # 这个阈值先保守设置，后续根据现场日志再调。
        reflection_warning = bool(white_points > 45000)

        print("OLD  left_x = %d, right_x = %d, lane_center_x = %d" % (
            old_info["left_x"], old_info["right_x"], old_info["lane_center_x"]
        ))
        print("NEAR offset = %d, MID offset = %d, curve_hint = %s" % (
            offset_near, offset_mid, curve_hint
        ))
        print("white_points = %d, reflection_warning = %s" % (
            white_points, str(reflection_warning)
        ))
        print("offset_error = %d" % offset_error)

        if lane_pub is not None:
            # 原来 /5000 会导致置信度长期等于 1.0，改成更合理的量级。
            lane_confidence = min(1.0, float(white_points) / 35000.0)
            lane_data = {
                "source": "lane",
                "white_points": int(white_points),
                "lane_confidence": float(lane_confidence),
                "offset_error": int(offset_error),
                "offset_near": int(offset_near),
                "offset_mid": int(offset_mid),
                "curve_hint": curve_hint,
                "reflection_warning": reflection_warning,
                "left_x": int(old_info["left_x"]),
                "right_x": int(old_info["right_x"]),
                "lane_center_x": int(old_info["lane_center_x"])
            }
            msg = String()
            msg.data = json.dumps(lane_data)
            lane_pub.publish(msg)
        last_print_time = now


# ============================================================
# 主函数
# ============================================================

def main():
    global FRONT_CAMERA_TOPIC
    global DEBUG_FRONT_TOPIC
    global DEBUG_WHITE_TOPIC
    global LANE_SEND_TOPIC
    global front_pub
    global white_pub
    global lane_pub

    rospy.init_node("camera_yihan_white_only", anonymous=True)
    # Keep the old defaults for standalone use, but follow the parking
    # launch's camera topic when a replay or alternate USB mapping is used.
    FRONT_CAMERA_TOPIC = rospy.get_param(
        "~image_topic", FRONT_CAMERA_TOPIC)
    DEBUG_FRONT_TOPIC = rospy.get_param(
        "~raw_topic", DEBUG_FRONT_TOPIC)
    DEBUG_WHITE_TOPIC = rospy.get_param(
        "~white_topic", DEBUG_WHITE_TOPIC)
    LANE_SEND_TOPIC = rospy.get_param(
        "~lane_topic", LANE_SEND_TOPIC)

    front_pub = rospy.Publisher(
        DEBUG_FRONT_TOPIC,
        Image,
        queue_size=1
    )

    white_pub = rospy.Publisher(
        DEBUG_WHITE_TOPIC,
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

    print("camera_yihan_white_only started")
    print("subscribe: %s" % FRONT_CAMERA_TOPIC)
    print("publish raw: %s" % DEBUG_FRONT_TOPIC)
    print("publish white mask: %s" % DEBUG_WHITE_TOPIC)
    print("WHITE_L_MIN = %d" % WHITE_L_MIN)
    print("WHITE_S_MAX = %d" % WHITE_S_MAX)
    print("WHITE_RGB_MIN = %d" % WHITE_RGB_MIN)
    print("WHITE_RGB_DELTA_MAX = %d" % WHITE_RGB_DELTA_MAX)
    print("LOCAL_CONTRAST_MIN = %d" % LOCAL_CONTRAST_MIN)
    print("STRONG_L_MIN = %d" % STRONG_L_MIN)
    print("CLOSE_KERNEL = %d x %d" % (CLOSE_KERNEL_W, CLOSE_KERNEL_H))
    print("FILTER_CENTER_VERTICAL_REFLECTION = %s" % str(FILTER_CENTER_VERTICAL_REFLECTION))

    rospy.spin()


if __name__ == "__main__":
    main()
