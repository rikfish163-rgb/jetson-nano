#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
from __future__ import print_function

import time
import cv2
import numpy as np
import rospy
import json
import threading
import hashlib
from collections import deque

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge, CvBridgeError

cv2.setNumThreads(1)


# ============================================================
# ROS 话题配置
# ============================================================

# 前摄像头图像话题
FRONT_CAMERA_TOPIC = "/front/usb_cam/image_raw"

# 原图发布话题，网页端看这个：/stream/front_raw
DEBUG_FRONT_TOPIC = "/debug/front_raw"

# 白线检测结果发布话题，网页端看这个：/stream/warped_image
DEBUG_WHITE_TOPIC = "/debug/warped_image"
DEBUG_METRIC_BEV_TOPIC = "/debug/metric_bev"
DEBUG_LANE_TRACKING_TOPIC = "/debug/lane_tracking"
LANE_SEND_TOPIC = "/vision/front_lane"
LANE_PATH_TOPIC = "/vision/lane_path"
LANE_CONFIDENCE_TOPIC = "/vision/lane_confidence"
LANE_PATH_FRAME_ID = "base_link"


# ============================================================
# 固定相机标定与 Metric BEV 参数
# ============================================================

CAMERA_SIZE = (640, 360)
BEV_SIZE = (480, 400)

K = np.array([
    [338.724360, 0.0, 342.299347],
    [0.0, 338.759483, 169.956657],
    [0.0, 0.0, 1.0]
], dtype=np.float64)

D = np.array([
    -0.313909,
     0.080347,
     0.001009,
    -0.000759,
     0.0
], dtype=np.float64)

H = np.array([
    [-0.455307634051, -1.851847127629, 416.759846959926],
    [ 0.072741868805, -3.467660494260, 575.376931923200],
    [ 0.000124867696, -0.007523542869,   1.000000000000]
], dtype=np.float64)


# ============================================================
# 自适应 Sliding Window 调试参数
# ============================================================

BEV_WIDTH = 480
BEV_HEIGHT = 400
PX_PER_CM = 4.0

NUM_WINDOWS = 10
WINDOW_HEIGHT = 40

WINDOW_HALF_WIDTH = 36
WINDOW_HALF_WIDTH_LOST = 56

LANE_WIDTH_NOMINAL_CM = 60.5
LANE_WIDTH_NOMINAL_PX = 242.0
LANE_WIDTH_MIN_CM = 59.0
LANE_WIDTH_MAX_CM = 62.0
LANE_WIDTH_MIN_PX = LANE_WIDTH_MIN_CM * PX_PER_CM
LANE_WIDTH_MAX_PX = LANE_WIDTH_MAX_CM * PX_PER_CM
LANE_WIDTH_EMA_ALPHA = 0.08

CONTROL_Y_MIN = 120
CONTROL_Y_MAX = 400

MIN_LANE_PATH_POINTS = 2
MIN_LANE_PATH_FORWARD_SPAN_M = 0.15

MAX_EMPTY_WINDOWS = 2
MIN_WINDOW_PIXELS = 12

EMA_ALPHA = 0.45
LANE_WIDTH_HISTORY_SIZE = 15

# 轻量峰值检测与局部趋势约束。
INITIAL_HIST_HEIGHT = 80
INITIAL_SEARCH_HALF_WIDTH = 72
HIST_SMOOTH_SIZE = 9
HIST_SMOOTH_KERNEL = (
    np.ones(HIST_SMOOTH_SIZE, dtype=np.float32) /
    float(HIST_SMOOTH_SIZE)
)
PEAK_CENTROID_HALF_WIDTH = 10
MAX_PEAK_WIDTH = 30
MIN_PEAK_RESPONSE = 2.0
MIN_PEAK_TO_MEAN = 1.35
MAX_PREDICTION_DELTA = 24.0
MAX_MEASUREMENT_OFFSET = 30.0
MAX_MEASUREMENT_OFFSET_LOST = 48.0
TRACK_GOOD_CONFIDENCE = 0.55
MIN_REINIT_OBSERVATIONS = 4
CENTER_OUTLIER_THRESHOLD_PX = 45.0
LOW_CONFIDENCE_THRESHOLD = 0.45
MIN_GEOMETRY_SCORE = 0.30
MAX_WIDTH_TANGENT_OFFSET = 55.0
MIN_WIDTH_SAMPLES = 2
CENTER_CANDIDATE_MAX_DISTANCE = 80.0
MAX_CENTER_LINK_DISTANCE_PER_WINDOW = 85.0
MAX_CENTER_TURN_DEGREES = 70.0
MAX_TEMPORAL_GAP_SECONDS = 0.5
WIDTH_CONFLICT_SCALE_PX = 12.0

# Explicit lane-center tracking modes. NO_LANE is a fail-safe state, not a
# usable tracking mode.
LANE_MODE_BOTH_SIDES = "BOTH_SIDES"
LANE_MODE_LEFT_ONLY = "LEFT_ONLY"
LANE_MODE_RIGHT_ONLY = "RIGHT_ONLY"
LANE_MODE_NONE = "NO_LANE"
MIN_MODE_BOUNDARY_POINTS = MIN_LANE_PATH_POINTS

# Spatial center fitting uses only NumPy and runs on the small set of window
# centers. It therefore adds no new runtime dependency.
CENTER_FIT_MIN_POINTS = 3
CENTER_FIT_MIN_SCALE_PX = 3.0
CENTER_FIT_HUBER_K = 2.0
CENTER_FIT_BLEND_BOTH = 0.80
CENTER_FIT_BLEND_SINGLE = 0.90

PROFILE_INTERVAL_FRAMES = 100
PROFILE_KEYS = (
    "input_cvbridge",
    "white_mask",
    "undistort_remap",
    "bev_transform",
    "lane_tracking",
    "center_confidence",
    "debug_drawing",
    "debug_msg_conversion",
    "ros_publish",
    "callback_total"
)

WHITE_PROFILE_INTERVAL_FRAMES = 100
WHITE_PROFILE_KEYS = (
    "roi_copy",
    "bgr_to_hls",
    "hls_split",
    "hls_threshold",
    "bgr_split",
    "bgr_minmax_spread",
    "bgr_threshold",
    "gaussian_blur",
    "local_contrast",
    "strong_threshold",
    "mask_logic",
    "roi_apply",
    "median_blur",
    "morph_open",
    "morph_close",
    "connected_components",
    "component_filter_rebuild",
    "final_morphology"
)


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

OPEN_KERNEL = cv2.getStructuringElement(
    cv2.MORPH_RECT,
    (OPEN_KERNEL_SIZE, OPEN_KERNEL_SIZE)
)
CLOSE_KERNEL = cv2.getStructuringElement(
    cv2.MORPH_RECT,
    (CLOSE_KERNEL_W, CLOSE_KERNEL_H)
)

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
metric_bev_pub = None
lane_tracking_pub = None
lane_pub = None
lane_path_pub = None
lane_confidence_pub = None
undistort_map_x = None
undistort_map_y = None
last_print_time = 0.0

previous_left_points = [None] * NUM_WINDOWS
previous_right_points = [None] * NUM_WINDOWS
previous_center_points = [None] * NUM_WINDOWS
lane_width_history = deque(maxlen=LANE_WIDTH_HISTORY_SIZE)
lane_width_est_px = float(LANE_WIDTH_NOMINAL_PX)
previous_tracking_confidence = 0.0
previous_tracking_time = None
last_confidence_components = {}

profile_frame_count = 0
profile_totals = dict((key, 0.0) for key in PROFILE_KEYS)
profile_batch_start_time = None

white_profile_frame_count = 0
white_profile_totals = dict((key, 0.0) for key in WHITE_PROFILE_KEYS)


# ============================================================
# ROS 图像发布函数
# ============================================================

def publisher_has_subscribers(pub):
    if pub is None:
        return False
    try:
        return pub.get_num_connections() > 0
    except AttributeError:
        return True


def begin_frame_profile():
    global profile_batch_start_time
    if profile_batch_start_time is None:
        profile_batch_start_time = time.time()
    return {}


def record_profile_time(frame_profile, key, elapsed_seconds):
    if frame_profile is not None:
        frame_profile[key] = frame_profile.get(key, 0.0) + elapsed_seconds


def record_white_profile_time(white_profile, key, elapsed_seconds):
    white_profile[key] = white_profile.get(key, 0.0) + elapsed_seconds


def finish_white_profile(white_profile):
    global white_profile_frame_count
    global white_profile_totals

    for key in WHITE_PROFILE_KEYS:
        white_profile_totals[key] += white_profile.get(key, 0.0)
    white_profile_frame_count += 1

    if white_profile_frame_count < WHITE_PROFILE_INTERVAL_FRAMES:
        return

    values = []
    for key in WHITE_PROFILE_KEYS:
        average_ms = (
            white_profile_totals[key] * 1000.0 /
            float(white_profile_frame_count)
        )
        values.append("%s=%.2f" % (key, average_ms))

    print("WHITE_PROFILE avg_ms[%d] %s" % (
        white_profile_frame_count,
        " ".join(values)
    ))

    white_profile_frame_count = 0
    white_profile_totals = dict(
        (key, 0.0) for key in WHITE_PROFILE_KEYS
    )


def finish_frame_profile(frame_profile):
    global profile_frame_count
    global profile_totals
    global profile_batch_start_time

    for key in PROFILE_KEYS:
        profile_totals[key] += frame_profile.get(key, 0.0)
    profile_frame_count += 1

    if profile_frame_count < PROFILE_INTERVAL_FRAMES:
        return

    values = []
    for key in PROFILE_KEYS:
        average_ms = profile_totals[key] * 1000.0 / float(profile_frame_count)
        values.append("%s=%.2f" % (key, average_ms))

    batch_elapsed = max(0.001, time.time() - profile_batch_start_time)
    wall_fps = float(profile_frame_count) / batch_elapsed

    print("PROFILE avg_ms[%d] wall_fps=%.2f %s" % (
        profile_frame_count,
        wall_fps,
        " ".join(values)
    ))

    if len(last_confidence_components) > 0:
        print("CONF components center=%.2f boundary=%.2f width=%.2f "
              "continuity=%.2f direction=%.2f temporal=%.2f reinit=%s" % (
                  last_confidence_components.get("center", 0.0),
                  last_confidence_components.get("boundary", 0.0),
                  last_confidence_components.get("width", 0.0),
                  last_confidence_components.get("continuity", 0.0),
                  last_confidence_components.get("direction", 0.0),
                  last_confidence_components.get("temporal", 0.0),
                  str(last_confidence_components.get("reinitialized", False))
              ))

    profile_frame_count = 0
    profile_totals = dict((key, 0.0) for key in PROFILE_KEYS)
    profile_batch_start_time = None




def bev_point_to_vehicle_m(u, v):
    """将Metric BEV像素转换到以后轴中心为原点的车辆坐标系。"""
    x_forward_m = (600.0 - float(v)) / 400.0
    y_left_m = (240.0 - float(u)) / 400.0
    return x_forward_m, y_left_m


def select_nearest_lane_path_segment(center_segments, diagnostic=None):
    """Publish all usable current-frame centers, ordered near to far."""
    points = [p for segment in center_segments for p in segment
              if point_is_usable(p)
              and np.isfinite(p['x']) and np.isfinite(p['y'])
              and CONTROL_Y_MIN <= p['y'] <= CONTROL_Y_MAX]
    points.sort(key=lambda p: bev_point_to_vehicle_m(p['x'],p['y'])[0])
    if diagnostic is not None:
        diagnostic.update(reason='ok' if points else 'no_trusted_centers',
            segment_lengths=[len(s) for s in center_segments],
            selection='all_current_points', point_count=len(points))
    return points


def build_lane_path_message(center_segments, stamp):
    """由冻结的center segments构造near-to-far vehicle-frame Path。"""
    path_msg = Path()
    path_msg.header.stamp = stamp
    path_msg.header.frame_id = LANE_PATH_FRAME_ID

    selected_points = select_nearest_lane_path_segment(center_segments)
    for point in selected_points:
        x_forward_m, y_left_m = bev_point_to_vehicle_m(
            point["x"],
            point["y"]
        )
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = LANE_PATH_FRAME_ID
        pose.pose.position.x = x_forward_m
        pose.pose.position.y = y_left_m
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        path_msg.poses.append(pose)

    return path_msg


# ============================================================
# Metric BEV
# ============================================================

def init_undistort_maps():
    """
    为固定的 640x360 前摄在节点启动时预计算去畸变映射。
    """
    global undistort_map_x
    global undistort_map_y
    global new_camera_matrix

    new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
        K,
        D,
        CAMERA_SIZE,
        1.0,
        CAMERA_SIZE
    )

    float_map_x, float_map_y = cv2.initUndistortRectifyMap(
        K,
        D,
        None,
        new_camera_matrix,
        CAMERA_SIZE,
        cv2.CV_32FC1
    )

    # INTER_NEAREST 使用 nninterpolation=True 的固定点 map；启动时只转换一次。
    undistort_map_x, undistort_map_y = cv2.convertMaps(
        float_map_x,
        float_map_y,
        cv2.CV_16SC2,
        nninterpolation=True
    )


def make_metric_bev(white_mask, frame_profile=None):
    """
    将现有白线二值 mask 去畸变后映射到 4 pixel/cm 的 Metric BEV。
    """
    h, w = white_mask.shape[:2]

    if (w, h) != CAMERA_SIZE:
        rospy.logwarn_throttle(
            5.0,
            "metric BEV expects 640x360 mask, got %dx%d" % (w, h)
        )
        return None

    remap_start = time.time()
    undistorted_white_mask = cv2.remap(
        white_mask,
        undistort_map_x,
        undistort_map_y,
        interpolation=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0
    )
    record_profile_time(
        frame_profile,
        "undistort_remap",
        time.time() - remap_start
    )

    bev_start = time.time()
    metric_bev_mask = cv2.warpPerspective(
        undistorted_white_mask,
        H,
        BEV_SIZE,
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0
    )
    record_profile_time(
        frame_profile,
        "bev_transform",
        time.time() - bev_start
    )

    return metric_bev_mask


# ============================================================
# 自适应 Sliding Window 车道追踪（仅调试）
# ============================================================

def clip_x(x):
    return float(np.clip(float(x), 0.0, float(BEV_WIDTH - 1)))


def point_is_usable(point):
    return point is not None and point.get("usable", False) and point.get("x") is not None


def get_lane_width_estimate():
    return float(lane_width_est_px)


def find_histogram_peak(binary_img, y0, y1, x_pred, search_half_width,
                        max_measurement_offset):
    """
    在预测位置附近搜索平滑列直方图峰，并仅用峰附近像素计算加权质心。
    返回 lane_x、峰值质量和实际搜索范围。
    """
    y0 = max(0, int(y0))
    y1 = min(BEV_HEIGHT, int(y1))
    x_pred = clip_x(x_pred)

    x0 = max(0, int(round(x_pred - search_half_width)))
    x1 = min(BEV_WIDTH, int(round(x_pred + search_half_width + 1)))
    search_range = (x0, x1)

    if y1 <= y0 or x1 <= x0:
        return None, 0.0, search_range

    roi = binary_img[y0:y1, x0:x1]
    column_histogram = np.count_nonzero(roi, axis=0).astype(np.float32)

    if column_histogram.size == 0 or float(column_histogram.sum()) < MIN_WINDOW_PIXELS:
        return None, 0.0, search_range

    smoothed = np.convolve(
        column_histogram,
        HIST_SMOOTH_KERNEL,
        mode="same"
    )

    peak_index = int(np.argmax(smoothed))
    peak_value = float(smoothed[peak_index])
    mean_value = float(np.mean(smoothed))
    peak_to_mean = peak_value / max(0.5, mean_value)

    if peak_value < MIN_PEAK_RESPONSE or peak_to_mean < MIN_PEAK_TO_MEAN:
        return None, 0.0, search_range

    # 宽而平的响应通常来自横向岔线；纵向边界应形成较窄的列峰。
    half_peak = peak_value * 0.5
    peak_left = peak_index
    peak_right = peak_index

    while peak_left > 0 and smoothed[peak_left - 1] >= half_peak:
        peak_left -= 1
    while peak_right + 1 < smoothed.size and smoothed[peak_right + 1] >= half_peak:
        peak_right += 1

    if peak_right - peak_left + 1 > MAX_PEAK_WIDTH:
        return None, 0.0, search_range

    centroid_left = max(0, peak_index - PEAK_CENTROID_HALF_WIDTH)
    centroid_right = min(column_histogram.size, peak_index + PEAK_CENTROID_HALF_WIDTH + 1)
    centroid_weights = column_histogram[centroid_left:centroid_right]
    pixel_count = float(centroid_weights.sum())

    if pixel_count < MIN_WINDOW_PIXELS:
        return None, 0.0, search_range

    centroid_columns = np.arange(
        centroid_left,
        centroid_right,
        dtype=np.float32
    )
    lane_x = float(x0) + float(
        np.sum(centroid_columns * centroid_weights) / pixel_count
    )

    if abs(lane_x - x_pred) > float(max_measurement_offset):
        return None, 0.0, search_range

    concentration_score = min(1.0, max(0.0, (peak_to_mean - 1.0) / 3.0))
    pixel_score = min(1.0, pixel_count / float(MIN_WINDOW_PIXELS * 4))
    quality = 0.65 * concentration_score + 0.35 * pixel_score

    return clip_x(lane_x), float(quality), search_range


def initialize_lane_bases(binary_img, lane_width_est):
    """
    使用车辆中心和当前车道宽先验，在底部合理邻域内初始化左右边界。
    """
    vehicle_center_x = BEV_WIDTH / 2.0
    expected_left = vehicle_center_x - lane_width_est / 2.0
    expected_right = vehicle_center_x + lane_width_est / 2.0
    y0 = BEV_HEIGHT - INITIAL_HIST_HEIGHT

    left_x, left_quality, _ = find_histogram_peak(
        binary_img,
        y0,
        BEV_HEIGHT,
        expected_left,
        INITIAL_SEARCH_HALF_WIDTH,
        INITIAL_SEARCH_HALF_WIDTH
    )
    right_x, right_quality, _ = find_histogram_peak(
        binary_img,
        y0,
        BEV_HEIGHT,
        expected_right,
        INITIAL_SEARCH_HALF_WIDTH,
        INITIAL_SEARCH_HALF_WIDTH
    )

    # A dashed boundary can have no paint in the nearest two windows. Seed
    # from current trusted farther pixels within the same lateral prior.
    # Missing windows remain unobserved and never become control points.
    if left_x is None:
        left_x, left_quality, _ = find_histogram_peak(
            binary_img, CONTROL_Y_MIN, y0, expected_left,
            INITIAL_SEARCH_HALF_WIDTH, INITIAL_SEARCH_HALF_WIDTH)
    if right_x is None:
        right_x, right_quality, _ = find_histogram_peak(
            binary_img, CONTROL_Y_MIN, y0, expected_right,
            INITIAL_SEARCH_HALF_WIDTH, INITIAL_SEARCH_HALF_WIDTH)

    return left_x, right_x, left_quality, right_quality


def predict_window_x(points, base_x, window_index, previous_points, use_previous):
    usable_points = [point for point in points if point_is_usable(point)]

    if len(usable_points) >= 2:
        x_last = float(usable_points[-1]["x"])
        x_before = float(usable_points[-2]["x"])
        delta = float(np.clip(
            x_last - x_before,
            -MAX_PREDICTION_DELTA,
            MAX_PREDICTION_DELTA
        ))
        x_pred = x_last + delta
    elif len(usable_points) == 1:
        x_pred = float(usable_points[-1]["x"])
    else:
        x_pred = float(base_x)

    if use_previous and previous_points is not None and window_index < len(previous_points):
        previous_point = previous_points[window_index]
        if point_is_usable(previous_point):
            if len(usable_points) == 0:
                x_pred = float(previous_point["x"])
            else:
                x_pred = 0.70 * x_pred + 0.30 * float(previous_point["x"])

    return clip_x(x_pred)


def trace_boundary(binary_img, side, base_x, base_inferred,
                   previous_points_for_side, use_previous):
    points = []
    missing_count = 0
    tracking_lost = base_x is None

    for window_index in range(NUM_WINDOWS):
        y1 = BEV_HEIGHT - window_index * WINDOW_HEIGHT
        y0 = y1 - WINDOW_HEIGHT
        y_center = int((y0 + y1) / 2)

        if tracking_lost:
            points.append({
                "x": None,
                "y": y_center,
                "usable": False,
                "observed": False,
                "predicted": False,
                "score": 0.0,
                "search_x": None,
                "search_half_width": WINDOW_HALF_WIDTH_LOST,
                "used_lost_width": True,
                "geometry_ok": False
            })
            continue

        x_pred = predict_window_x(
            points,
            base_x,
            window_index,
            previous_points_for_side,
            use_previous
        )

        first_half_width = WINDOW_HALF_WIDTH
        if base_inferred and window_index == 0:
            first_half_width = WINDOW_HALF_WIDTH_LOST

        lane_x, quality, _ = find_histogram_peak(
            binary_img,
            y0,
            y1,
            x_pred,
            first_half_width,
            MAX_MEASUREMENT_OFFSET if first_half_width == WINDOW_HALF_WIDTH
            else MAX_MEASUREMENT_OFFSET_LOST
        )
        used_half_width = first_half_width

        # 局部失败时扩大一次搜索范围，不改变预测中心。
        if lane_x is None and first_half_width < WINDOW_HALF_WIDTH_LOST:
            lane_x, quality, _ = find_histogram_peak(
                binary_img,
                y0,
                y1,
                x_pred,
                WINDOW_HALF_WIDTH_LOST,
                MAX_MEASUREMENT_OFFSET_LOST
            )
            used_half_width = WINDOW_HALF_WIDTH_LOST

        if lane_x is not None:
            missing_count = 0
            points.append({
                "x": float(lane_x),
                "y": y_center,
                "usable": True,
                "observed": True,
                "predicted": False,
                "score": float(quality),
                "search_x": float(x_pred),
                "search_half_width": int(used_half_width),
                "used_lost_width": used_half_width == WINDOW_HALF_WIDTH_LOST,
                "geometry_ok": True
            })
        else:
            missing_count += 1

            if missing_count <= MAX_EMPTY_WINDOWS:
                points.append({
                    "x": float(x_pred),
                    "y": y_center,
                    "usable": True,
                    "observed": False,
                    "predicted": True,
                    "score": 0.0,
                    "search_x": float(x_pred),
                    "search_half_width": int(used_half_width),
                    "used_lost_width": True,
                    "geometry_ok": True
                })
            else:
                # Keep looking in farther windows using the bounded prior.
                # Do not permanently discard observed paint after a gap.
                points.append({
                    "x": None,
                    "y": y_center,
                    "usable": False,
                    "observed": False,
                    "predicted": False,
                    "score": 0.0,
                    "search_x": float(x_pred),
                    "search_half_width": int(used_half_width),
                    "used_lost_width": True,
                    "geometry_ok": False
                })

    return points


def track_boundaries_once(binary_img, lane_width_est, use_previous):
    if use_previous:
        left_base = None
        right_base = None

        if len(previous_left_points) > 0 and point_is_usable(previous_left_points[0]):
            left_base = float(previous_left_points[0]["x"])
        if len(previous_right_points) > 0 and point_is_usable(previous_right_points[0]):
            right_base = float(previous_right_points[0]["x"])

        if left_base is None and right_base is None:
            use_previous = False

    if not use_previous:
        left_base, right_base, _, _ = initialize_lane_bases(
            binary_img,
            lane_width_est
        )

    left_inferred = False
    right_inferred = False

    if left_base is None and right_base is not None:
        left_base = clip_x(right_base - lane_width_est)
        left_inferred = True
    elif right_base is None and left_base is not None:
        right_base = clip_x(left_base + lane_width_est)
        right_inferred = True

    left_points = trace_boundary(
        binary_img,
        "left",
        left_base,
        left_inferred,
        previous_left_points if use_previous else None,
        use_previous
    )
    right_points = trace_boundary(
        binary_img,
        "right",
        right_base,
        right_inferred,
        previous_right_points if use_previous else None,
        use_previous
    )

    return left_points, right_points


def count_observed(points):
    return sum(1 for point in points if point_is_usable(point) and point.get("observed", False))


def point_in_trusted_control_region(point):
    if point is None:
        return False
    source_y = float(point.get("source_y", point.get("y", -1.0)))
    return CONTROL_Y_MIN <= source_y < CONTROL_Y_MAX


def boundary_point_is_reliable(point, trusted_only=False):
    if not point_is_usable(point):
        return False
    if not point.get("observed", False):
        return False
    if not point.get("geometry_ok", True):
        return False
    if float(point.get("score", 0.0)) < MIN_GEOMETRY_SCORE:
        return False
    if trusted_only and not point_in_trusted_control_region(point):
        return False
    return True


def get_local_boundary_geometry(boundary_points, window_index, side,
                                trusted_only=False):
    """
    用目标层附近最多三个可靠观测估计单位切线和指向车道内部的单位法向。
    """
    boundary_point = boundary_points[window_index]
    if not boundary_point_is_reliable(boundary_point, trusted_only):
        return None

    candidates = []
    for candidate_index in range(
            max(0, window_index - 2),
            min(NUM_WINDOWS, window_index + 3)):
        candidate = boundary_points[candidate_index]
        if boundary_point_is_reliable(candidate, trusted_only):
            candidates.append((
                abs(candidate_index - window_index),
                candidate_index,
                candidate
            ))

    candidates.sort(key=lambda item: (item[0], item[1]))
    candidates = candidates[:3]
    candidates.sort(key=lambda item: item[1])

    if len(candidates) < 2:
        return None

    first_point = candidates[0][2]
    last_point = candidates[-1][2]
    tangent_x = float(last_point["x"] - first_point["x"])
    tangent_y = float(last_point["y"] - first_point["y"])
    tangent_norm = float(np.hypot(tangent_x, tangent_y))

    if tangent_norm <= 1.0:
        return None

    tangent_x /= tangent_norm
    tangent_y /= tangent_norm

    normal_x = -tangent_y
    normal_y = tangent_x

    if side == "left" and normal_x < 0.0:
        normal_x = -normal_x
        normal_y = -normal_y
    elif side == "right" and normal_x > 0.0:
        normal_x = -normal_x
        normal_y = -normal_y

    return {
        "tangent_x": float(tangent_x),
        "tangent_y": float(tangent_y),
        "normal_x": float(normal_x),
        "normal_y": float(normal_y)
    }


def project_normal_distance(source_points, opposite_points,
                            window_index, side):
    geometry = get_local_boundary_geometry(
        source_points,
        window_index,
        side,
        trusted_only=True
    )
    if geometry is None:
        return None

    source_point = source_points[window_index]
    best_distance = None
    best_tangent_offset = None

    for opposite_point in opposite_points:
        if not boundary_point_is_reliable(opposite_point, trusted_only=True):
            continue

        delta_x = float(opposite_point["x"] - source_point["x"])
        delta_y = float(opposite_point["y"] - source_point["y"])
        normal_distance = (
            delta_x * geometry["normal_x"] +
            delta_y * geometry["normal_y"]
        )
        tangent_offset = abs(
            delta_x * geometry["tangent_x"] +
            delta_y * geometry["tangent_y"]
        )

        if normal_distance <= 0.0:
            continue
        if best_tangent_offset is None or tangent_offset < best_tangent_offset:
            best_tangent_offset = tangent_offset
            best_distance = normal_distance

    if best_distance is None or best_tangent_offset > MAX_WIDTH_TANGENT_OFFSET:
        return None

    return float(best_distance)


def update_lane_width_estimate(left_points, right_points):
    """
    只使用可信 ROI 内、沿局部法向投影得到的物理宽度更新慢速历史。
    """
    global lane_width_est_px

    attempted_distances = []
    accepted_distances = []

    for window_index in range(NUM_WINDOWS):
        left_distance = project_normal_distance(
            left_points,
            right_points,
            window_index,
            "left"
        )
        right_distance = project_normal_distance(
            right_points,
            left_points,
            window_index,
            "right"
        )

        for normal_distance in (left_distance, right_distance):
            if normal_distance is None:
                continue
            attempted_distances.append(normal_distance)
            if LANE_WIDTH_MIN_PX <= normal_distance <= LANE_WIDTH_MAX_PX:
                accepted_distances.append(normal_distance)

    if len(accepted_distances) >= MIN_WIDTH_SAMPLES:
        frame_width = float(np.median(np.asarray(
            accepted_distances,
            dtype=np.float32
        )))
        if LANE_WIDTH_MIN_PX <= frame_width <= LANE_WIDTH_MAX_PX:
            lane_width_history.append(frame_width)
            history_width = float(np.median(np.asarray(
                lane_width_history,
                dtype=np.float32
            )))
            lane_width_est_px = (
                (1.0 - LANE_WIDTH_EMA_ALPHA) * lane_width_est_px +
                LANE_WIDTH_EMA_ALPHA * history_width
            )
            lane_width_est_px = float(np.clip(
                lane_width_est_px,
                LANE_WIDTH_MIN_PX,
                LANE_WIDTH_MAX_PX
            ))

    width_measurement_consistency = None
    if len(attempted_distances) > 0:
        current_width_median = float(np.median(np.asarray(
            attempted_distances,
            dtype=np.float32
        )))
        if current_width_median < LANE_WIDTH_MIN_PX:
            width_error = LANE_WIDTH_MIN_PX - current_width_median
        elif current_width_median > LANE_WIDTH_MAX_PX:
            width_error = current_width_median - LANE_WIDTH_MAX_PX
        else:
            width_error = 0.0
        width_measurement_consistency = float(np.clip(
            1.0 - width_error / WIDTH_CONFLICT_SCALE_PX,
            0.0,
            1.0
        ))

    return (
        get_lane_width_estimate(),
        len(accepted_distances),
        len(attempted_distances),
        width_measurement_consistency
    )


def estimate_normal_center(boundary_points, window_index, side, lane_width_est):
    boundary_point = boundary_points[window_index]

    if not boundary_point_is_reliable(boundary_point):
        return None

    geometry = get_local_boundary_geometry(
        boundary_points,
        window_index,
        side,
        trusted_only=False
    )
    half_width = lane_width_est / 2.0
    used_normal = geometry is not None

    if geometry is not None:
        center_x = (
            float(boundary_point["x"]) +
            geometry["normal_x"] * half_width
        )
        center_y = (
            float(boundary_point["y"]) +
            geometry["normal_y"] * half_width
        )
    else:
        if side == "left":
            center_x = float(boundary_point["x"]) + half_width
        else:
            center_x = float(boundary_point["x"]) - half_width
        center_y = float(boundary_point["y"])

    return {
        "x": clip_x(center_x),
        "y": float(np.clip(center_y, 0.0, float(BEV_HEIGHT - 1))),
        "source_y": float(boundary_point["y"]),
        "window_index": int(window_index),
        "score": float(boundary_point.get("score", 0.0)),
        "usable": True,
        "observed": False,
        "compensated": True,
        "used_normal": used_normal,
        "geometry_agreement": True
    }


def count_reliable_boundary_points(points, trusted_only=True):
    return sum(
        1 for point in points
        if boundary_point_is_reliable(point, trusted_only=trusted_only)
    )


def determine_lane_tracking_mode(left_points, right_points):
    """
    Select one frame-level center construction mode.

    A single mode is used for the complete frame so that individual windows do
    not alternate between left- and right-derived center points.
    """
    left_count = count_reliable_boundary_points(left_points)
    right_count = count_reliable_boundary_points(right_points)

    left_ready = left_count >= MIN_MODE_BOUNDARY_POINTS
    right_ready = right_count >= MIN_MODE_BOUNDARY_POINTS

    if left_ready and right_ready:
        return LANE_MODE_BOTH_SIDES
    if left_ready:
        return LANE_MODE_LEFT_ONLY
    if right_ready:
        return LANE_MODE_RIGHT_ONLY
    return LANE_MODE_NONE


def make_two_side_center(left_point, right_point, window_index):
    """
    Pair the two observed boundaries at the same longitudinal window.

    This is deliberately different from independently offsetting the two noisy
    local normals. Pairing at one fixed BEV row prevents the two center
    candidates from jumping apart on a curve.
    """
    if not boundary_point_is_reliable(left_point):
        return None
    if not boundary_point_is_reliable(right_point):
        return None

    left_score = max(0.05, float(left_point.get("score", 0.0)))
    right_score = max(0.05, float(right_point.get("score", 0.0)))
    center_x = 0.5 * (float(left_point["x"]) + float(right_point["x"]))
    center_y = 0.5 * (float(left_point["y"]) + float(right_point["y"]))

    return {
        "x": clip_x(center_x),
        "y": float(np.clip(center_y, 0.0, float(BEV_HEIGHT - 1))),
        "source_y": float(center_y),
        "window_index": int(window_index),
        "score": float(0.5 * (left_score + right_score)),
        "usable": True,
        "observed": True,
        "compensated": False,
        "predicted": False,
        "used_normal": False,
        "geometry_agreement": True,
        "lane_mode": LANE_MODE_BOTH_SIDES
    }


def build_center_points(left_points, right_points, lane_width_est,
                        lane_mode=None, preferred_side=None):
    """One frame-wide reference and one normal-offset algorithm for all modes.

    All observations contribute to the normal estimate without outlier removal.
    The preferred boundary wins when available; otherwise support determines the
    frame's reference. The other side supplies width measurements, never a
    different construction at individual windows.
    """
    if lane_mode is None:
        lane_mode = determine_lane_tracking_mode(left_points,right_points)
    candidates = []
    for side, points in (('left',left_points),('right',right_points)):
        observed = [p if boundary_point_is_reliable(p,True) else None for p in points]
        support = [p for p in observed if p is not None]
        if len(support) < MIN_MODE_BOUNDARY_POINTS:
            continue
        # Estimate normals using every observation; retain the measured vertices.
        degree = 2 if len(support) >= 5 else 1
        coefficients = np.polyfit([p['y'] for p in support],[p['x'] for p in support],degree)
        quality = (side == preferred_side, len(support),
                   max(p['y'] for p in support)-min(p['y'] for p in support),
                   sum(p.get('score',0.) for p in support))
        candidates.append((quality,side,observed,coefficients))
    if not candidates:
        return [None] * NUM_WINDOWS
    _,side,boundary,coefficients = max(candidates,key=lambda item:item[0])
    offset_distance = (30.0 * PX_PER_CM
        if side == 'left' and (preferred_side == 'left' or lane_mode == LANE_MODE_LEFT_ONLY)
        else lane_width_est / 2.0)
    derivative = np.polyder(coefficients)
    centers = []
    for window_index, point in enumerate(boundary):
        if not point_is_usable(point):
            centers.append(None)
            continue
        slope = float(np.polyval(derivative,point['y']))
        offset = offset_distance / np.hypot(1.,slope)
        if side == 'right':
            offset = -offset
        y = float(point['y']-slope*offset)
        center = dict(point)
        center.update(x=float(point['x']+offset),y=y,source_y=y,lane_mode=lane_mode,
                      window_index=window_index,compensated=True,used_normal=True,
                      geometry_agreement=True)
        centers.append(center)
    result = centers
    for point in result:
        if point is not None:
            point.update(center_method='fitted_boundary_normal',reference_side=side)
    return result


def fit_center_curve_robust(center_points):
    """
    Fit x_lateral_pixel=f(y_forward_pixel) with a lightweight robust fit.

    Returns coefficients and the observed source-y interval. The latter keeps
    gap filling interpolation-only; the function never extrapolates a center
    path beyond current observations.
    """
    fit_points = [
        point for point in center_points
        if point_is_usable(point) and point_in_trusted_control_region(point)
    ]

    if len(fit_points) < CENTER_FIT_MIN_POINTS:
        return None

    y_values = np.asarray([
        float(point.get("source_y", point["y"]))
        for point in fit_points
    ], dtype=np.float64)
    x_values = np.asarray([
        float(point["x"])
        for point in fit_points
    ], dtype=np.float64)

    if len(np.unique(y_values)) < CENTER_FIT_MIN_POINTS:
        return None

    degree = 2 if len(fit_points) >= 4 else 1
    base_weights = np.asarray([
        max(0.10, min(1.0, float(point.get("score", 0.0)))) *
        (0.70 if point.get("compensated", False) else 1.0)
        for point in fit_points
    ], dtype=np.float64)
    robust_weights = np.ones(len(fit_points), dtype=np.float64)
    coefficients = None

    try:
        for _ in range(3):
            fit_weights = np.sqrt(base_weights * robust_weights)
            coefficients = np.polyfit(
                y_values,
                x_values,
                degree,
                w=fit_weights
            )
            residuals = x_values - np.polyval(coefficients, y_values)
            residual_median = float(np.median(residuals))
            absolute_residuals = np.abs(residuals - residual_median)
            mad = float(np.median(absolute_residuals))
            scale = max(CENTER_FIT_MIN_SCALE_PX, 1.4826 * mad)
            cutoff = CENTER_FIT_HUBER_K * scale
            robust_weights = np.ones(len(fit_points), dtype=np.float64)
            outlier_mask = absolute_residuals > cutoff
            robust_weights[outlier_mask] = (
                cutoff / np.maximum(absolute_residuals[outlier_mask], 1.0)
            )
    except (TypeError, ValueError, np.linalg.LinAlgError):
        return None

    if coefficients is None or not np.all(np.isfinite(coefficients)):
        return None

    return {
        "coefficients": coefficients,
        "min_y": float(np.min(y_values)),
        "max_y": float(np.max(y_values))
    }


def smooth_center_points_spatially(center_points, lane_mode):
    """
    Smooth the complete center curve and fill only internal window gaps.

    Existing points retain a small part of their measured position. Missing
    points are synthesized only between the nearest and farthest current
    observations, never beyond them.
    """
    fit_result = fit_center_curve_robust(center_points)
    if fit_result is None:
        return center_points

    coefficients = fit_result["coefficients"]
    min_y = fit_result["min_y"]
    max_y = fit_result["max_y"]
    fit_blend = (
        CENTER_FIT_BLEND_BOTH
        if lane_mode == LANE_MODE_BOTH_SIDES
        else CENTER_FIT_BLEND_SINGLE
    )
    smoothed = []

    for window_index in range(NUM_WINDOWS):
        nominal_y = float(
            BEV_HEIGHT - window_index * WINDOW_HEIGHT - WINDOW_HEIGHT / 2
        )
        current_point = center_points[window_index]

        if not (CONTROL_Y_MIN <= nominal_y <= CONTROL_Y_MAX):
            smoothed.append(current_point)
            continue

        if nominal_y < min_y or nominal_y > max_y:
            smoothed.append(current_point)
            continue

        fitted_x = clip_x(float(np.polyval(coefficients, nominal_y)))

        if point_is_usable(current_point):
            stable_point = dict(current_point)
            stable_point["x"] = float(
                fit_blend * fitted_x +
                (1.0 - fit_blend) * float(current_point["x"])
            )
            stable_point["y"] = nominal_y
            stable_point["source_y"] = nominal_y
            stable_point["spatially_smoothed"] = True
            smoothed.append(stable_point)
        else:
            smoothed.append({
                "x": fitted_x,
                "y": nominal_y,
                "source_y": nominal_y,
                "window_index": int(window_index),
                "score": 0.0,
                "usable": True,
                "observed": False,
                "compensated": True,
                "predicted": True,
                "used_normal": False,
                "geometry_agreement": True,
                "lane_mode": lane_mode,
                "spatially_smoothed": True
            })

    return smoothed


def vector_turn_degrees(first_vector, second_vector):
    first_norm = float(np.hypot(first_vector[0], first_vector[1]))
    second_norm = float(np.hypot(second_vector[0], second_vector[1]))
    if first_norm <= 1.0 or second_norm <= 1.0:
        return 180.0
    cosine = (
        first_vector[0] * second_vector[0] +
        first_vector[1] * second_vector[1]
    ) / (first_norm * second_norm)
    cosine = float(np.clip(cosine, -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def build_center_segments(center_points):
    """Keep one ordered point set without continuity or turn-angle gating."""
    points = [p for p in center_points if point_is_usable(p)]
    points.sort(key=lambda p: float(p['y']),reverse=True)
    return [points] if points else []


def calculate_tracking_confidence(left_points, right_points,
                                  center_points, center_segments,
                                  accepted_width_count,
                                  attempted_width_count,
                                  width_measurement_consistency,
                                  lane_width_est,
                                  reinitialized,
                                  temporal_comparable):
    global last_confidence_components

    trusted_window_count = sum(
        1 for window_index in range(NUM_WINDOWS)
        if CONTROL_Y_MIN <= (
            BEV_HEIGHT - window_index * WINDOW_HEIGHT - WINDOW_HEIGHT / 2
        ) < CONTROL_Y_MAX
    )

    trusted_left_count = sum(
        1 for point in left_points
        if boundary_point_is_reliable(point, trusted_only=True)
    )
    trusted_right_count = sum(
        1 for point in right_points
        if boundary_point_is_reliable(point, trusted_only=True)
    )
    trusted_centers = [
        point for point in center_points
        if point_is_usable(point) and point_in_trusted_control_region(point)
    ]

    center_coverage = float(len(trusted_centers)) / float(trusted_window_count)
    boundary_coverage = float(
        trusted_left_count + trusted_right_count
    ) / float(2 * trusted_window_count)

    lane_width_est_score = 1.0 if (
        LANE_WIDTH_MIN_PX <= lane_width_est <= LANE_WIDTH_MAX_PX
    ) else 0.0

    if attempted_width_count >= MIN_WIDTH_SAMPLES:
        accepted_ratio = min(
            1.0,
            float(accepted_width_count) / float(attempted_width_count)
        )
        measurement_score = (
            width_measurement_consistency
            if width_measurement_consistency is not None else 0.5
        )

        if accepted_width_count >= MIN_WIDTH_SAMPLES:
            # 有可靠当前帧测量时，采样接受率只作轻量加分，不再一票否决。
            width_score = 0.85 + 0.15 * accepted_ratio
        elif accepted_width_count > 0:
            width_score = (
                0.70 +
                0.20 * accepted_ratio +
                0.10 * measurement_score
            )
        else:
            # 完全没有物理范围内测量时，才视为明确冲突。
            width_score = (
                0.35 * lane_width_est_score +
                0.65 * measurement_score
            )
    else:
        # 当前帧测量不足代表 unknown，而不是物理错误。
        width_score = 0.75 * lane_width_est_score

    direction_scores = []
    valid_links = 0

    for segment in center_segments:
        trusted_segment = [
            point for point in segment
            if point_in_trusted_control_region(point)
        ]
        if len(trusted_segment) == 0:
            continue
        valid_links += max(0, len(trusted_segment) - 1)

        for point_index in range(2, len(trusted_segment)):
            point_a = trusted_segment[point_index - 2]
            point_b = trusted_segment[point_index - 1]
            point_c = trusted_segment[point_index]
            turn_degrees = vector_turn_degrees(
                (point_b["x"] - point_a["x"], point_b["y"] - point_a["y"]),
                (point_c["x"] - point_b["x"], point_c["y"] - point_b["y"])
            )
            direction_scores.append(max(
                0.0,
                1.0 - turn_degrees / MAX_CENTER_TURN_DEGREES
            ))

    if len(trusted_centers) >= 2:
        continuity_score = float(valid_links) / float(len(trusted_centers) - 1)
    elif len(trusted_centers) == 1:
        continuity_score = 0.35
    else:
        continuity_score = 0.0

    if len(direction_scores) > 0:
        direction_score = float(np.mean(np.asarray(
            direction_scores,
            dtype=np.float32
        )))
    elif len(trusted_centers) >= 2:
        direction_score = 0.70
    else:
        direction_score = 0.35 if len(trusted_centers) == 1 else 0.0

    previous_differences = []
    if temporal_comparable and not reinitialized:
        for window_index, center_point in enumerate(center_points):
            if not point_is_usable(center_point):
                continue
            if not point_in_trusted_control_region(center_point):
                continue
            previous_point = previous_center_points[window_index]
            if point_is_usable(previous_point):
                previous_differences.append(float(np.hypot(
                    center_point["x"] - previous_point["x"],
                    center_point["y"] - previous_point["y"]
                )))

    if not temporal_comparable or reinitialized:
        temporal_score = 0.90
    elif len(previous_differences) > 0:
        median_difference = float(np.median(np.asarray(
            previous_differences,
            dtype=np.float32
        )))
        temporal_score = float(np.clip(
            1.0 - median_difference / 60.0,
            0.0,
            1.0
        ))
    else:
        temporal_score = 0.85

    confidence = (
        0.32 * center_coverage +
        0.16 * boundary_coverage +
        0.18 * width_score +
        0.14 * continuity_score +
        0.10 * direction_score +
        0.10 * temporal_score
    )

    # 物理宽度与时序一致性连续调制总分，不使用0.55等离散硬上限。
    confidence *= 0.65 + 0.35 * center_coverage
    confidence *= 0.30 + 0.70 * width_score
    confidence *= 0.50 + 0.50 * temporal_score

    last_confidence_components = {
        "center": float(center_coverage),
        "boundary": float(boundary_coverage),
        "width": float(width_score),
        "continuity": float(continuity_score),
        "direction": float(direction_score),
        "temporal": float(temporal_score),
        "reinitialized": bool(reinitialized)
    }

    return float(np.clip(confidence, 0.0, 1.0))


def stabilize_center_points(center_points, tracking_confidence):
    stabilized = []

    for window_index in range(NUM_WINDOWS):
        current_point = center_points[window_index]
        previous_point = previous_center_points[window_index]

        if not point_is_usable(current_point):
            stabilized.append(None)
            continue

        stable_point = dict(current_point)

        if point_is_usable(previous_point):
            delta_x = abs(float(current_point["x"] - previous_point["x"]))

            if (
                    delta_x > CENTER_OUTLIER_THRESHOLD_PX and
                    tracking_confidence < LOW_CONFIDENCE_THRESHOLD):
                stable_point["x"] = float(previous_point["x"])
                stable_point["y"] = float(previous_point["y"])
                stable_point["observed"] = False
                stable_point["compensated"] = True
            else:
                stable_point["x"] = float(
                    EMA_ALPHA * current_point["x"] +
                    (1.0 - EMA_ALPHA) * previous_point["x"]
                )
                stable_point["y"] = float(
                    EMA_ALPHA * current_point["y"] +
                    (1.0 - EMA_ALPHA) * previous_point["y"]
                )

        stabilized.append(stable_point)

    return stabilized


def find_dashed_left_boundary(mask):
    """Recognize separated dash components, independent of their image side.

    This course's dashed divider is the LEFT boundary. Require a single,
    well-supported chain; continuous solids and ambiguous parallel chains do
    not establish this semantic identity. All returned observations have pixels.
    """
    roi = mask.copy()
    roi[:CONTROL_Y_MIN] = 0
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(roi, 8)
    candidates = [i for i in range(1, count)
                  if 40 <= stats[i, cv2.CC_STAT_AREA] <= 800
                  and 4 <= stats[i, cv2.CC_STAT_WIDTH] <= 60
                  and 8 <= stats[i, cv2.CC_STAT_HEIGHT] <= 45]
    if not 5 <= len(candidates) <= 24:
        return None
    selected = list(candidates)
    while len(selected) >= max(5, int(np.ceil(.75 * len(candidates)))):
        xy = centroids[selected]
        if len(np.unique(xy[:, 1])) < 3:
            return None
        coefficients = np.polyfit(xy[:, 1], xy[:, 0], 2)
        residual = np.abs(xy[:, 0] - np.polyval(coefficients, xy[:, 1]))
        if np.max(residual) <= 8.0:
            break
        del selected[int(np.argmax(residual))]
    else:
        return None
    gaps = np.diff(np.sort(xy[:, 1]))
    if np.ptp(xy[:, 1]) < 120 or np.min(gaps) < 15 or np.max(gaps) > 85:
        return None
    # Centroid spacing alone also accepts a solid stripe split by tiny cracks.
    # Require real dark gaps between paint components before asserting LEFT.
    ordered = sorted(selected,key=lambda i:stats[i,cv2.CC_STAT_TOP])
    paint_gaps = [float(stats[b,cv2.CC_STAT_TOP] -
        (stats[a,cv2.CC_STAT_TOP]+stats[a,cv2.CC_STAT_HEIGHT]))
        for a,b in zip(ordered,ordered[1:])]
    median_height = float(np.median(stats[selected,cv2.CC_STAT_HEIGHT]))
    if float(np.median(paint_gaps)) < max(5.0,.25*median_height):
        return None
    accepted_labels = np.zeros(count, dtype=np.bool_)
    accepted_labels[selected] = True
    support = accepted_labels[labels]
    points = []
    for index in range(NUM_WINDOWS):
        y1 = BEV_HEIGHT - index * WINDOW_HEIGHT
        y0 = y1 - WINDOW_HEIGHT
        ys, xs = np.nonzero(support[y0:y1])
        point = dict(x=None, y=(y0+y1)/2., usable=False, observed=False,
                     predicted=False, score=0., geometry_ok=False)
        if len(xs) >= 12:
            y = float(np.mean(ys) + y0)
            point.update(x=float(np.polyval(coefficients, y)), y=y,
                         usable=True, observed=True, score=.9, geometry_ok=True)
        points.append(point)
    return points, coefficients


def apply_dashed_left_boundary(dashed, right_points, lane_width):
    """Discard the outside-left solid or divider mislabelled as RIGHT."""
    left_points, coefficients = dashed
    derivative = np.polyder(coefficients)
    checked_right = []
    for point in right_points:
        point = dict(point)
        if point_is_usable(point):
            y = float(point['y'])
            normal_distance = (point['x'] - np.polyval(coefficients, y)) / np.hypot(
                1., np.polyval(derivative, y))
            if not .55 * lane_width <= normal_distance <= 1.5 * lane_width:
                point.update(x=None, usable=False, observed=False, geometry_ok=False)
        checked_right.append(point)
    return left_points, checked_right


def dashed_left_centers(left_points, coefficients, lane_width):
    """Offset the observed divider toward our lane with actual normal geometry.

    Keep the offset's forward coordinate and do not clamp it to the image edge.
    A model-derived center can lie outside the visible BEV while its supporting
    boundary is still observed; it is not an observation of another white line.
    """
    derivative = np.polyder(coefficients)
    centers = []
    for index, point in enumerate(left_points):
        if not boundary_point_is_reliable(point):
            centers.append(None)
            continue
        slope = float(np.polyval(derivative, point['y']))
        offset = lane_width / (2. * np.hypot(1., slope))
        y = float(point['y'] - slope * offset)
        centers.append(dict(x=float(point['x'] + offset), y=y, source_y=y,
                            window_index=index, score=point['score'], usable=True,
                            observed=False, compensated=True, predicted=False,
                            used_normal=True, geometry_agreement=True,
                            lane_mode=LANE_MODE_LEFT_ONLY))
    return centers


def fit_continuous_center_points(center_points):
    """Fit actual offset coordinates; interpolate only supported internal gaps.

    The input must describe one boundary-derived center, not a mixture of
    midpoint and offset constructions. A full fit and leave-one-out fits
    identify isolated bad points without joining unrelated path fragments.
    """
    points = [p for p in center_points if point_is_usable(p)
              and point_in_trusted_control_region(p)]
    if len(points) < 3:
        return center_points
    ys = np.asarray([p['y'] for p in points], dtype=np.float64)
    xs = np.asarray([p['x'] for p in points], dtype=np.float64)
    degree = 2 if len(points) >= 5 else 1
    best = None
    indices = np.arange(len(points))
    for omitted in [None] + list(indices):
        subset = indices if omitted is None else indices[indices != omitted]
        if len(np.unique(ys[subset])) < degree + 1:
            continue
        coefficients = np.polyfit(ys[subset], xs[subset], degree)
        residuals = np.abs(xs - np.polyval(coefficients, ys))
        inliers = residuals <= 12.0
        count = int(np.sum(inliers))
        if count < max(3, int(np.ceil(.65 * len(points)))):
            continue
        score = (count, -float(np.sum(residuals[inliers])))
        if best is None or score > best[0]:
            best = (score, inliers)
    if best is None:
        return [None] * NUM_WINDOWS
    inliers = best[1]
    coefficients = np.polyfit(ys[inliers], xs[inliers], degree)
    support = np.sort(ys[inliers])
    result = [None] * NUM_WINDOWS
    for index in range(NUM_WINDOWS):
        lower = BEV_HEIGHT - (index + 1) * WINDOW_HEIGHT
        upper = lower + WINDOW_HEIGHT
        if upper <= support[0] or lower > support[-1]:
            continue
        y = float(np.clip((lower + upper) / 2.0, support[0], support[-1]))
        right = int(np.searchsorted(support, y))
        if (0 < right < len(support) and abs(support[right]-y) > 1e-6
                and support[right]-support[right-1] > 2 * WINDOW_HEIGHT):
            continue
        result[index] = dict(x=float(np.polyval(coefficients,y)), y=y,
            source_y=y, window_index=index, usable=True, observed=False,
            predicted=True, compensated=True, used_normal=True,
            geometry_agreement=True, spatially_smoothed=True,
            score=float(np.mean([p.get('score',0.) for p,k in zip(points,inliers) if k])),
            lane_mode=points[0].get('lane_mode',LANE_MODE_LEFT_ONLY))
    return result


def track_metric_lane(metric_bev_mask, frame_profile=None, source_stamp=None):
    """
    返回仅用于调试绘制的左右边界点、中心路径、车道宽和置信度。
    """
    global previous_left_points
    global previous_right_points
    global previous_center_points
    global previous_tracking_confidence
    global previous_tracking_time

    current_tracking_time = time.time() if source_stamp is None else float(source_stamp)
    temporal_comparable = (
        previous_tracking_time is not None and
        0.0 <= current_tracking_time - previous_tracking_time <=
        MAX_TEMPORAL_GAP_SECONDS
    )
    lane_width_est = get_lane_width_estimate()
    use_previous = temporal_comparable and previous_tracking_confidence >= TRACK_GOOD_CONFIDENCE
    reinitialized = not use_previous

    lane_tracking_start = time.time()
    left_points, right_points = track_boundaries_once(
        metric_bev_mask,
        lane_width_est,
        use_previous
    )

    observed_count = count_observed(left_points) + count_observed(right_points)

    # Retry the alternate search when current-frame observations are sparse.
    # Recent history guides search windows only; it never supplies path points.
    if observed_count < MIN_REINIT_OBSERVATIONS and (use_previous or temporal_comparable):
        retry_left, retry_right = track_boundaries_once(
            metric_bev_mask,
            lane_width_est,
            not use_previous
        )
        retry_observed_count = count_observed(retry_left) + count_observed(retry_right)

        if retry_observed_count >= observed_count:
            left_points = retry_left
            right_points = retry_right
            reinitialized = use_previous

    record_profile_time(
        frame_profile,
        "lane_tracking",
        time.time() - lane_tracking_start
    )

    center_confidence_start = time.time()
    dashed = find_dashed_left_boundary(metric_bev_mask)
    if dashed is not None:
        left_points, right_points = apply_dashed_left_boundary(
            dashed, right_points, lane_width_est)
        # The previous center may belong to the adjacent lane. Never blend it
        # into a path whose divider identity is established by this frame.
        reinitialized = True
    (
        lane_width_est,
        accepted_width_count,
        attempted_width_count,
        width_measurement_consistency
    ) = update_lane_width_estimate(
        left_points,
        right_points
    )
    lane_mode = determine_lane_tracking_mode(
        left_points,
        right_points
    )
    center_points = build_center_points(
        left_points,
        right_points,
        lane_width_est,
        lane_mode,
        # The course's left boundary remains authoritative when it becomes solid.
        # Irregular right markings must not move the path or its lateral offset.
        preferred_side='left'
    )
    center_segments = build_center_segments(center_points)
    tracking_confidence = calculate_tracking_confidence(
        left_points,
        right_points,
        center_points,
        center_segments,
        accepted_width_count,
        attempted_width_count,
        width_measurement_consistency,
        lane_width_est,
        reinitialized,
        temporal_comparable
    )
    record_profile_time(
        frame_profile,
        "center_confidence",
        time.time() - center_confidence_start
    )

    previous_left_points = left_points
    previous_right_points = right_points
    previous_center_points = center_points
    previous_tracking_confidence = tracking_confidence
    previous_tracking_time = current_tracking_time

    return {
        "dashed_left": dashed is not None,
        "left_points": left_points,
        "right_points": right_points,
        "center_points": center_points,
        "center_segments": center_segments,
        "lane_width_est": float(lane_width_est),
        "tracking_confidence": float(tracking_confidence),
        "lane_mode": lane_mode,
        "left_valid_count": count_observed(left_points),
        "right_valid_count": count_observed(right_points)
    }


def draw_boundary_debug(debug_img, points, observed_color):
    last_point = None

    for point in points:
        y_center = int(point["y"])

        if point.get("search_x") is not None:
            half_width = int(point.get("search_half_width", WINDOW_HALF_WIDTH))
            search_x = int(round(point["search_x"]))
            y0 = max(0, y_center - WINDOW_HEIGHT / 2)
            y1 = min(BEV_HEIGHT - 1, y_center + WINDOW_HEIGHT / 2)
            x0 = max(0, search_x - half_width)
            x1 = min(BEV_WIDTH - 1, search_x + half_width)
            rectangle_color = (0, 165, 255) if point.get("used_lost_width", False) else (90, 90, 90)
            cv2.rectangle(debug_img, (x0, y0), (x1, y1), rectangle_color, 1)

        if not point_is_usable(point):
            last_point = None
            continue

        draw_point = (int(round(point["x"])), y_center)

        if point.get("observed", False):
            point_color = observed_color
            radius = 4
        else:
            point_color = (0, 255, 255)
            radius = 3

        if last_point is not None:
            cv2.line(debug_img, last_point, draw_point, observed_color, 2)
        cv2.circle(debug_img, draw_point, radius, point_color, -1)
        last_point = draw_point


def make_lane_tracking_debug(metric_bev_mask, tracking_result):
    debug_img = cv2.cvtColor(metric_bev_mask, cv2.COLOR_GRAY2BGR)

    cv2.line(
        debug_img,
        (0, CONTROL_Y_MIN),
        (BEV_WIDTH - 1, CONTROL_Y_MIN),
        (255, 255, 0),
        1
    )
    cv2.putText(
        debug_img,
        "trusted control region",
        (8, CONTROL_Y_MIN - 6),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (255, 255, 0),
        1,
        cv2.LINE_AA
    )

    draw_boundary_debug(
        debug_img,
        tracking_result["left_points"],
        (255, 80, 0)
    )
    draw_boundary_debug(
        debug_img,
        tracking_result["right_points"],
        (0, 0, 255)
    )

    path_diagnostic = {}
    selected_path = select_nearest_lane_path_segment(tracking_result['center_segments'],path_diagnostic)
    selected_ids = set(id(p) for p in selected_path)
    for center_point in tracking_result["center_points"]:
        if not point_is_usable(center_point):
            continue

        draw_point = (
            int(round(center_point["x"])),
            int(round(center_point["y"]))
        )

        point_color = (0, 255, 0) if id(center_point) in selected_ids else (100,100,100)
        cv2.circle(debug_img, draw_point, 4, point_color, -1)

    for center_segment in [selected_path]:
        for point_index in range(1, len(center_segment)):
            previous_point = center_segment[point_index - 1]
            current_point = center_segment[point_index]
            cv2.line(
                debug_img,
                (
                    int(round(previous_point["x"])),
                    int(round(previous_point["y"]))
                ),
                (
                    int(round(current_point["x"])),
                    int(round(current_point["y"]))
                ),
                (0, 255, 0),
                2
            )

    cv2.rectangle(debug_img, (0, 0), (BEV_WIDTH - 1, 66), (0, 0, 0), -1)
    cv2.putText(
        debug_img,
        "left valid: %d  right valid: %d" % (
            tracking_result["left_valid_count"],
            tracking_result["right_valid_count"]
        ),
        (8, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (255, 255, 255),
        1,
        cv2.LINE_AA
    )
    cv2.putText(
        debug_img,
        "lane width: %.1f cm" % (
            tracking_result["lane_width_est"] / PX_PER_CM
        ),
        (8, 41),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (255, 255, 255),
        1,
        cv2.LINE_AA
    )
    cv2.putText(
        debug_img,
        "confidence: %.2f  mode: %s" % (
            tracking_result["tracking_confidence"],
            tracking_result.get("lane_mode", LANE_MODE_NONE)
        ),
        (8, 62),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (255, 255, 255),
        1,
        cv2.LINE_AA
    )

    reference = next((p.get('reference_side') for p in tracking_result['center_points'] if p), None)
    cv2.putText(debug_img, 'normal-fit reference: %s' % (reference or 'none'),
                (8,83), cv2.FONT_HERSHEY_SIMPLEX, .45, (255,255,255), 1)
    cv2.putText(debug_img, 'published path: %d  %s' % (len(selected_path),path_diagnostic['reason']),
                (8,101), cv2.FONT_HERSHEY_SIMPLEX, .42, (255,255,255), 1)
    return debug_img


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

def filter_white_components(binary_img, white_profile=None):
    """
    连通域过滤。

    重点：
    1. 不能全局删除竖向区域，否则右侧白线会被误删；
    2. 只过滤中间偏下的竖向反光；
    3. 左右两侧尽量保留，因为白线经常出现在左右两侧；
    4. 过滤面积过大、很稀碎的小噪声。
    """

    img_h, img_w = binary_img.shape[:2]

    stage_start = time.time()
    num_labels, labels, stats, centroids = (
        cv2.connectedComponentsWithStatsWithAlgorithm(
            binary_img,
            8,
            cv2.CV_32S,
            cv2.CCL_GRANA
        )
    )
    if white_profile is not None:
        record_white_profile_time(
            white_profile,
            "connected_components",
            time.time() - stage_start
        )

    stage_start = time.time()
    x = stats[:, cv2.CC_STAT_LEFT].astype(np.float64)
    y = stats[:, cv2.CC_STAT_TOP].astype(np.float64)
    bw = stats[:, cv2.CC_STAT_WIDTH].astype(np.float64)
    bh = stats[:, cv2.CC_STAT_HEIGHT].astype(np.float64)
    area = stats[:, cv2.CC_STAT_AREA].astype(np.float64)

    fill_ratio = area / (bw * bh)
    cx = x + bw / 2.0
    cy = y + bh / 2.0
    in_center = (
        (cx >= img_w * 0.35) &
        (cx <= img_w * 0.65)
    )

    keep = (
        (area >= MIN_COMPONENT_AREA) &
        (bw > 1) &
        (bh > 1) &
        (area <= MAX_COMPONENT_AREA)
    )
    keep &= ~((area < 80) & (fill_ratio < MIN_FILL_RATIO))

    if FILTER_CENTER_VERTICAL_REFLECTION:
        keep &= ~(
            in_center &
            (cy > img_h * 0.45) &
            (bh > 35) &
            (bh > bw * 1.7)
        )

    keep &= ~(in_center & (area < 120) & (bh > bw * 3.0))
    keep &= ~(in_center & (fill_ratio < 0.08) & (area < 300))
    keep[0] = False

    label_lut = np.zeros(num_labels, dtype=np.uint8)
    label_lut[keep] = 255
    cleaned = label_lut[labels]

    if white_profile is not None:
        record_white_profile_time(
            white_profile,
            "component_filter_rebuild",
            time.time() - stage_start
        )

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

    white_profile = {}

    # 1. 地面 ROI
    stage_start = time.time()
    h, w = frame_bgr.shape[:2]
    y_start = int(h * FLOOR_Y_RATIO)
    gaussian_radius = 15
    halo_start = max(0, y_start - gaussian_radius)
    halo_offset = y_start - halo_start
    frame_halo = frame_bgr[halo_start:h, :]
    frame_roi = frame_bgr[y_start:h, :]
    record_white_profile_time(
        white_profile,
        "roi_copy",
        time.time() - stage_start
    )

    # 2. HLS 白色粗筛
    stage_start = time.time()
    hls_halo = cv2.cvtColor(frame_halo, cv2.COLOR_BGR2HLS)
    record_white_profile_time(
        white_profile,
        "bgr_to_hls",
        time.time() - stage_start
    )

    stage_start = time.time()
    l_halo = cv2.extractChannel(hls_halo, 1)
    l_channel = l_halo[halo_offset:, :]
    s_channel = cv2.extractChannel(hls_halo[halo_offset:, :], 2)
    record_white_profile_time(
        white_profile,
        "hls_split",
        time.time() - stage_start
    )

    stage_start = time.time()
    mask_l = cv2.inRange(l_channel, WHITE_L_MIN, 255)
    mask_s = cv2.inRange(s_channel, 0, WHITE_S_MAX)
    mask_hls = cv2.bitwise_and(mask_l, mask_s)
    record_white_profile_time(
        white_profile,
        "hls_threshold",
        time.time() - stage_start
    )

    # 3. BGR 真白色判断
    stage_start = time.time()
    b_channel, g_channel, r_channel = cv2.split(frame_roi)
    record_white_profile_time(
        white_profile,
        "bgr_split",
        time.time() - stage_start
    )

    stage_start = time.time()
    max_rgb = cv2.max(cv2.max(r_channel, g_channel), b_channel)
    min_rgb = cv2.min(cv2.min(r_channel, g_channel), b_channel)

    delta_rgb = cv2.subtract(max_rgb, min_rgb)
    record_white_profile_time(
        white_profile,
        "bgr_minmax_spread",
        time.time() - stage_start
    )

    # 三个通道都比较亮
    stage_start = time.time()
    mask_rgb_bright = cv2.inRange(min_rgb, WHITE_RGB_MIN, 255)

    # 三个通道差距不能太大
    mask_rgb_neutral = cv2.inRange(delta_rgb, 0, WHITE_RGB_DELTA_MAX)

    mask_rgb_white = cv2.bitwise_and(mask_rgb_bright, mask_rgb_neutral)
    record_white_profile_time(
        white_profile,
        "bgr_threshold",
        time.time() - stage_start
    )

    # 4. 局部对比度判断
    # 白线应该比附近地面亮，而不是整片地面一起亮
    stage_start = time.time()
    l_blur_halo = cv2.GaussianBlur(l_halo, (31, 31), 0)
    l_blur = l_blur_halo[halo_offset:, :]
    record_white_profile_time(
        white_profile,
        "gaussian_blur",
        time.time() - stage_start
    )

    stage_start = time.time()
    local_contrast = cv2.subtract(l_channel, l_blur)

    mask_contrast = cv2.inRange(local_contrast, LOCAL_CONTRAST_MIN, 255)
    record_white_profile_time(
        white_profile,
        "local_contrast",
        time.time() - stage_start
    )

    # 5. 强白线判断
    # 对非常亮的白线放宽局部对比度限制
    stage_start = time.time()
    mask_strong_l = cv2.inRange(l_channel, STRONG_L_MIN, 255)
    record_white_profile_time(
        white_profile,
        "strong_threshold",
        time.time() - stage_start
    )

    # 6. 综合白线判断
    # 条件 A：HLS 像白色 + RGB 像白色 + 比周围亮
    stage_start = time.time()
    mask_normal_white = cv2.bitwise_and(mask_hls, mask_rgb_white)
    mask_normal_white = cv2.bitwise_and(mask_normal_white, mask_contrast)

    # 条件 B：RGB 像白色 + 亮度非常高
    mask_strong_white = cv2.bitwise_and(mask_rgb_white, mask_strong_l)

    roi_white_mask = cv2.bitwise_or(mask_normal_white, mask_strong_white)
    record_white_profile_time(
        white_profile,
        "mask_logic",
        time.time() - stage_start
    )

    # 7. 放回完整 640x360 mask；上半部与原 ROI mask 逻辑完全一致为0。
    stage_start = time.time()
    white_mask = np.zeros((h, w), dtype=np.uint8)
    white_mask[y_start:h, :] = roi_white_mask
    record_white_profile_time(
        white_profile,
        "roi_apply",
        time.time() - stage_start
    )

    # 8. 中值滤波，减少孤立噪点
    stage_start = time.time()
    white_mask = cv2.medianBlur(white_mask, 3)
    record_white_profile_time(
        white_profile,
        "median_blur",
        time.time() - stage_start
    )

    # 9. 开运算：去小白点
    stage_start = time.time()
    white_mask = cv2.morphologyEx(
        white_mask,
        cv2.MORPH_OPEN,
        OPEN_KERNEL
    )
    record_white_profile_time(
        white_profile,
        "morph_open",
        time.time() - stage_start
    )

    # 10. 闭运算：连接断裂虚线/边线
    stage_start = time.time()
    white_mask = cv2.morphologyEx(
        white_mask,
        cv2.MORPH_CLOSE,
        CLOSE_KERNEL
    )
    record_white_profile_time(
        white_profile,
        "morph_close",
        time.time() - stage_start
    )

    # 11. 连通域过滤
    white_mask = filter_white_components(white_mask, white_profile)

    # 12. 再轻微闭运算一次，让白线块更完整
    stage_start = time.time()
    white_mask = cv2.morphologyEx(
        white_mask,
        cv2.MORPH_CLOSE,
        CLOSE_KERNEL
    )
    record_white_profile_time(
        white_profile,
        "final_morphology",
        time.time() - stage_start
    )

    finish_white_profile(white_profile)

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



# ============================================================
# 主函数
# ============================================================




# ROS transport retained for the complete mission; perception above is the backup.
LANE_OBSERVATION_TOPIC = "/vision/lane_observation"
lane_observation_pub = None
left_boundary_pub = None
processing_hz = 0.0
max_image_age = 0.5
latest_image_msg = None
latest_image_lock = threading.Lock()
processing_lock = threading.Lock()
last_processed_source_stamp = None
processing_timer = None

def publish_image(pub, img, encoding=None, frame_profile=None, source_header=None):
    """
    将 OpenCV 图像发布为 ROS Image。
    """
    if img is None or not publisher_has_subscribers(pub):
        return

    try:
        if img.dtype != np.uint8:
            img = np.clip(img, 0, 255).astype(np.uint8)

        if encoding is None:
            if len(img.shape) == 2:
                encoding = "mono8"
            else:
                encoding = "bgr8"

        conversion_start = time.time()
        msg = bridge.cv2_to_imgmsg(img, encoding=encoding)
        if source_header is not None:
            msg.header = source_header
        else:
            msg.header.stamp = rospy.Time.now()
        record_profile_time(
            frame_profile,
            "debug_msg_conversion",
            time.time() - conversion_start
        )

        publish_start = time.time()
        pub.publish(msg)
        record_profile_time(
            frame_profile,
            "ros_publish",
            time.time() - publish_start
        )

    except Exception as e:
        rospy.logwarn("publish_image failed: %s" % str(e))



def configure_lane_windows(height, min_span):
    global WINDOW_HEIGHT, NUM_WINDOWS, MIN_LANE_PATH_FORWARD_SPAN_M
    global MAX_EMPTY_WINDOWS
    global previous_left_points, previous_right_points, previous_center_points
    if height not in (20, 40) or not .05 <= min_span <= .30:
        raise ValueError('lane windows must be 20/40 px; min span 0.05..0.30 m')
    WINDOW_HEIGHT = int(height)
    # In near mode, spend the extra samples only in the trusted control band.
    # Rows above CONTROL_Y_MIN never enter the published control path.
    NUM_WINDOWS = (BEV_HEIGHT-CONTROL_Y_MIN if height == 20 else BEV_HEIGHT) // WINDOW_HEIGHT
    MAX_EMPTY_WINDOWS = 80 // WINDOW_HEIGHT  # preserve the original 0.20 m gap budget
    MIN_LANE_PATH_FORWARD_SPAN_M = float(min_span)
    previous_left_points = [None] * NUM_WINDOWS
    previous_right_points = [None] * NUM_WINDOWS
    previous_center_points = [None] * NUM_WINDOWS



def build_left_boundary_message(points, stamp):
    """Reuse observed boundary windows; no predictions or center-line guesses."""
    msg = Path()
    msg.header.stamp, msg.header.frame_id = stamp, LANE_PATH_FRAME_ID
    for point in sorted(points, key=lambda p: -p['y']):
        if not boundary_point_is_reliable(point, trusted_only=True):
            continue
        x,y = bev_point_to_vehicle_m(point['x'],point['y'])
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x,pose.pose.position.y = x,y
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)
    return msg



def build_observed_lane_boundaries(tracking_result, stamp):
    """Export continuous observed paint paths without a half-width offset."""
    result = {}
    for side in ('left', 'right'):
        raw = tracking_result[side + '_points'] if tracking_result else []
        # Raw tracker points encode their window by list position. Segment
        # construction needs that index explicitly; do not mutate tracker state.
        points = [dict(p, window_index=i) if boundary_point_is_reliable(p, trusted_only=True)
                  else None for i,p in enumerate(raw)]
        selected = select_nearest_lane_path_segment(build_center_segments(points))
        result[side.upper()] = [list(bev_point_to_vehicle_m(p['x'],p['y']))
                                for p in selected]
    return result



def process_image(msg):
    global last_print_time

    callback_start = time.time()
    frame_profile = begin_frame_profile()
    input_start = time.time()
    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("cv_bridge failed: %s" % str(e))
        return
    record_profile_time(
        frame_profile,
        "input_cvbridge",
        time.time() - input_start
    )

    white_start = time.time()
    white_mask = detect_white_line(frame)
    record_profile_time(
        frame_profile,
        "white_mask",
        time.time() - white_start
    )

    metric_bev_mask = make_metric_bev(white_mask, frame_profile)
    lane_tracking_debug = None
    tracking_result = None

    if metric_bev_mask is not None:
        try:
            tracking_result = track_metric_lane(
                metric_bev_mask,
                frame_profile,
                source_stamp=msg.header.stamp.to_sec()
            )
            if publisher_has_subscribers(lane_tracking_pub):
                debug_start = time.time()
                lane_tracking_debug = make_lane_tracking_debug(
                    metric_bev_mask,
                    tracking_result
                )
                record_profile_time(
                    frame_profile,
                    "debug_drawing",
                    time.time() - debug_start
                )
        except Exception as e:
            rospy.logwarn_throttle(
                5.0,
                "lane tracking debug failed: %s" % str(e)
            )

    if tracking_result is None:
        center_segments = []
        tracking_confidence = 0.0
    else:
        center_segments = tracking_result["center_segments"]
        tracking_confidence = float(np.clip(
            tracking_result["tracking_confidence"],
            0.0,
            1.0
        ))

    lane_path_msg = build_lane_path_message(
        center_segments,
        msg.header.stamp
    )
    confidence_msg = Float32()
    confidence_msg.data = tracking_confidence

    path_publish_start = time.time()
    if lane_observation_pub is not None:
        diagnostic = {}
        select_nearest_lane_path_segment(center_segments, diagnostic)
        diagnostic['dashed_left'] = bool(tracking_result and tracking_result.get('dashed_left'))
        diagnostic['center_method'] = 'fitted_boundary_normal'
        diagnostic['reference_side'] = next((p.get('reference_side')
            for p in tracking_result['center_points'] if p), None) if tracking_result else None
        observation = dict(
            stamp=msg.header.stamp.to_sec(), seq=int(msg.header.seq),
            frame=LANE_PATH_FRAME_ID, confidence=tracking_confidence,
            followed_boundary=({'LEFT_ONLY': 'LEFT', 'RIGHT_ONLY': 'RIGHT'}.get(
                tracking_result.get('lane_mode'), 'UNKNOWN') if tracking_result else 'UNKNOWN'),
            points=[[p.pose.position.x, p.pose.position.y] for p in lane_path_msg.poses],
            boundaries=build_observed_lane_boundaries(tracking_result,msg.header.stamp),
            diagnostic=diagnostic if tracking_result else dict(reason='tracking_failed'),
            processing_ms=1000.0*(time.time()-callback_start))
        lane_observation_pub.publish(String(data=json.dumps(observation, allow_nan=False)))
    lane_confidence_pub.publish(confidence_msg)
    lane_path_pub.publish(lane_path_msg)
    if left_boundary_pub is not None:
        left_boundary_pub.publish(build_left_boundary_message(
            tracking_result['left_points'] if tracking_result else [],msg.header.stamp))
    record_profile_time(
        frame_profile,
        "ros_publish",
        time.time() - path_publish_start
    )

    publish_image(front_pub, frame, encoding="bgr8", frame_profile=frame_profile,
                  source_header=msg.header)
    publish_image(white_pub, white_mask, encoding="mono8", frame_profile=frame_profile,
                  source_header=msg.header)
    publish_image(
        metric_bev_pub,
        metric_bev_mask,
        encoding="mono8",
        frame_profile=frame_profile,
        source_header=msg.header
    )
    publish_image(
        lane_tracking_pub,
        lane_tracking_debug,
        encoding="bgr8",
        frame_profile=frame_profile,
        source_header=msg.header
    )

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
            lane_publish_start = time.time()
            lane_pub.publish(msg)
            record_profile_time(
                frame_profile,
                "ros_publish",
                time.time() - lane_publish_start
            )
        last_print_time = now

    record_profile_time(
        frame_profile,
        "callback_total",
        time.time() - callback_start
    )
    finish_frame_profile(frame_profile)



def source_stamp_key(msg):
    stamp = msg.header.stamp
    return int(stamp.secs), int(stamp.nsecs)



def image_callback(msg):
    global latest_image_msg

    if processing_hz <= 0.0:
        processing_lock.acquire()
        try:
            process_image(msg)
        finally:
            processing_lock.release()
        return

    with latest_image_lock:
        latest_image_msg = msg



def process_latest_image(_event):
    global last_processed_source_stamp

    if not processing_lock.acquire(False):
        return

    try:
        with latest_image_lock:
            msg = latest_image_msg
            if msg is None:
                return

            source_stamp = source_stamp_key(msg)
            if source_stamp == last_processed_source_stamp:
                return
            last_processed_source_stamp = source_stamp

        if max_image_age > 0.0:
            age = (rospy.Time.now() - msg.header.stamp).to_sec()
            if not 0 <= age <= max_image_age:
                rospy.logwarn_throttle(
                    5.0,
                    "dropping stale front image: age=%.3fs limit=%.3fs" % (
                        age,
                        max_image_age
                    )
                )
                return

        process_image(msg)
    finally:
        processing_lock.release()



def build_runtime_calibration(stamp):
    """Read-only startup evidence from this process, including hard-coded settings."""
    constants = {}
    for key, value in list(globals().items()):
        if not key.isupper():
            continue
        if isinstance(value, np.ndarray):
            constants[key] = value.tolist()
        elif isinstance(value, (bool, int, float, str, tuple, list)):
            constants[key] = value
    with open(__file__, 'rb') as source:
        source_hash = hashlib.sha256(source.read()).hexdigest()
    return dict(stamp=float(stamp), source_kind='running_process',
                source_file=__file__, source_sha256=source_hash,
                H=H.tolist(), K=K.tolist(), D=D.tolist(),
                rectified_camera_matrix=new_camera_matrix.tolist(),
                origin_uv=[240., 600.], pixels_per_metre=400.,
                transform='undistorted 640x360 image -> metric BEV; rear axle origin',
                processing_hz=processing_hz, max_image_age=max_image_age,
                constants=constants)



def check_lane_owner(publishers, topic):
    owners = publishers.get(topic, [])
    if owners:
        raise RuntimeError('lane topic already published by %s; stop the old preview/driver first: %s' %
                           (', '.join(owners), topic))



def main():
    global FRONT_CAMERA_TOPIC
    global DEBUG_FRONT_TOPIC
    global DEBUG_WHITE_TOPIC
    global DEBUG_METRIC_BEV_TOPIC
    global DEBUG_LANE_TRACKING_TOPIC
    global LANE_SEND_TOPIC
    global LANE_PATH_TOPIC
    global LANE_CONFIDENCE_TOPIC
    global front_pub
    global white_pub
    global metric_bev_pub
    global lane_tracking_pub
    global lane_pub
    global lane_path_pub
    global lane_confidence_pub
    global lane_observation_pub
    global left_boundary_pub
    global processing_hz
    global max_image_age
    global processing_timer

    rospy.init_node("camera_yihan_white_only", anonymous=True)
    import rosgraph
    publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
    check_lane_owner(dict(publishers), rospy.get_param('~lane_observation_topic', LANE_OBSERVATION_TOPIC))
    configure_lane_windows(rospy.get_param('~window_height',40),
                           float(rospy.get_param('~min_path_span',.15)))
    processing_hz = max(
        0.0,
        float(rospy.get_param("~processing_hz", 0.0))
    )
    max_image_age = max(
        0.0,
        float(rospy.get_param("~max_image_age", 0.5))
    )
    # Preserve the historical defaults while allowing the parking launch to
    # own the complete camera/pose topic graph.  This is especially important
    # for replay sources and for a swapped USB camera; otherwise only the
    # blue-marker node would follow a custom image topic and lane_pose would
    # silently receive no data.
    FRONT_CAMERA_TOPIC = rospy.get_param(
        "~image_topic", FRONT_CAMERA_TOPIC)
    DEBUG_FRONT_TOPIC = rospy.get_param(
        "~raw_topic", DEBUG_FRONT_TOPIC)
    DEBUG_WHITE_TOPIC = rospy.get_param(
        "~white_topic", DEBUG_WHITE_TOPIC)
    DEBUG_METRIC_BEV_TOPIC = rospy.get_param(
        "~metric_bev_topic", DEBUG_METRIC_BEV_TOPIC)
    DEBUG_LANE_TRACKING_TOPIC = rospy.get_param(
        "~lane_tracking_topic", DEBUG_LANE_TRACKING_TOPIC)
    LANE_SEND_TOPIC = rospy.get_param(
        "~lane_topic", LANE_SEND_TOPIC)
    LANE_PATH_TOPIC = rospy.get_param(
        "~lane_path_topic", LANE_PATH_TOPIC)
    LANE_CONFIDENCE_TOPIC = rospy.get_param(
        "~lane_confidence_topic", LANE_CONFIDENCE_TOPIC)
    init_undistort_maps()
    calibration_pub = rospy.Publisher('/vision/lane_calibration', String, queue_size=1, latch=True)
    calibration_pub.publish(String(data=json.dumps(
        build_runtime_calibration(rospy.Time.now().to_sec()), allow_nan=False)))

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

    metric_bev_pub = rospy.Publisher(
        DEBUG_METRIC_BEV_TOPIC,
        Image,
        queue_size=1
    )

    lane_tracking_pub = rospy.Publisher(
        DEBUG_LANE_TRACKING_TOPIC,
        Image,
        queue_size=1
    )

    lane_pub = rospy.Publisher(LANE_SEND_TOPIC, String, queue_size=1)
    left_boundary_pub = rospy.Publisher('/vision/left_boundary',Path,queue_size=1)
    lane_path_pub = rospy.Publisher(LANE_PATH_TOPIC, Path, queue_size=1)
    lane_observation_pub = rospy.Publisher(
        rospy.get_param('~lane_observation_topic', LANE_OBSERVATION_TOPIC), String, queue_size=1)
    lane_confidence_pub = rospy.Publisher(
        LANE_CONFIDENCE_TOPIC,
        Float32,
        queue_size=1
    )
    rospy.Subscriber(
        FRONT_CAMERA_TOPIC,
        Image,
        image_callback,
        queue_size=1,
        buff_size=2 ** 24
    )

    if processing_hz > 0.0:
        processing_timer = rospy.Timer(
            rospy.Duration(1.0 / processing_hz),
            process_latest_image
        )

    print("camera_yihan_white_only started")
    print("subscribe: %s" % FRONT_CAMERA_TOPIC)
    print("processing_hz = %.2f" % processing_hz)
    print("max_image_age = %.3f" % max_image_age)
    print("publish raw: %s" % DEBUG_FRONT_TOPIC)
    print("publish white mask: %s" % DEBUG_WHITE_TOPIC)
    print("publish metric BEV: %s" % DEBUG_METRIC_BEV_TOPIC)
    print("publish lane tracking: %s" % DEBUG_LANE_TRACKING_TOPIC)
    print("publish lane path: %s" % LANE_PATH_TOPIC)
    print("publish lane confidence: %s" % LANE_CONFIDENCE_TOPIC)
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
