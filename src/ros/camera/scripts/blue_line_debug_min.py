#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Read-only front-camera blue-line diagnostic node.

This node publishes debug images and candidate geometry only.  It never
publishes a vehicle-control topic.
"""
from __future__ import print_function

import json
import math

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String


IMAGE_TOPIC = "/front/usb_cam/image_raw"
CAMERA_SIZE = (640, 360)
BEV_SIZE = (480, 400)
PX_PER_M = 400.0
ORIGIN_U = 240.0
ORIGIN_V = 600.0

K = np.array([
    [338.724360, 0.0, 342.299347],
    [0.0, 338.759483, 169.956657],
    [0.0, 0.0, 1.0]
], dtype=np.float64)

D = np.array([
    -0.313909, 0.080347, 0.001009, -0.000759, 0.0
], dtype=np.float64)

H_METRIC = np.array([
    [-0.455307634051, -1.851847127629, 416.759846959926],
    [ 0.072741868805, -3.467660494260, 575.376931923200],
    [ 0.000124867696, -0.007523542869,   1.000000000000]
], dtype=np.float64)


def finite(value):
    value = float(value)
    return not math.isnan(value) and not math.isinf(value)


class BlueLineDebug(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.image_topic = rospy.get_param("~image_topic", IMAGE_TOPIC)
        self.h_min = int(rospy.get_param("~h_min", 90))
        self.h_max = int(rospy.get_param("~h_max", 135))
        self.s_min = int(rospy.get_param("~s_min", 45))
        self.v_min = int(rospy.get_param("~v_min", 25))
        self.min_area_px = int(rospy.get_param("~min_area_px", 20))
        self.publish_hz = float(rospy.get_param("~publish_hz", 10.0))
        if not 0 <= self.h_min <= self.h_max <= 179:
            raise ValueError("invalid hue range")
        if not 0 <= self.s_min <= 255 or not 0 <= self.v_min <= 255:
            raise ValueError("invalid saturation/value threshold")
        if self.min_area_px < 1 or self.publish_hz <= 0.0:
            raise ValueError("invalid debug parameters")

        new_k, _ = cv2.getOptimalNewCameraMatrix(
            K, D, CAMERA_SIZE, 1.0, CAMERA_SIZE)
        self.map_x, self.map_y = cv2.initUndistortRectifyMap(
            K, D, None, new_k, CAMERA_SIZE, cv2.CV_32FC1)
        self.last_publish = 0.0

        self.bev_pub = rospy.Publisher(
            "/debug/blue_min/bev", Image, queue_size=1)
        self.mask_pub = rospy.Publisher(
            "/debug/blue_min/bev_mask", Image, queue_size=1)
        self.overlay_pub = rospy.Publisher(
            "/debug/blue_min/overlay", Image, queue_size=1)
        self.metrics_pub = rospy.Publisher(
            "/debug/blue_min/metrics", String, queue_size=1)
        rospy.Subscriber(
            self.image_topic, Image, self.callback,
            queue_size=1, buff_size=2 ** 24)
        rospy.loginfo(
            "blue_line_debug_min image=%s HSV H[%d,%d] S>=%d V>=%d",
            self.image_topic, self.h_min, self.h_max,
            self.s_min, self.v_min)

    @staticmethod
    def metric(u, v):
        return ((ORIGIN_V - float(v)) / PX_PER_M,
                (ORIGIN_U - float(u)) / PX_PER_M)

    @staticmethod
    def contours(mask):
        return cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    def geometry(self, contour):
        area = float(cv2.contourArea(contour))
        rect = cv2.minAreaRect(contour)
        side_a, side_b = float(rect[1][0]), float(rect[1][1])
        thickness_px, length_px = sorted((side_a, side_b))
        if thickness_px <= 0.0:
            return None
        box = cv2.boxPoints(rect)
        edge_a = box[1] - box[0]
        edge_b = box[2] - box[1]
        edge = edge_a if np.linalg.norm(edge_a) >= np.linalg.norm(edge_b) else edge_b
        angle_deg = math.degrees(math.atan2(float(edge[1]), float(edge[0])))
        while angle_deg <= -90.0:
            angle_deg += 180.0
        while angle_deg > 90.0:
            angle_deg -= 180.0
        x_forward, y_left = self.metric(rect[0][0], rect[0][1])
        result = {
            "area_px": int(round(area)),
            "length_m": length_px / PX_PER_M,
            "thickness_m": thickness_px / PX_PER_M,
            "aspect_ratio": length_px / thickness_px,
            "angle_deg": angle_deg,
            "x_forward_m": x_forward,
            "y_left_m": y_left,
        }
        if not all(finite(value) for value in result.values()):
            return None
        return result

    def publish_image(self, publisher, image, encoding, header):
        try:
            output = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
            output.header = header
            output.header.frame_id = "base_link"
            publisher.publish(output)
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "blue debug image failed: %s", str(exc))

    def callback(self, message):
        now = rospy.Time.now().to_sec()
        if now - self.last_publish < 1.0 / self.publish_hz:
            return
        self.last_publish = now
        try:
            frame = self.bridge.imgmsg_to_cv2(message, "bgr8")
            if (frame.shape[1], frame.shape[0]) != CAMERA_SIZE:
                raise ValueError("expected 640x360 image")

            rectified = cv2.remap(
                frame, self.map_x, self.map_y, cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT, borderValue=0)
            bev = cv2.warpPerspective(
                rectified, H_METRIC, BEV_SIZE, flags=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT, borderValue=0)
            hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
            lower = np.array([self.h_min, self.s_min, self.v_min], dtype=np.uint8)
            upper = np.array([self.h_max, 255, 255], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)

            found = []
            for contour in self.contours(mask):
                if cv2.contourArea(contour) < self.min_area_px:
                    continue
                item = self.geometry(contour)
                if item is not None:
                    found.append((item, contour))
            found.sort(key=lambda pair: pair[0]["area_px"], reverse=True)

            overlay = bev.copy()
            for item, contour in found:
                cv2.drawContours(overlay, [contour], -1, (0, 255, 0), 2)
                u = int(round(ORIGIN_U - item["y_left_m"] * PX_PER_M))
                v = int(round(ORIGIN_V - item["x_forward_m"] * PX_PER_M))
                cv2.circle(overlay, (u, v), 4, (0, 0, 255), -1)
            cv2.line(overlay, (int(ORIGIN_U), 0),
                     (int(ORIGIN_U), BEV_SIZE[1] - 1), (255, 0, 0), 1)

            result = {
                "schema": "blue_line_debug_min_v1",
                "stamp": message.header.stamp.to_sec(),
                "blue_pixels": int(cv2.countNonZero(mask)),
                "contour_count": int(len(found)),
                "threshold": {
                    "h_min": self.h_min, "h_max": self.h_max,
                    "s_min": self.s_min, "v_min": self.v_min,
                },
                "candidates": [item for item, _ in found],
            }
            self.metrics_pub.publish(String(
                data=json.dumps(result, allow_nan=False, sort_keys=True)))
            self.publish_image(self.bev_pub, bev, "bgr8", message.header)
            self.publish_image(self.mask_pub, mask, "mono8", message.header)
            self.publish_image(self.overlay_pub, overlay, "bgr8", message.header)
            rospy.loginfo_throttle(
                1.0, "blue_min pixels=%d candidates=%d",
                result["blue_pixels"], result["contour_count"])
        except (CvBridgeError, ValueError, TypeError) as exc:
            rospy.logwarn_throttle(2.0, "blue debug rejected frame: %s", str(exc))


if __name__ == "__main__":
    rospy.init_node("blue_line_debug_min", anonymous=False)
    BlueLineDebug()
    rospy.spin()
