#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
"""Publish a rectified metric BEV for the existing rear USB camera topic."""

from __future__ import print_function

import os
import sys
import threading

import cv2
import numpy as np
import rospy
import yaml

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

cv2.setNumThreads(1)


DEFAULT_IMAGE_TOPIC = "/rear/usb_cam/image_raw"
DEFAULT_OUTPUT_TOPIC = "/debug/rear_bev"
DEFAULT_CONFIG = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "calibration", "rear_bev.yaml"))


class RearBevNode(object):
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", DEFAULT_IMAGE_TOPIC)
        self.output_topic = rospy.get_param("~output_topic", DEFAULT_OUTPUT_TOPIC)
        self.config_path = rospy.get_param("~calibration_file", DEFAULT_CONFIG)
        self.processing_hz = max(
            0.0,
            float(rospy.get_param("~processing_hz", 0.0))
        )
        self.max_image_age = max(
            0.0,
            float(rospy.get_param("~max_image_age", 0.5))
        )

        self.bridge = CvBridge()
        self.publisher = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.frame_count = 0
        self._latest_image_msg = None
        self._latest_image_lock = threading.Lock()
        self._processing_lock = threading.Lock()
        self._last_processed_source_stamp = None
        self._processing_timer = None
        self._load_config(self.config_path)

        self.subscriber = rospy.Subscriber(
            self.image_topic,
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )

        if self.processing_hz > 0.0:
            self._processing_timer = rospy.Timer(
                rospy.Duration(1.0 / self.processing_hz),
                self._process_latest_image
            )

        rospy.loginfo("rear_bev_node started")
        rospy.loginfo("subscribe: %s", self.image_topic)
        rospy.loginfo("publish: %s", self.output_topic)
        rospy.loginfo("calibration: %s", self.config_path)
        rospy.loginfo("processing_hz: %.2f; max_image_age: %.3f",
                      self.processing_hz, self.max_image_age)
        rospy.loginfo("input: %d x %d; output: %d x %d",
                      self.image_width, self.image_height,
                      self.bev_width, self.bev_height)

    @staticmethod
    def _matrix(data, shape, name):
        try:
            array = np.asarray(data, dtype=np.float64).reshape(shape)
        except Exception:
            raise RuntimeError("invalid %s in BEV YAML" % name)
        if not np.isfinite(array).all():
            raise RuntimeError("%s contains NaN or Inf" % name)
        return array

    def _load_config(self, path):
        if not os.path.isfile(path):
            raise RuntimeError("BEV calibration file does not exist: %s" % path)

        with open(path, "r") as stream:
            data = yaml.safe_load(stream)
        if not isinstance(data, dict):
            raise RuntimeError("BEV calibration YAML is not a mapping: %s" % path)

        self.image_width = int(data["image_width"])
        self.image_height = int(data["image_height"])
        self.bev_width = int(data["bev_width"])
        self.bev_height = int(data["bev_height"])
        self.K = self._matrix(data["camera_matrix"], (3, 3), "camera_matrix")
        self.D = np.asarray(
            data["distortion_coefficients"], dtype=np.float64
        ).reshape(-1, 1)
        self.new_K = self._matrix(
            data["rectified_camera_matrix"], (3, 3),
            "rectified_camera_matrix"
        )
        self.H = self._matrix(
            data["homography_rectified_to_bev"], (3, 3),
            "homography_rectified_to_bev"
        )

        if self.image_width <= 0 or self.image_height <= 0:
            raise RuntimeError("invalid input image dimensions in BEV YAML")
        if self.bev_width <= 0 or self.bev_height <= 0:
            raise RuntimeError("invalid BEV dimensions in BEV YAML")

        self.map_x, self.map_y = cv2.initUndistortRectifyMap(
            self.K,
            self.D,
            None,
            self.new_K,
            (self.image_width, self.image_height),
            cv2.CV_16SC2,
        )

    @staticmethod
    def _source_stamp_key(msg):
        stamp = msg.header.stamp
        return int(stamp.secs), int(stamp.nsecs)

    def _process_image(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5.0, "cv_bridge failed: %s" % exc)
            return

        height, width = image.shape[:2]
        if width != self.image_width or height != self.image_height:
            rospy.logwarn_throttle(
                5.0,
                "BEV calibration expects %dx%d, got %dx%d; keep camera mode fixed",
                self.image_width,
                self.image_height,
                width,
                height,
            )
            return

        rectified = cv2.remap(
            image,
            self.map_x,
            self.map_y,
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )
        bev = cv2.warpPerspective(
            rectified,
            self.H,
            (self.bev_width, self.bev_height),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )

        try:
            output = self.bridge.cv2_to_imgmsg(bev, encoding="bgr8")
            output.header = msg.header
            self.publisher.publish(output)
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5.0, "BEV publish failed: %s" % exc)
            return

        self.frame_count += 1
        if self.frame_count == 1:
            rospy.loginfo("first rear BEV frame published")

    def _image_callback(self, msg):
        if self.processing_hz <= 0.0:
            self._processing_lock.acquire()
            try:
                self._process_image(msg)
            finally:
                self._processing_lock.release()
            return

        with self._latest_image_lock:
            self._latest_image_msg = msg

    def _process_latest_image(self, _event):
        if not self._processing_lock.acquire(False):
            return

        try:
            with self._latest_image_lock:
                msg = self._latest_image_msg
                if msg is None:
                    return

                source_stamp = self._source_stamp_key(msg)
                if source_stamp == self._last_processed_source_stamp:
                    return
                self._last_processed_source_stamp = source_stamp

            if self.max_image_age > 0.0:
                age = (rospy.Time.now() - msg.header.stamp).to_sec()
                if not 0 <= age <= self.max_image_age:
                    rospy.logwarn_throttle(
                        5.0,
                        "dropping stale rear image: age=%.3fs limit=%.3fs" % (
                            age,
                            self.max_image_age
                        )
                    )
                    return

            self._process_image(msg)
        finally:
            self._processing_lock.release()


def main():
    rospy.init_node("rear_bev_node", anonymous=False)
    try:
        RearBevNode()
    except Exception as exc:
        rospy.logfatal("rear_bev_node cannot start: %s", exc)
        return 2
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
