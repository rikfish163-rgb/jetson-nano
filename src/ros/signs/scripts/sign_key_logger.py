#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from __future__ import print_function

import argparse
import os
import select
import sys
import threading
import time

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from sign_classifier_cv import SignClassifier
from std_msgs.msg import String
import json

class DataSender:
    def __init__(self, send_channel):
        self.pub = rospy.Publisher(send_channel, String, queue_size=1)
        rospy.sleep(0.2)

    def send(self, data):
        msg = String()
        msg.data = json.dumps(data)
        self.pub.publish(msg)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL = os.path.join(SCRIPT_DIR, "resnet18.onnx")
DEFAULT_CAPTURE = "/tmp/sign_key_logger_latest.jpg"


class LatestFrame(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frame = None
        self.stamp = None

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        with self.lock:
            self.frame = frame
            self.stamp = msg.header.stamp.to_sec() if msg.header.stamp else time.time()

    def snapshot(self):
        with self.lock:
            if self.frame is None:
                return None, None
            return self.frame.copy(), self.stamp


def classify_frame(classifier, frame, capture_path):
    cv2.imwrite(capture_path, frame)
    return classifier.classify_frame(frame)


def main():
    parser = argparse.ArgumentParser(description="Press Enter to classify the latest ROS camera frame.")
    parser.add_argument("--image-topic", default="/cam0/image_raw", help="ROS image topic")
    parser.add_argument("--model", default=DEFAULT_MODEL, help="ONNX model path")
    parser.add_argument("--capture-path", default=DEFAULT_CAPTURE, help="temporary captured frame path")
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("sign_key_logger", anonymous=True)
    latest = LatestFrame()
    rospy.Subscriber(args.image_topic, Image, latest.callback, queue_size=1, buff_size=2 ** 24)
    classifier = SignClassifier(args.model)

    print("ready: topic=%s" % args.image_topic)
    print("press Enter to classify latest frame, q then Enter to quit")
    label_sender = DataSender("/camera_hts/send")
    while not rospy.is_shutdown():
        readable, _, _ = select.select([sys.stdin], [], [], 0.2)
        if not readable:
            continue
        line = sys.stdin.readline()
        if line.strip().lower() == "q":
            break

        frame, stamp = latest.snapshot()
        if frame is None:
            print("no_frame_yet: waiting for %s" % args.image_topic)
            continue

        class_idx, confidence, label, speed, steering_angle = classify_frame(classifier, frame, args.capture_path)
	complex_data = {
            "class_idx": class_idx,
            "confidence": confidence,
            "label": label
        }
	label_sender.send(complex_data)
        print(
            "[%s] class_idx=%d class=%s confidence=%.4f speed=%s steering_angle=%s frame_stamp=%s"
            % (
                time.strftime("%H:%M:%S"),
                class_idx,
                label,
                confidence,
                speed,
                steering_angle,
                stamp,
            )
        
        )


if __name__ == "__main__":
    main()
