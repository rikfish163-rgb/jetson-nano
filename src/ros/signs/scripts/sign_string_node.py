#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from __future__ import print_function

import argparse
from collections import deque
import json
import math
import os
import time

from sign_actions import LABELS as MODEL_LABELS


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL = os.path.join(SCRIPT_DIR, "resnet18.onnx")
DEFAULT_IMAGE_TOPIC = "/front/usb_cam/image_raw"
# Feed the existing hts camera bridge.  It forwards this to
# /camera_hts/receive, where main.py consumes the fresh label.
DEFAULT_OUTPUT_TOPIC = "/camera_hts/send"
DEFAULT_INFERENCE_HZ = 2.0

# Keep one source of truth for the seven model classes.  The public payload
# uses upper-case labels, while the model and sign_actions.py use lower-case
# labels and call the last class ``park``.
LABEL_MAP = dict(
    (label, "PARKING" if label == "park" else label.upper())
    for label in MODEL_LABELS)
SUPPORTED_LABELS = tuple(LABEL_MAP[label] for label in MODEL_LABELS)

# The classifier was historically fed a colour-isolated sign image.  Feeding
# the whole 640x360 lane frame makes the gray floor/white line a confident
# false RED prediction.  Limit generic inference to a compact, saturated
# sign-like component in the upper field of view.  The bottom exclusion keeps
# blue floor tape from becoming a fake traffic sign.
GREEN_HSV_LOWER = (35, 60, 40)
GREEN_HSV_UPPER = (95, 255, 255)
GREEN_MIN_AREA_RATIO = 0.01
GREEN_MIN_COMPONENT_RATIO = 0.006
SIGN_SEARCH_BOTTOM_RATIO = 0.72
# Muted blue boards under venue lighting must retain the whole coloured face.
SIGN_MIN_SATURATION = 45
SIGN_MIN_VALUE = 40
SIGN_MIN_AREA_RATIO = 0.002
SIGN_MAX_ASPECT_RATIO = 4.5
SIGN_PADDING_RATIO = 0.50


def _valid_bgr_frame(frame):
    return (frame is not None and getattr(frame, "ndim", 0) == 3 and
            frame.shape[2] == 3 and frame.shape[0] > 0 and
            frame.shape[1] > 0)


def extract_sign_roi(frame, return_bounds=False):
    """Return the largest colour-sign candidate with a padded model crop.

    The seven-class model must not be asked to classify a lane-only frame.
    Traffic boards in this project have a saturated red, green, or blue
    region, so use those pixels as a conservative candidate mask and reject
    thin/elongated floor markings.  Green start detection remains a separate
    one-shot path in :func:`detect_green_start_label`.
    """
    if not _valid_bgr_frame(frame):
        return None

    import cv2
    import numpy as np

    height, width = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Include red at both ends of the OpenCV hue range, plus green and blue.
    red_low = cv2.inRange(
        hsv, np.array([0, SIGN_MIN_SATURATION, SIGN_MIN_VALUE]),
        np.array([12, 255, 255]))
    red_high = cv2.inRange(
        hsv, np.array([165, SIGN_MIN_SATURATION, SIGN_MIN_VALUE]),
        np.array([179, 255, 255]))
    green = cv2.inRange(
        hsv, np.array([30, SIGN_MIN_SATURATION, SIGN_MIN_VALUE]),
        np.array([100, 255, 255]))
    blue = cv2.inRange(
        hsv, np.array([90, SIGN_MIN_SATURATION, SIGN_MIN_VALUE]),
        np.array([135, 255, 255]))
    mask = cv2.bitwise_or(red_low, red_high)
    mask = cv2.bitwise_or(mask, green)
    mask = cv2.bitwise_or(mask, blue)
    search_bottom = int(height * SIGN_SEARCH_BOTTOM_RATIO)
    mask[search_bottom:, :] = 0
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    frame_area = float(height * width)
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area / frame_area < SIGN_MIN_AREA_RATIO:
            continue
        x, y, box_width, box_height = cv2.boundingRect(contour)
        if box_width <= 0 or box_height <= 0:
            continue
        aspect = max(float(box_width) / box_height,
                     float(box_height) / box_width)
        if aspect > SIGN_MAX_ASPECT_RATIO:
            continue
        if y + box_height / 2.0 >= search_bottom:
            continue
        candidates.append((area, x, y, box_width, box_height))

    if not candidates:
        return None
    area,x,y,box_width,box_height = max(candidates)
    pad_x = max(8, int(box_width * SIGN_PADDING_RATIO))
    pad_y = max(8, int(box_height * SIGN_PADDING_RATIO))
    x0,y0 = max(0,x-pad_x),max(0,y-pad_y)
    x1,y1 = min(width,x+box_width+pad_x),min(height,y+box_height+pad_y)
    crop=frame[y0:y1,x0:x1].copy()
    return (crop,(x,y,box_width,box_height)) if return_bounds else crop


def detect_green_start_label(frame, return_bounds=False):
    """Return ``green`` when *frame* contains a solid green marker.

    This is deliberately a conservative geometric/color check.  It returns no
    label for a gray lane or for a few isolated green pixels, and the caller
    still applies the existing multi-frame confirmation window.
    """
    if (frame is None or getattr(frame, "ndim", 0) != 3 or
            frame.shape[2] != 3):
        return None

    import cv2

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        GREEN_HSV_LOWER,
        GREEN_HSV_UPPER,
    )
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    height, width = mask.shape[:2]
    frame_area = float(height * width)
    if frame_area <= 0.0:
        return None
    if cv2.countNonZero(mask) / frame_area < GREEN_MIN_AREA_RATIO:
        return None

    contours_info = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours_info[-2]
    if not contours:
        return None
    largest_component_ratio = max(
        cv2.contourArea(contour) for contour in contours) / frame_area
    if largest_component_ratio < GREEN_MIN_COMPONENT_RATIO:
        return None
    if return_bounds:
        return cv2.boundingRect(max(contours,key=cv2.contourArea))
    return "green"


def label_payload(label, confidence, confidence_threshold):
    if float(confidence) < float(confidence_threshold):
        return None
    return str(label)

def _finite_float(value):
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    if math.isnan(number) or math.isinf(number):
        return None
    return number


def inference_interval(hz, post_green_hz, green_latched):
    """Return the minimum callback interval for the current sign phase."""
    rate = post_green_hz if green_latched else hz
    rate = _finite_float(rate)
    if rate is None or rate <= 0.0:
        raise ValueError("sign inference rate must be positive and finite")
    return 1.0 / rate


def normalize_model_label(label):
    if label is None:
        return ""
    return LABEL_MAP.get(str(label).strip().lower(), "")


class MultiFrameSignConfirmer(object):
    def __init__(self, confidence_threshold, window_size, required_votes):
        self.confidence_threshold = float(confidence_threshold)
        self.window_size = int(window_size)
        self.required_votes = int(required_votes)
        if not 0.0 <= self.confidence_threshold <= 1.0:
            raise ValueError("sign_confidence_threshold must be in [0, 1]")
        if self.window_size < 1:
            raise ValueError("sign_window_size must be positive")
        if not 1 <= self.required_votes <= self.window_size:
            raise ValueError("sign_required_votes must be in [1, window_size]")
        self.samples = deque(maxlen=self.window_size)

    def _winning_label(self):
        counts = {}
        for label, _confidence in self.samples:
            counts[label] = counts.get(label, 0) + 1

        winner = None
        winner_count = 0
        for label, _confidence in reversed(self.samples):
            count = counts[label]
            if count > winner_count:
                winner = label
                winner_count = count
        return winner, winner_count

    def update(self, label, confidence, timestamp=None):
        confidence = _finite_float(confidence)
        normalized = normalize_model_label(label)
        valid = bool(normalized and confidence is not None and
                     confidence >= self.confidence_threshold)
        if not valid:
            # A missing sign is a boundary between observations.  Do not let
            # old RED votes survive a lane-only interval and confirm a later
            # unrelated frame.
            self.samples.clear()
        else:
            self.samples.append((normalized, confidence))

        confirmed = False
        output_label = normalized
        output_confidence = confidence if confidence is not None else 0.0
        if valid:
            winner, winner_count = self._winning_label()
            if winner is not None and winner_count >= self.required_votes:
                matching = [item[1] for item in self.samples
                            if item[0] == winner]
                output_label = winner
                output_confidence = sum(matching) / float(len(matching))
                confirmed = True

        if timestamp is None:
            timestamp = time.time()
        return {
            "valid": bool(valid),
            "label": output_label,
            "confidence": float(output_confidence),
            "confirmed": bool(confirmed),
            "timestamp": float(timestamp),
        }


class SignStringNode(object):
    def __init__(self, image_topic, publish_topic, model_path,
                 confidence_threshold, window_size, required_votes, hz, debug,
                 post_green_hz=1.0, green_start_only=False):
        import rospy
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image
        from std_msgs.msg import String

        # This node is deliberately a low-priority perception worker.  The
        # white-line process must retain the CPU needed for its 20 Hz loop;
        # OpenCV's default pool otherwise turns one DNN inference into more
        # than one fully busy Nano core.
        for variable in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS",
                         "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS",
                         "OPENCV_FOR_THREADS_NUM"):
            os.environ[variable] = "1"
        import cv2
        try:
            cv2.setNumThreads(1)
        except AttributeError:
            pass
        self.bridge = CvBridge()
        self.green_start_only = bool(green_start_only)
        self.classifier = None
        if not self.green_start_only:
            from sign_classifier_cv import SignClassifier
            self.classifier = SignClassifier(model_path)
        self.pub = rospy.Publisher(publish_topic, String, queue_size=1)
        self.rospy = rospy
        self.string_msg = String
        self.confirmer = MultiFrameSignConfirmer(
            confidence_threshold, window_size, required_votes)
        self.debug = debug
        self.min_interval = inference_interval(hz, post_green_hz, False)
        self.post_green_interval = inference_interval(
            hz, post_green_hz, True)
        self.green_start_latched = False
        self.last_infer_time = 0.0

        rospy.Subscriber(image_topic, Image, self.image_cb, queue_size=1, buff_size=2 ** 24)

    def image_cb(self, msg):
        now = time.time()
        interval = (self.post_green_interval
                    if self.green_start_latched else self.min_interval)
        if now - self.last_infer_time < interval:
            return
        self.last_infer_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            label = (None if self.green_start_latched
                     else detect_green_start_label(frame))
            if label:
                class_idx, confidence = -1, 1.0
                source = "green_color"
                roi_found = True
            elif self.green_start_only:
                class_idx, confidence, label = -1, 0.0, ""
                source = "green_color_wait"
                roi_found = False
            else:
                sign_roi = extract_sign_roi(frame)
                if sign_roi is None:
                    class_idx, confidence, label = -1, 0.0, ""
                    source = "no_sign_roi"
                    roi_found = False
                else:
                    class_idx, confidence, label = \
                        self.classifier.classify_sign(sign_roi)
                    source = "onnx_sign_roi"
                    roi_found = True
            payload = self.confirmer.update(label, confidence, now)
            payload["class_idx"] = int(class_idx)
            payload["source"] = source
            payload["roi_found"] = bool(roi_found)
        except Exception as exc:
            self.rospy.logerr_throttle(
                1.0, "traffic sign inference failed: %s" % exc)
            return

        if (str(payload.get("label", "")).strip().upper() == "GREEN" and
                payload.get("valid") is True and
                payload.get("confirmed") is True):
            if not self.green_start_latched:
                self.rospy.loginfo(
                    "Confirmed GREEN start; throttling sign inference to %.2f Hz "
                    "while lane following owns motion",
                    1.0 / self.post_green_interval)
            self.green_start_latched = True

        self.pub.publish(self.string_msg(data=json.dumps(
            payload, separators=(",", ":"))))

        if self.debug:
            self.rospy.loginfo_throttle(
                1.0,
                "sign source=%s label=%s idx=%d conf=%.3f valid=%s confirmed=%s" % (
                    source,
                    payload["label"], class_idx, confidence,
                    payload["valid"], payload["confirmed"]))


def main():
    import rospy

    parser = argparse.ArgumentParser(
        description="Publish confirmed traffic-sign perception as JSON.")
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC,
                        help="ROS image topic")
    parser.add_argument("--publish-topic", default=DEFAULT_OUTPUT_TOPIC,
                        help="traffic-sign perception topic")
    parser.add_argument("--model", default=DEFAULT_MODEL, help="ONNX model path")
    parser.add_argument("--confidence-threshold", type=float, default=0.60,
                        help="fallback min confidence; ROS param takes priority")
    parser.add_argument("--window-size", type=int, default=5,
                        help="fallback voting window; ROS param takes priority")
    parser.add_argument("--required-votes", type=int, default=4,
                        help="fallback required votes; ROS param takes priority")
    parser.add_argument("--hz", type=float, default=DEFAULT_INFERENCE_HZ,
                        help="max inference/publish frequency")
    parser.add_argument("--post-green-hz", type=float, default=1.0,
                        help="inference frequency after confirmed GREEN")
    parser.add_argument("--debug", action="store_true",
                        help="print throttled classification status")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("sign_string_node", anonymous=False)
    image_topic = rospy.get_param("~image_topic", args.image_topic)
    publish_topic = rospy.get_param("~publish_topic", args.publish_topic)
    model_path = rospy.get_param("~model", args.model)
    confidence_threshold = rospy.get_param(
        "~sign_confidence_threshold", args.confidence_threshold)
    window_size = rospy.get_param("~sign_window_size", args.window_size)
    required_votes = rospy.get_param(
        "~sign_required_votes", args.required_votes)
    hz = rospy.get_param("~hz", args.hz)
    post_green_hz = rospy.get_param("~post_green_hz", args.post_green_hz)
    debug = rospy.get_param("~debug", args.debug)

    SignStringNode(
        image_topic=image_topic,
        publish_topic=publish_topic,
        model_path=model_path,
        confidence_threshold=confidence_threshold,
        window_size=window_size,
        required_votes=required_votes,
        hz=hz,
        debug=debug,
        post_green_hz=post_green_hz,
        green_start_only=rospy.get_param('~green_start_only', False),
    )
    rospy.loginfo(
        "sign_string_node started: image=%s output=%s model=%s "
        "threshold=%.3f window=%d votes=%d hz=%.2f post_green_hz=%.2f",
        image_topic, publish_topic, model_path,
        float(confidence_threshold), int(window_size), int(required_votes),
        float(hz), float(post_green_hz),
    )
    rospy.spin()


if __name__ == "__main__":
    main()
