#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from __future__ import print_function

import os
import sys
import unittest


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)


class SignStringNodeTest(unittest.TestCase):
    def test_green_only_startup_does_not_load_model(self):
        import types
        import cv2
        import numpy as np
        from sign_string_node import SignStringNode
        saved = dict((name, sys.modules.get(name)) for name in (
            'rospy', 'cv_bridge', 'sensor_msgs', 'sensor_msgs.msg',
            'std_msgs', 'std_msgs.msg', 'sign_classifier_cv'))
        published = []
        def forbidden_model(*args):
            self.fail('green-only startup must not load the ONNX model')
        try:
            for name in saved:
                sys.modules[name] = types.ModuleType(name)
            rospy = sys.modules['rospy']
            rospy.Publisher = lambda *a, **k: type('Pub', (), {
                'publish': lambda s, msg: published.append(msg)})()
            rospy.Subscriber = lambda *a, **k: None
            rospy.loginfo = lambda *a: None
            rospy.logerr_throttle = lambda *a: self.fail(str(a))
            sys.modules['sensor_msgs.msg'].Image = object
            sys.modules['std_msgs.msg'].String = lambda **k: k
            sys.modules['cv_bridge'].CvBridge = lambda: type('Bridge', (), {
                'imgmsg_to_cv2': lambda s, msg, encoding: msg})()
            sys.modules['sign_classifier_cv'].SignClassifier = forbidden_model
            node = SignStringNode('/image', '/unused', '/missing.onnx',
                                  .6, 5, 4, 2, False, green_start_only=True)
            frame = np.zeros((200, 300, 3), dtype=np.uint8)
            frame[40:140, 100:200] = (0, 255, 0)
            for i in range(4):
                node.last_infer_time = 0
                node.image_cb(frame)
            self.assertTrue(node.green_start_latched)
            import json
            self.assertTrue(json.loads(published[-1]['data'])['confirmed'])
        finally:
            for name, module in saved.items():
                if module is None:
                    sys.modules.pop(name, None)
                else:
                    sys.modules[name] = module

    def test_muted_blue_board_keeps_context_in_crop(self):
        import cv2
        import numpy as np
        from sign_string_node import extract_sign_roi
        hsv = np.zeros((200, 300, 3), dtype=np.uint8)
        hsv[:, :, 2] = 145
        hsv[35:105, 115:185] = (115, 55, 140)
        roi = extract_sign_roi(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR))
        self.assertIsNotNone(roi)
        self.assertGreaterEqual(roi.shape[0], 120)
        self.assertGreaterEqual(roi.shape[1], 120)

    def test_model_labels_cover_all_seven_classes(self):
        from sign_string_node import (MultiFrameSignConfirmer,
                                      SUPPORTED_LABELS,
                                      normalize_model_label)

        raw_labels = ("red", "green", "straight", "left", "right",
                      "uturn", "park")
        expected = ("RED", "GREEN", "STRAIGHT", "LEFT", "RIGHT",
                    "UTURN", "PARKING")
        self.assertEqual(tuple(SUPPORTED_LABELS), expected)
        self.assertEqual(
            tuple(normalize_model_label(label) for label in raw_labels),
            expected)

        for raw_label, public_label in zip(raw_labels, expected):
            confirmer = MultiFrameSignConfirmer(0.60, 5, 4)
            result = None
            for index in range(4):
                result = confirmer.update(raw_label, 0.95, index)
            self.assertTrue(result["confirmed"])
            self.assertEqual(result["label"], public_label)

    def test_sign_roi_is_found_but_gray_lane_and_bottom_line_are_rejected(self):
        import numpy as np
        from sign_string_node import extract_sign_roi

        frame = np.full((200, 300, 3), 145, dtype=np.uint8)
        frame[35:105, 115:185] = (255, 0, 0)  # BGR blue sign board
        roi = extract_sign_roi(frame)
        self.assertIsNotNone(roi)
        self.assertGreaterEqual(roi.shape[0], 70)
        self.assertGreaterEqual(roi.shape[1], 70)

        lane_only = np.full((200, 300, 3), 145, dtype=np.uint8)
        lane_only[170:180, :] = (255, 0, 0)  # blue floor tape, not a sign
        self.assertIsNone(extract_sign_roi(lane_only))

    def test_invalid_detection_clears_old_votes(self):
        from sign_string_node import MultiFrameSignConfirmer

        confirmer = MultiFrameSignConfirmer(0.60, 5, 4)
        for index in range(3):
            self.assertFalse(confirmer.update("red", 0.95, index)["confirmed"])

        self.assertFalse(confirmer.update("", 0.0, 3)["confirmed"])
        result = confirmer.update("red", 0.95, 4)
        self.assertFalse(result["confirmed"])

    def test_green_color_region_is_detected_as_start_label(self):
        import numpy as np
        from sign_string_node import detect_green_start_label

        frame = np.zeros((100, 160, 3), dtype=np.uint8)
        frame[20:80, 50:110] = (0, 255, 0)

        self.assertEqual(detect_green_start_label(frame), "green")

    def test_gray_lane_does_not_create_green_start_label(self):
        import numpy as np
        from sign_string_node import detect_green_start_label

        frame = np.zeros((100, 160, 3), dtype=np.uint8)
        frame[70:85, :] = (190, 190, 190)

        self.assertIsNone(detect_green_start_label(frame))

    def test_high_confidence_result_publishes_plain_label(self):
        from sign_string_node import label_payload

        self.assertEqual(label_payload("red", 0.91, 0.60), "red")

    def test_low_confidence_result_is_not_published(self):
        from sign_string_node import label_payload

        self.assertIsNone(label_payload("red", 0.40, 0.60))

    def test_default_output_uses_the_hts_bridge_send_topic(self):
        from sign_string_node import DEFAULT_OUTPUT_TOPIC

        self.assertEqual(DEFAULT_OUTPUT_TOPIC, "/camera_hts/send")

    def test_default_classifier_rate_is_cpu_bounded(self):
        from sign_string_node import DEFAULT_INFERENCE_HZ, inference_interval

        self.assertAlmostEqual(DEFAULT_INFERENCE_HZ, 2.0)
        self.assertAlmostEqual(
            inference_interval(DEFAULT_INFERENCE_HZ, 1.0, False), 0.5)

    def test_sign_inference_is_throttled_after_green_start(self):
        from sign_string_node import inference_interval

        self.assertAlmostEqual(
            inference_interval(8.0, 1.0, False), 0.125)
        self.assertAlmostEqual(
            inference_interval(8.0, 1.0, True), 1.0)


if __name__ == "__main__":
    unittest.main()
