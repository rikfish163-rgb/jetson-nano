#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from __future__ import print_function

import os
import json

import cv2
import numpy as np

from sign_actions import LABELS, action_for_label, label_from_index


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL = os.path.join(SCRIPT_DIR, "resnet18.onnx")

IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


def softmax(logits):
    values = logits.astype(np.float32).reshape(-1)
    values = values - np.max(values)
    exp_values = np.exp(values)
    return exp_values / np.sum(exp_values)


def preprocess_bgr(frame):
    resized = cv2.resize(frame, (224, 224), interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    rgb = (rgb - IMAGENET_MEAN) / IMAGENET_STD
    chw = np.transpose(rgb, (2, 0, 1))
    return chw[np.newaxis, :, :, :].astype(np.float32)


class SignClassifier(object):
    def __init__(self, model_path=DEFAULT_MODEL, labels_path=None):
        self.model_path = model_path
        self.labels = list(LABELS)
        if labels_path:
            with open(labels_path) as stream:
                self.labels = json.load(stream)
            if self.labels not in (list(LABELS), list(LABELS)+['background']):
                raise ValueError('unsupported model label order: %r' % self.labels)
        self.net = cv2.dnn.readNetFromONNX(model_path)

    def classify_scores(self, frame):
        blob = preprocess_bgr(frame)
        self.net.setInput(blob, "input")
        logits = self.net.forward("logits")
        if int(np.asarray(logits).size) != len(self.labels):
            raise RuntimeError(
                "traffic sign model output has %d classes; expected %d" % (
                    int(np.asarray(logits).size), len(self.labels)))
        probs = softmax(logits)
        return dict((label,float(probs[i])) for i,label in enumerate(self.labels))

    def classify_sign(self, frame):
        scores = self.classify_scores(frame)
        probs = [scores[label] for label in self.labels]
        class_idx = int(np.argmax(probs))
        confidence = float(probs[class_idx])
        label = self.labels[class_idx]
        return class_idx, confidence, label

    def classify_frame(self, frame):
        """Legacy API retained for old offline tools.

        Formal perception code must use classify_sign() so classification is
        not coupled to the historical speed/steering lookup table.
        """
        class_idx, confidence, label = self.classify_sign(frame)
        speed, steering_angle = action_for_label(label)
        return class_idx, confidence, label, speed, steering_angle

    def classify_image(self, image_path):
        frame = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if frame is None:
            raise RuntimeError("failed to read image: %s" % image_path)
        return self.classify_frame(frame)
