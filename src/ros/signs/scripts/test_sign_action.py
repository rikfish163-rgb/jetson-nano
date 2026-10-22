#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from __future__ import print_function

import argparse
import os

from sign_classifier_cv import SignClassifier


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL = os.path.join(SCRIPT_DIR, "resnet18.onnx")


def main():
    parser = argparse.ArgumentParser(description="Classify one sign image and print the matched drive command.")
    parser.add_argument("image", help="image path")
    parser.add_argument("--model", default=DEFAULT_MODEL, help="ONNX model path")
    args = parser.parse_args()

    classifier = SignClassifier(args.model)
    class_idx, confidence, label, speed, steering_angle = classifier.classify_image(args.image)
    print("class_idx=%d" % class_idx)
    print("class=%s" % label)
    print("confidence=%.4f" % confidence)
    print("speed=%s" % speed)
    print("steering_angle=%s" % steering_angle)


if __name__ == "__main__":
    main()
