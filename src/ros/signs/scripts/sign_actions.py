#!/usr/bin/env python
# -*- coding: UTF-8 -*-

LABELS = ["red", "green", "straight", "left", "right", "uturn", "park"]

SIGN_ACTIONS = {
    "red": {"speed": 0, "steering_angle": 0},
    "green": {"speed": 20, "steering_angle": 0},
    "straight": {"speed": 20, "steering_angle": 0},
    "left": {"speed": 20, "steering_angle": 10},
    "right": {"speed": 20, "steering_angle": -10},
    "uturn": {"speed": 20, "steering_angle": 11},
    "park": {"speed": 0, "steering_angle": 0},
}


def normalize_label(label):
    return str(label).strip().lower()


def label_from_index(class_idx):
    idx = int(class_idx)
    if idx < 0 or idx >= len(LABELS):
        raise ValueError("class index out of range: %s" % class_idx)
    return LABELS[idx]


def action_for_label(label):
    normalized = normalize_label(label)
    if normalized not in SIGN_ACTIONS:
        raise ValueError("unknown sign label: %s" % label)
    action = SIGN_ACTIONS[normalized]
    return action["speed"], action["steering_angle"]


def action_for_index(class_idx):
    label = label_from_index(class_idx)
    speed, steering_angle = action_for_label(label)
    return label, speed, steering_angle
