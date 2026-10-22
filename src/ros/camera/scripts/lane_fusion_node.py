#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import rospy
import json
import time
from std_msgs.msg import String

FRONT_TOPIC = "/vision/front_lane"
REAR_TOPIC = "/vision/rear_lane"
FUSED_TOPIC = "/lane/send"

FRONT_TIMEOUT = 0.8
REAR_TIMEOUT = 0.8

MIN_CONF = 0.05
MAX_OFFSET_ABS = 400

front_data = None
rear_data = None
front_time = 0.0
rear_time = 0.0

last_offset = 0


def parse_msg(msg):
    try:
        return json.loads(msg.data)
    except Exception as e:
        rospy.logwarn("JSON parse failed: %s" % str(e))
        return None


def front_cb(msg):
    global front_data, front_time
    data = parse_msg(msg)
    if data is not None:
        front_data = data
        front_time = time.time()


def rear_cb(msg):
    global rear_data, rear_time
    data = parse_msg(msg)
    if data is not None:
        rear_data = data
        rear_time = time.time()


def get_offset(data):
    if data is None:
        return None

    if "offset_near" in data:
        return int(data.get("offset_near", 9999))

    if "offset_error" in data:
        return int(data.get("offset_error", 9999))

    return None


def get_conf(data):
    if data is None:
        return 0.0
    try:
        return float(data.get("lane_confidence", 0.0))
    except Exception:
        return 0.0


def is_valid(data, t, timeout):
    if data is None:
        return False

    if time.time() - t > timeout:
        return False

    offset = get_offset(data)
    conf = get_conf(data)

    if offset is None:
        return False

    if offset == 9999:
        return False

    if abs(offset) > MAX_OFFSET_ABS:
        return False

    if conf < MIN_CONF:
        return False

    return True


def fuse_once(pub):
    global last_offset

    front_valid = is_valid(front_data, front_time, FRONT_TIMEOUT)
    rear_valid = is_valid(rear_data, rear_time, REAR_TIMEOUT)

    front_offset = get_offset(front_data) if front_valid else None
    rear_offset = get_offset(rear_data) if rear_valid else None

    front_conf = get_conf(front_data) if front_valid else 0.0
    rear_conf = get_conf(rear_data) if rear_valid else 0.0

    curve_hint = "UNKNOWN"
    if front_data is not None:
        curve_hint = front_data.get("curve_hint", "UNKNOWN")

    if front_valid and rear_valid:
        # 前摄看前方，权重大；后摄用于稳定车身
        fused_offset = int(0.75 * front_offset + 0.25 * rear_offset)
        heading_error = int(front_offset - rear_offset)
        lane_confidence = min(1.0, 0.70 * front_conf + 0.30 * rear_conf)
        lane_state = "OK"

    elif front_valid:
        fused_offset = int(front_offset)
        heading_error = 0
        lane_confidence = front_conf
        lane_state = "FRONT_ONLY"

    elif rear_valid:
        # 后摄不能看前方，只用于前摄短暂失效时防止输出突变
        fused_offset = int(0.7 * last_offset + 0.3 * rear_offset)
        heading_error = 0
        lane_confidence = rear_conf * 0.6
        lane_state = "REAR_ONLY"

    else:
        fused_offset = int(last_offset)
        heading_error = 0
        lane_confidence = 0.0
        lane_state = "LOST_LINE"

    last_offset = fused_offset

    out = {
        "source": "lane",
        "offset_error": fused_offset,
        "heading_error": heading_error,
        "lane_confidence": lane_confidence,
        "curve_hint": curve_hint,
        "lane_state": lane_state,
        "front_valid": front_valid,
        "rear_valid": rear_valid,
        "front_offset": front_offset if front_offset is not None else 9999,
        "rear_offset": rear_offset if rear_offset is not None else 9999,
        "front_confidence": front_conf,
        "rear_confidence": rear_conf
    }

    pub.publish(json.dumps(out))

    rospy.loginfo(
        "FUSED offset=%d state=%s front=%s rear=%s heading=%d conf=%.2f curve=%s"
        % (
            fused_offset,
            lane_state,
            str(front_offset),
            str(rear_offset),
            heading_error,
            lane_confidence,
            curve_hint
        )
    )


def main():
    rospy.init_node("lane_fusion_node", anonymous=True)

    rospy.Subscriber(FRONT_TOPIC, String, front_cb)
    rospy.Subscriber(REAR_TOPIC, String, rear_cb)

    pub = rospy.Publisher(FUSED_TOPIC, String, queue_size=1)

    rospy.loginfo("lane_fusion_node started")
    rospy.loginfo("front=%s rear=%s fused=%s" % (FRONT_TOPIC, REAR_TOPIC, FUSED_TOPIC))

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        fuse_once(pub)
        rate.sleep()


if __name__ == "__main__":
    main()
