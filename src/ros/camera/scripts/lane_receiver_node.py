#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import json


OFFSET_DEADZONE = 10


class LaneReceiver:
    def __init__(self):
        rospy.Subscriber("/all/receive", String, self.callback)

    def callback(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            rospy.logwarn("JSON parse failed: %s" % str(e))
            return

        if data.get("source") != "lane":
            return

        # 蓝线数据：用于判断是否看到蓝线/停车线/路口触发线
        if "blue_points" in data:
            blue_points = int(data.get("blue_points", 0))
            blue_detected = bool(data.get("blue_detected", False))
            stop_line_trigger = bool(data.get("stop_line_trigger", False))

            rospy.loginfo(
                "BLUE -> blue_points: %d, blue_detected: %s, stop_line_trigger: %s"
                % (blue_points, str(blue_detected), str(stop_line_trigger))
            )

        # 白线数据：用于判断车辆相对车道中心的左右偏移
        if "white_points" in data:
            white_points = int(data.get("white_points", 0))
            lane_confidence = float(data.get("lane_confidence", 0.0))
            offset_error = int(data.get("offset_error", 9999))

            if offset_error == 9999:
                direction = "LOST_LINE"
            elif abs(offset_error) <= OFFSET_DEADZONE:
                direction = "CENTER"
            elif offset_error > OFFSET_DEADZONE:
                direction = "CAR_LEFT_NEED_RIGHT"
            else:
                direction = "CAR_RIGHT_NEED_LEFT"

            rospy.loginfo(
                "WHITE -> white_points: %d, lane_confidence: %.2f, offset_error: %d, direction: %s"
                % (white_points, lane_confidence, offset_error, direction)
            )


def main():
    rospy.init_node("lane_receiver_node", anonymous=True)
    LaneReceiver()
    rospy.loginfo("lane_receiver_node started, listening /all/receive")
    rospy.spin()


if __name__ == "__main__":
    main()