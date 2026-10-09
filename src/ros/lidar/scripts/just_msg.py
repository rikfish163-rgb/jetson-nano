#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import json
import math

MAX_DISPLAY_RANGE = 2.0

class DataSender:
    def __init__(self, topic_name):
        self.pub = rospy.Publisher(topic_name, String, queue_size=1)

    def send(self, data):
        msg = String()
        msg.data = json.dumps(data)
        self.pub.publish(msg)

sender = DataSender("/lidar/send")

def lidar_callback(scan_data):
    angles = np.linspace(scan_data.angle_min, scan_data.angle_max, len(scan_data.ranges))
    min_dist = 999.0
    closest_angle = 0.0
    usable_count = 0

    for i, r in enumerate(scan_data.ranges):
        if not np.isfinite(r) or r < max(0.05, scan_data.range_min) or r > scan_data.range_max:
            continue
        usable_count += 1
        angle = angles[i]
        if r < min_dist:
            min_dist = r
            closest_angle = angle

    # Always publish a heartbeat when the scan contains at least one usable
    # range.  The old code silently published nothing when every return was
    # farther than MAX_DISPLAY_RANGE (normal in an open room), which made the
    # parking safety adapter look stale even though /scan was healthy.  Keep
    # the source contract explicit: a usable far return is a clear candidate,
    # while an all-invalid scan remains invalid and must stop the vehicle.
    if usable_count > 0:
        representative_distance = min(min_dist, MAX_DISPLAY_RANGE)
        msg_data = {
            "valid": True,
            "angle": round(math.degrees(closest_angle), 2),
            "distance": round(representative_distance, 3)
        }
    else:
        msg_data = {
            "valid": False,
            "angle": None,
            "distance": None
        }
    sender.send(msg_data)
    rospy.loginfo_throttle(2.0, msg_data)

if __name__ == '__main__':
    rospy.init_node('lidar_data_node')
    rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)
    rospy.spin()
