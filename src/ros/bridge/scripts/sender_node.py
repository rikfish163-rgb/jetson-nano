#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
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

if __name__ == '__main__':
    sender = DataSender("/lidar/send")
    count = 0
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        complex_data = {
            "id": count,
            "frame": count,
            "object": "person",
            "box": [100, 200, 300, 400],
            "pose": {"x": 1.2, "y": 3.4, "yaw": 0.6}
        }
        sender.send(complex_data)
        count += 1
        rate.sleep()