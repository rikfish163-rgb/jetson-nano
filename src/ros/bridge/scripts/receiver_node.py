#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import json

class LatestReceiver:
    def __init__(self, receive_channel):
        self.latest = None
        rospy.Subscriber(receive_channel, String, self.callback)
        rospy.sleep(0.2)

    def callback(self, msg):
        self.latest = msg.data

    def get_latest(self):
        return self.latest

if __name__ == '__main__':
    print("in")
    rospy.loginfo("in")
    receiver = LatestReceiver("/lidar/receive")
    while not rospy.is_shutdown():
        data_raw = receiver.get_latest()
        if data_raw is not None:
            data = json.loads(data_raw)
            rospy.loginfo("closest_point -> angle: %.2f бу  distance: %.3f m" % (data['angle'], data['distance']))
        rospy.sleep(0.05)
