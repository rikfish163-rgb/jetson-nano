#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

CHANNELS = {
    "lidar_to_all": ["/lidar/send", "/all/receive"],
    "lane_to_all": ["/lane/send", "/all/receive"],
    "lane_to_camera_yihan": ["/lane/send", "/camera_yihan/receive"],
}

buffers = {ch: None for ch in CHANNELS}

def callback_factory(channel_name):
    def callback(msg):
        buffers[channel_name] = msg.data
    return callback

def main():
    rospy.init_node("universal_bridge_node")
    publishers = {}

    for ch_name, (sub_topic, pub_topic) in CHANNELS.items():
        rospy.Subscriber(sub_topic, String, callback_factory(ch_name))
        publishers[ch_name] = rospy.Publisher(pub_topic, String, queue_size=1)

    rate = rospy.Rate(200)
    rospy.loginfo("Bridge started")

    while not rospy.is_shutdown():
        for ch_name in CHANNELS:
            if buffers[ch_name] is not None:
                publishers[ch_name].publish(buffers[ch_name])
                buffers[ch_name] = None
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
