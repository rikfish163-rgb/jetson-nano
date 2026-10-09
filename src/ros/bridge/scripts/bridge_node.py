#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import json

CHANNELS = {
    "lidar_to_main": ["/lidar/send", "/lidar/receive"],
    "camera_yihan_to_main": ["/camera_yihan/send", "/camera_yihan/receive"],
    "camera_hts_to_main": ["/camera_hts/send", "/camera_hts/receive"],
    "main_to_control": ["/control/send", "/control/receive"],
    "image_distribute": ["/image/send", "/image/receive"]
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
