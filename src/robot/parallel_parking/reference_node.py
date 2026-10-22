#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Camera-only P1 reference observations. Never publishes vehicle commands."""
import json
import threading
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from robot.parallel_parking.reference_vision import ReferenceVision,load_reference_config


class Node(object):
    def __init__(self):
        cfg=load_reference_config(rospy.get_param('~config_file'),
                                  rospy.get_param('~projection_file',''))
        cv2.setNumThreads(1)
        self.vision=ReferenceVision(cfg);self.bridge=CvBridge()
        self.lock=threading.Lock();self.latest=None
        self.pub=rospy.Publisher('/parallel_parking/p1_reference',String,queue_size=1)
        self.debug=rospy.Publisher('/parallel_parking/p1_reference_bev',Image,queue_size=1)
        rospy.Subscriber(rospy.get_param('~image_topic','/front/usb_cam/image_raw'),Image,
                         self.image,queue_size=1,buff_size=2**22)
        self.timer=rospy.Timer(rospy.Duration(.1),self.tick)

    def image(self,msg):
        with self.lock: self.latest=msg

    def tick(self,unused):
        with self.lock:msg=self.latest
        if msg is None: return
        stamp=msg.header.stamp.to_sec()
        if stamp<=self.vision.last_stamp: return
        if not 0<=rospy.Time.now().to_sec()-stamp<=.4: return
        try:
            output=self.vision.observe(self.bridge.imgmsg_to_cv2(msg,'bgr8'),stamp)
            if output is None: return
            result,bev=output
            self.pub.publish(String(data=json.dumps(result,allow_nan=False)))
            if bev is not None and self.debug.get_num_connections():
                debug=self.bridge.cv2_to_imgmsg(bev,'bgr8');debug.header=msg.header
                self.debug.publish(debug)
            rospy.loginfo_throttle(2.,'P1 reference: %s, candidates=%d',
                result['reason'],len(result['p1_lines']))
            if result.get('rejected_lines'):
                rospy.logwarn_throttle(2.,'P1 terminal width %.3f m differs from 0.36 m; check camera projection',
                    result['rejected_lines'][0]['width_m'])
        except (ValueError,cv2.error) as exc:
            rospy.logerr_throttle(2.,'P1 reference failed: %s',str(exc))


if __name__=='__main__':
    rospy.init_node('parallel_p1_reference')
    Node()
    rospy.spin()
