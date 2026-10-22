#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Sensor-only parking producer. Never publishes a chassis/control command."""
import argparse
import json
import os
import threading
import time
import cv2
import rospy
import rospkg
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from robot.common.config import read_mapping
from robot.common.contracts import number,decode,validate_config
from robot.parking.timed_core import validate_scene
from robot.parking.timed_vision import BayLineVision,both_boundary_curves,PairBuffer


class Node(object):
    def __init__(self):
        self.cfg = rospy.get_param('/competition/config')
        validate_config(self.cfg)
        if self.cfg.get('parking_mode') != 'timed_sequence':
            raise ValueError('timed parking vision requires timed_sequence mode')
        self.options = self.cfg['parking_timed']
        cv2.setNumThreads(1)
        root = rospkg.RosPack().get_path('robocup_competition')
        projection = read_mapping(os.path.join(root,'parallel_parking/reference_camera.yaml'))
        camera = dict(self.cfg['front_camera'],H=projection['front_camera']['H'])
        self.vision = BayLineVision(dict(self.cfg,front_camera=camera),argparse.Namespace(**self.options))
        self.bridge,self.lock = CvBridge(),threading.Lock()
        self.buffer = PairBuffer()
        self.stop = threading.Event()
        self.output = rospy.Publisher('/competition/parking_timed_scene',String,queue_size=1)
        self.debug = rospy.Publisher('/competition/debug/parking_timed',Image,queue_size=1)
        rospy.Subscriber(self.cfg['front_image_topic'],Image,self.image,queue_size=3,buff_size=2**22)
        rospy.Subscriber(self.cfg['lane_observation_topic'],String,self.lane,queue_size=3)
        rospy.on_shutdown(self.stop.set)
        self.worker = threading.Thread(target=self.work)
        self.worker.daemon = True
        self.worker.start()

    def image(self,msg):
        stamp = msg.header.stamp.to_sec()
        with self.lock:
            self.buffer.put('image',stamp,msg)

    def lane(self,msg):
        try:
            data,stamp = decode(msg.data,rospy.Time.now().to_sec(),self.cfg.get('ground_timeout',1.25))
            if data.get('frame') != self.cfg['lane_frame']:
                raise ValueError('lane frame mismatch')
            with self.lock:
                self.buffer.put('lane',stamp,data)
        except (ValueError,KeyError,TypeError) as exc:
            rospy.logwarn_throttle(2,'Timed parking lane rejected: %s',str(exc))

    def work(self):
        while not self.stop.is_set() and not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            with self.lock:
                # Sensor production must not wait for best-effort controller
                # telemetry: a delayed status can consume the entire entry timeout.
                pair = self.buffer.newest(now,self.cfg.get('ground_timeout',1.25))
            if pair is None:
                self.stop.wait(.005)
                continue
            image,lane = pair
            try:
                started = time.time()
                stamp = image.header.stamp.to_sec()
                frame = self.bridge.imgmsg_to_cv2(image,'bgr8')
                self.vision.set_lane_observation(lane)
                count,debug = self.vision.observe(frame)
                curves = both_boundary_curves(lane.get('boundaries',{}),
                    self.options['curve_min_curvature'],self.options['curve_min_turn_deg'])
                data = dict(curves,source='timed_parking_front',frame='base_link',stamp=stamp,
                            lines=count,lane_observation=lane,
                            processing_ms=1000.*(time.time()-started))
                validate_scene(data,self.cfg,rospy.Time.now().to_sec())
                self.output.publish(String(data=json.dumps(data,allow_nan=False)))
                if self.debug.get_num_connections():
                    result = self.bridge.cv2_to_imgmsg(debug,'bgr8')
                    result.header = image.header
                    self.debug.publish(result)
            except (ValueError,KeyError,TypeError,cv2.error,CvBridgeError) as exc:
                rospy.logwarn_throttle(2,'Timed parking frame rejected: %s',str(exc))
            # Keep the sensor warm without processing every camera frame.
            self.stop.wait(.2)


if __name__ == '__main__':
    rospy.init_node('parking_timed_vision')
    Node()
    rospy.spin()
