#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Compute shared viewer geometry only during stopped side bay selection."""
from __future__ import division
import json
import math
import threading
import time
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from robot.common.contracts import decode,number
from robot.parking.side_selection import validate_settings
from robot.lidar.side_bays import measure


def selection_request(status,now):
    task = status.get('parking_timed') or {}
    if (not 0 <= now-number(status.get('stamp')) <= 1. or
            not status.get('enabled') or status.get('state') != 'TIMED_PARKING' or
            not task.get('select_by_lidar') or task.get('phase') != 'SELECT_SLOT'):
        return None
    return number((task.get('selection') or {}).get('started'))


def scan_observation(msg,cfg,request,now):
    options = cfg['parking_side_lidar']
    stamp = msg.header.stamp.to_sec()
    result = dict(source='side_parking_lidar',frame=msg.header.frame_id,stamp=stamp,
                  request_stamp=request,valid=False,valid_rays=0,counts=None,
                  line=None,boxes=[])
    if (msg.header.frame_id != cfg['scan_frame'] or stamp <= request or
            not 0 <= now-stamp <= options['scan_timeout_s']):
        return result
    amin,inc,rmin,rmax = map(number,(msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max))
    if not 0 < abs(inc) <= .1 or not 0 <= rmin < rmax or len(msg.ranges) > 5000:
        return result
    points = []
    for i,value in enumerate(msg.ranges):
        if math.isnan(value) or math.isinf(value) or not max(0.,rmin) < value <= rmax:
            continue
        angle = amin+i*inc
        points.append((value*math.cos(angle),value*math.sin(angle)))
    result['valid_rays'] = len(points)
    if len(points) < options['min_valid_rays']:
        return result
    geometry = measure(points)
    result.update(geometry)
    result['valid'] = geometry['line'] is not None and len(geometry['boxes']) == 2
    if result['valid']:
        result['counts'] = dict((b['name'],b['count']) for b in geometry['boxes'])
    return result


class Node(object):
    def __init__(self):
        self.cfg = rospy.get_param('/competition/config')
        self.options = validate_settings(self.cfg)
        self.lock,self.stop = threading.Lock(),threading.Event()
        self.scan,self.hint,self.last = None,None,None
        self.output = rospy.Publisher('/competition/parking_side_bays',String,queue_size=1)
        # Callback stores only the newest message. Geometry never blocks receipt.
        rospy.Subscriber(self.cfg['scan_topic'],LaserScan,self.receive_scan,queue_size=1,buff_size=2**22)
        rospy.Subscriber('/competition/parking_hint',String,self.receive_status,queue_size=1)
        rospy.on_shutdown(self.stop.set)
        self.worker = threading.Thread(target=self.work)
        self.worker.daemon = True
        self.worker.start()

    def receive_scan(self,msg):
        with self.lock:
            self.scan = msg

    def receive_status(self,msg):
        try:
            data,stamp = decode(msg.data,rospy.Time.now().to_sec(),1.)
            with self.lock:
                self.hint = data
        except (ValueError,KeyError,TypeError):
            with self.lock:
                self.hint = None

    def work(self):
        while not self.stop.is_set() and not rospy.is_shutdown():
            try:
                now = rospy.Time.now().to_sec()
                with self.lock:
                    msg,hint = self.scan,self.hint
                request = selection_request(hint,now) if hint else None
                if request is not None and msg is not None:
                    key = (request,msg.header.stamp.to_sec())
                    if key != self.last:
                        self.last = key
                        started = time.time()
                        try:
                            data = scan_observation(msg,self.cfg,request,now)
                        except (ValueError,KeyError,TypeError):
                            data = dict(source='side_parking_lidar',frame=self.cfg['scan_frame'],
                                        stamp=msg.header.stamp.to_sec(),request_stamp=request,
                                        valid=False,valid_rays=0,counts=None,line=None,boxes=[])
                        data['processing_ms'] = 1000.*(time.time()-started)
                        # Reject a frame that aged out during geometry processing.
                        if not 0 <= rospy.Time.now().to_sec()-data['stamp'] <= self.options['scan_timeout_s']:
                            data['valid'] = False
                        self.output.publish(String(data=json.dumps(data,allow_nan=False)))
            except (ValueError,KeyError,TypeError) as exc:
                rospy.logwarn_throttle(2,'Side bay scan rejected: %s',str(exc))
            self.stop.wait(1./self.options['processing_hz'])


if __name__ == '__main__':
    rospy.init_node('parking_side_lidar')
    Node()
    rospy.spin()
