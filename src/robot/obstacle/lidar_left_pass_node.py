#!/usr/bin/env python2
"""Isolated lidar left-pass trial with ROS image and MJPEG preview."""
from __future__ import division
import json
import math
import threading
import time
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np
import rosgraph
import rospy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

from robot.obstacle.lidar_left_pass_core import LeftPass, clusters


class PreviewHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            body = ('<html><head><meta name="viewport" content="width=device-width,initial-scale=1">'
                    '<title>Lidar left pass</title></head><body style="background:#111;color:white;font-family:sans-serif">'
                    '<h2>Lidar left pass</h2><img src="/stream" style="max-width:100%"></body></html>')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while not rospy.is_shutdown():
                    frame = self.server.node.jpeg
                    if frame:
                        self.wfile.write('--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n' % len(frame))
                        self.wfile.write(frame)
                        self.wfile.write('\r\n')
                    time.sleep(.1)
            except (IOError, RuntimeError):
                pass
        else:
            self.send_error(404)

    def log_message(self, *args):
        pass


class Node(object):
    def __init__(self):
        self.live = bool(rospy.get_param('~live', False))
        self.enabled = bool(rospy.get_param('~enabled', False))
        self.scan_topic = rospy.get_param('~scan_topic', '/scan')
        self.scan_frame = rospy.get_param('~scan_frame', 'laser_link')
        self.timeout = float(rospy.get_param('~scan_timeout', .5))
        self.scan_x = float(rospy.get_param('~scan_x', 0.))
        self.scan_y = float(rospy.get_param('~scan_y', 0.))
        self.scan_yaw = float(rospy.get_param('~scan_yaw', 0.))
        self.control = LeftPass(clearance=float(rospy.get_param('~clearance_m', .20)),
                                body_half=float(rospy.get_param('~body_width_m', .24))/2,
                                wheelbase=float(rospy.get_param('~wheelbase_m', .26)),
                                front=float(rospy.get_param('~front_extent_m', .33)),
                                rear=float(rospy.get_param('~rear_extent_m', .07)),
                                speed_raw=int(rospy.get_param('~speed_raw', 8)),
                                max_steer_raw=int(rospy.get_param('~max_steer_raw', 12)))
        if self.live and self.enabled:
            publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
            if dict(publishers).get('/control/cmd'):
                raise RuntimeError('another /control/cmd publisher exists')
        self.command = rospy.Publisher('/control/cmd' if self.live and self.enabled
                                       else '/lidar_left_pass/control_preview', String, queue_size=1)
        self.image = rospy.Publisher('/lidar_left_pass/image', Image, queue_size=1)
        self.status = rospy.Publisher('/lidar_left_pass/status', String, queue_size=1)
        self.lock = threading.RLock()
        self.scan = None
        self.scan_at = -1.
        self.jpeg = None
        self.seq = 0
        self.subscription = rospy.Subscriber(self.scan_topic, LaserScan, self.on_scan, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(.05), self.tick)
        self.server = HTTPServer((rospy.get_param('~preview_host', '0.0.0.0'),
                                  int(rospy.get_param('~preview_port', 8091))), PreviewHandler)
        self.server.node = self
        thread = threading.Thread(target=self.server.serve_forever)
        thread.daemon = True
        thread.start()
        rospy.on_shutdown(self.shutdown)

    def on_scan(self, msg):
        try:
            if msg.header.frame_id != self.scan_frame or not 2 <= len(msg.ranges) <= 20000:
                return
            stamp = msg.header.stamp.to_sec()
            completed = stamp + msg.time_increment*(len(msg.ranges)-1)
            now = rospy.Time.now().to_sec()
            if not 0 <= now-completed <= self.timeout:
                return
            raw = clusters(msg.ranges, msg.angle_min, msg.angle_increment,
                           msg.range_min, msg.range_max)
            co, si = math.cos(self.scan_yaw), math.sin(self.scan_yaw)
            def transform(p):
                return (self.scan_x+co*p[0]-si*p[1], self.scan_y+si*p[0]+co*p[1])
            detected = []
            for cluster in raw:
                points = [transform(p) for p in cluster['points']]
                x, y = transform((cluster['x'], cluster['y']))
                detected.append(dict(x=x, y=y, radius=cluster['radius'], points=points))
            points = []
            for i, distance in enumerate(msg.ranges):
                if not math.isnan(distance) and not math.isinf(distance) and msg.range_min <= distance <= min(msg.range_max, 3.):
                    angle = msg.angle_min+i*msg.angle_increment
                    points.append(transform((distance*math.cos(angle), distance*math.sin(angle))))
            with self.lock:
                self.scan = (detected, points)
                self.scan_at = completed
        except (ValueError, TypeError, OverflowError):
            rospy.logwarn_throttle(2., 'left-pass scan rejected')

    def tick(self, event):
        now = rospy.Time.now().to_sec()
        with self.lock:
            valid = self.scan is not None and 0 <= now-self.scan_at <= self.timeout
            detected, points = self.scan if valid else ([], [])
            if not valid:
                speed, steer, reason = 0, 0, 'scan_missing_or_stale'
            else:
                speed, steer, reason = self.control.step(detected, points)
            if not self.enabled:
                speed, steer, reason = 0, 0, 'disabled_preview'
            payload = dict(version=1, seq=self.seq, speed_raw=speed, steering_raw=steer)
            self.seq = (self.seq+1) % 256
            self.command.publish(String(data=json.dumps(payload)))
            target = self.control.target
            status = dict(phase=self.control.phase, reason=reason, live=self.live and self.enabled,
                          clearance_m=self.control.clearance, target=target and
                          dict(x=target['x'], y=target['y'], radius=target['radius']),
                          command=payload, scan_age_s=now-self.scan_at if valid else None)
            self.status.publish(String(data=json.dumps(status)))
            frame = self.draw(points, detected, status)
            message = Image()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = 'vehicle_rear_axle'
            message.height, message.width = frame.shape[:2]
            message.encoding, message.is_bigendian = 'bgr8', 0
            message.step = message.width*3
            message.data = frame.tostring()
            self.image.publish(message)
            ok, encoded = cv2.imencode('.jpg', frame)
            if ok:
                self.jpeg = encoded.tostring()

    def draw(self, points, detected, status):
        width, height, scale = 800, 700, 190.
        frame = np.zeros((height, width, 3), np.uint8)
        origin = (width//2, height-105)
        def pixel(p):
            return (int(origin[0]-p[1]*scale), int(origin[1]-p[0]*scale))
        for distance in (.5, 1., 1.5, 2.):
            cv2.circle(frame, origin, int(distance*scale), (45,45,45), 1)
        cv2.line(frame, pixel((0,-2)), pixel((0,2)), (60,60,60), 1)
        for p in points:
            cv2.circle(frame, pixel(p), 1, (160,160,160), -1)
        for cluster in detected:
            cv2.circle(frame, pixel((cluster['x'], cluster['y'])), 4, (0,180,255), -1)
        target = self.control.target
        if target:
            center = pixel((target['x'], target['y']))
            cv2.circle(frame, center, max(2,int(target['radius']*scale)), (0,0,255), 2)
            offset = self.control.body_half+self.control.clearance+target['radius']
            cv2.circle(frame, pixel((target['x'], target['y']+offset)), 6, (0,255,0), 2)
        x0, y0 = pixel((0,0))
        x1, y1 = pixel((self.control.front, self.control.body_half))
        x2, y2 = pixel((-self.control.rear, -self.control.body_half))
        cv2.rectangle(frame, (min(x1,x2),min(y1,y2)), (max(x1,x2),max(y1,y2)), (255,80,0), 2)
        cv2.putText(frame, 'FRONT +X / LEFT +Y', (15,28), cv2.FONT_HERSHEY_SIMPLEX, .7, (230,230,230), 2)
        cv2.putText(frame, 'phase=%s reason=%s' % (status['phase'],status['reason']),
                    (15,58), cv2.FONT_HERSHEY_SIMPLEX, .55, (230,230,230), 1)
        cv2.putText(frame, 'clearance=%.2fm  speed=%d steer=%d  %s' %
                    (status['clearance_m'],status['command']['speed_raw'],
                     status['command']['steering_raw'],'LIVE' if status['live'] else 'PREVIEW'),
                    (15,82), cv2.FONT_HERSHEY_SIMPLEX, .55, (230,230,230), 1)
        return frame

    def shutdown(self):
        if self.live and self.enabled:
            for _ in range(4):
                self.command.publish(String(data=json.dumps(dict(version=1,seq=self.seq,
                                                                  speed_raw=0,steering_raw=0))))
                time.sleep(.05)
        self.server.shutdown()


if __name__ == '__main__':
    rospy.init_node('lidar_left_pass')
    Node()
    rospy.spin()
