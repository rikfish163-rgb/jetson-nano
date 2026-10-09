#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Read-only stop evidence recorder; never publishes control commands."""
from __future__ import print_function
import json
import os
import threading
import time


def stop_key(data):
    command = data.get('command', [])
    if (len(command) != 2 or command[0] != 0 or
            data.get('state') in (None, 'WAIT_GREEN', 'DISABLED') or
            data.get('reason') == 'uturn_wait_start'):
        return None
    return data.get('state'), data.get('reason')


def main():
    import argparse
    import cv2
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if not os.path.isdir(args.output):
        os.makedirs(args.output)
    rospy.init_node('stop_frame_recorder', anonymous=True)
    bridge = CvBridge()
    lock = threading.Lock()
    latest, diagnostics = {}, {}
    status = [None]

    def frame(msg, name):
        with lock:
            latest[name] = msg

    def receive(msg, name):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                return
            with lock:
                if name == 'control':
                    status[0] = value
                else:
                    diagnostics[name] = dict(received_at=time.time(), data=value)
        except (ValueError, TypeError):
            pass

    topics = dict(front='/front/usb_cam/image_raw',
                  front_bev='/competition/debug/front_bev',
                  uturn_blue='/competition/debug/uturn_blue')
    subscribers = [rospy.Subscriber(topic, Image, frame, callback_args=name,
                   queue_size=1, buff_size=2**22) for name, topic in topics.items()]
    for name, topic in [('control', '/competition/status'),
                        ('perception', '/competition/perception_status'),
                        ('base', '/base_controller/status')]:
        subscribers.append(rospy.Subscriber(topic, String, receive,
                           callback_args=name, queue_size=1))
    rospy.loginfo('Stop frame recorder armed: %s', args.output)
    previous, last_saved = None, 0.
    while not rospy.is_shutdown():
        with lock:
            data = status[0]
            frames = dict(latest)
            diag = dict(diagnostics)
        key = stop_key(data) if data else None
        if key is None:
            previous = None
        elif key != previous and time.time()-last_saved >= 2.:
            front = frames.get('front')
            # Do not label an old camera image as the stop-time frame.
            if front is not None and abs(data['stamp']-front.header.stamp.to_sec()) <= 1.:
                name = '%d_%s' % (int(time.time()*1000), data['state'])
                folder = os.path.join(args.output, name)
                os.makedirs(folder)
                evidence = dict(control=data, diagnostics=diag, images={})
                for label, msg in frames.items():
                    stamp = msg.header.stamp.to_sec()
                    entry = dict(stamp=stamp, delta_from_stop_s=stamp-data['stamp'])
                    evidence['images'][label] = entry
                    if abs(entry['delta_from_stop_s']) > 1.:
                        entry['skipped'] = 'stale'
                        continue
                    try:
                        img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                        path = os.path.join(folder, label+'.jpg')
                        if not cv2.imwrite(path, img):
                            raise IOError('image write failed')
                        entry['file'] = path
                    except Exception as exc:
                        entry['error'] = str(exc)
                with open(os.path.join(folder, 'status.json'), 'w') as stream:
                    json.dump(evidence, stream, indent=2)
                rospy.loginfo('Stop evidence saved: %s (%s)', folder, key)
                previous, last_saved = key, time.time()
        time.sleep(.1)


if __name__ == '__main__':
    main()
