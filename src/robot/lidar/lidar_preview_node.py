#!/usr/bin/env python2
"""Read-only lidar processing node for the lidar team's standalone bench."""
import json
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from robot.lidar.scan import Scan


class Node(object):
    def __init__(self):
        self.cfg = rospy.get_param('/competition/config')
        self.output = rospy.Publisher('/modules/lidar/observation', String, queue_size=1)
        self.subscription = rospy.Subscriber(self.cfg['scan_topic'], LaserScan,
                                             self.observe, queue_size=1)

    def observe(self, msg):
        try:
            stamp = msg.header.stamp.to_sec()
            if not 0 <= rospy.Time.now().to_sec()-stamp <= self.cfg['sensor_timeout']:
                raise ValueError('stale scan')
            if len(msg.ranges) > 20000:
                raise ValueError('too many lidar rays')
            scan = Scan(msg.ranges, msg.angle_min, msg.angle_increment,
                        msg.range_min, msg.range_max, (0., 0., 0.),
                        self.cfg['lidar'], stamp)
            row = dict(stamp=stamp, obstacle_frame='vehicle_rear_axle', cluster_frame='lidar',
                       valid_rays=scan.valid_rays,
                       obstacles=scan.obstacles, clusters=scan.cluster_diagnostics)
            self.output.publish(String(data=json.dumps(row, allow_nan=False)))
        except (ValueError, TypeError, KeyError) as exc:
            rospy.logwarn_throttle(2., 'lidar preview rejected: %s', str(exc))


if __name__ == '__main__':
    rospy.init_node('lidar_preview')
    Node()
    rospy.spin()
