"""Lidar processing can run alone and never opens a motor command publisher."""
import copy
import imp
import json
import os
import unittest
from test_core import CONFIG


class Value(object):
    def __init__(self, **values):
        self.__dict__.update(values)


class LidarPreviewTest(unittest.TestCase):
    def setUp(self):
        path = os.path.join(os.path.dirname(__file__), '..', 'lidar', 'lidar_preview_node.py')
        self.module = imp.load_source('lidar_preview_fixture', path)
        self.messages, self.topics, self.warnings = [], [], []
        def publisher(topic, *args, **kwargs):
            self.topics.append(topic)
            return Value(publish=lambda msg: self.messages.append(json.loads(msg.data)))
        self.module.rospy = Value(get_param=lambda name: copy.deepcopy(CONFIG),
            Publisher=publisher, Subscriber=lambda *args, **kwargs: None,
            Time=Value(now=lambda: Value(to_sec=lambda: 10.)),
            logwarn_throttle=lambda *args: self.warnings.append(args))
        self.node = self.module.Node()

    def scan(self, stamp=10.):
        return Value(header=Value(stamp=Value(to_sec=lambda: stamp)), ranges=[3.]*360,
                     angle_min=-3.14159265, angle_increment=6.2831853/360,
                     range_min=.05, range_max=3.)

    def test_only_observation_publisher_and_metric_scan(self):
        self.node.observe(self.scan())
        self.assertEqual(self.topics, ['/modules/lidar/observation'])
        self.assertEqual(self.messages[0]['valid_rays'], 360)
        self.assertEqual(self.messages[0]['obstacle_frame'], 'vehicle_rear_axle')
        self.assertEqual(self.messages[0]['cluster_frame'], 'lidar')

    def test_stale_scan_is_rejected(self):
        self.node.observe(self.scan(1.))
        self.assertEqual(self.messages, [])
        self.assertTrue(self.warnings)

    def test_oversized_scan_is_rejected(self):
        scan = self.scan()
        scan.ranges = [3.]*20001
        self.node.observe(scan)
        self.assertEqual(self.messages, [])
        self.assertTrue(self.warnings)
