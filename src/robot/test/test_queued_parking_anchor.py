"""ROS adapter regression for parking anchors cached during an action.

The test uses the production ``Node.sign`` callback with a real Controller,
but constructs ``Node`` via ``__new__`` so no ROS publishers, subscribers, or
timers are created.
"""
from __future__ import division, print_function

import copy
import json
import os
import threading
import unittest

from robot.common.config import load_config
from robot.master.controller import Controller

try:
    import rospy as _rospy  # noqa: F401
except ImportError:
    _rospy = None

try:
    import test_ros_adapter as _ros_fixtures
except ImportError:
    _ros_fixtures = None


CONFIG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..',
                                           'config'))


class _FakeProjector(object):
    def __init__(self, point=(1.0, .1)):
        self.point = point
        self.calls = []

    def position(self, bounds):
        self.calls.append(copy.deepcopy(bounds))
        return self.point


@unittest.skipUnless(_rospy is not None and _ros_fixtures is not None,
                     'requires sourced ROS rospy Python environment')
class QueuedParkingAnchorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Reuse the adapter import path used by the existing ROS callback
        # tests.  This does not construct Node or start any ROS resources.
        _ros_fixtures.RosAdapterTests.setUpClass()
        cls.adapter = _ros_fixtures.RosAdapterTests.adapter

    def setUp(self):
        self.nodes = []

    def tearDown(self):
        for node in self.nodes:
            node.core.close()

    def make_node(self):
        clock = _ros_fixtures._Clock(2.0)
        fake_rospy = _ros_fixtures._FakeRospy(clock)
        self.adapter.rospy = fake_rospy
        node = self.adapter.Node.__new__(self.adapter.Node)
        node.cfg = load_config(CONFIG_DIR)
        self.assertTrue(node.cfg['parking_sign_association'])
        node.cfg.update(lidar_enabled=False, parking_mode='forward_plan')
        node.core = Controller(node.cfg)
        node.lock = threading.RLock()
        node.source_pose = lambda stamp: (10.0, 20.0, 0.0)
        node.parking_projector = _FakeProjector()
        node.last_sign_event = None
        node.last_state_event = None
        node.clock = clock
        node.fake_rospy = fake_rospy
        node.core.state = 'MANEUVER'
        node.core.action = 'RIGHT'
        node.core.action_started = 1.0
        node.core.parking_route_ready = True
        self.nodes.append(node)
        return node

    def sign(self, node, stamp, confidence=.99, bounds=None, now=None):
        node.clock.value = stamp if now is None else now
        payload = dict(stamp=stamp, label='PARKING',
                       confidence=confidence,
                       sign_bounds=bounds or [551, 71, 71, 84],
                       source='yolov5s')
        node.sign(_ros_fixtures._BoolMessage(json.dumps(payload)))

    def test_active_action_three_parking_votes_project_world_anchor(self):
        node = self.make_node()
        for stamp in (2.6, 2.7, 2.8):
            self.sign(node, stamp)

        self.assertEqual(node.core.action, 'RIGHT')
        self.assertIsNone(node.core.pending)
        self.assertEqual(node.core.next_direction, 'PARKING')
        self.assertEqual(node.core.sign_info['decision'],
                         'stored_next_direction')
        self.assertIsNotNone(node.core.parking_sign)
        self.assertEqual(node.core.parking_sign['stamp'], 2.8)
        self.assertEqual(node.core.parking_sign['point'], (11.0, 20.1))
        self.assertEqual(len(node.parking_projector.calls), 3)

    def test_before_grace_low_confidence_duplicate_and_expired_do_not_project(self):
        node = self.make_node()
        self.sign(node, 1.2)
        self.assertEqual(node.core.sign_info['decision'],
                         'action_sign_ignored')
        self.assertEqual(node.parking_projector.calls, [])

        node = self.make_node()
        self.sign(node, 2.6, confidence=.7)
        self.assertEqual(node.core.sign_info['decision'],
                         'low_confidence_or_unknown')
        self.assertEqual(node.parking_projector.calls, [])

        node = self.make_node()
        node.core.sign_stamp = 2.6
        self.sign(node, 2.6)
        self.assertEqual(node.core.sign_info['decision'], 'stale_or_duplicate')
        self.assertEqual(node.parking_projector.calls, [])

        node = self.make_node()
        node.clock.value = 4.0
        self.sign(node, 1.0, now=4.0)
        self.assertEqual(node.parking_projector.calls, [])
        self.assertIsNone(node.core.parking_sign)


if __name__ == '__main__':
    unittest.main()
