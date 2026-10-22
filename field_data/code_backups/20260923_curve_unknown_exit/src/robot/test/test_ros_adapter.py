#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""No-node tests for the ROS adapter callbacks.

The adapter is loaded with ``imp.load_source`` only when rospy is importable.
Node.__init__ is deliberately never called: all ROS-facing state is replaced
with a fake clock, parameter store, and logging sink, while the core remains a
real ROS-independent Controller.
"""

from __future__ import division, print_function

import copy
import json
import math
import os
import sys
import threading
import unittest
from collections import deque

import yaml


try:
    import rospy as _rospy  # noqa: F401
except ImportError:
    _rospy = None

try:
    import imp
except ImportError:
    imp = None


PACKAGE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
SOURCE_DIR = os.path.dirname(PACKAGE_DIR)
SCRIPT_PATH = os.path.join(PACKAGE_DIR, 'master', 'controller_node.py')
if SOURCE_DIR not in sys.path:
    sys.path.insert(0, SOURCE_DIR)

from robot.master.controller import Controller# noqa: E402


with open(os.path.join(PACKAGE_DIR, "config", "competition.yaml"), "r") as stream:
    CONFIG = yaml.safe_load(stream)


class _Clock(object):
    def __init__(self, value):
        self.value = float(value)


class _Stamp(object):
    def __init__(self, value):
        self.value = float(value)

    def to_sec(self):
        return self.value


class _FakeTime(object):
    def __init__(self, clock):
        self.clock = clock

    def now(self):
        return _Stamp(self.clock.value)

    def from_sec(self, value):
        return _Stamp(value)


class _FakeRospy(object):
    def __init__(self, clock):
        self.clock = clock
        self.Time = _FakeTime(clock)
        self.params = {"~enabled": False}
        self.warnings = []
        self.errors = []
        self.infos = []

    def loginfo(self, message, *args):
        self.infos.append((message,args))

    def get_param(self, name, default=None):
        return self.params.get(name, default)

    def logwarn_throttle(self, period, message, *args):
        self.warnings.append((period, message, args))

    def logerr_throttle(self, period, message, *args):
        self.errors.append((period, message, args))


class _Header(object):
    def __init__(self, stamp, frame_id):
        self.stamp = _Stamp(stamp)
        self.frame_id = frame_id


class _Vector3(object):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _Quaternion(object):
    def __init__(self, yaw=0.0):
        self.x = 0.0
        self.y = 0.0
        self.z = math.sin(yaw / 2.0)
        self.w = math.cos(yaw / 2.0)


class _PoseValue(object):
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.position = _Vector3(x, y, 0.0)
        self.orientation = _Quaternion(yaw)


class _PoseContainer(object):
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.pose = _PoseValue(x, y, yaw)


class _OdomMessage(object):
    def __init__(self, stamp, parent="odom", child="base_link",
                 x=0.0, y=0.0, yaw=0.0):
        self.header = _Header(stamp, parent)
        self.child_frame_id = child
        self.pose = _PoseContainer(x, y, yaw)


class _Drive(object):
    def __init__(self, speed, steering):
        self.speed = speed
        self.steering_angle = steering


class _AppliedMessage(object):
    def __init__(self, speed, steering):
        self.drive = _Drive(speed, steering)


class _BoolMessage(object):
    def __init__(self, value):
        self.data = value


@unittest.skipUnless(
    _rospy is not None and imp is not None,
    "requires sourced ROS rospy Python environment with imp.load_source",
)
class RosAdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            cls.adapter = imp.load_source(
                "robocup_competition_controller_node_test", SCRIPT_PATH
            )
        except ImportError as exc:
            raise unittest.SkipTest("ROS adapter dependencies unavailable: %s" % exc)

    def setUp(self):
        self.clock = _Clock(10.0)
        self.fake_rospy = _FakeRospy(self.clock)
        self.adapter.rospy = self.fake_rospy
        self.node = self.adapter.Node.__new__(self.adapter.Node)
        self.node.cfg = copy.deepcopy(CONFIG)
        self.node.cfg["pose_mode"] = "odom"
        self.node.core = Controller(self.node.cfg)
        self.addCleanup(self.node.core.close)
        self.node.lock = threading.RLock()
        self.node.odom_origin = None
        self.node.odom_frame = None
        self.node.applied = (0, 0)
        self.node.applied_at = -1.0
        self.node.last_sign_event = None
        self.node.last_state_event = None

    def test_scan_callback_uses_independent_lidar_timeout(self):
        n=self.node
        n.source_pose=lambda stamp:(0.,0.,0.)
        msg=type('ScanMessage',(),{})()
        msg.header=_Header(9.3,n.cfg['scan_frame'])
        msg.ranges=[float('inf')]*360
        msg.time_increment=.1/359
        msg.angle_min=-math.pi;msg.angle_increment=math.pi/180
        msg.range_min=.05;msg.range_max=6.
        n.scan(msg)  # Last ray 9.4, age .6 s: beyond visual timeout, within lidar.
        self.assertIsNotNone(n.core.scan)
        self.assertAlmostEqual(n.core.scan.completed_stamp,9.4)
        self.assertTrue(n.core.scan_ready(10.))
        self.clock.value=11.
        msg.header=_Header(10.,n.cfg['scan_frame'])  # .9 s stale, reject.
        n.scan(msg)
        self.assertAlmostEqual(n.core.scan.stamp,9.3)
        self.assertFalse(n.core.scan_ready(11.))

    def test_status_and_ros_log_include_stop_evidence_and_sign_decision(self):
        class Sink(object):
            def __init__(self):
                self.messages = []
            def publish(self,msg):
                self.messages.append(msg)
        n = self.node
        n.cfg['pose_mode'] = 'command_model'
        n.cfg['wait_green'] = False
        n.cfg.update(left_turn_radius=.60,right_turn_radius=.55)
        n.core.state = 'LANE'
        n.core.last_completed_action = 'RIGHT'
        n.seq,n.live,n.last_tick,n.status_at = 0,False,10.0,-1.0
        n.history = deque(maxlen=100)
        n.output,n.status = Sink(),Sink()
        n.publish_path = lambda now: None
        n.sign(_BoolMessage(json.dumps(dict(stamp=10.0,label='RIGHT',confidence=.4))))
        n.tick(None)
        self.assertFalse(self.fake_rospy.errors)
        status = json.loads(n.status.messages[-1].data)
        self.assertEqual(status['sign']['decision'],'low_confidence_or_unknown')
        self.assertEqual(status['sign']['label'],'RIGHT')
        self.assertEqual(status['obstacle_check']['kind'],'unknown')
        self.assertIn('next_direction',status)
        self.assertEqual(status['turn_action'],'RIGHT')
        self.assertEqual(status['turn']['turn_radius'],.55)
        self.assertEqual(status['turn_profiles']['left']['turn_radius'],.60)
        self.assertEqual(status['turn_profiles']['right']['turn_radius'],.55)
        self.assertTrue(any('competition_state' in msg for msg,args in self.fake_rospy.infos))

    def test_visual_direction_caches_without_scan(self):
        n=self.node
        n.core.state='LANE'
        self.assertIsNone(n.core.scan)
        for i in range(3):
            self.clock.value=10+i*.1
            n.sign(_BoolMessage(json.dumps(dict(stamp=self.clock.value,label='RIGHT',confidence=.99))))
        self.assertEqual(n.core.pending,'RIGHT')

    def test_parking_retains_unclipped_p_anchor_through_confirmation(self):
        n=self.node;n.core.state='LANE'
        n.cfg.update(parking_sign_association=True,parking_mode='forward_center',lidar_enabled=False)
        n.history=deque([(10.,(.1,0,0))])
        for i,bounds in enumerate(([551,71,71,84],[590,61,50,100],[620,54,20,111])):
            self.clock.value=10+i*.1
            n.sign(_BoolMessage(json.dumps(dict(stamp=self.clock.value,label='PARKING',
                confidence=.99,sign_bounds=bounds,source='yolov5s'))))
        self.assertEqual(n.core.pending,'PARKING')
        self.assertEqual(n.core.parking_sign['stamp'],10.)
        self.assertGreater(n.core.parking_sign['point'][0],.9)
        n.core.set_pose((.1,0,0),10.2)
        from blue_test_helpers import enter_blue_action
        enter_blue_action(n.core,'PARKING',10.6)
        self.assertEqual(n.core.parking_entry.sign_anchor,n.core.parking_sign['point'])
        self.assertTrue(n.core.parking_entry.associate)

    def test_startup_green_cannot_release_red_later(self):
        n=self.node;n.core.state='WAIT_GREEN'
        self.clock.value=10
        n.sign(_BoolMessage(json.dumps(dict(stamp=self.clock.value,label='GREEN',
            confidence=1.0,startup_only=True))))
        self.assertEqual(n.core.state,'STARTUP_STRAIGHT')
        self.assertEqual(n.core.sign_info['decision'],'green_release')
        n.core.red=True
        for i in range(4):
            self.clock.value=11+i*.1
            n.sign(_BoolMessage(json.dumps(dict(stamp=self.clock.value,label='GREEN',
                confidence=1.0,startup_only=True))))
        self.assertTrue(n.core.red)

    def test_startup_tag_is_only_for_green(self):
        n=self.node;n.core.state='WAIT_GREEN'
        for i in range(3):
            self.clock.value=10+i*.1
            n.sign(_BoolMessage(json.dumps(dict(stamp=self.clock.value,label='LEFT',
                confidence=.99,startup_only=True))))
        self.assertIsNone(n.core.pending)
        self.assertEqual(n.core.state,'WAIT_GREEN')

    def test_delayed_blue_uses_capture_pose_without_relaxing_lane_scan_timeout(self):
        self.node.cfg['ground_timeout'] = 1.25
        self.node.history = deque([(9.2,(0,0,0)),(9.3,(.025,0,0)),
                                   (9.4,(.05,0,0)),(10.0,(.2,0,0))])
        self.node.core.set_pose((.2,0,0),10.0)
        self.node.core.pending='STRAIGHT'
        data = dict(stamp=9.2,source='front',part='markers',frame='base_link',
                    markers=[dict(kind='junction',x=.6,y=0)],slots=[])
        for stamp in (9.2,9.3,9.4):
            data['stamp']=stamp
            data['markers'][0]['x']=.6-(stamp-9.2)*.25
            self.node.ground(_BoolMessage(json.dumps(data)))
        self.assertIsNotNone(self.node.core.marker)
        self.assertAlmostEqual(self.node.core.marker[0][0],.6)
        self.assertAlmostEqual(self.node.core.pose[0],.2)
        self.assertEqual(self.node.cfg['sensor_timeout'],.5)
        data.update(stamp=8.0)
        self.node.ground(_BoolMessage(json.dumps(data)))
        self.assertEqual(self.node.core.marker[1],9.4)
        self.assertTrue(self.fake_rospy.warnings)

    def test_future_odom_stamp_is_rejected_before_origin_mutation(self):
        original_pose = self.node.core.pose
        self.node.odom(_OdomMessage(11.0, parent="odom", x=4.0, y=5.0))

        self.assertIsNone(self.node.odom_origin)
        self.assertIsNone(self.node.odom_frame)
        self.assertEqual(self.node.core.pose, original_pose)
        self.assertTrue(self.fake_rospy.warnings)

    def lane_observation(self, **changes):
        data = dict(stamp=10.0, frame='base_link', confidence=.8,
                    points=[[.6, .1], [.8, .2], [1., .3]])
        data.update(changes)
        return _BoolMessage(json.dumps(data))

    def test_lane_observation_uses_own_confidence_even_if_legacy_arrives_first(self):
        self.node.history = deque()
        self.node.confidence, self.node.confidence_time = .01, 10.0
        self.node.lane_observation(self.lane_observation())
        self.assertEqual(len(self.node.core.lane), 3)
        self.assertAlmostEqual(self.node.core.lane_confidence, .8)
        self.node.conf(_BoolMessage(.99))
        self.clock.value = 10.1
        self.node.lane_observation(self.lane_observation(confidence=.1, stamp=10.1))
        self.assertAlmostEqual(self.node.core.lane_confidence, .1)

    def test_empty_lane_observation_clears_previous_path(self):
        self.node.history = deque()
        self.node.lane_observation(self.lane_observation())
        self.clock.value = 10.1
        self.node.lane_observation(self.lane_observation(points=[], confidence=0, stamp=10.1))
        self.assertEqual(self.node.core.lane, [])

    def test_curve_entry_reaches_controller_and_next_straight_frame_releases(self):
        self.node.history = deque()
        self.node.core.cfg['speed_raw']['lane']=20
        self.node.core.cfg['lane_curve_speed_raw']=16
        curve=dict(direction='RIGHT',entry_side='LEFT',entry_stamp=9.8)
        self.node.lane_observation(self.lane_observation(
            points=[[.5,-.02],[.7,-.02],[.9,-.02]],
            diagnostic=dict(curve_entry=curve)))
        self.assertEqual(self.node.core.lane_command(10.)[0],16)
        self.assertEqual(self.node.core.lane_curve['entry_side'],'LEFT')
        self.clock.value=10.1
        self.node.lane_observation(self.lane_observation(stamp=10.1,
            points=[[.5,0],[.7,0],[.9,0]],diagnostic=dict(curve_entry=None)))
        self.assertIsNone(self.node.core.lane_curve)
        self.assertEqual(self.node.core.lane_command(10.1)[0],20)

    def test_invalid_curve_entry_cannot_replace_current_observation(self):
        self.node.history=deque()
        self.node.lane_observation(self.lane_observation())
        self.clock.value=10.1
        for curve in ([],dict(direction='RIGHT',entry_side='RIGHT',entry_stamp=10.),
                      dict(direction='RIGHT',entry_side='LEFT',entry_stamp=11.),
                      dict(direction='RIGHT',entry_side='LEFT',entry_stamp=float('nan'))):
            self.node.lane_observation(self.lane_observation(stamp=10.1,
                diagnostic=dict(curve_entry=curve)))
            self.assertEqual(self.node.core.lane_stamp,10.)

    def test_curve_without_entry_cannot_drive_straight_or_reverse_without_support(self):
        self.node.history=deque()
        self.node.core.cfg['speed_raw']['lane']=20
        self.node.core.cfg['lane_curve_speed_raw']=16
        self.node.lane_observation(self.lane_observation(
            points=[[.5,.001],[.7,.001],[.9,.001]],
            diagnostic=dict(bend_direction='RIGHT',curve_entry=None)))
        speed,steer=self.node.core.lane_command(10.)
        self.assertEqual((speed,steer),(0,0.))
        self.assertEqual(self.node.core.reason,'curve_path_missing')
        self.assertIsNone(self.node.core.lane_curve['entry_side'])

    def test_logged_curve_packet_without_entry_keeps_recent_right_steering(self):
        self.node.history=deque()
        self.node.core.cfg['speed_raw']['lane']=20
        self.node.core.cfg['lane_curve_speed_raw']=16
        self.node.core.cfg['steering_command_scale_rad']=.1
        diagnostic=dict(bend_direction='RIGHT',curve_entry=None)
        self.node.lane_observation(self.lane_observation(
            points=[[.5,-.15],[.7,-.15],[.9,-.15]],diagnostic=diagnostic))
        speed,steer=self.node.core.lane_command(10.)
        self.assertEqual((speed,steer),(16,-.1))
        self.clock.value=10.1
        self.node.lane_observation(self.lane_observation(stamp=10.1,
            points=[[.5,0],[.7,0],[.9,0]],diagnostic=diagnostic))
        self.assertEqual(self.node.core.lane_command(10.1),(16,-.1))
        self.assertTrue(self.node.core.curve_steering['holding'])

    def test_inner_arc_curvature_reaches_steering_and_rejects_bad_geometry(self):
        self.node.history=deque()
        self.node.core.cfg['steering_command_scale_rad']=.1
        curve=dict(direction='RIGHT',entry_side='LEFT',entry_stamp=9.8,
                   curvature_m_inv=-.9,curvature_stamp=10.)
        self.node.lane_observation(self.lane_observation(diagnostic=dict(curve_entry=curve)))
        self.assertAlmostEqual(self.node.core.lane_command(10.)[1],-.1)
        self.clock.value=10.1
        for curvature,stamp in ((.9,10.1),(-.9,10.),(-5.,10.1),(float('nan'),10.1)):
            bad=dict(curve,curvature_m_inv=curvature,curvature_stamp=stamp)
            self.node.lane_observation(self.lane_observation(stamp=10.1,diagnostic=dict(curve_entry=bad)))
            self.assertEqual(self.node.core.lane_stamp,10.)

    def test_observed_boundaries_are_atomic_validated_and_cleared(self):
        self.node.history = deque()
        rows = [[.6,-.3],[.8,-.3],[1.,-.3]]
        self.node.lane_observation(self.lane_observation(boundaries={'RIGHT':rows}))
        original = self.node.core.lane_boundaries
        self.assertEqual(len(original['RIGHT']),3)
        self.clock.value = 10.1
        for bad in ({'RIGHT':[[.6,float('nan')]]}, {'OTHER':rows},
                    {'RIGHT':rows[::-1]}, {'RIGHT':rows[:2]}, []):
            self.node.lane_observation(self.lane_observation(stamp=10.1,boundaries=bad))
            self.assertEqual(self.node.core.lane_stamp,10.)
            self.assertEqual(self.node.core.lane_boundaries,original)
        self.node.lane_observation(self.lane_observation(stamp=10.1))
        self.assertEqual(self.node.core.lane_boundaries,{})

    def test_lane_observation_rejects_bad_inputs_without_overwriting_good_frame(self):
        self.node.history = deque()
        self.node.lane_observation(self.lane_observation())
        original = list(self.node.core.lane)
        for change in (dict(stamp=9.0), dict(stamp=11.), dict(frame='map'),
                       dict(confidence=float('nan')), dict(confidence=1.1),
                       dict(points=[[.6, float('inf')]]), dict(points='bad'),
                       dict(points=[[.6]]), dict(points=[[.6, 0]]*1001),
                       dict(points=[[9., 0]]), dict(points=[[.9, 0],[.6, 0]])):
            self.node.lane_observation(self.lane_observation(**change))
            self.assertEqual(self.node.core.lane, original)
        self.assertEqual(len(self.fake_rospy.warnings), 11)

    def test_lane_observation_enforces_producer_minimum_geometry(self):
        self.node.history = deque()
        for points in ([[.6,0]], [[.6,0],[.8,0]], [[.6,0],[.61,0],[.62,0]]):
            self.node.lane_observation(self.lane_observation(points=points))
            self.assertEqual(self.node.core.lane, [])
        self.node.cfg['lane_min_path_span'] = .1
        self.node.lane_observation(self.lane_observation(points=[[.6,0],[.65,0],[.7,0]]))
        self.assertEqual(len(self.node.core.lane),3)

    def test_lane_observation_rejects_deep_json_and_oversized_message(self):
        self.node.history = deque()
        for raw in ('['*2000+'0'+']'*2000, ' '*65537):
            self.node.lane_observation(_BoolMessage(raw))
            self.assertEqual(self.node.core.lane,[])
        self.assertEqual(len(self.fake_rospy.warnings),2)

    def test_changing_odom_parent_frame_is_rejected(self):
        self.node.odom(_OdomMessage(10.0, parent="odom", x=1.0, y=2.0))
        original_origin = self.node.odom_origin
        original_pose = self.node.core.pose

        self.clock.value = 10.1
        self.node.odom(_OdomMessage(10.1, parent="map", x=2.0, y=3.0))

        self.assertEqual(self.node.odom_frame, "odom")
        self.assertEqual(self.node.odom_origin, original_origin)
        self.assertEqual(self.node.core.pose, original_pose)
        self.assertTrue(self.fake_rospy.warnings)

    def test_applied_raw_over_contract_clears_command(self):
        self.node.applied = (8.0, 0.2)
        self.node.applied_at = 9.0

        self.node.applied_cb(_AppliedMessage(101.0, 0.0))
        self.assertEqual(self.node.applied, (0, 0))
        self.assertEqual(self.node.applied_at, 9.0)

        self.node.applied = (8.0, 0.2)
        self.node.applied_cb(_AppliedMessage(0.0, 23.0))
        self.assertEqual(self.node.applied, (0, 0))

    def test_estop_false_cannot_clear_while_enabled_but_clears_disabled(self):
        self.node.core.estop = True
        self.fake_rospy.params["~enabled"] = True
        self.node.estop(_BoolMessage(False))
        self.assertTrue(self.node.core.estop)

        self.fake_rospy.params["~enabled"] = False
        self.node.estop(_BoolMessage(False))
        self.assertFalse(self.node.core.estop)


if __name__ == "__main__":
    unittest.main()
