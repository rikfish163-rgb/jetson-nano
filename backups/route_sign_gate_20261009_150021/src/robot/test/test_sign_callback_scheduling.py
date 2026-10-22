"""Exercise the production callback without starting ROS or publishing motion."""
import ast
import json
import math
import os
import threading
import time
import unittest


class Box(object):
    def __init__(self, **values):
        self.__dict__.update(values)


class TrackedLock(object):
    def __init__(self):
        self.held = False

    def __enter__(self):
        self.held = True

    def __exit__(self, *args):
        self.held = False


class SignCallbackSchedulingTests(unittest.TestCase):
    def setUp(self):
        path = os.path.join(os.path.dirname(__file__), '../master/controller_node.py')
        with open(path) as source:
            self.source = source.read()
        tree = ast.parse(self.source)
        node_class = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Node')
        node_class.body = [n for n in node_class.body if isinstance(n, ast.FunctionDef) and n.name == 'sign']
        tree.body = [node_class]
        self.lock = TrackedLock()
        self.logs = []
        self.clock = 10.
        def log(*args):
            self.assertFalse(self.lock.held, 'logging must not hold the control lock')
            self.logs.append(args)
        scope = dict(time=time, math=math, json=json,
                     rospy=Box(Time=Box(now=lambda: Box(to_sec=lambda: self.clock)),
                               loginfo=log, logwarn_throttle=log),
                     decode=lambda data, now, timeout: (json.loads(data), 10.),
                     number=float, world=lambda pose, p: (pose[0]+p[0],pose[1]+p[1]))
        exec(compile(tree,path,'exec'),scope)
        self.node = scope['Node']()
        self.node.lock = self.lock
        self.node.cfg = dict(sign_timeout=1.,sign_confidence=.7)
        self.node.last_sign_event = None
        self.decision = 'stored_pending'
        self.core = Box(state='LANE',action=None,pending='PARKING',sign_stamp=10.,
                        next_direction=None,parking_sign=None,parking_entry=None)
        def observe(label, confidence, stamp, now, right_visible=None):
            self.observed_right_visible = right_visible
            self.core.sign_info = dict(label=label,decision=self.decision,stamp=stamp)
        self.core.observe_sign = observe
        self.node.core = self.core
        self.node.source_pose = lambda stamp: (1.,2.,0.)
        self.calls = 0
        self.hook = lambda: None
        def project(bounds):
            self.assertFalse(self.lock.held, 'projection must not hold the control lock')
            self.calls += 1
            self.hook()
            return (.5,.1)
        self.node.parking_projector = Box(position=project)
        self.msg = Box(data=json.dumps(dict(label='PARKING',confidence=.9,sign_bounds=[1,2,3,4])))

    def test_ignored_parking_does_no_projection(self):
        for decision in ('parking_wait_route_action','action_sign_ignored',
                         'stale_or_duplicate','parking_disabled','low_confidence_or_unknown'):
            self.decision = decision
            self.node.sign(self.msg)
        self.assertEqual(self.calls,0)
        self.assertIsNone(self.core.parking_sign)

    def test_all_candidates_preserve_visibility_below_acceptance_threshold(self):
        self.msg.data = json.dumps(dict(label='LEFT', confidence=.95,
            detections=[dict(label='left', confidence=.95), dict(label='right', confidence=.55)]))
        self.node.sign(self.msg)
        self.assertTrue(self.observed_right_visible)

    def test_empty_detection_list_reports_absence(self):
        self.msg.data = json.dumps(dict(label='', confidence=0., detections=[]))
        self.node.sign(self.msg)
        self.assertIs(self.observed_right_visible, False)

    def test_legacy_message_does_not_invent_candidate_visibility(self):
        self.node.sign(self.msg)
        self.assertIsNone(self.observed_right_visible)

    def test_accepted_parking_projects_outside_lock(self):
        self.node.sign(self.msg)
        self.assertEqual(self.calls,1)
        self.assertEqual(self.core.parking_sign,dict(point=(1.5,2.1),stamp=10.))

    def test_action_change_during_projection_discards_result(self):
        self.hook = lambda: setattr(self.core,'action','RIGHT')
        self.node.sign(self.msg)
        self.assertIsNone(self.core.parking_sign)

    def test_new_sign_during_projection_discards_result(self):
        self.hook = lambda: setattr(self.core,'sign_stamp',10.1)
        self.node.sign(self.msg)
        self.assertIsNone(self.core.parking_sign)

    def test_projection_that_becomes_stale_is_discarded(self):
        self.hook = lambda: setattr(self,'clock',12.)
        self.node.sign(self.msg)
        self.assertIsNone(self.core.parking_sign)

    def test_disabled_association_does_no_projection(self):
        self.node.parking_projector = None
        self.node.sign(self.msg)
        self.assertEqual(self.calls,0)

    def test_warmup_precedes_subscriptions_and_timers(self):
        warmup = self.source.index('self.parking_projector = ParkingSignProjector(self.cfg)')
        self.assertLess(warmup,self.source.index('rospy.Subscriber('))
        self.assertLess(warmup,self.source.index('rospy.Timer('))


if __name__ == '__main__':
    unittest.main()
