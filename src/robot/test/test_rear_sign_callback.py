"""Projection must not block control or apply to a changed reverse task."""
import ast
import copy
import json
import os
import unittest

from test_core import CONFIG
from test_sign_callback_scheduling import Box, TrackedLock
from robot.common.contracts import decode, number
from robot.common.geometry import world
from robot.parking.reverse_entry import ReverseStraightEntry


class RearSignCallbackTests(unittest.TestCase):
    def setUp(self):
        path=os.path.join(os.path.dirname(__file__),'../master/controller_node.py')
        with open(path) as stream:tree=ast.parse(stream.read())
        node_class=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Node')
        node_class.body=[n for n in node_class.body if isinstance(n,ast.FunctionDef) and n.name=='rear_sign']
        tree.body=[node_class]
        self.clock=10.5;self.lock=TrackedLock();self.logs=[]
        def log(*args):
            self.assertFalse(self.lock.held)
            self.logs.append(args)
        scope=dict(json=json,decode=decode,number=number,world=world,
            rospy=Box(Time=Box(now=lambda:Box(to_sec=lambda:self.clock)),
                      loginfo=log,logwarn_throttle=log))
        exec(compile(tree,path,'exec'),scope)
        self.node=scope['Node']()
        self.cfg=copy.deepcopy(CONFIG)
        self.entry=ReverseStraightEntry(self.cfg,(0.,0.,0.),10.,wait_for_sign=True)
        self.node.core=Box(state='PARKING',parking_entry=self.entry,estop=False,red=False)
        self.node.cfg=self.cfg;self.node.lock=self.lock
        self.node.source_pose=lambda stamp:(0.,0.,0.)
        self.hook=lambda:None
        def project(bounds):
            self.assertFalse(self.lock.held)
            self.hook()
            return (-.9,.24)
        self.node.rear_parking_projector=Box(position=project)
        self.msg=Box(data=json.dumps(dict(camera_source='rear',label='PARKING',
            confidence=.96,stamp=10.4,sign_bounds=[200,100,80,80])))

    def test_projection_and_logging_release_control_lock(self):
        self.node.rear_sign(self.msg)
        self.assertEqual(self.entry.inner.sign_anchor,(-.9,-.24))
        self.assertTrue(self.logs)

    def test_projection_that_finishes_stale_is_not_applied(self):
        def expire():self.clock=13.
        self.hook=expire
        self.node.rear_sign(self.msg)
        self.assertIsNone(self.entry.inner.sign_anchor)
        self.assertIsNone(self.entry.parking_started)

    def test_projection_cannot_retarget_replacement_task(self):
        replacement=ReverseStraightEntry(self.cfg,(0.,0.,0.),10.,wait_for_sign=True)
        def replace():self.node.core.parking_entry=replacement
        self.hook=replace
        self.node.rear_sign(self.msg)
        self.assertIsNone(self.entry.parking_started)
        self.assertIsNone(replacement.parking_started)

    def test_new_image_updates_existing_p_position_without_rearming(self):
        self.node.rear_sign(self.msg)
        started=self.entry.parking_started
        boundary=self.entry.geometry_after
        self.clock=10.8
        self.node.rear_parking_projector=Box(position=lambda bounds:(-.65,.18))
        message=Box(data=json.dumps(dict(camera_source='rear',label='PARKING',
            confidence=.97,stamp=10.7,sign_bounds=[195,95,90,90])))
        self.node.rear_sign(message)
        self.assertEqual(self.entry.inner.sign_anchor,(-.65,-.18))
        self.assertEqual(self.entry.parking_started,started)
        self.assertEqual(self.entry.geometry_after,boundary)
        self.assertEqual(self.entry.rear_sign['decision'],'parking_p_position_updated')

    def test_clipped_new_image_keeps_last_valid_position(self):
        self.node.rear_sign(self.msg)
        self.clock=10.8
        self.node.rear_parking_projector=Box(position=lambda bounds:None)
        message=Box(data=json.dumps(dict(camera_source='rear',label='PARKING',
            confidence=.97,stamp=10.7,sign_bounds=[195,95,90,90])))
        self.node.rear_sign(message)
        self.assertEqual(self.entry.inner.sign_anchor,(-.9,-.24))
        self.assertEqual(self.entry.rear_sign['decision'],'parking_p_observed_unprojected')


if __name__=='__main__':unittest.main()
