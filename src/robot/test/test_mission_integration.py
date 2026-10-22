"""Unified mission transitions; pure core tests, no ROS nodes or actuators."""
from __future__ import division
import copy
import math
import os
import unittest
import xml.etree.ElementTree as ET

from test_auto_parking import CONFIG, Scan
from robot.master.controller import Controller
from robot.master.controller import _SimpleFuture
from robot.common.geometry import bicycle
from robot.common.geometry import local
from robot.common.geometry import wrap
from robot.common.geometry import inside_slot
from robot.common.planning import intersection_path


class MissionIntegrationTests(unittest.TestCase):
    def test_full_launch_enables_mission_but_not_actuators_by_default(self):
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        full = ET.parse(os.path.join(root,'launch','full.launch')).getroot()
        defaults = dict((arg.get('name'),arg.get('default')) for arg in full.findall('arg'))
        self.assertEqual(defaults['blue_default_straight'], 'true')
        for key in ('wait_green','parking_enabled','start_rear'):
            self.assertEqual(defaults[key], 'true')
        self.assertEqual(defaults['parking_slot'], 'AUTO')
        self.assertEqual(defaults['sign_ttl'], '0')
        for key in ('live','enabled','start_actuators'):
            self.assertEqual(defaults[key], 'false')
        forwarded = dict((a.get('name'),a.get('value')) for a in full.find('include').findall('arg'))
        self.assertEqual(forwarded['blue_default_straight'], '$(arg blue_default_straight)')

    def core(self, **overrides):
        cfg = copy.deepcopy(CONFIG)
        cfg.update(wait_green=False, sign_ttl=0, parking_slot='AUTO',
                   blue_default_straight=True, intersection_wait_s=1.0)
        cfg.update(overrides)
        c = Controller(cfg)
        c.scan = Scan()
        c.scan.stamp = 1.0
        self.addCleanup(c.close)
        return c

    def blue(self, c, stamp=1.0, kind='junction', x=.3, y=0):
        c.observe_ground(dict(source='front', part='markers', slots=[],
                             markers=[dict(kind=kind, x=x, y=y)]), stamp)

    def sign(self, c, label, start=1.1):
        for i in range(3):
            now = start+i*.1
            c.observe_sign(label, .99, now, now)

    def test_cached_straight_waits_then_searches_without_white(self):
        c = self.core()
        self.sign(c, 'STRAIGHT', .1)
        self.blue(c)
        speed, steer = c.tick(1.0)
        self.assertEqual(speed, 0)
        self.assertEqual(steer, 0)
        self.assertEqual((c.state, c.action), ('INTERSECTION_WAIT', 'STRAIGHT'))
        self.assertEqual(c.action_source, 'sign')
        self.assertIsNone(c.marker)
        c.scan.stamp = 2.0
        c.observe_lane([],0.0,2.0)
        self.assertEqual(c.tick(2.0),(26,0.0))
        self.assertEqual((c.state,c.action),('MANEUVER','STRAIGHT'))

    def test_cached_left_has_priority_and_keeps_explicit_wait(self):
        c = self.core()
        c.pending = 'LEFT'
        self.blue(c)
        self.assertEqual(c.tick(1.0), (0, 0.0))
        self.assertEqual((c.state, c.action), ('INTERSECTION_WAIT', 'LEFT'))
        self.assertEqual(c.action_source, 'sign')

    def test_far_short_expired_or_offroad_blue_does_not_dispatch(self):
        for kwargs, now in ((dict(x=1.4), 1.0), (dict(kind='tick'), 1.0),
                            ({}, 10.0), (dict(y=.9), 1.0)):
            c = self.core()
            self.blue(c, **kwargs)
            c.scan.stamp = now
            c.tick(now)
            self.assertIsNone(c.action)

    def test_straight_preserves_red_green_estop_and_obstacle_stops(self):
        for name in ('red', 'estop', 'green', 'obstacle'):
            c = self.core(wait_green=name == 'green')
            self.sign(c, 'STRAIGHT', .1)
            self.blue(c)
            c.red, c.estop = name == 'red', name == 'estop'
            if name == 'obstacle':
                c.scan.obstacles = [(.4, 0)]
            self.assertEqual(c.tick(1.0), (0, 0.0))

    def test_blue_never_interrupts_active_left_or_uturn(self):
        for action in ('LEFT', 'UTURN'):
            c = self.core()
            c.pending = action
            self.blue(c)
            c.tick(1.0)
            self.blue(c, stamp=1.1, x=.5, y=.5)
            c.scan.stamp = 1.1
            c.tick(1.1)
            self.assertEqual(c.action, action)

    def test_parking_requires_fresh_votes_after_every_direction(self):
        for action in ('LEFT', 'RIGHT', 'STRAIGHT', 'UTURN'):
            c = self.core()
            c.action, c.state = action, 'UTURN' if action == 'UTURN' else 'MANEUVER'
            self.sign(c, 'PARKING')
            self.assertIsNone(c.next_direction)
            self.assertEqual(c.action, action)
            c.resume_lane()
            self.assertIsNone(c.pending)
            self.sign(c, 'PARKING',1.5)
            self.assertEqual(c.pending, 'PARKING')
            self.blue(c, stamp=2.0)
            c.scan.stamp = 2.0
            c.tick(2.0)
            self.assertEqual(c.state, 'PARKING_SCAN')

    def test_disabled_parking_is_not_queued(self):
        c = self.core(parking_enabled=False)
        c.action, c.state = 'LEFT', 'MANEUVER'
        self.sign(c, 'PARKING')
        self.assertIsNone(c.next_direction)

    def test_next_sign_cannot_overwrite_first_queued_action(self):
        c = self.core()
        c.action, c.state = 'LEFT', 'MANEUVER'
        self.sign(c, 'PARKING')
        self.sign(c, 'RIGHT', 2.0)
        self.assertIsNone(c.next_direction)

    def test_unarmed_blue_dispatches_default_straight(self):
        c = self.core()
        self.blue(c)
        c.tick(1.0)
        self.assertEqual(c.action,'STRAIGHT')
        self.assertEqual(c.action_source,'blue_default')
        self.assertIsNone(c.marker)

    def test_cached_straight_crosses_missing_white_and_releases_once(self):
        c = self.core()
        self.sign(c, 'STRAIGHT', .1)
        self.blue(c)
        for i in range(500):
            now = 1+i*.05
            c.scan.stamp = now
            visible = c.pose[0] > .20
            c.observe_ground(dict(source='front',part='markers',slots=[],markers=
                [dict(kind='junction',x=.3,y=0)] if c.pose[0] > .72 else []),now)
            c.observe_lane([(.3,0),(.6,0)] if visible else [], .99 if visible else 0.0, now)
            speed, steer = c.tick(now)
            c.set_pose(bicycle(c.pose, speed*.008*.05, steer, c.cfg['wheelbase']), now)
            if c.state == 'LANE' and c.completed_at_pose is not None:
                break
        self.assertEqual(c.state, 'LANE', c.reason)
        self.assertIsNotNone(c.completed_at_pose)
        self.assertIsNone(c.action)
        # Re-observing the consumed physical line must not trigger another action.
        xy = local(c.pose, c.consumed_marker)
        self.blue(c, now+.1, x=xy[0], y=xy[1])
        self.assertIsNone(c.marker)

    def test_auto_accepts_both_bay_kinds_with_correct_dimensions(self):
        c = self.core()
        c.observe_ground(dict(source='front', part='slots', markers=[], slots=[
            dict(x=.6,y=-.45,yaw=math.pi,kind='parallel'),
            dict(x=1.5,y=-.55,yaw=math.pi/2,kind='perpendicular')]), 1.0)
        self.assertEqual(len(c.parking_candidates), 2)
        parallel, perpendicular = c.parking_candidates
        self.assertEqual(parallel['kind'], 'parallel')
        self.assertEqual(parallel['length'], c.cfg['slots']['P1']['length'])
        self.assertAlmostEqual(parallel['pose'][2], 0)
        self.assertEqual(perpendicular['length'], c.cfg['slots']['P4']['length'])
        c.slot, c.slot_locked = parallel, True
        c.observe_ground(dict(source='rear', part='slots', markers=[], slots=[
            dict(x=.6,y=-.45,yaw=0,kind='parallel')]), 1.1)
        self.assertEqual(c.slot_stamp, 1.1)
        self.assertEqual(c.slot['kind'], 'parallel')

    def test_bypass_preserves_pending_instruction(self):
        c = self.core()
        c.pending, c.pending_at = 'LEFT', 1.0
        c.start_follow(intersection_path(c.pose,'STRAIGHT',c.cfg), 'BYPASS', 1.1)
        self.assertEqual(c.pending, 'LEFT')
        self.sign(c, 'PARKING', 1.2)
        c.resume_lane()
        self.assertEqual(c.pending, 'LEFT')

    def test_bypass_without_pending_ignores_parking(self):
        c = self.core()
        c.start_follow(intersection_path(c.pose,'STRAIGHT',c.cfg), 'BYPASS', 1.0)
        self.sign(c, 'PARKING')
        c.resume_lane()
        self.assertIsNone(c.pending)

    def test_selected_bay_confirmation_must_match_kind_and_heading(self):
        for kind, yaw in (('perpendicular', 0), ('parallel', math.pi/2)):
            c = self.core()
            slot = dict(pose=(.6,-.45,0), length=.70,width=.36,kind='parallel')
            c.future = _SimpleFuture()
            c.future._set_result(([(0,0,0,1,0),(.47,-.45,0,-1,0)],'planned',slot,[]))
            c.state, c.parking_trigger_at, c.parking_candidate_stamp = 'AUTO_PLANNING',1.0,1.0
            c.parking_candidates = [dict(slot,kind=kind,pose=(.6,-.45,yaw))]
            c.tick(1.0)
            self.assertEqual(c.state, 'PARKING_SCAN')
            self.assertFalse(c.slot_locked)

    def test_auto_parallel_real_planner_and_follower_reach_terminal(self):
        c = self.core(parking_observe_s=.5)
        self.sign(c, 'PARKING', .1)
        self.blue(c)
        c.tick(1.0)
        slots = [dict(x=.6,y=-.45,yaw=0,kind='parallel')]
        for now in (1.2, 1.8):
            c.observe_ground(dict(source='front',part='slots',markers=[],slots=slots),now)
        c.scan.stamp = 1.8
        c.tick(1.8)
        self.assertEqual(c.state, 'AUTO_PLANNING', c.reason)
        c.future.result(timeout=15)
        c.tick(1.8)
        self.assertEqual(c.state, 'PARKING', c.reason)
        for i in range(1000):
            now = 1.85+i*.05
            c.scan.stamp = now
            x,y = local(c.pose,c.slot['pose'])
            c.observe_ground(dict(source='rear' if i%2 else 'front',part='slots',markers=[],
                slots=[dict(x=x,y=y,yaw=wrap(c.slot['pose'][2]-c.pose[2]),kind='parallel')]),now)
            c.set_pose(c.pose,now)
            speed,steer = c.tick(now)
            if c.state == 'FINISHED':
                break
            gain = c.cfg['raw_to_mps']['forward' if speed >= 0 else 'reverse']
            c.pose = bicycle(c.pose,speed*gain*.05,steer,c.cfg['wheelbase'])
        self.assertEqual(c.state, 'FINISHED', c.reason)
        self.assertTrue(inside_slot(c.pose,c.slot,c.cfg))


if __name__ == '__main__':
    unittest.main()
