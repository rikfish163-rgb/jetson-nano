"""One-shot direction commands must release control to the existing lane policy."""
from __future__ import division
import copy
import math
import os
import sys
import unittest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.join(ROOT,'src'))
from robot.master.controller import Controller
from robot.lidar.scan import Scan
from robot.common.geometry import world
from robot.common.planning import intersection_path


class DirectionHandoffTests(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(ROOT,'config','competition.yaml')) as f:
            cfg = yaml.safe_load(f)
        # Fixed historical poses below belong to the original 0.45 m entry.
        cfg.update(wait_green=False,sign_ttl=0,turn_entry=.45,left_turn_entry=.45,turn_exit=.30,
                   turn_radius=.65,turn_angle_deg=90,exit_yaw_tolerance=.30,
                   exit_lateral_tolerance=.18,lookahead=.55,action_lookahead=.30)
        self.c = Controller(copy.deepcopy(cfg))
        self.addCleanup(self.c.close)
        # Captured live path start and stopped command-model pose, 2026-09-07.
        start = (.9085457827581499,-.14710819411567982,-.2600560726247649)
        self.c.start_follow(intersection_path(start,'LEFT',cfg),'LEFT',0.0)
        self.stopped_pose = (2.2745323966456517,.9862862616963881,1.453825773286587)
        self.curve = [(.55,.1059956518),(.65,.1476823004),(.75,.1622457359),(1.15,-.0521489729)]

    def observe(self,now,points=None,pose=None):
        self.c.set_pose(pose or self.stopped_pose,now)
        self.c.scan = Scan([float('inf')]*360,-math.pi,2*math.pi/360,.05,6,
                           self.c.pose,self.c.cfg['lidar'],now)
        self.c.observe_lane(points or self.curve,.419,now)
        return self.c.tick(now)

    def test_live_curved_exit_releases_left_without_hitting_precise_goal(self):
        for now in (10.0,10.1,10.2):
            command = self.observe(now)
        self.assertEqual(self.c.state,'LANE',self.c.reason)
        self.assertIsNone(self.c.action)
        self.assertIsNone(self.c.pending)
        self.assertIsNone(self.c.follower)
        self.assertEqual(self.c.action_started,0.0)
        self.assertGreater(command[0],0)
        self.assertGreater(command[1],0)  # follow the visible bend, not the old endpoint
        self.observe(60.0)
        self.assertEqual(self.c.state,'LANE')

    def test_reacquire_accepts_bend_even_past_old_search_limit(self):
        self.c.state = 'REACQUIRE'
        for now in (10.0,10.1,10.2):
            self.observe(now)
        self.assertEqual(self.c.state,'LANE',self.c.reason)
        self.assertIsNone(self.c.action)

    def test_visible_original_lane_does_not_cancel_an_unfinished_turn(self):
        start = self.c.follower.path[0][:3]
        for now in (1.0,1.1,1.2):
            self.observe(now,[(.55,0),(.65,0),(.85,0)],start)
        self.assertEqual(self.c.state,'MANEUVER')
        self.assertEqual(self.c.action,'LEFT')

    def test_crossing_lane_is_not_a_followable_exit(self):
        for now in (10.0,10.1,10.2):
            self.observe(now,[(.55,0),(.55,.3)])
        self.assertEqual(self.c.action,'LEFT')
        self.assertNotEqual(self.c.state,'LANE')

    def test_next_direction_is_stored_then_waits_for_next_marker(self):
        for now in (10.0,10.1,10.2):
            self.observe(now)
        for now in (10.3,10.4,10.5):
            self.c.observe_sign('RIGHT',.99,now,now)
        # The second sign belongs to the next junction but the current LEFT
        # action still owns the blue line until it completes.
        self.assertIsNone(self.c.pending)
        self.assertEqual(self.c.next_direction,'RIGHT')
        self.observe(10.6)
        self.assertEqual(self.c.state,'LANE')
        self.assertIsNone(self.c.action)
        self.assertEqual(self.c.pending,'RIGHT')
        self.c.observe_ground(dict(source='front',markers=[dict(kind='junction',x=.3,y=0)],slots=[]),10.7)
        self.observe(10.7)
        self.assertIn(self.c.state,('INTERSECTION_WAIT','MANEUVER'))
        self.assertEqual(self.c.action,'RIGHT')

    def test_same_completed_sign_rearms_for_next_marker(self):
        for now in (10.0,10.1,10.2):
            self.observe(now)
        for now in (10.3,10.4,10.5):
            self.c.observe_sign('LEFT',.99,now,now)
        self.assertIsNone(self.c.pending)
        self.assertEqual(self.c.next_direction,'LEFT')
        self.observe(10.6)
        self.assertEqual(self.c.pending,'LEFT')
        self.assertIsNone(self.c.action)
        self.assertEqual(self.c.last_completed_action,'LEFT')

    def test_real_exit_handoff_discards_right_seen_during_left(self):
        # These frames are from the board that launched LEFT.  They arrive
        # before the bounded action grace period and must not seed the queue.
        for now in (.6,.7,.8):
            self.c.observe_sign('RIGHT',.99,now,now)
        self.assertIsNone(self.c.next_direction)
        for now in (10.0,10.1,10.2):
            self.observe(now)
        self.assertEqual(self.c.state,'LANE')
        self.assertEqual(self.c.last_completed_action,'LEFT')
        self.assertIsNone(self.c.action)
        self.assertIsNone(self.c.pending)
        for now in (10.21,10.22):
            self.c.observe_sign('RIGHT',.99,now,now)
        self.assertEqual(self.c.pending,'RIGHT')
        self.assertIsNone(self.c.next_direction)
        self.c.observe_ground(dict(source='front',markers=[dict(kind='junction',x=.3,y=0)],slots=[]),10.3)
        self.observe(10.3)
        self.assertEqual(self.c.action,'RIGHT')

    def test_right_exit_can_handoff_before_the_straight_tail_finishes(self):
        # This case exercises the ordinary path follower, not full-lock RIGHT.
        self.c.cfg['right_turn_full_lock']=False
        path = intersection_path((0,0,0),'RIGHT',self.c.cfg)
        self.c.start_follow(path,'RIGHT',0.0)
        end = path[-1][:3]
        xy = world(end,(-.15,0))
        pose = tuple(xy)+(end[2],)
        for now in (10.0,10.1,10.2):
            self.observe(now,[(.55,-.10),(.65,-.14),(.85,-.17)],pose)
        self.assertEqual(self.c.state,'LANE',self.c.reason)
        self.assertIsNone(self.c.follower)
        self.assertEqual(self.c.last_completed_action,'RIGHT')


if __name__ == '__main__':
    unittest.main()
