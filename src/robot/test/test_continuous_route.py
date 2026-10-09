"""GREEN -> fixed-raw lane following -> repeated sign/blue actions in one run."""
from __future__ import division
import math
import os
import sys
import unittest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.join(ROOT,'src'))
from robot.master.controller import Controller
from robot.lidar.scan import Scan
from robot.common.geometry import bicycle
from robot.common.geometry import wrap
from robot.common.geometry import world
from robot.common.geometry import local


class ContinuousRouteTests(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(ROOT,'config','competition.yaml')) as f:
            cfg = yaml.safe_load(f)
        cfg.update(wait_green=True,sign_ttl=0,intersection_wait_s=1.0,
                   turn_entry=.45,turn_radius=.65,turn_angle_deg=90.0,turn_exit=.30,
                   straight_distance=1.50,lookahead=.55,action_lookahead=.30,
                   exit_yaw_tolerance=.30,exit_lane_heading_tolerance=1.0472,
                   parking_enabled=False,right_turn_full_lock=False)
        cfg['speed_raw']['lane'],cfg['speed_raw']['action'] = 20,18
        self.c = Controller(cfg)
        self.addCleanup(self.c.close)
        self.now = 1.0
        self.test_exit_line = None

    def step(self,lane=True):
        self.now += .05
        c = self.c
        c.scan = Scan([float('inf')]*360,-math.pi,2*math.pi/360,.05,6,c.pose,c.cfg['lidar'],self.now)
        if lane:
            c.observe_lane([(.55,0),(.65,.01),(.85,.03)],.95,self.now)
        elif c.straight_search is not None:
            c.observe_lane([],0.0,self.now)
        markers,blue_lines=[],[]
        if c.straight_search is None:
            self.test_exit_line=None
        elif c.straight_search.get('travelled_m',0) > .72:
            if self.test_exit_line is None:
                self.test_exit_line=(world(c.pose,(.3,0)),c.pose[2])
            point,yaw=self.test_exit_line
            x,y=local(c.pose,point)
            if x>0:
                markers=[dict(kind='junction',x=x,y=y)]
                blue_lines=[dict(x=x,y=y,yaw=wrap(yaw-c.pose[2]),length=.8)]
        c.observe_ground(dict(source='front',part='markers',markers=markers,blue_lines=blue_lines,slots=[]),self.now)
        command = c.tick(self.now)
        c.set_pose(bicycle(c.pose,command[0]*c.cfg['raw_to_mps']['forward']*.05,
                           command[1],c.cfg['wheelbase']),self.now)
        return command

    def vote(self,label):
        votes = (3 if label == 'GREEN' else
                 self.c.cfg['parking_sign_votes'] if label == 'PARKING' else
                 self.c.cfg['sign_votes'])
        for unused in range(votes):
            self.now += .34
            self.c.observe_sign(label,.99,self.now,self.now)

    def test_green_left_straight_right_then_lane_continues(self):
        for unused in range(4):
            self.assertEqual(self.step(),(0,0.0))
        self.assertEqual(self.c.state,'WAIT_GREEN')
        self.vote('GREEN')
        self.assertEqual(self.step()[0],20)
        self.assertEqual(self.c.state,'STARTUP_STRAIGHT')
        self.now += .001
        self.c.observe_ground(dict(source='front',part='markers',
            markers=[dict(kind='junction',x=.3,y=0)],slots=[]),self.now)
        self.step()
        self.assertEqual(self.c.state,'LANE')
        completed = []
        for action in ('LEFT','STRAIGHT','RIGHT'):
            self.vote(action)
            self.assertEqual(self.c.pending,action)
            # Every sign waits for a distinct junction while lane control continues.
            for unused in range(120):
                self.assertEqual(self.step()[0],20)
                self.assertEqual(self.c.state,'LANE')
            start_yaw = self.c.pose[2]
            self.now += .001  # A new camera frame, not the last empty frame.
            self.c.observe_ground(dict(source='front',part='markers',
                markers=[dict(kind='junction',x=.3,y=0)],slots=[]),self.now)
            self.assertEqual(self.step(),(0,0.0))
            self.assertEqual(self.c.state,'INTERSECTION_WAIT')
            for unused in range(10):
                self.assertEqual(self.step(),(0,0.0))
            saw_motion_without_lane = False
            for unused in range(650):
                # Real perception continues running; only the intersection has no paint.
                at_exit = (self.c.state == 'REACQUIRE' or
                           (self.c.state == 'MANEUVER' and self.c.direction_exit_reached()))
                if self.c.straight_search is not None:
                    at_exit = self.c.straight_search.get('travelled_m',0) > .20
                command = self.step(lane=at_exit)
                saw_motion_without_lane |= command[0] > 0 and not at_exit
                if self.c.state == 'LANE' and self.c.action is None:
                    break
            self.assertEqual(self.c.state,'LANE',self.c.reason)
            self.assertIsNone(self.c.pending)
            self.assertIsNone(self.c.follower)
            self.assertEqual(self.c.action_started,0)
            self.assertEqual(self.c.last_completed_action,action)
            self.assertTrue(saw_motion_without_lane)
            expected = {'LEFT':math.pi/2,'STRAIGHT':0,'RIGHT':-math.pi/2}[action]
            self.assertLess(abs(wrap(self.c.pose[2]-start_yaw-expected)),.30)
            completed.append(action)
            for unused in range(10):
                self.assertEqual(self.step()[0],20)
        self.assertEqual(completed,['LEFT','STRAIGHT','RIGHT'])
        # A previous action's 40-second timer must not stop subsequent lane following.
        self.now += 45
        self.assertEqual(self.step()[0],20)
        self.assertEqual(self.c.state,'LANE')

    def test_red_interrupts_lane_and_green_resumes_fixed_speed(self):
        self.vote('GREEN')
        self.assertEqual(self.step()[0],20)
        self.vote('RED')
        self.assertEqual(self.step(),(0,0.0))
        self.vote('GREEN')
        self.assertEqual(self.step()[0],20)

    def test_continuous_route_with_different_left_and_right_profiles(self):
        self.c.cfg.update(left_turn_entry=.45,left_turn_radius=.60,left_turn_angle_deg=90,left_turn_exit=.30,
                          right_turn_entry=.20,right_turn_radius=.55,right_turn_angle_deg=90,right_turn_exit=.25)
        self.test_green_left_straight_right_then_lane_continues()

    def test_continuous_route_with_immediate_left_and_unchanged_right(self):
        self.c.cfg.update(left_turn_entry=0.0,left_turn_radius=.60,left_turn_angle_deg=90,left_turn_exit=.30,
                          right_turn_entry=.20,right_turn_radius=.55,right_turn_angle_deg=90,right_turn_exit=.25)
        self.test_green_left_straight_right_then_lane_continues()


if __name__ == '__main__':
    unittest.main()
