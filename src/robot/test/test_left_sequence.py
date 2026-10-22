from __future__ import division
import copy
import math
import os
import sys
import unittest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))
from robot.master.controller import Controller
from robot.common.contracts import command_to_model_steering
from robot.lidar.scan import Scan
from robot.common.geometry import bicycle
from robot.common.geometry import wrap
from robot.common.geometry import collision
from robot.common.geometry import local
from robot.common.planning import Follower
from robot.common.planning import intersection_path


class LeftSequenceTests(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(ROOT, 'config', 'competition.yaml')) as f:
            self.cfg = yaml.safe_load(f)
        self.cfg.update(wait_green=False, sign_ttl=0.0, intersection_wait_s=1.0,
                        latch_direction_sign=True,turn_entry=.45,turn_radius=.65,
                        turn_angle_deg=90.0,turn_exit=.30,lookahead=.55,action_lookahead=.30,
                        exit_yaw_tolerance=.30,exit_lateral_tolerance=.18)
        self.cfg['speed_raw']['action'] = 18
        self.c = Controller(copy.deepcopy(self.cfg))
        self.addCleanup(self.c.close)

    def fresh(self, now):
        self.c.scan = Scan([float('inf')]*360, -math.pi, 2*math.pi/360,
                           .05, 6, self.c.pose, self.cfg['lidar'], now)

    def vote(self, label, start):
        for i in range(3):
            self.c.observe_sign(label, .99, start+i*.1, start+i*.1)

    def test_default_left_steers_on_first_motion_after_blue_wait(self):
        # Load the actual shipped profile, not the historical entry override.
        with open(os.path.join(ROOT, 'config', 'competition.yaml')) as f:
            cfg = yaml.safe_load(f)
        cfg.update(wait_green=False, intersection_wait_s=1.0, action_lookahead=.30)
        cfg['speed_raw']['action'] = 18
        c = Controller(cfg)
        self.addCleanup(c.close)
        for i in range(3):
            c.observe_sign('LEFT', .99, 1+i*.1, 1+i*.1)
        c.observe_ground(dict(source='front', markers=[dict(kind='junction', x=cfg['marker_trigger_x']-.01, y=0)], slots=[]), 2.0)
        for now in (2.0, 2.9, 3.0):
            c.scan = Scan([float('inf')]*360, -math.pi, 2*math.pi/360,
                          .05, 6, c.pose, cfg['lidar'], now)
            command = c.tick(now)
            if now < 3.0:
                self.assertEqual(command, (0, 0.0))
                self.assertEqual(c.state, 'INTERSECTION_WAIT')
        self.assertEqual(c.state, 'MANEUVER')
        self.assertEqual(c.action, 'LEFT')
        self.assertIsNone(c.pending)
        self.assertEqual(command[0], 18)
        self.assertGreater(command[1], 0.0)
        self.assertLessEqual(command[1], cfg['max_steer'])

    def test_explicit_entry_still_preserves_straight_approach(self):
        # Reproduce the old late-onset command; keep deliberate entry tuning.
        cfg = copy.deepcopy(self.cfg)
        cfg.update(left_turn_entry=.45, right_turn_entry=.20)
        for action in ('LEFT', 'RIGHT'):
            cfg['action_lookahead'] = .10
            follower = Follower(intersection_path((0, 0, 0), action, cfg), cfg)
            raw, steer = follower.command((0, 0, 0), 0)
            self.assertEqual(raw, 18)
            self.assertAlmostEqual(steer, 0.0)

    def test_front_blue_survives_newer_rear_frame_and_duplicate_is_ignored(self):
        self.c.observe_ground(dict(source='rear', markers=[], slots=[]), 2.2)
        front = dict(source='front', markers=[dict(kind='junction', x=.6, y=0)], slots=[])
        self.c.observe_ground(front, 2.1)
        self.assertIsNotNone(self.c.marker)
        self.c.observe_ground(dict(source='front', markers=[dict(kind='junction', x=1.2, y=0)], slots=[]), 2.0)
        self.assertAlmostEqual(self.c.marker[0][0], .6)

    def test_split_blue_and_slots_at_same_stamp_both_arrive(self):
        self.c.cfg['parking_slot'] = 'P5'
        self.c.observe_ground(dict(source='front', part='markers', markers=[dict(kind='junction', x=.6, y=0)], slots=[]), 2.0)
        self.c.observe_ground(dict(source='front', part='slots', markers=[], slots=[dict(kind='perpendicular', x=.8, y=-.5, yaw=math.pi/2)]), 2.0)
        self.assertIsNotNone(self.c.marker)
        self.assertIsNotNone(self.c.slot)

    def test_collision_matches_existing_body_geometry_for_rotated_poses(self):
        cfg = self.cfg
        margin = cfg['obstacle_margin']
        points = [(x*.07,y*.07) for x in range(-8,9) for y in range(-8,9)]
        for pose in ((0,0,0),(.2,-.1,.7),(-.4,.2,-2.5)):
            for point in points:
                x,y = local(pose,point)
                expected = (-cfg['rear_overhang']-margin <= x <= cfg['wheelbase']+cfg['front_overhang']+margin and
                            abs(y) <= cfg['body_width']/2+margin)
                self.assertEqual(collision(pose,[point],cfg),expected)

    def test_left_is_stored_until_blue_then_stop_wait_turn_and_follow_curve(self):
        self.vote('LEFT', 1.0)
        self.vote('RIGHT', 2.0)
        self.assertEqual(self.c.pending, 'LEFT')
        self.fresh(8.0)
        self.c.tick(8.0)
        self.assertEqual(self.c.pending, 'LEFT')
        self.c.observe_ground(dict(source='front', markers=[dict(kind='junction', x=.3, y=0)], slots=[]), 8.0)
        self.assertEqual(self.c.tick(8.0), (0, 0.0))
        self.assertEqual(self.c.state, 'INTERSECTION_WAIT')
        self.fresh(8.9)
        self.assertEqual(self.c.tick(8.9), (0, 0.0))
        self.assertIsNone(self.c.follower)
        # Fresh empty images through the junction; lane geometry returns at exit.
        moved = False
        for i in range(600):
            now = 9.0+i*.05
            self.fresh(now)
            self.c.observe_ground(dict(source='front', markers=[], slots=[]), now)
            if self.c.state == 'REACQUIRE':
                self.c.observe_lane([(.2, 0), (.4, .01), (.8, .28)], .99, now)
            raw, steer = self.c.tick(now)
            moved = moved or (self.c.state == 'MANEUVER' and raw > 0)
            self.c.set_pose(bicycle(self.c.pose, raw*self.cfg['raw_to_mps']['forward']*.05,
                                    command_to_model_steering(steer, self.cfg), self.cfg['wheelbase']), now)
            if self.c.state == 'LANE' and self.c.completed_at_pose is not None:
                break
        self.assertTrue(moved)
        self.assertEqual(self.c.state, 'LANE', self.c.reason)
        self.assertIsNone(self.c.pending)
        self.assertLess(abs(wrap(self.c.pose[2]-math.pi/2)), self.cfg['exit_yaw_tolerance'])


if __name__ == '__main__':
    unittest.main()
