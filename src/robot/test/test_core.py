#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Bounded offline tests for the ROS-independent competition core.

The package is installed below ``src/`` by catkin.  Keeping this test
standalone makes it runnable from the source tree on both ROS Melodic's
Python 2.7 and a local Python 3 interpreter without importing ROS.
"""

from __future__ import division, print_function

import copy
import math
import os
import sys
import time
import unittest

import yaml


PACKAGE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
SOURCE_DIR = os.path.dirname(PACKAGE_DIR)
if SOURCE_DIR not in sys.path:
    sys.path.insert(0, SOURCE_DIR)

from robot.lidar.scan import Scan as lidar_Scan
from robot.common import geometry as geometry
from robot.common import planning as planning# noqa: E402
from robot.master.controller import Controller# noqa: E402


with open(os.path.join(PACKAGE_DIR, "config", "competition.yaml"), "r") as stream:
    CONFIG = yaml.safe_load(stream)


def _scan(stamp):
    """Build a valid 360-ray snapshot with one occupied and one unknown ray."""
    ray_count = 360
    ranges = [3.0] * ray_count
    ranges[180] = 1.0
    ranges[270] = float("nan")
    return lidar_Scan(
        ranges,
        -math.pi,
        2.0 * math.pi / ray_count,
        0.05,
        3.0,
        (0.0, 0.0, 0.0),
        {"x": 0.0, "y": 0.0, "yaw": 0.0, "range_cap": 3.0},
        stamp,
    )


class CoreTest(unittest.TestCase):
    def _controller(self, **overrides):
        cfg = copy.deepcopy(CONFIG)
        cfg.update(overrides)
        controller = Controller(cfg)
        self.addCleanup(controller.close)
        return controller

    @staticmethod
    def _votes(controller, label, start):
        for offset in (0.0, 0.1, 0.2):
            stamp = start + offset
            controller.observe_sign(label, 0.95, stamp, stamp)

    @staticmethod
    def _simulate_follower(follower, path, cfg, steps=120):
        pose = tuple(path[0][:3])
        first_command = None
        for index in range(steps):
            command = follower.command(pose, index * 0.05)
            if first_command is None:
                first_command = command
            if follower.done:
                return pose, first_command
            step = 0.025 if command[0] >= 0 else -0.025
            pose = geometry.bicycle(pose, step, command[1], cfg["wheelbase"])
        return pose, first_command

    def test_scan_classifies_free_occupied_and_unknown_rays(self):
        scan = _scan(10.0)

        self.assertEqual(scan.classify((0.5, 0.0)), "FREE")
        self.assertEqual(scan.classify((1.0, 0.0)), "OCCUPIED")
        self.assertEqual(scan.classify((0.0, 0.5)), "UNKNOWN")

    def test_tick_stops_when_scan_is_stale(self):
        controller = self._controller(wait_green=False)
        controller.scan = _scan(0.0)

        command = controller.tick(1.0)

        self.assertEqual(command, (0, 0.0))
        self.assertEqual(controller.reason, "scan_missing_or_stale")

    def test_green_start_requires_three_frames(self):
        c=self._controller(wait_green=True)
        for stamp in (1,1.1):
            c.observe_sign('GREEN',.8,stamp,stamp)
            self.assertEqual(c.state,'WAIT_GREEN')
        c.observe_sign('GREEN',.8,1.2,1.2)
        self.assertEqual(c.state,'STARTUP_STRAIGHT')
        self.assertEqual(c.sign_info['decision'],'green_release')
        self.assertEqual(c.sign_info['votes'],3)

    def test_green_below_threshold_never_accumulates(self):
        c=self._controller(wait_green=True)
        for i,label in enumerate(('GREEN','','','GREEN')):
            c.observe_sign(label,.599 if label else 0,1+i*.1,1+i*.1)
        self.assertEqual(c.state,'WAIT_GREEN')

    def test_green_start_ignores_duplicate_stale_and_low_confidence(self):
        c=self._controller(wait_green=True)
        c.observe_sign('GREEN',.1,1,1)
        c.observe_sign('GREEN',.95,1,1.1)
        c.observe_sign('GREEN',.95,.9,1.1)
        c.observe_sign('GREEN',.95,1.2,10)
        self.assertEqual(c.state,'WAIT_GREEN')
        for stamp in (10,10.1,10.2):
            c.observe_sign('GREEN',.8,stamp,stamp)
        self.assertEqual(c.state,'STARTUP_STRAIGHT')

    def test_red_parking_and_green_all_use_single_frame(self):
        for label in ('RED','PARKING'):
            c=self._controller(wait_green=False)
            c.observe_sign(label,.8 if label=='RED' else .6,1,1)
            self.assertTrue(c.red if label=='RED' else c.pending==label)
        c=self._controller(wait_green=False)
        c.red=True;c.observe_sign('GREEN',.6,1,1)
        self.assertFalse(c.red)

    def test_direction_accepts_one_frame_at_point_six(self):
        for label in ('LEFT', 'RIGHT', 'STRAIGHT'):
            for middle in ('', label, 'GREEN'):
                c = self._controller(wait_green=False, sign_ttl=0)
                c.observe_sign(label, .59, 1.0, 1.0)
                self.assertIsNone(c.pending)
                c.observe_sign(middle, .59 if middle else 0.0, 1.1, 1.1)
                c.observe_sign(label, .6, 1.2, 1.2)
                self.assertEqual(c.pending, label)
                self.assertIsNone(c.action)  # sign acceptance still waits for blue

    def test_direction_window_does_not_mix_labels_or_accumulate_forever(self):
        for observations in ((('LEFT', .59), ('RIGHT', .59), ('STRAIGHT', .59)),
                             (('LEFT', .59), ('', 0), ('', 0), ('LEFT', .59)),
                             (('LEFT', .599), ('LEFT', .599), ('LEFT', .599))):
            c = self._controller(wait_green=False)
            for i, (label, score) in enumerate(observations):
                c.observe_sign(label, score, 1+i*.1, 1+i*.1)
            self.assertIsNone(c.pending)

    def test_direction_window_ignores_duplicate_and_expires_after_source_gap(self):
        c = self._controller(wait_green=False)
        c.observe_sign('LEFT', .59, 1.0, 1.0)
        c.observe_sign('LEFT', .9, 1.0, 1.1)
        c.observe_sign('LEFT', .9, .9, 1.1)
        c.observe_sign('LEFT', .9, 1.2, 10.0)
        self.assertIsNone(c.pending)
        c.observe_sign('LEFT', .59, 10.0, 10.0)
        self.assertIsNone(c.pending)
        c.observe_sign('LEFT', .8, 10.1, 10.1)
        self.assertEqual(c.pending, 'LEFT')

    def test_single_frame_direction_reuses_blue_wait_and_locks_action(self):
        c = self._controller(wait_green=False, intersection_wait_s=1.0)
        for t, label in ((1.0, 'LEFT'), (1.1, ''), (1.2, 'LEFT')):
            c.observe_sign(label, .8 if label else 0.0, t, t)
        self.assertEqual(c.pending, 'LEFT')
        self.assertIsNone(c.dispatch(1.3))
        c.observe_ground(dict(source='front', markers=[dict(kind='junction',
            x=c.cfg['marker_trigger_x']+.10,y=.26)],slots=[]),1.35)
        self.assertIsNone(c.dispatch(1.35))
        c.observe_ground(dict(source='front', markers=[dict(kind='junction',
            x=c.cfg['marker_trigger_x']-.01,y=.26)],slots=[]),1.4)
        self.assertEqual(c.dispatch(1.4), (0, 0.0))
        self.assertEqual(c.reason, 'blue_stop_wait_left')
        for t, label in ((1.5, 'RIGHT'), (1.6, ''), (1.7, 'RIGHT')):
            c.observe_sign(label, .8 if label else 0.0, t, t)
        self.assertIsNone(c.next_direction)
        self.assertEqual(c.action, 'LEFT')

    def test_three_red_observations_latch_stop(self):
        controller = self._controller(wait_green=False)
        self._votes(controller, "RED", 1.0)

        self.assertTrue(controller.red)
        self.assertEqual(controller.tick(1.2), (0, 0.0))
        self.assertEqual(controller.reason, "red_latched")

    def test_three_green_observations_clear_red_latch(self):
        controller = self._controller(wait_green=False)
        self._votes(controller, "RED", 1.0)
        self._votes(controller, "GREEN", 2.0)

        self.assertFalse(controller.red)
        self.assertEqual(controller.state, "LANE")

    def test_duplicate_sign_timestamp_does_not_add_vote(self):
        c=self._controller(wait_green=False)
        c.observe_sign('RED',.8,1,1)
        c.observe_sign('GREEN',.99,1,1.1)
        self.assertTrue(c.red)
        self.assertEqual(c.sign_stamp,1)
        self.assertEqual(c.sign_count,1)
        c.observe_sign('GREEN',.6,2,2)
        self.assertFalse(c.red)

    def test_short_ground_marker_is_ignored_for_turn_dispatch(self):
        controller = self._controller(wait_green=False)
        controller.pending = "LEFT"
        controller.observe_ground(
            {"markers": [{"kind": "tick", "x": 0.20, "y": 0.0, "length": 0.10}]},
            1.0,
        )

        self.assertIsNone(controller.marker)
        self.assertIsNone(controller.dispatch(1.0))
        self.assertEqual(controller.state, "LANE")

    def test_long_junction_marker_dispatches_pending_turn(self):
        controller = self._controller(wait_green=False)
        controller.pending = "LEFT"
        controller.observe_ground(
            {"markers": [{"kind": "junction", "x": 0.20, "y": 0.0, "length": 0.50}]},
            1.0,
        )

        command = controller.dispatch(1.0)

        self.assertEqual(command, (0, 0.0))
        self.assertEqual(controller.state, "MANEUVER")
        self.assertEqual(controller.action, "LEFT")
        self.assertIsNone(controller.pending)

    def test_missing_lane_uses_bounded_gap_command(self):
        controller = self._controller(wait_green=False)
        controller.gap_origin = controller.pose
        controller.gap_start = 0.0
        controller.gap_steer = 0.18
        controller.observe_lane([], 0.0, 1.0)

        command = controller.lane_command(1.0)

        self.assertEqual(controller.state, "GAP")
        self.assertEqual(command, (CONFIG["speed_raw"]["gap"], 0.18))

    def test_inside_slot_requires_clearance_for_all_four_tires(self):
        cfg = copy.deepcopy(CONFIG)
        slot = {"pose": (0.0, 0.0, 0.0), "length": 0.45, "width": 0.38}
        contained = (-cfg["wheelbase"] / 2.0, 0.0, 0.0)
        shifted = (0.08, 0.0, 0.0)

        self.assertEqual(len(geometry.tires(contained, cfg)), 4)
        self.assertTrue(geometry.inside_slot(contained, slot, cfg))
        self.assertFalse(geometry.inside_slot(shifted, slot, cfg))

    def test_follower_simulation_tracks_forward_left_turn(self):
        cfg = copy.deepcopy(CONFIG)
        path = planning.intersection_path((0.0, 0.0, 0.0), "LEFT", cfg)
        follower = planning.Follower(path, cfg)

        pose, first_command = self._simulate_follower(follower, path, cfg)

        self.assertGreater(first_command[0], 0)
        self.assertLessEqual(abs(first_command[1]), cfg["max_steer"])
        self.assertTrue(follower.done)
        self.assertLessEqual(geometry.distance(pose, path[-1]), cfg["path_goal_tolerance"])
        self.assertLessEqual(
            abs(geometry.wrap(pose[2] - path[-1][2])),
            cfg["path_yaw_tolerance"],
        )

    def test_follower_simulation_tracks_reverse_turn(self):
        cfg = copy.deepcopy(CONFIG)
        path = []
        planned_pose = (0.0, 0.0, 0.0)
        planned_steer = 0.30
        for unused in range(45):
            path.append(tuple(planned_pose) + (-1, planned_steer))
            planned_pose = geometry.bicycle(
                planned_pose, -0.025, planned_steer, cfg["wheelbase"]
            )
        follower = planning.Follower(path, cfg, parking=True)

        pose, first_command = self._simulate_follower(follower, path, cfg)

        self.assertLess(first_command[0], 0)
        self.assertGreater(first_command[1], 0)
        self.assertLessEqual(abs(first_command[1]), cfg["max_steer"])
        self.assertTrue(follower.done)
        self.assertLess(pose[0], 0.0)
        self.assertGreater(pose[1], 0.0)
        self.assertLessEqual(geometry.distance(pose, path[-1]), cfg["path_goal_tolerance"])
        self.assertLessEqual(
            abs(geometry.wrap(pose[2] - path[-1][2])),
            cfg["path_yaw_tolerance"],
        )

    def test_hybrid_plan_finds_bounded_reverse_parking_path(self):
        cfg = copy.deepcopy(CONFIG)
        # Use the target Nano's configured CPU budget, not the desktop budget.
        start = (0.0, 0.0, 0.0)
        goal = (-cfg["wheelbase"] / 2.0, 0.0, 0.0)

        def allowed(point):
            return -1.0 <= point[0] <= 1.0 and -0.8 <= point[1] <= 0.8

        began = time.time()
        path, reason = planning.hybrid_plan(
            start, goal, cfg, obstacles=(), allowed=allowed, final_direction=-1
        )
        elapsed = time.time() - began

        self.assertLessEqual(elapsed, cfg["planner_timeout"] + 0.25)
        self.assertEqual(reason, "planned")
        self.assertTrue(path)
        self.assertEqual(path[-1][3], -1)
        self.assertLessEqual(
            geometry.distance(path[-1], goal), cfg["path_goal_tolerance"]
        )

    def test_hybrid_plan_finds_bounded_forward_uturn_path(self):
        cfg = copy.deepcopy(CONFIG)
        # Use the target Nano's configured CPU budget, not the desktop budget.
        start = (0.0, 0.0, 0.0)
        goal = (0.0, cfg["lane_width"], math.pi)

        def allowed(point):
            return (
                -cfg["lane_width"] <= point[0] <= 1.5 * cfg["lane_width"]
                and -cfg["lane_width"] / 2.0 <= point[1] <= 1.5 * cfg["lane_width"]
            )

        began = time.time()
        path, reason = planning.hybrid_plan(
            start, goal, cfg, obstacles=(), allowed=allowed, final_direction=1
        )
        elapsed = time.time() - began

        self.assertLessEqual(elapsed, cfg["planner_timeout"] + 0.25)
        self.assertEqual(reason, "planned")
        self.assertTrue(path)
        self.assertEqual(path[-1][3], 1)
        self.assertLessEqual(
            geometry.distance(path[-1], goal), cfg["path_goal_tolerance"]
        )
        self.assertLessEqual(
            abs(geometry.wrap(path[-1][2] - goal[2])),
            cfg["path_yaw_tolerance"],
        )


if __name__ == "__main__":
    unittest.main()
