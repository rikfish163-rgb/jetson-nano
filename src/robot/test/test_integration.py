#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Offline controller integration tests using stamped synthetic inputs.

These tests exercise the state machine through its callback-like interfaces;
they do not import ROS, start nodes, run the planner, or touch hardware.
"""

from __future__ import division, print_function
from robot.lidar.scan import Scan as lidar_Scan

import copy
import math
import os
import sys
import unittest

import yaml


PACKAGE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
SOURCE_DIR = os.path.dirname(PACKAGE_DIR)
if SOURCE_DIR not in sys.path:
    sys.path.insert(0, SOURCE_DIR)

from robot.common import geometry as geometry
from robot.common import planning as planning# noqa: E402
from robot.master.controller import Controller# noqa: E402


with open(os.path.join(PACKAGE_DIR, "config", "competition.yaml"), "r") as stream:
    CONFIG = yaml.safe_load(stream)


class _DoneFollower(object):
    done = True

    def command(self, pose, now):
        return 0, 0.0


class IntegrationTest(unittest.TestCase):
    def _controller(self, **overrides):
        cfg = copy.deepcopy(CONFIG)
        cfg["wait_green"] = False
        cfg.update(overrides)
        controller = Controller(cfg)
        self.addCleanup(controller.close)
        return controller

    @staticmethod
    def _clear_scan(stamp, cfg):
        cap = cfg["lidar"]["range_cap"]
        return lidar_Scan(
            [float("inf")] * 360,
            -math.pi,
            2.0 * math.pi / 360,
            0.05,
            cap,
            (0.0, 0.0, 0.0),
            {"x": 0.0, "y": 0.0, "yaw": 0.0, "range_cap": cap},
            stamp,
        )

    @staticmethod
    def _blocked_scan(stamp, cfg):
        cap = cfg["lidar"]["range_cap"]
        ranges = [float("inf")] * 360
        ranges[180] = 0.20
        return lidar_Scan(
            ranges,
            -math.pi,
            2.0 * math.pi / 360,
            0.05,
            cap,
            (0.0, 0.0, 0.0),
            {"x": 0.0, "y": 0.0, "yaw": 0.0, "range_cap": cap},
            stamp,
        )

    @staticmethod
    def _correct_lane():
        return [(0.30, 0.0), (0.60, 0.0)]

    @staticmethod
    def _wrong_lane():
        return [(0.30, 0.0), (0.30, 0.30)]

    def _prepare_reacquire(self, controller):
        controller.set_pose((0.0, 0.0, 0.0), 0.0)
        controller.state = "REACQUIRE"
        controller.exit_pose = (0.0, 0.0, 0.0)
        controller.exit_count = 0
        controller.exit_stamp = -1.0
        controller.action_started = 0.0

    def _fresh_lane_tick(self, controller, points, stamp):
        controller.scan = self._clear_scan(stamp, controller.cfg)
        controller.observe_lane(points, 1.0, stamp)
        return controller.tick(stamp)

    def test_marker_remains_world_fixed_after_vehicle_leaves_front_fov(self):
        controller = self._controller()
        start_pose = (1.0, 2.0, math.pi / 4.0)
        local_marker = (0.30, 0.10)
        expected_world_marker = geometry.world(start_pose, local_marker)
        controller.set_pose(start_pose, 10.0)
        controller.observe_ground(
            {"markers": [{"kind": "junction", "x": local_marker[0],
                          "y": local_marker[1], "length": 0.50}],
             "slots": []},
            10.1,
        )

        moved_xy = geometry.world(start_pose, (0.80, 0.0))
        moved_pose = (moved_xy[0], moved_xy[1], start_pose[2])
        controller.set_pose(moved_pose, 11.0)
        controller.observe_ground({"markers": [], "slots": []}, 11.1)

        self.assertIsNotNone(controller.marker)
        self.assertIsNotNone(controller.parking_marker)
        for actual, expected in zip(controller.marker[0], expected_world_marker):
            self.assertAlmostEqual(actual, expected, places=7)
        for actual, expected in zip(controller.parking_marker[0], expected_world_marker):
            self.assertAlmostEqual(actual, expected, places=7)
        self.assertLess(geometry.local(controller.pose, controller.marker[0])[0], 0.0)

    def test_wrong_exit_frames_cannot_reacquire_lane(self):
        controller = self._controller()
        self._prepare_reacquire(controller)

        for stamp in (1.0, 2.0, 3.0):
            self._fresh_lane_tick(controller, self._wrong_lane(), stamp)

        self.assertEqual(controller.state, "REACQUIRE")
        self.assertEqual(controller.exit_count, 0)

    def test_correct_exit_requires_three_fresh_consecutive_lane_frames(self):
        controller = self._controller()
        self._prepare_reacquire(controller)

        self._fresh_lane_tick(controller, self._correct_lane(), 1.0)
        self.assertEqual(controller.state, "REACQUIRE")
        self.assertEqual(controller.exit_count, 1)
        self._fresh_lane_tick(controller, self._correct_lane(), 1.1)
        self.assertEqual(controller.state, "REACQUIRE")
        self.assertEqual(controller.exit_count, 2)
        self._fresh_lane_tick(controller, self._correct_lane(), 1.2)

        self.assertEqual(controller.state, "LANE")
        self.assertEqual(controller.exit_count, 3)
        self.assertEqual(controller.completed_at_pose, controller.pose)

    def test_same_sign_label_rearms_next_marker_after_rearm_distance(self):
        controller = self._controller()
        self._prepare_reacquire(controller)
        for stamp in (1.0, 1.1, 1.2):
            self._fresh_lane_tick(controller, self._correct_lane(), stamp)
        self.assertEqual(controller.state, "LANE")
        self.assertIsNotNone(controller.completed_at_pose)

        controller.set_pose((1.20, 0.0, 0.0), 4.0)
        for stamp in (5.0, 5.1, 5.2):
            controller.observe_sign("LEFT", 0.95, stamp, stamp)

        self.assertEqual(controller.pending, "LEFT")
        controller.observe_ground(
            {"markers": [{"kind": "junction", "x": 0.20, "y": 0.0,
                          "length": 0.50}],
             "slots": []},
            5.3,
        )
        controller.dispatch(5.3)
        self.assertEqual(controller.state, "MANEUVER")
        self.assertEqual(controller.action, "LEFT")

    def test_bypass_is_rejected_when_any_sweep_leaves_dashed_zones(self):
        controller = self._controller(
            bypass_enabled=True,
            bypass_zones=[[-1.0, 4.0, -0.20, 0.20]],
        )
        controller.set_pose((0.0, 0.0, 0.0), 0.0)
        controller.scan = self._blocked_scan(1.0, controller.cfg)

        command = controller.checked_command(
            (controller.cfg["speed_raw"]["lane"], 0.0), 1.0, True
        )

        self.assertEqual(command, (0, 0.0))
        self.assertEqual(controller.state, "WAIT_OBSTACLE")
        self.assertEqual(controller.reason, "lidar_obstacle_in_sweep")
        self.assertIsNone(controller.follower)
        self.assertIsNone(controller.action)

    def test_bypass_path_returns_to_zero_lateral_at_end(self):
        cfg = copy.deepcopy(CONFIG)
        start = (0.0, 0.0, 0.0)
        path = planning.bypass_path(start, cfg)
        lateral = [abs(geometry.local(start, point)[1]) for point in path]

        self.assertTrue(path)
        self.assertGreater(max(lateral), cfg["bypass_offset"] * 0.9)
        self.assertAlmostEqual(lateral[-1], 0.0, places=7)

    def test_finished_state_is_latched_against_new_inputs(self):
        controller = self._controller()
        controller.state = "FINISHED"
        controller.pending = "LEFT"
        first = controller.tick(1.0)

        controller.set_pose((2.0, 2.0, 0.0), 2.0)
        controller.observe_lane(self._correct_lane(), 1.0, 2.0)
        second = controller.tick(20.0)

        self.assertEqual(first, (0, 0.0))
        self.assertEqual(second, (0, 0.0))
        self.assertEqual(controller.state, "FINISHED")
        self.assertEqual(controller.reason, "four_wheels_inside_terminal")

    def test_duplicate_lane_snapshot_does_not_advance_exit_count(self):
        controller = self._controller()
        self._prepare_reacquire(controller)
        self._fresh_lane_tick(controller, self._correct_lane(), 1.0)
        self.assertEqual(controller.exit_count, 1)

        controller.scan = self._clear_scan(1.1, controller.cfg)
        controller.observe_lane(self._correct_lane(), 1.0, 1.0)
        controller.tick(1.1)

        self.assertEqual(controller.state, "REACQUIRE")
        self.assertEqual(controller.exit_count, 1)
        self.assertEqual(controller.exit_stamp, 1.0)

    def test_duplicate_pose_snapshot_does_not_advance_parking_finish_count(self):
        controller = self._controller(parking_settle_frames=2)
        controller.state = "PARKING"
        controller.action = "PARKING"
        controller.action_started = 0.0
        controller.follower = _DoneFollower()
        controller.slot = {"pose": (0.0, 0.0, 0.0), "length": 0.45,
                           "width": 0.38}
        controller.slot_stamp = 1.0
        contained_pose = (-controller.cfg["wheelbase"] / 2.0, 0.0, 0.0)
        controller.set_pose(contained_pose, 1.0)
        controller.scan = self._clear_scan(1.0, controller.cfg)
        controller.tick(1.0)
        self.assertEqual(controller.finished_count, 1)
        self.assertEqual(controller.state, "PARKING")

        controller.scan = self._clear_scan(1.1, controller.cfg)
        controller.tick(1.1)
        self.assertEqual(controller.finished_count, 1)
        self.assertEqual(controller.state, "PARKING")

        controller.set_pose(contained_pose, 2.0)
        controller.scan = self._clear_scan(2.0, controller.cfg)
        controller.tick(2.0)
        self.assertEqual(controller.finished_count, 2)
        self.assertEqual(controller.state, "FINISHED")


if __name__ == "__main__":
    unittest.main()
