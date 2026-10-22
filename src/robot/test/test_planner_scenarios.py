#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Bounded parking planner regressions using the real bicycle follower loop."""

from __future__ import division

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

from robot.common import geometry as geometry
from robot.common import planning as planning# noqa: E402


with open(os.path.join(PACKAGE_DIR, "config", "competition.yaml"), "r") as stream:
    CONFIG = yaml.safe_load(stream)


def _parking_problem(name, center, yaw, require_initial_forward=False):
    cfg = copy.deepcopy(CONFIG)
    cfg["planner_require_initial_forward"] = require_initial_forward
    profile = cfg["slots"][name]
    start = (0.0, 0.0, 0.0)
    slot_pose = tuple(center) + (yaw,)
    goalxy = geometry.world(slot_pose, (-cfg["wheelbase"] / 2.0, 0.0))
    goal = tuple(goalxy) + (yaw,)

    rel = geometry.local(start, slot_pose)
    xlo = min(0.0, rel[0]) - cfg["parking_search_x_margin"]
    xhi = max(0.0, rel[0]) + cfg["parking_search_x_margin"]
    side = -1 if cfg["parking_mouth_side"] == "right" else 1
    corners = [
        geometry.local(start, geometry.world(slot_pose, (x, y)))
        for x in (-profile["length"] / 2.0, profile["length"] / 2.0)
        for y in (-profile["width"] / 2.0, profile["width"] / 2.0)
    ]
    mouth = max(point[1] for point in corners) if side < 0 else min(point[1] for point in corners)

    def allowed(point):
        x, y = geometry.local(start, point)
        in_road = xlo <= x <= xhi and (
            mouth <= y <= cfg["parking_road_half_width"]
            if side < 0
            else -cfg["parking_road_half_width"] <= y <= mouth
        )
        sx, sy = geometry.local(slot_pose, point)
        in_slot = abs(sx) <= profile["length"] / 2.0 and abs(sy) <= profile["width"] / 2.0
        return in_road or in_slot

    return cfg, start, goal, allowed


def _simulate_parking(path, start, cfg, max_steps=1600):
    follower = planning.Follower(path, cfg, parking=True)
    pose = tuple(start)
    zero_steps = 0
    dt = 0.05
    for step in range(max_steps):
        command = follower.command(pose, step * dt)
        if follower.done:
            return follower, pose, zero_steps, step + 1
        if command[0] == 0:
            zero_steps += 1
            continue
        scale = cfg["raw_to_mps"]["forward" if command[0] > 0 else "reverse"]
        pose = geometry.bicycle(
            pose,
            command[0] * scale * dt,
            command[1],
            cfg["wheelbase"],
        )
    return follower, pose, zero_steps, max_steps


class PlannerScenarioTest(unittest.TestCase):
    def _assert_parking_scenario(self, name, center, yaw, require_initial_forward=False):
        cfg, start, goal, allowed = _parking_problem(
            name, center, yaw, require_initial_forward
        )
        began = time.time()
        path, reason = planning.hybrid_plan(
            start, goal, cfg, obstacles=(), allowed=allowed, final_direction=-1
        )
        elapsed = time.time() - began

        self.assertLessEqual(elapsed, cfg["planner_timeout"] + 0.25)
        self.assertEqual(reason, "planned")
        self.assertTrue(path)
        self.assertIn(path[0][3], (-1, 1))
        self.assertEqual(path[-1][3], -1)
        self.assertGreaterEqual(
            sum(path[index][3] != path[index - 1][3] for index in range(1, len(path))),
            1,
        )
        self.assertLessEqual(
            max(geometry.distance(path[index - 1], path[index]) for index in range(1, len(path))),
            cfg["planner_step"] + 1e-9,
        )

        follower, pose, zero_steps, steps = _simulate_parking(path, start, cfg)

        self.assertTrue(follower.done)
        self.assertGreater(zero_steps, 0)
        self.assertLessEqual(steps, 1600)
        self.assertLessEqual(geometry.distance(pose, goal), cfg["path_goal_tolerance"])
        self.assertLessEqual(
            abs(geometry.wrap(pose[2] - goal[2])),
            cfg["path_yaw_tolerance"],
        )

    def test_p4_perpendicular_parking_plans_and_follows(self):
        self._assert_parking_scenario("P4", (0.65, -0.55), math.pi / 2.0)

    def test_p1_parallel_parking_plans_and_follows(self):
        self._assert_parking_scenario("P1", (0.60, -0.45), 0.0)

    def test_p4_explicit_forward_departure_keeps_reverse_goal(self):
        self._assert_parking_scenario(
            "P4", (0.65, -0.55), math.pi / 2.0, require_initial_forward=True
        )


if __name__ == "__main__":
    unittest.main()
