#!/usr/bin/env python
"""Standalone measured-scene tests for P1/P2/P3 parallel parking."""
from __future__ import division

import copy
import math
import os
import sys
import unittest

import yaml


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))

from robot.common import geometry as geometry# noqa: E402
from robot.parallel_parking import planner as parallel_parking# noqa: E402
from robot.parallel_parking.planner import ParallelParking
from robot.parallel_parking.planner import decode_parallel_scene
from robot.parallel_parking.planner import inside_slot


with open(os.path.join(ROOT, 'config', 'competition.yaml')) as stream:
    CONFIG = yaml.safe_load(stream)


class ParallelParkingTest(unittest.TestCase):
    def test_reject_huge_finite_pose_and_slot_yaw(self):
        for field in ('pose','slot'):
            raw=self.scene()
            if field=='pose':raw['pose']=[0,0,1e308]
            else:raw['slot']['pose']=[.6,-.45,1e308]
            with self.assertRaises(ValueError):decode_parallel_scene(raw)

    def test_changed_map_checks_remaining_path_not_already_travelled_poses(self):
        task=self.task()
        task.accept_plan(task.plan())
        past=task.follower.path[0][:2]
        task.follower.index=len(task.follower.path)-1
        task.follower.segment_end=task.follower.index
        task.observe(dict(self.scene(stamp=1.1),pose=task.goal,obstacles=[list(past)]))
        task.command(1.1)
        self.assertNotEqual(task.phase,'BLOCKED')

    def test_parking_goal_yaw_tolerance_is_local_and_consistent(self):
        cfg=copy.deepcopy(CONFIG)
        cfg['path_yaw_tolerance']=.5
        cfg['parallel_parking_goal_yaw_tolerance_deg']=5
        task=ParallelParking(cfg,self.scene())
        self.assertAlmostEqual(task.cfg['path_yaw_tolerance'],math.radians(5))
        self.assertEqual(cfg['path_yaw_tolerance'],.5)

    def scene(self, side=-1, stamp=1.0, slot_pose=None, width=.36):
        if slot_pose is None:
            slot_pose = (.60, side * .45, 0.0)
        return dict(
            frame='parallel_local', pose=(0.0, 0.0, 0.0), stamp=stamp,
            pose_source='vision',
            regions=[[(-2.0, -2.0), (2.0, -2.0),
                      (2.0, 2.0), (-2.0, 2.0)]],
            obstacles=[],
            slot=dict(id='P1', pose=slot_pose, length=.70, width=width),
        )

    def task(self, side=-1, width=.36):
        return ParallelParking(copy.deepcopy(CONFIG), self.scene(side, width=width))

    def test_scene_requires_measured_pose_and_complete_slot(self):
        raw = self.scene()
        for change in (dict(pose_source='command_model'),
                       dict(slot=dict(id='P1', pose=(.6, -.45, 0),
                                      length=.70)),
                       dict(regions=[])):
            candidate = copy.deepcopy(raw)
            candidate.update(change)
            with self.assertRaises(ValueError):
                decode_parallel_scene(candidate)

    def test_double_arc_mirrors_right_and_left_parallel_bays(self):
        for side, expected_steer_sign in ((-1, -1), (1, 1)):
            task = self.task(side)
            path, reason = task.plan()
            self.assertEqual(reason, 'parallel_parking_double_arc')
            self.assertTrue(path)
            self.assertIn(-1, [row[3] for row in path])
            self.assertEqual(path[-1][3], -1)
            first_reverse = next(row for row in path if row[3] == -1)
            self.assertGreater(first_reverse[4] * expected_steer_sign, 0)
            self.assertLess(geometry.distance(path[-1], task.goal), 1e-6)
            self.assertTrue(inside_slot(path[-1][:3], task.slot, task.cfg))

    def test_narrow_slot_fails_closed_before_search(self):
        task = self.task(width=.20)
        path, reason = task.plan()
        self.assertEqual(path, [])
        self.assertEqual(reason, 'parallel_parking_slot_too_narrow')
        task.accept_plan((path, reason))
        self.assertEqual(task.phase, 'BLOCKED')

    def test_body_margin_is_checked_in_addition_to_four_wheels(self):
        task = self.task(width=.25)
        task.cfg['parallel_parking_slot_body_margin_m'] = .01
        path, reason = task.plan()
        self.assertEqual(path, [])
        self.assertEqual(reason, 'parallel_parking_slot_too_narrow')

    def test_pose_xy_bound_and_self_crossing_star_are_rejected(self):
        outside = self.scene()
        outside['pose'] = (20.01, 0, 0)
        with self.assertRaises(ValueError):
            decode_parallel_scene(outside)
        star = self.scene()
        star['regions'] = [[(0, 0), (2, 2), (0, 1), (2, 0), (0, 2)]]
        with self.assertRaises(ValueError):
            decode_parallel_scene(star)

    def test_parallel_speed_override_is_local_to_the_task(self):
        cfg = copy.deepcopy(CONFIG)
        cfg['parallel_parking_speed_raw'] = 12
        self.assertEqual(cfg['speed_raw']['parking'], 16)
        task = ParallelParking(cfg, self.scene())
        self.assertEqual(task.cfg['speed_raw']['parking'], 12)
        self.assertEqual(cfg['speed_raw']['parking'], 16)

    def test_executor_timeout_fails_closed(self):
        cfg = copy.deepcopy(CONFIG)
        cfg['parallel_parking_timeout_s'] = .05
        task = ParallelParking(cfg, self.scene())
        task.accept_plan(task.plan())
        task.observe(dict(self.scene(stamp=1.1), pose=list(task.start)))
        self.assertEqual(task.command(1.1), (0, 0.0))
        self.assertEqual(task.phase, 'BLOCKED')
        self.assertEqual(task.reason, 'parallel_parking_timeout')

    def test_hybrid_is_fallback_when_double_arc_heading_is_not_usable(self):
        task = self.task()
        task.slot['pose'] = (.60, -.45, .20)
        task.goal = parallel_parking._goal_from_cfg(task.start, task.slot, task.cfg)
        fallback = [tuple(task.start) + (-1, 0.0),
                    tuple(task.goal) + (-1, 0.0)]
        original = parallel_parking.hybrid_plan
        calls = []

        def fake_hybrid(start, goal, cfg, obstacles=(), allowed=None,
                        final_direction=-1):
            calls.append((start, goal, final_direction))
            return fallback, 'planned'

        parallel_parking.hybrid_plan = fake_hybrid
        try:
            path, reason = task.plan()
        finally:
            parallel_parking.hybrid_plan = original
        self.assertEqual(reason, 'parallel_parking_hybrid')
        self.assertEqual(path, fallback)
        self.assertEqual(calls[0][2], -1)

    def test_observation_keeps_locked_slot_and_rejects_reference_change(self):
        task = self.task()
        original_goal = task.goal
        updated = self.scene(stamp=1.1, slot_pose=(.61, -.45, .01))
        task.observe(updated)
        self.assertEqual(task.goal, original_goal)
        self.assertEqual(task.scene['pose'], updated['pose'])
        changed = self.scene(stamp=1.2)
        changed['slot']['id'] = 'P2'
        task.observe(changed)
        self.assertEqual(task.phase, 'BLOCKED')
        self.assertEqual(task.reason, 'parallel_parking_reference_changed')

    def test_execution_finishes_only_after_fresh_slot_confirmations(self):
        task = self.task()
        planned = task.plan()
        task.accept_plan(planned)
        self.assertEqual(task.phase, 'TRACK')
        # The path follower owns motion tracking.  Marking it complete here
        # isolates this class's measured terminal-confirmation contract from
        # the separate bicycle-following simulation tests.
        task.follower.done = True
        pose = list(task.goal)
        now = 1.1
        task.observe(dict(self.scene(stamp=now), pose=pose))
        self.assertEqual(task.command(now), (0, 0.0))
        self.assertEqual(task.phase, 'CONFIRM')
        for stamp in (now, now + .1):
            task.observe(dict(self.scene(stamp=stamp), pose=pose))
            self.assertEqual(task.command(stamp), (0, 0.0))
            self.assertEqual(task.phase, 'CONFIRM')
        stamp = now + .2
        task.observe(dict(self.scene(stamp=stamp), pose=pose))
        self.assertEqual(task.command(stamp), (0, 0.0))
        self.assertEqual(task.phase, 'DONE')
        self.assertEqual(task.reason, 'parallel_parking_complete')

    def test_stale_pose_and_blocked_remaining_path_stop_motion(self):
        task = self.task()
        task.accept_plan(task.plan())
        self.assertEqual(task.command(2.0), (0, 0.0))
        self.assertEqual(task.reason, 'parallel_parking_pose_stale')
        task = self.task()
        task.accept_plan(task.plan())
        blocked = task.follower.path[20]
        task.observe(dict(self.scene(stamp=1.1, slot_pose=(.60, -.45, 0.0)),
                           obstacles=[list(blocked[:2])]))
        self.assertEqual(task.command(1.1), (0, 0.0))
        self.assertEqual(task.phase, 'BLOCKED')
        self.assertEqual(task.reason, 'parallel_parking_remaining_path_blocked')


if __name__ == '__main__':
    unittest.main()
