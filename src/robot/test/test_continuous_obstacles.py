"""Offline regressions for continuous obstacle protection.

These tests use only the in-process Controller and synthetic scans.  They do
not start ROS nodes and do not exercise a vehicle or actuator.
"""
from __future__ import division
import os
import sys
import unittest

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))

from robot.master.controller import Controller


class _SyntheticScan(object):
    """Small scan fixture with explicit freshness and occupancy semantics."""

    def __init__(self, stamp, obstacles=(), valid_rays=360,
                 shape_filter=True, classification='FREE'):
        self.stamp = stamp
        self.valid_rays = valid_rays
        self.obstacles = list(obstacles)
        self.motion_obstacles = list(obstacles)
        self.shape_filter = shape_filter
        self.shape_mode = 'line_reject'
        self.round_clusters = []
        self._classification = classification

    def classify(self, point):
        if self._classification == 'FREE':
            return 'FREE'
        return self._classification

    def evidence(self, point):
        return dict(source='synthetic_scan')


class ContinuousObstacleTests(unittest.TestCase):
    def core(self):
        with open(os.path.join(ROOT, 'config', 'competition.yaml')) as stream:
            cfg = yaml.safe_load(stream)
        cfg.update(wait_green=False, lidar_enabled=True,
                   timed_bypass_enabled=True, timed_bypass_settle_s=0.0,
                   timed_bypass_left_s=0.1, timed_bypass_right_s=0.1)
        controller = Controller(cfg)
        self.addCleanup(controller.close)
        return controller

    @staticmethod
    def scan(controller, stamp, obstacles=(), valid_rays=360,
             shape_filter=True, classification='FREE'):
        controller.scan = _SyntheticScan(
            stamp, obstacles, valid_rays, shape_filter, classification)

    def test_completed_bypass_does_not_disable_lidar_for_later_near_obstacle(self):
        controller = self.core()
        # This is the old failure: completion was incorrectly treated as a
        # permanent global lidar bypass.
        controller.timed_bypass_completed = True
        self.scan(controller, 1.0, obstacles=[(.20, 0.0)])

        self.assertEqual(controller.checked_command((20, 0.0), 1.0, False),
                         (0, 0.0))
        self.assertEqual(controller.obstacle_check['kind'], 'obstacle')

    def test_timed_bypass_keeps_near_field_collision_guard(self):
        controller = self.core()
        self.confirm_target(controller,1.0)

        # The obstacle is now in the active turn's near-field sweep.  A timed
        # maneuver must stop on it instead of unconditionally issuing the turn.
        self.scan(controller, 1.5, obstacles=[(.20, 0.0)])
        self.assertEqual(controller.tick(1.5), (0, 0.0))
        self.assertEqual(controller.obstacle_check['kind'], 'obstacle')

    def test_timed_bypass_stops_on_unknown_near_field_space(self):
        controller = self.core()
        self.confirm_target(controller,1.0)

        # A valid scan with unresolved occupancy is still unsafe for the
        # active turn; unknown space must stop rather than be treated as free.
        self.scan(controller, 1.5, valid_rays=360, classification='UNKNOWN')
        self.assertEqual(controller.tick(1.5), (0, 0.0))
        self.assertEqual(controller.obstacle_check['kind'], 'unknown')

    def test_completed_bypass_allows_a_distinct_later_obstacle_to_trigger(self):
        controller = self.core()
        controller.timed_bypass_completed = True
        controller.timed_bypass_seen = [dict(point=(.20, 0.0), stamp=1.0)]
        self.confirm_target(controller,2.0)
        result = controller.checked_command((20, 0.0), 2.4, False)
        self.assertGreater(result[0], 0)
        self.assertEqual(controller.state, 'TIMED_BYPASS')

    def confirm_target(self,controller,start):
        for i in range(3):
            now=start+i*.2
            controller.observe_lane([(.5,0),(.7,0),(.9,0),(1.1,0)],.9,now)
            self.scan(controller,now,obstacles=[(.49,0.)])
            controller.execute('obstacle','begin_timed_bypass',now)
        self.assertEqual(controller.state,'TIMED_BYPASS')

    def test_completed_entity_is_not_triggered_again_after_protection_reopens(self):
        controller = self.core()
        controller.timed_bypass_seen = []
        controller.timed_bypass = dict(
            phase='REACQUIRE', elapsed_s=0.0, last=1.0,
            trigger_point=(.49, 0.0), trigger_world=(.49, 0.0))
        controller.state = 'TIMED_BYPASS'
        controller.action = 'BYPASS'

        # Keep completion focused on the obstacle module's bookkeeping.
        controller._runtime.overrides.update({
            'lane_valid': lambda *args: True,
            'lane_tracking_ready': lambda *args: True,
            'lane_command': lambda *args: (20, 0.0),
            'checked_command': lambda *args: args[0],
        })
        controller.execute('obstacle', 'timed_bypass_tick', 1.1)
        controller._runtime.overrides.pop('checked_command', None)
        self.assertEqual(len(controller.timed_bypass_seen), 1)

        # Simulate the protection being active again after the event.  The
        # same entity is ignored, while the normal sweep still remains armed.
        controller.timed_bypass_completed = False
        self.scan(controller, 1.2, obstacles=[(.49, 0.0)])
        self.assertEqual(controller.checked_command((20, 0.0), 1.2, True),
                         (20, 0.0))
        self.assertIsNone(controller.timed_bypass)
        self.assertEqual(controller.state, 'LANE')

    def test_lidar_disabled_remains_an_explicit_bypass(self):
        controller = self.core()
        controller.cfg['lidar_enabled'] = False
        controller.scan = None

        self.assertEqual(controller.checked_command((20, 0.0), 1.0, True),
                         (20, 0.0))
        self.assertEqual(controller.obstacle_check,
                         dict(kind='disabled', reason='config_disabled'))

    def test_shape_filter_does_not_authorize_unknown_ordinary_space(self):
        controller = self.core()
        self.scan(controller, 1., classification='UNKNOWN')
        self.assertEqual(controller.checked_command((20, 0.), 1., False), (0, 0.))
        self.assertEqual(controller.obstacle_check['kind'], 'unknown')

    def test_non_target_raw_obstacle_still_stops_collision_sweep(self):
        controller = self.core()
        self.scan(controller, 1., obstacles=[(.20, 0.)])
        controller.scan.motion_obstacles = []
        self.assertEqual(controller.checked_command((20, 0.), 1., False), (0, 0.))
        self.assertEqual(controller.obstacle_check['kind'], 'obstacle')


if __name__ == '__main__':
    unittest.main()
