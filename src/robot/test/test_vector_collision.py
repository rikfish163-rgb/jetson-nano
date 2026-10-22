"""Array acceleration retains the scalar footprint and every raw return."""
import math
import os
import random
import unittest
import numpy as np
from robot.common.config import load_config
from robot.common.geometry import collision, world
from robot.master.controller import Controller


class VectorCollisionTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config(os.path.join(os.path.dirname(__file__), '../config'))

    def compare(self, pose, points):
        self.assertEqual(collision(pose, points, self.cfg),
                         collision(pose, np.asarray(points).reshape(-1, 2), self.cfg))

    def test_random_rotated_clouds_match_scalar(self):
        rng = random.Random(42)
        for unused in range(100):
            pose = (rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-math.pi, math.pi))
            points = [(rng.uniform(-2, 2), rng.uniform(-2, 2)) for i in range(50)]
            self.compare(pose, points)

    def test_boundary_and_empty_clouds_match_scalar(self):
        cfg = self.cfg
        lo = -cfg['rear_overhang']-cfg['obstacle_margin']
        hi = cfg['wheelbase']+cfg['front_overhang']+cfg['obstacle_margin']
        half = cfg['body_width']/2+cfg['obstacle_margin']
        for pose in ((0, 0, 0), (1, 2, .7)):
            self.compare(pose, [])
            for x in (lo-1e-8, lo, hi, hi+1e-8):
                for y in (-half-1e-8, -half, 0, half, half+1e-8):
                    self.compare(pose, [world(pose, (x, y))])
        self.compare((0, 0, 0), [(float('nan'), 0), (10, 10)])

    def test_mutated_raw_obstacles_are_not_cached_or_filtered(self):
        cfg = self.cfg
        cfg.update(wait_green=False, lidar_enabled=True)
        c = Controller(cfg)
        self.addCleanup(c.close)
        c.scan = type('Scan', (), dict(obstacles=[(3, 3)], shape_filter=True,
                                      shape_mode='line_reject', round_clusters=[],
                                      motion_obstacles=[], stamp=1))()
        self.assertTrue(c.sweep_clear([(0, 0, 0)], False))
        c.scan.obstacles[:] = [(.2, 0)]
        self.assertFalse(c.sweep_clear([(0, 0, 0)], False))
