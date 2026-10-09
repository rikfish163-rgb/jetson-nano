import os
import sys
import unittest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))
from robot.common.contracts import validate_config


class DefaultConfigResetTests(unittest.TestCase):
    def test_default_profile_overwrites_previous_course_test_mode(self):
        cfg = dict(uturn_course_test=True)
        for name in ('competition.yaml', 'maneuvers.yaml'):
            with open(os.path.join(ROOT, 'config', name)) as f:
                cfg.update(yaml.safe_load(f))
        self.assertFalse(cfg['uturn_course_test'])
        validate_config(cfg)

    def test_production_lidar_category_defaults_are_explicit(self):
        from robot.common.config import load_config
        cfg = load_config(os.path.join(ROOT, 'config'))
        self.assertTrue(cfg['straight_lidar_once'])
        self.assertFalse(cfg['parking_lidar_enabled'])
        validate_config(cfg)
