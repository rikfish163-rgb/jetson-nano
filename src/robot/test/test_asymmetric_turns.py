"""Independent left/right geometry and exit handoff; no hardware output."""
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
from robot.common.geometry import world
from robot.common.geometry import wrap
from robot.common.planning import intersection_path
from robot.common.contracts import validate_config


class AsymmetricTurnTests(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(ROOT,'config','competition.yaml')) as f:
            self.cfg = yaml.safe_load(f)
        self.cfg.update(wait_green=False,turn_entry=.45,turn_radius=.65,turn_exit=.30,
            left_turn_entry=.40,left_turn_radius=.60,left_turn_exit=.20,left_turn_angle_deg=85,
            right_turn_entry=.20,right_turn_radius=.90,right_turn_exit=.75,right_turn_angle_deg=70)

    def test_left_and_right_endpoints_use_their_own_geometry(self):
        for action,side,sign in (('LEFT','left',1),('RIGHT','right',-1)):
            path = intersection_path((0,0,0),action,self.cfg)
            radius = self.cfg[side+'_turn_radius']
            angle = math.radians(self.cfg[side+'_turn_angle_deg'])
            entry,tail = self.cfg[side+'_turn_entry'],self.cfg[side+'_turn_exit']
            expected = (entry+radius*math.sin(angle)+tail*math.cos(angle),
                        sign*(radius*(1-math.cos(angle))+tail*math.sin(angle)),sign*angle)
            for actual,wanted in zip(path[-1][:3],expected):
                self.assertAlmostEqual(actual,wanted,places=7)

    def test_changing_right_settings_does_not_change_left_path(self):
        original = intersection_path((0,0,0),'LEFT',self.cfg)
        self.cfg.update(right_turn_radius=1.2,right_turn_entry=.7,right_turn_angle_deg=110,right_turn_exit=.9)
        self.assertEqual(intersection_path((0,0,0),'LEFT',self.cfg),original)

    def test_right_handoff_uses_right_tail_not_shared_tail(self):
        c = Controller(self.cfg)
        self.addCleanup(c.close)
        c.start_follow(intersection_path((0,0,0),'RIGHT',self.cfg),'RIGHT',1)
        end = c.exit_pose
        c.set_pose(tuple(world(end,(-self.cfg['right_turn_exit']+.03,0)))+(end[2],),2)
        self.assertTrue(c.direction_exit_reached())
        c.set_pose(tuple(world(end,(-self.cfg['right_turn_exit']-.03,0)))+(end[2],),3)
        self.assertFalse(c.direction_exit_reached())

    def test_radius_is_still_limited_by_vehicle_steering(self):
        self.cfg.update(right_turn_entry=0,right_turn_exit=0,right_turn_radius=.1,right_turn_angle_deg=90)
        path = intersection_path((0,0,0),'RIGHT',self.cfg)
        minimum = self.cfg['wheelbase']/math.tan(self.cfg['max_steer'])
        self.assertAlmostEqual(path[-1][0],minimum,places=7)
        self.assertAlmostEqual(path[-1][1],-minimum,places=7)
        self.assertTrue(all(abs(p[4]) <= self.cfg['max_steer']+1e-10 for p in path))

    def test_legacy_parameters_remain_the_fallback(self):
        legacy = dict((k,v) for k,v in self.cfg.items() if not k.startswith(('left_turn_','right_turn_')))
        explicit = copy.deepcopy(legacy)
        for side in ('left','right'):
            for key in ('turn_entry','turn_radius','turn_exit','turn_angle_deg'):
                explicit[side+'_'+key] = legacy[key]
        for action in ('LEFT','RIGHT','STRAIGHT','UTURN'):
            self.assertEqual(intersection_path((0,0,0),action,legacy),intersection_path((0,0,0),action,explicit))

    def test_invalid_side_geometry_is_rejected(self):
        for key,value in (('left_turn_radius',float('nan')),('right_turn_radius',0),
                          ('right_turn_entry',-.1),('left_turn_exit',float('inf')),
                          ('right_turn_angle_deg',181)):
            cfg = copy.deepcopy(self.cfg)
            cfg[key] = value
            with self.assertRaises(ValueError):
                validate_config(cfg)


if __name__ == '__main__':
    unittest.main()
