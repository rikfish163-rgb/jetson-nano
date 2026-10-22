import os
import sys
import unittest
import yaml
ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.join(ROOT,'src'))
from robot.master.controller import Controller

class ActionSignLockTests(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(ROOT,'config','competition.yaml')) as f: cfg=yaml.safe_load(f)
        cfg.update(wait_green=False,lidar_enabled=False)
        self.c=Controller(cfg);self.addCleanup(self.c.close)

    def vote(self,label,t):
        for i in range(3):self.c.observe_sign(label,.99,t+i*.1,t+i*.1)

    def test_pending_direction_can_be_corrected_before_blue(self):
        self.vote('STRAIGHT',1)
        for i,label in enumerate(('RIGHT','LEFT','UTURN','STRAIGHT')):
            self.vote(label,2+i)
            self.assertEqual(self.c.pending,label)
            self.assertIsNone(self.c.next_direction)
        self.vote('PARKING',8)
        self.assertEqual(self.c.pending,'STRAIGHT')

    def test_active_action_ignores_votes_and_requires_new_votes_after_exit(self):
        self.c.state,self.c.action='MANEUVER','STRAIGHT'
        self.vote('RIGHT',1)
        self.assertIsNone(self.c.next_direction)
        self.c.resume_lane()
        self.c.observe_sign('RIGHT',.9,2,2)
        self.assertIsNone(self.c.pending)
        self.c.observe_sign('RIGHT',.9,2.2,2.2)
        self.assertEqual(self.c.pending,'RIGHT')

    def test_parking_and_active_actions_ignore_route_signs(self):
        for i,state in enumerate(('PARKING','INTERSECTION_WAIT','UTURN')):
            self.c.state=state
            self.c.action='PARKING'
            self.vote('LEFT',1+i)
            self.assertIsNone(self.c.pending)
            self.assertIsNone(self.c.next_direction)

    def test_red_stop_and_green_release_remain_available(self):
        self.c.state,self.c.action='MANEUVER','STRAIGHT'
        self.vote('RED',1);self.assertTrue(self.c.red)
        self.vote('GREEN',2);self.assertFalse(self.c.red)
        self.assertEqual(self.c.action,'STRAIGHT')

if __name__=='__main__':unittest.main()
