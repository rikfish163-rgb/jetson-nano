import os
import sys
import unittest
import yaml
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT,'src'))
from robot.master.controller import Controller
from test_straight_alignment import finish_blue_exit

class StraightBlueExitTests(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(ROOT,'config','competition.yaml')) as f: cfg=yaml.safe_load(f)
        cfg.update(wait_green=False,lidar_enabled=False)
        self.c=Controller(cfg);self.addCleanup(self.c.close)
        self.c.pending='STRAIGHT'
        self.front(1,.3)
        self.c.tick(1)
        self.front(2)
        self.c.tick(2)

    def front(self,t,x=None,kind='junction'):
        self.c.observe_ground(dict(source='front',part='markers',slots=[],markers=[] if x is None else
            [dict(kind=kind,x=x,y=0)]),t)

    def test_white_alone_never_releases(self):
        for t in (2.1,2.2,2.3,2.4):
            self.front(t)
            self.c.observe_lane([(.3,0),(.5,0),(.7,0)],.99,t)
            self.assertEqual(self.c.tick(t),(26,0))
        self.assertEqual(self.c.state,'MANEUVER')

    def test_same_line_far_line_and_short_line_do_not_release(self):
        for t,x,kind in ((2.1,.3,'junction'),(2.2,1.0,'junction'),(2.3,.3,'tick')):
            self.front(t,x,kind)
            self.assertEqual(self.c.tick(t),(26,0))
            self.assertEqual(self.c.state,'MANEUVER')

    def test_next_line_consumed_and_legacy_queued_sign_discarded(self):
        self.c.next_direction='RIGHT'
        self.c.set_pose((.7,0,0),2.5)
        self.front(2.5,.3)
        self.c.observe_lane([(.3,0),(.5,0),(.7,0)],.99,2.5)
        finish_blue_exit(self.c,2.5)
        self.assertEqual(self.c.state,'LANE')
        self.assertIsNone(self.c.pending)
        self.front(2.9);self.c.tick(2.9)
        self.assertIsNone(self.c.action)

    def test_next_line_without_white_releases_but_stops(self):
        self.c.set_pose((.7,0,0),2.5);self.front(2.5,.3)
        self.assertEqual(finish_blue_exit(self.c,2.5),(0,0))
        self.assertEqual(self.c.state,'LANE')

    def test_left_curving_lane_takes_control_only_after_next_blue(self):
        for t in (2.1,2.2):
            self.c.observe_sign('LEFT',.99,t,t)
        self.c.observe_lane([(.3,.03),(.5,.08),(.7,.12)],.99,2.3)
        self.front(2.3)
        self.assertEqual(self.c.tick(2.3),(26,0))
        self.c.set_pose((.7,0,0),2.5)
        self.c.observe_lane([(.3,.03),(.5,.08),(.7,.12)],.99,2.5)
        self.front(2.5,.3)
        speed,steer=finish_blue_exit(self.c,2.5)
        self.assertGreater(speed,0)
        self.assertGreater(steer,0)
        self.assertEqual(self.c.state,'LANE')
        self.assertIsNone(self.c.action)
        self.assertIsNone(self.c.pending)

    def test_stale_blue_does_not_release(self):
        self.c.set_pose((.7,0,0),2.5);self.front(2.5,.3)
        self.assertEqual(self.c.tick(4),(0,0))
        self.assertEqual(self.c.state,'MANEUVER')

if __name__=='__main__': unittest.main()
