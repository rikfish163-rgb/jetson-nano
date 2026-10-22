"""Isolated max-left handoff tests; never create a ROS node or publisher."""
import os
import sys
import math
import unittest
import yaml
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.join(ROOT,'src'))
from robot.turn.left_lock import LeftLockController
from robot.lidar.scan import Scan


class LeftLockEntryTest(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(ROOT,'config','competition.yaml')) as f:
            cfg = yaml.safe_load(f)
        self.c = LeftLockController(cfg,18,12,3)
        self.addCleanup(self.c.close)

    def step(self,t,points=None):
        self.c.scan = Scan([float('inf')]*360,-math.pi,math.pi/180,.05,6,
                           self.c.pose,self.c.cfg['lidar'],t)
        if points is not None:
            self.c.observe_lane(points,.8,t)
        return self.c.tick(t)

    def test_no_line_means_constant_full_left(self):
        for t in (1,1.1,1.2):
            self.assertEqual(self.step(t),(18,self.c.cfg['max_steer']))

    def test_three_forward_lane_frames_handoff_without_ninety_degree_gate(self):
        points = [(.55,0),(.65,0),(.85,0)]
        for t in (1,1.1,1.2):
            self.assertEqual(self.step(t,points),(18,self.c.cfg['max_steer']))
        command = self.step(1.3,points)
        self.assertEqual(self.c.pose[2],0)
        self.assertEqual(self.c.state,'LANE')
        self.assertIsNone(self.c.action)
        self.assertEqual(command,(self.c.cfg['speed_raw']['lane'],0.0))
        self.assertEqual(self.step(1.4,points),command)

    def test_same_frame_does_not_complete_handoff(self):
        self.step(1,[(.55,0),(.65,0),(.85,0)])
        self.step(1.1)
        self.step(1.2)
        self.assertEqual(self.c.state,'LEFT_LOCK')

    def test_cross_line_not_accepted(self):
        for t in (1,1.1,1.2):
            self.step(t,[(.55,0),(.55,.3)])
        self.assertEqual(self.c.state,'LEFT_LOCK')

    def test_timeout_latches_stop(self):
        self.step(1)
        self.assertEqual(self.step(13.1),(0,0.0))
        self.assertEqual(self.c.reason,'left_lock_timeout')
        self.assertEqual(self.step(13.2,[(.55,0),(.65,0)]),(0,0.0))

    def test_estop_and_missing_lidar_still_stop(self):
        self.assertEqual(self.c.tick(1),(0,0.0))
        self.c.estop = True
        self.assertEqual(self.step(2),(0,0.0))

    def test_no_direction_or_parking_task_can_interrupt_test(self):
        for t in (1,1.1,1.2):
            self.c.observe_sign('PARKING',.99,t,t)
        self.assertIsNone(self.c.pending)
        self.assertEqual(self.c.state,'LEFT_LOCK')


if __name__ == '__main__':
    unittest.main()
