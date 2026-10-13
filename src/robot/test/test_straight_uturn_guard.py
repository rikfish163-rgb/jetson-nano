import os
import unittest
from robot.common.config import load_config
from robot.master.controller import Controller


class StraightUturnGuardTests(unittest.TestCase):
    def setUp(self):
        cfg = load_config(os.path.join(os.path.dirname(__file__), '../config'))
        cfg.update(wait_green=False, lidar_enabled=False, straight_distance=1.4,
                   direction_sign_votes=1)
        self.c = Controller(cfg)
        self.addCleanup(self.c.close)
        self.c.start_follow([(0,0,0,1,0),(1.4,0,0,1,0)], 'STRAIGHT', 10.)

    def ground(self, stamp, x=None):
        self.c.observe_ground(dict(source='front', part='markers', slots=[],
            markers=[] if x is None else [dict(kind='junction',x=x,y=0.,length=.6)],
            blue_lines=[] if x is None else [dict(x=x,y=0.,yaw=0.,length=.6)]),stamp)




    def test_parking_bypasses_straight_distance_guard(self):
        c=self.c
        c.cfg.update(parking_sign_votes=1, parking_side_lidar_enabled=False)
        c.set_pose((.2,0,0),12.)
        c.observe_sign('PARKING',.99,12.,12.)
        self.assertEqual(c.next_direction,'PARKING')
        c.front_marker_stamp=12.
        c.execute('turn','straight_search_tick',12.)
        self.assertEqual(c.action,'PARKING')

    def test_red_and_green_remain_immediate(self):
        self.c.observe_sign('RED',.99,12.,12.)
        self.assertTrue(self.c.red)
        self.c.observe_sign('GREEN',.99,12.1,12.1)
        self.assertFalse(self.c.red)


    def test_next_maneuver_clears_straight_guard(self):
        self.c.start_follow([(0,0,0,1,0),(1,0,0,1,0)],'LEFT',12.)
        self.assertIsNone(self.c.straight_uturn_guard)

if __name__=='__main__':unittest.main()
