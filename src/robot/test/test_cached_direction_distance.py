import os
import unittest
from robot.common.config import load_config
from robot.master.controller import Controller

class CachedDirectionDistanceTests(unittest.TestCase):
    def test_cached_uturn_survives_and_dispatches_at_distance(self):
        self.check_cached_direction('UTURN',1.4)

    def test_cached_right_waits_for_1p5(self):
        self.check_cached_direction('RIGHT',1.5)

    def check_cached_direction(self,label,limit):
        cfg=load_config(os.path.join(os.path.dirname(__file__),'../config'))
        cfg.update(wait_green=False,lidar_enabled=False,straight_distance=limit,direction_sign_votes=1)
        c=Controller(cfg);self.addCleanup(c.close)
        c.start_follow([(0,0,0,1,0),(1.4,0,0,1,0)],'STRAIGHT',10.)
        c.observe_sign(label,.99,12.,12.)
        self.assertEqual(c.next_direction,label)
        c.resume_lane()
        c.observe_sign('',0.,12.1,12.1)
        c.set_pose((limit-.01,0,0),13.)
        for t in (13.,13.2,13.4):
            c.observe_ground(dict(source='front',part='markers',slots=[],
                markers=[dict(kind='junction',x=.65,y=0.,length=.6)],
                blue_lines=[dict(x=.65,y=0.,yaw=0.,length=.6)]),t)
        c.dispatch(13.4)
        self.assertIsNone(c.action)
        self.assertEqual(c.pending,label)
        c.set_pose((limit,0,0),13.5)
        c.dispatch(13.5)
        self.assertEqual(c.action,label)

if __name__=='__main__':unittest.main()
