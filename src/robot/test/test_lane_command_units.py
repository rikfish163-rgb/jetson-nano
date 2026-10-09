import math
import unittest
import os
import yaml
from test_direction_single_frame import DirectionSingleFrameTests
from robot.common.contracts import encode_command

class LaneCommandUnitTests(unittest.TestCase):
    def core(self):
        root=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        with open(os.path.join(root,'config','competition.yaml')) as f:
            cfg=yaml.safe_load(f)
        cfg.update(wait_green=False,lidar_enabled=False)
        from robot.master.controller import Controller
        c=Controller(cfg);self.addCleanup(c.close)
        return c
    def test_lane_preserves_legacy_direct_angle_and_point_one_encoding(self):
        c=self.core();c.cfg.update(steering_command_scale_rad=.1,lookahead=.45)
        c.observe_lane([(.5,.1),(.8,.1)],.99,1)
        speed,steer=c.lane_command(1)
        physical=math.atan2(2*c.cfg['wheelbase']*.1,.26)
        self.assertAlmostEqual(steer,physical)
        raw=encode_command(speed,steer,c.cfg,0)['steering_raw']
        self.assertEqual(raw,c.cfg['steering_raw_limit'])
        self.assertAlmostEqual(c.gap_steer,steer)

    def test_default_straight_crosses_old_limit_and_exits_at_16m_blue(self):
        c=self.core()
        def front(t,x=None):
            c.observe_ground(dict(source='front',part='markers',slots=[],markers=[] if x is None else
                [dict(kind='junction',x=x,y=0)]),t)
        c.observe_sign('STRAIGHT',.6868,1,1)
        front(1,.3);self.assertEqual(c.tick(1),(0,0))
        front(2);self.assertEqual(c.tick(2),(26,0))
        c.set_pose((.9,0,0),7);front(7)
        self.assertEqual(c.tick(7),(26,0))
        self.assertEqual(c.state,'MANEUVER')
        c.set_pose((1.6,0,0),10);front(10,.3)
        c.observe_lane([(.5,0),(.8,0)],.99,10)
        self.assertEqual(c.tick(10),(c.cfg['speed_raw']['lane'],0))
        self.assertEqual(c.state,'LANE')
        self.assertIsNone(c.action)
