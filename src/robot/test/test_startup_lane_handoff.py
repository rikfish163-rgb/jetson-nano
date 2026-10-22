"""The 1.25 m start must hand a fresh curve to lane control immediately."""
import os
import unittest
from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.master.controller import Controller


class StartupLaneHandoffTests(unittest.TestCase):
    def test_slow_lane_handoff_does_not_require_detected_curve(self):
        paths = ([ (.5,0),(.7,0),(.9,0),(1.1,0) ],
                 [ (.5,0),(.7,-.04),(.9,-.12),(1.1,-.24) ],
                 [ (.5,0),(.7,.01),(.9,.02) ])
        for path in paths:
            cfg=load_config(os.path.join(os.path.dirname(__file__),'../config'))
            cfg.update(wait_green=True,lidar_enabled=False,parking_enabled=False,
                       lane_curvature_preview=True,straight_distance=1.25,
                       straight_speed_raw=24,lane_curve_speed_raw=12)
            cfg['speed_raw'].update(lane=12,action=24,gap=16)
            core=Controller(cfg)
            try:
                for i in range(int(cfg['sign_votes'])):
                    core.observe_sign('GREEN',.99,1.+i*.01,1.+i*.01)
                core.observe_ground(dict(source='front',part='markers',markers=[],slots=[]),1.1)
                self.assertEqual(core.tick(1.1),(24,0))
                core.set_pose((1.24,0,0),1.2)
                self.assertEqual(core.tick(1.2),(24,0))
                core.set_pose((1.25,0,0),1.3)
                core.observe_lane(path,.9,1.3)
                core.observe_ground(dict(source='front',part='markers',markers=[],slots=[]),1.3)
                self.assertEqual(core.tick(1.3)[0],12)
                self.assertEqual(core.state,'LANE')
                core.observe_lane([],.9,1.4)
                core.observe_ground(dict(source='front',part='markers',markers=[],slots=[]),1.4)
                self.assertEqual(core.tick(1.4)[0],12)
                self.assertEqual(core.state,'GAP')
            finally:
                core.close()

    def test_both_curve_directions_steer_at_the_distance_handoff(self):
        for sign in (-1,1):
            cfg=load_config(os.path.join(os.path.dirname(__file__),'../config'))
            cfg.update(wait_green=True,lidar_enabled=False,parking_enabled=False,
                       lane_curvature_preview=True,lookahead=.8,
                       steering_command_scale_rad=.03)
            core=Controller(cfg)
            try:
                for i in range(int(cfg['sign_votes'])):
                    core.observe_sign('GREEN',.99,1.+i*.01,1.+i*.01)
                core.observe_ground(dict(source='front',part='markers',markers=[],slots=[]),1.1)
                self.assertEqual(core.tick(1.1),(20,0))
                self.assertEqual(core.state,'STARTUP_STRAIGHT')
                core.set_pose((1.25,0,0),1.2)
                core.observe_lane([(.5,0),(.7,sign*.04),(.9,sign*.12),
                                   (1.1,sign*.24)],.9,1.2)
                core.observe_ground(dict(source='front',part='markers',markers=[],slots=[]),1.2)
                speed,steer=core.tick(1.2)
                raw=encode_command(speed,steer,cfg,0)['steering_raw']
                self.assertEqual(core.state,'LANE')
                self.assertEqual(core.reason,'tracking_lane')
                self.assertGreater(speed,0)
                self.assertGreater(raw*sign,0)
                self.assertIsNone(core.startup_started)
            finally:
                core.close()
