"""Fixed steering applies only to the post-blue signed STRAIGHT segment."""
import os
import unittest
from robot.common.config import load_config
from robot.common.contracts import encode_command, validate_config
from robot.master.controller import Controller
from blue_test_helpers import enter_blue_action, observe_direction


class FixedStraightTests(unittest.TestCase):
    def core(self):
        cfg=load_config(os.path.join(os.path.dirname(os.path.dirname(__file__)),'config'))
        cfg.update(wait_green=False,lidar_enabled=False,steering_command_scale_rad=.03)
        validate_config(cfg)
        c=Controller(cfg);self.addCleanup(c.close)
        return c

    def start(self,c):
        observe_direction(c,'STRAIGHT',.99,.6)
        enter_blue_action(c,'STRAIGHT',1.)
        self.assertEqual(c.action_source,'sign')

    def test_fixed_raw_with_and_without_opposing_blue_reference(self):
        for steering_sign in (1,-1):
            for lines in ([],[dict(x=.6,y=-.15,yaw=-1.57079632679,length=.7)]):
                c=self.core();c.cfg['steering_sign']=steering_sign
                self.start(c)
                c.observe_ground(dict(source='front',part='markers',markers=[],slots=[],blue_lines=lines),1.1)
                command=c.tick(1.1)
                self.assertEqual(encode_command(command[0],command[1],c.cfg,0)['steering_raw'],-2)
                self.assertEqual(c.reason,'straight_fixed_steer')
                self.assertIsNone(c.straight_search['right_blue'])

    def test_distance_completion_returns_to_normal_lane_command(self):
        c=self.core();self.start(c)
        c.pose=(c.cfg.get('straight_distance',1.25)+.01,0.,0.)
        c.front_marker_stamp=1.1
        c.lane_command=lambda now:(30,.009)
        self.assertEqual(c.tick(1.1),(30,.009))
        self.assertEqual(c.state,'LANE')

    def test_right_exit_straight_does_not_use_sign_trim(self):
        c=self.core();self.start(c);c.action_source='right_blue'
        c.front_marker_stamp=1.1
        self.assertEqual(c.tick(1.1)[1],0.)

    def test_stale_perception_still_stops(self):
        c=self.core();self.start(c)
        self.assertEqual(c.tick(3.)[0],0)
        self.assertEqual(c.reason,'straight_front_stale')

    def test_green_startup_is_independent_of_straight_trim(self):
        commands=[]
        for raw in (-2,0,2):
            c=self.core();c.cfg['straight_steering_raw']=raw
            c.begin_startup(1.)
            c.front_marker_stamp=1.1
            commands.append(c.tick(1.1))
        self.assertEqual(commands[0],commands[1])
        self.assertEqual(commands[1],commands[2])

    def test_invalid_raw_rejected(self):
        for value in (23,-23,.5,float('nan')):
            c=self.core();c.cfg['straight_steering_raw']=value
            with self.assertRaises(ValueError):validate_config(c.cfg)


if __name__=='__main__':unittest.main()
