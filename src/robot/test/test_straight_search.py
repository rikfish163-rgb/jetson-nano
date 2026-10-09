"""Blue-stop and straight visual acquisition, without ROS or actuators."""
import math
import os
import sys
import unittest
import yaml
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT,'src'))
from robot.master.controller import Controller
from test_straight_alignment import finish_blue_exit


class StraightSearchTests(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(ROOT,'config','competition.yaml')) as f:
            cfg = yaml.safe_load(f)
        cfg.update(wait_green=False,lidar_enabled=False,blue_default_straight=True)
        self.c = Controller(cfg)
        self.addCleanup(self.c.close)

    def trigger(self, action=None):
        self.c.pending = action
        self.c.observe_ground(dict(source='front',markers=[dict(kind='junction',x=.3,y=0)],slots=[]),1)
        self.assertEqual(self.c.tick(1),(0,0))

    def lane(self,t,points=None,confidence=.8):
        self.c.observe_ground(dict(source='front',part='markers',markers=[],slots=[]),t)
        self.c.observe_lane(points if points is not None else [(.3,0),(.5,0),(.7,0)],confidence,t)

    def test_unsigned_blue_stops_one_second_then_searches_at_zero_steer(self):
        self.c.gap_steer = .3
        self.trigger()
        self.assertEqual((self.c.state,self.c.action),('INTERSECTION_WAIT','STRAIGHT'))
        self.assertEqual(self.c.tick(1.99),(0,0))
        self.lane(2,[])
        self.assertEqual(self.c.tick(2),(26,0))
        self.assertEqual(self.c.straight_search['phase'],'WAIT_BLUE')

    def test_unsigned_blue_defaults_to_straight(self):
        self.c.follow_left_boundary = True
        self.trigger(None)
        self.assertEqual(self.c.action,'STRAIGHT')
        self.assertEqual(self.c.action_source,'blue_default')
        self.assertEqual(self.c.tick(1.99),(0,0))
        self.lane(2,[])
        self.assertEqual(self.c.tick(2),(26,0))
        self.assertFalse(self.c.follow_left_boundary)

    def test_cached_right_wins_over_unsigned_blue_after_left_reference(self):
        self.c.follow_left_boundary=True
        self.c.cfg['intersection_wait_s']=1
        self.trigger('RIGHT')
        self.assertEqual(self.c.action,'RIGHT')
        self.assertEqual(self.c.action_source,'sign')
        self.assertEqual(self.c.tick(1.9),(0,0))
        self.assertLess(self.c.tick(2)[0],0)
        self.assertFalse(self.c.follow_left_boundary)

    def test_explicitly_disabled_fallback_requires_sign(self):
        self.c.cfg['blue_default_straight'] = False
        self.trigger(None)
        self.assertIsNone(self.c.action)

    def test_observed_straight_is_cached_until_blue(self):
        for t in (.1,.2,.3):
            self.c.observe_sign('STRAIGHT',.99,t,t)
        self.assertEqual(self.c.pending,'STRAIGHT')
        self.c.tick(.4)
        self.assertIsNone(self.c.action)
        self.c.observe_ground(dict(source='front',markers=[dict(kind='junction',x=.3,y=0)],slots=[]),1)
        self.assertEqual(self.c.tick(1),(0,0))
        self.assertEqual(self.c.action,'STRAIGHT')
        self.assertEqual(self.c.action_source,'sign')

    def test_signed_straight_uses_bounded_visual_search(self):
        self.trigger('STRAIGHT')
        self.c.observe_ground(dict(source='front',part='markers',markers=[],slots=[]),2)
        self.lane(2,[])
        self.assertEqual(self.c.tick(2),(26,0))
        self.assertEqual(self.c.straight_search['phase'],'WAIT_BLUE')
        self.assertEqual(self.c.action,'STRAIGHT')

    def test_new_straight_path_requires_next_blue_to_hand_over(self):
        self.trigger()
        for t in (2,2.1,2.2,2.3):
            self.lane(t)
            command = self.c.tick(t)
        self.assertEqual(self.c.state,'MANEUVER')
        self.c.set_pose((.7,0,0),2.4)
        self.c.observe_ground(dict(source='front',part='markers',slots=[],
            markers=[dict(kind='junction',x=.3,y=0)]),2.4)
        self.c.observe_lane([(.3,0),(.5,0),(.7,0)],.8,2.4)
        command = finish_blue_exit(self.c,2.4)
        self.assertEqual(self.c.state,'LANE')
        self.assertIsNone(self.c.action)
        self.assertIsNone(self.c.straight_search)
        self.assertGreater(command[0],0)

    def test_duplicate_frames_do_not_confirm(self):
        self.trigger()
        self.lane(2)
        for t in (2,2.1,2.2): self.c.tick(t)
        self.assertEqual(self.c.state,'MANEUVER')

    def test_missing_camera_stops_instead_of_blind_search(self):
        self.trigger()
        self.assertEqual(self.c.tick(2.3),(0,0))
        self.assertEqual(self.c.reason,'straight_front_stale')

    def test_crosswise_path_cannot_take_over(self):
        self.trigger()
        for t in (2,2.1,2.2,2.3):
            self.lane(t,[(.4,-.3),(.41,0),(.42,.3)])
            self.assertEqual(self.c.tick(t),(26,0))
        self.assertEqual(self.c.straight_search['candidate_reason'],'no_next_blue')

    def test_offset_lane_cannot_take_over(self):
        self.trigger()
        for t in (2,2.1,2.2,2.3):
            self.lane(t,[(.3,.5),(.5,.5),(.7,.5)])
            self.assertEqual(self.c.tick(t),(26,0))

    def test_distance_limit_latches_stop(self):
        self.trigger()
        self.lane(2,[]);self.c.tick(2)
        self.c.set_pose((2.21,0,0),3)
        self.lane(3,[])
        self.assertEqual(self.c.tick(3),(0,0))
        self.assertEqual(self.c.reason,'straight_search_distance_limit')
        self.lane(3.1)
        self.assertEqual(self.c.tick(3.1),(0,0))

    def test_time_limit_latches_stop(self):
        self.trigger()
        self.lane(2,[]);self.c.tick(2)
        self.lane(22,[])
        self.assertEqual(self.c.tick(22),(0,0))
        self.assertEqual(self.c.reason,'straight_search_timeout')

    def test_left_instruction_keeps_priority(self):
        self.c.cfg['intersection_wait_s']=1
        self.trigger('LEFT')
        self.assertEqual(self.c.action,'LEFT')
        self.lane(2)
        self.c.tick(2)
        self.assertIsNone(self.c.straight_search)

    def test_red_and_estop_override_search(self):
        self.trigger();self.lane(2,[]);self.c.tick(2)
        self.c.red=True
        self.assertEqual(self.c.tick(2.1),(0,0))
        self.c.red=False;self.c.estop=True
        self.assertEqual(self.c.tick(2.2),(0,0))

    def test_next_direction_is_ignored_during_search(self):
        self.trigger();self.lane(2,[]);self.c.tick(2)
        for t in (2.1,2.2): self.c.observe_sign('RIGHT',.9,t,t)
        self.assertIsNone(self.c.next_direction)


if __name__ == '__main__': unittest.main()
