"""Minimum-distance and visual handoff regressions; no hardware."""
import unittest
import test_direction_single_frame as fixtures
from blue_test_helpers import enter_blue_action, observe_direction


class Straight165Tests(unittest.TestCase):
    def core(self):
        c=fixtures.DirectionSingleFrameTests.__dict__['core'](self)
        c.cfg['speed_raw']['lane']=26
        c.cfg['straight_speed_raw']=26
        return c

    def start(self,c):
        observe_direction(c,'STRAIGHT',.99,.7)
        enter_blue_action(c,'STRAIGHT',1.1)

    def test_old_command_cannot_lower_minimum(self):
        c=self.core();c.cfg['straight_distance']=1.2;self.start(c)
        for t,x in ((1.2,1.19),(1.3,1.21),(1.4,1.249)):
            c.set_pose((x,0,0),t);c.observe_lane([(.3,0),(.6,0)],.99,t)
            self.assertEqual(c.tick(t),(26,0.))
            self.assertEqual(c.state,'MANEUVER')
        c.set_pose((1.251,0,0),1.5)
        for t in (1.5,1.6,1.7):
            c.observe_lane([(.3,0),(.6,0)],.99,t);c.tick(t)
        self.assertEqual(c.state,'LANE')

    def test_distance_completion_waits_for_valid_exit_votes(self):
        for points in ([],[(.3,.25),(.6,.25)],[(.3,-.25),(.6,-.25)],
                       [(.3,0),(.6,.25)],[(.3,0),(.45,.2),(.6,0)]):
            c=self.core();self.start(c);c.set_pose((1.3,0,0),1.2)
            c.observe_lane(points,.99,1.2);c.tick(1.2)
            self.assertEqual(c.action,'STRAIGHT')
            self.assertEqual(c.straight_search['phase'],'WAIT_EXIT')
            self.assertEqual(c.reason,'straight_wait_exit_lane')

    def test_missing_camera_after_distance_keeps_action_stopped(self):
        c=self.core();self.start(c);c.set_pose((1.3,0,0),1.2)
        self.assertEqual(c.tick(3.),(0,0.))
        self.assertEqual(c.action,'STRAIGHT')
        self.assertEqual(c.reason,'straight_wait_exit_lane')

    def test_empty_lane_cannot_shorten_straight_or_extend_it_to_search_limit(self):
        c=self.core();self.start(c)
        for t,x in ((1.2,.1),(1.3,.8),(1.4,1.249)):
            c.set_pose((x,0,0),t)
            c.observe_lane([],.0,t)
            self.assertEqual(c.tick(t),(26,0.))
            self.assertEqual(c.action,'STRAIGHT')
        c.set_pose((1.25,0,0),1.5);c.observe_lane([],.0,1.5)
        self.assertEqual(c.tick(1.5),(0,0.))
        self.assertEqual(c.action,'STRAIGHT')
        self.assertEqual(c.reason,'straight_wait_exit_lane')

    def test_green_counts_from_release_without_blue(self):
        c=self.core();c.set_pose((3.,2.,0),.9)
        c.state='WAIT_GREEN';c.observe_sign('GREEN',.99,1.,1.)
        c.set_pose((4.249,2.,0),1.2)
        c.observe_ground(dict(source='front',part='markers',slots=[],markers=[]),1.2)
        c.observe_lane([(.3,0),(.6,0)],.99,1.2)
        self.assertEqual(c.tick(1.2),(26,0.))
        self.assertEqual(c.startup_origin,(3.,2.,0))
        c.set_pose((4.25,2.,0),1.3)
        c.observe_lane([(.3,.2),(.6,.4)],.99,1.3);c.tick(1.3)
        self.assertEqual(c.state,'LANE')

    def test_signed_distance_origin_is_bumper_stop_not_sign(self):
        c=self.core();observe_direction(c,'STRAIGHT',.99,.7)
        c.set_pose((5.,2.,0),.8)
        enter_blue_action(c,'STRAIGHT',1.1)
        self.assertEqual(c.straight_search['origin'],(5.,2.,0))
        c.set_pose((6.249,2.,0),1.2)
        c.observe_lane([(.3,0),(.6,0)],.99,1.2);c.tick(1.2)
        self.assertEqual(c.state,'MANEUVER')


if __name__=='__main__':unittest.main()
