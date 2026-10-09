"""Right lock until a fresh new blue junction, without actuators."""
import math
import os
import sys
import unittest
import yaml
ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.join(ROOT,'src'))
from robot.master.controller import Controller
from robot.common.planning import intersection_path


class RightBlueTests(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(ROOT,'config','competition.yaml')) as f: cfg=yaml.safe_load(f)
        cfg.update(wait_green=False,lidar_enabled=False,right_exit_on_blue=True,
                   right_turn_full_lock=True,right_reverse_entry_m=0,steering_command_scale_rad=.1)
        self.c=Controller(cfg)
        self.addCleanup(self.c.close)
        self.c.start_follow(intersection_path(self.c.pose,'RIGHT',cfg),'RIGHT',1)

    def front(self,t,markers=None):
        self.c.observe_ground(dict(source='front',part='markers',markers=markers or [],slots=[]),t)

    def test_missing_left_and_past_angle_keep_lock_within_deadline(self):
        for t,angle in ((2,0),(3,-125),(4,-170)):
            self.front(t)
            self.c.set_pose((0,0,math.radians(angle)),t)
            self.c.observe_left_boundary([],t)
            self.assertEqual(self.c.tick(t),(self.c.cfg['speed_raw']['action'],-.1))
            self.assertEqual(self.c.right_lock['phase'],'LOCK')
            self.assertEqual(self.c.action,'RIGHT')

    def test_fresh_new_blue_starts_straight_without_stop_or_distance_wait(self):
        self.front(2);self.c.tick(2)
        for t in (2.01,2.02): self.c.observe_sign('LEFT',.99,t,t)
        self.front(2.1,[dict(kind='junction',x=.8,y=.1)])
        self.assertEqual(self.c.tick(2.1),(26,0))
        self.assertEqual(self.c.action,'STRAIGHT')
        self.assertEqual(self.c.state,'MANEUVER')
        self.assertIsNone(self.c.right_lock)
        self.assertIsNone(self.c.marker)
        self.assertIsNone(self.c.next_direction)
        for t in (2.2,2.3,2.4):
            self.c.observe_lane([(.3,0),(.5,0),(.7,0)],.8,t)
            self.c.tick(t)
        self.assertEqual(self.c.state,'MANEUVER')
        # The current handoff is distance-based; no second blue or alignment
        # confirmation is required after the right-exit straight begins.
        x,y,yaw=self.c.pose
        self.c.set_pose((x+1.249*math.cos(yaw),y+1.249*math.sin(yaw),yaw),2.5)
        self.front(2.5)
        self.c.tick(2.5)
        self.assertEqual(self.c.state,'MANEUVER')
        self.c.set_pose((x+1.25*math.cos(yaw),y+1.25*math.sin(yaw),yaw),2.6)
        self.c.observe_lane([(.3,0),(.5,0),(.7,0)],.8,2.6)
        self.front(2.6)
        self.c.tick(2.6)
        for t in (2.7,2.8):
            self.c.observe_lane([(.3,0),(.5,0),(.7,0)],.8,t)
            self.c.tick(t)
        self.assertEqual(self.c.state,'LANE')
        self.assertIsNone(self.c.pending)

    def test_old_entry_line_stale_candidate_and_short_tick_do_not_exit(self):
        self.c.consumed_marker=(.3,0)
        self.front(2);self.c.tick(2)
        self.c.set_pose((-.45,0,0),2.1)
        self.front(2.1,[dict(kind='junction',x=.75,y=0)])
        self.assertLess(self.c.tick(2.1)[1],0)
        self.c.marker=((2,0),2.1)
        self.front(3,[dict(kind='tick',x=.3,y=0)])
        self.assertLess(self.c.tick(3)[1],0)
        self.assertEqual(self.c.action,'RIGHT')

    def test_reverse_entry_finishes_before_new_blue_can_exit(self):
        self.c.cfg['right_reverse_entry_m']=.45
        self.front(2,[dict(kind='junction',x=.8,y=0)])
        self.assertLess(self.c.tick(2)[0],0)
        self.c.set_pose((-.46,0,0),2.1)
        self.front(2.1)
        self.assertEqual(self.c.tick(2.1),(0,0))
        self.front(2.2)
        self.assertLess(self.c.tick(2.2)[1],0)
        self.front(2.3,[dict(kind='junction',x=.8,y=0)])
        self.assertEqual(self.c.tick(2.3),(26,0))

    def test_camera_loss_estop_and_red_still_stop(self):
        self.front(2);self.c.tick(2)
        self.assertEqual(self.c.tick(4),(0,0))
        self.assertEqual(self.c.reason,'right_blue_front_stale')
        self.front(4.1)
        self.c.estop=True
        self.assertEqual(self.c.tick(4.1),(0,0))
        self.c.estop=False;self.c.red=True
        self.assertEqual(self.c.tick(4.1),(0,0))


if __name__=='__main__': unittest.main()
