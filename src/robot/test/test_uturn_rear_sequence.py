"""UTURN: complete the first left, align, rear blue, front axle, second left."""
from __future__ import division
import math
import os
import sys
import unittest
import yaml
ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.join(ROOT,'src'))
from robot.master.controller import Controller
from robot.common.geometry import world
from robot.common.planning import intersection_path


class RearUturnTests(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(ROOT,'config','competition.yaml')) as f: cfg=yaml.safe_load(f)
        cfg.update(wait_green=False,lidar_enabled=False,intersection_wait_s=.2,
                   left_turn_entry=.12,left_turn_radius=.65,left_turn_exit=.30,
                   steering_command_scale_rad=.1,action_lookahead=.3)
        self.c=Controller(cfg)
        self.addCleanup(self.c.close)

    def ground(self,t,points=(),source='rear',yaw=0):
        self.c.observe_ground(dict(source=source,part='markers',slots=[],
            markers=[dict(kind='junction',x=x,y=y) for x,y in points],
            blue_lines=[dict(x=x,y=y,yaw=yaw,length=.6) for x,y in points]),t)

    def start(self):
        self.c.pending='UTURN'
        self.ground(1,[(.3,0)],'front')
        self.c.tick(1)
        self.c.tick(1.3)

    def align(self):
        self.start()
        # Isolated alignment fixtures use a 90-degree corner pose so their
        # blue targets remain distinct from the original stop line.
        self.c.pose=intersection_path(self.c.pose,'LEFT',self.c.cfg)[-1][:3]
        # Set up the phase before its first observation; each test supplies
        # either front or rear blue instead of implicitly losing front first.
        self.c.uturn_phase('ALIGN_FIRST',1.6)
        self.c.uturn.update(stable_since=None,aligned_frames=0,stable_s=0,
                            align_origin=self.c.pose,alignment_source='blue_normal',
                            rear_alignment_latched=False)
        self.assertEqual(self.c.uturn['phase'],'ALIGN_FIRST')

    def reverse(self):
        self.align()
        for t in (1.7,1.8,2,2.3,2.4):
            self.ground(t,[(.8,.2)],'front')
            self.c.tick(t)
        self.assertEqual(self.c.uturn['phase'],'BRAKE_REVERSE')
        self.ground(3.2)
        self.c.tick(3.2)
        self.assertEqual(self.c.uturn['phase'],'REVERSE_STRAIGHT')

    def test_blue_cannot_reverse_before_first_left_and_alignment(self):
        self.start()
        self.c.pose=(0,0,math.pi/4)
        for t in (2,2.1):
            self.ground(t,[(.3,0)],'front')
            self.c.tick(t)
        self.assertEqual(self.c.uturn['phase'],'LEFT_FIRST')

    def test_blue_heading_confirms_reverse_without_midpoint_or_distance_target(self):
        self.reverse()

    def test_rear_markers_are_not_forward_junction_triggers(self):
        self.ground(1,[(-.3,0)])
        self.assertEqual(len(self.c.rear_markers),1)
        self.assertIsNone(self.c.marker)

    def test_rear_camera_stale_stops_reverse_even_if_front_is_fresh(self):
        self.reverse()
        self.ground(5,[(1.2,0)],'front')
        self.assertEqual(self.c.tick(5),(0,0))
        self.assertEqual(self.c.reason,'uturn_rear_stale')

    def test_front_blue_does_not_end_reverse(self):
        self.reverse()
        for t in (3.3,3.4):
            self.ground(t)
            self.ground(t,[(1.21,0)],'front')
            self.assertLess(self.c.tick(t)[0],0)
        self.assertEqual(self.c.uturn['phase'],'REVERSE_STRAIGHT')

    def test_rear_lock_blind_tail_and_front_axle_trigger(self):
        self.reverse()
        origin=self.c.pose
        for t in (3.3,3.4):
            self.ground(t,[(-.1,0)])
            self.c.tick(t)
        self.assertIsNotNone(self.c.uturn['rear_line'])
        self.c.pose=tuple(world(origin,(-.25,0)))+(origin[2],)
        self.ground(3.5)
        self.assertLess(self.c.tick(3.5)[0],0)
        self.c.pose=tuple(world(origin,(-.35,0)))+(origin[2],)
        self.ground(3.6)
        self.assertEqual(self.c.tick(3.6),(0,0))
        self.assertEqual(self.c.uturn['phase'],'BRAKE_FORWARD')
        self.c.tick(4.1)
        self.assertEqual(self.c.uturn['phase'],'LEFT_SECOND')

    def test_rear_disappearing_far_from_blind_edge_stops(self):
        self.reverse()
        for t in (3.3,3.4):
            self.ground(t,[(-.6,0)])
            self.c.tick(t)
        self.ground(3.5)
        self.assertEqual(self.c.tick(3.5),(0,0))
        self.assertEqual(self.c.reason,'uturn_rear_line_lost')

    def test_reverse_without_line_is_distance_bounded(self):
        self.reverse()
        origin=self.c.uturn['reverse_origin']
        self.c.pose=tuple(world(origin,(-1.01,0)))+(origin[2],)
        self.ground(3.4)
        self.assertEqual(self.c.tick(3.4),(0,0))
        self.assertEqual(self.c.reason,'uturn_reverse_limit_wait_blue')

    def test_duplicate_rear_frame_does_not_lock_line(self):
        self.reverse()
        self.ground(3.3,[(-.1,0)])
        for t in (3.3,3.4,3.5): self.c.tick(t)
        self.assertIsNone(self.c.uturn.get('rear_line'))

    def test_duplicate_blue_does_not_confirm_alignment(self):
        self.align()
        self.ground(2,[(.33,0)],'front')
        self.c.tick(2)
        self.ground(2.1,[(.33,0)],'front')
        for t in (2.1,2.2,2.3,2.4): self.c.tick(t)
        self.assertEqual(self.c.uturn['phase'],'ALIGN_FIRST')
        self.assertEqual(self.c.uturn['aligned_frames'],1)

    def test_blue_direction_does_not_use_planned_exit_heading_gate(self):
        self.align()
        for t in (2.1,2.2):
            self.ground(t,[(1,.5)],'front',yaw=math.radians(40))
            self.c.observe_lane([(.2,0),(.4,0),(.6,0)],.9,t)
            command=self.c.tick(t)
        self.assertGreater(command[0],0)
        self.assertGreater(command[1],0)
        self.assertLessEqual(command[1],self.c.cfg['steering_command_scale_rad'])
        self.assertEqual(self.c.uturn['phase'],'ALIGN_FIRST')
        self.assertNotEqual(self.c.reason,'uturn_wrong_exit_line')

    def test_white_lane_alone_cannot_align_uturn(self):
        self.align()
        for t in (2,2.1,2.3,2.6,2.7):
            self.ground(t,[],'front')
            self.c.observe_lane([(.2,0),(.4,0),(.6,0)],.9,t)
            self.assertEqual(self.c.tick(t),(0,0))
        self.assertEqual(self.c.reason,'uturn_blue_target_unconfirmed')
        self.assertEqual(self.c.uturn['phase'],'ALIGN_FIRST')

    def test_original_start_blue_is_not_alignment_target(self):
        from robot.common.geometry import local
        self.align()
        p=local(self.c.pose,self.c.uturn['start_line'])
        for t in (2.1,2.2):
            self.ground(t,[p],'front')
            self.c.tick(t)
        self.assertIsNone(self.c.uturn.get('blue_target'))
        self.assertEqual(self.c.uturn['phase'],'ALIGN_FIRST')

    def test_predicted_blue_cannot_confirm_body_alignment(self):
        self.align()
        origin=self.c.pose
        for t in (2,2.1):
            self.ground(t,[(.55,0)],'front')
            self.c.tick(t)
        self.c.pose=tuple(world(origin,(.21,0)))+(origin[2],)
        self.ground(2.8,[],'front')
        self.assertEqual(self.c.tick(2.8),(0,0))
        self.assertTrue(self.c.uturn['blue_predicted'])
        self.assertEqual(self.c.uturn['phase'],'ALIGN_FIRST')
        self.assertEqual(self.c.uturn['aligned_frames'],0)

    def test_blue_disappearing_far_away_stops_for_blue_not_lane(self):
        self.align()
        for t in (2,2.1):
            self.ground(t,[(1,0)],'front')
            self.c.tick(t)
        self.ground(2.2,[],'front')
        self.assertEqual(self.c.tick(2.2),(0,0))
        self.assertEqual(self.c.reason,'uturn_blue_target_lost')

    def test_valid_blue_does_not_override_alignment_distance_limit(self):
        self.align()
        self.c.pose=tuple(world(self.c.pose,(.51,0)))+(self.c.pose[2],)
        for t in (2,2.1):
            self.ground(t,[(.8,0)],'front',yaw=-.3)
            command=self.c.tick(t)
        self.assertEqual(command,(0,0))
        self.assertEqual(self.c.reason,'uturn_alignment_distance_limit')

    def test_blue_steering_ignores_conflicting_white_lane(self):
        self.align()
        for t in (2,2.1):
            self.ground(t,[(.8,0)],'front',yaw=.2)
            self.c.observe_lane([(.2,-.2),(.4,-.2),(.6,-.2)],.9,t)
            command=self.c.tick(t)
        self.assertGreater(command[1],0)

    def test_camera_stale_cannot_continue_blue_alignment(self):
        self.align()
        for t in (2,2.1):
            self.ground(t,[(.5,0)],'front',yaw=.2)
            self.c.tick(t)
        self.assertEqual(self.c.tick(3),(0,0))
        self.assertEqual(self.c.reason,'uturn_blue_camera_stale')

    def test_missing_blue_breaks_candidate_votes(self):
        self.align()
        self.ground(2,[(.8,0)],'front')
        self.c.tick(2)
        self.ground(2.1,[],'front')
        self.c.tick(2.1)
        self.ground(2.2,[(.8,0)],'front')
        self.c.tick(2.2)
        self.assertIsNone(self.c.uturn.get('blue_target'))

    def check_blue_alignment_motion(self, initial_error):
        from robot.common.geometry import bicycle
        from robot.common.geometry import local
        from robot.common.geometry import wrap
        self.align()
        point=world(self.c.pose,(1.1,0))
        yaw=wrap(self.c.pose[2]+initial_error)
        for i in range(240):
            t=2+i*.05
            heading=wrap(yaw-self.c.pose[2])
            self.ground(t,[local(self.c.pose,point)],'front',yaw=heading)
            speed,command=self.c.tick(t)
            self.assertLessEqual(abs(command),self.c.cfg['steering_command_scale_rad'])
            if self.c.uturn['phase']=='BRAKE_REVERSE': break
            physical=command/self.c.cfg['steering_command_scale_rad']*self.c.cfg['max_steer']
            self.c.set_pose(bicycle(self.c.pose,speed*.008*.05,physical,self.c.cfg['wheelbase']),t)
        self.assertEqual(self.c.uturn['phase'],'BRAKE_REVERSE',self.c.reason)
        self.assertLessEqual(abs(wrap(yaw-self.c.pose[2])),math.radians(8))
        self.assertGreaterEqual(self.c.uturn['stable_s'],.5)

    def test_blue_normal_left_error_converges_without_white_lane(self):
        self.check_blue_alignment_motion(.35)

    def test_blue_normal_right_error_converges_without_white_lane(self):
        self.check_blue_alignment_motion(-.35)

    def test_reversing_heading_drift_stops(self):
        self.reverse()
        self.c.pose=self.c.pose[:2]+(self.c.pose[2]+.2,)
        self.ground(3.4)
        self.assertEqual(self.c.tick(3.4),(0,0))
        self.assertEqual(self.c.reason,'uturn_reverse_heading_drift')

    def test_axle_overshoot_does_not_start_second_left(self):
        self.reverse()
        origin=self.c.pose
        for t in (3.3,3.4):
            self.ground(t,[(-.1,0)])
            self.c.tick(t)
        self.c.pose=tuple(world(origin,(-.45,0)))+(origin[2],)
        self.ground(3.5)
        self.assertEqual(self.c.tick(3.5),(0,0))
        self.assertEqual(self.c.reason,'uturn_front_axle_overshoot')

    def test_lidar_obstacle_still_blocks_reverse(self):
        from robot.lidar.scan import Scan
        self.reverse()
        self.c.cfg['lidar_enabled']=True
        self.c.scan=Scan([float('inf')]*360,-math.pi,math.pi/180,.05,6,
                         self.c.pose,self.c.cfg['lidar'],3.4)
        self.c.scan.obstacles=[world(self.c.pose,(-.16,0))]
        self.ground(3.4)
        self.assertEqual(self.c.tick(3.4),(0,0))


if __name__=='__main__': unittest.main()
