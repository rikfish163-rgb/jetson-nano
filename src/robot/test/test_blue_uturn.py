"""Dispatch and integrated U-turn regression tests; no actuators."""
from __future__ import division
import copy
import math
import os
import sys
import unittest
import yaml
ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.join(ROOT,'src'))
from robot.master.controller import Controller
from robot.common.geometry import bicycle
from robot.common.geometry import local
from robot.common.geometry import world
from robot.common.geometry import wrap
from robot.lidar.scan import Scan
from robot.common.contracts import validate_config


class BlueUturnTests(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(ROOT,'config','competition.yaml')) as f: self.cfg=yaml.safe_load(f)
        self.cfg.update(wait_green=False,lidar_enabled=False,intersection_wait_s=.2,
                        left_turn_entry=.12,left_turn_radius=.65,left_turn_exit=.30,
                        action_lookahead=.30)
        self.cfg['speed_raw']['action']=18
        self.c=Controller(copy.deepcopy(self.cfg))
        self.addCleanup(self.c.close)

    def ground(self,t,points=(),source='front',yaw=0):
        self.c.observe_ground(dict(source=source,part='markers',slots=[],
            markers=[dict(kind='junction',x=x,y=y) for x,y in points],
            blue_lines=[dict(x=x,y=y,yaw=yaw,length=.6) for x,y in points]),t)

    def start(self):
        for t in (1,1.1,1.2): self.c.observe_sign('UTURN',.99,t,t)
        self.assertEqual(self.c.pending,'UTURN')
        self.ground(2,[(.3,0)])
        self.c.tick(2)
        self.assertEqual(self.c.uturn['phase'],'WAIT_START')
        self.c.tick(2.3)
        self.assertEqual(self.c.uturn['phase'],'LEFT_FIRST')

    def test_uturn_single_point95_caches(self):
        self.c.observe_sign('UTURN',.95,1,1)
        self.assertEqual(self.c.pending,'UTURN')

    def test_uturn_single_point60_caches(self):
        self.c.observe_sign('UTURN',.6,1,1)
        self.assertEqual(self.c.pending,'UTURN')

    def test_uturn_misses_expire_votes(self):
        self.c.observe_sign('UTURN',.599,1,1)
        self.c.observe_sign('',0,1.1,1.1)
        self.c.observe_sign('LEFT',.59,1.2,1.2)
        self.c.observe_sign('UTURN',.599,1.3,1.3)
        self.assertIsNone(self.c.pending)

    def test_uturn_stale_frames_do_not_confirm(self):
        self.c.observe_sign('UTURN',.99,1,3)
        self.assertIsNone(self.c.pending)
        self.c.observe_sign('UTURN',.6,3,3)
        self.assertEqual(self.c.pending,'UTURN')

    def test_low_confidence_does_not_cache_uturn(self):
        for t in (1,1.1,1.2,1.3): self.c.observe_sign('UTURN',.599,t,t)
        self.assertIsNone(self.c.pending)

    def test_confirmed_uturn_before_green_is_cached_without_starting(self):
        self.c.state='WAIT_GREEN'
        self.c.cfg['sign_ttl']=0
        for t in (1,1.1,1.2): self.c.observe_sign('UTURN',.99,t,t)
        self.assertEqual(self.c.pending,'UTURN')
        self.assertEqual(self.c.state,'WAIT_GREEN')
        self.ground(1.3,[(.3,0)])
        self.assertEqual(self.c.tick(1.3),(0,0))
        self.assertIsNone(self.c.action)
        for t in (2,2.1): self.c.observe_sign('GREEN',.99,t,t)
        self.ground(2.2,[(.3,0)])
        self.assertEqual(self.c.tick(2.2),(0,0))
        self.assertEqual(self.c.state,'UTURN')
        self.assertIsNone(self.c.pending)
        self.assertEqual(self.c.action,'UTURN')
        self.assertEqual(self.c.uturn['phase'],'WAIT_START')
        self.c.tick(2.6)
        self.assertEqual(self.c.uturn['phase'],'LEFT_FIRST')

    def test_wait_green_still_rejects_unconfirmed_uturn(self):
        self.c.state='WAIT_GREEN'
        for t in (1,1.1,1.2): self.c.observe_sign('UTURN',.599,t,t)
        self.assertIsNone(self.c.pending)
        self.assertEqual(self.c.tick(1.3),(0,0))

    def test_first_turn_reuses_left_profile_and_second_has_no_entry(self):
        self.start()
        path=self.c.follower.path
        self.assertAlmostEqual(path[-1][2],math.pi/2)
        self.assertTrue(any(abs(p[4]-math.atan(.26/.65))<1e-8 for p in path))
        self.c.pose=(.65,.5,math.radians(85))
        self.c.uturn_follow(3,second=True)
        self.assertAlmostEqual(wrap(self.c.exit_pose[2]-math.pi),0)
        self.assertEqual(self.c.follower.cfg['left_turn_entry'],0)
        self.assertAlmostEqual(self.c.follower.cfg['left_turn_angle_deg'],95)

    def test_blue_at_sixty_degrees_does_not_interrupt_first_left(self):
        self.start()
        self.c.pose=(.6,.5,math.radians(60))
        for t in (3,3.1,3.2):
            self.ground(t,[(.8,0)])
            self.c.tick(t)
        self.assertEqual(self.c.uturn['phase'],'LEFT_FIRST')

    def test_first_left_uses_same_exit_confirmation_as_normal_left(self):
        self.start()
        ordinary=Controller(copy.deepcopy(self.cfg))
        self.addCleanup(ordinary.close)
        ordinary.action='LEFT'
        ordinary.exit_pose=self.c.exit_pose
        self.c.pose=ordinary.pose=self.c.exit_pose
        for t in (3,3.1,3.2):
            points=[(.2,0),(.4,0),(.6,0)]
            self.c.observe_lane(points,.9,t)
            ordinary.observe_lane(points,.9,t)
            expected=ordinary.direction_exit_reached() and ordinary.exit_lane_confirmed(t)
            self.ground(t)
            self.c.tick(t)
            self.assertEqual(self.c.uturn['phase']=='ALIGN_FIRST',expected)

    def test_first_arc_without_blue_waits_for_target(self):
        self.start()
        self.c.pose=self.c.exit_pose
        self.ground(3)
        self.c.tick(3)
        self.assertEqual(self.c.uturn['phase'],'FIRST_REACQUIRE')
        self.assertGreater(self.c.tick(3.1)[0],0)

    def test_stop_overrides_and_timeout(self):
        self.start()
        self.c.red=True
        self.assertEqual(self.c.tick(3),(0,0))
        self.c.red=False
        self.c.estop=True
        self.assertEqual(self.c.tick(3),(0,0))
        self.c.estop=False
        self.c.tick(50)
        self.assertEqual(self.c.state,'FAULT')
        self.assertEqual(self.c.reason,'action_timeout')

    def test_invalid_reverse_configuration_rejected(self):
        for values in ({'uturn_brake_lead_m':float('nan')},
                       {'uturn_brake_lead_m':.2},
                       {'uturn_reverse_speed_raw':0},
                       {'uturn_reverse_max_distance':-1}):
            cfg=copy.deepcopy(self.cfg)
            cfg.update(values)
            with self.assertRaises(ValueError): validate_config(cfg)

    def test_complete_sequence_with_fixed_rear_blue_and_bicycle_motion(self):
        self.start()
        phases=set()
        rear_line=None
        front_line=world(self.c.exit_pose,(.8,0))
        front_yaw=self.c.exit_pose[2]
        reverse_distance=0
        for i in range(850):
            now=2.35+i*.05
            phase=self.c.uturn['phase'] if self.c.uturn else 'LANE'
            phases.add(phase)
            bx,by=local(self.c.pose,front_line)
            heading=wrap(front_yaw-self.c.pose[2])
            visible=.50<=bx<=1.50 and abs(by)<.55 and abs(heading)<math.radians(75)
            self.ground(now,[(bx,by)] if visible and phase in ('LEFT_FIRST','ALIGN_FIRST') else [],yaw=heading)
            if phase in ('LEFT_FIRST','FIRST_REACQUIRE','ALIGN_FIRST','LEFT_SECOND'):
                if phase=='LEFT_SECOND': self.c.observe_sign('RIGHT',.99,now,now)
                heading=wrap(self.c.exit_pose[2]-self.c.pose[2])
                if abs(heading)<.4:
                    a,b=local(self.c.pose,self.c.exit_pose)
                    slope=math.tan(heading)
                    self.c.observe_lane([(x,b+slope*(x-a)) for x in (.2,.4,.6,.8)],.99,now)
            if phase=='REVERSE_STRAIGHT':
                if rear_line is None: rear_line=world(self.c.uturn['reverse_origin'],(-.45,0))
                x,y=local(self.c.pose,rear_line)
                self.ground(now,[(x,y)] if -1.07<=x<=-.075 else [],'rear')
            raw,steer=self.c.tick(now)
            gain=.008 if raw>=0 else .007
            if raw<0:
                self.assertEqual(steer,0)
                reverse_distance-=raw*gain*.05
            self.c.set_pose(bicycle(self.c.pose,raw*gain*.05,steer,self.cfg['wheelbase']),now)
            if self.c.action is None: break
        self.assertEqual(self.c.state,'LANE',self.c.reason)
        self.assertEqual(self.c.last_completed_action,'UTURN')
        self.assertIsNone(self.c.pending)
        self.assertTrue({'LEFT_FIRST','ALIGN_FIRST','BRAKE_REVERSE','REVERSE_STRAIGHT',
                         'BRAKE_FORWARD','LEFT_SECOND'}.issubset(phases))
        self.assertTrue(.6<reverse_distance<.9)
        self.assertLess(abs(wrap(self.c.pose[2]-math.pi)),.3)


if __name__=='__main__': unittest.main()
