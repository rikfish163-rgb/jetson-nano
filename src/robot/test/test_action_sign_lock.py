import os
import sys
import unittest
import yaml
ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.join(ROOT,'src'))
from robot.master.controller import Controller

class ActionSignLockTests(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(ROOT,'config','competition.yaml')) as f: cfg=yaml.safe_load(f)
        cfg.update(wait_green=False,lidar_enabled=False)
        self.c=Controller(cfg);self.addCleanup(self.c.close)

    def vote(self,label,t):
        for i in range(3):self.c.observe_sign(label,.99,t+i*.1,t+i*.1)

    def test_pending_direction_stays_locked_until_its_blue_action(self):
        self.vote('STRAIGHT',1)
        pending_at=self.c.pending_at
        for i,label in enumerate(('RIGHT','LEFT','UTURN','STRAIGHT','PARKING')):
            self.vote(label,2+i)
            self.assertEqual(self.c.pending,'STRAIGHT')
            self.assertEqual(self.c.pending_at,pending_at)
            self.assertIsNone(self.c.next_direction)

    def test_pending_parking_cannot_be_replaced_by_another_action(self):
        self.c.cfg['parking_enabled']=True
        self.vote('PARKING',1)
        pending_at=self.c.pending_at
        for i,label in enumerate(('LEFT','RIGHT','STRAIGHT','UTURN','PARKING')):
            self.vote(label,2+i)
            self.assertEqual(self.c.pending,'PARKING')
            self.assertEqual(self.c.pending_at,pending_at)
            self.assertIsNone(self.c.next_direction)

    def test_active_action_ignores_votes_and_requires_new_votes_after_exit(self):
        # Source frames from the sign that launched the action belong to the
        # old junction even if callbacks arrive while the maneuver runs.
        self.c.start_follow([(0,0,0,1,0),(1.25,0,0,1,0)],'STRAIGHT',10)
        self.c.set_pose((1.1,0,0),1)
        for i,label in enumerate(('RIGHT','STRAIGHT','UTURN','LEFT','PARKING')):
            self.vote(label,1+i)
            self.assertIsNone(self.c.pending)
            self.assertIsNone(self.c.next_direction)
            self.assertEqual(self.c.action,'STRAIGHT')
        self.c.resume_lane()
        self.assertIsNone(self.c.pending)
        self.c.observe_sign('RIGHT',.9,20,20)
        self.assertIsNone(self.c.pending)
        self.c.observe_sign('RIGHT',.9,20.2,20.2)
        self.assertEqual(self.c.pending,'RIGHT')

    def test_next_direction_accepts_a_fresh_route_after_action_grace(self):
        self.c.action,self.c.state,self.c.action_started = 'RIGHT','MANEUVER',10.
        for stamp in (11.6,11.7):
            self.c.observe_sign('RIGHT',.99,stamp,stamp)
        self.assertEqual(self.c.action,'RIGHT')
        self.assertEqual(self.c.next_direction,'RIGHT')
        self.assertEqual(self.c.sign_info['decision'],'stored_next_direction')

    def test_duplicate_source_frame_cannot_queue_twice(self):
        self.c.action,self.c.state,self.c.action_started = 'RIGHT','MANEUVER',10.
        self.c.observe_sign('RIGHT',.99,11.6,11.6)
        self.c.observe_sign('RIGHT',.99,11.6,11.7)
        self.assertIsNone(self.c.next_direction)
        self.assertEqual(self.c.sign_info['decision'],'stale_or_duplicate')

    def test_next_direction_is_single_entry_and_red_remains_available(self):
        self.c.action,self.c.state,self.c.action_started = 'RIGHT','MANEUVER',10.
        for stamp in (11.6,11.7,11.8):
            self.c.observe_sign('RIGHT',.99,stamp,stamp)
        self.assertEqual(self.c.next_direction,'RIGHT')
        self.assertEqual(self.c.sign_info['decision'],'next_direction_occupied')
        self.c.observe_sign('RED',.99,12.,12.)
        self.assertTrue(self.c.red)
        self.assertEqual(self.c.next_direction,'RIGHT')

    def test_fault_and_finished_actions_cannot_open_next_direction_queue(self):
        for i,state in enumerate(('FAULT','FINISHED')):
            self.c.action,self.c.state,self.c.action_started = 'RIGHT',state,10.
            # Each state gets a fresh source sequence.  Reusing the first
            # state's stamps would exercise stale-frame rejection instead of
            # the FAULT/FINISHED queue gate.
            base=11.6+i
            for stamp in (base,base+.1):
                self.c.observe_sign('RIGHT',.99,stamp,stamp)
            self.assertIsNone(self.c.next_direction)
            self.assertEqual(self.c.sign_info['decision'],'action_sign_ignored')
            self.c.next_direction,self.c.sign_label,self.c.sign_count = None,'',0

    def test_parking_disabled_is_reported_before_action_lock(self):
        self.c.cfg['parking_enabled'] = False
        self.c.action,self.c.state,self.c.action_started = 'RIGHT','MANEUVER',10.
        self.c.observe_sign('PARKING',.99,11.6,11.6)
        self.assertIsNone(self.c.next_direction)
        self.assertEqual(self.c.sign_info['decision'],'parking_disabled')

    def test_parking_can_queue_after_action_when_enabled(self):
        self.c.cfg['parking_enabled'] = True
        self.c.parking_route_ready = True
        self.c.action,self.c.state,self.c.action_started = 'RIGHT','MANEUVER',10.
        for stamp in (11.6,11.7,11.8):
            self.c.observe_sign('PARKING',.99,stamp,stamp)
        self.assertEqual(self.c.next_direction,'PARKING')
        self.assertEqual(self.c.sign_info['decision'],'stored_next_direction')

    def test_bypass_cannot_cache_straight_before_completion(self):
        self.c.state,self.c.action='TIMED_BYPASS','BYPASS'
        self.vote('STRAIGHT',1)
        self.assertIsNone(self.c.pending)
        self.assertIsNone(self.c.next_direction)
        self.c.resume_lane()
        self.c.observe_sign('STRAIGHT',.99,2,2)
        self.assertIsNone(self.c.pending)
        self.c.observe_sign('STRAIGHT',.99,2.1,2.1)
        self.assertEqual(self.c.pending,'STRAIGHT')

    def test_bypass_preserves_the_one_sign_cached_before_the_obstacle(self):
        self.vote('RIGHT',1)
        pending_at=self.c.pending_at
        self.c.state,self.c.action='TIMED_BYPASS','BYPASS'
        self.vote('STRAIGHT',2)
        self.c.resume_lane()
        self.assertEqual(self.c.pending,'RIGHT')
        self.assertEqual(self.c.pending_at,pending_at)
        self.vote('LEFT',3)
        self.assertEqual(self.c.pending,'RIGHT')

    def test_startup_straight_cannot_cache_the_next_route_action(self):
        self.c.state='STARTUP_STRAIGHT'
        for i,label in enumerate(('LEFT','RIGHT','STRAIGHT','UTURN')):
            self.vote(label,1+i)
            self.assertIsNone(self.c.pending)
            self.assertIsNone(self.c.next_direction)

    def test_completion_discards_queue_with_source_at_action_start(self):
        self.c.state,self.c.action,self.c.action_source='MANEUVER','RIGHT','sign'
        self.c.action_started=2.
        self.c.next_direction,self.c.next_direction_at='RIGHT',2.
        self.c.marker=((2,0),2)
        self.c.marker_candidate={'point':(2,0)}
        self.c.resume_lane()
        self.assertIsNone(self.c.pending)
        self.assertIsNone(self.c.next_direction)
        self.assertIsNone(self.c.marker)

    def test_completion_promotes_fresh_direction_queue_and_clears_marker(self):
        self.c.state,self.c.action,self.c.action_source='MANEUVER','RIGHT','sign'
        self.c.action_started=2.
        self.c.next_direction,self.c.next_direction_at='RIGHT',3.6
        self.c.marker=((2,0),2.1)
        self.c.marker_candidate={'point':(2,0)}
        self.c.resume_lane()
        self.assertEqual(self.c.pending,'RIGHT')
        self.assertEqual(self.c.pending_at,3.6)
        self.assertIsNone(self.c.next_direction)
        self.assertIsNone(self.c.marker)

    def test_completion_promotes_queued_parking_after_direction_action(self):
        self.c.state,self.c.action,self.c.action_source='MANEUVER','RIGHT','sign'
        self.c.action_started=2.
        self.c.next_direction,self.c.next_direction_at='PARKING',3.6
        self.c.marker=((2,0),2.1)
        self.c.marker_candidate={'point':(2,0)}
        self.c.resume_lane()
        self.assertEqual(self.c.pending,'PARKING')
        self.assertEqual(self.c.pending_at,3.6)
        self.assertIsNone(self.c.next_direction)
        self.assertIsNone(self.c.marker)

    def test_queued_right_waits_for_a_distinct_blue_line(self):
        # This is the real camera-to-dispatch handoff: the entry line already
        # consumed by the completed RIGHT must stay inert, while a later line
        # at least marker_rearm_distance away consumes the queued RIGHT.
        c=self.c
        c.cfg.update(blue_timed_enabled=False,intersection_wait_s=0.,
                     right_timed_enabled=False,right_turn_full_lock=False)
        c.state,c.action,c.action_source='MANEUVER','RIGHT','sign'
        c.action_started=2.
        c.next_direction,c.next_direction_at='RIGHT',3.6
        c.consumed_marker=(.3,0.)
        c.blue_consumed=True
        c.resume_lane()
        self.assertEqual(c.pending,'RIGHT')
        self.assertIsNone(c.next_direction)

        def frame(stamp,pose_x,marker_x):
            c.set_pose((pose_x,0.,0.),stamp)
            c.observe_ground(dict(source='front',part='markers',slots=[],
                markers=[dict(kind='junction',x=marker_x,y=0.,length=.6)],
                blue_lines=[dict(x=marker_x,y=0.,yaw=0.,length=.6)]),stamp)
            return c.tick(stamp)

        # The consumed line remains visible after handoff and cannot retrigger.
        for stamp in (2.3,2.4,2.5):
            frame(stamp,0.,.3)
        self.assertIsNone(c.action)
        self.assertIsNone(c.marker)
        # Move to the next line's approach; it is a distinct physical marker.
        for stamp in (3.0,3.1,3.2):
            frame(stamp,.8,.3)
        self.assertEqual(c.action,'RIGHT')
        self.assertIn(c.state,('INTERSECTION_WAIT','BLUE_APPROACH','MANEUVER'))

    def test_queued_parking_uses_the_next_distinct_blue_line(self):
        c=self.c
        c.cfg.update(blue_timed_enabled=False,intersection_wait_s=0.,
                     parking_enabled=True,parking_route_ready=True,
                     parking_mode='forward_plan',parking_slot='P4')
        c.state,c.action,c.action_source='MANEUVER','RIGHT','sign'
        c.action_started=2.
        c.next_direction,c.next_direction_at='PARKING',3.6
        c.consumed_marker=(.3,0.)
        c.blue_consumed=True
        c.resume_lane()
        self.assertEqual(c.pending,'PARKING')
        self.assertIsNone(c.next_direction)

        for stamp in (3.7,3.8,3.9):
            c.set_pose((.8,0.,0.),stamp)
            c.observe_ground(dict(source='front',part='markers',slots=[],
                markers=[dict(kind='junction',x=.3,y=0.,length=.6)],
                blue_lines=[dict(x=.3,y=0.,yaw=0.,length=.6)]),stamp)
            c.tick(stamp)
        self.assertEqual(c.action,'PARKING')
        self.assertEqual(c.state,'PARALLEL_PARKING')

    def test_confirmed_cache_does_not_expire_and_admit_another_sign(self):
        self.c.cfg['sign_ttl']=.25
        self.vote('STRAIGHT',1)
        self.c.observe_lane([(.3,0),(.6,0)],.99,2)
        self.c.observe_ground(dict(source='front',part='markers',markers=[],slots=[]),2)
        self.c.tick(2)
        self.assertEqual(self.c.pending,'STRAIGHT')
        self.vote('RIGHT',3)
        self.assertEqual(self.c.pending,'STRAIGHT')

    def test_production_blue_alignment_then_125_unlocks_next_sign(self):
        from robot.common.config import load_config
        cfg=load_config(os.path.join(ROOT,'config'))
        cfg.update(wait_green=False,lidar_enabled=False)
        self.assertTrue(cfg['blue_timed_enabled'])
        self.assertEqual(cfg['straight_distance'],1.25)
        c=Controller(cfg);self.addCleanup(c.close)
        cam=cfg['front_camera']

        def frame(t,row=None,angle=0.):
            import math
            lines=[] if row is None else [dict(
                x=(cam['origin_v']-row*(cam['bev_height']-1))/cam['pixels_per_m'],
                y=0.,yaw=math.radians(angle),length=.6)]
            c.observe_lane([(.3,0),(.6,0),(.9,0)],.99,t)
            c.observe_ground(dict(source='front',part='markers',slots=[],
                blue_lines=lines,markers=[dict(kind='junction',x=line['x'],y=0.,length=.6)
                                         for line in lines]),t)
            return c.tick(t)

        c.observe_sign('STRAIGHT',.99,1.,1.)
        c.observe_sign('STRAIGHT',.99,1.1,1.1)
        self.assertEqual(c.pending,'STRAIGHT')
        frame(1.1)
        self.assertIsNone(c.action)  # Sign alone cannot start the maneuver.
        for t,row,angle in ((1.2,.30,20.),(1.3,.35,20.),(1.4,.40,20.),
                            (1.5,.45,0.),(1.6,.50,0.),(1.7,.55,0.),
                            (1.8,.72,0.),(1.9,.74,0.),(2.,.76,0.)):
            frame(t,row,angle)
        self.assertEqual(c.action,'STRAIGHT')
        self.assertTrue(c.blue_approach['image_timing']['aligned'])
        c.set_pose((2.,.4,0.),2.)
        now=2.
        for i in range(1,51):
            now=2.+i*.1
            # Frames seen during BLUE_APPROACH/BLUE_STOP belong to the
            # current junction.  Exercise the next sign after MANEUVER has
            # started and its grace period has elapsed.
            if c.state == 'MANEUVER' and now > c.action_started+c.cfg['sign_timeout']:
                c.observe_sign('UTURN',.99,now,now)
            frame(now)
            if c.state=='MANEUVER':break
        self.assertEqual(c.state,'MANEUVER',c.reason)
        # Once the straight action has run beyond the old-sign grace period,
        # two fresh UTURN frames are a real next instruction. It stays queued
        # while this straight action owns the current blue-line maneuver.
        queue_start=max(now+.1,c.action_started+c.cfg['sign_timeout']+.1)
        for offset in (0.,.1):
            now=queue_start+offset
            c.observe_sign('UTURN',.99,now,now)
            frame(now)
        self.assertEqual(c.action,'STRAIGHT')
        self.assertEqual(c.next_direction,'UTURN')
        self.assertIsNone(c.pending)
        origin=c.straight_search['origin']
        self.assertEqual(origin,(2.,.4,0.))
        for progress in (1.249,1.249):
            now+=.1
            c.set_pose((origin[0]+progress,origin[1],origin[2]),now)
            c.observe_sign('UTURN',.99,now,now)
            frame(now)
            self.assertEqual(c.action,'STRAIGHT')
            self.assertEqual(c.state,'MANEUVER')
            self.assertIsNone(c.pending)
            self.assertEqual(c.next_direction,'UTURN')
        now+=.1
        c.set_pose((origin[0]+1.25,origin[1],origin[2]),now)
        frame(now)
        self.assertEqual(c.state,'LANE')
        self.assertIsNone(c.action)
        self.assertEqual(c.pending,'UTURN')
        self.assertIsNone(c.next_direction)
        for i in range(3):
            now+=.1
            frame(now,.45)
        self.assertEqual(c.action,'UTURN')
        self.assertIsNone(c.pending)

    def test_each_new_direction_can_trigger_a_distinct_blue_after_125(self):
        from robot.common.config import load_config
        for label in ('LEFT','RIGHT','STRAIGHT','UTURN'):
            cfg=load_config(os.path.join(ROOT,'config'))
            cfg.update(wait_green=False,lidar_enabled=False)
            c=Controller(cfg);self.addCleanup(c.close)
            c.start_follow([(0,0,0,1,0),(1.25,0,0,1,0)],'STRAIGHT',0.)
            c.consumed_marker=(.35,0.)
            c.blue_consumed=True
            c.set_pose((1.249,0.,0.),1.)

            def frame(t):
                c.observe_lane([(.3,0),(.6,0),(.9,0)],.99,t)
                c.observe_ground(dict(source='front',part='markers',slots=[],
                    markers=[dict(kind='junction',x=.8,y=0.,length=.6)],
                    blue_lines=[dict(x=.8,y=0.,yaw=0.,length=.6)]),t)
                return c.tick(t)

            for t in (1.,1.1):
                c.observe_sign(label,.99,t,t)
                frame(t)
                self.assertEqual(c.action,'STRAIGHT')
                self.assertIsNone(c.pending)
            c.set_pose((1.25,0.,0.),1.2)
            frame(1.2)
            self.assertEqual(c.state,'LANE')
            self.assertIsNone(c.pending)
            c.observe_sign(label,.99,1.3,1.3)
            self.assertIsNone(c.pending)
            c.observe_sign(label,.99,1.4,1.4)
            self.assertEqual(c.pending,label)
            for t in (1.5,1.6,1.7):frame(t)
            self.assertEqual(c.action,label)
            self.assertIsNone(c.pending)

    def test_parking_and_active_actions_ignore_route_signs(self):
        for i,state in enumerate(('PARKING','INTERSECTION_WAIT','UTURN')):
            self.c.state=state
            self.c.action='PARKING'
            self.vote('LEFT',1+i)
            self.assertIsNone(self.c.pending)
            self.assertIsNone(self.c.next_direction)

    def test_red_stop_and_green_release_remain_available(self):
        self.c.state,self.c.action='MANEUVER','STRAIGHT'
        self.vote('RED',1);self.assertTrue(self.c.red)
        self.vote('GREEN',2);self.assertFalse(self.c.red)
        self.assertEqual(self.c.action,'STRAIGHT')

if __name__=='__main__':unittest.main()
