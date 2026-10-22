import os
import sys
import unittest
import yaml
ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.join(ROOT,'src'))
from robot.master.controller import Controller
from blue_test_helpers import enter_blue_action, clear_blue

class DirectionSingleFrameTests(unittest.TestCase):
    def core(self):
        with open(os.path.join(ROOT,'config','competition.yaml')) as f:
            cfg=yaml.safe_load(f)
        cfg.update(wait_green=False,lidar_enabled=False)
        c=Controller(cfg);self.addCleanup(c.close)
        return c

    def test_two_frames_at_threshold_confirm_all_directions(self):
        for label in ('LEFT','RIGHT','STRAIGHT','UTURN'):
            c=self.core();c.observe_sign(label,.8,1,1)
            self.assertIsNone(c.pending)
            c.observe_sign(label,.8,1.2,1.2)
            self.assertEqual(c.pending,label)
            self.assertEqual(c.sign_info['decision'],'stored_pending')

    def test_repeated_below_threshold_never_confirms(self):
        c=self.core()
        for t in (1,1.1,1.2):c.observe_sign('STRAIGHT',.799999,t,t)
        self.assertIsNone(c.pending)

    def test_two_signed_straights_then_uturn_wait_for_separate_markers(self):
        c=self.core()
        c.cfg.update(blue_default_straight=False,sign_ttl=0)
        for index,label in enumerate(('STRAIGHT','STRAIGHT','UTURN')):
            now=10.0+index*10.0
            # New line episode: the previous line has left the camera view.
            clear_blue(c,now-1.)
            # Re-observe immediately after handoff, before travelling the
            # old sign_rearm_distance. The next blue line gates execution.
            c.observe_sign(label,.99,now,now)
            c.observe_sign(label,.99,now+.05,now+.05)
            self.assertEqual(c.pending,label)
            self.assertEqual(c.sign_info['decision'],'stored_pending')
            c.dispatch(now)
            self.assertIsNone(c.action)
            c.set_pose((index*2.0,0,0),now+.1)
            enter_blue_action(c,label,now+.5)
            self.assertEqual(c.action,label)
            self.assertEqual(c.action_source,'sign')
            if label != 'UTURN':
                c.resume_lane()
                c.observe_ground(dict(source='front',part='markers',slots=[],
                    markers=[dict(kind='junction',x=.3,y=0)]),now+.6)
                self.assertIsNone(c.marker)  # consumed line cannot retrigger
            else:
                self.assertEqual(c.state,'UTURN')

    def test_stale_frame_rejected_and_current_action_locked(self):
        c=self.core();c.observe_sign('STRAIGHT',.99,1,10)
        self.assertIsNone(c.pending)
        c.observe_sign('STRAIGHT',.82,10,10)
        c.observe_sign('STRAIGHT',.82,10.1,10.1)
        c.observe_sign('LEFT',.99,10.2,10.2)
        self.assertEqual(c.pending,'STRAIGHT')
        self.assertEqual(c.sign_info['decision'],'action_sign_ignored')

    def test_one_wrong_right_frame_cannot_confirm_before_straight(self):
        c=self.core()
        c.last_completed_action='STRAIGHT'
        c.observe_sign('RIGHT',.811439514,38.282694,38.4)
        self.assertIsNone(c.pending)
        c.observe_sign('STRAIGHT',.9422,38.482940,38.6)
        self.assertIsNone(c.pending)
        c.observe_sign('STRAIGHT',.9477,38.686471,38.8)
        self.assertEqual(c.pending,'STRAIGHT')
        self.assertEqual(c.sign_info['decision'],'stored_pending')
        self.assertIsNone(c.action)

    def test_new_direction_observations_cannot_replace_a_locked_action(self):
        c=self.core();c.observe_sign('RIGHT',.99,1.,1.)
        c.observe_sign('RIGHT',.99,1.05,1.05)
        for t,label in ((1.1,'STRAIGHT'),(1.2,'LEFT'),(1.3,'STRAIGHT'),(1.4,'LEFT')):
            c.observe_sign(label,.99,t,t)
        self.assertEqual(c.pending,'RIGHT')
        c.action='RIGHT';c.state='BLUE_APPROACH';c.pending=None
        for t in (1.5,1.6,1.7):c.observe_sign('STRAIGHT',.99,t,t)
        self.assertIsNone(c.pending)
        self.assertEqual(c.action,'RIGHT')

    def test_fresh_or_duplicate_frames_cannot_replace_pending_before_completion(self):
        c=self.core()
        for t in (1.,1.2):c.observe_sign('RIGHT',.99,t,t)
        c.observe_sign('STRAIGHT',.99,1.4,1.4)
        c.observe_sign('STRAIGHT',.99,1.4,1.5)
        self.assertEqual(c.pending,'RIGHT')
        c.observe_sign('STRAIGHT',.99,1.6,1.6)
        self.assertEqual(c.pending,'RIGHT')
        self.assertEqual(c.sign_info['decision'],'action_sign_ignored')
        c.action,c.state,c.pending='RIGHT','MANEUVER',None
        c.resume_lane()
        c.observe_sign('STRAIGHT',.99,2.,2.)
        self.assertIsNone(c.pending)
        c.observe_sign('STRAIGHT',.99,2.2,2.2)
        self.assertEqual(c.pending,'STRAIGHT')

    def test_low_confidence_unknown_and_time_gap_break_confirmation(self):
        for label,conf in (('',0.),('RIGHT',.79)):
            c=self.core();c.observe_sign('RIGHT',.99,1.,1.)
            c.observe_sign(label,conf,1.2,1.2)
            c.observe_sign('RIGHT',.99,1.4,1.4)
            self.assertIsNone(c.pending)
            c.observe_sign('RIGHT',.99,1.6,1.6)
            self.assertEqual(c.pending,'RIGHT')
        c=self.core();c.observe_sign('RIGHT',.99,1.,1.)
        c.observe_sign('RIGHT',.99,4.,4.)
        self.assertIsNone(c.pending)

if __name__=='__main__':unittest.main()
