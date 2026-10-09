"""Rear blue fallback, steering sign, and locked-line reverse handoff."""
import math
import unittest
from test_uturn_rear_sequence import RearUturnTests
from robot.common.geometry import world
from robot.common.geometry import local
from robot.common.geometry import wrap
from robot.common.geometry import bicycle


class RearAlignmentTests(unittest.TestCase):
    def setUp(self):
        f=RearUturnTests('test_white_lane_alone_cannot_align_uturn')
        f.setUp()
        self.addCleanup(f.doCleanups)
        f.align()
        self.c,self.ground=f.c,f.ground

    def rear(self,t,yaw=0,points=None):
        self.ground(t,[(-.4,0)] if points is None else points,'rear',yaw=yaw)
        return self.c.tick(t)

    def test_rear_blue_replaces_missing_front_and_conflicting_right(self):
        for t in (2,2.1):
            self.c.observe_lane([(.5,-.3),(.7,-.4),(.9,-.5)],.9,t)
            cmd=self.rear(t,.3)
        self.assertGreater(cmd[0],0)
        self.assertGreater(cmd[1],0)
        self.assertEqual(self.c.uturn['alignment_source'],'rear_blue_normal')

    def test_front_is_used_first_even_when_rear_is_available(self):
        for t in (2,2.1):
            self.ground(t,[(.8,0)],'front',yaw=-.3)
            cmd=self.rear(t,.3)
            self.assertFalse(self.c.uturn['rear_alignment_latched'])
        self.assertLess(cmd[1],0)
        self.assertEqual(self.c.uturn['alignment_source'],'blue_normal')

    def test_front_loss_latches_rear_before_rear_confirmation(self):
        self.ground(2,[],'front')
        self.c.tick(2)
        self.assertTrue(self.c.uturn['rear_alignment_latched'])
        for t in (2.1,2.2):
            self.ground(t,[(.8,0)],'front',yaw=-.3)
            self.assertEqual(self.c.tick(t),(0,0))
        self.assertEqual(self.c.uturn['alignment_source'],'rear_blue_normal')

    def test_stale_duplicate_and_empty_rear_do_not_confirm(self):
        self.rear(2)
        self.rear(2.1)
        for t in (2.2,2.4): self.c.tick(t)
        self.assertEqual(self.c.uturn['aligned_frames'],1)
        self.assertEqual(self.c.tick(2.7),(0,0))
        self.assertEqual(self.c.uturn['aligned_frames'],0)
        self.rear(2.8,points=[])
        self.assertEqual(self.c.uturn['phase'],'ALIGN_FIRST')

    def test_front_return_cannot_override_rear_or_reset_stability(self):
        for t in (2,2.1,2.3): self.rear(t)
        for t in (2.4,2.5):
            self.ground(t,[(.8,0)],'front',yaw=-.3)
            cmd=self.rear(t)
        self.assertEqual(cmd,(0,0))
        self.assertEqual(self.c.uturn['alignment_source'],'rear_blue_normal')
        self.assertGreaterEqual(self.c.uturn['aligned_frames'],4)

    def test_missing_rear_does_not_switch_back_to_front(self):
        self.rear(2,.3)
        self.rear(2.1,.3)
        for t in (2.2,2.3):
            self.ground(t,[(.8,0)],'front',yaw=-.3)
            self.assertEqual(self.rear(t,points=[]),(0,0))
        self.assertEqual(self.c.uturn['alignment_source'],'rear_blue_normal')

    def test_predicted_front_does_not_override_measured_rear(self):
        for t in (2,2.1):
            self.ground(t,[(.5,0)],'front',yaw=-.3)
            self.c.tick(t)
        for t in (2.2,2.3):
            self.ground(t,[],'front')
            cmd=self.rear(t,.3)
        self.assertGreater(cmd[1],0)
        self.assertEqual(self.c.uturn['alignment_source'],'rear_blue_normal')

    def test_locked_rear_cannot_jump_to_another_line(self):
        self.rear(2,.3)
        self.rear(2.1,.3)
        target=self.c.uturn['rear_blue_target']
        self.assertEqual(self.rear(2.2,-.3,points=[(-.9,0)]),(0,0))
        self.assertEqual(self.c.uturn['rear_blue_target'],target)

    def test_pre_alignment_rear_frame_cannot_steer(self):
        t=self.c.uturn['phase_started']-.01
        self.ground(t,[(-.4,0)],'rear',yaw=.3)
        self.assertEqual(self.c.tick(t+.1),(0,0))
        self.assertEqual(self.c.uturn['rear_blue_reason'],'rear_blue_stale')

    def test_short_or_off_axis_rear_line_cannot_steer(self):
        for t in (2,2.1):
            self.ground(t,[(-.4,.5)],'rear',yaw=.3)
            self.assertEqual(self.c.tick(t),(0,0))
        for t in (2.2,2.3):
            self.c.observe_ground(dict(source='rear',part='markers',markers=[],slots=[],
                blue_lines=[dict(x=-.4,y=0,yaw=.3,length=.1)]),t)
            self.assertEqual(self.c.tick(t),(0,0))

    def test_aligned_rear_target_survives_reverse_and_starts_full_left(self):
        for t in (2,2.1,2.3,2.6): self.rear(t,points=[(-.1,0)])
        self.assertEqual(self.c.uturn['phase'],'BRAKE_REVERSE')
        target=self.c.uturn['rear_blue_target']['point']
        self.rear(3.1,points=[(-.1,0),(-.05,.2)])
        self.assertEqual(self.c.uturn['phase'],'REVERSE_STRAIGHT')
        self.assertEqual(self.c.uturn['rear_line'],target)
        origin=self.c.pose
        self.c.pose=tuple(world(origin,(-.35,0)))+(origin[2],)
        self.assertEqual(self.rear(3.2,points=[]),(0,0))
        self.assertEqual(self.c.uturn['phase'],'BRAKE_FORWARD')
        self.c.tick(3.7)
        self.assertEqual(self.c.uturn['phase'],'LEFT_SECOND')

    def check_motion(self,error):
        point=world(self.c.pose,(-.4,0))
        yaw=wrap(self.c.pose[2]+error)
        for i in range(240):
            t=2+i*.05
            cmd=self.rear(t,wrap(yaw-self.c.pose[2]),[local(self.c.pose,point)])
            if self.c.uturn['phase']=='BRAKE_REVERSE': break
            physical=cmd[1]/self.c.cfg['steering_command_scale_rad']*self.c.cfg['max_steer']
            self.c.set_pose(bicycle(self.c.pose,cmd[0]*.008*.05,physical,self.c.cfg['wheelbase']),t)
        self.assertEqual(self.c.uturn['phase'],'BRAKE_REVERSE',self.c.reason)
        self.assertLessEqual(abs(wrap(yaw-self.c.pose[2])),math.radians(8))

    def test_positive_rear_normal_converges(self): self.check_motion(.35)
    def test_negative_rear_normal_converges(self): self.check_motion(-.35)


if __name__=='__main__': unittest.main()
