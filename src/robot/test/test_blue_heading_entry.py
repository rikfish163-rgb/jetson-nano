"""Blue normal controls body heading before a separate 0.36 m entry."""
import math
import os
import unittest
from robot.common.config import load_config
from robot.master.controller import Controller
from blue_test_helpers import observe_direction


class BlueHeadingEntryTests(unittest.TestCase):
    def setUp(self):
        cfg=load_config(os.path.join(os.path.dirname(os.path.dirname(__file__)),'config'))
        cfg.update(wait_green=False,lidar_enabled=False,blue_default_straight=False,
                   sign_ttl=0,intersection_wait_s=1.,straight_speed_raw=24)
        self.c=Controller(cfg);self.addCleanup(self.c.close)
        observe_direction(self.c,'STRAIGHT',.99,.9)

    def frame(self,t,heading,x=.9,path=None):
        self.c.observe_lane(path if path is not None else [(.5,0),(.8,0)],.9,t)
        self.c.observe_ground(dict(source='front',part='markers',slots=[],
            markers=[] if heading is None else [dict(kind='junction',x=x,y=0.,length=.6)],
            blue_lines=[] if heading is None else [dict(x=x,y=0.,yaw=heading,length=.6)]),t)
        return self.c.tick(t)

    def arm(self,heading):
        for t in (1.,1.1,1.2):command=self.frame(t,heading)
        self.assertEqual(self.c.blue_approach['phase'],'ALIGN')
        return command

    def test_aligned_car_keeps_fixed_one_second_window(self):
        self.arm(0.)
        for t in (1.3,1.4,1.5,2.19):
            self.assertGreater(self.frame(t,0.)[0],0)
            self.assertEqual(self.c.blue_approach['phase'],'ALIGN')
        self.assertGreater(self.frame(2.2,0.)[0],0)
        self.assertEqual(self.c.blue_approach['phase'],'ADVANCE')

    def test_one_second_forces_advance_even_with_bad_or_missing_blue(self):
        for missing in (False,True):
            if self.c.action is not None:
                self.c=Controller(self.c.cfg);self.addCleanup(self.c.close)
                observe_direction(self.c,'STRAIGHT',.99,.9)
            self.arm(.4)
            self.c.set_pose((.12,0,.05),2.19)
            self.assertGreater(self.frame(2.19,None if missing else .35)[1],0)
            self.assertEqual(self.c.blue_approach['phase'],'ALIGN')
            command=self.frame(2.2,None if missing else .35)
            s=self.c.blue_approach
            self.assertEqual(command[0],24)
            self.assertGreater(command[1],0)  # keep correcting the residual heading while advancing
            self.assertEqual(s['phase'],'ADVANCE')
            self.assertEqual(s['alignment_completion'],'timed')
            self.assertEqual(s['advance_origin'],(.12,0,.05))
            self.assertAlmostEqual(s['advance_travelled_m'],0.)
            self.assertGreater(self.frame(2.3,None)[1],0)

    def test_alignment_steers_towards_blue_normal_despite_opposite_lane(self):
        self.arm(.3)
        command=self.frame(1.3,.3,path=[(.5,-.3),(.8,-.3)])
        self.assertGreater(command[0],0)
        self.assertGreater(command[1],0)
        self.assertEqual(self.c.reason,'blue_align_heading')

    def test_right_alignment_has_negative_steering(self):
        self.assertLess(self.arm(-.3)[1],0)

    def test_reversed_contour_endpoints_keep_same_heading(self):
        before=self.arm(.3)
        self.assertAlmostEqual(self.frame(1.3,.3+math.pi)[1],before[1])

    def test_alignment_travel_is_not_part_of_entry_distance(self):
        self.arm(.3)
        self.c.set_pose((.20,0,.3),1.3)
        for t in (1.3,1.4,1.5,2.2):self.frame(t,0.,x=.70)
        s=self.c.blue_approach
        self.assertEqual(s['phase'],'ADVANCE')
        for actual,expected in zip(s['advance_origin'],(.20,0,.3)):
            self.assertAlmostEqual(actual,expected)
        self.assertAlmostEqual(s['advance_travelled_m'],0.)

    def test_repeated_ticks_cannot_confirm_heading_without_new_images(self):
        self.arm(0.)
        for t in (1.21,1.22,1.23):self.c.tick(t)
        self.assertEqual(self.c.blue_approach['phase'],'ALIGN')

    def test_missing_blue_keeps_confirmed_heading_without_claiming_completion(self):
        self.arm(.3)
        command=self.frame(1.8,None)
        self.assertGreater(command[0],0)
        self.assertGreater(command[1],0)
        self.assertEqual(self.c.blue_approach['phase'],'ALIGN')
        self.assertEqual(self.c.reason,'blue_align_latched_heading')
        self.assertEqual(self.c.blue_approach['align_frames'],0)

    def test_rejected_angle_jump_cannot_reverse_alignment(self):
        self.arm(.3)
        command=self.frame(1.3,-.5)
        self.assertGreater(command[0],0)
        self.assertGreater(command[1],0)
        self.assertAlmostEqual(self.c.blue_approach['yaw'],.3)

    def test_camera_stream_loss_still_stops_latched_alignment(self):
        self.arm(.3)
        self.assertEqual(self.c.tick(3.),(0,0.))
        self.assertEqual(self.c.reason,'blue_approach_front_stale')

    def test_recorded_missing_line_heading_finishes_entry_without_reobservation(self):
        from robot.common.geometry import bicycle, local
        from robot.common.contracts import command_to_model_steering
        self.arm(-.3)
        # Latest field stop: accepted normal -0.4617 rad, residual error
        # about -7.76 degrees; fresh images contain no transverse blue.
        self.c.blue_approach['yaw']=-.4617082014303122
        self.c.set_pose((.4,0,-.4617082014303122+math.radians(7.763255626)),1.3)
        cfg=self.c.cfg
        for i in range(200):
            t=1.4+i*.05
            speed,steer=self.frame(t,None,path=[])
            if self.c.state=='BLUE_STOP':break
            self.assertGreater(speed,0,self.c.reason)
            p=bicycle(self.c.pose,speed*cfg['raw_to_mps']['forward']*.05,
                      command_to_model_steering(steer,cfg),cfg['wheelbase'])
            self.c.set_pose(p,t+.05)
        self.assertEqual(self.c.state,'BLUE_STOP')
        s=self.c.blue_approach
        self.assertEqual(s['alignment_completion'],'timed')
        self.assertGreater(s['advance_origin'][0],.4)
        self.assertGreaterEqual(s['advance_travelled_m'],.36)
        self.assertLess(s['advance_travelled_m'],.375)
        t+=1.1
        self.frame(t,None,path=[])
        self.assertEqual(self.c.action,'STRAIGHT')
        self.assertEqual(self.c.reason,'straight_continue')
        self.assertAlmostEqual(self.c.straight_search['travelled_m'],0.)

    def test_marker_without_measured_orientation_cannot_start_alignment(self):
        for t in (1.,1.1,1.2):
            self.c.observe_ground(dict(source='front',part='markers',slots=[],
                markers=[dict(kind='junction',x=.9,y=0.,length=.6)]),t)
            self.c.tick(t)
        self.assertIsNone(self.c.action)

    def test_bicycle_feedback_uses_timed_alignment_then_separate_distance_both_sides(self):
        from robot.common.geometry import bicycle, local
        from robot.common.contracts import command_to_model_steering
        cfg=self.c.cfg
        for target in (-.3,.3):
            c=Controller(cfg);self.addCleanup(c.close)
            observe_direction(c,'STRAIGHT',.99,.9)
            for i in range(400):
                t=1.+i*.05
                x,y=local(c.pose,(1.2,0))
                lines=[dict(x=x,y=y,yaw=target-c.pose[2],length=.8)] if x>=.5 else []
                c.observe_ground(dict(source='front',part='markers',slots=[],blue_lines=lines,
                    markers=[dict(kind='junction',x=x,y=y,length=.8)] if lines else []),t)
                c.observe_lane([(.5,0),(.8,0)],.9,t)
                speed,steer=c.tick(t)
                if c.state=='BLUE_STOP':break
                pose=bicycle(c.pose,speed*cfg['raw_to_mps']['forward']*.05,
                             command_to_model_steering(steer,cfg),cfg['wheelbase'])
                c.set_pose(pose,t+.05)
            self.assertEqual(c.state,'BLUE_STOP',c.reason)
            s=c.blue_approach
            self.assertGreater(s['advance_origin'][0],.10)
            self.assertGreaterEqual(s['advance_travelled_m'],.36)
            self.assertLess(s['advance_travelled_m'],.375)
            self.assertAlmostEqual(s['alignment_elapsed_s'],1.)
            self.assertLess(abs(c.pose[2]-target),abs(s['advance_origin'][2]-target))
            self.assertIsNone(c.straight_search)


if __name__=='__main__':unittest.main()
