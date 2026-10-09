"""Offline event replay: no ROS publishers or hardware access."""
import math
import os
import unittest
from robot.common.config import load_config
from robot.master.controller import Controller


class BlueStopLineTests(unittest.TestCase):
    def car(self):
        cfg = load_config(os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config'))
        cfg.update(wait_green=False, lidar_enabled=False, sign_ttl=0,
                   blue_default_straight=False, intersection_wait_s=1.,
                   straight_speed_raw=24, lookahead=.55, parking_mode='forward_center')
        cfg['blue_align_duration_s']=.1  # test entry/stop independently of the timer
        cfg['speed_raw']['lane'] = 24
        c = Controller(cfg)
        self.addCleanup(c.close)
        return c

    def frame(self, c, t, x=None, y=0., yaw=0., path=None):
        c.observe_lane(path if path is not None else [(.5,0),(.7,0),(.9,0)], .9, t)
        c.observe_ground(dict(source='front',part='markers',slots=[],
            markers=[] if x is None else [dict(kind='junction',x=x,y=y,length=.6)],
            blue_lines=[] if x is None else [dict(x=x,y=y,yaw=yaw,length=.6)]),t)

    def arm(self, c, action='STRAIGHT', x=.9):
        c.pending, c.pending_at = action, .9
        for t in (1.,1.1,1.2,1.25,1.3):
            self.frame(c,t,x)
            c.tick(t)
        self.assertEqual(c.state,'BLUE_APPROACH')
        self.assertEqual(c.action,action)

    def stop(self, c, x=.9):
        c.set_pose((.36,0,0),2.)
        self.frame(c,2.)
        self.assertEqual(c.tick(2.),(0,0.))
        self.assertEqual(c.state,'BLUE_STOP')
        self.frame(c,2.9)
        self.assertEqual(c.tick(2.9),(0,0.))
        self.frame(c,3.01)
        c.tick(3.01)

    def test_different_detection_ranges_advance_same_distance_after_alignment(self):
        for x in (.65,1.10):
            c=self.car()
            self.arm(c,x=x)
            c.set_pose((.359,0,0),1.9)
            self.frame(c,1.9)
            self.assertGreater(c.tick(1.9)[0],0)
            self.stop(c,x)
            self.assertEqual(c.action,'STRAIGHT')
            self.assertAlmostEqual(c.straight_search['origin'][0],.36)

    def test_after_alignment_does_not_chase_crossing_white_path(self):
        c=self.car()
        self.arm(c,x=1.1)
        path=[(.5,-.3),(.7,-.3),(.9,-.3)]
        self.frame(c,1.4,1.1,path=path)
        self.assertEqual(c.tick(1.4),(24,0.))

    def test_side_blue_and_short_tick_cannot_dispatch(self):
        c=self.car();c.pending='STRAIGHT'
        for t in (1.,1.1,1.2):
            self.frame(c,t,.7,-.3,math.pi/2)
            c.tick(t)
        self.assertIsNone(c.action)

    def test_transverse_paint_wholly_in_adjacent_lane_is_ignored(self):
        c=self.car();c.pending='LEFT'
        for t in (1.,1.1,1.2):
            self.frame(c,t,.7,.55,0.)
            c.tick(t)
        self.assertIsNone(c.action)

    def test_one_bad_frame_does_not_start_an_action(self):
        c=self.car();c.pending='STRAIGHT'
        self.frame(c,1.,.9);c.tick(1.)
        self.frame(c,1.1);c.tick(1.1)
        self.assertIsNone(c.action)

    def test_all_direction_actions_wait_at_blue_before_starting(self):
        for action in ('LEFT','RIGHT','UTURN','STRAIGHT','PARKING'):
            c=self.car();self.arm(c,action)
            self.assertIsNone(c.follower)
            self.assertIsNone(c.parking_entry)
            self.stop(c)
            self.assertEqual(c.action,action)
            self.assertNotIn(c.state,('BLUE_APPROACH','BLUE_STOP'))

    def test_sign_alone_only_caches_including_parking(self):
        for action in ('LEFT','RIGHT','UTURN','STRAIGHT','PARKING'):
            c=self.car();c.pending=action
            self.frame(c,1.)
            c.tick(1.)
            self.assertIsNone(c.action)
            self.assertEqual(c.pending,action)

    def test_visual_updates_correct_target_but_next_line_cannot_replace_it(self):
        c=self.car();self.arm(c)
        # Stay in heading alignment so the current target may be refreshed.
        c.blue_approach['phase']='ALIGN';c.blue_approach['align_frames']=0
        self.frame(c,1.4,.95);c.tick(1.4)
        self.assertAlmostEqual(c.blue_approach['point'][0],.95)
        self.frame(c,1.5,1.5);c.tick(1.5)
        self.assertAlmostEqual(c.blue_approach['point'][0],.95)

    def test_blank_frame_does_not_rearm_consumed_blue(self):
        c=self.car();self.arm(c);self.stop(c)
        c.resume_lane();c.pending='RIGHT'
        self.frame(c,3.1);c.tick(3.1)
        for t in (3.2,3.3,3.4):
            self.frame(c,t,.8);c.tick(t)
        self.assertIsNone(c.action)
        for t in (3.5,3.7,3.9):
            self.frame(c,t);c.tick(t)
        for t in (4.,4.1,4.2):
            self.frame(c,t,.8);c.tick(t)
        self.assertEqual(c.state,'BLUE_APPROACH')
        self.assertEqual(c.action,'RIGHT')

    def test_straight_waits_at_distance_for_exit_confirmation(self):
        c=self.car();self.arm(c);self.stop(c)
        origin=c.pose[0]
        c.set_pose((origin+1.26,0,0),4.)
        self.frame(c,4.,path=[(.5,.3),(.7,.3)])
        c.tick(4.)
        self.assertEqual(c.state,'MANEUVER')
        self.assertEqual(c.action,'STRAIGHT')
        self.assertEqual(c.reason,'straight_wait_exit_lane')

    def test_empty_lane_after_straight_stops_without_discarding_action(self):
        c=self.car();self.arm(c);self.stop(c)
        c.set_pose((c.pose[0]+1.25,0,0),4.)
        self.frame(c,4.,path=[])
        self.assertEqual(c.tick(4.),(0,0.))
        self.assertEqual(c.action,'STRAIGHT')
        self.assertEqual(c.straight_search['phase'],'WAIT_EXIT')
        self.assertEqual(c.reason,'straight_wait_exit_lane')

    def test_sensor_loss_and_red_stop_without_discarding_event(self):
        c=self.car();self.arm(c)
        self.assertEqual(c.tick(3.),(0,0.))
        self.assertEqual(c.action,'STRAIGHT')
        self.frame(c,3.1,.9);c.red=True
        self.assertEqual(c.tick(3.1),(0,0.))
        c.red=False
        self.assertGreater(c.tick(3.1)[0],0)

    def test_oblique_line_first_requires_body_heading_alignment(self):
        c=self.car();c.pending='LEFT'
        c.cfg['blue_align_duration_s']=1.
        for t in (1.,1.1,1.2):
            self.frame(c,t,.9,.2,math.pi/6);c.tick(t)
        stop_x=.9+.2*math.tan(math.pi/6)-.33
        c.set_pose((stop_x,0,0),2.)
        self.frame(c,2.)
        command=c.tick(2.)
        self.assertGreater(command[0],0)
        self.assertGreater(command[1],0)
        self.assertEqual(c.blue_approach['phase'],'ALIGN')
        self.assertEqual(c.reason,'blue_align_latched_heading')

    def test_parking_anchor_survives_approach_and_one_second_wait(self):
        c=self.car();c.parking_sign=dict(point=(1.5,.1),stamp=1.)
        self.arm(c,'PARKING');self.stop(c)
        self.assertEqual(c.parking_entry.sign_anchor,(1.5,.1))

    def test_stop_telemetry_reports_advance_remaining_and_wait(self):
        from robot.master.telemetry import controller_status
        c=self.car();self.arm(c)
        c.set_pose((.36,0,0),2.);self.frame(c,2.);c.tick(2.)
        status=controller_status(c,c.cfg,2.2,False,True,(0,0.),0)
        self.assertAlmostEqual(status['wait_remaining'],.8)
        self.assertAlmostEqual(status['blue_approach']['remaining_m'],0.)

    def test_bypass_keeps_pending_sign_until_confirmed_lane_handoff(self):
        c=self.car();c.action='BYPASS';c.state='TIMED_BYPASS';c.pending='STRAIGHT'
        c.timed_bypass=dict(phase='REACQUIRE',elapsed_s=0.,last=None)
        for t in (1.,1.1,1.2):
            self.frame(c,t,path=[])
            self.assertEqual(c.tick(t),(0,0.))
            self.assertEqual(c.action,'BYPASS')
        for t in (1.3,1.4,1.5):
            self.frame(c,t);c.tick(t)
        self.assertEqual(c.state,'LANE')
        self.assertEqual(c.pending,'STRAIGHT')

    def test_full_seven_stage_uturn_hands_back_to_lane(self):
        c=self.car();self.arm(c,'UTURN');self.stop(c)
        phases=set()
        for i in range(1,800):
            now=3.01+i*.05
            self.frame(c,now)
            c.tick(now)
            if c.timed_uturn is not None:
                phases.add(c.timed_uturn.status()['phase'])
            if c.state=='LANE':break
        self.assertEqual(c.state,'LANE')
        self.assertTrue(set('SEGMENT_%d'%i for i in range(1,8)).issubset(phases))
        self.assertFalse(c.course_stop_pending)

    def test_parking_blue_to_associated_bay_finish(self):
        c=self.car();c.parking_sign=dict(point=(1.21,0.),stamp=1.)
        self.arm(c,'PARKING');self.stop(c)
        c.observe_ground(dict(source='front',part='parking_lines',lines=[
            [(.4,-.19),(.85,-.19)],[(.4,.19),(.85,.19)],
            [(.85,-.19),(.85,.19)]]),3.1)
        self.assertGreater(c.tick(3.1)[0],0)
        self.assertEqual(c.state,'PARKING')
        c.set_pose((.841,0.,0.),3.2)
        c.observe_ground(dict(source='front',part='parking_lines',lines=[]),3.2)
        self.assertEqual(c.tick(3.2),(0,0.))
        self.assertEqual(c.state,'FINISHED')


if __name__=='__main__':unittest.main()
