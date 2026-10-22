"""Early steering runs through the real mission/observation boundary, no ROS."""
import math
import os
import unittest

from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.common.geometry import bicycle
from robot.master.controller import Controller
from robot.motion.calibration import raw_angle, speed_gain


class BluePrealignTests(unittest.TestCase):
    def make(self,action='STRAIGHT'):
        cfg=load_config(os.path.join(os.path.dirname(os.path.dirname(__file__)),'config'))
        # Keep this synthetic regression independent of field speed/angle tuning.
        cfg['blue_stop_test'].update(align_speed_raw=20,align_tolerance_deg=10,
                                    heading_tolerance_deg=15)
        cfg.update(wait_green=False,lidar_enabled=False,sign_ttl=0,
                   blue_default_straight=False,steering_command_scale_rad=.03)
        c=Controller(cfg);self.addCleanup(c.close)
        c.pending=action;c.pending_at=1
        c.lane_command=lambda now:(30,.004)
        c.checked_command=lambda command,now,allow_bypass:command
        return c

    def frame(self,c,t,row,angle=0,kind='junction',y=0):
        cam=c.cfg['front_camera']
        x=(cam['origin_v']-row*(cam['bev_height']-1))/cam['pixels_per_m']
        c.observe_ground(dict(source='front',part='markers',slots=[],
            markers=[dict(kind=kind,x=x,y=y,length=.8)],
            blue_lines=[dict(x=x,y=y,yaw=math.radians(angle),length=.8)]),t)
        return c.tick(t)

    def test_sign_slows_lane_before_line_without_changing_lane_steering(self):
        for action in ('LEFT','RIGHT','STRAIGHT','UTURN'):
            c=self.make(action)
            self.assertEqual(c.tick(1),(20,.004))
            self.assertIsNone(c.action)
            self.assertIsNone(c.blue_prealign)

    def test_real_guard_keeps_slow_approach_diagnostic(self):
        c=self.make();del c._runtime.overrides['checked_command']
        self.assertEqual(c.tick(1),(20,.004))
        self.assertEqual(c.reason,'direction_sign_slow_approach')

    def test_delayed_image_uses_observation_pose_not_new_control_pose(self):
        c=self.make();self.frame(c,1,.3,-20)
        c.set_pose((.08,0,0),1.1)
        speed,steer=c.tick(1.1)
        self.assertEqual(speed,20)
        self.assertLess(steer,0)
        self.assertIsNotNone(c.blue_prealign)
        self.assertEqual(c.blue_prealign['sequence']['start'],1)

    def test_camera_delay_within_ground_budget_preserves_candidate_alignment(self):
        c=self.make()
        self.frame(c,1,.3,-20)
        for i in range(1,8):
            speed,steer=c.tick(1+i*.1)
            self.assertEqual(speed,20)
            self.assertLess(steer,0)
            self.assertIsNotNone(c.blue_prealign)
        self.assertEqual(c.marker_candidate['count'],1)
        self.assertIsNone(c.action)
        self.assertNotIn('trigger',c.blue_prealign['sequence'])

    def test_first_line_frame_steers_before_three_frame_dispatch(self):
        for action in ('LEFT','RIGHT','STRAIGHT','UTURN'):
            for angle in (-21.08,21.08):
                c=self.make(action)
                speed,steer=self.frame(c,1,.30,angle)
                self.assertEqual(speed,20)
                self.assertGreater(steer*angle,0)
                self.assertIsNone(c.action)
                self.assertIsNone(c.marker)
                self.assertFalse(c.blue_consumed)
                self.assertEqual(c.marker_candidate['count'],1)
                self.assertIn('prealign',c.reason)
                self.frame(c,1.1,.33,angle)
                self.assertIsNone(c.action)
                self.frame(c,1.2,.36,angle)
                self.assertEqual(c.action,action)
                self.assertTrue(c.blue_approach['prealigned_during_confirmation'])
                self.assertEqual(c.blue_approach['image_timing']['start'],1)
                self.assertIsNone(c.blue_prealign)

    def test_alignment_votes_survive_dispatch_no_early_timer(self):
        c=self.make()
        self.frame(c,1,.3,2)
        c.tick(1.05)
        self.assertEqual(c.blue_prealign['sequence']['align_frames'],1)
        self.frame(c,1.1,.34,2);self.frame(c,1.2,.38,2)
        seq=c.blue_approach['image_timing']
        self.assertTrue(seq['aligned'])
        self.assertEqual(seq['align_frames'],3)
        self.assertNotIn('trigger',seq)

    def test_brief_scheduler_delay_can_still_confirm_pending_uturn(self):
        c=self.make('UTURN')
        self.frame(c,1,.3,-20)
        self.frame(c,1.421277,.34,-20)
        self.assertNotEqual(c.state,'FAULT',c.reason)
        self.assertIsNone(c.action)
        self.frame(c,1.521277,.36,-20)
        self.assertEqual(c.action,'UTURN')
        self.assertEqual(c.state,'BLUE_APPROACH')
        self.assertNotIn('trigger',c.blue_approach['image_timing'])

    def test_long_scheduler_delay_still_stops_prealignment(self):
        c=self.make('UTURN')
        self.frame(c,1,.3,-20)
        # Keep the same valid observation; the control clock alone exceeds
        # the recovery limit, while the camera is still within its .6 s TTL.
        self.assertEqual(c.tick(1.500001),(0,0))
        self.assertEqual(c.state,'FAULT')
        self.assertEqual(c.reason,'blue_prealign_control_gap')

    def test_unconfirmed_line_cannot_start_timer_even_with_one_vote_settings(self):
        c=self.make()
        c.cfg['blue_stop_test'].update(align_confirm_frames=1,confirm_frames=1)
        self.frame(c,1,.7,0)
        self.assertIsNone(c.action)
        self.assertNotIn('trigger',c.blue_prealign['sequence'])

    def test_no_sign_parking_consumed_or_wrong_paint_do_not_prealign(self):
        for action in (None,'PARKING'):
            c=self.make(action);self.frame(c,1,.3,20)
            self.assertIsNone(c.blue_prealign)
            self.assertIsNone(c.action)
        for kind,y,angle in [('tick',0,20),('junction',1,20),('junction',0,80)]:
            c=self.make();self.frame(c,1,.3,angle,kind,y)
            self.assertIsNone(c.blue_prealign)
        c=self.make();c.blue_consumed=True
        self.frame(c,1,.3,20)
        self.assertIsNone(c.blue_prealign)

    def test_one_false_candidate_does_not_reserve_action_or_old_heading(self):
        c=self.make();self.frame(c,1,.3,-20)
        c.observe_ground(dict(source='front',part='markers',slots=[],markers=[],blue_lines=[]),1.1)
        self.assertEqual(c.tick(1.1),(20,.004))
        self.assertIsNone(c.blue_prealign)
        self.assertIsNone(c.action)
        self.assertFalse(c.blue_consumed)
        self.frame(c,1.2,.3,20)
        self.assertEqual(c.blue_prealign['sequence']['start'],1.2)

    def test_sensor_or_guard_stop_cannot_resume_stale_prealignment(self):
        c=self.make();self.frame(c,1,.3,-20)
        c.stop('scan_missing_or_stale')
        self.assertEqual(c.state,'FAULT')
        self.assertEqual(c.tick(1.1),(0,0))
        c=self.make();c.checked_command=lambda *args:(0,0)
        self.assertEqual(self.frame(c,1,.3,-20),(0,0))
        self.assertEqual(c.state,'FAULT')

    def test_closed_loop_early_alignment_reaches_timer_and_action(self):
        # Synthetic physical feedback, not a claim about unseen recorded frames.
        # The normal starts at the measured 21.08 degree error, but is visible
        # earlier than the logged 48.8 percent. Include asymmetric chassis turns.
        for sign in (-1,1):
            c=self.make()
            c.pose=(0,0,-math.radians(sign*21.08))
            # Infinite stripe X=d, represented by a finite segment centered on
            # the current forward-ray crossing. Its yaw is the stripe normal.
            cam=c.cfg['front_camera']
            initial_x=(cam['origin_v']-.30*(cam['bev_height']-1))/cam['pixels_per_m']
            d=initial_x*math.cos(c.pose[2])
            saw_timed=False
            for i in range(180):
                t=1+i*.1
                yaw=c.pose[2]
                x=(d-c.pose[0])/math.cos(yaw)
                row=(c.cfg['front_camera']['origin_v']-x*c.cfg['front_camera']['pixels_per_m'])/(c.cfg['front_camera']['bev_height']-1.)
                # Once timed, keep the camera alive without requiring a stripe.
                seq=(c.blue_approach or {}).get('image_timing',{})
                if 'trigger' in seq:
                    saw_timed=True
                    c.observe_ground(dict(source='front',part='markers',slots=[],markers=[],blue_lines=[]),t)
                    command=c.tick(t)
                else:
                    command=self.frame(c,t,row,-math.degrees(yaw))
                self.assertNotEqual(c.state,'FAULT',c.reason)
                if c.state=='BLUE_STOP':break
                raw=encode_command(command[0],command[1],c.cfg,0)
                speed=raw['speed_raw']
                c.set_pose(bicycle(c.pose,speed*speed_gain(c.cfg,speed)*.1,
                    raw_angle(c.cfg,speed,raw['steering_raw']),c.cfg['wheelbase']),t)
            self.assertTrue(saw_timed)
            self.assertEqual(c.state,'BLUE_STOP')
            self.assertLessEqual(abs(math.degrees(c.pose[2])),
                                 c.cfg['blue_stop_test']['heading_tolerance_deg'])
            calls=[]
            c.begin_blue_action=lambda now:calls.append(now) or (0,0)
            until=c.blue_approach['stop_until']
            c.front_marker_stamp=until+.01
            c.tick(until+.01)
            self.assertEqual(len(calls),1)


if __name__=='__main__':unittest.main()
