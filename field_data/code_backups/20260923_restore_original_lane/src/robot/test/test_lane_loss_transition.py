"""Only loss/reacquisition limits steering; ordinary following stays original."""
import os
import unittest
from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.master.controller import Controller


class LaneLossTransitionTests(unittest.TestCase):
    def setUp(self):
        root=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cfg=load_config(os.path.join(root,'config'))
        cfg.update(lidar_enabled=False,wait_green=False,blue_default_straight=False,
                   lookahead=.55,steering_command_scale_rad=.1,
                   lane_curve_speed_raw=24)
        cfg['speed_raw']['lane']=24
        self.c=Controller(cfg)
        self.addCleanup(self.c.close)

    def observe(self,t,y,confidence=.9):
        self.c.observe_lane([] if y is None else [(.5,y),(.7,y),(.9,y)],confidence,t)
        speed,steer=self.c.tick(t)
        return speed,encode_command(speed,steer,self.c.cfg,0)['steering_raw']

    def test_normal_following_has_no_gain_speed_or_confirmation_limit(self):
        self.assertEqual(self.observe(1.,-.3,.4),(24,-22))
        self.assertEqual(self.observe(1.05,.3,.4),(24,22))

    def test_recorded_right_path_is_not_flipped_by_wrong_left_bend_label(self):
        # 2026-09-23 19:08:08: every target was right of the car, but the
        # bend label changed to LEFT and the guard emitted full left lock.
        points=[(.507488,-.340826),(.618713,-.389958),(.729768,-.436040),
                (.840654,-.479055),(.951370,-.519003),(1.061915,-.555875)]
        self.c.observe_lane(points,.9,1.,{},'LEFT',3)
        speed,steer=self.c.tick(1.)
        self.assertEqual(encode_command(speed,steer,self.c.cfg,0)['steering_raw'],-22)

    def test_confirmed_bend_uses_observed_far_target_instead_of_sign_flip(self):
        points=[(.48,.02),(.64,-.03),(.8,-.10),(.96,-.16)]
        self.c.observe_lane(points,.9,1.,{},'RIGHT',3)
        speed,steer=self.c.tick(1.)
        import math
        expected=math.atan2(2*self.c.cfg['wheelbase']*points[-1][1],
                            sum(v*v for v in points[-1]))
        self.assertAlmostEqual(steer,expected)

    def test_confirmed_bend_corrects_opposite_steering_symmetrically(self):
        for stamp, direction, y, expected in ((1.,'RIGHT',.04,-1), (2.,'LEFT',-.04,1)):
            self.c.observe_lane([(.5,y),(.7,-y),(.9,-5*y)],.9,stamp,{},direction,3)
            speed, steer = self.c.tick(stamp)
            self.assertEqual(speed,24)
            self.assertGreater(steer*expected,0)
            self.assertEqual(self.c.lane_source,'center_bend_'+direction.lower())

    def test_unknown_or_unconfirmed_bend_releases_direction_guard(self):
        for stamp, direction, frames, expected in (
                (1.,'RIGHT',3,-1),(1.1,'UNKNOWN',0,1),
                (1.2,'RIGHT',2,1),(1.3,'RIGHT',3,-1)):
            self.c.observe_lane([(.5,.04),(.7,-.04),(.9,-.20)],.9,stamp,{},direction,frames)
            self.assertGreater(self.c.tick(stamp)[1]*expected,0)
        self.assertEqual(self.c.tick(2.),(0,0.))

    def test_bend_guard_does_not_override_maneuver_steering(self):
        self.c.observe_lane([(.5,.04),(.7,.04),(.9,.04)],.9,1.,{},'RIGHT',3)
        self.c.action='LEFT'
        self.assertGreater(self.c.lane_command(1.)[1],0)

    def test_old_bend_observation_cannot_replace_new_frame(self):
        self.c.observe_lane([(.5,.04),(.7,-.04),(.9,-.20)],.9,2.,{},'RIGHT',3)
        self.c.observe_lane([(.5,-.04),(.7,-.04),(.9,-.04)],.9,1.,{},'LEFT',3)
        self.assertLess(self.c.tick(2.)[1],0)

    def test_one_far_point_cannot_override_remaining_path(self):
        self.c.observe_lane([(.5,.04),(.7,.04),(.9,-.2)],.9,1.,{},'RIGHT',3)
        self.assertGreater(self.c.tick(1.)[1],0)

    def test_bend_target_also_controls_curve_slowdown(self):
        self.c.cfg['lane_curve_speed_raw']=16
        self.c.observe_lane([(.5,.005),(.7,-.1),(.9,-.3)],.9,1.,{},'RIGHT',3)
        speed,steer=self.c.tick(1.)
        self.assertEqual(speed,16)
        self.assertEqual(encode_command(speed,steer,self.c.cfg,0)['steering_raw'],-22)

    def test_curve_speed_reduction_preserves_full_steering_both_sides(self):
        self.c.cfg['lane_curve_speed_raw']=16
        self.assertEqual(self.observe(1.,-.3),(16,-22))
        self.assertEqual(self.observe(1.05,.3),(16,22))

    def test_curve_speed_is_progressive_and_never_accelerates_low_setting(self):
        self.c.cfg['lane_curve_speed_raw']=16
        self.assertEqual(self.observe(1.,0.),(24,0))
        speed,raw=self.observe(1.05,.02)
        self.assertTrue(16<speed<24,(speed,raw))
        self.assertTrue(8<raw<22,raw)
        self.c.cfg['speed_raw']['lane']=12
        self.assertEqual(self.observe(1.1,.3),(12,22))

    def test_reversal_guard_does_not_restore_straight_speed_on_curve(self):
        self.c.cfg['lane_curve_speed_raw']=16
        self.observe(1.,-.3)
        self.observe(1.1,None,0)
        speed,raw=self.observe(1.15,.3)
        self.assertEqual(speed,16)
        self.assertTrue(-22<raw<0,raw)

    def test_curve_speed_does_not_change_loss_or_stale_stop(self):
        self.c.cfg['lane_curve_speed_raw']=16
        self.observe(1.,-.3)
        self.assertEqual(self.observe(1.1,None,0),(16,-22))
        self.assertEqual(self.c.tick(2.),(0,0.))

    def test_curve_exit_resumes_straight_speed_and_centering_is_immediate(self):
        self.c.cfg['lane_curve_speed_raw']=16
        self.observe(1.,-.3)
        self.assertEqual(self.observe(1.05,0.),(16,0))
        self.assertEqual(self.observe(1.1,0.),(24,0))

    def test_deployed_config_enables_curve_speed(self):
        root=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.assertEqual(load_config(os.path.join(root,'config'))['lane_curve_speed_raw'],16)

    def test_near_curve_correction_can_increase_but_never_reverse_steering(self):
        self.c.cfg.update(lookahead=.7,lane_curve_lookahead_m=.45)
        for sign,t in ((-1,1.),(1,2.)):
            self.c.observe_lane([(.45,sign*.07),(.7,sign*.04),(.9,sign*.03)],.9,t)
            speed,steer=self.c.tick(t)
            self.assertEqual(encode_command(speed,steer,self.c.cfg,0)['steering_raw'],sign*22)
        self.c.observe_lane([(.45,-.15),(.7,.04),(.9,.03)],.9,3.)
        self.assertGreater(self.c.tick(3.)[1],0)

    def test_loss_holds_and_recovery_cannot_instantly_reverse_full_lock(self):
        self.assertEqual(self.observe(1.,-.3),(24,-22))
        self.assertEqual(self.observe(1.1,None,0),(16,-22))
        speed,raw=self.observe(1.15,.3)
        self.assertEqual(speed,24)
        self.assertTrue(-22<raw<0,raw)
        previous=raw
        for i in range(1,16):
            speed,raw=self.observe(1.15+.05*i,.3)
            self.assertLessEqual(abs(raw-previous),5)
            previous=raw
        self.assertEqual(raw,22)
        self.assertIsNone(self.c.lane_recovery)
        self.assertEqual(self.observe(2.,-.3),(24,-22))

    def test_same_direction_full_lock_not_weakened_after_loss(self):
        self.observe(1.,-.3)
        self.observe(1.1,None,0)
        self.assertEqual(self.observe(1.15,-.3),(24,-22))
        self.assertIsNone(self.c.lane_recovery)

    def test_same_direction_increase_reaches_full_lock_immediately(self):
        for sign,start in ((-1,1.),(1,3.)):
            speed,raw=self.observe(start,sign*.02)
            self.assertTrue(0<abs(raw)<22)
            self.observe(start+.1,None,0)
            self.assertEqual(self.observe(start+.15,sign*.3),(24,sign*22))
            self.assertIsNone(self.c.lane_recovery)

    def test_reduction_and_centering_are_immediate(self):
        for target,start in ((-.02,1.),(0.,3.),(.02,5.)):
            sign=-1 if target<=0 else 1
            self.observe(start,sign*.3)
            self.observe(start+.1,None,0)
            command=self.observe(start+.15,target)
            self.assertIsNone(self.c.lane_recovery)
            self.assertEqual(command,self.observe(start+.2,target))
            self.assertLess(abs(command[1]),12)

    def test_reversal_can_be_cancelled_toward_original_side(self):
        self.observe(1.,-.3);self.observe(1.1,None,0)
        self.assertLess(self.observe(1.15,.3)[1],0)
        self.assertEqual(self.observe(1.2,-.3),(24,-22))
        self.assertIsNone(self.c.lane_recovery)

    def test_repeated_gap_and_low_confidence_cannot_bypass_limit(self):
        self.observe(1.,-.3)
        self.observe(1.1,.3,.2)
        self.assertLess(self.observe(1.15,.3)[1],0)
        self.observe(1.2,None,0)
        self.assertLess(self.observe(1.25,.3)[1],0)

    def test_repeated_loss_cannot_extend_reversal_window(self):
        for sign,start in ((-1,1.),(1,4.)):
            self.observe(start,sign*.3)
            self.observe(start+.1,None,0)
            self.assertTrue(abs(self.observe(start+.15,-sign*.3)[1])<22)
            for dt in (.2,.3,.4,.5,.6):
                self.observe(start+dt,None,0)
                self.observe(start+dt+.01,-sign*.3)
            self.assertEqual(self.observe(start+.66,-sign*.3),(24,-sign*22))
            self.assertIsNone(self.c.lane_recovery)
            # A genuinely new loss still guards the next sudden reversal.
            self.observe(start+.7,None,0)
            self.assertTrue(abs(self.observe(start+.75,sign*.3)[1])<22)

    def test_long_loss_stops_then_recovers_from_centered_command(self):
        self.observe(1.,-.3)
        self.observe(1.1,None,0)
        self.assertEqual(self.observe(5.01,None,0),(0,0))
        self.assertEqual(self.observe(5.05,.3),(24,22))

    def test_fresh_empty_frames_use_configured_gap_duration(self):
        self.observe(1.,-.3)
        self.assertEqual(self.observe(1.1,None,0),(16,-22))
        self.assertEqual(self.observe(1.7,None,0),(16,-22))
        self.assertEqual(self.observe(4.9,None,0),(16,-22))
        self.assertEqual(self.observe(5.01,None,0),(0,0))

    def test_shorter_configured_gap_duration_is_respected(self):
        self.c.cfg['gap_max_seconds']=.3
        self.observe(1.,-.3)
        self.assertEqual(self.observe(1.2,None,0),(16,-22))
        self.assertEqual(self.observe(1.31,None,0),(0,0))

    def test_gap_distance_still_stops_before_timeout(self):
        self.observe(1.,-.3)
        self.c.set_pose((self.c.cfg['gap_max_distance']+.01,0,0),1.2)
        self.assertEqual(self.observe(1.2,None,0),(0,0))
        self.assertEqual(self.c.reason,'gap_limit_wait_for_lane')

    def test_expired_gap_still_stops_then_recovers_from_zero(self):
        self.observe(1.,-.3)
        self.assertEqual(self.observe(6.,None,0),(0,0))
        speed,raw=self.observe(6.05,.3)
        self.assertEqual(speed,24)
        self.assertEqual(raw,22)  # From stopped/centered is not a reversal.

    def test_stale_camera_stops_but_centered_restart_is_not_a_reversal(self):
        self.observe(1.,-.3)
        self.assertEqual(self.c.tick(2.),(0,0.))
        self.assertEqual(self.observe(2.05,.3),(24,22))

    def test_no_initial_lane_does_not_create_recovery_restriction(self):
        self.assertEqual(self.observe(1.,None,0),(0,0))
        self.assertEqual(self.observe(1.1,.3),(24,22))

    def test_bypass_remains_full_lock_and_does_not_inherit_recovery(self):
        self.c.cfg['lane_curve_speed_raw']=16
        self.observe(1.,-.3);self.observe(1.1,None,0)
        self.c.state='TIMED_BYPASS';self.c.action='BYPASS'
        self.c.timed_bypass=dict(phase='RIGHT',elapsed_s=0.,last=2.)
        duration=self.c.cfg.get('timed_bypass_right_s',6.)
        end=2.+duration
        for i in range(int(round(duration/.05))):
            t=2.+i*.05
            self.assertEqual(self.observe(t,.3),(self.c.cfg['speed_raw']['action'],-22))
        self.assertEqual(self.observe(end,.3),(0,0))
        self.assertEqual(self.c.reason,'bypass_wait_exit_lane')
        for t in (end+.1,end+.2,end+.3):
            command=self.observe(t,.1)
        self.assertGreater(command[0],0)
        self.assertGreater(command[1],0)
        self.assertIsNone(self.c.lane_recovery)


if __name__=='__main__':unittest.main()
