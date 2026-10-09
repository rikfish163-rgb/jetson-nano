"""Right-bend intent survives false targets; real exits and maneuvers release it."""
import os
import unittest
from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.master.controller import Controller


class RightCurveGuardTests(unittest.TestCase):
    RIGHT = [(.5,0.),(.7,-.04),(.9,-.12),(1.1,-.24)]
    FALSE_LEFT = [(.77,.29),(.85,.28),(.98,.27),(1.05,.25)]
    STRAIGHT = [(.5,.02),(.7,.02),(.9,.02),(1.1,.02)]

    def setUp(self):
        cfg=load_config(os.path.join(os.path.dirname(__file__),'../config'))
        cfg.update(wait_green=False,lidar_enabled=False,blue_default_straight=False,
                   steering_command_scale_rad=.03,lane_lateral_full_scale_m=.075)
        self.c=Controller(cfg)
        self.addCleanup(self.c.close)

    def command(self,stamp,points,now=None):
        self.c.observe_lane(points,.8,stamp)
        speed,steer=self.c.lane_command(stamp if now is None else now)
        self.c.issued_steer=steer
        return speed,encode_command(speed,steer,self.c.cfg,0)['steering_raw']

    def test_first_clear_right_bend_blocks_opposite_target_and_gap(self):
        self.assertEqual(self.command(1.,self.RIGHT),(20,-22))
        self.assertEqual(self.command(1.1,self.FALSE_LEFT),(16,-22))
        self.assertIsNone(self.c.lane_target['target'])
        self.assertEqual(self.c.lane_target['selection'],'right_curve_hold')
        self.assertEqual(self.command(1.2,[]),(16,-22))

    def test_gap_does_not_copy_wrong_sign_from_applied_feedback(self):
        self.command(1.,self.RIGHT)
        self.c.observe_applied_steering(.03,1.09)
        self.assertEqual(self.command(1.1,[]),(16,-22))

    def test_first_recognized_right_bend_immediately_uses_full_raw(self):
        path=[(.5,.04),(.7,0.),(.9,-.04)]
        self.assertEqual(self.command(1.,path),(20,-22))

    def test_right_lock_keeps_full_raw_when_target_offset_shrinks(self):
        self.command(1.,self.RIGHT)
        path=[(.5,.12),(.7,.04),(.9,-.02)]
        self.assertEqual(self.command(1.1,path),(20,-22))
        self.assertEqual(self.c.lane_target['selection'],'right_curve_full_lock')
        self.assertEqual(self.command(1.2,[]),(16,-22))

    def test_full_right_raw_is_independent_of_command_scale(self):
        path=[(.5,.04),(.7,0.),(.9,-.04)]
        for i,scale in enumerate((.01,.03,.1)):
            self.c.cfg['steering_command_scale_rad']=scale
            self.assertEqual(self.command(1.+i,path),(20,-22))

    def test_lidar_timeout_stops_speed_without_recentering_right_bend(self):
        self.command(1.,self.RIGHT)
        self.c.cfg['lidar_enabled']=True
        self.assertEqual(self.c.tick(1.1),(0,-.03))
        self.assertEqual(self.c.reason,'scan_missing_or_stale')
        self.assertIsNotNone(self.c.lane_curve_lock)

    def test_rejected_frames_do_not_refresh_gap_time_or_distance(self):
        self.c.cfg['gap_max_seconds']=.3
        self.command(1.,self.RIGHT)
        self.command(1.2,self.FALSE_LEFT)
        self.assertEqual(self.command(1.31,self.FALSE_LEFT),(0,0))
        self.assertEqual(self.c.reason,'gap_limit_wait_for_lane')
        self.assertEqual(self.command(1.4,self.RIGHT),(20,-22))
        self.c.set_pose((.81,0.,0.),1.5)
        self.assertEqual(self.command(1.5,self.FALSE_LEFT),(0,0))

    def test_exit_requires_distinct_aligned_frames_and_elapsed_time(self):
        self.command(1.,self.RIGHT)
        self.assertEqual(self.command(1.1,self.STRAIGHT)[1],-22)
        for t in (1.2,1.3,1.4,1.5):
            self.assertEqual(self.command(1.1,self.STRAIGHT,now=t)[1],-22)
        self.assertIsNotNone(self.c.lane_curve_lock)
        for t in (1.6,1.7,1.8,1.9):self.command(t,self.STRAIGHT)
        self.assertIsNone(self.c.lane_curve_lock)
        self.assertGreater(self.command(2.,self.STRAIGHT)[1],0)
        self.assertEqual(self.command(2.1,self.RIGHT)[1],-22)
        self.assertIsNotNone(self.c.lane_curve_lock)

    def test_wrong_straight_lane_and_sparse_points_do_not_release(self):
        self.command(1.,self.RIGHT)
        for t in (1.1,1.3,1.5,1.7,1.9):
            self.assertLessEqual(self.command(t,[(.5,.25),(.8,.25),(1.1,.25)])[1],0)
        self.assertIsNotNone(self.c.lane_curve_lock)
        self.command(2.,[(.8,0.)])
        self.assertIsNotNone(self.c.lane_curve_lock)

    def test_straight_exit_allows_normal_small_lateral_correction(self):
        self.command(1.,self.RIGHT)
        # Pixel jitter must not look like a bend; after exit a 12 cm offset
        # still needs ordinary left correction within this 60 cm lane.
        path=[(.5,.12),(.7,.13),(.9,.11),(1.1,.12)]
        for t in (1.1,1.2,1.3,1.4,1.5):self.command(t,path)
        self.assertIsNone(self.c.lane_curve_lock)
        self.assertGreater(self.command(1.6,path)[1],0)

    def test_recorded_skewed_exit_releases_without_manual_alignment(self):
        self.command(1.,self.RIGHT)
        # Run 20260930_011909, source 1790702407.6140318: a real straight
        # exit points left by 13 degrees because the car still needs correction.
        path=[(.626784712,-.075966904),(.723303755,-.052261866),
              (.819791023,-.029153978),(.916247681,-.006146625),
              (1.012674951,.017599122),(1.109074119,.035574609)]
        for t in (1.1,1.2,1.3,1.4):
            self.assertEqual(self.command(t,path),(20,-22))
            self.assertEqual(self.c.state,'LANE')
        speed,raw=self.command(1.5,path)
        self.assertEqual(speed,20)
        self.assertGreater(raw,0)
        self.assertIsNone(self.c.lane_curve_lock)

    def test_recorded_exit_far_offset_does_not_require_already_aligned_car(self):
        self.command(1.,self.RIGHT)
        # Same run, source 1790702411.134001: the far point is 17.8 cm left,
        # but the straight line passes near the car and must be followed.
        path=[(.739642775,.069516953),(.835762230,.099360753),
              (.931834233,.126623347),(1.027860236,.154041723),
              (1.123841801,.178014386)]
        for t in (1.1,1.2,1.3,1.4,1.5):speed,raw=self.command(t,path)
        self.assertEqual(speed,20)
        self.assertGreater(raw,0)
        self.assertIsNone(self.c.lane_curve_lock)

    def test_real_empty_frame_breaks_exit_confirmation(self):
        self.command(1.,self.RIGHT)
        path=[(.5,-.06),(.7,0.),(.9,.06),(1.1,.12)]
        self.command(1.1,path)
        self.command(1.2,path)
        self.assertEqual(self.command(1.25,[]),(16,-22))
        # Only real, consecutive exit observations may release the guard;
        # an empty frame must not count as successful exit confirmation.
        for t in (1.3,1.4,1.5,1.6):
            self.assertEqual(self.command(t,path),(20,-22))
            self.assertIsNotNone(self.c.lane_curve_lock)
        self.assertGreater(self.command(1.7,path)[1],0)
        self.assertIsNone(self.c.lane_curve_lock)

    def test_straight_segment_still_facing_right_does_not_release(self):
        path=[(.5,-.06),(.7,-.12),(.9,-.18),(1.1,-.24)]
        for t in (1.,1.1,1.2,1.3,1.4,1.5):
            self.assertEqual(self.command(t,path),(20,-22))
        self.assertIsNotNone(self.c.lane_curve_lock)
        self.assertEqual(self.c.lane_curve_lock['exit_frames'],0)

    def test_recorded_far_tail_still_right_facing_does_not_release(self):
        self.command(1.,self.RIGHT)
        # Recorded path 021005: the global fit is near -8 degrees, while the
        # supported far suffix remains right-facing at about -13 degrees.
        path=[(.5601,.0274),(.6116,.0291),(.7031,.0264),(.7680,.0201),
              (.8381,.0092),(.9066,-.0055),(1.0064,-.0341)]
        for t in (1.1,1.2,1.3,1.4,1.5):
            self.assertEqual(self.command(t,path)[1],-22)
            self.assertIsNotNone(self.c.lane_curve_lock)
            self.assertEqual(self.c.lane_curve_lock['exit_frames'],0)

    def test_sharp_left_path_is_not_an_exit_correction(self):
        self.command(1.,self.RIGHT)
        path=[(.5,-.10),(.7,.06),(.9,.22),(1.1,.38)]
        for t in (1.1,1.2,1.3,1.4,1.5):
            self.assertEqual(self.command(t,path),(16,-22))
        self.assertIsNotNone(self.c.lane_curve_lock)

    def test_stale_stream_stops_and_lifecycle_clears_lock(self):
        self.command(1.,self.RIGHT)
        self.assertEqual(self.c.lane_command(1.51),(0,-.03))
        self.c.begin_startup(2.)
        self.assertIsNone(self.c.lane_curve_lock)
        self.c.state='LANE'
        self.command(3.,self.RIGHT)
        self.c.start_follow([(0.,0.,0.,1,0.),(.2,.1,.2,1,0.)],'LEFT',3.1)
        self.assertIsNone(self.c.lane_curve_lock)
        self.c.resume_lane()
        self.assertIsNone(self.c.lane_curve_lock)

    def test_repeated_sensor_stops_preserve_angle_then_fresh_lane_resumes(self):
        self.command(1.,self.RIGHT)
        for now in (1.51,1.7,2.):
            self.assertEqual(self.c.tick(now),(0,-.03))
            self.assertEqual(self.c.reason,'lane_stream_stale')
        self.assertEqual(self.command(2.1,self.RIGHT),(20,-22))

    def test_sensor_stop_does_not_count_as_exit_evidence(self):
        self.command(1.,self.RIGHT)
        for stamp in (1.1,1.2,1.3,1.4):self.command(stamp,self.STRAIGHT)
        self.c.cfg['lidar_enabled']=True
        self.assertEqual(self.c.tick(1.41),(0,-.03))
        self.assertEqual(self.c.lane_curve_lock['exit_frames'],0)
        self.c.cfg['lidar_enabled']=False
        self.assertEqual(self.command(1.5,self.STRAIGHT),(20,-22))
        self.assertIsNotNone(self.c.lane_curve_lock)

    def test_other_stops_keep_zero_steering_even_with_curve_lock(self):
        for reason in ('emergency_stop','red_latched','odom_stale',
                       'gap_limit_wait_for_lane','obstacle_in_path'):
            self.command(1.,self.RIGHT)
            self.assertEqual(self.c.stop(reason),(0,0.))
            self.assertEqual(self.c.issued_steer,0.)

    def test_sensor_timeout_without_curve_lock_keeps_zero_steering(self):
        self.command(1.,self.FALSE_LEFT)
        self.assertEqual(self.c.tick(1.51),(0,0.))
        self.c.cfg['lidar_enabled']=True
        self.assertEqual(self.c.tick(1.6),(0,0.))

    def test_gap_sensor_timeout_keeps_angle_but_estop_and_red_override(self):
        self.command(1.,self.RIGHT)
        self.assertEqual(self.command(1.1,[]),(16,-22))
        self.assertEqual(self.c.tick(1.61),(0,-.03))
        self.c.estop=True
        self.assertEqual(self.c.tick(1.62),(0,0.))
        self.c.estop=False
        self.c.red=True
        self.assertEqual(self.c.tick(1.63),(0,0.))

    def test_left_correction_without_right_bend_is_unchanged(self):
        self.assertEqual(self.command(1.,self.FALSE_LEFT),(20,22))
        self.assertIsNone(self.c.lane_curve_lock)

    def test_special_states_clear_lock_before_their_commands(self):
        for i,state in enumerate(('TIMED_BYPASS','BLUE_APPROACH','BLUE_STOP',
                'UTURN','PARKING','PARALLEL_PARKING','STARTUP_STRAIGHT')):
            stamp=10.+i
            self.c.state='LANE'
            self.command(stamp,self.RIGHT)
            self.assertIsNotNone(self.c.lane_curve_lock)
            self.c.state=state
            self.c.estop=True  # exercise entry cleanup without actuating a maneuver
            self.assertEqual(self.c.tick(stamp+.01),(0,0.))
            self.assertIsNone(self.c.lane_curve_lock)
            self.c.estop=False


if __name__=='__main__':unittest.main()
