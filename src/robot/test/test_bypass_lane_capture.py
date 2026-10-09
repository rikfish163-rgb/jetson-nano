"""After the minimum arc, confirm a capturable bypass lane while stopped."""
import os
import unittest
from robot.common.config import load_config
from robot.common.contracts import validate_config
from robot.common.geometry import local
from robot.master.controller import Controller
from test_continuous_obstacles import _SyntheticScan


class BypassLaneCaptureTests(unittest.TestCase):
    CENTER=[(.55,.03),(.70,.03),(.85,.03),(1.,.03)]

    def core(self,elapsed=4.4):
        cfg=load_config(os.path.join(os.path.dirname(__file__),'../config'))
        cfg.update(wait_green=False,lidar_enabled=True,timed_bypass_enabled=True,
                   lane_curvature_preview=True,lane_curve_speed_raw=16,
                   steering_command_scale_rad=.03,timed_bypass_right_s=4.5,
                   timed_bypass_right_max_s=6.,timed_bypass_speed_raw=26)
        cfg['speed_raw'].update(action=24,lane=24)
        c=Controller(cfg);self.addCleanup(c.close)
        c.state='TIMED_BYPASS';c.action='BYPASS';c.issued_steer=-.03
        c.timed_bypass=dict(phase='RIGHT',last=1.,elapsed_s=elapsed,
            trigger_world=(-.5,0),trigger_point=(.8,0))
        return c

    def tick(self,c,path,stamp,confidence=.8,source_stamp=None):
        c.observe_lane(path,confidence,stamp if source_stamp is None else source_stamp)
        c.scan=_SyntheticScan(stamp)
        return c.execute('obstacle','timed_bypass_tick',stamp).value

    def test_default_right_minimum_is_four_seconds(self):
        cfg=load_config(os.path.join(os.path.dirname(__file__),'../config'))
        self.assertEqual(cfg['timed_bypass_right_s'],4.)
        self.assertEqual(cfg['timed_bypass_right_max_s'],6.)

    def test_production_minimum_does_not_release_at_recorded_three_seconds(self):
        cfg=load_config(os.path.join(os.path.dirname(__file__),'../config'))
        c=self.core(3.)
        c.cfg['timed_bypass_right_s']=cfg['timed_bypass_right_s']
        for stamp in (1.064331,1.3,1.6,1.9):
            self.assertEqual(self.tick(c,self.CENTER,stamp),(26,-.03))
            self.assertEqual(c.timed_bypass['phase'],'RIGHT')
        self.assertEqual(self.tick(c,self.CENTER,2.),(0,0.))
        self.assertEqual(c.timed_bypass['phase'],'REACQUIRE')
        self.assertAlmostEqual(c.timed_bypass['right_completed_s'],4.)

    def test_stable_lane_cannot_end_right_before_minimum(self):
        c=self.core(4.1)
        for now in (1.1,1.2,1.3):
            self.assertEqual(self.tick(c,self.CENTER,now),(26,-.03))
        self.assertEqual(c.timed_bypass['phase'],'RIGHT')

    def test_capturable_lane_stops_full_lock_while_confirming(self):
        c=self.core(3.0);c.cfg['timed_bypass_right_s']=3.0
        self.assertEqual(self.tick(c,self.CENTER,1.16305995),(0,0.))
        self.assertEqual(c.timed_bypass['phase'],'REACQUIRE')
        self.assertAlmostEqual(c.timed_bypass['right_completed_s'],3.16305995)
        self.assertEqual(c.timed_bypass['lane_exit']['frames'],1)
        self.assertFalse(c.timed_bypass_completed)
        self.assertEqual(self.tick(c,[],1.25,0.),(0,0.))
        self.assertEqual(c.timed_bypass['phase'],'REACQUIRE')

    def test_three_fresh_consistent_frames_handoff_after_minimum(self):
        c=self.core()
        for now in (1.1,1.2):
            self.assertEqual(self.tick(c,self.CENTER,now),(0,0.))
        speed,steer=self.tick(c,self.CENTER,1.3)
        self.assertIsNone(c.timed_bypass)
        self.assertEqual(c.state,'LANE')
        self.assertTrue(c.timed_bypass_completed)
        self.assertEqual(c.reason,'bypass_visual_lane_handoff')
        self.assertGreater(speed,0)
        self.assertLessEqual(speed,26)
        self.assertGreater(steer,-.03)

    def test_single_valid_frame_at_old_five_second_endpoint_does_not_handoff(self):
        c=self.core(4.95)
        self.assertEqual(self.tick(c,self.CENTER,1.1),(0,0.))
        self.assertEqual(c.timed_bypass['phase'],'REACQUIRE')

    def test_missing_lane_continues_search_past_five_seconds(self):
        c=self.core(4.95)
        self.assertEqual(self.tick(c,[],1.1,0.),(26,-.03))
        self.assertEqual(c.timed_bypass['phase'],'RIGHT')

    def test_no_lane_stops_search_at_six_second_limit(self):
        c=self.core(5.8)
        self.assertEqual(self.tick(c,[],1.1,0.),(26,-.03))
        self.assertEqual(self.tick(c,[],1.2,0.),(0,0.))
        self.assertEqual(c.timed_bypass['phase'],'REACQUIRE')
        self.assertEqual(c.reason,'bypass_wait_exit_lane')
        self.assertAlmostEqual(c.timed_bypass['right_completed_s'],6.)

    def test_stopped_search_can_handoff_after_three_new_lane_frames(self):
        c=self.core(5.9)
        self.assertEqual(self.tick(c,[],1.1,0.),(0,0.))
        for now in (1.2,1.3):
            self.assertEqual(self.tick(c,self.CENTER,now),(0,0.))
        self.assertGreater(self.tick(c,self.CENTER,1.4)[0],0)
        self.assertIsNone(c.timed_bypass)

    def test_repeated_source_image_does_not_confirm_exit(self):
        c=self.core(4.5)
        for now in (1.1,1.2,1.3):
            self.assertEqual(self.tick(c,self.CENTER,now,source_stamp=1.1),(0,0.))
        self.assertIsNotNone(c.timed_bypass)

    def test_alternating_detected_lanes_reset_confirmation(self):
        c=self.core(4.5)
        for i,offset in enumerate((-.2,.2,-.2,.2)):
            self.assertEqual(self.tick(c,[(x,offset) for x,y in self.CENTER],
                                      1.1+i*.1),(0,0.))
        self.assertIsNotNone(c.timed_bypass)

    def test_world_frame_consistency_allows_vehicle_motion_during_capture(self):
        c=self.core();road=[(.6,0.),(.8,0.),(1.,0.),(1.2,0.)]
        for i in range(3):
            now=1.1+i*.1
            c.set_pose((i*.025,0.,i*.015),now)
            command=self.tick(c,[local(c.pose,p) for p in road],now)
        self.assertGreater(command[0],0)
        self.assertIsNone(c.timed_bypass)

    def test_far_sideways_or_low_confidence_path_cannot_handoff(self):
        for path,confidence in [([(x,.6) for x,y in self.CENTER],.8),
                ([(.5,0),(.6,.12),(.7,.24),(.8,.36)],.8),
                (self.CENTER,.1146234375)]:
            c=self.core()
            for now in (1.1,1.2,1.3):
                self.assertEqual(self.tick(c,path,now,confidence),(26,-.03))
            self.assertIsNotNone(c.timed_bypass)

    def test_trigger_still_ahead_cannot_complete_bypass(self):
        c=self.core();c.timed_bypass['trigger_world']=(.2,0.)
        for now in (1.1,1.2,1.3):
            self.assertEqual(self.tick(c,self.CENTER,now),(26,-.03))
        self.assertIsNotNone(c.timed_bypass)

    def test_aligned_sparse_lane_can_handoff_without_being_already_centered(self):
        c=self.core();path=[(.6,-.3),(.72,-.3)]
        for now in (1.1,1.2):self.assertEqual(self.tick(c,path,now,.329595),(0,0.))
        self.assertGreater(self.tick(c,path,1.3,.329595)[0],0)
        self.assertIsNone(c.timed_bypass)

    def test_safety_stop_pauses_minimum_motion_timer(self):
        c=self.core(4.2);c.scan=None
        self.assertEqual(c.execute('obstacle','timed_bypass_tick',1.1).value,(0,0.))
        self.assertEqual(c.reason,'scan_missing_or_stale')
        self.assertAlmostEqual(c.timed_bypass['elapsed_s'],4.3)
        self.assertEqual(self.tick(c,self.CENTER,10.),(26,-.03))
        self.assertAlmostEqual(c.timed_bypass['elapsed_s'],4.3)
        self.assertEqual(self.tick(c,self.CENTER,10.1),(26,-.03))

    def test_heading_estimate_cannot_replace_visual_confirmation(self):
        c=self.core(4.5)
        c.timed_bypass.update(right_origin_heading_rad=1.,left_turn_angle_rad=1.)
        c.set_pose((0.,0.,.02),1.05)
        self.assertEqual(self.tick(c,[],1.1,0.),(26,-.03))
        self.assertEqual(c.timed_bypass['phase'],'RIGHT')

    def test_invalid_search_limit_rejected(self):
        c=self.core()
        for value in (4.4,12.1,float('nan'),True):
            c.cfg['timed_bypass_right_max_s']=value
            with self.assertRaises(ValueError):validate_config(c.cfg)


if __name__=='__main__':unittest.main()
