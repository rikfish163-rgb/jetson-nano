"""Image-trigger timing and guard tests. No ROS or hardware publishers."""
import copy
import unittest
from test_core import CONFIG
from robot.turn.blue_stop_test import BlueStopTest


class BlueStopTests(unittest.TestCase):
    def make(self):
        settings=dict(speed_raw=12,trigger_row_ratio=.65,trigger_band_ratio=.2,
                      confirm_frames=3,heading_tolerance_deg=10,forward_seconds=1,
                      max_search_seconds=15,camera_timeout_s=.6,align_speed_raw=12,
                      align_max_steering_raw=14,align_tolerance_deg=5,
                      align_confirm_frames=3,align_timeout_s=8)
        c=BlueStopTest(copy.deepcopy(CONFIG),settings)
        self.addCleanup(c.close)
        c.scan_ready=lambda now:True
        c.checked_command=lambda command,now,allow_bypass:command
        return c

    def frame(self,c,t,row=None,angle=0):
        import math
        cam=c.cfg['front_camera']
        lines=[] if row is None else [dict(x=(cam['origin_v']-row*(cam['bev_height']-1))/cam['pixels_per_m'],y=0,yaw=math.radians(angle),length=.6)]
        c.observe_ground(dict(source='front',part='markers',blue_lines=lines),t)
        return c.tick(t)

    def test_far_approach_confirm_then_fixed_time(self):
        c=self.make()
        self.frame(c,.8,.15);self.frame(c,.9,.18)
        self.assertEqual(self.frame(c,1,.2),(12,0))
        self.frame(c,1.1,.66)
        c.tick(1.15)
        self.assertEqual(c.test_confirmations,1)
        self.frame(c,1.2,.68);self.frame(c,1.3,.70)
        self.assertEqual(c.test_trigger,1.3)
        for t in (1.5,1.7,1.9,2.1):self.frame(c,t)
        self.assertEqual(self.frame(c,2.31),(0,0))
        self.assertTrue(c.test_done)
        self.assertEqual(self.frame(c,2.4,.3),(0,0))

    def test_too_close_and_skew_stop(self):
        c=self.make();self.assertEqual(self.frame(c,1,.9),(0,0))
        self.assertEqual(c.test_result,'blue_test_past_trigger_band')
        c=self.make();self.assertEqual(self.frame(c,1,.7,20),(0,0))
        self.assertEqual(c.test_result,'blue_test_alignment_distance_insufficient')

    def test_steers_both_directions_with_raw_limit_then_centers(self):
        from robot.common.contracts import encode_command
        for angle in (-20,20):
            c=self.make()
            speed,steer=self.frame(c,1,.3,angle)
            self.assertGreater(steer*angle,0)
            raw=encode_command(speed,steer,c.cfg,0)
            self.assertEqual(abs(raw['steering_raw']),14)
            for t in (1.1,1.2,1.3):self.frame(c,t,.4,2)
            self.assertTrue(c.test_sequence['aligned'])
            self.assertEqual(self.frame(c,1.4,.5,2),(12,0))
            for t in (1.5,1.6,1.7):self.frame(c,t,.7,6)
            self.assertEqual(c.test_trigger,1.7)
            self.assertEqual(self.frame(c,1.8,.8,20),(12,0))

    def test_alignment_votes_need_distinct_consecutive_good_frames(self):
        c=self.make();self.frame(c,1,.3,2)
        c.tick(1.1)
        self.assertEqual(c.test_sequence['align_frames'],1)
        self.frame(c,1.2,.4,7)
        self.assertEqual(c.test_sequence['align_frames'],0)
        self.frame(c,1.3,.45,2);self.frame(c,1.4,.5,2)
        self.assertFalse(c.test_sequence.get('aligned',False))
        self.frame(c,1.5,.55,2)
        self.assertTrue(c.test_sequence['aligned'])
        self.frame(c,1.6,.6,12)
        self.assertFalse(c.test_sequence['aligned'])
        self.assertEqual(self.frame(c,1.7,.7,11.84),(0,0))
        self.assertIsNone(c.test_trigger)

    def test_already_straight_at_trigger_can_finish_votes_without_fault(self):
        c=self.make()
        self.frame(c,1,.60,-6)
        self.assertEqual(self.frame(c,1.1,.66,-4.8),(12,0))
        self.assertFalse(c.test_sequence.get('aligned',False))
        self.assertIsNone(c.test_trigger)
        c.tick(1.15)
        self.assertEqual(c.test_sequence['align_frames'],1)
        self.frame(c,1.2,.68,-4.8)
        self.assertIsNone(c.test_trigger)
        self.frame(c,1.3,.70,-4.8)
        self.assertTrue(c.test_sequence['aligned'])
        self.assertEqual(c.test_trigger,1.3)

    def test_alignment_timeout_and_lost_target_stop(self):
        c=self.make();c.test_settings['align_timeout_s']=.3
        for t in (1,1.1,1.2):self.frame(c,t,.3,20)
        self.assertEqual(self.frame(c,1.31,.4,20),(0,0))
        self.assertEqual(c.test_result,'blue_test_alignment_timeout')
        c=self.make();self.frame(c,1,.3,20)
        self.assertEqual(self.frame(c,1.1),(0,0))
        self.assertEqual(c.test_result,'blue_test_blue_lost_before_trigger')

    def test_invalid_alignment_settings_rejected(self):
        from robot.turn.blue_timed import validate_settings
        c=self.make()
        for key,value in [('align_max_steering_raw',23),('align_confirm_frames',1.5),
                          ('align_tolerance_deg',11),('align_speed_raw',0)]:
            settings=dict(c.test_settings);settings[key]=value
            with self.assertRaises(ValueError):validate_settings(settings,c.cfg)

    def test_feedback_reduces_both_heading_errors(self):
        import math
        from robot.common.contracts import encode_command
        from robot.motion.calibration import raw_angle, speed_gain
        from robot.common.geometry import bicycle
        for initial in (-20,20):
            c=self.make();pose=(0,0,-math.radians(initial))
            for i in range(70):
                command=self.frame(c,1+i*.1,.3,-math.degrees(pose[2]))
                raw=encode_command(command[0],command[1],c.cfg,0)
                speed=raw['speed_raw']
                pose=bicycle(pose,speed*speed_gain(c.cfg,speed)*.1,
                             raw_angle(c.cfg,speed,raw['steering_raw']),c.cfg['wheelbase'])
                if c.test_sequence.get('aligned'):break
            self.assertFalse(c.test_done)
            self.assertTrue(c.test_sequence.get('aligned'))
            self.assertLessEqual(abs(math.degrees(pose[2])),5)

    def test_control_gap_and_stale_camera_stop_without_resuming(self):
        c=self.make();self.frame(c,1,.3,20)
        self.assertEqual(self.frame(c,1.501,.4,10),(0,0))
        self.assertEqual(c.test_result,'blue_test_control_gap')
        self.assertEqual(self.frame(c,1.6,.4,0),(0,0))
        c=self.make();self.frame(c,1,.3,20)
        for t in (1.2,1.4):c.tick(t)
        self.assertEqual(c.tick(1.61),(0,0))
        self.assertEqual(c.test_result,'blue_test_sensor_lost')

    def test_sensor_and_obstacle_stop_latch(self):
        c=self.make();self.frame(c,1,.3);c.scan_ready=lambda now:False
        self.assertEqual(c.tick(1.1),(0,0));self.assertTrue(c.test_done)
        c=self.make();c.checked_command=lambda *args:(0,0)
        self.assertEqual(self.frame(c,1,.3),(0,0));self.assertTrue(c.test_done)

    def test_wait_for_camera_and_estop(self):
        c=self.make();self.assertEqual(c.tick(1),(0,0))
        self.assertIsNone(c.test_start)
        c.estop=True;self.assertEqual(c.tick(1.1),(0,0));self.assertTrue(c.test_done)

    def test_center_crossing_filters_adjacent_paint(self):
        c=self.make();c.test_lines=[dict(x=.8,y=1,yaw=0,length=.4)]
        self.assertIsNone(c.row_candidate())


if __name__=='__main__':unittest.main()
