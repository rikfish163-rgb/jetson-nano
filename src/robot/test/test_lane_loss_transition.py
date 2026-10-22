"""Legacy path output with existing empty-path, timeout and distance stops."""
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
                   lookahead=.45,steering_command_scale_rad=.1,lane_lateral_full_scale_m=.30)
        cfg['speed_raw']['lane']=20
        self.c=Controller(cfg)
        self.addCleanup(self.c.close)

    def observe(self,t,y,confidence=.9):
        self.c.observe_lane([] if y is None else [(.5,y),(.7,y),(.9,y)],confidence,t)
        speed,steer=self.c.tick(t)
        return speed,encode_command(speed,steer,self.c.cfg,0)['steering_raw']

    def test_first_path_tracks_immediately_in_both_directions(self):
        self.assertEqual(self.observe(1.,-.1),(20,-7))
        self.assertEqual(self.observe(1.05,.1),(20,7))

    def test_empty_holds_command_but_current_points_override_low_confidence(self):
        self.assertEqual(self.observe(1.,-.1),(20,-7))
        self.assertEqual(self.observe(1.1,None,0),(16,-7))
        self.assertEqual(self.observe(1.2,.1,.2),(20,7))

    def test_valid_path_reacquires_without_added_reversal_filter(self):
        self.observe(1.,-.1)
        self.observe(1.1,None,0)
        self.assertEqual(self.observe(1.15,.1),(20,7))
        self.assertIsNone(self.c.lane_recovery)

    def test_fresh_empty_frames_obey_gap_time_limit(self):
        self.observe(1.,-.1)
        self.assertEqual(self.observe(4.9,None,0),(16,-7))
        self.assertEqual(self.observe(5.01,None,0),(0,0))
        self.assertEqual(self.observe(5.05,.1),(20,7))

    def test_shorter_configured_gap_duration_is_respected(self):
        self.c.cfg['gap_max_seconds']=.3
        self.observe(1.,-.1)
        self.assertEqual(self.observe(1.2,None,0),(16,-7))
        self.assertEqual(self.observe(1.31,None,0),(0,0))

    def test_gap_distance_stops_before_timeout(self):
        self.observe(1.,-.1)
        self.c.set_pose((self.c.cfg['gap_max_distance']+.01,0,0),1.2)
        self.assertEqual(self.observe(1.2,None,0),(0,0))
        self.assertEqual(self.c.reason,'gap_limit_wait_for_lane')

    def test_stale_camera_stops_and_fresh_path_resumes(self):
        self.observe(1.,-.1)
        self.assertEqual(self.c.tick(2.),(0,0.))
        self.assertEqual(self.observe(2.05,.1),(20,7))

    def test_no_initial_lane_stops(self):
        self.assertEqual(self.observe(1.,None,0),(0,0))
        self.assertEqual(self.observe(1.1,.1),(20,7))


if __name__=='__main__':unittest.main()
