"""Road approach uses the normal lane controller without ROS or actuators."""
import json
import os
import unittest
from robot.common.config import load_config
from robot.parallel_parking.lane_follow import LaneFollower


class LaneFollowTests(unittest.TestCase):
    def follower(self):
        cfg=load_config(os.path.join(os.path.dirname(os.path.dirname(__file__)),'config'))
        return LaneFollower(cfg,15)

    def observe(self,follower,y,stamp=10.):
        follower.observe(json.dumps(dict(stamp=stamp,frame='base_link',confidence=.95,
            points=[[.2,y],[.6,y],[1.,y]])),stamp)

    def test_left_drift_is_corrected_right_at_speed_15(self):
        follower=self.follower()
        self.observe(follower,-.06)
        speed,steer=follower.command(10.1)
        self.assertEqual(speed,15)
        self.assertLess(steer,0)

    def test_right_drift_is_corrected_left(self):
        follower=self.follower()
        self.observe(follower,.06)
        speed,steer=follower.command(10.1)
        self.assertEqual(speed,15)
        self.assertGreater(steer,0)

    def test_centered_path_is_straight_and_stale_lane_stops(self):
        follower=self.follower()
        self.assertEqual(follower.command(10.)[0],0)
        self.observe(follower,0.)
        self.assertEqual(follower.command(10.1),(15,0))
        self.assertEqual(follower.command(11.)[0],0)

    def test_nonfinite_path_and_wrong_frame_are_rejected(self):
        follower=self.follower()
        for frame,y in [('base_link',float('nan')),('map',0.)]:
            with self.assertRaises(ValueError):
                follower.observe(json.dumps(dict(stamp=10.,frame=frame,confidence=.9,
                    points=[[.2,y],[.6,y]])),10.)

    def test_short_visual_gap_retains_correction_then_stops(self):
        follower=self.follower()
        self.observe(follower,-.06)
        good=follower.command(10.)
        follower.observe(json.dumps(dict(stamp=10.1,frame='base_link',confidence=0.,points=[])),10.1)
        self.assertEqual(follower.command(10.1),good)
        self.assertEqual(follower.command(10.3)[0],0)


if __name__=='__main__':
    unittest.main()
