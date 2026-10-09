"""Fresh sparse paint must not deadlock lane following or bypass handoff."""
import os
import unittest
from robot.common.config import load_config
from robot.master.controller import Controller
from test_continuous_obstacles import _SyntheticScan


class PartialLaneContinuityTests(unittest.TestCase):
    def setUp(self):
        cfg = load_config(os.path.join(os.path.dirname(__file__), '../config'))
        cfg.update(wait_green=False, lidar_enabled=False, lane_curvature_preview=True,
                   steering_command_scale_rad=.03, lane_curve_speed_raw=16)
        cfg['speed_raw'].update(lane=24, action=24)
        self.c = Controller(cfg)
        self.addCleanup(self.c.close)

    def command(self, path, stamp, confidence):
        self.c.observe_lane(path, confidence, stamp)
        return self.c.tick(stamp)

    def test_current_short_path_keeps_following_for_twenty_seconds(self):
        self.command([(.5,.02),(.7,.02),(.9,.02)], 1., .9)
        for i in range(1,201):
            speed, steer = self.command([(.6,.04),(.7,.03)], 1.+i*.1, .329595)
            self.assertEqual(speed,16)
            self.assertIsNotNone(self.c.lane_target['target'])
        self.assertEqual(self.c.lane_source,'partial_center')
        self.assertIsNone(self.c.lane_speed_state)

    def test_usable_short_path_resumes_after_unreliable_stop(self):
        self.command([(.5,0),(.7,0),(.9,0)], 1., .9)
        self.assertEqual(self.command([(.5,0)], 1.3, .1)[0],0)
        self.assertEqual(self.command([(.6,.04),(.7,.03)], 2., .329595)[0],16)

    def test_recorded_three_point_low_coverage_path_can_correct(self):
        # Saved image 1790724972.571: paint was present despite score .234.
        path=[(.822614,.266669),(.922614,.286470),(1.022614,.248335)]
        speed,steer=self.command(path,1.,.234)
        self.assertEqual(speed,16)
        self.assertGreater(steer,0)

    def test_recorded_four_dash_short_center_can_resume(self):
        # Normal offset shortens forward span to 6.8--7.4 cm in these views.
        paths=[[(.557247,-.272270),(.631024,-.335298)],
               [(.553937,-.270794),(.622340,-.326663)]]
        for i,path in enumerate(paths):
            speed,steer=self.command(path,1.+i*.1,.336)
            self.assertEqual(speed,16)
            self.assertLess(steer,0)

    def test_incoherent_or_single_points_do_not_refresh_gap_budget(self):
        self.command([(.5,0),(.7,0),(.9,0)], 1., .9)
        for path in ([ (.5,0) ],[(.5,0),(.6,.4),(.7,-.4)]):
            self.assertEqual(self.command(path, 2., .32)[0],0)
        self.assertEqual(self.command([], 6., .1)[0],0)

    def test_partial_path_staleness_still_stops(self):
        self.command([(.6,.04),(.7,.03)], 1., .329595)
        self.assertEqual(self.c.tick(1.6)[0],0)
        self.assertEqual(self.c.reason,'lane_stream_stale')

    def test_bypass_reacquires_recorded_two_point_exit_without_wait(self):
        self.c.cfg.update(lidar_enabled=True,timed_bypass_enabled=True)
        self.c.state='TIMED_BYPASS';self.c.action='BYPASS'
        self.c.timed_bypass=dict(phase='RIGHT',elapsed_s=5.95,last=1.,
            trigger_point=(.8,0),trigger_world=(.8,0))
        self.c.scan=_SyntheticScan(1.05)
        speed,steer=self.command([(.921858,.261068),(1.021858,.330959)],1.05,.38298)
        self.assertEqual(speed,16)
        self.assertIsNone(self.c.timed_bypass)
        self.assertTrue(self.c.timed_bypass_completed)

    def test_real_collision_overrides_partial_tracking(self):
        self.c.cfg.update(lidar_enabled=True,timed_bypass_enabled=False)
        self.c.scan=_SyntheticScan(1.,obstacles=[(.20,0)])
        self.assertEqual(self.command([(.6,.04),(.7,.03)],1.,.329595)[0],0)
        self.assertEqual(self.c.reason,'lidar_obstacle_in_sweep')


if __name__ == '__main__':
    unittest.main()
