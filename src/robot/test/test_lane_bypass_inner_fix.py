"""Bounded lane corrections and the requested two/six-second bypass."""
import os
import unittest
from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.master.controller import Controller
from test_continuous_obstacles import _SyntheticScan


class LaneBypassInnerFixTests(unittest.TestCase):
    def setUp(self):
        cfg = load_config(os.path.join(os.path.dirname(__file__), '../config'))
        cfg.update(wait_green=False, lidar_enabled=False,
                   lane_curvature_preview=True, steering_command_scale_rad=.03,
                   timed_bypass_speed_raw=20)
        self.c = Controller(cfg)
        self.addCleanup(self.c.close)

    def command(self, points, stamp, confidence=.9):
        self.c.observe_lane(points, confidence, stamp)
        speed, steer = self.c.tick(stamp)
        return speed, encode_command(speed, steer, self.c.cfg, 0)['steering_raw']

    def enter_bend(self):
        self.command([(.5,0),(.7,-.04),(.9,-.12),(1.1,-.24)], 1.)

    def test_fresh_smaller_right_offset_reduces_lock_angle(self):
        self.enter_bend()
        before = encode_command(0,self.c.gap_steer,self.c.cfg,0)['steering_raw']
        speed, raw = self.command([(.5,.12),(.7,.04),(.9,-.02)], 1.1)
        self.assertGreater(speed, 0)
        self.assertGreater(raw,before)
        self.assertLessEqual(abs(raw-before),9)
        self.assertIsNone(self.c.lane_curve_lock)
        self.assertEqual(self.command([], 1.2)[1], raw)

    def test_repeated_near_left_path_corrects_inward_drift(self):
        self.enter_bend()
        path = [(.6,.09),(.7,.08),(.8,.06),(.9,.03)]
        self.command(path, 1.1)
        # Correction progresses with elapsed control time, without a lock.
        for stamp in (1.2,1.3,1.4,1.5):
            speed, raw = self.command(path, stamp)
        self.assertGreater(speed, 0)
        self.assertGreater(raw, 0)
        self.assertIsNone(self.c.lane_curve_lock)

    def test_current_short_span_after_handoff_continues_measured_tracking(self):
        _, raw = self.command([(.5,.03),(.7,.03),(.9,.03)], 1.)
        start = self.c.gap_start
        speed, held = self.command([(.5,.03),(.62,.03)], 1.1, .385)
        self.assertGreater(speed, 0)
        self.assertLessEqual(abs(held-raw),9)
        self.assertGreater(self.c.gap_start, start)
        self.assertGreater(self.command([(.5,.03),(.62,.03)], 1.3, .385)[0], 0)

    def test_far_other_lane_cannot_release_right_bend(self):
        self.c.cfg['lane_curvature_preview'] = False  # legacy guard only
        self.enter_bend()
        path = [(.77,.29),(.85,.28),(.98,.27),(1.05,.25)]
        for stamp in (1.1,1.21,1.32):
            self.assertEqual(self.command(path, stamp)[1], -22)
            self.assertIsNotNone(self.c.lane_curve_lock)

    def test_empty_frame_holds_angle_then_center_correction_resumes(self):
        self.enter_bend()
        path = [(.6,.09),(.7,.08),(.8,.06),(.9,.03)]
        raw = self.command(path, 1.1)[1]
        self.assertEqual(self.command([], 1.15)[1],raw)
        for stamp in (1.21,1.3,1.4,1.5):
            speed,raw = self.command(path,stamp)
        self.assertGreater(speed,0)
        self.assertGreater(raw,0)

    def test_requested_two_five_seconds_then_live_lane(self):
        self.assertEqual(self.c.cfg['timed_bypass_left_s'], 2.)
        self.assertEqual(self.c.cfg['timed_bypass_right_s'], 5.)
        self.c.cfg.update(lidar_enabled=True, timed_bypass_enabled=True)
        for now in (.6,.8,1.):
            self.c.scan = _SyntheticScan(now, obstacles=[(.6,0)])
            self.command([(.5,0),(.7,0),(.9,0),(1.1,0)], now)
        first = {}
        for i in range(1,186):
            now = 1.+i*.05
            self.c.scan = _SyntheticScan(now)
            speed, raw = self.command([(.5,0),(.7,0),(.9,0),(1.1,0)], now)
            phase = self.c.timed_bypass['phase'] if self.c.timed_bypass else 'LANE'
            if phase == 'LANE' and phase not in first:
                self.assertEqual(speed,12)
                self.assertLessEqual(abs(raw+22),5)  # bounded steering handoff
            first.setdefault(phase, now)
            if phase == 'LEFT': self.assertEqual((speed,raw), (20,22))
            if phase == 'RIGHT': self.assertEqual((speed,raw), (20,-22))
        self.assertAlmostEqual(first['RIGHT']-first['LEFT'], 2.)
        self.assertAlmostEqual(first['LANE']-first['RIGHT'], 5.)
        self.assertGreaterEqual(speed,12)
        self.assertLessEqual(speed,20)
        self.assertEqual(raw,0)


if __name__ == '__main__':
    unittest.main()
