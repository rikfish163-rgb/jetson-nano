"""Recorded curve entry, frame-based filtering and retained input protection."""
import math
import os
import unittest
from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.master.controller import Controller
from robot.lane.preview import curvature_preview


class LanePreviewTests(unittest.TestCase):
    ENTRY = [(.591923201,-.004454907), (.671884314,.007609418),
             (.751295300,.014575017), (.830439446,.016596653),
             (.909614227,.012993243), (.989110911,.002449286)]

    def setUp(self):
        cfg = load_config(os.path.join(os.path.dirname(__file__), '../config'))
        cfg.update(wait_green=False, lidar_enabled=False, steering_command_scale_rad=.03,
                   lane_curvature_preview=True, lane_curve_speed_raw=12)
        self.c = Controller(cfg)
        self.addCleanup(self.c.close)

    def command(self, points, stamp=1., confidence=.9, now=None):
        self.c.observe_lane(points, confidence, stamp)
        speed, steer = self.c.lane_command(stamp if now is None else now)
        return speed, encode_command(speed, steer, self.c.cfg, 0)['steering_raw']

    def test_recorded_entry_turns_right_before_far_offset_grows(self):
        speed, raw = self.command(self.ENTRY)
        self.assertLessEqual(raw, -4)
        self.assertEqual(speed, 12)

    def test_mirrored_left_curve_is_symmetric(self):
        left = self.command([(x,-y) for x,y in self.ENTRY])[1]
        self.c.resume_lane()
        self.assertEqual(left, -self.command(self.ENTRY, 2.)[1])

    def test_bend_lock_and_gap_slow_down(self):
        command = self.command([(.5,0),(.7,-.04),(.9,-.12),(1.1,-.24)])
        self.assertEqual(command, (12,-22))
        self.assertEqual(self.command([], 1.1), command)

    def test_straight_retains_lane_speed(self):
        self.assertEqual(self.command([(.5,0),(.7,0),(.9,0),(1.1,0)]), (20,0))

    def test_unreliable_and_single_point_do_not_refresh_gap(self):
        self.command(self.ENTRY)
        origin = self.c.gap_start
        self.assertEqual(self.command(self.ENTRY, 1.1, .1)[0], 0)
        self.assertEqual(self.c.gap_start, origin)
        self.assertEqual(self.command([(.7,0)], 1.2)[0], 0)

    def test_stale_frame_stops_speed(self):
        self.command(self.ENTRY)
        self.assertEqual(self.command(self.ENTRY, 1., now=1.6)[0], 0)

    def test_low_confidence_stop_keeps_bend_wheel_angle(self):
        path = [(.5,0),(.7,-.04),(.9,-.12),(1.1,-.24)]
        raw = self.command(path)[1]
        self.assertEqual(self.command(path, 1.1, .1), (0,raw))

    def test_filter_does_not_count_control_ticks_as_observations(self):
        self.command(self.ENTRY)
        before = dict(self.c.lane_preview)
        self.command(self.ENTRY, now=1.05)
        self.assertEqual(self.c.lane_preview, before)

    def test_curvature_jitter_is_reduced_without_holding_opposite_bend(self):
        raw, filtered = [], []
        for i in range(40):
            k = -.8 + (.2 if i%2 else -.2)
            points = [(x, .5*k*(x-.7)**2) for x in (.5,.6,.7,.8,.9,1.)]
            self.command(points, 1.+i*.1)
            raw.append(self.c.lane_preview['measured_curvature'])
            filtered.append(self.c.lane_preview['curvature'])
        def variance(values):
            mean = sum(values)/len(values)
            return sum((v-mean)**2 for v in values)/len(values)
        self.assertLess(variance(filtered[10:]), .5*variance(raw[10:]))
        # A real sign change must be admitted promptly, not filtered forever.
        for i in range(5):
            self.command([(x,.4*(x-.7)**2) for x in (.5,.6,.7,.8,.9,1.)], 5.+i*.1)
        self.assertGreater(self.c.lane_preview['curvature'], .35)

    def test_long_source_gap_reinitializes_filter(self):
        previous = curvature_preview(self.ENTRY,1.,None,.5)
        mirror = [(x,-y) for x,y in self.ENTRY]
        current = curvature_preview(mirror,2.,previous,.5)
        self.assertAlmostEqual(current['curvature'],current['measured_curvature'])

    def test_degenerate_and_inconsistent_paths_do_not_invent_curve(self):
        self.assertIsNone(curvature_preview([(.5,0),(.5,.01),(.9,0),(.9,.01)],1.,None,.5))
        self.assertIsNone(curvature_preview([(.5,0),(.6,.3),(.7,-.3),(.8,0),(.9,0)],1.,None,.5))
