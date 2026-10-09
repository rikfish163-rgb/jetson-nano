#!/usr/bin/env python2
from __future__ import division
import os
import sys
import unittest
import cv2
import numpy as np
ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))
from parking_line_stop_core import StopRun, LineVision
from parking_line_stop_test import arguments
from parking_lane_curve import boundary_shape, both_boundary_curves, LaneCurveVision
from robot.parallel_parking.reference_vision import load_reference_config


class StopTests(unittest.TestCase):
    def ready(self):
        task = StopRun()
        task.start(0.)
        task.observe(2, 100., 0.)
        task.observe(1, 100.1, .1)
        return task

    def test_all_lines_must_disappear(self):
        task = self.ready()
        for t in (.2, .4, .6):
            task.observe(1, 100+t, t)
            self.assertEqual((30, -5), task.tick(t, 100+t))

    def test_confirmed_disappearance_latches_stop(self):
        task = self.ready()
        for t in (.2, .35, .51):
            task.observe(0, 100+t, t, True)
        self.assertEqual((0, 0), task.tick(.51, 100.51))
        self.assertEqual('BAY_END_CONFIRMED', task.reason)
        task.observe(2, 100.7, .7)
        self.assertEqual((0, 0), task.tick(.7, 100.7))

    def test_first_joint_candidate_brakes_and_noise_cannot_resume(self):
        task = self.ready()
        for t, n in ((.2, 0), (.35, 4), (.4, 3), (.5, 6), (.65, 2)):
            task.observe(n, 100+t, t, True)
            self.assertEqual((0, 0), task.tick(t, 100+t))
        self.assertFalse(task.finished)
        self.assertEqual('STOP_VERIFY_END', task.reason)

    def test_blank_from_start_never_claims_completion(self):
        task = StopRun()
        task.start(0.)
        for t in np.arange(0., 5.1, .1):
            task.observe(0, 100+t, t)
        self.assertEqual((0, 0), task.tick(5.1, 105.1))
        self.assertEqual('NO_LINES_SEEN', task.reason)

    def test_repeated_seen_frame_does_not_arm(self):
        task = StopRun()
        task.start(0.)
        for t in (0., .1, .2):
            task.observe(1, 100., t)
        self.assertFalse(task.seen)
        self.assertEqual((30, -5), task.tick(.2, 100.2))

    def test_repeated_blank_frame_cannot_confirm_end(self):
        task = self.ready()
        task.observe(0, 100.2, .2, True)
        task.observe(0, 100.2, .4, True)
        self.assertEqual(1, task.missing_count)
        self.assertEqual((0, 0), task.tick(.2, 100.2))
        self.assertEqual('STOP_VERIFY_END', task.reason)
        self.assertEqual((0, 0), task.tick(.51, 100.51))
        self.assertFalse(task.finished)

    def test_countdown_seen_then_blank_straight_road_keeps_moving(self):
        task = StopRun()
        task.observe(1, 100., 0.)
        task.observe(2, 100.1, .1)
        self.assertTrue(task.seen)
        self.assertEqual((0, 0), task.tick(.1, 100.1))
        task.start(.2)
        task.observe(0, 100.2, .2)
        self.assertEqual((30, -5), task.tick(.2, 100.2))
        self.assertEqual('FORWARD_WAIT_BOTH_CURVES', task.reason)

    def test_no_lines_on_straight_does_not_complete(self):
        task = self.ready()
        for t in (.2, .4, .6, .8):
            task.observe(0, 100+t, t, False)
            self.assertEqual((30, -5), task.tick(t, 100+t))
        self.assertFalse(task.finished)

    def test_curve_evidence_loss_resets_confirmation(self):
        task = self.ready()
        for t, curved in ((.2, True), (.4, True), (.5, False), (.6, True), (.8, True)):
            task.observe(0, 100+t, t, curved)
            self.assertEqual((0, 0), task.tick(t, 100+t))
        self.assertFalse(task.finished)
        task.observe(0, 101., 1., True)
        self.assertEqual((0, 0), task.tick(1., 101.))

    def test_ambiguous_end_exits_stopped_without_claiming_confirmation(self):
        task = self.ready()
        task.observe(0, 100.2, .2, True)
        self.assertEqual((0, 0), task.tick(.2, 100.2))
        for t in (.4, .6, .8, 1., 1.2, 1.4, 1.6, 1.8, 2., 2.3):
            task.observe(2, 100+t, t, False)
            self.assertEqual((0, 0), task.tick(t, 100+t))
        self.assertTrue(task.finished)
        self.assertEqual('END_UNCONFIRMED_STOPPED', task.reason)

    def test_real_log_alternating_zero_one_on_curve_brakes_at_first_zero(self):
        task = self.ready()
        for t, count, curved in ((.2, 1, False), (.4, 0, False), (.6, 1, True)):
            task.observe(count, 100+t, t, curved)
            self.assertEqual((30, -5), task.tick(t, 100+t))
        for t, count in ((.8, 0), (1., 1), (1.2, 0), (1.4, 1), (1.6, 0), (1.8, 2)):
            task.observe(count, 100+t, t, True)
            self.assertEqual((0, 0), task.tick(t, 100+t))
        self.assertFalse(task.finished)

    def test_countdown_blank_does_not_latch(self):
        task = StopRun()
        task.observe(1, 100., 0.)
        task.observe(1, 100.1, .1)
        task.observe(0, 100.2, .2)
        self.assertIsNone(task.missing_since)
        self.assertFalse(task.seen)

    def test_steering_override_and_zero_on_abort(self):
        task = StopRun(steering=-6)
        task.start(0.)
        task.observe(1, 100., 0.)
        self.assertEqual((30, -6), task.tick(0., 100.))
        self.assertEqual((0, 0), task.abort('operator'))
        self.assertEqual(-3, arguments([]).steering)
        self.assertEqual(-6, arguments(['--steering', '-6']).steering)

    def test_camera_dropout_is_fault_not_success(self):
        task = self.ready()
        self.assertEqual((0, 0), task.tick(.7, 100.7))
        self.assertEqual('CAMERA_TIMEOUT', task.reason)

    def test_old_capture_stamp_stops_even_with_fresh_processing(self):
        task = self.ready()
        task.observe(1, 100.2, .8)
        self.assertEqual((0, 0), task.tick(.8, 100.8))

    def test_deadline(self):
        task = self.ready()
        task.observe(1, 120., 20.)
        self.assertEqual((0, 0), task.tick(20., 120.))
        self.assertEqual('MAX_RUN_TIMEOUT', task.reason)

    def test_single_seen_frame_does_not_arm(self):
        task = StopRun()
        task.start(0.)
        task.observe(1, 100., 0.)
        for t in (.1, .3, .5):
            task.observe(0, 100+t, t)
        self.assertFalse(task.seen)
        self.assertEqual((30, -5), task.tick(.5, 100.5))

    def test_invalid_parameters(self):
        for argv in (['--speed', '31'], ['--lost-seconds', 'nan'],
                     ['--max-seconds', '2'], ['--lost-frames', '1'], ['--steering', '-23'],
                     ['--end-verify-seconds', '.5', '--lost-seconds', '.5']):
            self.assertRaises(ValueError, arguments, argv)


class CurveTests(unittest.TestCase):
    def points(self, side=.3, a=0., slope=0.):
        return [[x, side+slope*(x-.7)+a*(x-.7)**2] for x in np.linspace(.5, 1.2, 12)]

    def test_slanted_straights_are_not_curves(self):
        result = both_boundary_curves(dict(LEFT=self.points(.3, slope=.35), RIGHT=self.points(-.3, slope=.35)))
        self.assertFalse(result['both_curved'])
        self.assertEqual('STRAIGHT', result['left']['kind'])

    def test_both_left_and_right_bends(self):
        for a in (-1., 1.):
            result = both_boundary_curves(dict(LEFT=self.points(.3, a), RIGHT=self.points(-.3, a)))
            self.assertTrue(result['both_curved'])

    def test_one_side_missing_or_straight_does_not_count(self):
        for other in ([], self.points(-.3)):
            result = both_boundary_curves(dict(LEFT=self.points(.3, 1.), RIGHT=other))
            self.assertFalse(result['both_curved'])

    def test_short_or_noisy_geometry_unknown(self):
        self.assertEqual('UNKNOWN', boundary_shape(self.points()[:3])['kind'])
        points = self.points(.3, 1.)
        points[5][1] += .25
        self.assertEqual('UNKNOWN', boundary_shape(points)['kind'])

    def test_opposite_bends_rejected(self):
        self.assertFalse(both_boundary_curves(dict(LEFT=self.points(.3, 1.), RIGHT=self.points(-.3, -1.)))['both_curved'])

    def test_parking_rail_width_is_not_lane_pair(self):
        self.assertFalse(both_boundary_curves(dict(LEFT=self.points(.1, 1.), RIGHT=self.points(-.1, 1.)))['both_curved'])


class VisionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cfg = load_reference_config(os.path.join(ROOT, 'src/robot/config/competition.yaml'),
                                    os.path.join(ROOT, 'src/robot/parallel_parking/reference_camera.yaml'))
        cls.vision = LineVision(cfg)

    def mask(self, lines):
        c = self.vision.camera
        mask = np.zeros((int(c['origin_v']), c['bev_width']+720), np.uint8)
        for a, b in lines:
            points = [(int(c['origin_u']-y*c['pixels_per_m']), int(c['origin_v']-x*c['pixels_per_m'])) for x, y in (a, b)]
            cv2.line(mask, points[0], points[1], 255, 3)
        return mask

    def test_three_then_one_then_none(self):
        lines = [[(x, -.25), (x, -.65)] for x in (.6, 1.2, 1.8)]
        self.assertEqual(3, len(self.vision.segments(self.mask(lines))))
        self.assertEqual(1, len(self.vision.segments(self.mask(lines[:1]))))
        self.assertEqual(0, len(self.vision.segments(self.mask([]))))

    def test_longitudinal_lane_and_other_side_ignored(self):
        mask = self.mask([[(.4, -.25), (2., -.25)], [(1., .25), (1., .65)]])
        self.assertEqual([], self.vision.segments(mask))

    def test_slanted_transverse_line(self):
        mask = self.mask([[(1., -.25), (1.10, -.65)]])
        self.assertTrue(self.vision.segments(mask))

    def test_near_partial_line_is_still_seen(self):
        mask = self.mask([[(.15, -.25), (.15, -.38)]])
        self.assertTrue(self.vision.segments(mask))

    def test_full_image_pipeline(self):
        count, bev = self.vision.observe(np.zeros((360, 640, 3), np.uint8))
        self.assertEqual(0, count)
        self.assertEqual(3, bev.shape[2])

    def test_short_floor_fragments_do_not_acquire(self):
        self.vision.history = []
        self.assertEqual([], self.vision.filter_lines([[(1., -.25), (1., -.34)]]))

    def test_confirmed_line_can_shrink_toward_camera(self):
        self.vision.history = []
        self.assertTrue(self.vision.filter_lines([[(1., -.25), (1., -.65)]]))
        self.assertTrue(self.vision.filter_lines([[ (.9, -.25), (.9, -.34)]]))
        self.assertEqual([], self.vision.filter_lines([]))

    def test_unrelated_short_fragment_is_not_previous_line(self):
        self.vision.history = []
        self.vision.filter_lines([[(1., -.25), (1., -.65)]])
        self.assertEqual([], self.vision.filter_lines([[(2., -.75), (2., -.84)]]))

    def test_thin_paint_supported_but_wide_patch_rejected(self):
        line = [(1., -.25), (1., -.65)]
        self.assertTrue(self.vision.paint_supported(line, self.mask([line])))
        wide = cv2.dilate(self.mask([line]), np.ones((25, 25), np.uint8))
        self.assertFalse(self.vision.paint_supported(line, wide))

    def test_actual_camera_bay_line_is_detected(self):
        self.vision.history = []
        frame = cv2.imread(os.path.join(ROOT, 'tools/testdata/parking_line_stop/bay_visible.jpg'))
        self.assertIsNotNone(frame)
        self.assertGreater(self.vision.observe(frame)[0], 0)

    def test_actual_camera_curve_fragments_do_not_acquire(self):
        self.vision.history = []
        frame = cv2.imread(os.path.join(ROOT, 'tools/testdata/parking_line_stop/no_bay.jpg'))
        self.assertIsNotNone(frame)
        self.assertEqual(0, self.vision.observe(frame)[0])


if __name__ == '__main__':
    unittest.main()
