# -*- coding: utf-8 -*-
"""No-motion regression tests, including actual synthetic OpenCV masks."""
from __future__ import division
import os
import sys
import unittest
import math
import numpy as np
import cv2

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))
from robot.common.config import read_mapping
from p4_forward_core import ForwardRun, validate
from p4_forward_vision import BayVision, geometry


def config():
    return validate(read_mapping(os.path.join(ROOT, 'src/robot/config/p4_forward_test.yaml')))


def scene(row=.65, bottom=True, sides=True, y=.04, heading=.03):
    return dict(sides_valid=sides, bottom_valid=bottom, center_y_m=y,
                bottom_y_m=y, bottom_row=row, heading_rad=heading,
                heading_deg=math.degrees(heading), image_height=360, image_ok=True, visible_side_m=.3)


class StateTests(unittest.TestCase):
    def setUp(self):
        self.cfg = config()
        self.run = ForwardRun(self.cfg)
        self.t = 0.
        self.run.start(self.t)

    def feed(self, value=None, step=.05):
        self.t += step
        self.run.observe_scene(scene() if value is None else value)
        self.run.observe(3, self.t, self.t)
        return self.run.tick(self.t, self.t)

    def search(self):
        while self.run.phase == 'PREPARE':
            self.feed()
        self.assertEqual(self.run.phase, 'SEARCH')

    def align(self):
        self.search()
        for i in range(3):
            command = self.feed()
        self.assertEqual(self.run.phase, 'SIDE_ALIGN')
        for i in range(3):
            command = self.feed()
        self.assertEqual(self.run.phase, 'ALIGN')
        self.assertEqual(command[0], 15)

    def near(self):
        self.align()
        for row in (.80, .87, .89):
            self.feed(scene(row=row))
        self.assertTrue(self.run.near_armed)

    def test_full_flow_and_frozen_steering(self):
        for i in range(39):
            self.assertEqual(self.feed(), (30, 0))
        self.assertEqual(self.feed(), (0, 0))
        for i in range(2):
            self.assertEqual(self.feed(), (0, 0))
        self.assertEqual(self.feed()[0], 15)
        self.assertEqual(self.run.phase, 'SIDE_ALIGN')
        for unused in range(3):
            self.feed()
        for row in (.8, .87, .89):
            self.feed(scene(row=row))
        frozen = self.run.last_steering
        for i in range(2):
            self.assertEqual(self.feed(scene(bottom=False), .1), (0, 0))
        self.assertEqual(self.feed(scene(bottom=False), .1), (30, frozen))
        final_at = self.t
        for unused in range(19):
            self.assertEqual(self.feed(scene(bottom=False, sides=False)), (30, frozen))
        self.assertEqual(self.feed(scene(bottom=False, sides=False)), (0, 0))
        self.assertEqual(self.run.reason, 'P4_COMPLETE')
        self.assertAlmostEqual(self.t-final_at, 1., places=5)
        self.assertEqual(self.feed(), (0, 0))

    def test_no_lines_times_out_stopped(self):
        self.search()
        while not self.run.finished:
            self.assertEqual(self.feed(scene(bottom=False, sides=False)), (0, 0))
        self.assertEqual(self.run.reason, 'SEARCH_TIMEOUT')

    def side_align(self):
        self.search()
        sides = scene(bottom=False)
        sides['bottom_y_m'] = None
        sides['bottom_row'] = None
        for unused in range(3):
            command = self.feed(sides)
        self.assertEqual(self.run.phase, 'SIDE_ALIGN')
        self.assertEqual(command[0], 15)
        return sides

    def test_sides_guide_before_bottom_and_then_confirm_switch(self):
        sides = self.side_align()
        for unused in range(12):
            speed, angle = self.feed(sides)
            self.assertEqual(speed, 15)
            self.assertEqual(angle, self.run.steering(sides, use_bottom=False))
        bottom = scene()
        bottom['bottom_y_m'] = -.08
        for unused in range(2):
            self.feed(bottom)
            self.assertEqual(self.run.phase, 'SIDE_ALIGN')
        self.feed(bottom)
        self.assertEqual(self.run.phase, 'ALIGN')
        self.assertEqual(self.run.last_steering, self.run.steering(bottom))
        self.assertNotEqual(self.run.last_steering, self.run.steering(bottom, use_bottom=False))
        self.assertFalse(self.run.near_armed)

    def test_unconfirmed_bottom_loss_keeps_side_guidance(self):
        sides = self.side_align()
        self.feed(scene())
        for unused in range(8):
            self.run.observe(3, self.t, self.t)
        self.assertEqual(self.run.bottom_seen_count, 1)
        self.assertEqual(self.run.phase, 'SIDE_ALIGN')
        self.assertEqual(self.feed(sides)[0], 15)
        self.assertEqual(self.run.bottom_seen_count, 0)
        for unused in range(2):
            self.feed(scene())
        self.assertEqual(self.run.phase, 'SIDE_ALIGN')
        self.feed(scene())
        self.assertEqual(self.run.phase, 'ALIGN')

    def test_side_guidance_loss_jump_and_timeout_stop(self):
        self.side_align()
        self.assertEqual(self.feed(scene(bottom=False, sides=False)), (0, 0))
        self.assertEqual(self.run.reason, 'SIDE_GEOMETRY_LOST_OR_TARGET_JUMP')
        self.setUp()
        self.side_align()
        self.assertEqual(self.feed(scene(bottom=False, y=.42)), (0, 0))
        self.assertEqual(self.run.reason, 'SIDE_GEOMETRY_LOST_OR_TARGET_JUMP')
        self.setUp()
        sides = self.side_align()
        while not self.run.finished:
            self.feed(sides)
        self.assertEqual(self.run.reason, 'SIDE_ALIGN_TIMEOUT')
        self.assertFalse(self.run.near_armed)

    def test_early_bottom_loss_never_triggers_final(self):
        self.align()
        self.assertEqual(self.feed(scene(bottom=False)), (0, 0))
        self.assertEqual(self.run.reason, 'BOTTOM_LOST_BEFORE_IMAGE_EDGE')

    def test_duplicate_frame_cannot_confirm_acquisition(self):
        self.search()
        self.feed()
        for unused in range(10):
            self.run.observe(3, self.t, self.t)
        self.assertEqual(self.run.seen_count, 1)
        self.assertEqual(self.run.phase, 'SEARCH')

    def test_duplicate_frame_cannot_confirm_disappearance(self):
        self.near()
        self.feed(scene(bottom=False))
        for unused in range(10):
            self.run.observe(2, self.t, self.t)
        self.assertEqual(self.run.missing_count, 1)
        self.assertEqual(self.run.phase, 'VERIFY')

    def test_bottom_reappearance_stops(self):
        self.near()
        self.feed(scene(bottom=False))
        self.assertEqual(self.feed(), (0, 0))
        self.assertEqual(self.run.reason, 'BOTTOM_REAPPEARED_DURING_CONFIRMATION')

    def test_entire_u_may_leave_image_after_near_edge_motion(self):
        self.near()
        for unused in range(2):
            self.assertEqual(self.feed(scene(bottom=False, sides=False), .1), (0, 0))
        self.assertEqual(self.feed(scene(bottom=False, sides=False), .1)[0], 30)
        self.assertEqual(self.run.phase, 'FINAL')

    def test_visible_bottom_with_recent_short_rails_is_not_disappearance(self):
        self.near()
        frozen = self.run.last_steering
        partial = scene(row=.95, sides=False)
        partial['tracking_bottom'] = True
        self.assertEqual(self.feed(partial), (15, frozen))
        self.assertEqual(self.run.phase, 'ALIGN')
        for unused in range(11):
            self.feed(partial)
        self.assertTrue(self.run.finished)
        self.assertEqual(self.run.reason, 'GEOMETRY_LOST_OR_TARGET_JUMP')

    def test_short_rails_cannot_acquire_initial_bay(self):
        self.search()
        short = scene()
        short['visible_side_m'] = .025
        for unused in range(5):
            self.assertEqual(self.feed(short), (0, 0))
        self.assertEqual(self.run.phase, 'SEARCH')

    def test_side_loss_and_adjacent_target_stop(self):
        self.align()
        self.assertEqual(self.feed(scene(sides=False)), (0, 0))
        self.setUp()
        self.align()
        self.assertEqual(self.feed(scene(y=.42)), (0, 0))
        self.assertEqual(self.run.reason, 'GEOMETRY_LOST_OR_TARGET_JUMP')

    def test_no_edge_motion_cannot_arm(self):
        self.search()
        for unused in range(8):
            self.feed(scene(row=.9))
        self.assertFalse(self.run.near_armed)
        self.assertEqual(self.feed(scene(bottom=False)), (0, 0))

    def test_control_gap_and_stale_camera_abort(self):
        self.feed(step=.3)
        self.assertEqual(self.run.reason, 'CONTROL_GAP')
        self.setUp()
        self.feed()
        self.run.stamp = -10
        self.assertEqual(self.run.tick(self.t+.05, self.t+.05), (0, 0))
        self.assertEqual(self.run.reason, 'CAMERA_TIMEOUT')

    def test_bad_image_aborts_even_final(self):
        self.run.phase = 'FINAL'
        bad = scene(bottom=False, sides=False)
        bad['image_ok'] = False
        self.assertEqual(self.feed(bad), (0, 0))
        self.assertEqual(self.run.reason, 'IMAGE_INVALID')

    def test_steering_direction_saturation(self):
        self.assertGreater(self.run.steering(scene(y=.1, heading=0)), 0)
        self.assertLess(self.run.steering(scene(y=-.1, heading=0)), 0)
        self.assertEqual(self.run.steering(scene(y=2, heading=1)), 22)

    def test_invalid_config_rejected(self):
        for key, value in [('final_s', 10), ('seen_frames', 1), ('lost_frames', 1),
                           ('prepare_speed_raw', float('nan')), ('align_speed_raw', True)]:
            bad = dict(self.cfg)
            bad[key] = value
            with self.assertRaises(ValueError):
                validate(bad)


class VisionTests(unittest.TestCase):
    def setUp(self):
        class Args(object):
            p4_config = config()
        self.vision = BayVision({}, Args())
        self.cfg = config()

    def mask(self, bottom=True, second=True, width=.38, cross=False, end_x=.85):
        c = self.vision.camera
        mask = np.zeros((int(c['origin_v']), c['bev_width']+720), np.uint8)
        def draw(a, b):
            def pixel(p):
                return (int(round(c['origin_u']-p[1]*c['pixels_per_m'])),
                        int(round(c['origin_v']-p[0]*c['pixels_per_m'])))
            cv2.line(mask, pixel(a), pixel(b), 255, 3)
        draw((end_x-.45, -width/2), (end_x, -width/2))
        if second:
            draw((end_x-.45, width/2), (end_x, width/2))
        if bottom:
            x = end_x-.25 if cross else end_x
            draw((x, -width/2), (x, width/2))
        return mask

    def analyze(self, **kwargs):
        mask = self.mask(**kwargs)
        return self.vision.analyze_mask(mask, np.ones(mask.shape, np.uint8), 360)

    def test_synthetic_u_detected_and_projected(self):
        data = self.analyze()
        self.assertTrue(data['sides_valid'])
        self.assertTrue(data['bottom_valid'])
        self.assertAlmostEqual(data['bottom_midpoint_m'][0], .85, delta=.035)
        self.assertAlmostEqual(data['bottom_midpoint_m'][1], 0, delta=.02)
        self.assertTrue(0 < data['bottom_row'] < 1)

    def test_missing_bottom_not_inferred(self):
        data = self.analyze(bottom=False)
        self.assertTrue(data['sides_valid'])
        self.assertFalse(data['bottom_valid'])

    def test_adjacent_bottom_does_not_replace_centered_sides(self):
        mask = self.mask(bottom=False)
        c = self.vision.camera
        def pixel(x, y):
            return (int(round(c['origin_u']-y*c['pixels_per_m'])),
                    int(round(c['origin_v']-x*c['pixels_per_m'])))
        cv2.line(mask, pixel(.4, .57), pixel(.85, .57), 255, 3)
        cv2.line(mask, pixel(.85, .19), pixel(.85, .57), 255, 3)
        data = self.vision.analyze_mask(mask, np.ones(mask.shape, np.uint8), 360)
        self.assertTrue(data['sides_valid'])
        self.assertAlmostEqual(data['center_y_m'], 0, delta=.03)
        self.assertFalse(data['bottom_valid'])

    def test_one_rail_wrong_width_and_middle_cross_rejected(self):
        self.assertFalse(self.analyze(second=False)['sides_valid'])
        self.assertFalse(self.analyze(width=.7)['sides_valid'])
        self.assertFalse(self.analyze(cross=True)['bottom_valid'])

    def test_invalid_ground_cannot_be_missing_bottom_evidence(self):
        mask = self.mask()
        data = self.vision.analyze_mask(mask, np.zeros(mask.shape, np.uint8), 360)
        self.assertFalse(data['sides_valid'])

    def test_real_camera_adapter_black_frame_rejected(self):
        count, debug = self.vision.observe(np.zeros((360, 640, 3), np.uint8))
        self.assertEqual(count, 0)
        self.assertFalse(self.vision.scene_data()['image_ok'])

    def test_calibrated_image_edge_preserves_visible_bottom(self):
        self.vision.detector.bev(np.zeros((360, 640, 3), np.uint8), parking=True)
        valid = self.vision.detector.parking_valid
        near = []
        for x in (.85, .75, .68, .64, .62, .60, .59, .58, .57, .56):
            data = self.vision.analyze_mask(self.mask(end_x=x), valid, 360)
            self.assertTrue(data['bottom_valid'], 'lost OBSERVED bottom early at x=%.2f: %s' % (x, data))
            if data['bottom_row'] >= self.cfg['bottom_near_row']:
                near.append(data['bottom_row'])
        self.assertGreaterEqual(len(near), 2)
        data = self.vision.analyze_mask(self.mask(end_x=.48), valid, 360)
        self.assertFalse(data['bottom_valid'])

    def test_rotated_u_heading_and_center(self):
        theta = .15
        rotation = np.array([[math.cos(theta), -math.sin(theta)],
                             [math.sin(theta), math.cos(theta)]])
        points = [np.dot(rotation, p)+[0, .04] for p in
                  ([.4, -.19], [.85, -.19], [.4, .19], [.85, .19])]
        data = geometry([(points[0], points[1]), (points[2], points[3]), (points[1], points[3])],
                        self.cfg, lambda a, b: 1.)
        self.assertEqual(len(data), 1)
        self.assertTrue(data[0]['bottom_valid'])
        self.assertAlmostEqual(data[0]['heading_rad'], theta)

    def test_measured_geometry_drawn_on_front_image(self):
        data = self.analyze()
        self.assertEqual(len(data['bottom_line_m']), 2)
        self.vision.data = data
        frame = np.zeros((360, 640, 3), np.uint8)
        overlay = self.vision.overlay(frame)
        self.assertEqual(overlay.shape, frame.shape)
        self.assertEqual(int(frame.sum()), 0)
        self.assertGreater(int(np.all(overlay == [0, 255, 0], axis=2).sum()), 20)
        self.assertGreater(int(np.all(overlay == [0, 0, 255], axis=2).sum()), 20)
        self.vision.data = dict(sides_valid=False, bottom_valid=False)
        missing = self.vision.overlay(frame)
        self.assertEqual(int(np.all(missing == [0, 255, 0], axis=2).sum()), 0)
        self.assertEqual(int(np.all(missing == [0, 0, 255], axis=2).sum()), 0)


class EntryTests(unittest.TestCase):
    def test_preview_never_calls_ros_runner(self):
        import p4_forward_test as entry
        old = entry.shared.run
        def unexpected(*args, **kwargs):
            raise AssertionError('preview called ROS runner')
        entry.shared.run = unexpected
        try:
            self.assertEqual(entry.main([]), 0)
        finally:
            entry.shared.run = old


if __name__ == '__main__':
    unittest.main()
