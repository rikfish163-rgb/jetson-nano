"""A temporarily missing reference must not terminate the camera timer."""
import imp
import json
import os
import time
import unittest

import numpy as np
import rospy

from test_lane_transport import Sink


class ProcessingRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.camera = imp.load_source('lane_processing_recovery_test', os.path.join(
            os.path.dirname(__file__), '..', 'scripts', 'camera_yihan_web.py'))
        self.warnings = []
        original = rospy.logwarn_throttle
        rospy.logwarn_throttle = lambda period, text: self.warnings.append(text)
        self.addCleanup(setattr, rospy, 'logwarn_throttle', original)

    def point(self, x, y):
        return dict(x=float(x), y=float(y), observed=True, usable=True,
                    geometry_ok=True, score=.9)

    def message(self, stamp):
        msg = self.camera.bridge.cv2_to_imgmsg(
            np.zeros((360, 640, 3), np.uint8), encoding='bgr8')
        msg.header.stamp = rospy.Time.from_sec(stamp)
        return msg

    def test_boundary_export_filters_empty_windows_before_sorting(self):
        camera = self.camera
        msg = camera.build_left_boundary_message(
            [None, self.point(120, 220), None, self.point(120, 380)],
            rospy.Time.from_sec(1.))
        self.assertEqual(len(msg.poses), 2)
        self.assertLess(msg.poses[0].pose.position.x, msg.poses[1].pose.position.x)
        empty = camera.build_left_boundary_message([None] * camera.NUM_WINDOWS,
                                                  rospy.Time.from_sec(2.))
        self.assertEqual(empty.poses, [])

    def test_debug_drawing_does_not_connect_across_empty_windows(self):
        camera = self.camera
        canvas = np.zeros((400, 480, 3), np.uint8)
        camera.draw_boundary_debug(canvas,
            [self.point(120, 380), None, self.point(120, 220)], (255, 80, 0))
        self.assertEqual(int(canvas[300, 120].sum()), 0)
        self.assertGreater(int(canvas[380, 120].sum()), 0)

    def test_full_frame_publishes_pending_reference_then_recovers(self):
        camera = self.camera
        names = ('front_pub', 'white_pub', 'metric_bev_pub', 'lane_tracking_pub',
                 'lane_path_pub', 'lane_confidence_pub', 'lane_observation_pub',
                 'left_boundary_pub')
        for name in names:
            setattr(camera, name, Sink())
        mask = np.zeros((400, 480), np.uint8)
        current = [[self.point(120, 380-40*i) for i in range(7)]]
        camera.make_metric_bev = lambda *args: mask.copy()
        camera.track_boundaries_once = lambda *args: (
            current[0] + [None] * (camera.NUM_WINDOWS-len(current[0])),
            [None] * camera.NUM_WINDOWS)
        camera.find_dashed_left_boundary = lambda *args: None
        camera.last_print_time = time.time()
        sequence = [(1., 120, False), (1.1, 365, True), (1.4, 365, True),
                    (1.6, 120, True), (1.7, 120, True), (1.8, 120, False)]
        for index, (stamp, x, pending) in enumerate(sequence):
            current[0] = [self.point(x, 380-40*i) for i in range(7)]
            camera.process_image(self.message(stamp))
            observation = json.loads(camera.lane_observation_pub.messages[-1].data)
            self.assertAlmostEqual(observation['stamp'], stamp, places=8)
            self.assertEqual(bool(observation['points']), not pending)
            if pending:
                self.assertEqual(observation['diagnostic']['reference_association'],
                                 'pending_reference')
                self.assertEqual(camera.left_boundary_pub.messages[-1].poses, [])
            for name in names:
                self.assertEqual(len(getattr(camera, name).messages), index+1, name)
        self.assertEqual(self.warnings, [])

    def test_bad_frame_keeps_timer_alive_for_next_capture(self):
        camera = self.camera
        calls, errors = [], []
        def process(msg):
            calls.append(msg.header.stamp.to_sec())
            if len(calls) == 1:
                raise TypeError('injected frame failure')
        original = rospy.logerr_throttle
        rospy.logerr_throttle = lambda period, text: errors.append(text)
        self.addCleanup(setattr, rospy, 'logerr_throttle', original)
        camera.process_image = process
        camera.max_image_age = 0.
        for stamp in (1., 2.):
            camera.latest_image_msg = self.message(stamp)
            camera.process_latest_image(None)
            self.assertTrue(camera.processing_lock.acquire(False))
            camera.processing_lock.release()
        self.assertEqual(calls, [1., 2.])
        self.assertEqual(len(errors), 1)
        self.assertIn('injected frame failure', errors[0])


if __name__ == '__main__':
    unittest.main()
