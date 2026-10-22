"""Source-frame identity and failure diagnostics without a ROS master."""
import imp
import json
import os
import time
import unittest
import numpy as np
import rospy


class Sink(object):
    def __init__(self):
        self.messages = []
    def get_num_connections(self):
        return 1
    def publish(self, msg):
        self.messages.append(msg)


class TransportTests(unittest.TestCase):
    def setUp(self):
        self.lane = imp.load_source('lane_transport_test', os.path.join(
            os.path.dirname(__file__), '..', 'scripts', 'camera_yihan_web.py'))
        self.lane.init_undistort_maps()

    def test_duplicate_lane_producer_is_rejected_before_start(self):
        self.lane.check_lane_owner({}, '/vision/lane_observation')
        with self.assertRaises(RuntimeError):
            self.lane.check_lane_owner(
                {'/vision/lane_observation': ['/front_lane_preview']},
                '/vision/lane_observation')
        self.lane.check_lane_owner(
            {'/preview/lane': ['/front_lane_preview']}, '/vision/lane_observation')

    def test_all_images_and_control_observation_share_capture_stamp(self):
        lane = self.lane
        names = ('front_pub','white_pub','metric_bev_pub','lane_tracking_pub',
                 'lane_path_pub','lane_confidence_pub','lane_observation_pub','left_boundary_pub')
        for name in names:
            setattr(lane,name,Sink())
        lane.last_print_time = time.time()
        msg = lane.bridge.cv2_to_imgmsg(np.zeros((360,640,3),np.uint8), encoding='bgr8')
        msg.header.stamp = rospy.Time.from_sec(42.25)
        msg.header.seq = 17
        lane.process_image(msg)
        for name in names[:5]+('left_boundary_pub',):
            self.assertEqual(getattr(lane,name).messages[0].header.stamp, msg.header.stamp)
        observation = json.loads(lane.lane_observation_pub.messages[0].data)
        self.assertEqual(observation['stamp'],42.25)
        self.assertEqual(observation['seq'],17)
        self.assertEqual(observation['points'],[])
        self.assertEqual(observation['confidence'],0)
        self.assertEqual(observation['diagnostic']['reason'],'no_trusted_centers')
        self.assertEqual(lane.previous_tracking_time,42.25)

    def test_runtime_calibration_reports_actual_process_values(self):
        lane = self.lane
        lane.configure_lane_windows(20,.10)
        evidence = lane.build_runtime_calibration(42.25)
        self.assertEqual(evidence['source_kind'],'running_process')
        self.assertEqual(evidence['H'],lane.H.tolist())
        self.assertEqual(evidence['constants']['WINDOW_HEIGHT'],20)
        self.assertEqual(evidence['constants']['MIN_LANE_PATH_FORWARD_SPAN_M'],.10)
        self.assertEqual(evidence['rectified_camera_matrix'],lane.new_camera_matrix.tolist())
        self.assertEqual(len(evidence['source_sha256']),64)
        json.dumps(evidence,allow_nan=False)

    def test_left_boundary_message_skips_empty_and_unreliable_windows(self):
        lane = self.lane
        stamp = rospy.Time.from_sec(42.25)
        points = [None, dict(x=None, usable=False),
                  dict(x=350., y=380., usable=True, observed=True,
                       geometry_ok=True, score=1.),
                  dict(x=340., y=300., usable=True, observed=True,
                       geometry_ok=True, score=1.)]
        msg = lane.build_left_boundary_message(points, stamp)
        self.assertEqual(msg.header.stamp, stamp)
        self.assertEqual(len(msg.poses), 2)
        self.assertLess(msg.poses[0].pose.position.x, msg.poses[1].pose.position.x)
        self.assertEqual(lane.build_left_boundary_message([None]*10, stamp).poses, [])

    def test_right_of_divider_timer_keeps_publishing_after_empty_frames(self):
        lane = self.lane
        lane.configure_lane_windows(40,.15)
        lane.LANE_REFERENCE_MODE = 'RIGHT_OF_DIVIDER'
        lane.max_image_age = 0.0
        names = ('front_pub','white_pub','metric_bev_pub','lane_tracking_pub',
                 'lane_path_pub','lane_confidence_pub','lane_observation_pub','left_boundary_pub')
        for name in names:
            setattr(lane,name,Sink())
        lane.last_print_time = time.time()
        for seq, visible in enumerate((False, True, False, True)):
            mask = np.zeros((400,480),np.uint8)
            if visible:
                mask[120:400,348:353] = 255
            lane.make_metric_bev = lambda *args, **kwargs: mask.copy()
            msg = lane.bridge.cv2_to_imgmsg(np.zeros((360,640,3),np.uint8), encoding='bgr8')
            msg.header.stamp = rospy.Time.from_sec(42.+seq*.1)
            msg.header.seq = seq
            lane.latest_image_msg = msg
            lane.process_latest_image(None)
            observation = json.loads(lane.lane_observation_pub.messages[-1].data)
            self.assertEqual(observation['seq'],seq)
            self.assertEqual(bool(observation['points']),visible)
            self.assertEqual(bool(lane.left_boundary_pub.messages[-1].poses),visible)
            for name in names:
                self.assertEqual(len(getattr(lane,name).messages),seq+1)
        self.assertTrue(lane.processing_lock.acquire(False))
        lane.processing_lock.release()

    def test_observed_white_line_publishes_across_multiple_frames(self):
        lane = self.lane
        lane.configure_lane_windows(40,.15)
        for name in ('front_pub','white_pub','metric_bev_pub','lane_tracking_pub',
                     'lane_path_pub','lane_confidence_pub','lane_observation_pub','left_boundary_pub'):
            setattr(lane,name,Sink())
        mask = np.zeros((400,480),np.uint8)
        mask[120:400,358:363] = 255
        lane.make_metric_bev = lambda *args, **kwargs: mask.copy()
        lane.last_print_time = time.time()
        for seq in range(3):
            msg = lane.bridge.cv2_to_imgmsg(np.zeros((360,640,3),np.uint8),encoding='bgr8')
            msg.header.stamp = rospy.Time.from_sec(42.+seq*.1)
            msg.header.seq = seq
            lane.process_image(msg)
            observation = json.loads(lane.lane_observation_pub.messages[-1].data)
            self.assertEqual(observation['seq'],seq)
            self.assertGreaterEqual(len(observation['points']),3)
            self.assertGreaterEqual(len(observation['boundaries']['RIGHT']),3)
        self.assertEqual(len(lane.lane_observation_pub.messages),3)

    def test_timeout_forgets_old_center_on_reacquisition(self):
        lane = self.lane
        lane.configure_lane_windows(40,.15)
        mask = np.zeros((400,480),np.uint8)
        mask[120:400,358:363] = 255
        first = lane.track_metric_lane(mask,source_stamp=10.)
        lane.previous_center_points = [dict(p,x=p['x']+60) if p else None
                                       for p in first['center_points']]
        second = lane.track_metric_lane(mask,source_stamp=11.)
        for a,b in zip(first['center_points'],second['center_points']):
            if a is not None:
                self.assertAlmostEqual(a['x'],b['x'],places=5)

    def test_short_near_segment_reports_reason_without_using_far_segment(self):
        lane = self.lane
        points = [dict(x=240.,y=380.-40*i,window_index=i,usable=True) for i in range(7)]
        diagnostic = {}
        self.assertEqual(lane.select_nearest_lane_path_segment(
            [points[:1],points[2:]],diagnostic),[])
        self.assertEqual(diagnostic['reason'],'too_few_near_points')
        self.assertEqual(diagnostic['segment_lengths'],[1,5])


if __name__ == '__main__':
    unittest.main()
