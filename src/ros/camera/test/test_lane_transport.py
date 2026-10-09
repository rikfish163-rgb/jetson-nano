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
        # The original tracker retains a small confidence floor on blank frames;
        # an empty path must still be published, so the controller cannot drive.
        self.assertLess(observation['confidence'],.35)
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

    def test_tracking_history_uses_capture_timestamp(self):
        lane = self.lane
        lane.configure_lane_windows(40,.15)
        mask = np.zeros((400,480),np.uint8)
        mask[120:400,358:363] = 255
        first = lane.track_metric_lane(mask,source_stamp=10.)
        self.assertEqual(lane.previous_tracking_time,10.)
        second = lane.track_metric_lane(mask,source_stamp=11.)
        self.assertEqual(lane.previous_tracking_time,11.)
        self.assertEqual(first['lane_mode'],second['lane_mode'])

    def test_current_far_paint_survives_blank_near_windows(self):
        lane=self.lane
        lane.configure_lane_windows(40,.15)
        mask=np.zeros((400,480),np.uint8)
        mask[120:280,358:363]=255
        tracking=lane.track_metric_lane(mask,source_stamp=10.)
        observed=[p for p in tracking['right_points'] if p.get('observed',False)]
        self.assertGreaterEqual(len(observed),3)
        self.assertTrue(all(120 <= p['y'] < 280 for p in observed))
        self.assertGreater(len(tracking['center_points']),0)

    def test_short_near_segment_retains_far_points_in_direct_mode(self):
        lane = self.lane
        points = [dict(x=240.,y=380.-40*i,window_index=i,usable=True) for i in range(7)]
        diagnostic = {}
        self.assertEqual(lane.select_nearest_lane_path_segment(
            [points[:1],points[2:]],diagnostic),points[:1]+points[2:])
        self.assertEqual(diagnostic['reason'],'ok')
        self.assertEqual(diagnostic['selection'],'all_current_points')
        self.assertEqual(diagnostic['segment_lengths'],[1,5])

    def test_four_separated_current_dashes_are_a_usable_divider(self):
        lane=self.lane
        lane.configure_lane_windows(40,.15)
        mask=np.zeros((400,480),np.uint8)
        # Recorded stopped view: four paint patches spanning about 98 pixels.
        for y,x in [(269,285),(299,255),(332,227),(367,202)]:
            mask[y-9:y+9,x-6:x+6]=255
        tracking=lane.track_metric_lane(mask,source_stamp=10.)
        self.assertTrue(tracking['dashed_left'])
        self.assertGreaterEqual(len(lane.select_nearest_lane_path_segment(
            tracking['center_segments'])),2)

    def test_four_random_patches_do_not_establish_divider(self):
        lane=self.lane
        lane.configure_lane_windows(40,.15)
        mask=np.zeros((400,480),np.uint8)
        for y,x in [(269,285),(299,100),(332,227),(367,202)]:
            mask[y-9:y+9,x-6:x+6]=255
        self.assertIsNone(lane.find_dashed_left_boundary(mask))

    def test_four_dashes_with_one_unrelated_patch_still_track(self):
        lane=self.lane
        lane.configure_lane_windows(40,.15)
        mask=np.zeros((400,480),np.uint8)
        for y,x in [(125,43),(278,278),(308,249),(342,221),(377,197)]:
            mask[y-9:y+9,x-6:x+6]=255
        self.assertIsNotNone(lane.find_dashed_left_boundary(mask))


if __name__ == '__main__':
    unittest.main()
