import unittest
from robot.signs.candidate_gate import RouteSignGate,validate_settings
from test_sign_training_roi import TrainingRoiTests,Value


class GateTests(unittest.TestCase):
    def setUp(self):
        self.gate=RouteSignGate(dict(enabled=True))

    def result(self,label='PARKING',bounds=(271,100,29,48),confidence=.94,stamp=10.,shape=(360,640,3)):
        return self.gate.evaluate(label,confidence,bounds,shape,stamp)

    def test_far_uturn_cannot_pass_with_high_confidence_or_more_frames(self):
        for i in range(8):
            result=self.result(label='UTURN',bounds=(515,103,28,35),confidence=.99,stamp=10.+i*.333)
            self.assertEqual(result[0],'')
            self.assertEqual(result[1],'too_small')

    def test_height_boundary_edge_right_and_scaled_image(self):
        for height,shape,accepted in ((44,(360,640,3),False),(45,(360,640,3),True),
                                      (52,(360,640,3),True),(89,(720,1280,3),False),
                                      (90,(720,1280,3),True)):
            self.setUp()
            for i in range(3):
                result=self.result(label='RIGHT',bounds=(shape[1]-4,100,4,height),shape=shape,stamp=10.+i*.333)
            self.assertEqual(result[0],'RIGHT' if accepted else '')

    def test_obvious_left_board_is_excluded_even_with_high_confidence(self):
        for i in range(6):
            result=self.result(bounds=(20,80,100,100),confidence=.99,stamp=10.+i*.3)
            self.assertEqual(result[0],'')
            self.assertEqual(result[1],'outside_route_region')

    def test_image_left_of_center_is_not_same_as_left_route(self):
        self.result(label='LEFT',bounds=(220,83,45,48))
        self.assertEqual(self.result(label='LEFT',bounds=(236,80,49,51),stamp=10.33)[0],'LEFT')

    def test_low_confidence_board_resets_track(self):
        self.result()
        self.assertEqual(self.result(confidence=.78,stamp=10.33)[1],'low_confidence')
        self.assertEqual(self.result(stamp=10.66)[2]['frames'],1)

    def test_different_same_class_boards_cannot_share_votes(self):
        self.result(bounds=(250,100,20,50))
        a=self.result(bounds=(520,100,20,50),stamp=10.33)
        self.assertEqual(a[2]['frames'],1)
        self.assertEqual(self.result(bounds=(520,100,20,50),stamp=10.66)[0],'PARKING')

    def test_missing_or_stale_observation_resets_track(self):
        self.result()
        self.result(label='',bounds=None,confidence=0.,stamp=10.3)
        self.assertEqual(self.result(stamp=10.6)[2]['frames'],1)
        self.assertEqual(self.result(stamp=12.)[2]['frames'],1)

    def test_duplicate_source_does_not_add_votes(self):
        self.result()
        self.assertEqual(self.result()[1],'duplicate_source_frame')
        self.assertEqual(self.result(stamp=10.33)[2]['frames'],2)

    def test_red_and_green_are_not_blocked_by_side_gate(self):
        for label in ('RED','GREEN'):
            self.assertEqual(self.result(label=label,bounds=(0,0,20,20))[0],label)

    def test_scaled_image_keeps_the_same_region(self):
        a=self.result(bounds=(542,200,58,56),shape=(720,1280,3))
        self.assertAlmostEqual(a[2]['center_ratio'],285.5/640.)

    def test_configuration_validation_rejects_invalid_values(self):
        for cfg in (dict(enabled='true'),dict(left_center_ratio=.8),dict(small_confidence=float('nan')),
                    dict(normal_frames=2.5),dict(small_frames=2,normal_frames=3)):
            with self.assertRaises(ValueError):validate_settings(cfg)


class NodeGateTests(TrainingRoiTests):
    def prepare(self,detections):
        self.node.backend='yolo_trt'
        self.node.route_gate=RouteSignGate(dict(enabled=True))
        self.node.detector=Value(detect=lambda frame:detections)

    def detection(self,label,bounds,confidence=.94):
        return dict(label=label,bounds=bounds,confidence=confidence,scores={label:confidence})

    def infer_at(self,stamp):
        self.node.latest.header.stamp=Value(to_sec=lambda:stamp)
        self.module.rospy.Time.now=lambda:Value(to_sec=lambda:stamp+.1)
        self.node.infer(None)
        self.assertFalse(self.warnings)
        return self.messages[-1]

    def test_large_left_foreign_board_does_not_hide_front_p_candidate(self):
        foreign=self.detection('right',[0,80,150,150])
        own=self.detection('park',[271,100,29,48])
        self.prepare([foreign,own])
        for i in range(3):info=self.infer_at(10.+i*.333)
        self.assertEqual(info['label'],'PARKING')
        self.assertEqual(info['route_detections'],[own])
        self.assertEqual(len(info['detections']),2)
        self.assertEqual(info['sign_bounds'],own['bounds'])

    def test_only_left_detection_still_preserves_rejected_pixels(self):
        self.prepare([self.detection('park',[10,80,80,90])])
        info=self.infer_at(10.)
        self.assertEqual(info['top_label'],'PARKING')
        self.assertEqual(info['label'],'')
        self.assertEqual(info['range_reason'],'outside_route_region')
        self.assertEqual(info['route_detections'],[])

    def test_small_raw_confidence_and_source_stamp_remain_in_evidence(self):
        self.prepare([self.detection('park',[300,103,26,23],.9355)])
        info=self.infer_at(10.)
        self.assertEqual(info['label'],'')
        self.assertEqual(info['range_reason'],'too_small')
        self.assertEqual(info['confidence'],.9355)
        self.assertEqual(info['stamp'],10.)

    def test_far_uturn_then_near_straight_with_gate_enabled(self):
        self.prepare([self.detection('uturn',[515,103,28,35],.99)])
        for i in range(6):
            info=self.infer_at(10.+i*.333)
            self.assertEqual(info['label'],'')
            self.assertEqual(info['range_reason'],'too_small')
        self.prepare([self.detection('straight',[528,43,98,109],.968)])
        self.infer_at(12.)
        self.assertEqual(self.infer_at(12.333)['label'],'STRAIGHT')

    def test_wide_short_board_does_not_hide_tall_edge_right(self):
        far=self.detection('uturn',[200,100,150,35],.99)
        edge=self.detection('right',[619,80,21,52],.94)
        self.prepare([far,edge])
        self.infer_at(10.)
        info=self.infer_at(10.333)
        self.assertEqual(info['label'],'RIGHT')
        self.assertEqual(info['route_detections'],[edge])


if __name__=='__main__':unittest.main()
