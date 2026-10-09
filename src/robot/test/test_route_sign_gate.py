import unittest
from robot.signs.candidate_gate import RouteSignGate,validate_settings
from test_sign_training_roi import TrainingRoiTests,Value


class GateTests(unittest.TestCase):
    def setUp(self):
        self.gate=RouteSignGate(dict(enabled=True))

    def result(self,label='PARKING',bounds=(271,100,29,28),confidence=.94,stamp=10.,shape=(360,640,3)):
        return self.gate.evaluate(label,confidence,bounds,shape,stamp)

    def test_small_real_p_board_confirms_three_distinct_frames(self):
        results=[self.result(stamp=10.+i*.333) for i in range(3)]
        self.assertEqual([r[0] for r in results],['','','PARKING'])
        self.assertEqual(results[-1][2]['frames'],3)

    def test_obvious_left_board_is_excluded_even_with_high_confidence(self):
        for i in range(6):
            result=self.result(bounds=(20,80,100,100),confidence=.99,stamp=10.+i*.3)
            self.assertEqual(result[0],'')
            self.assertEqual(result[1],'outside_route_region')

    def test_image_left_of_center_is_not_same_as_left_route(self):
        self.result(label='LEFT',bounds=(220,83,45,48))
        self.assertEqual(self.result(label='LEFT',bounds=(236,80,49,51),stamp=10.33)[0],'LEFT')

    def test_small_low_confidence_board_resets_track(self):
        self.result()
        self.assertEqual(self.result(confidence=.88,stamp=10.33)[1],'low_confidence')
        self.assertEqual(self.result(stamp=10.66)[2]['frames'],1)

    def test_different_same_class_boards_cannot_share_votes(self):
        self.result(bounds=(250,100,20,20))
        a=self.result(bounds=(520,100,20,20),stamp=10.33)
        self.assertEqual(a[2]['frames'],1)
        self.assertEqual(self.result(bounds=(520,100,20,20),stamp=10.66)[0],'')

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
        own=self.detection('park',[271,100,29,28])
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
        self.assertEqual(info['range_reason'],'candidate_tracking')
        self.assertEqual(info['confidence'],.9355)
        self.assertEqual(info['stamp'],10.)


if __name__=='__main__':unittest.main()
