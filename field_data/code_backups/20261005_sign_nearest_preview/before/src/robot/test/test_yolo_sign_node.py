from test_sign_training_roi import TrainingRoiTests, Value

class YoloSignNodeTests(TrainingRoiTests):
    def test_unwatched_debug_topics_do_not_render_images(self):
        def unexpected(*args):
            self.fail('unwatched debug image was rendered or published')
        publisher = Value(get_num_connections=lambda:0, publish=unexpected)
        self.node.debug_frame = publisher
        self.node.debug_crop = publisher
        self.node.bridge.cv2_to_imgmsg = unexpected
        self.node.infer(None)
        self.assertTrue(self.messages)
        self.assertFalse(self.warnings)

    def test_yolo_full_frame_preserves_stamp_and_selects_larger_board(self):
        n=self.node;n.backend='yolo_trt'
        seen=[]
        detections=[dict(label='park',confidence=.99,bounds=[10,10,20,20],scores={'park':.99}),
                    dict(label='right',confidence=.90,bounds=[200,220,80,80],scores={'right':.90})]
        def detect(frame):seen.append(frame);return detections
        n.detector=Value(detect=detect)
        n.infer(None)
        self.assertIs(seen[0],self.frame)
        self.assertEqual(self.calls,[])
        info=self.messages[-1]
        self.assertEqual(info['label'],'RIGHT')
        self.assertEqual(info['source'],'yolov5s')
        self.assertEqual(info['stamp'],10.0)
        self.assertEqual(info['sign_bounds'],[200,220,80,80])
        self.assertEqual(len(info['detections']),2)
        self.assertFalse(self.warnings)

    def test_yolo_empty_frame_publishes_unknown(self):
        self.node.backend='yolo_trt';self.node.detector=Value(detect=lambda frame:[])
        self.node.infer(None)
        self.assertEqual(self.calls,[])
        self.assertEqual(self.messages[-1]['label'],'')
        self.assertEqual(self.messages[-1]['detections'],[])
        self.assertFalse(self.warnings)
