from test_sign_training_roi import TrainingRoiTests, Value

class YoloSignNodeTests(TrainingRoiTests):
    def test_rear_small_p_triggers_first_frame_anywhere_despite_larger_right(self):
        import json
        from robot.signs.candidate_gate import RouteSignGate
        n=self.node;n.backend='yolo_trt';rear_messages=[]
        n.route_gate=RouteSignGate(dict(enabled=True),.8,.125)
        n.rear_route_gate=RouteSignGate(dict(enabled=True,left_center_ratio=0.),.8,.125)
        n.rear_pub=Value(publish=lambda msg:rear_messages.append(json.loads(msg.data)))
        p=dict(label='park',confidence=.95,bounds=[10,80,20,12],scores={'park':.95})
        right=dict(label='right',confidence=.95,bounds=[300,80,90,90],scores={'right':.95})
        n.detector=Value(detect=lambda frame:[right,p])
        n.infer(None)
        self.assertEqual(self.messages[-1]['route_gate']['frames'],1)
        self.assertEqual(self.messages[-1]['top_label'],'RIGHT')
        n.status(Value(data=json.dumps(dict(state='PARKING',parking_entry=dict(camera_source='rear')))))
        for stamp in (10.,10.05):
            n.rear_callback(Value(header=Value(stamp=Value(to_sec=lambda:stamp))))
            n.infer(None)
        self.assertEqual([m['label'] for m in rear_messages],['PARKING','PARKING'])
        self.assertEqual(rear_messages[0]['sign_min_height_ratio'],0.)
        self.assertEqual(rear_messages[-1]['sign_bounds'],p['bounds'])
        self.assertEqual(rear_messages[-1]['camera_source'],'rear')
        self.assertEqual(n.route_gate.track['count'],1)
        self.assertFalse(self.warnings)

    def test_rear_p_still_requires_confidence_and_fresh_frame(self):
        import json
        n=self.node;n.backend='yolo_trt';n.rear_requested=True;received=[]
        n.rear_pub=Value(publish=lambda msg:received.append(json.loads(msg.data)))
        p=dict(label='park',confidence=.79,bounds=[10,80,20,12],scores={'park':.79})
        n.detector=Value(detect=lambda frame:[p])
        n.rear_callback(Value(header=Value(stamp=Value(to_sec=lambda:10.))))
        n.infer(None)
        self.assertEqual(received[-1]['label'],'')
        p['confidence']=.95
        n.infer(None)
        self.assertEqual(len(received),1)
        n.rear_callback(Value(header=Value(stamp=Value(to_sec=lambda:8.))))
        n.infer(None)
        self.assertEqual(len(received),1)

    def test_route_height_gate_matches_recorded_far_and_edge_boards(self):
        n=self.node;n.backend='yolo_trt';n.min_height_ratio=.125
        for label,bounds,accepted in (
                ('uturn',[491,100,27,31],False),
                ('right',[619,100,21,52],True),
                ('straight',[582,53,58,123],True),
                ('park',[620,98,20,58],True),
                ('left',[100,100,50,44],False),
                ('left',[100,100,50,45],True),
                ('red',[100,100,20,20],True),
                ('green',[100,100,20,20],True)):
            n.last=-1
            detection=dict(label=label,confidence=.95,bounds=bounds,scores={label:.95})
            n.detector=Value(detect=lambda frame:[detection])
            n.infer(None)
            info=self.messages[-1]
            self.assertEqual(bool(info['label']),accepted,label)
            self.assertEqual(info['detections'],[detection])
            if not accepted:self.assertEqual(info['range_reason'],'too_small')
        self.assertFalse(self.warnings)

    def test_route_height_gate_scales_with_image_height(self):
        n=self.node;n.backend='yolo_trt';n.min_height_ratio=.125
        import numpy as np
        self.frame=np.zeros((720,1280,3),dtype=np.uint8)
        detection=dict(label='uturn',confidence=.95,bounds=[100,100,54,62],scores={'uturn':.95})
        n.detector=Value(detect=lambda frame:[detection])
        n.infer(None)
        self.assertEqual(self.messages[-1]['label'],'')

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
        self.assertEqual(info['detections'][0]['bounds'],[200,220,80,80])
        self.assertFalse(self.warnings)

    def test_yolo_nearest_low_confidence_withholds_label_without_far_fallback(self):
        n=self.node;n.backend='yolo_trt'
        near=dict(label='right',confidence=.70,bounds=[200,220,80,80],scores={'right':.70})
        far=dict(label='park',confidence=.99,bounds=[10,10,20,20],scores={'park':.99})
        n.detector=Value(detect=lambda frame:[far,near])
        n.infer(None)
        info=self.messages[-1]
        self.assertEqual(info['label'],'')
        self.assertEqual(info['top_label'],'RIGHT')
        self.assertEqual(info['confidence'],.70)
        self.assertEqual(info['sign_bounds'],near['bounds'])
        self.assertEqual(info['detections'],[near,far])
        self.assertEqual(info['stamp'],10.0)
        self.assertFalse(self.warnings)

    def test_yolo_empty_frame_publishes_unknown(self):
        self.node.backend='yolo_trt';self.node.detector=Value(detect=lambda frame:[])
        self.node.infer(None)
        self.assertEqual(self.calls,[])
        self.assertEqual(self.messages[-1]['label'],'')
        self.assertEqual(self.messages[-1]['detections'],[])
        self.assertFalse(self.warnings)
