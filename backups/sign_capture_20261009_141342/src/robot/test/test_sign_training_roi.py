"""Training-compatible ROI inference without vehicle commands."""
import imp
import json
import os
import threading
import unittest
import numpy as np

class Value(object):
    def __init__(self, **kwargs): self.__dict__.update(kwargs)

class TrainingRoiTests(unittest.TestCase):
    def setUp(self):
        self.module = imp.load_source('full_frame_node', os.path.join(os.path.dirname(__file__), '..', 'signs', 'sign_node.py'))
        self.warnings=[]
        self.module.rospy=Value(Time=Value(now=lambda:Value(to_sec=lambda:10.1)),
            logwarn_throttle=lambda *a:self.warnings.append(a),loginfo_throttle=lambda *a:None)
        n=self.node=self.module.Node.__new__(self.module.Node)
        n.capture_dir=''
        n.lock=threading.Lock(); n.last=-1; n.timeout=1.5; n.waiting_green=True; n.threshold=.8
        self.frame=np.zeros((360,640,3),dtype=np.uint8)
        n.latest=Value(header=Value(stamp=Value(to_sec=lambda:10.0)))
        n.bridge=Value(imgmsg_to_cv2=lambda *a:self.frame)
        self.calls=[]; self.messages=[]
        self.scores=dict(red=.02,green=.02,straight=.04,left=.02,right=.85,uturn=.02,park=.03)
        def classify(frame):
            self.calls.append(frame)
            return self.scores
        n.classifier=Value(classify_scores=classify)
        n.normalize=lambda label:'PARKING' if label=='park' else label.upper()
        n.pub=Value(publish=lambda msg:self.messages.append(json.loads(msg.data)))
        self.crop=self.frame[20:100,40:140].copy()
        self.bounds=(48,28,84,64)
        n.roi=lambda frame,return_bounds=False:(self.crop,self.bounds)

    def test_only_training_roi_is_classified_with_all_scores(self):
        self.node.infer(None)
        self.assertEqual(len(self.calls),1)
        self.assertIs(self.calls[0],self.crop)
        r=self.messages[-1]
        self.assertEqual(r['label'],'RIGHT')
        self.assertEqual(r['confidence'],.85)
        self.assertEqual(len(r['scores']),7)
        self.assertAlmostEqual(sum(r['scores'].values()),1)
        self.assertEqual(r['input_mode'],'training_roi')
        self.assertEqual(r['sign_bounds'],list(self.bounds))
        self.assertIsNone(r['sign_depth_m'])
        self.assertFalse(r['startup_only'])
        self.assertFalse(self.warnings)

    def test_no_candidate_publishes_unknown_without_classifying_background(self):
        self.node.roi=lambda *a,**kw:None
        self.node.infer(None)
        self.assertEqual(self.calls,[])
        self.assertEqual(self.messages[-1]['label'],'')
        self.assertEqual(self.messages[-1]['confidence'],0.0)
        self.assertEqual(self.messages[-1]['scores'],{})
        self.assertEqual(self.messages[-1]['range_reason'],'no_candidate')
        self.assertFalse(self.warnings)

    def test_real_collection_extractor_matches_model_input(self):
        import sys
        scripts=os.path.abspath(os.path.join(os.path.dirname(__file__),'..','..','ros','signs','scripts'))
        sys.path.insert(0,scripts)
        from sign_string_node import extract_sign_roi
        self.frame[50:130,100:180]=(255,0,0)
        self.node.roi=extract_sign_roi
        self.node.infer(None)
        np.testing.assert_array_equal(self.calls[0],extract_sign_roi(self.frame))

    def test_low_confidence_still_published(self):
        self.scores=dict(red=.15,green=.15,straight=.14,left=.14,right=.16,uturn=.13,park=.13)
        self.node.infer(None)
        self.assertEqual(len(self.messages),1)
        self.assertEqual(self.messages[-1]['confidence'],.16)
        self.assertEqual(self.messages[-1]['label'],'')
        self.assertEqual(self.messages[-1]['top_label'],'RIGHT')

    def test_exact_threshold_is_accepted(self):
        self.scores['right']=.8
        self.node.infer(None)
        self.assertEqual(self.messages[-1]['label'],'RIGHT')

    def test_duplicate_frame_not_republished(self):
        self.node.infer(None); self.node.infer(None)
        self.assertEqual(len(self.messages),1)

    def test_stale_frame_not_published(self):
        self.node.latest.header.stamp=Value(to_sec=lambda:8.0)
        self.node.infer(None)
        self.assertEqual(self.calls,[])

    def test_background_winner_never_publishes_parking(self):
        self.scores=dict(park=.09,background=.91)
        self.node.infer(None)
        r=self.messages[-1]
        self.assertEqual(r['label'],'')
        self.assertEqual(r['top_label'],'BACKGROUND')
        self.assertEqual(r['scores']['BACKGROUND'],.91)

    def test_board_diagnostics_are_published(self):
        self.node.roi_mode='board'
        def roi(frame,return_bounds=False,diagnostics=None):
            diagnostics.update(candidate_count=0,rejected_counts={'no_light_glyph':2},regions=[])
            return None
        self.node.roi=roi
        self.node.infer(None)
        self.assertEqual(self.messages[-1]['roi_diagnostics']['rejected_counts'],{'no_light_glyph':2})
        self.assertEqual(self.calls,[])
        self.assertFalse(self.warnings)

    def test_debug_header_does_not_hide_top_of_camera_image(self):
        self.node.roi=lambda *a,**kw:None
        self.frame[:60,:]=(120,160,220)
        frames=[]
        self.module.rospy.get_name=lambda:'test_sign'
        self.node.bridge.cv2_to_imgmsg=lambda pixels,encoding:Value(pixels=pixels)
        self.node.debug_frame=Value(get_num_connections=lambda:1,
                                    publish=lambda msg:frames.append(msg.pixels))
        self.node.debug_crop=Value(get_num_connections=lambda:1,publish=lambda msg:None)
        self.node.infer(None)
        self.assertEqual(frames[0].shape,(464,640,3))
        np.testing.assert_array_equal(frames[0][104:],self.frame)
        self.assertFalse(self.warnings)

    def test_rejected_evidence_is_rate_and_count_limited(self):
        self.node.roi=lambda *a,**kw:None
        self.node.capture_dir='unused'
        self.node.reject_capture_limit=2
        self.module.rospy.loginfo=lambda *a:None
        saved=[]
        self.module.save_capture=lambda directory,frame,crop,info:saved.append(info) or 'capture'
        for stamp in (10.,10.1,12.1,14.2):
            self.module.rospy.Time.now=lambda:Value(to_sec=lambda:stamp+.1)
            self.node.latest.header.stamp=Value(to_sec=lambda:stamp)
            self.node.infer(None)
        self.assertEqual(len(saved),2)
        self.assertTrue(all(row['label']=='' for row in saved))
        self.assertFalse(self.warnings)

if __name__=='__main__': unittest.main()
