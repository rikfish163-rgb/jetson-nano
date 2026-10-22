from __future__ import division
import unittest
import numpy as np
from yolo_detector import preprocess, decode, decode_heads, select_detection

class DetectorTests(unittest.TestCase):
    def test_sparse_heads_preserve_final_detections(self):
        rng = np.random.RandomState(42)
        heads = [rng.normal(-7, 1, (1,36,s,s)).astype(np.float32)
                 for s in (80,40,20)]
        for head in heads:
            # Include overlapping candidates, every class and threshold edges.
            values = head.reshape(1,3,12,head.shape[-2],head.shape[-1])
            for label in range(7):
                values[0,0,:,label+1,label+2] = 0
                values[0,0,4,label+1,label+2] = 5
                values[0,0,5+label,label+1,label+2] = 5
            values[0,1,4,2,3] = 0  # objectness exactly 0.5
            values[0,1,5,2,3] = 80
        full = decode_heads(heads)
        sparse = decode_heads(heads, confidence=.5)
        self.assertLess(sparse.shape[1], 100)
        self.assertEqual(decode(full,(360,640,3),1,(0,140)),
                         decode(sparse,(360,640,3),1,(0,140)))

    def test_sparse_heads_empty_background(self):
        heads = [np.full((1,36,s,s),-20,np.float32) for s in (80,40,20)]
        output = decode_heads(heads, confidence=.5)
        self.assertEqual(output.shape, (1,0,12))
        self.assertEqual(decode(output,(360,640,3),1,(0,140)), [])

    def test_preprocess_matches_original_pixels_for_different_sizes(self):
        rng = np.random.RandomState(11)
        for h,w in ((360,640),(480,640),(801,321),(240,320)):
            frame = rng.randint(0,256,(h,w,3)).astype(np.uint8)
            blob,ratio,(left,top) = preprocess(frame)
            import cv2
            nw,nh = int(round(w*ratio)),int(round(h*ratio))
            canvas = np.full((640,640,3),114,np.uint8)
            canvas[top:top+nh,left:left+nw] = cv2.resize(frame,(nw,nh))
            reference = canvas[:,:,::-1].transpose(2,0,1)[None].astype(np.float32)/255.
            np.testing.assert_allclose(blob,reference,rtol=0,atol=1e-7)
            self.assertTrue(blob.flags.c_contiguous)

    def test_raw_heads_apply_each_anchors_own_dimensions(self):
        heads=[np.zeros((1,36,size,size),np.float32) for size in (80,40,20)]
        result=decode_heads(heads)
        self.assertEqual(result.shape,(1,25200,12))
        np.testing.assert_array_equal(result[0,0,:4],[4,4,10,13])
        np.testing.assert_array_equal(result[0,6400,:4],[4,4,16,30])
        np.testing.assert_array_equal(result[0,12800,:4],[4,4,33,23])
        np.testing.assert_array_equal(result[0,19200,:4],[8,8,30,61])
    def row(self, x, y, w, h, index=2, confidence=.95):
        row=np.zeros(12,dtype=np.float32)
        row[:5]=[x,y,w,h,1.0];row[5+index]=confidence
        return row

    def test_letterbox_rgb_and_coordinates_at_bottom(self):
        frame=np.zeros((360,640,3),np.uint8);frame[:]=(10,20,30)
        blob,ratio,padding=preprocess(frame)
        self.assertEqual(blob.shape,(1,3,640,640))
        np.testing.assert_allclose(blob[0,:,140,0],np.array([30,20,10])/255.,atol=1e-6)
        detections=decode(np.array([self.row(320,470,80,40)]),frame.shape,ratio,padding)
        self.assertEqual(detections[0]['bounds'],[280,310,80,40])
        self.assertEqual(detections[0]['label'],'straight')

    def test_nms_and_nonfinite_output(self):
        rows=np.array([self.row(200,250,80,80),self.row(201,251,80,80),self.row(50,50,10,10)])
        rows[2,0]=float('nan')
        self.assertEqual(len(decode(rows,(360,640,3),1,(0,140))),1)

    def test_clipped_and_padding_only_boxes(self):
        rows=np.array([self.row(5,150,30,40),self.row(50,10,20,10)])
        d=decode(rows,(360,640,3),1,(0,140))
        self.assertEqual(len(d),1)
        self.assertEqual(d[0]['bounds'],[0,0,20,30])

    def test_larger_accepted_sign_over_distant_high_score(self):
        a=dict(bounds=[10,10,20,20],confidence=.99,label='park')
        b=dict(bounds=[10,200,100,100],confidence=.9,label='right')
        self.assertIs(select_detection([a,b],.8),b)
        b['confidence']=.7
        self.assertIs(select_detection([a,b],.8),a)
        self.assertIsNone(select_detection([],.8))

if __name__=='__main__':unittest.main()
