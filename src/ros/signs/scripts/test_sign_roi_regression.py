"""Regression coverage for the pre-geometry candidate contract."""
import unittest
import cv2
import numpy as np
from sign_string_node import extract_sign_roi, detect_green_start_label


class SignRoiRegressionTests(unittest.TestCase):
    def test_dim_glyph_does_not_discard_entire_blue_board(self):
        frame=np.full((360,640,3),110,dtype=np.uint8)
        cv2.circle(frame,(320,120),40,(220,80,20),-1)
        cv2.arrowedLine(frame,(320,140),(320,100),(135,135,135),8)
        self.assertIsNotNone(extract_sign_roi(frame))

    def test_preserves_rectangular_context_crop_and_unpadded_bounds(self):
        frame=np.full((360,640,3),110,dtype=np.uint8)
        cv2.rectangle(frame,(220,80),(319,139),(220,80,20),-1)
        crop,(x,y,w,h)=extract_sign_roi(frame,True)
        px,py=max(8,int(w*.5)),max(8,int(h*.5))
        expected=frame[max(0,y-py):min(360,y+h+py),max(0,x-px):min(640,x+w+px)]
        np.testing.assert_array_equal(crop,expected)
        self.assertGreater(crop.shape[1],crop.shape[0])

    def test_search_edge_is_not_a_new_whole_candidate_rejection(self):
        frame=np.full((360,640,3),110,dtype=np.uint8)
        cv2.circle(frame,(320,240),40,(220,80,20),-1)
        self.assertIsNotNone(extract_sign_roi(frame))

    def test_green_start_keeps_close_partial_board(self):
        frame=np.zeros((360,640,3),dtype=np.uint8)
        frame[:,:200]=(0,190,0)
        self.assertEqual(detect_green_start_label(frame),'green')


if __name__=='__main__':unittest.main()
