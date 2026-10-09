import unittest
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src/ros/signs/scripts')))
import cv2
import numpy as np
from board_roi import extract_board_roi


class BoardTests(unittest.TestCase):
    def arrow(self, image, x, y, size):
        cv2.rectangle(image,(x,y),(x+size-1,y+size-1),(220,70,20),-1)
        points=np.array([[.5,.15],[.8,.45],[.6,.45],[.6,.85],
                         [.4,.85],[.4,.45],[.2,.45]])
        cv2.fillPoly(image,[(points*size+[x,y]).astype(np.int32)],(245,245,245))

    def test_arrow_at_all_image_positions(self):
        for x,y in ((0,0),(292,0),(584,0),(0,152),(584,152),
                    (0,304),(292,304),(584,304)):
            image=np.full((360,640,3),90,np.uint8)
            self.arrow(image,x,y,56)
            self.assertIsNotNone(extract_board_roi(image), (x,y))

    def test_small_arrow_independent_of_frame_area(self):
        for width,height in ((640,360),(1280,720)):
            for size in (16,24,36):
                image=np.full((height,width,3),90,np.uint8)
                self.arrow(image,width-size-3,height-size-3,size)
                self.assertIsNotNone(extract_board_roi(image), (width,size))

    def test_large_arrow_and_clipped_padding(self):
        image=np.full((360,640,3),90,np.uint8)
        self.arrow(image,0,0,350)
        crop,bounds=extract_board_roi(image,True)
        self.assertEqual(bounds,(0,0,350,350))
        self.assertEqual(crop.shape[0],360)

    def test_floor_line_at_bottom_is_not_a_board(self):
        image=np.full((360,640,3),90,np.uint8)
        cv2.rectangle(image,(0,310),(639,345),(220,70,20),-1)
        self.assertIsNone(extract_board_roi(image))

    def test_tiny_colored_noise_is_rejected(self):
        image=np.full((360,640,3),90,np.uint8)
        for x in range(10,600,30):
            cv2.circle(image,(x,320),3,(0,0,220),-1)
        self.assertIsNone(extract_board_roi(image))

    def test_small_tapered_cone_with_light_patch_is_not_a_board(self):
        image=np.full((360,640,3),90,np.uint8)
        cv2.fillPoly(image,[np.array([[80,300],[68,324],[92,324]])],(220,70,20))
        cv2.rectangle(image,(77,309),(82,313),(240,240,240),-1)
        self.assertIsNone(extract_board_roi(image))

    def test_diagnostics_report_rejections_and_selected_candidate(self):
        image=np.full((360,640,3),90,np.uint8)
        self.arrow(image,500,300,48)
        cv2.rectangle(image,(50,290),(110,350),(220,70,20),-1)
        diagnostics={}
        _,bounds=extract_board_roi(image,True,diagnostics)
        self.assertEqual(bounds,(500,300,48,48))
        self.assertEqual(diagnostics['candidate_count'],1)
        self.assertEqual(diagnostics['rejected_counts'],{'no_light_glyph':1})

    def test_blue_solid_obstacle_is_not_an_arrow_board(self):
        image=np.full((360,640,3),90,np.uint8)
        cv2.rectangle(image,(250,90),(350,170),(220,70,20),-1)
        self.assertIsNone(extract_board_roi(image))

    def test_complete_board_is_tightly_cropped(self):
        image=np.full((360,640,3),90,np.uint8)
        cv2.rectangle(image,(250,80),(350,180),(220,70,20),-1)
        cv2.putText(image,'P',(267,157),cv2.FONT_HERSHEY_SIMPLEX,2,(230,230,230),7)
        crop,bounds=extract_board_roi(image,True)
        self.assertLess(crop.shape[0],135)
        self.assertLess(crop.shape[1],135)
        self.assertLessEqual(bounds[0],250)

    def test_larger_blue_obstacle_does_not_hide_smaller_board(self):
        image=np.full((360,640,3),90,np.uint8)
        cv2.rectangle(image,(20,40),(220,200),(220,70,20),-1)
        cv2.rectangle(image,(400,70),(470,140),(220,70,20),-1)
        cv2.putText(image,'P',(410,126),cv2.FONT_HERSHEY_SIMPLEX,1.5,(230,230,230),5)
        _,bounds=extract_board_roi(image,True)
        self.assertGreater(bounds[0],350)

    def test_red_and_green_solid_boards_remain_valid(self):
        for color in ((0,0,220),(0,190,0)):
            image=np.full((360,640,3),90,np.uint8)
            cv2.circle(image,(320,120),40,color,-1)
            self.assertIsNotNone(extract_board_roi(image))

    def test_larger_valid_board_wins_among_two_boards(self):
        image=np.full((360,640,3),90,np.uint8)
        for x,y,size in ((60,80,60),(400,60,110)):
            cv2.rectangle(image,(x,y),(x+size,y+size),(220,70,20),-1)
            cv2.putText(image,'P',(x+10,y+size-10),cv2.FONT_HERSHEY_SIMPLEX,size/55.,(230,230,230),5)
        self.assertGreater(extract_board_roi(image,True)[1][0],350)

    def test_side_view_red_is_preserved(self):
        image=np.full((360,640,3),90,np.uint8)
        cv2.rectangle(image,(200,80),(230,170),(0,0,220),-1)
        self.assertIsNotNone(extract_board_roi(image))

    def test_wide_blue_base_with_white_mark_is_rejected(self):
        image=np.full((360,640,3),90,np.uint8)
        cv2.rectangle(image,(200,80),(300,140),(220,70,20),-1)
        cv2.putText(image,'P',(225,130),cv2.FONT_HERSHEY_SIMPLEX,1,(230,230,230),3)
        self.assertIsNone(extract_board_roi(image))


if __name__=='__main__': unittest.main()
