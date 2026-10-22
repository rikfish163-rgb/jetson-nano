import json
import os
import shutil
import tempfile
import unittest
import cv2
import numpy as np
from robot.signs.capture import save_capture


class SignCaptureTests(unittest.TestCase):
    def test_missing_candidate_preserves_frame_and_rejection_without_fake_crop(self):
        directory = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, directory)
        frame = np.full((20,30,3),90,np.uint8)
        info = dict(stamp=124.5,label='',range_reason='no_candidate',
                    roi_diagnostics={'rejected_counts':{'no_light_glyph':1}})
        prefix = save_capture(directory,frame,None,info)
        self.assertTrue(np.array_equal(cv2.imread(prefix+'_frame.png'),frame))
        self.assertFalse(os.path.exists(prefix+'_crop.png'))
        with open(prefix+'.json') as stream:
            self.assertEqual(json.load(stream),info)

    def test_preserves_exact_source_frame_crop_and_scores(self):
        directory = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, directory)
        frame = np.arange(12*16*3, dtype=np.uint8).reshape(12,16,3)
        crop = frame[2:9,3:12].copy()
        info = dict(stamp=123.456, label='LEFT', confidence=.81,
                    scores={'LEFT':.81,'STRAIGHT':.19}, sign_bounds=[4,3,6,4])
        prefix = save_capture(directory, frame, crop, info)
        self.assertTrue(np.array_equal(cv2.imread(prefix+'_frame.png'), frame))
        self.assertTrue(np.array_equal(cv2.imread(prefix+'_crop.png'), crop))
        with open(prefix+'.json') as stream:
            self.assertEqual(json.load(stream), info)


if __name__ == '__main__':
    unittest.main()
