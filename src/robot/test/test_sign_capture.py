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



class CaptureWriterTests(unittest.TestCase):
    def setUp(self):
        from robot.signs.capture import CaptureWriter
        self.directory = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.directory)
        self.errors = []
        self.saved = []
        self.factory = CaptureWriter

    def writer(self, **kwargs):
        writer = self.factory(self.directory, on_error=self.errors.append,
                              on_saved=lambda prefix, info:self.saved.append(prefix),
                              **kwargs)
        self.addCleanup(writer.close)
        return writer

    def test_worker_preserves_pixels_even_when_source_is_reused(self):
        writer = self.writer(min_free_bytes=0)
        frame = np.full((20,30,3),90,np.uint8)
        crop = frame[2:10,3:12].copy()
        info = dict(stamp=200., label='STRAIGHT', confidence=.91)
        self.assertTrue(writer.submit(frame, crop, info))
        frame[:] = 0
        crop[:] = 0
        info['label'] = 'RIGHT'
        writer.close()
        self.assertEqual(len(self.saved),1)
        prefix = self.saved[0]
        self.assertTrue(np.all(cv2.imread(prefix+'_frame.png') == 90))
        self.assertTrue(np.all(cv2.imread(prefix+'_crop.png') == 90))
        with open(prefix+'.json') as stream:
            self.assertEqual(json.load(stream)['label'],'STRAIGHT')
        self.assertFalse(self.errors)
        self.assertFalse(writer.submit(frame,crop,info))

    def test_capacity_stops_saving_without_deleting_existing_files(self):
        existing = os.path.join(self.directory,'existing.txt')
        with open(existing,'w') as stream:stream.write('keep')
        writer = self.writer(max_bytes=1,min_free_bytes=0)
        writer.submit(np.zeros((20,30,3),np.uint8),None,dict(stamp=1.,label='GREEN'))
        writer.close()
        self.assertEqual(self.saved,[])
        self.assertTrue(writer.disabled)
        self.assertIn('session limit',self.errors[0])
        self.assertEqual(os.listdir(self.directory),['existing.txt'])

    def test_low_disk_space_does_not_write_images(self):
        writer = self.writer(min_free_bytes=10**30)
        writer.submit(np.zeros((20,30,3),np.uint8),None,dict(stamp=1.,label='GREEN'))
        writer.close()
        self.assertEqual(self.saved,[])
        self.assertIn('low disk',self.errors[0])
        self.assertEqual(os.listdir(self.directory),[])

    def test_slow_disk_never_blocks_inference_queue(self):
        import threading
        import robot.signs.capture as module
        original = module.save_capture
        started, release = threading.Event(), threading.Event()
        def slow_save(*args):
            started.set()
            release.wait(2.)
            return original(*args)
        module.save_capture = slow_save
        self.addCleanup(setattr,module,'save_capture',original)
        self.addCleanup(release.set)
        writer = self.writer(queue_size=1,min_free_bytes=0)
        frame = np.zeros((20,30,3),np.uint8)
        self.assertTrue(writer.submit(frame,None,dict(stamp=1.,label='GREEN')))
        self.assertTrue(started.wait(1.))
        self.assertTrue(writer.submit(frame,None,dict(stamp=2.,label='GREEN')))
        self.assertFalse(writer.submit(frame,None,dict(stamp=3.,label='GREEN')))
        release.set()
        writer.close()
        self.assertEqual(len(self.saved),2)
        self.assertIn('queue full',self.errors[0])

if __name__ == '__main__':
    unittest.main()
