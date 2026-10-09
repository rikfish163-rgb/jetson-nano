import os
import shutil
import tempfile
import unittest
from guided_collect import inventory, next_batch, TARGETS, capture_target
from common import dataset_digest


class GuidedTests(unittest.TestCase):
    def test_explicit_1000_applies_to_every_class(self):
        for label in ('red', 'right', 'background'):
            self.assertEqual([capture_target(label, n, 1000) for _, n in TARGETS],
                             [700, 200, 100])
            self.assertEqual(capture_target(label, 500, 1000), 1000)

    def test_background_is_counted_and_reviewed(self):
        root = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, root)
        labels = ['red', 'background']
        before = dataset_digest(root, labels)
        folder = os.path.join(root, 'train', 'background', 'session', 'roi')
        os.makedirs(folder)
        with open(os.path.join(folder, 'a.png'), 'wb') as f: f.write(b'cone')
        self.assertEqual(inventory(root, labels)['train']['background'], 1)
        self.assertNotEqual(dataset_digest(root, labels), before)
        self.assertEqual(sum(capture_target('background', n) for _, n in TARGETS), 1500)
        self.assertEqual(capture_target('right', 350), 350)

    def test_plan_is_500_and_batches_do_not_overrun(self):
        self.assertEqual(sum(n for _, n in TARGETS), 500)
        self.assertEqual(next_batch(0, 350), 50)
        self.assertEqual(next_batch(49, 350), 1)
        self.assertEqual(next_batch(349, 350), 1)
        self.assertEqual(next_batch(350, 350), 0)
        with self.assertRaises(ValueError): next_batch(351, 350)

    def test_resume_counts_only_roi_images(self):
        root = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, root)
        for kind in ('roi', 'frames', 'missed'):
            folder = os.path.join(root, 'train', 'red', 'session', kind)
            os.makedirs(folder)
            with open(os.path.join(folder, 'a.png'), 'wb') as f: f.write(b'image')
        self.assertEqual(inventory(root)['train']['red'], 1)
        self.assertEqual(inventory(root)['test']['park'], 0)

    def test_review_digest_detects_same_count_edits(self):
        root = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, root)
        folder = os.path.join(root, 'train', 'red', 'session', 'roi')
        os.makedirs(folder)
        path = os.path.join(folder, 'a.png')
        with open(path, 'wb') as f: f.write(b'first')
        before = dataset_digest(root)
        with open(path, 'wb') as f: f.write(b'other')
        self.assertNotEqual(dataset_digest(root), before)
        self.assertEqual(inventory(root)['train']['red'], 1)
        os.rename(path, os.path.join(folder, 'b.png'))
        self.assertNotEqual(dataset_digest(root), before)


if __name__ == '__main__': unittest.main()
