import unittest
import numpy as np
from sign_classifier_cv import SignClassifier
from sign_actions import LABELS


class FakeNet(object):
    def setInput(self, *args): pass
    def forward(self, *args): return np.array([[0]*7+[10]], np.float32)


class ClassifierLabelsTests(unittest.TestCase):
    def test_eight_class_background_is_not_a_sign(self):
        classifier = SignClassifier.__new__(SignClassifier)
        classifier.labels = list(LABELS)+['background']
        classifier.net = FakeNet()
        scores = classifier.classify_scores(np.zeros((32,32,3), np.uint8))
        self.assertGreater(scores['background'], .99)
        self.assertEqual(classifier.classify_sign(np.zeros((32,32,3), np.uint8))[2], 'background')

    def test_output_mismatch_is_rejected(self):
        classifier = SignClassifier.__new__(SignClassifier)
        classifier.labels = list(LABELS)
        classifier.net = FakeNet()
        with self.assertRaises(RuntimeError):
            classifier.classify_scores(np.zeros((32,32,3), np.uint8))

if __name__ == '__main__': unittest.main()
