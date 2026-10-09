"""Dependency-free contract tests. No camera, model training or deployment."""
import os
import shutil
import tempfile
import unittest
from common import LABELS,samples,check_overlap,metrics,early_stop_due


class ContractTests(unittest.TestCase):
    def test_eight_class_data_and_metrics(self):
        labels = LABELS + ['background']
        for label in labels:
            folder=os.path.join(self.root,'train',label,'session','roi')
            os.makedirs(folder)
            with open(os.path.join(folder,'a.png'),'wb') as f:f.write(label.encode())
        self.assertEqual([y for _,y in samples(self.root,'train',labels)],list(range(8)))
        matrix=[[0]*8 for _ in labels]
        for i in range(8):matrix[i][i]=10
        matrix[7][0]=10
        report=metrics(matrix,labels)
        self.assertEqual(report['recall']['background'],.5)
        self.assertAlmostEqual(report['macro_recall'],7.5/8)

    def test_early_stop_counts_since_best_and_can_be_disabled(self):
        self.assertFalse(early_stop_due(28,14,15))
        self.assertTrue(early_stop_due(29,14,15))
        self.assertFalse(early_stop_due(29,29,15))
        self.assertFalse(early_stop_due(100,1,0))
    def setUp(self):
        self.root=tempfile.mkdtemp(prefix='sign_retrain_test_')
        self.addCleanup(shutil.rmtree,self.root)

    def test_order_and_metrics(self):
        self.assertEqual(LABELS,['red','green','straight','left','right','uturn','park'])
        m=[[0]*7 for _ in LABELS]
        for i in range(7):m[i][i]=9
        m[6][5]=1
        self.assertAlmostEqual(metrics(m)['park_to_uturn'],.1)
        self.assertEqual(metrics(m)['uturn_to_park'],0)

    def test_missing_class_rejected(self):
        with self.assertRaises(ValueError):samples(self.root,'train')

    def test_session_layout_and_split_overlap(self):
        for label in LABELS:
            folder=os.path.join(self.root,'train',label,'session1','roi')
            os.makedirs(folder)
            with open(os.path.join(folder,'a.png'),'wb') as f:f.write(label.encode())
        rows=samples(self.root,'train')
        self.assertEqual([y for _,y in rows],list(range(7)))
        with self.assertRaises(ValueError):check_overlap(dict(train=rows,val=rows))


if __name__=='__main__':unittest.main()
