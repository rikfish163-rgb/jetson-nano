import unittest
from test_direction_single_frame import DirectionSingleFrameTests

class AllSignThresholdTests(unittest.TestCase):
    core=DirectionSingleFrameTests.__dict__['core']
    def test_all_labels_accept_point_eight(self):
        for label in ('RED','GREEN','LEFT','RIGHT','STRAIGHT','UTURN'):
            c=self.core()
            if label=='GREEN':c.state='WAIT_GREEN'
            c.observe_sign(label,.8,1,1)
            if label=='GREEN':
                self.assertEqual(c.state,'STARTUP_STRAIGHT')
                self.assertEqual(c.sign_info['votes'],1)
            elif label=='RED':self.assertTrue(c.red)
            else:
                self.assertIsNone(c.pending)
                c.observe_sign(label,.8,1.2,1.2)
                self.assertEqual(c.pending,label)

    def test_parking_requires_three_consecutive_frames(self):
        c=self.core()
        self.assertEqual(c.cfg['parking_sign_votes'],3)
        for stamp in (1.0,1.1):
            c.observe_sign('PARKING',.9,stamp,stamp)
            self.assertIsNone(c.pending)
            self.assertEqual(c.sign_info['decision'],'parking_voting')
        c.observe_sign('PARKING',.9,1.2,1.2)
        self.assertEqual(c.pending,'PARKING')
        self.assertEqual(c.sign_info['votes'],3)

    def test_parking_vote_sequence_resets_on_other_or_rejected_frame(self):
        for label,confidence in (('',0.0),('GREEN',.9),('PARKING',.799999)):
            c=self.core()
            c.observe_sign('PARKING',.9,1.0,1.0)
            c.observe_sign('PARKING',.9,1.1,1.1)
            c.observe_sign(label,confidence,1.2,1.2)
            for stamp in (1.3,1.4):
                c.observe_sign('PARKING',.9,stamp,stamp)
                self.assertIsNone(c.pending)
            c.observe_sign('PARKING',.9,1.5,1.5)
            self.assertEqual(c.pending,'PARKING')

    def test_green_start_resets_on_other_label_or_low_confidence(self):
        for label,confidence in (('',0),('RED',.9),('RIGHT',.9),('GREEN',.799999)):
            c=self.core();c.state='WAIT_GREEN'
            c.cfg['sign_votes']=3
            c.observe_sign('GREEN',.9,1,1)
            c.observe_sign('GREEN',.9,1.1,1.1)
            c.observe_sign(label,confidence,1.2,1.2)
            for stamp in (1.3,1.4):
                c.observe_sign('GREEN',.9,stamp,stamp)
                self.assertEqual(c.state,'WAIT_GREEN')
            c.observe_sign('GREEN',.9,1.5,1.5)
            self.assertEqual(c.state,'STARTUP_STRAIGHT')

    def test_green_start_counts_only_fresh_distinct_frames_and_resets_after_gap(self):
        c=self.core();c.state='WAIT_GREEN'
        c.cfg['sign_votes']=3
        c.observe_sign('GREEN',.9,1,1)
        c.observe_sign('GREEN',.9,1,1.1)
        c.observe_sign('GREEN',.9,.9,1.1)
        c.observe_sign('GREEN',.9,1.1,10)
        self.assertEqual(c.state,'WAIT_GREEN')
        start=2+c.cfg['sign_timeout']
        for stamp in (start,start+.1):
            c.observe_sign('GREEN',.9,stamp,stamp)
            self.assertEqual(c.state,'WAIT_GREEN')
        c.observe_sign('GREEN',.9,start+.2,start+.2)
        self.assertEqual(c.state,'STARTUP_STRAIGHT')

    def test_red_requires_point_eight_even_during_uturn(self):
        for confidence in (.6,.7154243588447571,.799999,.8):
            c=self.core()
            c.state,c.action='UTURN','UTURN'
            c.observe_sign('RED',confidence,1,1)
            self.assertEqual(c.red,confidence>=.8)

    def test_all_labels_reject_below_threshold(self):
        for label in ('RED','GREEN','LEFT','RIGHT','STRAIGHT','UTURN','PARKING'):
            c=self.core();c.state='WAIT_GREEN' if label=='GREEN' else 'LANE'
            for t in (1,1.1,1.2):c.observe_sign(label,.799999,t,t)
            self.assertFalse(c.red)
            self.assertIsNone(c.pending)
            if label=='GREEN':self.assertEqual(c.state,'WAIT_GREEN')
