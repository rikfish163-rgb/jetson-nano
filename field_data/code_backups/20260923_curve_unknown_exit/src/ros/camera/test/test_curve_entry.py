"""The straight outside edge and curved inside edge identify curve entry."""
from test_lane_bend_memory import BendMemoryTests


class CurveEntryTests(BendMemoryTests):
    def frame(self, left, right, stamp):
        return self.lane.update_curve_entry(left,right,stamp)

    def test_entry_side_is_symmetric_and_needs_distinct_frames(self):
        for side,sign in (('LEFT',-1),('RIGHT',1)):
            self.lane.configure_lane_windows(40,.15)
            left=self.boundary(0 if sign<0 else .3,.3)
            right=self.boundary(-.3 if sign<0 else 0,-.3)
            for unused in range(4):
                self.assertEqual(self.frame(left,right,1.)['direction'],0)
            self.assertEqual(self.frame(left,right,1.1)['direction'],0)
            phase=self.frame(left,right,1.2)
            self.assertEqual(phase['direction'],sign)
            self.assertEqual(phase['entry_side'],side)

    def test_two_straight_or_missing_edge_never_start_curve(self):
        for t in (1.,1.1,1.2,1.3):
            self.assertEqual(self.frame(self.boundary(0,.3),self.boundary(0,-.3),t)['direction'],0)
        for t in (2.,2.1,2.2):
            self.assertEqual(self.frame(self.boundary(0,.3),[],t)['direction'],0)

    def test_entry_holds_through_opposite_noise_and_exits_on_straight(self):
        for t in (1.,1.1,1.2):
            self.frame(self.boundary(0,.3),self.boundary(-.3,-.3),t)
        for t in (1.3,1.4,1.5):
            self.assertEqual(self.frame(self.boundary(.3,.3),self.boundary(0,-.3),t)['direction'],-1)
        self.assertEqual(self.frame([],[],1.6)['direction'],-1)
        for i in range(4):
            self.assertEqual(self.frame(self.boundary(0,.3),self.boundary(0,-.3),1.7+i*.1)['direction'],-1)
        self.assertEqual(self.frame(self.boundary(0,.3),self.boundary(0,-.3),2.1)['direction'],0)

    def test_stale_entry_expires(self):
        for t in (1.,1.1,1.2):
            self.frame(self.boundary(0,.3),self.boundary(-.3,-.3),t)
        self.assertEqual(self.frame([],[],2.3)['direction'],0)

    def test_single_aligned_exit_edge_releases_without_requiring_missing_paint(self):
        for t in (1.,1.1,1.2):
            self.frame(self.boundary(0,.3),self.boundary(-.3,-.3),t)
        for i in range(5):
            phase=self.frame(self.boundary(0,.3),[],1.3+i*.1)
        self.assertEqual(phase['direction'],0)

    def test_real_tracker_publishes_entry_evidence(self):
        import cv2
        import numpy as np
        mask=np.zeros((400,480),np.uint8)
        for points in (self.boundary(0,.3),self.boundary(-.3,-.3)):
            poly=np.array([[int(p['x']),int(p['y'])] for p in points],np.int32)
            cv2.polylines(mask,[poly],False,255,7)
        for t in (1.,1.1,1.2):
            result=self.lane.track_metric_lane(mask,source_stamp=t)
        self.assertEqual(result['path_diagnostic']['curve_entry']['direction'],'RIGHT')
        self.assertEqual(result['path_diagnostic']['curve_entry']['entry_side'],'LEFT')
