import imp
import os
import unittest
import cv2

HERE=os.path.dirname(__file__)


class ContinuousDashedPathTests(unittest.TestCase):
    def setUp(self):
        self.c=imp.load_source('continuous_dashed_test',os.path.join(HERE,'../scripts/camera_yihan_web.py'))

    def test_recorded_midpoint_normal_mixture_no_longer_fragments(self):
        mask=cv2.imread(os.path.join(HERE,'data/fragmented_dashed_curve.png'),0)
        for stamp in (1.,1.1,1.2):
            r=self.c.track_metric_lane(mask,source_stamp=stamp)
            diagnostic={}
            path=self.c.select_nearest_lane_path_segment(r['center_segments'],diagnostic)
            self.assertEqual(diagnostic['reason'],'ok',diagnostic)
            self.assertEqual(len(r['center_segments']),1)
            self.assertGreaterEqual(len(path),3)
            self.assertGreater(r['tracking_confidence'],.35)
            self.assertTrue(all(a['y']>b['y'] for a,b in zip(path,path[1:])))

    def points(self):
        return [dict(x=200+.1*y,y=y,source_y=y,window_index=i,
                     usable=True,score=.9,compensated=True,observed=False)
                for i,y in enumerate((380.,340.,300.,260.,220.,180.,140.))]

    def test_single_outlier_removed_and_short_internal_gap_filled(self):
        pts=self.points()
        pts[3]['x']+=100
        pts[1]=None
        result=self.c.fit_continuous_center_points(pts)
        used=[p for p in result if p]
        self.assertGreaterEqual(len(used),6)
        for p in used:
            self.assertAlmostEqual(p['x'],200+.1*p['y'],places=5)
            self.assertTrue(140<=p['y']<=380)

    def test_no_extrapolation_and_no_bridge_over_large_hole(self):
        pts=self.points()
        for i in (2,3,4):pts[i]=None
        result=self.c.fit_continuous_center_points(pts)
        self.assertFalse(any(p and 180<p['y']<340 for p in result))
        short=self.c.fit_continuous_center_points(self.points()[2:5])
        self.assertFalse(any(p and not 220<=p['y']<=300 for p in short))


if __name__=='__main__':unittest.main()
