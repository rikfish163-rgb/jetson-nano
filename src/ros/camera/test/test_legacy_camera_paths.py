"""Recorded legacy image corpus checked against unified path invariants."""
import imp
import json
import os
import unittest
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))


class LegacyCameraPathsTests(unittest.TestCase):
    def test_without_dashed_identity_uses_one_continuous_geometry(self):
        with open(os.path.join(HERE, 'data', 'legacy_lane_paths.json')) as stream:
            golden = json.load(stream)
        for index, scenario in enumerate(golden['scenarios']):
            camera = imp.load_source('legacy_camera_check_%d' % index,
                os.path.join(HERE, '..', 'scripts', 'camera_yihan_web.py'))
            # Exercise the same fitter when semantic identity is unavailable.
            camera.find_dashed_left_boundary = lambda mask: None
            for frame in scenario:
                mask = cv2.imread(os.path.join(HERE, 'data', frame['image']), 0)
                result = camera.track_metric_lane(mask)
                path = camera.select_nearest_lane_path_segment(result['center_segments'])
                points = [camera.bev_point_to_vehicle_m(p['x'], p['y']) for p in path]
                self.assertEqual(result['lane_mode'], frame['lane_mode'], frame['image'])
                if frame['points']:
                    self.assertGreaterEqual(len(points), 3, frame['image'])
                self.assertTrue(all(a[0]<b[0] for a,b in zip(points,points[1:])))
                centers=[p for p in result['center_points'] if p]
                self.assertLessEqual(len(set(p['reference_side'] for p in centers)),1)
                self.assertTrue(all(p['center_method']=='fitted_boundary_normal' for p in centers))


if __name__ == '__main__':
    unittest.main()
