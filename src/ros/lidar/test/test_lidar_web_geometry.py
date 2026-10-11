"""Host-side geometry checks; no ROS installation or hardware required."""
import importlib.util
import json
import math
import os
import sys
import types
import unittest
from unittest.mock import patch

path = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'lidar_web_test.py')
messages = types.ModuleType('sensor_msgs.msg')
messages.LaserScan = object
with patch.dict(sys.modules, {'rospy': types.ModuleType('rospy'),
                             'sensor_msgs': types.ModuleType('sensor_msgs'),
                             'sensor_msgs.msg': messages}):
    spec = importlib.util.spec_from_file_location('lidar_web_test', path)
    viewer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(viewer)


class GeometryTest(unittest.TestCase):
    def test_midpoint_and_noise(self):
        points = [[0.30+i*.01, -.4 + .002*math.sin(i)] for i in range(41)]
        points += [[.1, -.8], [.9, -.1], [-.2, -.4], [.4, .4], [2, -2]]
        result = viewer.detect_line(points)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result['midpoint'][0], .5, places=2)
        self.assertAlmostEqual(result['midpoint'][1], -.4, places=2)
        self.assertAlmostEqual(result['length'], .4, places=2)
        self.assertEqual(result['count'], 41)

    def test_vertical_line(self):
        result = viewer.detect_line([[.4, -.2-i*.01] for i in range(40)])
        self.assertAlmostEqual(result['midpoint'][0], .4)
        self.assertAlmostEqual(result['midpoint'][1], -.395)

    def test_disconnected_segments_not_joined(self):
        points = [[.1+i*.01, -.2] for i in range(16)]
        points += [[.55+i*.01, -.2] for i in range(26)]
        result = viewer.detect_line(points)
        self.assertAlmostEqual(result['length'], .25)
        self.assertAlmostEqual(result['midpoint'][0], .675)

    def test_missing_or_outside_line(self):
        for points in ([], [[.1+i*.01, .4] for i in range(40)],
                       [[1.2+i*.01, -.4] for i in range(40)],
                       [[.4+i*.001, -.4] for i in range(9)]):
            result = viewer.measure(points)
            self.assertIsNone(result['line'])
            self.assertEqual(result['boxes'], [])
            self.assertIsNone(result['difference'])

    def test_box_translation_and_shared_boundary(self):
        # Dyadic midpoint keeps exact shared-edge test independent of trig noise.
        line = {'midpoint': [.5, -.5]}
        points = [[.2, -.7], [-.2, -.7], [-.5, -.7], [-.9, -.7],
                  [.6, -.7], [-.4, -.5]]
        with patch.object(viewer, 'detect_line', return_value=line):
            result = viewer.measure(points)
        self.assertEqual([b['count'] for b in result['boxes']], [2, 2])
        self.assertEqual(result['difference'], 0)
        self.assertAlmostEqual(result['boxes'][1]['bounds'][0], -.9)

    def test_scan_filter_and_loss_clears_result(self):
        v = viewer.Viewer()
        self.assertIsNone(json.loads(v.snapshot())['age'])
        scan = types.SimpleNamespace(ranges=[float('nan'), float('inf'), 0, 8, 1],
                                     range_min=.1, range_max=5, angle_min=0,
                                     angle_increment=.1,
                                     header=types.SimpleNamespace(frame_id='laser',
                                       stamp=types.SimpleNamespace(to_sec=lambda: 123)))
        v.observe(scan)
        result = json.loads(v.snapshot())
        self.assertEqual(len(result['points']), 1)
        self.assertAlmostEqual(result['points'][0][0], math.cos(.4))
        self.assertIsNone(result['line'])
        self.assertIsNone(result['difference'])


if __name__ == '__main__':
    unittest.main()
