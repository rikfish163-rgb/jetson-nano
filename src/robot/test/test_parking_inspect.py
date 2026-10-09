import imp
import os
import unittest

inspect = imp.load_source('parking_inspect', os.path.join(os.path.dirname(__file__),
                         '..', 'parking', 'parking_inspect.py'))


class ScanReadinessTest(unittest.TestCase):
    def test_missing_stale_future_and_invalid_scans_cannot_confirm_free(self):
        self.assertFalse(inspect.scan_ready(None, 10))
        scan = type('ScanStub', (object,), dict(stamp=10., valid_rays=1440))()
        self.assertTrue(inspect.scan_ready(scan, 10.2))
        self.assertFalse(inspect.scan_ready(scan, 10.6))
        self.assertFalse(inspect.scan_ready(scan, 9.9))
        scan.valid_rays = 0
        self.assertFalse(inspect.scan_ready(scan, 10.2))

    def test_launch_has_no_driving_node(self):
        import xml.etree.ElementTree as ET
        path = os.path.join(os.path.dirname(__file__), '..', 'launch', 'parking_inspect.launch')
        root = ET.parse(path).getroot()
        self.assertEqual(set(n.attrib['type'] for n in root.findall('node')),
                         set(['usb_cam_node', 'parking_inspect.py']))
        self.assertEqual(len(root.findall('include')), 1)
        self.assertIn('ls01b_v2', root.find('include').attrib['file'])


if __name__ == '__main__':
    unittest.main()
