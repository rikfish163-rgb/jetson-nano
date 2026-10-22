# -*- coding: utf-8 -*-
"""Single-main startup contracts; never starts ROS or vehicle nodes."""
import imp
import os
import sys
import tempfile
import types
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
from test_parking_main_guard import _install_import_stubs
_install_import_stubs()
if 'rospy' not in sys.modules:
    sys.modules['rospy'] = types.ModuleType('rospy')


class MainEntryTests(unittest.TestCase):
    def setUp(self):
        path = os.path.abspath(os.path.join(ROOT, '..', '..', 'robot', 'master', 'main.py'))
        self.module = imp.load_source('single_main_entry_test', path)
        self.params = {}
        fake = types.ModuleType('entry_fake_rospy')
        fake.has_param = lambda key: key in self.params
        fake.get_param = lambda key, default=None: self.params.get(key, default)
        fake.set_param = lambda key, value: self.params.__setitem__(key, value)
        self.module.rospy = fake

    def test_load_once_preserves_loaded_launch_config(self):
        self.params['/competition/config'] = {'marker_trigger_x': 0.6}
        self.module.load_competition_config('/missing/file')
        self.assertEqual(self.params['/competition/config'], {'marker_trigger_x': 0.6})

    def test_standalone_waits_for_explicit_enable(self):
        with tempfile.NamedTemporaryFile(mode='w') as stream:
            stream.write('sign_ttl: 5\nwait_green: false\nspeed_raw: {lane: 20}\n')
            stream.flush()
            self.module.load_competition_config(stream.name)
        self.assertTrue(self.params['~live'])
        self.assertFalse(self.params['~enabled'])
        self.assertTrue(self.params['/competition/config']['wait_green'])
        self.assertEqual(self.params['/competition/config']['sign_ttl'], 0)

    def test_shadow_flags_from_launch_are_preserved(self):
        self.params.update({'/competition/config': {}, '~live': False, '~enabled': False})
        self.module.load_competition_config('/missing/file')
        self.assertFalse(self.params['~live'])
        self.assertFalse(self.params['~enabled'])


if __name__ == '__main__':
    unittest.main()
