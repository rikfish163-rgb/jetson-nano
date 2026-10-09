"""Team config boundaries and complete first-party source assignment."""
import imp
import os
import unittest
from test_core import CONFIG
from robot.common.config import check_override
from robot.common.config import merge
from robot.common.config import read_mapping

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
TOOL = imp.load_source('module_workspace_tests_tool', os.path.join(ROOT, 'tools', 'module_workspace.py'))


class WorkspaceModuleTest(unittest.TestCase):
    def test_each_source_has_known_owner(self):
        spec = TOOL.catalog()
        files = list(TOOL.source_files())
        self.assertGreater(len(files), 100)
        for path in files:
            self.assertIn(TOOL.owner_of(path, spec), spec['modules'], path)

    def test_new_package_requires_owner(self):
        with self.assertRaises(ValueError):
            TOOL.owner_of('src/unassigned_package/controller.py', TOOL.catalog())

    def test_lane_cannot_change_another_speed_or_steering(self):
        spec = TOOL.catalog()
        check_override('lane', {'speed_raw': {'lane': 14}}, spec)
        with self.assertRaises(ValueError):
            check_override('lane', {'speed_raw': {'parking': 14}}, spec)
        with self.assertRaises(ValueError):
            check_override('lane', {'steering_sign': -1}, spec)

    def test_nested_override_preserves_other_teams_and_base(self):
        old = CONFIG['speed_raw']['lane']
        result = merge(CONFIG, {'speed_raw': {'lane': 9}})
        self.assertEqual(result['speed_raw']['lane'], 9)
        self.assertEqual(result['speed_raw']['parking'], CONFIG['speed_raw']['parking'])
        self.assertEqual(CONFIG['speed_raw']['lane'], old)

    def test_parameter_has_one_owner(self):
        owners = {}
        for owner, definition in TOOL.catalog()['modules'].items():
            for name in definition['parameters']:
                self.assertNotIn(name, owners, name)
                owners[name] = owner

    def test_fixture_cannot_replace_executor(self):
        import json
        import tempfile
        handle, path = tempfile.mkstemp(suffix='.json')
        os.close(handle)
        self.addCleanup(os.unlink, path)
        with open(path, 'w') as stream:
            json.dump(dict(module='motion', operation='stop', args=['fixture'],
                           state={'executor': 'not a resource'}), stream)
        with self.assertRaises(ValueError):
            TOOL.replay(path)
