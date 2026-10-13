import os
import shutil
import subprocess
import tempfile
import unittest

ROOT = os.environ.get('VEHICLE_ROOT', '/home/nano/robocup_ws')


class AutostartTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp)
        self.log = os.path.join(self.tmp, 'calls')
        for name in ('sudo', 'systemctl'):
            path = os.path.join(self.tmp, name)
            with open(path, 'w') as stream:
                stream.write('#!/bin/sh\nprintf "%s\\n" "$*" >> "$CALL_LOG"\n')
            os.chmod(path, 0o755)
        self.env = dict(os.environ, PATH=self.tmp+':'+os.environ['PATH'], CALL_LOG=self.log)

    def invoke(self, args):
        process = subprocess.Popen(['bash', os.path.join(ROOT, 'runall.sh')]+args,
                                   env=self.env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        process.communicate()
        return process.returncode

    def calls(self):
        if not os.path.exists(self.log):
            return ''
        with open(self.log) as stream:
            return stream.read()

    def test_enable_installs_without_starting(self):
        self.assertEqual(self.invoke(['--enable', 'true']), 0)
        calls = self.calls()
        self.assertIn('install -m 0644', calls)
        self.assertIn('systemctl daemon-reload', calls)
        self.assertIn('systemctl enable robocup-vehicle.service', calls)
        self.assertNotIn('--now', calls)
        self.assertNotIn('systemctl start', calls)

    def test_disable_only_disables_future_boots(self):
        self.assertEqual(self.invoke(['--enable=false']), 0)
        self.assertIn('systemctl disable robocup-vehicle.service', self.calls())
        self.assertNotIn('install ', self.calls())

    def test_invalid_or_mixed_options_do_nothing(self):
        for args in (['--enable'], ['--enable', 'yes'],
                     ['--enable', 'true', 'wait_green:=false']):
            self.assertNotEqual(self.invoke(args), 0)
        self.assertEqual(self.calls(), '')

    def test_service_forces_green_and_has_no_restart(self):
        with open(os.path.join(ROOT, 'tools/robocup-vehicle.service')) as stream:
            unit = stream.read()
        self.assertIn('User=nano', unit)
        self.assertIn('runall.sh wait_green:=true', unit)
        self.assertIn('Restart=no', unit)
        self.assertIn('WantedBy=multi-user.target', unit)


if __name__ == '__main__':
    unittest.main()
