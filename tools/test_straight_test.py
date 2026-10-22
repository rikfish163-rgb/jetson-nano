from __future__ import print_function
import os
import subprocess
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from straight_test import probe_command


class StraightTest(unittest.TestCase):
    def test_only_zero_steering_is_requested(self):
        command = probe_command('/workspace', 26, 2.0)
        self.assertEqual(command[command.index('--steering') + 1], '0')
        self.assertEqual(command[command.index('--speed') + 1], '26')
        self.assertEqual(command[command.index('--seconds') + 1], '2.0')

    def test_invalid_inputs_and_missing_confirmation_never_start_motion(self):
        script = os.path.join(os.path.dirname(__file__), 'straight_test.py')
        for arguments in [[], ['--speed', '31', '--confirm-clear'],
                          ['--speed', '-26', '--confirm-clear'],
                          ['--seconds', 'nan', '--confirm-clear'],
                          ['--seconds', 'inf', '--confirm-clear'],
                          ['--seconds', '0', '--confirm-clear'],
                          ['--seconds', '6', '--confirm-clear']]:
            process = subprocess.Popen([sys.executable, script] + arguments,
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            process.communicate()
            self.assertEqual(process.returncode, 2)


if __name__ == '__main__':
    unittest.main()
