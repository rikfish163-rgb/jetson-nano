import math
import os
import subprocess
import sys
import unittest
from car_calibration import CASES,command,radius_from_chord


class CalibrationTests(unittest.TestCase):
    def test_six_cases_use_bounded_existing_probe(self):
        for case,(direction,steer) in CASES.items():
            cmd=command('/workspace',case,26,3.)
            self.assertEqual(cmd[cmd.index('--speed')+1],str(direction*26))
            self.assertEqual(cmd[cmd.index('--steering')+1],str(steer))
            self.assertIn('bench_command.py',cmd[1])
        for speed,seconds in [(0,3),(31,3),(26,6),(26,float('nan'))]:
            with self.assertRaises(ValueError):command('/workspace','forward_zero',speed,seconds)

    def test_arc_geometry(self):
        # A 60-degree minor arc of radius 65 cm.
        self.assertAlmostEqual(radius_from_chord(65,65*(1-math.cos(math.pi/6))),65)
        with self.assertRaises(ValueError):radius_from_chord(20,30)

    def test_dry_runs_need_no_ros(self):
        script=os.path.join(os.path.dirname(__file__),'car_calibration.py')
        for case in CASES:
            self.assertEqual(subprocess.call([sys.executable,script,'run',case],
                             stdout=open(os.devnull,'w')),0)


if __name__=='__main__':unittest.main()
