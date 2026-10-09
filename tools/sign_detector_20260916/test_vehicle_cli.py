"""Nano launch argument checks with a fake roslaunch; no nodes or actuators."""
from __future__ import print_function
import os
import shutil
import subprocess
import tempfile
import unittest


SCRIPT=os.path.join(os.path.dirname(os.path.abspath(__file__)),'run_yolo_vehicle.sh')


class VehicleParkingArgumentsTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.mkdtemp(prefix='vehicle-cli-test-')
        self.addCleanup(shutil.rmtree,self.tmp)
        # Block ROS setup and every real roslaunch path, even if the car is
        # already running. Only this test's executable can handle the launch.
        self.envfile=os.path.join(self.tmp,'bash_env')
        with open(self.envfile,'w') as stream:
            stream.write('source() { :; }\npgrep() { return 1; }\n')
        executable=os.path.join(self.tmp,'roslaunch')
        with open(executable,'w') as stream:
            stream.write('#!/bin/bash\nprintf "MOCK_ROSLAUNCH\\n"\nprintf "%s\\n" "$@"\n')
        os.chmod(executable,0o755)

    def run_args(self,*args):
        env=dict(os.environ,PATH=self.tmp+':/usr/bin:/bin',BASH_ENV=self.envfile)
        process=subprocess.Popen(['/bin/bash',SCRIPT]+list(args),env=env,
            stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        stdout,stderr=process.communicate()
        return process.returncode,stdout.decode('utf-8'),stderr.decode('utf-8')

    def launch_args(self,*args):
        code,out,err=self.run_args(*args)
        self.assertEqual(code,0,err)
        self.assertIn('MOCK_ROSLAUNCH\n',out)
        rows=out.split('MOCK_ROSLAUNCH\n',1)[1].splitlines()
        return rows

    def test_all_parallel_slots_select_saved_profile(self):
        for slot in ('P1','P2','P3'):
            rows=self.launch_args('parking_slot:='+slot)
            self.assertIn('parking_slot:='+slot,rows)
            self.assertIn('parking_mode:=timed_sequence',rows)
            self.assertNotIn('parking_mode:=forward_plan',rows)

    def test_straight_flag_is_consumed_and_forwarded_for_both_bays(self):
        for slot in ('P4','P5'):
            rows=self.launch_args('--S','parking_slot:='+slot)
            self.assertNotIn('--S',rows)
            self.assertIn('parking_entry_style:=S',rows)
            self.assertIn('parking_slot:='+slot,rows)

    def test_turn_flag_is_consumed_and_forwarded(self):
        rows=self.launch_args('parking_slot:=P5','--T')
        self.assertNotIn('--T',rows)
        self.assertIn('parking_entry_style:=T',rows)
        self.assertIn('parking_enabled:=true',rows)

    def test_default_is_explicit_p4_straight_entry(self):
        rows=self.launch_args()
        self.assertIn('parking_slot:=P4',rows)
        self.assertIn('parking_entry_style:=S',rows)

    def test_conflicting_flags_never_launch(self):
        code,out,err=self.run_args('--S','--T','parking_slot:=P4')
        self.assertNotEqual(code,0)
        self.assertNotIn('MOCK_ROSLAUNCH',out)

    def test_perpendicular_flags_do_not_change_parallel_algorithm(self):
        code,out,err=self.run_args('parking_slot:=P3','--T')
        self.assertNotEqual(code,0)
        self.assertNotIn('MOCK_ROSLAUNCH',out)

    def test_unknown_slot_and_conflicting_legacy_mode_never_launch(self):
        for args in [('parking_slot:=P6',),('parking_slot:=P3','parking_mode:=forward_plan')]:
            code,out,err=self.run_args(*args)
            self.assertNotEqual(code,0)
            self.assertNotIn('MOCK_ROSLAUNCH',out)

    def test_other_vehicle_options_survive_argument_translation(self):
        rows=self.launch_args('parking_slot:=P2','parking_enabled:=false','lane_speed_raw:=32')
        self.assertIn('lane_speed_raw:=32',rows)
        self.assertEqual([v for v in rows if v.startswith('parking_enabled:=')][-1],'parking_enabled:=false')


if __name__=='__main__':unittest.main()
