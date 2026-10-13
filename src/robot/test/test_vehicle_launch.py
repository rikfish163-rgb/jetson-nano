"""Resolve and expand production launch settings without starting any nodes."""
import copy
import imp
import os
import subprocess
import tempfile
import unittest
import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
launcher = imp.load_source('vehicle_launch_test', os.path.join(ROOT, 'tools/vehicle_launch.py'))


class VehicleLaunchTests(unittest.TestCase):
    def setUp(self):
        self.path = os.path.join(ROOT, 'src/robot/config/vehicle_tuning.yaml')
        with open(self.path) as stream:
            self.tuning = yaml.safe_load(stream)

    def temporary(self, tuning):
        fd, path = tempfile.mkstemp(suffix='.yaml')
        os.close(fd)
        self.addCleanup(os.unlink, path)
        with open(path, 'w') as stream:
            yaml.safe_dump(tuning, stream)
        return path

    def expand(self, extra):
        unused, values = launcher.resolve(ROOT, extra)
        out = subprocess.check_output(['roslaunch', '--dump-params',
            'robocup_competition', 'full.launch'] + [k+':='+v for k,v in values.items()])
        return yaml.safe_load(out)

    def test_yaml_changes_reach_controller_and_node_parameters(self):
        t = copy.deepcopy(self.tuning)
        t['speed_raw']['lane'] = 31
        t['parking_side_lidar_enabled'] = False
        t['startup_follow_lane'] = True
        t['lookahead'] = .85
        t['left_timed_turn_s'] = 3.7
        t['launch']['front_fps'] = 11
        d = self.expand(['vehicle_tuning:='+self.temporary(t)])
        for key, value in [('speed_raw/lane',31),('parking_side_lidar_enabled',False),
                           ('startup_follow_lane',True),('lookahead',.85),('left_timed_turn_s',3.7)]:
            self.assertEqual(d['/competition/config/'+key],value)
        self.assertEqual(d['/front/front_camera/framerate'],11)
        unused, values=launcher.resolve(ROOT,['vehicle_tuning:='+self.temporary(t)])
        nodes=subprocess.check_output(['roslaunch','--nodes','robocup_competition','full.launch']+
                                      [k+':='+v for k,v in values.items()])
        self.assertNotIn('parking_side_lidar',nodes)

    def test_explicit_arguments_override_yaml(self):
        d=self.expand(['lane_speed_raw:=27','parking_side_lidar_enabled:=false',
                       '--follow-lane-start','--no-bypass'])
        self.assertEqual(d['/competition/config/speed_raw/lane'],27)
        self.assertFalse(d['/competition/config/parking_side_lidar_enabled'])
        self.assertTrue(d['/competition/config/startup_follow_lane'])
        self.assertFalse(d['/competition/config/timed_bypass_enabled'])

    def test_duplicate_control_settings_in_launch_are_rejected(self):
        t=copy.deepcopy(self.tuning)
        t['launch']['lane_speed_raw']=28
        self.assertRaises(ValueError,launcher.resolve,ROOT,
                          ['vehicle_tuning:='+self.temporary(t)])
        self.assertRaises(ValueError,launcher.resolve,ROOT,['misspelled_speed:=28'])

    def test_offline_config_agrees_with_startup_tuning(self):
        from robot.common.config import load_config
        c=load_config(os.path.dirname(self.path))
        self.assertEqual(c['speed_raw']['lane'],40)
        self.assertNotIn('launch',c)


if __name__=='__main__':unittest.main()
