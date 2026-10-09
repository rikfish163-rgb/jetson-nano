"""Disabled side-parking must not load or influence ordinary course control."""
import copy
import os
import subprocess
import sys
import tempfile
import shutil
import unittest
import yaml
from distutils.spawn import find_executable
from robot.common.contracts import validate_config
from robot.master.controller import Controller
from robot.master.runtime import ModuleRuntime

ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
with open(os.path.join(ROOT,'config','competition.yaml')) as stream:
    CONFIG=yaml.safe_load(stream)


class ParallelIsolationTests(unittest.TestCase):
    @unittest.skipUnless(find_executable('roslaunch'),'requires sourced ROS')
    def test_broken_parallel_yaml_is_never_opened_by_regular_launch(self):
        root=tempfile.mkdtemp(prefix='parallel-isolation-test-')
        self.addCleanup(shutil.rmtree,root)
        for name in ('camera','lane','lidar','master','motion','obstacle','parking','signs','turn','uturn'):
            os.symlink(os.path.join(ROOT,name),os.path.join(root,name))
        os.mkdir(os.path.join(root,'parallel_parking'))
        with open(os.path.join(root,'parallel_parking','config.yaml'),'w') as stream:
            stream.write('invalid_yaml: [')
        for mode in ('forward_center','reverse_plan','parallel_reverse'):
            p=subprocess.Popen(['roslaunch','--dump-params',os.path.join(ROOT,'launch','stack.launch'),
                                'module_config_dir:='+root,'parking_mode:='+mode],
                               stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            out,err=p.communicate()
            # Melodic dump-params may return zero even on a YAML load error.
            if mode=='parallel_reverse':
                self.assertIn(b'contains invalid YAML',err)
            else:
                self.assertEqual(p.returncode,0)
                self.assertEqual(err,b'')
                cfg=yaml.safe_load(out)
                self.assertEqual(cfg['/competition/config/parking_mode'],mode)
                self.assertFalse(any(k.startswith('/competition/config/parallel_parking_') for k in cfg))

    def test_regular_controller_does_not_import_parallel_module(self):
        source='''
import sys, yaml
from robot.master.controller import Controller
with open(sys.argv[1]) as stream: cfg=yaml.safe_load(stream)
cfg['parking_mode']='forward_center'
c=Controller(cfg)
assert not any(n.startswith('robot.parallel_parking') for n in sys.modules)
assert 'parallel_parking' not in c._runtime.modules
c.close()
'''
        subprocess.check_call([sys.executable,'-c',source,os.path.join(ROOT,'config','competition.yaml')])

    def test_disabled_parallel_parameters_do_not_block_regular_modes(self):
        for mode in ('forward_center','reverse_plan','forward_white'):
            cfg=copy.deepcopy(CONFIG)
            cfg.update(parking_mode=mode,parking_slot='AUTO',lidar_enabled=True,
                       parallel_parking_speed_raw='unused_invalid',parallel_parking_timeout_s=-1)
            validate_config(cfg)
        cfg['parking_mode']='parallel_reverse'
        with self.assertRaises(ValueError):validate_config(cfg)

    def test_disabled_scene_callback_has_no_effect(self):
        cfg=dict(copy.deepcopy(CONFIG),parking_mode='reverse_plan')
        c=Controller(cfg);self.addCleanup(c.close)
        before=copy.deepcopy(c.parallel_scene)
        c.observe_parallel_scene({'unrelated':'malformed side parking data'},1.)
        self.assertEqual(c.parallel_scene,before)
        self.assertIsNone(c.parallel_future)

    def test_only_parallel_mode_installs_parallel_operations(self):
        c=Controller(dict(copy.deepcopy(CONFIG),parking_mode='parallel_reverse'))
        self.addCleanup(c.close)
        self.assertIn('parallel_parking',c._runtime.modules)

    def test_ordinary_commands_match_runtime_with_unused_parallel_module(self):
        for mode in ('forward_center','reverse_plan'):
            cfg=dict(copy.deepcopy(CONFIG),parking_mode=mode,wait_green=False,lidar_enabled=False)
            ordinary=Controller(copy.deepcopy(cfg));reference=Controller(copy.deepcopy(cfg))
            reference._runtime=ModuleRuntime(parallel_enabled=True)
            self.addCleanup(ordinary.close);self.addCleanup(reference.close)
            for i in range(30):
                t=1.+i*.1
                for c in (ordinary,reference):
                    c.observe_lane([(.3,.04),(.5,.07),(.8,.12)],.95,t)
                    if i in (3,4):c.observe_sign('LEFT',.99,t,t)
                    if i in (8,9):c.observe_sign('RIGHT',.99,t,t)
                    if i in (15,16):c.observe_sign('PARKING',.99,t,t)
                self.assertEqual(ordinary.tick(t),reference.tick(t))
                for field in ('state','reason','pending','action','lane_source'):
                    self.assertEqual(getattr(ordinary,field),getattr(reference,field))


if __name__=='__main__':unittest.main()
