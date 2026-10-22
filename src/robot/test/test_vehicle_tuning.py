"""One-file tuning affects production commands; no ROS or actuators."""
import copy, os, tempfile, unittest, yaml
from robot.common.config import load_config
from robot.common.contracts import encode_command, validate_config
from robot.master.controller import Controller
from robot.uturn.timed import TimedUturn
from test_continuous_obstacles import _SyntheticScan
CONFIG_DIR=os.path.abspath(os.path.join(os.path.dirname(__file__),'../config'))

class VehicleTuningTests(unittest.TestCase):
    def config(self):
        cfg=load_config(CONFIG_DIR)
        cfg.update(wait_green=False,lidar_enabled=False,steering_command_scale_rad=.03)
        return cfg
    def core(self,cfg):
        c=Controller(cfg); self.addCleanup(c.close); return c
    def raw(self,c,command):
        d=encode_command(command[0],command[1],c.cfg,0)
        return d['speed_raw'],d['steering_raw']
    def test_custom_file_wins_and_unknown_key_rejected(self):
        fd,path=tempfile.mkstemp(suffix='.yaml'); os.close(fd)
        self.addCleanup(os.unlink,path)
        tuning={'left_timed_entry_speed_raw':24,'straight_speed_raw':25}
        with open(path,'w') as f: yaml.safe_dump(tuning,f)
        cfg=load_config(CONFIG_DIR,tuning_path=path)
        self.assertEqual(cfg['left_timed_entry_speed_raw'],24)
        self.assertEqual(cfg['straight_speed_raw'],25)
        with open(path,'w') as f: yaml.safe_dump({'misspelled_speed_raw':24},f)
        self.assertRaises(ValueError,load_config,CONFIG_DIR,tuning_path=path)
    def test_left_speed_and_partial_steering_reach_output(self):
        cfg=self.config(); cfg.update(left_timed_entry_speed_raw=24,left_timed_entry_steering_raw=7)
        c=self.core(cfg); c.start_follow([(0,0,0,1,0)],'LEFT',0)
        self.assertEqual(self.raw(c,c.execute('turn','left_lock_tick',0).value),(24,7))
    def test_right_exit_speed_reaches_output(self):
        cfg=self.config(); cfg['right_timed_exit_speed_raw']=17
        c=self.core(cfg); c.action='RIGHT'; c.state='MANEUVER'; c.front_marker_stamp=1
        c.right_lock={'phase':'WAIT_LANE','exit_started':1}
        self.assertEqual(self.raw(c,c.execute('turn','right_lock_tick',1).value),(17,0))
    def test_bypass_independent_speed_and_partial_steering(self):
        for phase,speed,steering in (('LEFT',23,15),('RIGHT',25,-17)):
            cfg=self.config(); cfg['lidar_enabled']=True
            prefix='timed_bypass_'+phase.lower()
            cfg[prefix+'_speed_raw']=speed; cfg[prefix+'_steering_raw']=steering
            c=self.core(cfg); c.state='TIMED_BYPASS'; c.action='BYPASS'
            c.timed_bypass=dict(phase=phase,last=1.,elapsed_s=.5); c.scan=_SyntheticScan(1.1)
            self.assertEqual(self.raw(c,c.tick(1.1)),(speed,steering))
    def test_uturn_partial_steering_and_time_preserve_pauses(self):
        cfg=self.config(); cfg['uturn_trial_sequence'][1].update(speed=24,steering=16,seconds=2.3)
        steps=TimedUturn(cfg).steps
        segment=next(s for s in steps if s[0]=='SEGMENT_2')
        self.assertEqual(segment[1],2.3)
        d=encode_command(segment[2][0],segment[2][1],cfg,0)
        self.assertEqual((d['speed_raw'],d['steering_raw']),(24,16))
        self.assertEqual(sum(s[1] for s in steps if s[0].startswith('GEAR_PAUSE')),2.8)
    def test_invalid_new_knobs_and_inactive_parking_profile_rejected(self):
        for key,value in [('right_timed_exit_speed_raw',0),('maneuver_search_speed_raw',True),
                          ('uturn_exit_curve_speed_raw',31),('timed_bypass_left_steering_raw',23)]:
            cfg=self.config(); cfg[key]=value
            self.assertRaises(ValueError,validate_config,cfg)
        cfg=self.config(); cfg['parking_timed']['sequences']['P2'][0]['seconds']=-1
        self.assertRaises(ValueError,validate_config,cfg)

if __name__=='__main__': unittest.main()
