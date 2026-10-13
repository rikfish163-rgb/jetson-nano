"""No-actuator regressions for the requested fixed bypass and handoff."""
import os, unittest
from robot.common.config import load_config
from robot.common.contracts import encode_command, validate_config
from robot.master.controller import Controller
from test_continuous_obstacles import _SyntheticScan

class FixedBypassSequenceTests(unittest.TestCase):
    PATH = [(.55,0.),(.70,0.),(.85,0.),(1.,0.)]
    def core(self):
        cfg = load_config(os.path.join(os.path.dirname(__file__),'../config'))
        cfg.update(wait_green=False,lidar_enabled=True,steering_command_scale_rad=.03)
        c = Controller(cfg)
        self.addCleanup(c.close)
        return c
    def tick(self,c,now,path=None):
        c.observe_lane(self.PATH if path is None else path,.8,now)
        c.scan = _SyntheticScan(now)
        speed,steer = c.tick(now)
        data = encode_command(speed,steer,c.cfg,0)
        return data['speed_raw'],data['steering_raw']
    def start(self,c):
        for now in (0.,.1,.2):
            c.observe_lane(self.PATH,.8,now)
            c.scan = _SyntheticScan(now,[(.8,0.)])
            result = c.execute('obstacle','begin_timed_bypass',now).value
        self.assertEqual(c.state,'TIMED_BYPASS')
        self.assertEqual(c.timed_bypass['phase'],'LEFT')
        self.assertEqual(result,(30,.03))
    def test_trigger_requires_three_fresh_scans(self):
        c = self.core()
        for now in (0.,.1):
            c.observe_lane(self.PATH,.8,now)
            c.scan = _SyntheticScan(now,[(.8,0.)])
            self.assertIsNone(c.execute('obstacle','begin_timed_bypass',now).value)
        c.observe_lane(self.PATH,.8,.2)
        c.scan = _SyntheticScan(.2,[(.8,0.)])
        self.assertEqual(c.execute('obstacle','begin_timed_bypass',.2).value,(30,.03))

    def test_disabled_bypass_keeps_lane_and_lidar_available(self):
        c=self.core();c.cfg['timed_bypass_enabled']=False
        for now in (0.,.1,.2,.3):
            c.observe_lane(self.PATH,.8,now)
            c.scan=_SyntheticScan(now,[(.8,0.)])
            self.assertGreater(c.tick(now)[0],0)
            self.assertEqual(c.state,'LANE')
            self.assertIsNone(c.timed_bypass)
            self.assertIsNone(c.timed_bypass_candidate)
        self.assertTrue(c.cfg['lidar_enabled'])
        self.assertTrue(c.cfg['left_bay_choice']['enabled'])

    def test_default_run_allows_only_one_completed_bypass(self):
        c=self.core()
        self.assertTrue(c.cfg['timed_bypass_enabled'])
        self.assertTrue(c.cfg['straight_lidar_once'])
        c.state='TIMED_BYPASS';c.action='BYPASS'
        c.timed_bypass=dict(phase='RETURN_LEFT',last=1.,
                           elapsed_s=c.cfg['timed_bypass_return_left_s']-.1,
                           trigger_world=(.8,0.))
        self.tick(c,1.2)
        self.assertTrue(c.timed_bypass_completed)
        # A different world-space target must also be ignored later in the run.
        c.pose=(5.,0.,0.)
        for now in (2.,2.1,2.2,2.3):
            c.observe_lane(self.PATH,.8,now)
            c.scan=_SyntheticScan(now,[(5.8,0.)])
            self.assertGreater(c.tick(now)[0],0)
            self.assertIsNone(c.timed_bypass)
            self.assertTrue(c.timed_bypass_completed)
    def test_fixed_three_arcs_and_immediate_normal_lane_handoff(self):
        c = self.core(); self.start(c)
        for i in range(1,15):
            self.assertEqual(self.tick(c,.2+i*.1),(30,22))
        self.assertEqual(self.tick(c,1.7),(30,-22))
        for i in range(1,30):
            self.assertEqual(self.tick(c,1.7+i*.1),(30,-22))
        self.assertEqual(self.tick(c,4.7),(30,22))
        for i in range(1,10):
            self.assertEqual(self.tick(c,4.7+i*.1),(30,22))
        self.tick(c,5.7)
        self.assertIsNone(c.timed_bypass)
        self.assertIsNone(c.action)
        self.assertEqual(c.state,'LANE')
        self.assertTrue(c.timed_bypass_completed)
        self.assertEqual(c.reason,'bypass_timed_lane_handoff')
        self.assertEqual(c.last_completed_action,'BYPASS')
    def test_missing_lane_does_not_shorten_arcs_but_stops_at_handoff(self):
        c = self.core(); self.start(c)
        for i in range(1,56):
            raw = self.tick(c,.2+i*.1,[])
            if i < 55:
                self.assertEqual(raw[0],30)
        self.assertEqual(raw,(0,0))
        self.assertIsNone(c.timed_bypass)
        self.assertEqual(c.reason,'no_initial_lane')
    def test_red_and_estop_pause_each_arc_timer(self):
        for phase in ('LEFT','RIGHT','RETURN_LEFT'):
            c = self.core(); c.state='TIMED_BYPASS'; c.action='BYPASS'
            c.timed_bypass=dict(phase=phase,last=1.,elapsed_s=.4)
            c.red=True
            self.assertEqual(self.tick(c,1.1),(0,0))
            self.assertAlmostEqual(c.timed_bypass['elapsed_s'],.4)
            c.red=False; c.estop=True
            self.assertEqual(self.tick(c,10.),(0,0))
            c.estop=False
            self.assertEqual(self.tick(c,20.)[0],30)
            self.assertAlmostEqual(c.timed_bypass['elapsed_s'],.4)
            self.tick(c,20.1)
            self.assertAlmostEqual(c.timed_bypass['elapsed_s'],.5)
    def test_bad_control_clock_stops_without_consuming_arc(self):
        c = self.core(); c.state='TIMED_BYPASS'; c.action='BYPASS'
        c.timed_bypass=dict(phase='RETURN_LEFT',last=1.,elapsed_s=.4)
        self.assertEqual(self.tick(c,2.),(0,0))
        self.assertEqual(c.reason,'bypass_control_clock_gap')
        self.assertAlmostEqual(c.timed_bypass['elapsed_s'],.4)
        self.assertEqual(self.tick(c,2.1),(30,22))
        self.assertAlmostEqual(c.timed_bypass['elapsed_s'],.4)
    def test_third_arc_tuning_controls_duration_speed_and_partial_steering(self):
        c = self.core()
        c.cfg.update(timed_bypass_return_left_s=.3,
                     timed_bypass_return_left_speed_raw=24,
                     timed_bypass_return_left_steering_raw=13)
        c.state='TIMED_BYPASS'; c.action='BYPASS'
        c.timed_bypass=dict(phase='RETURN_LEFT',last=1.,elapsed_s=0.)
        self.assertEqual(self.tick(c,1.1),(24,13))
        self.assertEqual(self.tick(c,1.2),(24,13))
        self.tick(c,1.3)
        self.assertIsNone(c.timed_bypass)
    def test_cached_sign_survives_and_once_only_policy_remains(self):
        c = self.core(); c.state='TIMED_BYPASS'; c.action='BYPASS'
        c.pending='STRAIGHT'; c.pending_at=.5
        c.timed_bypass=dict(phase='RETURN_LEFT',last=1.,elapsed_s=.9,
                           trigger_world=(.8,0.))
        self.tick(c,1.1)
        self.assertEqual(c.pending,'STRAIGHT')
        self.assertTrue(c.timed_bypass_completed)
        for now in (1.2,1.3,1.4):
            c.observe_lane(self.PATH,.8,now)
            c.scan = _SyntheticScan(now,[(.75,.1)])
            self.assertIsNone(c.execute('obstacle','begin_timed_bypass',now).value)
    def test_new_stage_parameters_are_validated(self):
        for key,values in (
            ('timed_bypass_return_left_s',(0,-1,6.1,float('nan'),True)),
            ('timed_bypass_return_left_speed_raw',(0,-1,31,1.5,True)),
            ('timed_bypass_return_left_steering_raw',(-23,23,1.5,True))):
            for value in values:
                c=self.core(); c.cfg[key]=value
                self.assertRaises(ValueError,validate_config,c.cfg)

if __name__ == '__main__': unittest.main()
