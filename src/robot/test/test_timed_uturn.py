import os,sys,unittest,yaml
from blue_test_helpers import enter_blue_action
ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.join(ROOT,'src'))
from robot.uturn.timed import TimedUturn
from robot.common.contracts import encode_command
from robot.master.controller import Controller

class TrialTests(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(ROOT,'config','competition.yaml')) as f:self.cfg=yaml.safe_load(f)
        self.cfg['steering_command_scale_rad']=.1

    def test_raw_commands_and_reversal_pauses(self):
        task=TimedUturn(self.cfg)
        self.assertEqual([encode_command(s[2][0],s[2][1],self.cfg,0)['steering_raw'] for s in task.steps],[0,22,0,-22,0,22,0])
        self.assertLess(task.steps[3][2][0],0)
        self.assertGreaterEqual(task.steps[2][1],.3)
        self.assertGreaterEqual(task.steps[4][1],.3)

    def test_right_is_mirrored(self):
        self.cfg['uturn_trial_side']='right'
        task=TimedUturn(self.cfg)
        self.assertLess(task.steps[1][2][1],0)
        self.assertGreater(task.steps[3][2][1],0)

    def test_reduced_raw_steering_keeps_fraction_under_python2(self):
        self.cfg['uturn_trial_steering_raw']=16
        command=TimedUturn(self.cfg).steps[1][2]
        self.assertEqual(encode_command(command[0],command[1],self.cfg,0)['steering_raw'],16)

    def test_model_endpoint_is_reversed_and_shifted(self):
        from robot.common.geometry import bicycle
        task=TimedUturn(self.cfg)
        pose=(0,0,0)
        import math
        for index in (0,1,3,5):
            _,duration,command=task.steps[index]
            speed=.208 if command[0]>0 else -.182
            angle=0 if not command[1] else math.copysign(math.atan(self.cfg['wheelbase']/.65),command[1])
            for _ in range(200):pose=bicycle(pose,speed*duration/200,angle,self.cfg['wheelbase'])
        self.assertAlmostEqual(pose[0],.12,delta=.01)
        self.assertAlmostEqual(pose[1],.6,delta=.01)
        self.assertAlmostEqual(abs(pose[2]),math.pi,delta=.01)

    def test_red_pause_freezes_trial_progress(self):
        self.cfg.update(uturn_trial_enabled=True,wait_green=False,lidar_enabled=False)
        c=Controller(self.cfg);self.addCleanup(c.close)
        c.state='UTURN';c.action='UTURN';c.action_started=1;c.uturn={};c.wait_until=0
        c.front_marker_stamp=1;c.tick(1)
        c.front_marker_stamp=1.1;c.tick(1.1)
        elapsed=c.timed_uturn.elapsed
        c.red=True;self.assertEqual(c.tick(1.2),(0,0))
        c.red=False;c.front_marker_stamp=2;c.tick(2)
        self.assertAlmostEqual(c.timed_uturn.elapsed,elapsed)

    def test_pause_does_not_count_stopped_time(self):
        task=TimedUturn(self.cfg)
        task.command(1);task.command(1.1);elapsed=task.elapsed
        task.pause();task.command(100)
        self.assertAlmostEqual(task.elapsed,elapsed)

    def test_clock_gap_rejected(self):
        task=TimedUturn(self.cfg);task.command(1)
        with self.assertRaises(ValueError):task.command(2)

    def test_finishes_zero(self):
        task=TimedUturn(self.cfg)
        for i in range(1000):task.command(i*.05)
        self.assertTrue(task.done)
        self.assertEqual(task.command(50),(0,0))

    def test_invalid_parameters(self):
        for key,val in [('uturn_trial_side','auto'),('uturn_trial_forward_mps',0),('uturn_trial_steering_raw',23),('uturn_trial_first_gain',float('nan'))]:
            cfg=dict(self.cfg);cfg[key]=val
            with self.assertRaises(ValueError):TimedUturn(cfg)

    def test_sign_blue_dispatch_and_stop_at_finish(self):
        self.cfg.update(uturn_trial_enabled=True,wait_green=False,lidar_enabled=False)
        c=Controller(self.cfg);self.addCleanup(c.close)
        c.observe_sign('UTURN',.9,.95,.95)
        c.observe_sign('UTURN',.9,1,1)
        enter_blue_action(c,'UTURN',1.5)
        for i in range(1,650):
            now=1.5+i*.05
            c.observe_ground(dict(source='front',part='markers',slots=[],markers=[]),now)
            c.tick(now)
        self.assertEqual(c.state,'FINISHED')
        self.assertEqual(c.reason,'uturn_trial_complete_stop')

if __name__=='__main__':unittest.main()
