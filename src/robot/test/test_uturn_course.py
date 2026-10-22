import os
import unittest
import yaml
from robot.master.controller import Controller
from robot.uturn.timed import TimedUturn
from robot.common.contracts import encode_command
from robot.common.contracts import validate_config


class CourseTests(unittest.TestCase):
    def setUp(self):
        root=os.path.join(os.path.dirname(__file__),'../config')
        with open(os.path.join(root,'competition.yaml')) as f:self.cfg=yaml.safe_load(f)
        with open(os.path.join(root,'uturn_course_test.yaml')) as f:self.cfg.update(yaml.safe_load(f))
        self.cfg.update(wait_green=True,lidar_enabled=False,steering_command_scale_rad=.1)
        self.cfg['speed_raw']['lane']=26
        self.c=Controller(self.cfg);self.addCleanup(self.c.close)

    def front(self,t,markers=None):
        self.c.observe_ground(dict(source='front',part='markers',slots=[],markers=markers or []),t)

    def blue(self,t):
        self.front(t,[dict(kind='junction',x=.30,y=0)])

    def start(self):
        for t in (1.,1.1,1.2):self.c.observe_sign('GREEN',.9,t,t)
        self.assertEqual(self.c.state,'STARTUP_STRAIGHT')

    def test_three_consecutive_green_and_only_uturn_cached(self):
        c=self.c
        c.observe_sign('GREEN',.9,1,1);c.observe_sign('',0,1.1,1.1)
        c.observe_sign('GREEN',.9,1.2,1.2);c.observe_sign('GREEN',.9,1.3,1.3)
        self.assertEqual(c.state,'WAIT_GREEN')
        c.observe_sign('GREEN',.9,1.4,1.4)
        c.observe_sign('LEFT',.9,1.5,1.5);self.assertIsNone(c.pending)
        self.blue(1.6);self.assertEqual(c.tick(1.6),(26,0))
        self.assertEqual(c.state,'STARTUP_STRAIGHT')
        c.observe_sign('UTURN',.6,1.7,1.7);self.assertEqual(c.pending,'UTURN')

    def test_entry_sequence_matches_field_trial(self):
        validate_config(self.cfg)
        task=TimedUturn(self.cfg)
        self.assertEqual(task.steps[0][0],'ENTRY')
        self.assertAlmostEqual(task.steps[0][1],.8)
        self.assertEqual(task.steps[0][2],(26,0))
        moving=[s for s in task.steps[1:] if s[2][0]]
        self.assertEqual(len(moving),7)
        for step,row in zip(moving,self.cfg['uturn_trial_sequence']):
            raw=encode_command(step[2][0],step[2][1],self.cfg,0)
            self.assertEqual(raw['speed_raw'],row['speed'])
            self.assertEqual(raw['steering_raw'],row['steering'])
            self.assertAlmostEqual(step[1],row['seconds'])

    def test_complete_then_new_blue_stops_and_latches(self):
        c=self.c;self.start();c.observe_sign('UTURN',.9,1.3,1.3)
        self.blue(1.4);self.assertEqual(c.tick(1.4),(0,0));self.assertEqual(c.state,'UTURN')
        # Current camera data and lane are available throughout the trial.
        for i in range(1,600):
            t=1.4+i*.05
            self.front(t);c.observe_lane([(.5,0),(.8,0)],.99,t)
            c.tick(t)
            if c.state=='LANE':break
        self.assertEqual(c.state,'LANE')
        self.assertTrue(c.course_stop_pending)
        self.blue(t+.05);c.tick(t+.05)
        self.assertNotEqual(c.state,'FINISHED') # line present at handover is not new
        for dt in (.1,.15,.2):
            self.front(t+dt);c.observe_lane([(.5,0),(.8,0)],.99,t+dt);c.tick(t+dt)
        c.observe_sign('LEFT',.99,t+.21,t+.21);self.assertIsNone(c.pending)
        self.blue(t+.25);self.assertEqual(c.tick(t+.25),(0,0))
        self.assertEqual(c.state,'FINISHED')
        from robot.master.telemetry import controller_status
        status=controller_status(c,self.cfg,t+.25,True,True,(0,0),0)
        self.assertTrue(status['uturn_course']['finished'])
        c.observe_sign('GREEN',.99,t+.3,t+.3)
        self.assertEqual(c.tick(t+.3),(0,0))
        self.assertEqual(c.reason,'uturn_course_exit_blue_stop')

    def test_invalid_sequence_rejected(self):
        self.cfg['uturn_trial_sequence']=[dict(speed=-26,steering=22,seconds=1)]
        with self.assertRaises(ValueError):validate_config(self.cfg)

    def test_duplicate_green_does_not_count_and_stale_camera_pauses_entry(self):
        c=self.c
        c.observe_sign('GREEN',.9,1,1)
        c.observe_sign('GREEN',.9,1,1.01)
        c.observe_sign('GREEN',.9,1.1,1.1)
        self.assertEqual(c.state,'WAIT_GREEN')
        c.observe_sign('GREEN',.9,1.2,1.2)
        c.observe_sign('UTURN',.9,1.3,1.3);self.blue(1.4);c.tick(1.4)
        self.front(2.5);self.assertEqual(c.tick(2.5),(26,0))
        self.front(2.6);c.tick(2.6)
        elapsed=c.timed_uturn.elapsed
        self.assertEqual(c.tick(4),(0,0))
        self.front(4.1);self.assertEqual(c.tick(4.1),(26,0))
        self.assertAlmostEqual(c.timed_uturn.elapsed,elapsed)


if __name__=='__main__':unittest.main()
