"""Production sequence settles after its reverse tail, then drives straight."""
import os
import unittest

from robot.common.config import load_config
from robot.common.contracts import encode_command, validate_config
from robot.master.controller import Controller
from robot.uturn.timed import TimedUturn


class DirectUturnResumeTests(unittest.TestCase):
    def make(self,done=True):
        cfg=load_config(os.path.join(os.path.dirname(os.path.dirname(__file__)),'config'))
        cfg.update(wait_green=False,lidar_enabled=False,steering_command_scale_rad=.03)
        validate_config(cfg)
        c=Controller(cfg);self.addCleanup(c.close)
        c.state=c.action='UTURN';c.action_started=1;c.wait_until=0
        c.uturn=dict(trial_last_yaw=0,trial_turn_rad=0)
        c.timed_uturn=TimedUturn(cfg)
        if done:c.timed_uturn.index=len(c.timed_uturn.steps)
        c.front_marker_stamp=10.
        def forbidden(*args):raise AssertionError('extra handoff confirmation was called')
        c.handoff_lane_confirmed=forbidden
        return c

    def test_current_profile_completes_with_same_tick_straight_command(self):
        c=self.make();self.assertEqual(c.cfg['uturn_trial_exit_max_s'],0)
        self.assertTrue(c.cfg['uturn_trial_resume_lane'])
        self.assertFalse(c.cfg['uturn_course_test'])
        # Neither accumulated turn angle nor blue/exit frames may gate entry.
        self.assertEqual(c.tick(10),(c.cfg['straight_speed_raw'],0.))
        self.assertEqual(c.state,'LANE')
        self.assertIsNone(c.action)
        self.assertIsNone(c.timed_uturn)
        self.assertIsNone(c.uturn)
        self.assertFalse(c.course_stop_pending)

    def test_actual_five_stages_then_straight_until_blue(self):
        c=self.make(done=False)
        rows=c.cfg['uturn_trial_sequence']
        self.assertEqual(len(rows),5)
        self.assertEqual(rows[-1],dict(speed=-30,steering=0,seconds=2.0))
        self.assertEqual(c.timed_uturn.steps[-3:],[
            ('GEAR_PAUSE_5',.7,(0,0.)),
            ('SEGMENT_5',2.,(-30,0.)),
            ('SETTLE',.7,(0,0.))])
        seen={}
        for i in range(200):
            now=1+i*.1;c.front_marker_stamp=now
            command=c.tick(now)
            if c.state=='LANE':
                self.assertEqual(command,(c.cfg['straight_speed_raw'],0.))
                break
            phase=c.uturn.get('phase','')
            self.assertNotEqual(phase,'EXIT_ALIGN')
            if phase.startswith('SEGMENT_'):
                raw=encode_command(command[0],command[1],c.cfg,0)
                seen[phase]=(raw['speed_raw'],raw['steering_raw'])
        self.assertEqual(c.state,'LANE')
        self.assertTrue(c.uturn_exit_straight)
        for i,row in enumerate(rows):
            self.assertEqual(seen['SEGMENT_%d'%(i+1)],(row['speed'],row['steering']))

    def test_front_camera_freshness_still_applies_after_handoff(self):
        c=self.make()
        c.front_marker_stamp=-1.
        command=c.tick(10)
        self.assertEqual(c.state,'LANE')
        self.assertEqual(command[0],0)
        self.assertEqual(c.reason,'uturn_exit_front_stale')
        self.assertIsNone(c.timed_uturn)

    def test_estop_and_lidar_still_prevent_motion(self):
        c=self.make();c.estop=True
        self.assertEqual(c.tick(10),(0,0))
        self.assertEqual(c.reason,'emergency_stop')
        c=self.make();c.cfg['lidar_enabled']=True
        self.assertEqual(c.tick(10),(0,0))
        self.assertEqual(c.reason,'scan_missing_or_stale')

    def test_explicit_no_resume_test_mode_stays_finished(self):
        c=self.make();c.cfg['uturn_trial_resume_lane']=False;c.front_marker_stamp=10
        self.assertEqual(c.tick(10),(0,0))
        self.assertEqual(c.state,'FINISHED')
        self.assertEqual(c.reason,'uturn_trial_complete_stop')


if __name__=='__main__':unittest.main()
