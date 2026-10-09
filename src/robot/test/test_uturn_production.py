"""Normal full/stack defaults must dispatch the field-tested seven segments."""
import os
import unittest
from blue_test_helpers import enter_blue_action
import yaml

from robot.common.contracts import encode_command, validate_config
from robot.master.controller import Controller
from robot.uturn.timed import TimedUturn
from robot.uturn.calibration import raw_angle, speed_gain
from robot.common.geometry import bicycle


class ProductionUturnTests(unittest.TestCase):
    def setUp(self):
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        from robot.common.config import load_config
        self.cfg = load_config(os.path.join(root,'config'))
        self.cfg.update(wait_green=False, lidar_enabled=False,
                        steering_command_scale_rad=.03, sign_ttl=0,
                        blue_default_straight=False)
        self.c = Controller(self.cfg)
        self.addCleanup(self.c.close)

    def front(self, now, blue=False):
        self.c.observe_ground(dict(source='front', part='markers', slots=[],
            markers=[dict(kind='junction', x=.3, y=0)] if blue else []), now)

    def test_default_profile_is_normal_seven_stage_mode(self):
        self.assertTrue(self.cfg['uturn_trial_enabled'])
        self.assertTrue(self.cfg['uturn_trial_resume_lane'])
        self.assertFalse(self.cfg['uturn_course_test'])
        validate_config(self.cfg)
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        with open(os.path.join(root, 'config/uturn_course_test.yaml')) as stream:
            field = yaml.safe_load(stream)
        # Compare against the original executed RAW 26 profile unchanged.
        self.assertEqual(self.cfg['uturn_trial_sequence'], field['uturn_trial_sequence'])
        task = TimedUturn(self.cfg)
        self.assertEqual(self.cfg['uturn_trial_entry_m'], .40)
        self.assertEqual(task.steps[0][0], 'ENTRY')
        self.assertAlmostEqual(task.steps[0][1], .40 / self.cfg['uturn_trial_forward_mps'])
        moving = [step for step in task.steps if step[0].startswith('SEGMENT_')]
        self.assertEqual(len(moving), 7)
        for step, row in zip(moving, field['uturn_trial_sequence']):
            raw = encode_command(step[2][0], step[2][1], self.cfg, 0)
            self.assertEqual((raw['speed_raw'], raw['steering_raw']),
                             (row['speed'], row['steering']))
            self.assertAlmostEqual(step[1], row['seconds'])

    def test_40cm_entry_finishes_before_seven_segments(self):
        task = TimedUturn(self.cfg)
        self.assertEqual(task.command(0), (26, 0.))
        for i in range(1, 16):
            self.assertEqual(task.command(i*.1), (26, 0.))
            self.assertEqual(task.status()['phase'], 'ENTRY')
        task.command(1.6)
        self.assertEqual(task.status()['phase'], 'GEAR_PAUSE_1')

    def test_sign_blue_seven_segments_then_resume_normal_mission(self):
        c = self.c
        c.observe_lane([(.5, 0), (.8, 0)], .99, 1)
        c.observe_sign('UTURN', .99, .95, .95)
        c.observe_sign('UTURN', .99, 1, 1)
        self.front(1)
        c.tick(1)
        self.assertEqual(c.pending, 'UTURN')
        self.assertNotEqual(c.state, 'UTURN')
        enter_blue_action(c,'UTURN',1.5)
        self.assertEqual(c.state, 'UTURN')
        phases = []
        command = (0, 0.)
        for i in range(1, 650):
            now = 1.5 + i * .05
            # Simulated command model only; no assertion of measured vehicle motion.
            raw = encode_command(command[0],command[1],self.cfg,0)['steering_raw']
            c.set_pose(bicycle(c.pose,command[0]*speed_gain(self.cfg,command[0])*.05,
                              raw_angle(self.cfg,command[0],raw),self.cfg['wheelbase']),now)
            self.front(now)
            c.observe_lane([(.5, 0), (.8, 0)], .99, now)
            command = c.tick(now)
            if c.timed_uturn is not None:
                phase = c.timed_uturn.status()['phase']
                if phase.startswith('SEGMENT_') and phase not in phases:
                    phases.append(phase)
                    raw = encode_command(command[0], command[1], self.cfg, 0)
                    expected = self.cfg['uturn_trial_sequence'][len(phases)-1]
                    self.assertEqual((raw['speed_raw'], raw['steering_raw']),
                                     (expected['speed'], expected['steering']))
            if c.state == 'LANE':
                break
        self.assertEqual(phases, ['SEGMENT_%d' % i for i in range(1, 8)])
        self.assertEqual(c.state, 'LANE')
        self.assertFalse(c.course_stop_pending)
        self.assertIsNone(c.action)
        c.observe_sign('STRAIGHT', .99, now+.1, now+.1)
        c.observe_sign('STRAIGHT', .99, now+.3, now+.3)
        self.assertEqual(c.pending, 'STRAIGHT')


if __name__ == '__main__':
    unittest.main()
