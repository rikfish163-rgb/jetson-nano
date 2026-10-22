"""No ROS, no publishers, no vehicle motion."""
import copy
import unittest
from test_core import CONFIG
from robot.turn.left_blue_test import LeftBlueTestController
from robot.common.contracts import encode_command


class LeftBlueTest(unittest.TestCase):
    def make(self, **settings):
        c = LeftBlueTestController(copy.deepcopy(CONFIG),settings)
        self.addCleanup(c.close)
        c.scan_ready = lambda now: True
        c.lane_valid = lambda now: True
        c.checked_command = lambda command,now,allow_bypass: command
        c.cfg['pose_mode'] = 'command_model'
        c.cfg['sensor_timeout'] = .5
        return c

    def step(self,c,t):
        c.lane_stamp = t
        return c.tick(t)

    def test_wait_and_same_production_lock(self):
        c=self.make(start_delay_s=1,entry_m=0,speed_raw=16)
        self.assertEqual(self.step(c,1),(0,0))
        self.assertEqual(self.step(c,1.5),(0,0))
        command=self.step(c,2.1)
        self.assertEqual(encode_command(command[0],command[1],c.cfg,0)['steering_raw'],22)
        self.assertEqual(command[0],16)
        self.assertEqual(c.left_lock['phase'],'LOCK')

    def test_sensor_loss_aborts_without_restart(self):
        c=self.make(start_delay_s=0,entry_m=0)
        self.step(c,1)
        c.scan_ready=lambda now:False
        self.assertEqual(self.step(c,1.1),(0,0))
        c.scan_ready=lambda now:True
        self.assertEqual(self.step(c,1.2),(0,0))
        self.assertTrue(c.test_finished)

    def test_timed_stop(self):
        c=self.make(start_delay_s=0,entry_m=0,lock_seconds=.5)
        self.step(c,1)
        self.assertEqual(self.step(c,1.6),(0,0))
        self.assertEqual(c.test_result,'left_test_timed_complete')

    def test_timeout_and_estop(self):
        c=self.make(start_delay_s=0,max_seconds=.5)
        self.step(c,1)
        self.assertEqual(self.step(c,1.6),(0,0))
        self.assertEqual(c.test_result,'left_test_timeout')
        c=self.make(start_delay_s=0)
        c.estop=True
        self.assertEqual(self.step(c,1),(0,0))
        self.assertTrue(c.test_finished)

    def test_visual_completion_stops_in_same_tick(self):
        c=self.make(start_delay_s=0,entry_m=0)
        self.step(c,1)
        def complete(now):
            c.state,c.action='LANE',None
            return (24,0)
        c.left_lock_tick=complete
        self.assertEqual(self.step(c,1.1),(0,0))
        self.assertEqual(c.test_result,'left_test_visual_complete')

    def test_bad_settings(self):
        for setting in ({'speed_raw':31},{'speed_raw':1.5},{'entry_m':-1},{'lock_seconds':float('nan')}):
            with self.assertRaises(ValueError):
                LeftBlueTestController(copy.deepcopy(CONFIG),setting)


if __name__ == '__main__':
    unittest.main()
