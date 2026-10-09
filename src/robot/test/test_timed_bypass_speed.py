"""Bypass speed is independent of ordinary junction action speed."""
import os
import unittest

from robot.common.config import load_config
from robot.common.contracts import encode_command, validate_config
from robot.master.controller import Controller
from test_continuous_obstacles import _SyntheticScan


class TimedBypassSpeedTests(unittest.TestCase):
    def config(self):
        cfg = load_config(os.path.join(os.path.dirname(__file__), '../config'))
        cfg.update(wait_green=False, lidar_enabled=True, steering_command_scale_rad=.03)
        return cfg

    def test_both_arcs_use_26_with_each_vehicle_profile_action_speed(self):
        for action_speed in (20, 24):
            for phase, raw_steer in (('LEFT', 22), ('RIGHT', -22)):
                cfg = self.config()
                cfg['speed_raw']['action'] = action_speed
                core = Controller(cfg)
                self.addCleanup(core.close)
                core.state, core.action = 'TIMED_BYPASS', 'BYPASS'
                core.timed_bypass = dict(phase=phase, last=1., elapsed_s=.5)
                core.scan = _SyntheticScan(1.1)
                speed, steer = core.tick(1.1)
                encoded = encode_command(speed, steer, cfg, 1)
                self.assertEqual(encoded['speed_raw'], 26)
                self.assertEqual(encoded['steering_raw'], raw_steer)
                self.assertEqual(core.cfg['speed_raw']['action'], action_speed)

    def test_speed_override_is_rejected_outside_command_range(self):
        for invalid in (0, -1, 61, float('nan'), True):
            cfg = self.config()
            cfg['timed_bypass_speed_raw'] = invalid
            with self.assertRaises(ValueError):
                validate_config(cfg)


if __name__ == '__main__':
    unittest.main()
