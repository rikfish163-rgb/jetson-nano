"""The whole-car forward profile shares measured-scene parking for all bays."""
import os
import unittest
from robot.common.config import load_config
from robot.common.contracts import validate_config
from robot.master.controller import Controller


class ForwardParkingDispatchTests(unittest.TestCase):
    def cfg(self, slot):
        cfg = load_config(os.path.join(os.path.dirname(__file__), '../config'))
        cfg.update(parking_mode='forward_plan', parking_slot=slot,
                   wait_green=False, lidar_enabled=False)
        return cfg

    def test_forward_profile_accepts_all_five_designated_bays(self):
        for slot in ('P1', 'P2', 'P3', 'P4', 'P5'):
            cfg = self.cfg(slot)
            validate_config(cfg)
            core = Controller(cfg)
            try:
                self.assertIn('parallel_parking', core._runtime.modules)
                core.pending, core.pending_at = 'PARKING', 1.
                core.marker = ((.1, 0), 1.1)
                core.front_marker_stamp = 1.1
                self.assertEqual(core.dispatch(1.1), (0, 0.))
                self.assertEqual(core.state, 'PARALLEL_PARKING')
                self.assertIsNone(core.pending)
                self.assertEqual(core.action, 'PARKING')
            finally:
                core.close()

    def test_forward_parking_requires_lidar_even_if_lane_lidar_disabled(self):
        core = Controller(self.cfg('P4'))
        self.addCleanup(core.close)
        core.state, core.action, core.action_started = 'PARALLEL_PARKING', 'PARKING', 1.
        self.assertEqual(core.tick(1.1), (0, 0.))
        self.assertEqual(core.reason, 'scan_missing_or_stale')


if __name__ == '__main__':
    unittest.main()
