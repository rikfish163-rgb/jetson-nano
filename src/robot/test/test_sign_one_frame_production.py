"""Use production config to verify one image reaches both sign stages."""
import os
import unittest

from robot.common.config import load_config
from robot.master.controller import Controller
from robot.signs.candidate_gate import RouteSignGate, validate_settings


class OneFrameProductionSignsTests(unittest.TestCase):
    def cfg(self):
        return load_config(os.path.join(os.path.dirname(__file__), '..', 'config'))

    def core(self):
        cfg = self.cfg()
        cfg['wait_green'] = False
        core = Controller(cfg)
        self.addCleanup(core.close)
        core.state = 'LANE'
        core.parking_route_ready = True
        return core

    def observe(self, core, label, stamp=1.):
        cfg = core.cfg
        gate = RouteSignGate(cfg['sign_route_gate'], cfg['sign_confidence'],
                             cfg['sign_min_height_ratio'])
        accepted, reason, diagnostic = gate.evaluate(
            label, .99, [300, 100, 70, 60], (360, 640, 3), stamp)
        self.assertEqual(accepted, label)
        if label not in ('RED', 'GREEN'):
            self.assertEqual(reason, 'route_confirmed')
            self.assertEqual(diagnostic['required_frames'], 1)
        core.observe_sign(accepted, .99, stamp, stamp, right_visible=(label == 'RIGHT'))

    def test_all_directions_are_stored_on_first_image(self):
        for label in ('LEFT', 'RIGHT', 'STRAIGHT', 'UTURN'):
            core = self.core()
            self.observe(core, label)
            self.assertEqual(core.pending, label)
            self.assertEqual(core.sign_info['decision'], 'stored_pending')

    def test_front_parking_is_stored_on_first_image(self):
        core = self.core()
        self.observe(core, 'PARKING')
        self.assertEqual(core.pending, 'PARKING')

    def test_next_route_is_queued_on_first_eligible_image(self):
        core = self.core()
        core.state, core.action = 'MANEUVER', 'LEFT'
        core.action_started = core.route_action_started = 1.
        self.observe(core, 'RIGHT', 3.)
        self.assertEqual(core.next_direction, 'RIGHT')
        self.assertEqual(core.sign_info['decision'], 'stored_next_direction')

    def test_green_and_red_use_one_image(self):
        core = self.core()
        core.state = 'WAIT_GREEN'
        self.observe(core, 'GREEN')
        self.assertNotEqual(core.state, 'WAIT_GREEN')
        self.observe(core, 'RED', 1.1)
        self.assertTrue(core.red)

    def test_one_frame_parameters_are_valid(self):
        validate_settings(dict(enabled=True, normal_frames=1, small_frames=1))
        cfg = self.cfg()
        for key in ('sign_votes', 'direction_sign_votes', 'parking_sign_votes'):
            self.assertEqual(cfg[key], 1)


if __name__ == '__main__':
    unittest.main()
