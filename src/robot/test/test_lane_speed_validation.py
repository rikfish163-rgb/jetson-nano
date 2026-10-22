import os
import unittest

from robot.common.config import load_config
from robot.common.contracts import validate_config


class LaneSpeedValidationTest(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config(os.path.join(os.path.dirname(__file__), '../config'))
        self.cfg['lane_curve_speed_raw'] = 24
        self.cfg['straight_speed_raw'] = 24
        self.cfg['speed_raw']['lane'] = 24
        self.cfg['speed_raw']['action'] = 24

    def test_vehicle_speed_24_passes_startup_validation(self):
        validate_config(self.cfg)

    def test_curve_speed_respects_configured_raw_limit(self):
        self.cfg['speed_raw_limit'] = 23
        self.cfg['straight_speed_raw'] = 20
        self.cfg['speed_raw']['lane'] = 20
        self.cfg['speed_raw']['action'] = 20
        with self.assertRaises(ValueError):
            validate_config(self.cfg)

    def test_curve_speed_still_requires_positive_integer(self):
        for value in (0, -1, 24.5, True, '24', float('nan')):
            self.cfg['lane_curve_speed_raw'] = value
            with self.assertRaises(ValueError):
                validate_config(self.cfg)


if __name__ == '__main__':
    unittest.main()
