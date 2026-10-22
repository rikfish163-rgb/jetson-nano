#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Regression checks for the P4/P5 dimensions read from the rules figure.

These values are map priors only.  They must not be confused with the
per-slot measurements required before the runtime calibration lock is lifted.
"""

from __future__ import print_function

import os
import sys
import unittest

import yaml


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
MAIN_SCRIPTS = os.path.join(ROOT, "main", "scripts")
if MAIN_SCRIPTS not in sys.path:
    sys.path.insert(0, MAIN_SCRIPTS)

from parking_controller import ParkingConfig  # noqa: E402


class ParkingMapPriorTest(unittest.TestCase):
    def test_p4_p5_figure_prior_is_450_by_380_mm(self):
        with open(os.path.join(ROOT, "main", "config", "parking.yaml"), "r") as stream:
            parking = yaml.safe_load(stream)
        with open(os.path.join(ROOT, "camera", "config", "rear_parking.yaml"), "r") as stream:
            rear = yaml.safe_load(stream)

        self.assertEqual(parking["slot_length_m"], 0.45)
        self.assertEqual(parking["slot_width_m"], 0.38)
        self.assertEqual(rear["expected_slot_width_m"], 0.38)
        self.assertEqual(ParkingConfig.DEFAULTS["slot_width_m"], 0.38)


if __name__ == "__main__":
    unittest.main()
