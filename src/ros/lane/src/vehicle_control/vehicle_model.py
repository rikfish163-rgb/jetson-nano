# -*- coding: utf-8 -*-
"""Minimal kinematic bicycle-model helpers for offline control tests."""

from __future__ import division

import math


DEFAULT_WHEELBASE = 0.29


def _is_finite(value):
    return not math.isnan(value) and not math.isinf(value)


class BicycleModel(object):
    """Convert path curvature into an equivalent front-wheel angle."""

    def __init__(self, wheelbase=DEFAULT_WHEELBASE):
        wheelbase = float(wheelbase)
        if not _is_finite(wheelbase) or wheelbase <= 0.0:
            raise ValueError("wheelbase must be a positive finite number")
        self.wheelbase = wheelbase

    def steering_from_curvature(self, curvature):
        curvature = float(curvature)
        if not _is_finite(curvature):
            raise ValueError("curvature must be finite")
        return math.atan(self.wheelbase * curvature)
