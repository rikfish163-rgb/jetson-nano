# -*- coding: utf-8 -*-
"""ROS-independent safety policy for the lane-control trial node."""

from __future__ import division

import math
from collections import namedtuple


TrialCommand = namedtuple(
    "TrialCommand",
    [
        "safe_stop",
        "reason",
        "speed",
        "angel",
        "raw_steering_rad",
        "steering_rad_clamped",
        "confidence",
        "path_point_count",
    ],
)


def _is_finite(value):
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return False
    return not math.isnan(value) and not math.isinf(value)


class LaneControlPolicy(object):
    """Convert a Pure Pursuit result into a fail-closed trial command."""

    def __init__(
        self,
        pure_pursuit,
        target_speed=0.0,
        min_confidence=0.55,
        path_timeout=0.25,
        max_steering_rad=0.35,
        angel_gain=1.0,
    ):
        target_speed = float(target_speed)
        min_confidence = float(min_confidence)
        path_timeout = float(path_timeout)
        max_steering_rad = float(max_steering_rad)
        angel_gain = float(angel_gain)

        if not _is_finite(target_speed) or target_speed < 0.0 or target_speed > 1.0:
            raise ValueError("target_speed must be finite and in [0, 1]")
        if not _is_finite(min_confidence) or min_confidence < 0.0 or min_confidence > 1.0:
            raise ValueError("min_confidence must be finite and in [0, 1]")
        if not _is_finite(path_timeout) or path_timeout <= 0.0:
            raise ValueError("path_timeout must be positive and finite")
        if not _is_finite(max_steering_rad) or max_steering_rad <= 0.0:
            raise ValueError("max_steering_rad must be positive and finite")
        if not _is_finite(angel_gain) or angel_gain < 0.0:
            raise ValueError("angel_gain must be non-negative and finite")

        self.pure_pursuit = pure_pursuit
        self.target_speed = target_speed
        self.min_confidence = min_confidence
        self.path_timeout = path_timeout
        self.max_steering_rad = max_steering_rad
        self.angel_gain = angel_gain

    def _stop(self, reason, confidence, point_count):
        return TrialCommand(
            safe_stop=True,
            reason=reason,
            speed=0.0,
            angel=0.0,
            raw_steering_rad=0.0,
            steering_rad_clamped=0.0,
            confidence=confidence,
            path_point_count=point_count,
        )

    def evaluate(
        self,
        path_points,
        path_update_time,
        confidence,
        confidence_update_time,
        now,
    ):
        """Evaluate current inputs; no previous command is retained or reused."""
        point_count = 0 if path_points is None else len(path_points)

        if path_points is None:
            return self._stop("path_missing", confidence, point_count)
        if not path_points:
            return self._stop("path_empty", confidence, point_count)
        if not _is_finite(now):
            return self._stop("time_non_finite", confidence, point_count)
        if not _is_finite(path_update_time):
            return self._stop("path_time_missing", confidence, point_count)
        if float(now) - float(path_update_time) > self.path_timeout:
            return self._stop("path_stale", confidence, point_count)
        if float(path_update_time) - float(now) > self.path_timeout:
            return self._stop("path_time_in_future", confidence, point_count)

        if not _is_finite(confidence):
            return self._stop("confidence_non_finite", confidence, point_count)
        if not _is_finite(confidence_update_time):
            return self._stop("confidence_time_missing", confidence, point_count)
        if float(now) - float(confidence_update_time) > self.path_timeout:
            return self._stop("confidence_stale", confidence, point_count)
        if float(confidence_update_time) - float(now) > self.path_timeout:
            return self._stop("confidence_time_in_future", confidence, point_count)
        if float(confidence) < self.min_confidence:
            return self._stop("confidence_low", float(confidence), point_count)

        result = self.pure_pursuit.compute(path_points)
        if not result.valid:
            return self._stop(
                "pure_pursuit_%s" % result.reason,
                float(confidence),
                point_count,
            )
        if not _is_finite(result.steering_angle):
            return self._stop("steering_non_finite", float(confidence), point_count)

        raw_steering = float(result.steering_angle)
        steering_clamped = max(
            -self.max_steering_rad,
            min(self.max_steering_rad, raw_steering),
        )
        angel = steering_clamped * self.angel_gain
        if not _is_finite(angel):
            return self._stop("angel_non_finite", float(confidence), point_count)

        return TrialCommand(
            safe_stop=False,
            reason="ok",
            speed=self.target_speed,
            angel=angel,
            raw_steering_rad=raw_steering,
            steering_rad_clamped=steering_clamped,
            confidence=float(confidence),
            path_point_count=point_count,
        )
