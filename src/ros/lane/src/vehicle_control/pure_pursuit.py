# -*- coding: utf-8 -*-
"""ROS-independent Pure Pursuit implementation for base_link path points."""

from __future__ import division

import math
from collections import namedtuple

from vehicle_control.vehicle_model import BicycleModel
from vehicle_control.vehicle_model import DEFAULT_WHEELBASE


PurePursuitResult = namedtuple(
    "PurePursuitResult",
    [
        "valid",
        "steering_angle",
        "curvature",
        "target_speed",
        "nearest_index",
        "target_index",
        "target_point",
        "lookahead_distance",
        "reason",
    ],
)


def _is_finite(value):
    return not math.isnan(value) and not math.isinf(value)


def _safe_result(reason):
    return PurePursuitResult(
        valid=False,
        steering_angle=0.0,
        curvature=0.0,
        target_speed=0.0,
        nearest_index=None,
        target_index=None,
        target_point=None,
        lookahead_distance=0.0,
        reason=reason,
    )


class PurePursuit(object):
    """Pure Pursuit for ordered ``(x_forward, y_left)`` path points.

    Positive steering means a left turn. Dynamic path errors return a safe
    result instead of raising, so callers cannot accidentally reuse a command.
    Invalid controller configuration raises ValueError during construction.
    """

    def __init__(
        self,
        wheelbase=DEFAULT_WHEELBASE,
        lookahead_distance=0.60,
        target_speed=0.0,
        front_offset=None,
    ):
        lookahead_distance = float(lookahead_distance)
        target_speed = float(target_speed)
        if not _is_finite(lookahead_distance) or lookahead_distance <= 0.0:
            raise ValueError("lookahead_distance must be positive and finite")
        if not _is_finite(target_speed) or target_speed < 0.0:
            raise ValueError("target_speed must be non-negative and finite")

        self.model = BicycleModel(wheelbase)
        self.lookahead_distance = lookahead_distance
        self.target_speed = target_speed
        # None retains radial selection for existing maneuver callers. Lane
        # following supplies rear-axle-to-front distance for a transverse line.
        if front_offset is not None:
            front_offset = float(front_offset)
            if not _is_finite(front_offset) or front_offset < 0.0:
                raise ValueError("front_offset must be finite and non-negative")
        self.front_offset = front_offset
        self.last_intersection = None

    def compute(self, path_points):
        self.last_intersection = None
        points, error = self._normalise_path(path_points)
        if error is not None:
            return _safe_result(error)

        forward_points = [point for point in points if point[1] > 0.0]
        if not forward_points:
            return _safe_result("no_forward_point")

        nearest_position = min(
            range(len(forward_points)),
            key=lambda position: self._distance_squared(
                forward_points[position][1],
                forward_points[position][2],
            ),
        )
        nearest = forward_points[nearest_position]

        if self.front_offset is not None:
            target = self._front_line_target(forward_points[nearest_position:])
        else:
            target = forward_points[-1]
            for candidate in forward_points[nearest_position:]:
                if math.hypot(candidate[1], candidate[2]) >= self.lookahead_distance:
                    target = candidate
                    break

        target_x = target[1]
        target_y = target[2]
        lookahead_distance = math.hypot(target_x, target_y)
        if lookahead_distance <= 1.0e-9:
            return _safe_result("zero_lookahead_distance")

        curvature = (2.0 * target_y) / (lookahead_distance ** 2)
        steering_angle = self.model.steering_from_curvature(curvature)
        if not _is_finite(curvature) or not _is_finite(steering_angle):
            return _safe_result("non_finite_output")

        return PurePursuitResult(
            valid=True,
            steering_angle=steering_angle,
            curvature=curvature,
            target_speed=self.target_speed,
            nearest_index=nearest[0],
            target_index=target[0],
            target_point=(target_x, target_y),
            lookahead_distance=lookahead_distance,
            reason="ok",
        )

    def _front_line_target(self, points):
        """First polyline crossing, then nearest existing vertex (no extrapolation)."""
        line_x = self.front_offset + self.lookahead_distance
        intersection = None
        for index, point in enumerate(points):
            if abs(point[1] - line_x) <= 1e-9:
                intersection = (line_x, point[2])
                break
            if index + 1 == len(points):
                continue
            next_point = points[index + 1]
            if (point[1] - line_x) * (next_point[1] - line_x) < 0.0:
                fraction = (line_x - point[1]) / (next_point[1] - point[1])
                intersection = (line_x, point[2] + fraction * (next_point[2] - point[2]))
                break
        if intersection is None:
            return min(points, key=lambda p: abs(p[1] - line_x))
        self.last_intersection = intersection
        return min(points, key=lambda p: self._distance_squared(
            p[1] - intersection[0], p[2] - intersection[1]))

    @staticmethod
    def _distance_squared(x_value, y_value):
        return x_value * x_value + y_value * y_value

    @staticmethod
    def _normalise_path(path_points):
        if path_points is None:
            return None, "empty_path"

        points = []
        try:
            iterator = enumerate(path_points)
            for index, point in iterator:
                try:
                    x_value = float(point[0])
                    y_value = float(point[1])
                except (IndexError, KeyError, TypeError, ValueError, OverflowError):
                    return None, "invalid_point"
                if not _is_finite(x_value) or not _is_finite(y_value):
                    return None, "non_finite_point"
                points.append((index, x_value, y_value))
        except TypeError:
            return None, "invalid_path"

        if not points:
            return None, "empty_path"
        return points, None


def compute_pure_pursuit(
    path_points,
    wheelbase=DEFAULT_WHEELBASE,
    lookahead_distance=0.60,
    target_speed=0.0,
):
    """Convenience wrapper for one-shot offline calculations."""
    controller = PurePursuit(
        wheelbase=wheelbase,
        lookahead_distance=lookahead_distance,
        target_speed=target_speed,
    )
    return controller.compute(path_points)
