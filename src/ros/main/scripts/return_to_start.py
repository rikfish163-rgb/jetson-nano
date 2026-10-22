#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Bounded visual/lidar-assisted reverse recovery to the start lane.

This is an escape/recovery composition, not a replacement for the P4/P5
parking FSM.  It is useful when the front of the vehicle is close to a wall or
field boundary: a global nearest-point obstacle must not prevent a carefully
checked reverse motion away from that front obstacle.  The node publishes only
the keyboard-override JSON consumed by the existing control bridge; the
bridge remains the sole publisher of ``/ackermann_cmd``.

The recovery contract is intentionally narrow:

* ``/scan`` must provide a fresh rear sector with enough valid samples;
* the rear sector must remain clear for the whole reverse segment;
* steering stays zero until a two-line rear-lane calibration is available;
* the segment stops as soon as the front sector clears or its hard timeout is
  reached; and
* shutdown/release always sends an explicit disabled keyboard command.

The pure sector helpers are usable without ROS and are covered by unit tests.
"""

from __future__ import print_function

import json
import math
import threading
import time


try:
    import rospy
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
except ImportError:  # Keep geometry helpers importable on a workstation.
    rospy = None
    LaserScan = None
    String = None


def finite(value):
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return False
    return not math.isnan(value) and not math.isinf(value)


def angular_distance_rad(angle_rad, centre_rad):
    """Return the shortest absolute angular distance in radians."""
    return abs(math.atan2(
        math.sin(float(angle_rad) - float(centre_rad)),
        math.cos(float(angle_rad) - float(centre_rad))))


def sector_min(ranges, angle_min_rad, angle_increment_rad,
               range_min_m, range_max_m, centre_deg, half_width_deg):
    """Return ``(minimum_distance, valid_sample_count)`` for a scan sector.

    LaserScan angle origins are not assumed to be ``-pi``.  The angular
    distance calculation therefore works for the observed Jetson stream whose
    angles run from 0 to 2*pi as well as the usual wrapped representation.
    """
    if ranges is None or not finite(angle_min_rad) or \
            not finite(angle_increment_rad):
        return None, 0
    if not finite(range_min_m) or not finite(range_max_m):
        return None, 0
    centre_rad = math.radians(float(centre_deg))
    half_width_rad = math.radians(abs(float(half_width_deg)))
    values = []
    for index, raw_range in enumerate(ranges):
        if not finite(raw_range):
            continue
        distance = float(raw_range)
        if distance < float(range_min_m) or distance > float(range_max_m):
            continue
        angle = float(angle_min_rad) + index * float(angle_increment_rad)
        if angular_distance_rad(angle, centre_rad) <= half_width_rad:
            values.append(distance)
    return (min(values), len(values)) if values else (None, 0)


def scan_sector_summary(scan_msg, sector_half_width_deg=45.0):
    """Extract conservative front/rear minima from a LaserScan message."""
    if scan_msg is None:
        return {
            "valid": False,
            "front_min_m": None,
            "rear_min_m": None,
            "front_count": 0,
            "rear_count": 0,
        }
    front, front_count = sector_min(
        scan_msg.ranges,
        scan_msg.angle_min,
        scan_msg.angle_increment,
        scan_msg.range_min,
        scan_msg.range_max,
        0.0,
        sector_half_width_deg,
    )
    rear, rear_count = sector_min(
        scan_msg.ranges,
        scan_msg.angle_min,
        scan_msg.angle_increment,
        scan_msg.range_min,
        scan_msg.range_max,
        180.0,
        sector_half_width_deg,
    )
    return {
        "valid": bool(front_count > 0 and rear_count > 0),
        "front_min_m": front,
        "rear_min_m": rear,
        "front_count": front_count,
        "rear_count": rear_count,
    }


def keyboard_payload(sequence, enabled, speed_raw=0, steering_raw=0):
    """Build the strict keyboard-override message consumed by the bridge."""
    return {
        "version": 1,
        "seq": int(sequence) % 256,
        "enabled": bool(enabled),
        "speed_raw": int(speed_raw if enabled else 0),
        "steering_raw": int(steering_raw if enabled else 0),
    }


class ReturnToStart(object):
    """One bounded reverse segment with a directional lidar safety gate."""

    def __init__(self):
        if rospy is None or LaserScan is None or String is None:
            raise RuntimeError("ROS sensor_msgs/std_msgs are required")

        self.keyboard_topic = rospy.get_param(
            "~keyboard_control_topic", "/keyboard/control_cmd")
        self.status_topic = rospy.get_param(
            "~status_topic", "/return_to_start/status")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.speed_raw = int(rospy.get_param("~speed_raw", -5))
        self.rear_clearance_m = float(
            rospy.get_param("~rear_clearance_m", 0.45))
        self.front_release_distance_m = float(
            rospy.get_param("~front_release_distance_m", 0.60))
        self.sector_half_width_deg = float(
            rospy.get_param("~sector_half_width_deg", 45.0))
        self.min_sector_samples = int(
            rospy.get_param("~min_sector_samples", 4))
        self.scan_timeout_s = float(
            rospy.get_param("~scan_timeout_s", 0.35))
        self.minimum_reverse_time_s = float(
            rospy.get_param("~minimum_reverse_time_s", 0.25))
        self.maximum_reverse_time_s = float(
            rospy.get_param("~maximum_reverse_time_s", 3.0))
        self.publish_rate_hz = float(
            rospy.get_param("~publish_rate_hz", 20.0))

        if self.speed_raw >= 0:
            raise ValueError("return_to_start speed_raw must be negative")
        if self.rear_clearance_m <= 0.0:
            raise ValueError("rear_clearance_m must be positive")
        if self.front_release_distance_m <= 0.0:
            raise ValueError("front_release_distance_m must be positive")
        if self.min_sector_samples <= 0:
            raise ValueError("min_sector_samples must be positive")
        if self.scan_timeout_s <= 0.0 or self.maximum_reverse_time_s <= 0.0:
            raise ValueError("scan and reverse timeouts must be positive")
        if self.minimum_reverse_time_s < 0.0 or \
                self.minimum_reverse_time_s > self.maximum_reverse_time_s:
            raise ValueError("invalid reverse time bounds")
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        self.lock = threading.Lock()
        self.scan_summary = None
        self.scan_received_at = None
        self.state = "waiting_for_scan"
        self.reason = "not_started"
        self.sequence = 0
        self.started_at = None
        self.publisher = rospy.Publisher(
            self.keyboard_topic, String, queue_size=1)
        self.status_publisher = rospy.Publisher(
            self.status_topic, String, queue_size=1)
        self.scan_subscriber = rospy.Subscriber(
            self.scan_topic, LaserScan, self.scan_callback, queue_size=1)
        rospy.on_shutdown(self.release)

    def scan_callback(self, message):
        summary = scan_sector_summary(
            message, sector_half_width_deg=self.sector_half_width_deg)
        with self.lock:
            self.scan_summary = summary
            self.scan_received_at = time.time()

    def publish_keyboard(self, enabled, speed_raw=0, steering_raw=0):
        payload = keyboard_payload(
            self.sequence, enabled, speed_raw, steering_raw)
        self.publisher.publish(String(data=json.dumps(
            payload, separators=(",", ":"))))
        self.sequence = (self.sequence + 1) % 256

    def publish_status(self, elapsed, summary):
        data = {
            "schema": "return_to_start_status_v1",
            "state": self.state,
            "reason": self.reason,
            "elapsed_s": float(max(0.0, elapsed)),
            "speed_raw": int(self.speed_raw if self.state == "reverse" else 0),
            "steering_raw": 0,
            "rear_min_m": (summary or {}).get("rear_min_m"),
            "front_min_m": (summary or {}).get("front_min_m"),
            "rear_count": int((summary or {}).get("rear_count", 0)),
            "front_count": int((summary or {}).get("front_count", 0)),
        }
        self.status_publisher.publish(String(data=json.dumps(
            data, separators=(",", ":"))))

    def release(self):
        # A few disabled messages make release deterministic even when the
        # bridge's keyboard timer is between callbacks.
        for _ in range(3):
            try:
                self.publish_keyboard(False)
            except Exception:
                pass

    def run(self):
        self.started_at = time.time()
        rate = rospy.Rate(self.publish_rate_hz)
        while not rospy.is_shutdown():
            now = time.time()
            elapsed = now - self.started_at
            with self.lock:
                summary = dict(self.scan_summary) \
                    if self.scan_summary is not None else None
                received_at = self.scan_received_at
            fresh = (summary is not None and received_at is not None and
                     0.0 <= now - received_at <= self.scan_timeout_s)

            if not fresh:
                self.state = "waiting_for_scan"
                self.reason = "scan_missing_or_stale"
                self.publish_keyboard(False)
            elif (not summary.get("valid") or
                  summary.get("rear_count", 0) < self.min_sector_samples or
                  summary.get("rear_min_m") is None):
                self.state = "blocked"
                self.reason = "rear_sector_invalid"
                self.publish_keyboard(False)
            elif summary["rear_min_m"] < self.rear_clearance_m:
                self.state = "blocked"
                self.reason = "rear_obstacle_detected"
                self.publish_keyboard(False)
            elif elapsed >= self.maximum_reverse_time_s:
                self.state = "done"
                self.reason = "maximum_reverse_time"
                self.publish_keyboard(False)
            elif (elapsed >= self.minimum_reverse_time_s and
                  summary.get("front_min_m") is not None and
                  summary["front_min_m"] >= self.front_release_distance_m):
                self.state = "done"
                self.reason = "front_sector_clear"
                self.publish_keyboard(False)
            else:
                self.state = "reverse"
                self.reason = "rear_sector_clear"
                # No calibrated two-line rear pose exists in the current
                # field view, so recovery is straight only.  A future lane
                # steering term must be added behind an explicit calibration
                # gate instead of guessing the camera mirror sign.
                self.publish_keyboard(True, self.speed_raw, 0)

            self.publish_status(elapsed, summary)
            if self.state in ("done", "blocked"):
                self.release()
                return 0
            rate.sleep()
        return 0


def main():
    if rospy is None:
        raise RuntimeError("ROS is required to run return_to_start")
    rospy.init_node("return_to_start")
    try:
        return ReturnToStart().run()
    except Exception as exc:
        rospy.logfatal("return_to_start cannot start: %s", exc)
        return 2


if __name__ == "__main__":
    main()
