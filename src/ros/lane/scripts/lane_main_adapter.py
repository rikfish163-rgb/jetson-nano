#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
"""Lane/Pure Pursuit adapter for the main controller's integer JSON contract."""
from __future__ import division

import json
import math
import threading

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Float32, String
from vehicle_control.pure_pursuit import PurePursuit


try:
    string_types = (basestring,)
except NameError:
    string_types = (str,)


def finite(value):
    try:
        return not math.isnan(float(value)) and not math.isinf(float(value))
    except (TypeError, ValueError, OverflowError):
        return False


def lost_line():
    return dict(source="lane", lane_state="LOST_LINE", lane_confidence=0.0,
                speed_raw=0, steering_raw=0)


class LaneMainAdapter(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.points = None
        self.path_time = None
        self.frame_valid = False
        self.confidence = None
        self.confidence_time = None
        self.config_error = None
        self.last_valid_payload = None
        self.last_valid_time = None
        self.path_topic = rospy.get_param(
            "~lane_path_topic", "/vision/lane_path")
        self.confidence_topic = rospy.get_param(
            "~lane_confidence_topic", "/vision/lane_confidence")
        self.output_topic = rospy.get_param(
            "~output_topic", "/camera_yihan/receive")
        for name in ("path_topic", "confidence_topic", "output_topic"):
            value = getattr(self, name)
            if (not isinstance(value, string_types) or not value.strip() or
                    not value.startswith("/")):
                raise ValueError("%s must be an absolute ROS topic" % name)
        self.normal_state = rospy.get_param("~normal_lane_state", None)
        if (not isinstance(self.normal_state, string_types) or
                not self.normal_state.strip()):
            self.normal_state = None
        elif self.normal_state == "LOST_LINE":
            self.normal_state = None
        try:
            speed = float(rospy.get_param("~speed_raw", 0))
            limit = float(rospy.get_param("~max_steering_raw", 22))
            self.min_confidence = float(rospy.get_param("~min_confidence", 0.15))
            self.input_timeout = float(rospy.get_param("~input_timeout", 0.25))
            self.hold_last_valid_time = float(rospy.get_param(
                "~hold_last_valid_time", 0.20))
            if not finite(speed) or abs(speed) > 100:
                raise ValueError("speed_raw must be finite and in [-100,100]")
            if not finite(limit):
                raise ValueError("max_steering_raw must be finite")
            if not finite(self.min_confidence) or not 0 <= self.min_confidence <= 1:
                raise ValueError("min_confidence must be finite and in [0,1]")
            if not finite(self.input_timeout) or self.input_timeout <= 0:
                raise ValueError("input_timeout must be positive and finite")
            if (not finite(self.hold_last_valid_time) or
                    not 0.0 <= self.hold_last_valid_time <= 0.30):
                raise ValueError(
                    "hold_last_valid_time must be finite and in [0,0.30]")
            self.speed_raw = int(round(speed))
            self.limit = int(max(0, min(22, limit)))
            self.pp = PurePursuit(
                wheelbase=rospy.get_param("~wheelbase", 0.29),
                lookahead_distance=rospy.get_param("~lookahead_distance", 0.60),
                target_speed=0.0)
        except Exception as exc:
            self.config_error = str(exc)
        self.publisher = rospy.Publisher(self.output_topic, String, queue_size=1)
        rospy.Subscriber(self.path_topic, Path, self.path_callback, queue_size=1)
        rospy.Subscriber(self.confidence_topic, Float32,
                         self.confidence_callback, queue_size=1)
        rospy.on_shutdown(self.publish_stop)
        rospy.loginfo(
            "lane_main_adapter topics: path=%s confidence=%s output=%s",
            self.path_topic, self.confidence_topic, self.output_topic)

    def path_callback(self, msg):
        try:
            valid = msg.header.frame_id == "base_link"
            points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        except Exception:
            valid, points = False, None
        with self.lock:
            self.points, self.frame_valid = points, valid
            self.path_time = rospy.Time.now().to_sec()

    def confidence_callback(self, msg):
        try:
            value = float(msg.data)
        except Exception:
            value = None
        with self.lock:
            self.confidence = value
            self.confidence_time = rospy.Time.now().to_sec()

    def _hold_or_stop(self, now, reason):
        with self.lock:
            payload = (dict(self.last_valid_payload)
                       if self.last_valid_payload is not None else None)
            valid_time = self.last_valid_time
        age = now - valid_time if finite(now) and finite(valid_time) else None
        if (payload is not None and age is not None and
                0.0 <= age <= self.hold_last_valid_time):
            return payload, "hold_last_valid: " + reason
        return lost_line(), reason

    def evaluate(self, now):
        try:
            return self._evaluate(now)
        except Exception as exc:
            return lost_line(), "exception: %s" % str(exc)

    def _evaluate(self, now):
        if self.config_error:
            return lost_line(), "invalid_config: %s" % self.config_error
        if self.normal_state is None:
            return lost_line(), "normal_lane_state must be explicitly configured"
        with self.lock:
            points = self.points
            frame_valid = self.frame_valid
            confidence = self.confidence
            path_time = self.path_time
            confidence_time = self.confidence_time
        if not frame_valid:
            return lost_line(), "frame_invalid"
        if not points:
            return self._hold_or_stop(now, "path_empty")
        if not finite(now):
            return lost_line(), "time_invalid"
        for name, stamp in (("path", path_time), ("confidence", confidence_time)):
            if not finite(stamp) or abs(now - stamp) > self.input_timeout:
                return lost_line(), name + "_timeout"
        if not finite(confidence) or confidence < self.min_confidence:
            return self._hold_or_stop(now, "confidence_invalid_or_low")
        result = self.pp.compute(points)
        if not result.valid or not finite(result.steering_angle):
            return self._hold_or_stop(now, "pure_pursuit_invalid")
        # Endpoint calibration: first linear approximation, not a full servo fit.
        raw_float = result.steering_angle / 0.1 * 22.0 
        steering = max(-self.limit, min(self.limit, int(round(raw_float))))
        payload = dict(source="lane", lane_state=self.normal_state,
                       lane_confidence=float(confidence),
                       speed_raw=int(self.speed_raw),
                       steering_raw=int(steering))
        with self.lock:
            self.last_valid_payload = dict(payload)
            self.last_valid_time = now
        return payload, "ok"

    def publish(self, payload):
        self.publisher.publish(String(data=json.dumps(payload, allow_nan=False,
                                                     separators=(",", ":"))))

    def publish_stop(self):
        try:
            self.publish(lost_line())
        except Exception:
            pass

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            payload, reason = self.evaluate(rospy.Time.now().to_sec())
            self.publish(payload)
            if reason.startswith("hold_last_valid:"):
                rospy.logwarn_throttle(
                    1.0, "lane_main_adapter HOLD_LAST_VALID: " +
                    reason.split(": ", 1)[1])
            elif reason != "ok":
                rospy.logwarn_throttle(1.0, "lane_main_adapter LOST_LINE: " + reason)
            else:
                rospy.loginfo_throttle(1.0, "lane_main_adapter speed=%d steering=%d confidence=%.3f" %
                                       (payload["speed_raw"], payload["steering_raw"],
                                        payload["lane_confidence"]))
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("lane_main_adapter")
    LaneMainAdapter().run()
