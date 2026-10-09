#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Write compact JSONL evidence for every parking control run.

The logger records text contracts and timestamps, not camera frames.  This
keeps the log small enough for the Jetson while allowing offline replay of
the exact observation/control/status sequence used during parameter tuning.
"""

from __future__ import print_function

import datetime
import hashlib
import json
import math
import os
import threading
import time

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


try:
    string_types = (basestring,)
except NameError:
    string_types = (str,)


class ParkingSessionLogger(object):
    def __init__(self):
        default_name = "parking_%s.jsonl" % datetime.datetime.now().strftime(
            "%Y%m%d_%H%M%S")
        default_path = os.path.join("/tmp", "parking_runs", default_name)
        self.path = rospy.get_param("~log_file", default_path)
        self.run_id = rospy.get_param(
            "~run_id", datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
        self.config_file = rospy.get_param("~config_file", "")
        self.config_sha256 = self._sha256(self.config_file)
        configured_files = rospy.get_param("~config_files", [])
        if isinstance(configured_files, string_types):
            configured_files = [item.strip() for item in
                                configured_files.split(",") if item.strip()]
        if not isinstance(configured_files, (list, tuple)):
            configured_files = []
        self.config_files = []
        for path in list(configured_files) + [self.config_file]:
            if not isinstance(path, string_types) or not path:
                continue
            if path not in self.config_files:
                self.config_files.append(path)
        self.config_hashes = dict(
            (path, self._sha256(path)) for path in self.config_files)
        self.topic_config = self._build_topic_config()
        self.runtime_config = {
            "slot_id": rospy.get_param("~slot_id", ""),
            "request_from_front_slot": bool(
                rospy.get_param("~request_from_front_slot", False)),
            "front_marker_input_mode": rospy.get_param(
                "~front_marker_input_mode", "metric_white"),
            "slot_input_mode": rospy.get_param(
                "~slot_input_mode", rospy.get_param(
                    "~front_marker_input_mode", "metric_white")),
            "start_line_input_mode": rospy.get_param(
                "~start_line_input_mode", "blue_raw"),
            # These switches are launch-time part of the visual/control
            # contract.  Recording them prevents a later tuning run from
            # being mistaken for an identical run that used a different pose
            # source or an extra legacy lane node.
            "derive_front_pose": bool(
                rospy.get_param("~derive_front_pose", False)),
            "use_candidate_exit": bool(
                rospy.get_param("~use_candidate_exit", False)),
            "start_cameras": bool(
                rospy.get_param("~start_cameras", False)),
            "start_lane_nodes": bool(
                rospy.get_param("~start_lane_nodes", False)),
            "start_lane_pose": bool(
                rospy.get_param("~start_lane_pose", False)),
            "enable_exit_pose": bool(
                rospy.get_param("~enable_exit_pose", False)),
            "exit_distance_offset_m": rospy.get_param(
                "~exit_distance_offset_m", 0.0),
            "parking_only": bool(
                rospy.get_param("~parking_only", True)),
            "require_calibrated_parking": bool(
                rospy.get_param("~require_calibrated_parking", True)),
            "allow_experimental_motion": bool(
                rospy.get_param("~allow_experimental_motion", False)),
            "experimental_speed_limit_raw": rospy.get_param(
                "~experimental_speed_limit_raw", 1),
            "experimental_steering_limit_raw": rospy.get_param(
                "~experimental_steering_limit_raw", 4),
            "sign_image_topic": rospy.get_param(
                "~sign_image_topic", "/front/usb_cam/image_raw"),
            "front_device": rospy.get_param("~front_device", ""),
            "rear_device": rospy.get_param("~rear_device", ""),
            "start_actuators": bool(
                rospy.get_param("~start_actuators", False)),
            "start_lidar_adapter": bool(
                rospy.get_param("~start_lidar_adapter", False)),
            "start_lidar_source": bool(
                rospy.get_param("~start_lidar_source", False)),
        }
        parent = os.path.dirname(os.path.abspath(self.path))
        if parent and not os.path.isdir(parent):
            os.makedirs(parent)
        self.lock = threading.Lock()
        self.stream = open(self.path, "a")
        self._write("logger_start", {
            "config_file": self.config_file,
            "config_sha256": self.config_sha256,
            "config_files": self.config_files,
            "config_hashes": self.config_hashes,
            "topics": self.topic_config,
            "runtime_config": self.runtime_config,
        })
        self._subscribe_topics()
        rospy.on_shutdown(self.close)
        rospy.loginfo("parking_session_logger writing %s", self.path)

    @staticmethod
    def _finite(value):
        try:
            value = float(value)
        except (TypeError, ValueError):
            return None
        if math.isnan(value) or math.isinf(value):
            return None
        return value

    @staticmethod
    def _stamp_seconds(stamp):
        try:
            value = stamp.to_sec()
        except AttributeError:
            value = stamp
        return ParkingSessionLogger._finite(value)

    def _build_topic_config(self):
        return {
            "observation": rospy.get_param(
                "~observation_topic", "/parking/observation"),
            # Image/BEV topics are recorded as graph metadata.  The logger
            # intentionally does not subscribe to raw images, but retaining
            # these names makes a custom camera/replay wiring reproducible.
            "front_image": rospy.get_param(
                "~front_image_topic", "/front/usb_cam/image_raw"),
            "rear_image": rospy.get_param(
                "~rear_image_topic", "/rear/usb_cam/image_raw"),
            "sign_image": rospy.get_param(
                "~sign_image_topic", "/front/usb_cam/image_raw"),
            "rear_bev": rospy.get_param(
                "~rear_bev_topic", "/debug/rear_bev"),
            "front_mask": rospy.get_param(
                "~front_mask_topic", "/debug/blue_metric_mask"),
            "front_overlay": rospy.get_param(
                "~front_overlay_topic", "/debug/blue_marker_overlay"),
            "slot_mask": rospy.get_param(
                "~slot_mask_topic", rospy.get_param(
                    "~front_mask_topic", "/debug/parking_slot_mask")),
            "slot_overlay": rospy.get_param(
                "~slot_overlay_topic", rospy.get_param(
                    "~front_overlay_topic", "/debug/parking_slot_overlay")),
            "start_line_mask": rospy.get_param(
                "~start_line_mask_topic", "/debug/parking_start_line_mask"),
            "start_line_overlay": rospy.get_param(
                "~start_line_overlay_topic",
                "/debug/parking_start_line_overlay"),
            "control": rospy.get_param(
                "~control_topic", "/control/cmd"),
            "status": rospy.get_param(
                "~status_topic", "/parking/status"),
            "lidar": rospy.get_param(
                "~lidar_topic", "/parking/lidar"),
            # The normalized topic is the safety input.  The raw stream is
            # optional evidence for debugging the legacy LS01B publisher.
            "lidar_raw": rospy.get_param("~lidar_raw_topic", ""),
            "sign": rospy.get_param(
                "~sign_topic", "/camera_hts/receive"),
            "front_candidate": rospy.get_param(
                "~front_candidate_topic", "/perception/parking_slot_candidate"),
            "slot_candidate": rospy.get_param(
                "~slot_candidate_topic", rospy.get_param(
                    "~front_candidate_topic",
                    "/perception/parking_slot_candidate")),
            "start_line": rospy.get_param(
                "~start_line_topic", "/perception/parking_start_line"),
            "front_pose": rospy.get_param(
                "~front_pose_topic", "/perception/front_slot_pose"),
            "front_metric": rospy.get_param(
                "~front_metric_topic", "/perception/front_parking_metric"),
            "rear_metric": rospy.get_param(
                "~rear_metric_topic", "/perception/rear_parking_metric"),
            "exit_pose": rospy.get_param(
                "~exit_pose_topic", "/perception/exit_pose"),
            "exit_metric": rospy.get_param(
                "~exit_metric_topic", "/perception/exit_metric"),
            "lane_path": rospy.get_param(
                "~lane_path_topic", "/vision/lane_path"),
            "lane_confidence": rospy.get_param(
                "~lane_confidence_topic", "/vision/lane_confidence"),
            # This is the output of ackermann_control_bridge.  It proves that
            # the central JSON command reached the ROS actuator interface,
            # but it is not a wheel-motion measurement.
            "actuator_command": rospy.get_param(
                "~actuator_command_topic", "/ackermann_cmd"),
            # base_controller publishes this optional status after its serial
            # write.  It proves the local serial path was exercised, not that
            # the chassis physically moved.
            "base_controller_status": rospy.get_param(
                "~base_controller_status_topic", "/base_controller/status"),
            # Empty means no odometry/encoder source is currently configured.
            "odometry": rospy.get_param("~odometry_topic", ""),
        }

    def _subscribe_topics(self):
        string_topics = (
            "observation",
            "control",
            "status",
            "lidar",
            "lidar_raw",
            "sign",
            "slot_candidate",
            "start_line",
            "front_candidate",
            "front_pose",
            "front_metric",
            "rear_metric",
            "exit_pose",
            "exit_metric",
            "base_controller_status",
        )
        subscribed_topics = set()
        for name in string_topics:
            topic = self.topic_config.get(name)
            if topic and topic not in subscribed_topics:
                rospy.Subscriber(
                    topic, String, self._callback,
                    callback_args=(name, topic), queue_size=10)
                subscribed_topics.add(topic)

        actuator_topic = self.topic_config.get("actuator_command")
        if actuator_topic:
            rospy.Subscriber(
                actuator_topic, AckermannDriveStamped,
                self._actuator_callback,
                callback_args=actuator_topic, queue_size=20)

        odometry_topic = self.topic_config.get("odometry")
        if odometry_topic:
            rospy.Subscriber(
                odometry_topic, Odometry, self._odometry_callback,
                callback_args=odometry_topic, queue_size=20)

    def _actuator_callback(self, message, topic):
        speed = self._finite(message.drive.speed)
        steering = self._finite(message.drive.steering_angle)
        self._write("actuator_command", {
            "topic": topic,
            "header_stamp": self._stamp_seconds(message.header.stamp),
            "frame_id": message.header.frame_id,
            "speed": speed,
            "steering_angle": steering,
            "speed_raw": speed,
            "steering_raw": steering,
            "valid": speed is not None and steering is not None,
        })

    def _odometry_callback(self, message, topic):
        pose = message.pose.pose
        twist = message.twist.twist
        values = {
            "position_x_m": self._finite(pose.position.x),
            "position_y_m": self._finite(pose.position.y),
            "position_z_m": self._finite(pose.position.z),
            "orientation_x": self._finite(pose.orientation.x),
            "orientation_y": self._finite(pose.orientation.y),
            "orientation_z": self._finite(pose.orientation.z),
            "orientation_w": self._finite(pose.orientation.w),
            "linear_x_mps": self._finite(twist.linear.x),
            "linear_y_mps": self._finite(twist.linear.y),
            "angular_z_radps": self._finite(twist.angular.z),
        }
        values.update({
            "topic": topic,
            "header_stamp": self._stamp_seconds(message.header.stamp),
            "frame_id": message.header.frame_id,
            "child_frame_id": message.child_frame_id,
            "valid": all(value is not None for value in values.values()),
        })
        self._write("odometry", values)

    @staticmethod
    def _sha256(path):
        if not path or not os.path.isfile(path):
            return None
        digest = hashlib.sha256()
        try:
            with open(path, "rb") as stream:
                while True:
                    block = stream.read(65536)
                    if not block:
                        break
                    digest.update(block)
        except IOError:
            return None
        return digest.hexdigest()

    def _callback(self, message, callback_args):
        name, topic = callback_args
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = message.data
        self._write(name, {"topic": topic, "payload": payload})

    def _write(self, event, data):
        record = {
            "wall_time": time.time(),
            "run_id": self.run_id,
            "event": event,
            "data": data,
        }
        with self.lock:
            if self.stream is None:
                return
            self.stream.write(json.dumps(
                record, separators=(",", ":"), allow_nan=False) + "\n")
            self.stream.flush()

    def close(self):
        with self.lock:
            if self.stream is not None:
                self.stream.close()
                self.stream = None


def main():
    rospy.init_node("parking_session_logger", anonymous=False)
    try:
        ParkingSessionLogger()
    except Exception as exc:
        rospy.logfatal("parking_session_logger cannot start: %s", exc)
        return 2
    rospy.spin()
    return 0


if __name__ == "__main__":
    main()
