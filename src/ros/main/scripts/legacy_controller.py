#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import json
import threading
import time

import rospy
from std_msgs.msg import Bool, String

from parking_controller import ParkingConfig, ParkingController


CONTROL_VERSION = 1
CONTROL_RATE_HZ = 20.0
SENSOR_TIMEOUT_SEC = 0.50
MAX_SPEED_RAW = 100
MAX_STEERING_RAW = 22
GREEN_START_SPEED_RAW = 20
GREEN_START_HOLD_SEC = 0.50
COMPETITION_WAIT_GREEN = "WAIT_GREEN"
COMPETITION_STARTING = "STARTING"
COMPETITION_RUNNING = "RUNNING"
MISSION_STATES = (
    "NORMAL", "INTERSECTION", "PARKING", "STOP", "FAULT")
PENDING_ACTIONS = (
    "NONE", "STRAIGHT", "LEFT", "RIGHT", "UTURN", "PARKING")
ACTIONABLE_TRAFFIC_SIGNS = frozenset(
    ("STRAIGHT", "LEFT", "RIGHT", "UTURN", "PARKING"))


try:
    integer_types = (int, long)
except NameError:
    integer_types = (int,)

try:
    string_types = (basestring,)
except NameError:
    string_types = (str,)


def update_pending_action(current_action, traffic_sign):
    """Latch a confirmed post-start mission sign without driving directly."""
    if current_action not in PENDING_ACTIONS:
        current_action = "NONE"
    if not isinstance(traffic_sign, dict):
        return current_action
    if (traffic_sign.get("valid") is not True or
            traffic_sign.get("confirmed") is not True):
        return current_action

    label = str(traffic_sign.get("label", "")).strip().upper()
    if label in ACTIONABLE_TRAFFIC_SIGNS:
        return label
    return current_action


def normalized_sign_label(sign_data):
    """Return a case-insensitive label from a camera sign payload."""
    if not isinstance(sign_data, dict):
        return ""
    return str(sign_data.get("label", "")).strip().lower()


def is_confirmed_sign_label(sign_data, labels):
    """Return true for an accepted label after its confirmation gate."""
    if not isinstance(sign_data, dict):
        return False
    accepted = set(str(label).strip().lower() for label in labels)
    if normalized_sign_label(sign_data) not in accepted:
        return False
    # Legacy plain labels have no validity fields and remain compatible.
    if "valid" in sign_data and sign_data.get("valid") is not True:
        return False
    if "confirmed" in sign_data and sign_data.get("confirmed") is not True:
        return False
    return True


def is_green_start_label(sign_data):
    """Return true only for a fresh, valid green start label.

    Structured sign publishers must explicitly mark the result valid and
    confirmed.  Legacy plain-label payloads do not carry those fields and are
    still accepted after ``decode_sensor_data`` has checked their freshness.
    """
    return is_confirmed_sign_label(sign_data, ("green",))


def resolve_parking_only(requested, config_valid):
    """Return the command-policy gate for a requested parking deployment.

    An invalid parking configuration is a safety condition, not a reason to
    let the process fall through to the legacy lane controller.
    """
    return bool(requested) or not bool(config_valid)


def should_wait_before_parking(parking_only, require_calibrated_parking,
                               calibration_complete,
                               allow_experimental_motion=False,
                               allow_preparking_approach=False):
    """Decide whether pre-trigger commands must remain stopped.

    The parking composition needs to drive along the lane until the selected
    bay is detected, but it must not turn that capability into an uncalibrated
    open-loop drive.  ``start_all.launch`` keeps its legacy behaviour by
    default; the parking launch opts into this gate explicitly.
    """
    preparking_open = bool(
        allow_experimental_motion and allow_preparking_approach)
    return (bool(parking_only) and not preparking_open) or (
        bool(require_calibrated_parking) and
        not bool(calibration_complete) and
        not bool(allow_experimental_motion)
    )


def preparking_lidar_block_reason(lidar_data, require_lidar_clear=True):
    """Return the fail-safe reason for blocking pre-parking lane motion.

    The parking FSM already applies this contract after a parking request.
    The pre-trigger lane-following phase must apply the same contract when
    the calibrated parking composition is allowed to move toward P4/P5;
    otherwise a missing or legacy lidar payload could silently authorize
    forward motion before the FSM takes ownership.
    """
    if not require_lidar_clear:
        return None
    if not isinstance(lidar_data, dict):
        return "lidar_missing"
    if lidar_data.get("emergency_stop") is True:
        return "emergency_stop"
    if lidar_data.get("schema") != "parking_lidar_v1":
        return "lidar_schema_invalid"
    frame_id = lidar_data.get(
        "frame_id", lidar_data.get("coordinate_frame"))
    if frame_id != "base_link":
        return "lidar_frame_invalid"
    if not isinstance(lidar_data.get("valid"), bool):
        return "lidar_valid_missing"
    if lidar_data.get("valid") is not True:
        return "lidar_invalid"
    if not isinstance(lidar_data.get("obstacle_detected"), bool):
        return "lidar_obstacle_status_missing"
    if lidar_data.get("obstacle_detected") is True:
        return "obstacle_detected"
    return None


def preparking_straight_command(forward_speed_raw, speed_limit_raw):
    """Build the fixed-heading command used before the parking trigger.

    The selected bay's blue start line is the only source that should release
    the parking manoeuvre.  A lane-following command can contain curvature
    from the surrounding track and therefore must not steer the vehicle during
    this straight pre-trigger phase.  Invalid limits fail closed to a zero
    command.
    """
    try:
        if isinstance(forward_speed_raw, bool) or isinstance(speed_limit_raw, bool):
            raise ValueError("raw limits must be integers")
        forward_speed_raw = int(forward_speed_raw)
        speed_limit_raw = int(speed_limit_raw)
    except (TypeError, ValueError, OverflowError):
        return {"speed_raw": 0, "steering_raw": 0}
    if forward_speed_raw <= 0 or speed_limit_raw <= 0:
        return {"speed_raw": 0, "steering_raw": 0}
    return {
        "speed_raw": min(forward_speed_raw, speed_limit_raw),
        "steering_raw": 0,
    }


def green_start_command(initial_speed_raw, speed_limit_raw):
    """Build the straight initial-speed command for a confirmed green label."""
    return preparking_straight_command(initial_speed_raw, speed_limit_raw)


def observation_parking_armed(observation):
    """Return the latched P-sign arm fact from the fused observation."""
    return bool(
        isinstance(observation, dict) and
        observation.get("schema") == "parking_observation_v1" and
        observation.get("parking_armed") is True)


def observation_parking_side(observation):
    """Return the concrete vehicle-relative side from fused target vision."""
    if not isinstance(observation, dict):
        return None
    value = observation.get("parking_side", observation.get("turn_side"))
    if not isinstance(value, string_types):
        return None
    value = value.strip().lower()
    if value in ("left", "l", "+1", "positive"):
        return "left"
    if value in ("right", "r", "-1", "negative"):
        return "right"
    return None


def observation_target_ready(observation):
    """Return true only for a concrete, pre-manoeuvre P4/P5 target."""
    if not isinstance(observation, dict):
        return False
    selected = observation.get("selected_slot_id")
    return bool(
        observation_parking_armed(observation) and
        observation.get("slot_selection_ready") is True and
        isinstance(selected, string_types) and
        selected.strip().upper() in ("P4", "P5") and
        observation_parking_side(observation) in ("left", "right") and
        observation.get("slot_obstacle_detected") is not True)


def observation_parking_trigger_visible(observation):
    """Return true only when the selected bay's blue start line is visible."""
    return bool(
        isinstance(observation, dict) and
        observation.get("schema") == "parking_observation_v1" and
        observation.get("parking_trigger_visible") is True)


class LatestReceiver(object):
    def __init__(self, receive_channel):
        self.latest = None
        self.latest_time = None
        self.lock = threading.Lock()
        self.subscriber = rospy.Subscriber(
            receive_channel, String, self.callback, queue_size=1)

    def callback(self, msg):
        with self.lock:
            self.latest = msg.data
            self.latest_time = time.time()

    def get_latest(self, max_age=None):
        with self.lock:
            if self.latest is None or self.latest_time is None:
                return None
            if max_age is not None and time.time() - self.latest_time > max_age:
                return None
            return self.latest


class DataSender(object):
    def __init__(self, send_channel):
        self.pub = rospy.Publisher(send_channel, String, queue_size=1)

    def send(self, data):
        msg = String()
        msg.data = json.dumps(data, separators=(",", ":"))
        self.pub.publish(msg)


class AutoDriveController(object):
    """Select one control mode and publish a validated raw command."""

    def __init__(self):
        # Parking launch points this receiver at the normalized
        # ``parking_lidar_v1`` topic.  The default keeps the legacy
        # start_all.launch topology unchanged for non-parking deployments.
        self.parking_lidar_topic = rospy.get_param(
            "~parking_lidar_topic", "/lidar/receive")
        self.receiver_lidar = LatestReceiver(self.parking_lidar_topic)
        self.receiver_camera_yihan = LatestReceiver("/camera_yihan/receive")
        self.receiver_camera_hts = LatestReceiver("/camera_hts/receive")
        self.traffic_sign_topic = rospy.get_param(
            "~traffic_sign_topic", "/perception/traffic_sign")
        self.receiver_traffic_sign = LatestReceiver(self.traffic_sign_topic)
        # Parking uses a dedicated observation topic so the legacy bridge
        # cannot mix an old /image/send payload with the strict parking
        # observation contract.  The default keeps ordinary start_all.launch
        # deployments backward compatible.
        self.parking_observation_topic = rospy.get_param(
            "~parking_observation_topic", "/image/receive")
        self.receiver_image = LatestReceiver(self.parking_observation_topic)
        self.parking_reset_subscriber = rospy.Subscriber(
            "/parking/reset", Bool, self.parking_reset_callback, queue_size=1)

        self.control_sender = DataSender("/control/cmd")
        self.parking_status_sender = DataSender("/parking/status")

        parking_config = rospy.get_param("~parking", {})
        self.parking_config_valid = True
        self.parking_config_error = None
        try:
            parking_config = ParkingConfig.from_dict(parking_config)
        except ValueError as exc:
            rospy.logerr("Invalid parking configuration: %s", exc)
            self.parking_config_valid = False
            self.parking_config_error = str(exc)
            parking_config = ParkingConfig({"calibration_complete": False})
        requested_experimental_motion = bool(rospy.get_param(
            "~allow_experimental_motion", False))
        requested_preparking_approach = bool(rospy.get_param(
            "~allow_preparking_approach", False))
        experimental_speed_limit_raw = rospy.get_param(
            "~experimental_speed_limit_raw", 1)
        experimental_steering_limit_raw = rospy.get_param(
            "~experimental_steering_limit_raw", 4)
        # A malformed config must never be made movable by the experimental
        # switch.  The controller applies its own hard numeric ceilings.
        self.allow_experimental_motion = (
            requested_experimental_motion and self.parking_config_valid)
        self.allow_preparking_approach = bool(
            requested_preparking_approach and self.allow_experimental_motion)
        self.parking_controller = ParkingController(
            parking_config,
            allow_experimental_motion=self.allow_experimental_motion,
            experimental_speed_limit_raw=experimental_speed_limit_raw,
            experimental_steering_limit_raw=experimental_steering_limit_raw,
        )
        self.sensor_timeout_sec = float(
            rospy.get_param("~sensor_timeout_s", SENSOR_TIMEOUT_SEC))
        self.parking_only = resolve_parking_only(
            rospy.get_param("~parking_only", False),
            self.parking_config_valid,
        )
        self.require_calibrated_parking = bool(rospy.get_param(
            "~require_calibrated_parking", False))
        # A malformed parking config must never fall through to the legacy
        # lane controller.  Keep the process alive for diagnostics, but force
        # the central command policy into zero-command parking-only wait mode.
        if not self.parking_config_valid:
            self.parking_only = True
            rospy.logerr(
                "Parking config invalid; forcing parking_only zero-command mode")
        elif self.allow_experimental_motion:
            # The opt-in route may either wait for a separately moving vehicle
            # or, when explicitly requested, drive straight toward the
            # selected bay.  The pre-parking lidar gate remains mandatory.
            self.parking_only = not self.allow_preparking_approach
            rospy.logwarn(
                "EXPERIMENTAL parking motion enabled: calibration_complete=%s, "
                "speed_limit=%d, steering_limit=%d, preparking_approach=%s",
                parking_config.calibration_complete,
                self.parking_controller.experimental_speed_limit_raw,
                self.parking_controller.experimental_steering_limit_raw,
                str(self.allow_preparking_approach),
            )
        self.allow_legacy_parking_trigger = bool(rospy.get_param(
            "~allow_legacy_parking_trigger", not self.parking_only))
        self.max_speed_raw = int(parking_config.max_speed_raw)
        self.max_steering_raw = int(parking_config.max_steering_raw)
        self.preparking_speed_raw = int(rospy.get_param(
            "~preparking_speed_raw", parking_config.forward_speed_raw))
        self.green_start_speed_raw = int(rospy.get_param(
            "~green_start_speed_raw", GREEN_START_SPEED_RAW))
        if self.green_start_speed_raw <= 0:
            rospy.logerr(
                "green_start_speed_raw must be positive; using %d",
                GREEN_START_SPEED_RAW)
            self.green_start_speed_raw = GREEN_START_SPEED_RAW
        self.competition_mode = bool(rospy.get_param(
            "~competition_mode", True))
        self.timed_obstacle = None
        if rospy.get_param('~timed_obstacle_enabled', False):
            from timed_obstacle import TimedObstacle
            from sensor_msgs.msg import LaserScan
            self.timed_obstacle = TimedObstacle(
                half_angle=float(rospy.get_param('~obstacle_half_angle_deg', 15.0)),
                distance=float(rospy.get_param('~obstacle_distance_m', 0.5)),
                center=float(rospy.get_param('~obstacle_center_deg', 0.0)),
                speed=int(rospy.get_param('~obstacle_speed_raw', 26)),
                turn_s=float(rospy.get_param('~obstacle_turn_s', 5.0)))
            self.obstacle_scan_subscriber = rospy.Subscriber(
                '/scan', LaserScan, self.update_obstacle_scan, queue_size=1)
        self.green_start_hold_sec = float(rospy.get_param(
            "~green_start_hold_s", GREEN_START_HOLD_SEC))
        if self.green_start_hold_sec < 0.0:
            rospy.logerr(
                "green_start_hold_s must not be negative; using %.2f",
                GREEN_START_HOLD_SEC)
            self.green_start_hold_sec = GREEN_START_HOLD_SEC
        # Fixed-heading is the safe default for the short segment between P
        # recognition and the blue trigger.  A separately calibrated lane
        # command can be opted in without changing the FSM contract.
        self.use_lane_for_preparking = bool(rospy.get_param(
            "~use_lane_for_preparking", False))
        if self.sensor_timeout_sec <= 0.0:
            rospy.logerr("sensor_timeout_s must be positive; using %.3f",
                         SENSOR_TIMEOUT_SEC)
            self.sensor_timeout_sec = SENSOR_TIMEOUT_SEC
        self.last_parking_state = None
        self.last_preparking_lidar_reason = None

        self.lane_data = None
        self.sign_label = None
        self.traffic_sign_data = None
        self.lidar_data = None
        self.image_data = None
        self.state = "NORMAL"
        self.pending_action = "NONE"
        self.command_sequence = 0
        # The competition gate is an edge-triggered start event.  Once a
        # confirmed GREEN arrives through /camera_hts/receive (the bridge
        # counterpart of hts/send), the state is latched for this process and
        # later GREEN timeouts cannot return the vehicle to WAIT_GREEN.
        self.competition_started = not self.competition_mode
        self.competition_state = (
            COMPETITION_RUNNING if self.competition_started
            else COMPETITION_WAIT_GREEN)
        self.green_start_until = 0.0

        rospy.on_shutdown(self.publish_stop)
        rospy.loginfo(
            "Main controller ready: observation=%s, publishing /control/cmd at %.1f Hz, "
            "competition_mode=%s, competition_state=%s",
            self.parking_observation_topic, CONTROL_RATE_HZ,
            str(self.competition_mode), self.competition_state)

    def parking_reset_callback(self, msg):
        if msg.data is not True:
            return
        if self.parking_controller.fault or self.parking_controller.done:
            self.parking_controller.reset()
            self.last_parking_state = None
            rospy.loginfo("Parking controller reset")
        else:
            rospy.logwarn(
                "Ignoring /parking/reset while parking state is %s",
                self.parking_controller.state)

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    @staticmethod
    def stop_command():
        return {"speed_raw": 0, "steering_raw": 0}

    def extract_raw_command(self, data):
        """Return a bounded command only when both raw fields are integers."""
        if not isinstance(data, dict):
            return None

        speed_raw = data.get("speed_raw")
        steering_raw = data.get("steering_raw")
        if (isinstance(speed_raw, bool) or
                isinstance(steering_raw, bool) or
                not isinstance(speed_raw, integer_types) or
                not isinstance(steering_raw, integer_types)):
            return None

        return {
            "speed_raw": self.clamp(
                int(speed_raw), -self.max_speed_raw, self.max_speed_raw),
            "steering_raw": self.clamp(
                int(steering_raw), -self.max_steering_raw, self.max_steering_raw)
        }

    def decode_sensor_data(self, receiver, source_name):
        raw_data = receiver.get_latest(self.sensor_timeout_sec)
        if raw_data is None:
            return None

        try:
            data = json.loads(raw_data)
            if not isinstance(data, dict):
                if source_name == "camera_hts" and isinstance(data, string_types):
                    return {
                        "schema": "traffic_sign_v1",
                        "label": data.strip().lower(),
                        "confidence": 1.0,
                    }
                raise ValueError("JSON root must be an object")
            return data
        except (TypeError, ValueError) as exc:
            # The current sign classifier publishes a plain String such as
            # ``park``.  Keep compatibility while the structured publisher is
            # being deployed; all other sensor channels remain strict JSON.
            if source_name == "camera_hts":
                label = str(raw_data).strip().lower()
                if label:
                    return {
                        "schema": "traffic_sign_v1",
                        "label": label,
                        "confidence": 1.0,
                    }
            rospy.logwarn_throttle(
                1.0, "Rejected %s data: %s" % (source_name, exc))
            return None

    def maybe_latch_competition_start(self):
        """Latch the one-time GREEN start event for the competition run."""
        if not getattr(self, "competition_mode", False):
            return False
        if getattr(self, "competition_started", False):
            return False
        if not is_green_start_label(self.sign_label):
            return False

        now = time.time()
        self.competition_started = True
        self.competition_start_time = now
        # GREEN is a one-shot gate, not a second driving mode.  Once the
        # marker is confirmed, hand ownership straight to the white-line
        # controller.  If its command is not ready yet, lane_following()
        # still fails safe to zero instead of driving a blind straight boost.
        self.green_start_until = 0.0
        self.competition_state = COMPETITION_RUNNING
        rospy.loginfo(
            "Competition start latched by confirmed GREEN: "
            "switching directly to lane_following, state=%s, "
            "lane_speed_raw=%d",
            self.competition_state, self.green_start_speed_raw)
        return True

    def competition_waiting_for_green(self):
        """Return true only before the one-time competition start event."""
        return bool(
            getattr(self, "competition_mode", False) and
            not getattr(self, "competition_started", False))

    def green_start_active(self):
        """Return true during the bounded straight initial-speed phase."""
        if (not getattr(self, "competition_mode", False) or
                not getattr(self, "competition_started", False) or
                getattr(self, "competition_state", None) !=
                COMPETITION_STARTING):
            return False
        if time.time() < float(getattr(self, "green_start_until", 0.0)):
            return True
        self.competition_state = COMPETITION_RUNNING
        return False

    def update_sensor_data(self):
        self.lane_data = self.decode_sensor_data(
            self.receiver_camera_yihan, "camera_yihan")
        self.sign_label = self.decode_sensor_data(
            self.receiver_camera_hts, "camera_hts")
        self.maybe_latch_competition_start()
        self.traffic_sign_data = self.decode_sensor_data(
            self.receiver_traffic_sign, "traffic_sign")
        # The active sign node publishes through hts/send -> camera_hts.  The
        # separate traffic_sign topic remains a compatibility input, but must
        # not disconnect post-start seven-class recognition when it is absent.
        if self.traffic_sign_data is None:
            self.traffic_sign_data = self.sign_label
        next_pending_action = update_pending_action(
            self.pending_action, self.traffic_sign_data)
        if next_pending_action != self.pending_action:
            rospy.loginfo(
                "pending_action: %s -> %s",
                self.pending_action,
                next_pending_action)
            self.pending_action = next_pending_action
        self.lidar_data = self.decode_sensor_data(
            self.receiver_lidar, "lidar")
        self.image_data = self.decode_sensor_data(
            self.receiver_image, "image")

    def radar_obstacle_avoidance(self):
        """Use an explicit lidar command; otherwise stop safely."""
        command = self.extract_raw_command(self.lidar_data)
        return command if command is not None else self.stop_command()

    def update_obstacle_scan(self, scan):
        self.timed_obstacle.update(
            scan.ranges, scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max, scan.header.stamp.to_sec())

    def reverse_parking(self):
        """Run the closed-loop parking FSM and publish a debug status."""
        parking_requested = self.is_parking_requested()

        if self.parking_controller.state == "idle":
            if not parking_requested:
                return self.stop_command()
            reform_armed = observation_parking_armed(self.image_data)
            if reform_armed:
                # P arms/selects the mission.  The independent blue start
                # line is the event that transfers command ownership to the
                # reverse-parking FSM; white bay geometry alone must not do it.
                if (not observation_target_ready(self.image_data) or
                        not observation_parking_trigger_visible(
                            self.image_data)):
                    command = self.stop_command()
                    self.publish_parking_status(command)
                    return command
                slot_id = self.image_data.get("selected_slot_id")
                parking_side = observation_parking_side(self.image_data)
                started = self.parking_controller.start(
                    slot_id, armed_only=False, parking_side=parking_side)
            else:
                # Preserve the legacy direct API for old structured sign
                # deployments; the reform path always enters WAIT_BLUE.
                slot_id = (self.image_data.get("slot_id")
                           if isinstance(self.image_data, dict) else None)
                if slot_id is None and isinstance(self.sign_label, dict):
                    slot_id = self.sign_label.get("slot_id")
                started = self.parking_controller.start(slot_id)
            if not started:
                command = self.parking_controller.step(
                    self.image_data, self.lidar_data)
                self.publish_parking_status(command)
                return command

        command = self.parking_controller.step(
            self.image_data, self.lidar_data)
        self.publish_parking_status(command)
        if command.get("state") != self.last_parking_state:
            rospy.loginfo(
                "Parking state: %s (%s)",
                command.get("state"), command.get("reason"))
            self.last_parking_state = command.get("state")
        return command

    def publish_parking_status(self, command):
        status = self.parking_controller.status(
            command=command,
            observation=self.image_data,
        )
        status.update({
            "main_config_valid": bool(self.parking_config_valid),
            "main_config_error": self.parking_config_error,
            "parking_only": bool(self.parking_only),
            "require_calibrated_parking": bool(
                self.require_calibrated_parking),
            "experimental_motion": bool(self.allow_experimental_motion),
            "allow_preparking_approach": bool(
                self.allow_preparking_approach),
            "use_lane_for_preparking": bool(
                getattr(self, "use_lane_for_preparking", False)),
            "preparking_speed_raw": int(
                getattr(self, "preparking_speed_raw", 0)),
            "preparking_lidar_block_reason": (
                self.last_preparking_lidar_reason),
        })
        self.parking_status_sender.send(status)

    def lane_following(self):
        """Use an explicit lane command; otherwise stop safely."""
        command = self.extract_raw_command(self.lane_data)
        return command if command is not None else self.stop_command()

    def preparking_approach(self):
        """Move straight until the selected front bay requests parking."""
        if getattr(self, "use_lane_for_preparking", False):
            lane_command = self.extract_raw_command(self.lane_data)
            if lane_command is not None:
                return lane_command
        speed_limit = getattr(
            self, "max_speed_raw", MAX_SPEED_RAW)
        if getattr(self, "allow_experimental_motion", False):
            speed_limit = self.parking_controller.experimental_speed_limit_raw
        forward_speed = getattr(
            self, "preparking_speed_raw",
            self.parking_controller.config.forward_speed_raw,
        )
        return preparking_straight_command(
            forward_speed,
            speed_limit,
        )

    def green_start(self):
        """Start straight at the configured initial speed for the start phase."""
        return green_start_command(
            self.green_start_speed_raw,
            getattr(self, "max_speed_raw", MAX_SPEED_RAW),
        )

    def traffic_sign_detection(self):
        """Fail-safe stop for a confirmed RED/stop/yield traffic sign."""
        return self.stop_command()

    def decide_control_mode(self):
        self.last_preparking_lidar_reason = None
        # Keep this call here as well as in update_sensor_data so unit tests or
        # alternate callers that provide a fresh sign and invoke the policy
        # directly still get the same edge-triggered transition.
        self.maybe_latch_competition_start()
        if self.competition_waiting_for_green():
            return "waiting_for_green"

        if getattr(self, 'timed_obstacle', None) is not None:
            self.timed_obstacle_command = self.timed_obstacle.command(rospy.get_time())
            if self.timed_obstacle_command is not None:
                return 'timed_obstacle'
            return 'lane_following'

        # Obstacle safety remains active after the GREEN latch.  This is an
        # independent stop condition; absence of GREEN itself is not one.
        if self.lidar_data and self.lidar_data.get("obstacle_detected", False):
            return "radar_obstacle_avoidance"

        parking_requested = self.is_parking_requested()
        parking_armed = observation_parking_armed(self.image_data)
        if self.parking_controller.done and not parking_requested:
            self.parking_controller.reset()
        if self.parking_controller.holding:
            return "reverse_parking"

        # P is an arm event.  Until a fresh target selector result identifies
        # a clear P4/P5 bay, keep the controller idle and let the explicit
        # pre-parking policy continue along the lane.  A blue line by itself
        # cannot enter the manoeuvre.
        if (parking_armed and observation_target_ready(self.image_data) and
                observation_parking_trigger_visible(self.image_data)):
            return "reverse_parking"
        if parking_armed:
            if should_wait_before_parking(
                    self.parking_only,
                    self.require_calibrated_parking,
                    self.parking_controller.config.calibration_complete,
                    self.allow_experimental_motion,
                    self.allow_preparking_approach):
                return "parking_only_wait"
            self.last_preparking_lidar_reason = preparking_lidar_block_reason(
                self.lidar_data,
                self.parking_controller.config.require_lidar_clear,
            )
            if self.last_preparking_lidar_reason is not None:
                return "parking_lidar_wait"
            return "parking_preparking_approach"

        # A legacy request has no reform arm field.  Keep it available only
        # through the explicit compatibility trigger.
        if parking_requested:
            return "reverse_parking"

        # The closed-loop parking launch uses this gate during bring-up so an
        # unrelated stale lane command cannot move the vehicle while the
        # parking sensors are being checked.  Normal start_all.launch keeps
        # the legacy lane-following behaviour unless this is enabled.
        if should_wait_before_parking(
                self.parking_only,
                self.require_calibrated_parking,
                self.parking_controller.config.calibration_complete,
                self.allow_experimental_motion,
                self.allow_preparking_approach):
            return "parking_only_wait"

        if (self.require_calibrated_parking or
                self.allow_preparking_approach):
            self.last_preparking_lidar_reason = preparking_lidar_block_reason(
                self.lidar_data,
                self.parking_controller.config.require_lidar_clear,
            )
            if self.last_preparking_lidar_reason is not None:
                return "parking_lidar_wait"

        # In competition mode the GREEN latch above has already transferred
        # ownership to lane_following.  Do not re-enter a temporary
        # green_start mode, even if an old STARTING state or a later GREEN
        # message is still present.  Non-competition launches preserve the
        # previous level-triggered GREEN behavior for compatibility.
        if (not getattr(self, "competition_mode", False) and
                is_green_start_label(self.sign_label)):
            return "green_start"

        if is_confirmed_sign_label(
                self.sign_label, ("red", "stop", "yield")):
            return "traffic_sign_detection"

        if self.allow_preparking_approach:
            return "parking_preparking_approach"

        return "lane_following"

    def is_parking_requested(self):
        # The parking-only composition has one authoritative request path:
        # the strict observation assembled by parking_observation_node.  The
        # raw sign channel remains an explicit opt-in for legacy deployments.
        if (isinstance(self.image_data, dict) and
                self.image_data.get("schema") == "parking_observation_v1" and
                self.image_data.get("parking_request") is True):
            return True
        if not self.allow_legacy_parking_trigger:
            return False
        if is_confirmed_sign_label(
                self.sign_label, ("park", "parking", "reverse_parking")):
            return True
        if (self.sign_label and
                self.sign_label.get("parking_request") is True):
            return True
        return False

    def command_for_mode(self, control_mode):
        handlers = {
            "timed_obstacle": lambda: self.timed_obstacle_command,
            "radar_obstacle_avoidance": self.radar_obstacle_avoidance,
            "reverse_parking": self.reverse_parking,
            "traffic_sign_detection": self.traffic_sign_detection,
            "green_start": self.green_start,
            "waiting_for_green": lambda: self.stop_command(),
            "lane_following": self.lane_following,
            "parking_preparking_approach": self.preparking_approach,
            "parking_only_wait": lambda: self.stop_command(),
            "parking_lidar_wait": lambda: self.stop_command(),
        }
        handler = handlers.get(control_mode)
        if handler is None:
            return self.stop_command()

        command = handler()
        return command if command is not None else self.stop_command()

    def send_control_command(self, command):
        command = self.extract_raw_command(command)
        if command is None:
            command = self.stop_command()

        message = {
            "version": CONTROL_VERSION,
            "seq": self.command_sequence,
            "speed_raw": int(command["speed_raw"]),
            "steering_raw": int(command["steering_raw"])
        }
        self.control_sender.send(message)
        self.command_sequence = (self.command_sequence + 1) % 256

    def publish_stop(self):
        self.send_control_command(self.stop_command())

    def main_loop(self):
        rate = rospy.Rate(CONTROL_RATE_HZ)
        while not rospy.is_shutdown():
            self.update_sensor_data()
            control_mode = self.decide_control_mode()
            command = self.command_for_mode(control_mode)
            if control_mode in (
                    "parking_only_wait",
                    "parking_lidar_wait",
                    "parking_preparking_approach"):
                # Keep a zero-speed parking heartbeat while the pre-trigger
                # safety gates are closed; this makes bring-up and lidar
                # failures observable without allowing motion.
                self.publish_parking_status(command)
            self.send_control_command(command)
            rate.sleep()


def run():
    """Legacy parking composition, selected explicitly by main.py."""
    controller = AutoDriveController()
    try:
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass
