#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Deterministic, metric, offline competition-course simulation.

The simulator is deliberately a thin world adapter around the existing
in-process ``robot.master.controller.Controller``.  It supplies observations
from a surveyed-scale course model, calls ``Controller.tick`` at a fixed
period, and integrates the returned command with the same bicycle model used
by the production safety/motion code.  It never starts ROS, opens an actuator,
or teleports the vehicle during a run.

Route coordinates are hypotheses read from the supplied screenshots.  Metric
arena, lane, turn-radius, vehicle, and parking-bay dimensions stay explicit
and are reported with their provenance in every JSON result.
"""
from __future__ import division

import argparse
import copy
import hashlib
import json
import math
import os
import sys

try:
    import yaml
except ImportError:
    yaml = None


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, 'src')
if SRC not in sys.path:
    sys.path.insert(0, SRC)
VEHICLE_CONTROL_SRC = os.path.join(SRC, 'ros', 'lane', 'src')
if VEHICLE_CONTROL_SRC not in sys.path:
    sys.path.insert(0, VEHICLE_CONTROL_SRC)

from robot.common.contracts import command_to_model_steering
from robot.common.config import load_config as load_robot_config
from robot.common.geometry import bicycle
from robot.common.geometry import distance
from robot.common.geometry import footprint
from robot.common.geometry import local
from robot.common.geometry import wrap
from robot.common.geometry import world
from robot.master.controller import Controller
from robot.lidar.scan import Scan


COURSE_DEFAULT = os.path.join(SRC, 'robot', 'config', 'course.yaml')
COMPETITION_DEFAULT = os.path.join(SRC, 'robot', 'config', 'competition.yaml')
MANEUVERS_DEFAULT = os.path.join(SRC, 'robot', 'config', 'maneuvers.yaml')

SOURCE_HASH_PATHS = {
    'lane_controller': os.path.join(SRC, 'robot', 'lane', 'controller.py'),
    'lane_preview': os.path.join(SRC, 'robot', 'lane', 'preview.py'),
    'motion_controller': os.path.join(SRC, 'robot', 'motion', 'controller.py'),
    'motion_tracker': os.path.join(SRC, 'robot', 'motion', 'tracker.py'),
    'shared_planner': os.path.join(SRC, 'robot', 'common', 'planning.py'),
    'contracts': os.path.join(SRC, 'robot', 'common', 'contracts.py'),
    'lidar_scan': os.path.join(SRC, 'robot', 'lidar', 'scan.py'),
    'config_loader': os.path.join(SRC, 'robot', 'common', 'config.py'),
    'obstacle_controller': os.path.join(SRC, 'robot', 'obstacle', 'controller.py'),
    'geometry': os.path.join(SRC, 'robot', 'common', 'geometry.py'),
    'master_controller': os.path.join(SRC, 'robot', 'master', 'controller.py'),
    'master_state_machine': os.path.join(SRC, 'robot', 'master', 'state_machine.py'),
    'turn_controller': os.path.join(SRC, 'robot', 'turn', 'controller.py'),
    'turn_planner': os.path.join(SRC, 'robot', 'turn', 'planner.py'),
    'parking_controller': os.path.join(SRC, 'robot', 'parking', 'controller.py'),
    'parking_planner': os.path.join(SRC, 'robot', 'parking', 'planner.py'),
    'parking_detection': os.path.join(SRC, 'robot', 'parking', 'detection.py'),
    'parking_config': os.path.join(SRC, 'robot', 'parking', 'config.yaml'),
    'camera_observations': os.path.join(SRC, 'robot', 'camera', 'observations.py'),
    'parking_scene': os.path.join(SRC, 'robot', 'parallel_parking', 'scene.py'),
    'parallel_parking_controller': os.path.join(
        SRC, 'robot', 'parallel_parking', 'controller.py'),
    'parallel_parking_planner': os.path.join(
        SRC, 'robot', 'parallel_parking', 'planner.py'),
    'parallel_parking_config': os.path.join(
        SRC, 'robot', 'parallel_parking', 'config.yaml'),
    'course': COURSE_DEFAULT,
    'competition_config': COMPETITION_DEFAULT,
    'maneuvers_config': MANEUVERS_DEFAULT,
    'lane_config': os.path.join(SRC, 'robot', 'lane', 'config.yaml'),
    'obstacle_config': os.path.join(SRC, 'robot', 'obstacle', 'config.yaml'),
    'vehicle_launch_script': os.path.join(
        ROOT, 'tools', 'sign_detector_20260916', 'run_yolo_vehicle.sh'),
    'simulator': os.path.abspath(__file__),
}


def source_hashes():
    """Hash the recorded control/model sources before and after a run."""
    result = {}
    for name in sorted(SOURCE_HASH_PATHS):
        path = SOURCE_HASH_PATHS[name]
        digest = hashlib.sha256()
        try:
            with open(path, 'rb') as stream:
                while True:
                    chunk = stream.read(1024 * 1024)
                    if not chunk:
                        break
                    digest.update(chunk)
            result[name] = dict(path=path, sha256=digest.hexdigest())
        except IOError as exc:
            result[name] = dict(path=path, error=str(exc))
    return result


def _read_yaml(path):
    if yaml is None:
        raise RuntimeError('PyYAML is required by the existing robot config loader')
    with open(path) as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError('expected YAML mapping: %s' % path)
    return value


def _merge(base, override):
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def load_course(path=COURSE_DEFAULT):
    """Read the declarative metric/inferred course model."""
    course = _read_yaml(path)
    if course.get('schema') != 'competition-course-v1':
        raise ValueError('unsupported course schema')
    if not course.get('routes'):
        raise ValueError('course has no routes')
    return course


def load_sim_config(competition_path=COMPETITION_DEFAULT,
                    maneuvers_path=MANEUVERS_DEFAULT):
    """Load the same competition + maneuver profile used by the launch.

    The startup overrides applied by ``master.main`` and the command-space
    scale exposed by the normal vehicle launch are made explicit. Obstacle,
    right-action, parking, wait, and U-turn policy remain production values;
    the simulator changes the input source, not the policy under test.
    """
    # Use the same catalog-checked module override order as stack.launch.
    # This is important for obstacle and explicit parallel-parking profiles:
    # reading only the two top-level files silently drops those overrides.
    cfg = load_robot_config(os.path.dirname(competition_path),
                            base_path=competition_path,
                            maneuver_path=maneuvers_path)
    cfg.update({
        'pose_mode': 'command_model',
        # master.main applies these startup overrides before the production
        # controller node starts.  Feed GREEN below instead of skipping that
        # gate in the offline run.
        'wait_green': True,
        'sign_ttl': 0,
        # Current no-argument vehicle script overrides used by the real
        # launch.  These are policy inputs, not simulator-only tuning.
        'lane_hz': 12,
        'lane_window_height': 40,
        'lane_min_span': 0.15,
        'lane_curvature_preview': True,
        'lane_curve_speed_raw': 12,
        'left_turn_radius': 0.65,
        'left_turn_exit': 0.30,
        'right_turn_radius': 0.55,
        'right_turn_exit': 0.25,
        'right_reverse_entry_m': 0.25,
        'right_exit_on_blue': True,
        'intersection_wait_s': 1.0,
        # full.launch currently passes this command-space scale to the
        # ordinary lane/action bridge; it is distinct from max_steer.
        'steering_command_scale_rad': 0.03,
    })
    cfg['lidar'] = dict(cfg.get('lidar', {}))
    return cfg


def _road_geometry(course):
    """Combine fixed map geometry with route-specific bay placement.

    The map's rounded boundary and islands are measured geometry.  Bay
    openings/placement inferred from the screenshots live in a separate
    provenance section, but the truth checker consumes one merged view.
    """
    provenance = course.get('provenance', {})
    geometry = {}
    for source in ('measured', 'inferred'):
        values = provenance.get(source, {}).get('road_geometry', {})
        if isinstance(values, dict):
            geometry = _merge(geometry, values)
    return geometry


def _pose_tuple(value):
    return (float(value[0]), float(value[1]), float(value[2]))


def _path_row(pose, steer=0.0):
    return tuple(pose) + (1, float(steer))


class RouteModel(object):
    """Expanded dense route path plus static event/parking geometry."""

    def __init__(self, route_name, route, cfg, dt):
        self.name = route_name
        self.definition = route
        self.cfg = cfg
        self.dt = dt
        self.path = [_path_row(_pose_tuple(route['start']))]
        self.events = []
        if route.get('centerline'):
            self._expand_centerline()
        else:
            raise ValueError('screenshot route requires an explicit centerline')
        self.progress = [0.0]
        for i in range(1, len(self.path)):
            self.progress.append(self.progress[-1] +
                                 distance(self.path[i - 1], self.path[i]))

    def _expand_centerline(self):
        """Densify a screenshot-derived world polyline at 25 mm."""
        points = [tuple(float(v) for v in point[:2])
                  for point in self.definition.get('centerline', [])]
        if not points:
            raise ValueError('centerline is empty')
        start = _pose_tuple(self.definition['start'])
        if distance(points[0], start[:2]) > .05:
            points.insert(0, start[:2])
        rows = []
        for i, point in enumerate(points):
            if i == 0:
                heading = start[2]
            elif i == len(points) - 1:
                heading = math.atan2(point[1] - points[i - 1][1],
                                     point[0] - points[i - 1][0])
            else:
                heading = math.atan2(points[i + 1][1] - points[i - 1][1],
                                     points[i + 1][0] - points[i - 1][0])
            rows.append((point[0], point[1], heading))
        self.path = [_path_row(rows[0])]
        for i in range(1, len(rows)):
            a, b = rows[i - 1], rows[i]
            length = distance(a, b)
            count = max(1, int(math.ceil(length / .025)))
            for j in range(1, count + 1):
                fraction = j / count
                x = a[0] + fraction * (b[0] - a[0])
                y = a[1] + fraction * (b[1] - a[1])
                self.path.append(_path_row((x, y, b[2])))
        for item in self.definition.get('events', []):
            marker = item.get('marker') or item.get('pose')
            if marker is None:
                raise ValueError('centerline event needs marker/pose: %r' % item)
            marker = _pose_tuple(marker)
            action = str(item['action']).upper()
            event = dict(id=str(item['id']), action=action,
                         slot=item.get('slot'), marker_pose=marker,
                         marker_progress=None, action_start=None,
                         target_pose=(_pose_tuple(item['target'])
                                      if item.get('target') is not None else None),
                         status='pending',
                         source='inferred_from_screenshot')
            event['marker_progress'] = self._path_progress_to_pose(marker)
            self.events.append(event)

    def _path_progress_to_pose(self, pose):
        if not self.path:
            return 0.0
        return min(distance(pose, p[:3]) for p in self.path)

    def nearest(self, pose, hint=0):
        """Return (index, along-route distance, Euclidean error).

        A monotone search window prevents a crossing in the inferred route
        from making the virtual camera jump to an earlier branch.
        """
        if not self.path:
            return 0, 0.0, float('inf')
        lo = max(0, int(hint) - 40)
        hi = min(len(self.path), int(hint) + 500)
        if hi <= lo:
            lo, hi = 0, len(self.path)
        best = min(range(lo, hi), key=lambda i: distance(pose, self.path[i]))
        return best, self.progress[best], distance(pose, self.path[best])

    def point_at(self, index):
        index = max(0, min(len(self.path) - 1, int(index)))
        return self.path[index][:3]


def _event_target_slot(event, course):
    slot_name = event.get('slot') or 'P4'
    spec = course.get('provenance', {}).get('measured', {}).get('slots', {}).get(slot_name, {})
    return slot_name, spec


class CompetitionSimulation(object):
    """One deterministic route execution and its metric evidence."""

    def __init__(self, route_name, course=None, cfg=None, dt=0.05,
                 course_path=COURSE_DEFAULT):
        self.course = course if course is not None else load_course(course_path)
        self.cfg = copy.deepcopy(cfg if cfg is not None else load_sim_config())
        if route_name not in self.course['routes']:
            raise KeyError('unknown route: %s' % route_name)
        route_definition = self.course['routes'][route_name]
        parking_events = [e for e in route_definition.get('events', [])
                          if str(e.get('action', '')).upper() == 'PARKING']
        if parking_events and parking_events[-1].get('slot'):
            # The P identity comes from the route screenshot. Select the
            # production entry profile that matches that fixed bay geometry.
            slot = parking_events[-1]['slot']
            self.cfg['parking_slot'] = slot
            # Both bay families use the production forward closed-loop profile.
            self.cfg['parking_mode'] = 'forward_plan'
        self.dt = float(dt)
        if not 0.01 <= self.dt <= .2:
            raise ValueError('dt must be between .01 and .2 s')
        self.route = RouteModel(route_name, route_definition,
                                self.cfg, self.dt)
        self.controller = Controller(self.cfg)
        self.now = 0.0
        self.closed = False
        self.nearest_index = 0
        self.active_event = 0
        self.stall_since = None
        self.failure = None
        self.trajectory = []
        self.event_log = []
        self._event_started_at = {}
        self.last_slot_observations = []
        self.source_hashes_before = source_hashes()
        self.controller.set_pose(_pose_tuple(self.route.definition['start']), 0.0)

    def close(self):
        if not self.closed:
            self.controller.close()
            self.closed = True

    def _fail(self, reason, extra=None):
        if self.failure is not None:
            return
        index, progress, error = self.route.nearest(self.controller.pose,
                                                    self.nearest_index)
        self.failure = dict(
            reason=str(reason),
            state=str(self.controller.state),
            controller_reason=str(getattr(self.controller, 'reason', '')),
            time_s=round(self.now, 6),
            pose=[round(float(v), 9) for v in self.controller.pose],
            route_progress_m=round(float(progress), 6),
            route_error_m=round(float(error), 6),
            path_index=int(index),
            event=(self.route.events[self.active_event]['id']
                   if self.active_event < len(self.route.events) else None))
        if extra:
            self.failure.update(extra)

    def _active_route_event(self):
        while self.active_event < len(self.route.events):
            event = self.route.events[self.active_event]
            if event['status'] == 'complete':
                self.active_event += 1
                continue
            return event
        return None

    def _update_event_status(self):
        event = self._active_route_event()
        if event is None:
            return
        action = event['action']
        if event['status'] == 'pending' and self.controller.action == action:
            event['status'] = 'active'
            self._event_started_at[event['id']] = self.now
            self.event_log.append(dict(id=event['id'], action=action,
                                       status='started', time_s=round(self.now, 6),
                                       pose=list(self.controller.pose)))
        if event['status'] == 'active':
            if action == 'PARKING' and self.controller.state == 'FINISHED':
                event['status'] = 'complete'
            elif (action != 'PARKING' and self.controller.action is None and
                  getattr(self.controller, 'last_completed_action', None) == action and
                  self.now > self._event_started_at.get(event['id'], -1) + .1):
                event['status'] = 'complete'
        if event['status'] == 'complete':
            self.event_log.append(dict(id=event['id'], action=action,
                                       status='complete', time_s=round(self.now, 6),
                                       pose=list(self.controller.pose),
                                       reason=str(self.controller.reason)))
            self.active_event += 1

    def _lane_observation(self):
        index, unused_progress, unused_error = self.route.nearest(
            self.controller.pose, self.nearest_index)
        self.nearest_index = max(self.nearest_index, index)
        points = []
        # A fresh, long observation is intentional: it satisfies the same
        # forward-point/span gate used by ordinary lane motion.
        for offset in range(2, 56):
            j = min(len(self.route.path) - 1, self.nearest_index + offset)
            x, y = local(self.controller.pose, self.route.point_at(j))
            if x > .03:
                points.append((x, y))
        # No generic straight line is fabricated when the mapped lane has
        # ended or the truth has left its forward field of view.  The real
        # controller must then report an unreliable/stale lane observation.
        return points, (.98 if len(points) >= 2 else 0.0)

    def _event_observation(self):
        event = self._active_route_event()
        if event is None:
            return None, [], []
        marker = event['marker_pose']
        x, y = local(self.controller.pose, marker)
        markers, lines = [], []
        # The sign is visible before the blue line. Repeating the same fresh
        # frame is how the production vote counter receives consecutive votes.
        if (self.controller.action is None and self.controller.pending is None and
                .55 <= x <= 1.20 and abs(y) <= self.cfg['lane_width']):
            self.controller.observe_sign(event['action'], .99, self.now, self.now)
        if .03 < x <= .72 and abs(y) <= self.cfg['lane_width']:
            markers.append(dict(kind='junction', x=x, y=y, length=.80))
            line_yaw = wrap(marker[2] - self.controller.pose[2])
            lines.append(dict(x=x, y=y, yaw=line_yaw, length=.80))
        return event, markers, lines

    def _parking_lines(self):
        """Return current-view paint edges from the measured bay rectangle."""
        parking = self._parking_event()
        if parking is None:
            return []
        target = parking['target_pose']
        unused_slot_name, spec = _event_target_slot(parking, self.course)
        length = float(spec.get('length_m', .45))
        width = float(spec.get('width_m', .38))
        half_length, half_width = length / 2.0, width / 2.0
        # Two side rails and the road-facing mouth are the actual measured
        # slot edges. No bumper clearance is substituted for bay dimensions.
        side_left = (world(target, (-half_length, -half_width)),
                     world(target, (half_length, -half_width)))
        side_right = (world(target, (-half_length, half_width)),
                      world(target, (half_length, half_width)))
        mouth = (world(target, (-half_length, -half_width)),
                 world(target, (-half_length, half_width)))
        return [side_left, side_right, mouth]

    def _parking_event(self):
        for event in self.route.events:
            if event['action'] == 'PARKING':
                return event
        return None

    def _parking_slot_observations(self, parking):
        """Project metric bay poses into the current camera frame.

        These rows are synthetic ground truth for the offline world adapter;
        they carry the route's explicit target identity and do not represent
        visual slot recognition. Perpendicular P4/P5 rows are both supplied
        because the production profile requires its configured candidate
        count before selecting the requested rank.
        """
        geometry = _road_geometry(self.course)
        poses = geometry.get('parking_slot_poses', {})
        target_name = parking.get('slot')
        if not target_name or target_name not in poses:
            return []
        target_spec = self.course['provenance']['measured']['slots'].get(target_name, {})
        if target_spec.get('kind') == 'parallel':
            names = [target_name]
        else:
            names = ['P4', 'P5']
        rows = []
        for name in names:
            pose = _pose_tuple(poses[name])
            spec = self.course['provenance']['measured']['slots'][name]
            relative = local(self.controller.pose, pose)
            rows.append(dict(
                id=name,
                x=relative[0], y=relative[1],
                yaw=wrap(pose[2] - self.controller.pose[2]),
                kind=spec['kind'], length=spec['length_m'],
                width=spec['width_m']))
        return rows

    def _observe(self):
        self.last_slot_observations = []
        if self.controller.state == 'WAIT_GREEN':
            # Green is a real input gate.  Feed the configured vote count once
            # per fresh frame rather than bypassing the startup transition.
            self.controller.observe_sign('GREEN', .99, self.now, self.now)
        # Start with +inf rays, then ray-cast any explicitly supplied course
        # cylinder.  No obstacle placement is guessed from a screenshot.
        ranges = self._scan_ranges()
        self.controller.scan = Scan(
            ranges, -math.pi, 2 * math.pi / 360,
            .05, 6.0, self.controller.pose, self.cfg['lidar'], self.now)
        lane, confidence = self._lane_observation()
        self.controller.observe_lane(lane, confidence, self.now)
        event, markers, lines = self._event_observation()
        self.controller.observe_ground(dict(
            source='front', part='markers', markers=markers,
            blue_lines=lines, slots=[]), self.now)
        if event is not None and event['action'] == 'PARKING':
            marker_x = local(self.controller.pose, event['marker_pose'])[0]
            if (marker_x <= self.cfg.get('straight_align_distance', 1.6) or
                    self.controller.pending == 'PARKING' or
                    self.controller.action == 'PARKING'):
                self.last_slot_observations = self._parking_slot_observations(event)
                if self.last_slot_observations:
                    self.controller.observe_ground(dict(
                        source='front', part='slots',
                        slots=self.last_slot_observations,
                        target_id=event.get('slot'),
                        synthetic_ground_truth=True), self.now)
        if self.controller.parking_entry is not None or self.controller.state == 'PARKING':
            local_lines = [[local(self.controller.pose, p) for p in pair]
                           for pair in self._parking_lines()]
            self.controller.observe_ground(dict(
                source='front', part='parking_lines', lines=local_lines), self.now)
        return event

    def _obstacles(self):
        rows = []
        rows.extend(self.course.get('obstacles', []) or [])
        rows.extend(self.route.definition.get('obstacles', []) or [])
        return rows

    def _scan_ranges(self):
        count = 360
        angle_min = -math.pi
        increment = 2 * math.pi / count
        ranges = [float('inf')] * count
        for obstacle in self._obstacles():
            center = obstacle.get('center') or obstacle.get('centre')
            if center is None:
                continue
            radius = float(obstacle.get('radius_m', obstacle.get('radius', .10)))
            cx, cy = local(self.controller.pose, center)
            for i in range(count):
                angle = angle_min + i * increment
                vx, vy = math.cos(angle), math.sin(angle)
                projection = cx * vx + cy * vy
                perpendicular = cx * cx + cy * cy - projection * projection
                if projection <= 0 or perpendicular >= radius * radius:
                    continue
                root = math.sqrt(max(0.0, radius * radius - perpendicular))
                hit = projection - root
                if hit < self.cfg['lidar'].get('range_cap', 6.0) and hit >= .05:
                    ranges[i] = min(ranges[i], hit)
        return ranges

    def _check_truth_geometry(self):
        half = float(self.course['provenance']['measured']['arena_width_m']) / 2.0
        geometry = _road_geometry(self.course)
        for point in footprint(self.controller.pose, self.cfg, spacing=.08):
            if not self._inside_outer(point, half, geometry):
                self._fail('road_boundary', dict(boundary_half_width_m=half,
                                                 offending_point=[point[0], point[1]]))
                return
            if not self._inside_road_union(point, geometry):
                self._fail('road_island', dict(offending_point=[point[0], point[1]],
                                               road_geometry='outer_minus_islands'))
                return
        for obstacle in self._obstacles():
            center = obstacle.get('center') or obstacle.get('centre')
            if center is None:
                continue
            radius = float(obstacle.get('radius_m', obstacle.get('radius', .10)))
            if any(distance(point, center) <= radius + self.cfg.get('obstacle_margin', .035)
                   for point in footprint(self.controller.pose, self.cfg, spacing=.08)):
                self._fail('obstacle_collision', dict(obstacle=obstacle))
                return

    @staticmethod
    def _inside_outer(point, half, geometry):
        radius = float(geometry.get('outer_corner_radius_m', 1.8))
        core = half - radius
        x, y = point
        if abs(x) > half or abs(y) > half:
            return False
        if abs(x) <= core or abs(y) <= core:
            return True
        cx = core if x >= 0 else -core
        cy = core if y >= 0 else -core
        return math.hypot(x - cx, y - cy) <= radius

    @staticmethod
    def _inside_box(point, box):
        return (float(box.get('xmin', -float('inf'))) <= point[0] <= float(box.get('xmax', float('inf'))) and
                float(box.get('ymin', -float('inf'))) <= point[1] <= float(box.get('ymax', float('inf'))))

    def _inside_road_union(self, point, geometry):
        """Outer rounded arena minus measured island footprints and openings."""
        x, y = point
        pill_centres = geometry.get('top_pill_centres_m', [])
        in_pill = (abs(y - 1.20) <= .60 and -1.20 <= x <= 1.20)
        in_pill = in_pill or any(distance(point, centre) <= .60 for centre in pill_centres)
        in_bottom = any(distance(point, centre) <= .60
                        for centre in geometry.get('bottom_island_centres_m', []))
        if not (in_pill or in_bottom):
            return True
        if any(self._inside_box(point, box)
               for box in geometry.get('central_openings', [])):
            return True
        for event in self.route.events:
            if event.get('action') == 'PARKING' and event.get('slot'):
                box = geometry.get('parking_bay_regions', {}).get(event['slot'])
                if box and self._inside_box(point, box):
                    return True
        return False

    def step(self):
        if self.failure is not None:
            return False
        self.now += self.dt
        self._observe()
        before = self.controller.pose
        try:
            command = self.controller.tick(self.now)
        except Exception as exc:
            self._fail('controller_exception:'+str(exc))
            return False
        if command is None or len(command) != 2:
            self._fail('invalid_controller_command', dict(command=command))
            return False
        speed, steering = command
        try:
            speed = float(speed)
            steering = float(steering)
            physical = command_to_model_steering(steering, self.cfg)
            gain = self.cfg['raw_to_mps']['forward' if speed >= 0 else 'reverse']
            truth_pose = bicycle(before, speed * gain * self.dt,
                                 physical, self.cfg['wheelbase'])
            # This is a measurement update of the command-integrated truth,
            # not a teleport. No pose is supplied to fake completion.
            self.controller.set_pose(truth_pose, self.now)
        except Exception as exc:
            self._fail('truth_integration_exception:'+str(exc))
            return False
        index, progress, error = self.route.nearest(self.controller.pose,
                                                    self.nearest_index)
        self.nearest_index = max(self.nearest_index, index)
        self._update_event_status()
        record = dict(
            t_s=round(self.now, 6),
            pose=[round(float(v), 9) for v in self.controller.pose],
            command_raw=round(speed, 6),
            command_steer=round(steering, 9),
            physical_steer=round(physical, 9),
            state=str(self.controller.state),
            reason=str(getattr(self.controller, 'reason', '')),
            action=getattr(self.controller, 'action', None),
            pending=getattr(self.controller, 'pending', None),
            route_progress_m=round(float(progress), 6),
            route_error_m=round(float(error), 6),
            lane_points=len(getattr(self.controller, 'lane', [])),
            scan_valid_rays=int(getattr(self.controller.scan, 'valid_rays', 0)),
            parking_slot_rows=len(self.last_slot_observations),
        )
        self.trajectory.append(record)
        if self.controller.state == 'FAULT':
            self._fail('controller_fault')
            return False
        self._check_truth_geometry()
        # A non-moving command may be an expected blue/cusp/planning stop. A
        # persistent stop still gets a localized first-failure record.
        if abs(speed) < 1e-9:
            if self.stall_since is None:
                self.stall_since = self.now
            elif (self.now - self.stall_since > 8.0 and
                  self.controller.state not in ('FINISHED', 'BLUE_APPROACH',
                                                'BLUE_STOP', 'INTERSECTION_WAIT',
                                                'PLANNING', 'PARKING')):
                self._fail('stalled:'+str(self.controller.reason))
        else:
            self.stall_since = None
        return self.failure is None

    def run(self, max_time=180.0):
        max_time = float(max_time)
        if max_time <= 0:
            raise ValueError('max_time must be positive')
        try:
            limit = int(math.ceil(max_time / self.dt))
            for unused in range(limit):
                if self.failure is not None:
                    break
                if (self.active_event >= len(self.route.events) and
                        self.controller.state == 'FINISHED'):
                    break
                if not self.step():
                    break
            if self.failure is None:
                if self.active_event < len(self.route.events):
                    self._fail('simulation_timeout')
                elif self.controller.state != 'FINISHED':
                    self._fail('route_not_terminal')
            status = 'passed' if self.failure is None else 'failed'
            result = self.result(status)
            return result
        finally:
            self.close()

    def result(self, status=None):
        if status is None:
            status = 'failed' if self.failure else 'running'
        measured = copy.deepcopy(self.course['provenance']['measured'])
        configured = copy.deepcopy(self.course['provenance']['configured_model'])
        inferred = copy.deepcopy(self.course['provenance']['inferred'])
        hashes_after = source_hashes()
        hashes_before = copy.deepcopy(self.source_hashes_before)
        return dict(
            schema='competition-sim-result-v1',
            route=self.route.name,
            source=self.route.definition.get('source'),
            placement=self.route.definition.get('placement'),
            status=status,
            metric_provenance=dict(measured=measured,
                                   configured_model=configured,
                                   inferred=inferred),
            sensor_provenance=dict(
                lane='fresh points projected from the mapped route centerline',
                parking_slots=('synthetic ground-truth metric bay poses projected '
                                'into the current vehicle frame; explicit route '
                                'identity is not visual recognition'),
                obstacles='explicit course/route circles only; no screenshot placement is guessed',
                cadence=('Controller.tick and the synthetic observation adapter run '
                         'at fixed %.3f Hz (dt %.3f s) with ideal no-delay '
                         'observations. Launch lane_hz=%.3f and ground_hz=%.3f are '
                         'recorded only; their sensor-link cadence is not emulated, '
                         'so this is not full sensor-link equivalence.' %
                         (1.0 / self.dt, self.dt, self.cfg['lane_hz'],
                          self.cfg.get('ground_hz', 0.0))),
                sensor_cadence_emulated=False),
            source_hashes=dict(before=hashes_before, after=hashes_after,
                              stable=(hashes_before == hashes_after)),
            model=dict(dt_s=self.dt, wheelbase_m=self.cfg['wheelbase'],
                       body_width_m=self.cfg['body_width'],
                       front_overhang_m=self.cfg['front_overhang'],
                       rear_overhang_m=self.cfg['rear_overhang'],
                       forward_command_raw=self.cfg['speed_raw']['lane'],
                       raw_to_mps_forward=self.cfg['raw_to_mps']['forward'],
                       raw_forward_mps=(self.cfg['speed_raw']['lane'] *
                                        self.cfg['raw_to_mps']['forward']),
                       max_steer_rad=self.cfg['max_steer'],
                       controller='robot.master.controller.Controller',
                       truth='robot.common.geometry.bicycle'),
            launch_profile=dict(
                wait_green=self.cfg['wait_green'],
                sign_ttl=self.cfg['sign_ttl'],
                lane_hz=self.cfg['lane_hz'],
                ground_hz=self.cfg.get('ground_hz'),
                lane_window_height=self.cfg['lane_window_height'],
                lane_min_span=self.cfg['lane_min_span'],
                lane_curvature_preview=self.cfg['lane_curvature_preview'],
                lane_curve_speed_raw=self.cfg['lane_curve_speed_raw'],
                steering_command_scale_rad=self.cfg['steering_command_scale_rad'],
                right_exit_on_blue=self.cfg['right_exit_on_blue'],
                intersection_wait_s=self.cfg['intersection_wait_s'],
                obstacle_timed_bypass=dict(
                    trigger_distance_m=self.cfg['timed_bypass_trigger_distance_m'],
                    settle_s=self.cfg['timed_bypass_settle_s'],
                    left_s=self.cfg['timed_bypass_left_s'],
                    right_s=self.cfg['timed_bypass_right_s']),
                parallel_parking=dict(
                    turn_radius_m=self.cfg['parallel_parking_turn_radius_m'],
                    confirm_frames=self.cfg['parallel_parking_confirm_frames'],
                    slot_size_tolerance_m=self.cfg['parallel_parking_slot_size_tolerance_m'])),
            duration_s=round(self.now, 6),
            failure=self.failure,
            events=copy.deepcopy(self.event_log),
            event_state=[dict(id=e['id'], action=e['action'], status=e['status'])
                         for e in self.route.events],
            trajectory=self.trajectory,
        )


def _svg_polyline(points, scale, ox, oy):
    return ' '.join('%.2f,%.2f' % (ox + scale * p[0], oy - scale * p[1])
                    for p in points)


def render_svg(sim_result, route_model, course, path):
    """Render a compact, deterministic top-down evidence drawing."""
    size, scale, ox, oy = 800, 110.0, 400.0, 400.0
    half = course['provenance']['measured']['arena_width_m'] / 2.0
    geometry = _road_geometry(course)
    route_points = [p[:3] for p in route_model.path]
    truth_points = [row['pose'] for row in sim_result.get('trajectory', [])]
    if not truth_points:
        truth_points = [route_model.definition['start']]
    event_markup = []
    for event in route_model.events:
        x, y, yaw = event['marker_pose']
        cx, cy = ox + scale * x, oy - scale * y
        color = '#148a55' if event['status'] == 'complete' else '#c74343'
        event_markup.append(
            '<circle cx="%.2f" cy="%.2f" r="6" fill="%s"/>' % (cx, cy, color))
        event_markup.append(
            '<text x="%.2f" y="%.2f" font-size="13" fill="#111">%s:%s</text>' %
            (cx + 8, cy - 8, event['id'], event['action']))
    solids = []
    top_centres = geometry.get('top_pill_centres_m', [])
    if top_centres:
        # The central rectangle joins the two measured R0.60 end caps.
        x0 = ox + scale * min(point[0] for point in top_centres)
        x1 = ox + scale * max(point[0] for point in top_centres)
        y0 = oy - scale * 1.80
        y1 = oy - scale * .60
        solids.append('<rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" fill="#fff" stroke="#777" stroke-width="1"/>' %
                      (x0, y0, x1 - x0, y1 - y0))
    for center in top_centres + geometry.get('bottom_island_centres_m', []):
        cx, cy = ox + scale * center[0], oy - scale * center[1]
        solids.append('<circle cx="%.2f" cy="%.2f" r="%.2f" fill="#fff" stroke="#777" stroke-width="1"/>' %
                      (cx, cy, scale * .60))
    for box in geometry.get('central_openings', []):
        x = ox + scale * box['xmin']
        y = oy - scale * box['ymax']
        solids.append('<rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" fill="#f2f2f2" stroke="#777" stroke-width="1"/>' %
                      (x, y, scale * (box['xmax'] - box['xmin']),
                       scale * (box['ymax'] - box['ymin'])))
    for slot, box in sorted(geometry.get('parking_bay_regions', {}).items()):
        x = ox + scale * box['xmin']
        y = oy - scale * box['ymax']
        solids.append('<rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" fill="none" stroke="#3980b8" stroke-width="1" stroke-dasharray="3,3"/>' %
                      (x, y, scale * (box['xmax'] - box['xmin']),
                       scale * (box['ymax'] - box['ymin'])))
        solids.append('<text x="%.2f" y="%.2f" font-size="10" fill="#3980b8">%s</text>' %
                      (x + 3, y + 12, slot))
    title = '%s — %s' % (sim_result['route'], sim_result['status'])
    failure = sim_result.get('failure') or {}
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<svg xmlns="http://www.w3.org/2000/svg" width="800" height="800" viewBox="0 0 800 800">',
        '<rect width="800" height="800" fill="#fafafa"/>',
        '<rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" rx="%.2f" ry="%.2f" fill="#f2f2f2" stroke="#222" stroke-width="3"/>' %
        (ox - scale * half, oy - scale * half, scale * 2 * half, scale * 2 * half,
         scale * geometry.get('outer_corner_radius_m', 1.8),
         scale * geometry.get('outer_corner_radius_m', 1.8)),
    ]
    lines.extend(solids)
    lines.extend([
        '<polyline points="%s" fill="none" stroke="#aaa" stroke-width="2" stroke-dasharray="7,7"/>' %
        _svg_polyline(route_points, scale, ox, oy),
        '<polyline points="%s" fill="none" stroke="#d22" stroke-width="3"/>' %
        _svg_polyline(truth_points, scale, ox, oy),
    ])
    lines.extend(event_markup)
    lines.extend([
        '<text x="20" y="28" font-size="18" font-family="sans-serif">%s</text>' % title,
        '<text x="20" y="52" font-size="12" font-family="monospace">duration=%.2fs failure=%s</text>' %
        (sim_result.get('duration_s', 0.0), str(failure.get('reason', 'none'))),
        '</svg>',
    ])
    with open(path, 'w') as stream:
        stream.write('\n'.join(lines))


def write_result(result, sim, output_dir):
    if not os.path.isdir(output_dir):
        os.makedirs(output_dir)
    stem = result['route']
    json_path = os.path.join(output_dir, stem + '.json')
    svg_path = os.path.join(output_dir, stem + '.svg')
    with open(json_path, 'w') as stream:
        json.dump(result, stream, sort_keys=True, indent=2)
        stream.write('\n')
    render_svg(result, sim.route, sim.course, svg_path)
    return json_path, svg_path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--course', default=COURSE_DEFAULT)
    parser.add_argument('--route', choices=['all', 'image1_left', 'image1_right', 'image2'],
                        default='all')
    parser.add_argument('--output-dir', default=os.path.join(ROOT, 'competition_sim_results'))
    parser.add_argument('--dt', type=float, default=.05)
    parser.add_argument('--max-time', type=float, default=180.0)
    parser.add_argument('--quiet', action='store_true')
    args = parser.parse_args(argv)
    course = load_course(args.course)
    names = list(course['routes']) if args.route == 'all' else [args.route]
    names.sort()
    summary = []
    for name in names:
        sim = CompetitionSimulation(name, course=course, dt=args.dt)
        result = sim.run(max_time=args.max_time)
        json_path, svg_path = write_result(result, sim, args.output_dir)
        row = dict(route=name, status=result['status'], failure=result.get('failure'),
                   json=json_path, svg=svg_path,
                   source_hashes=result.get('source_hashes'))
        summary.append(row)
        if not args.quiet:
            print('%s: %s%s' % (name, result['status'],
                                '' if not result.get('failure') else
                                ' (%s at %.2fs)' % (result['failure']['reason'],
                                                     result['failure']['time_s'])))
    summary_path = os.path.join(args.output_dir, 'summary.json')
    with open(summary_path, 'w') as stream:
        json.dump(summary, stream, sort_keys=True, indent=2)
        stream.write('\n')
    if not args.quiet:
        print('summary: %s' % summary_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
