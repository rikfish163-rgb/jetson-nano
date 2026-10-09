# -*- coding: utf-8 -*-
"""Build a measured local scene for configured parking bays.

The camera reports a bay pose relative to the current vehicle frame.  A
planner, however, needs one fixed bay frame while the vehicle pose changes as
the car approaches the bay.  This module performs that inversion and joins
the current lidar occupancy evidence without depending on ROS or mission
state.

The builder deliberately has no ``command_model`` input mode.  With an odom
pose it keeps the measured bay in the odom frame; without odom it uses the
camera bay observation itself as the fixed frame (``measured_bay``).  AUTO
requires an explicit target identity from the observation stream.  It never
chooses an arbitrary nearest empty bay.
"""
from __future__ import division

import math

from robot.common.geometry import local
from robot.common.geometry import world
from robot.common.geometry import wrap


try:
    _string_types = (basestring,)
except NameError:
    _string_types = (str,)


_PARALLEL_IDS = ('P1', 'P2', 'P3')
_FORWARD_IDS = ('P1', 'P2', 'P3', 'P4', 'P5')
_BAY_KINDS = ('parallel', 'perpendicular')
_POSE_SOURCES = ('vision', 'odom', 'fused', 'lidar')


def _finite(value, label):
    try:
        value = float(value)
    except (TypeError, ValueError):
        raise ValueError('invalid %s' % label)
    if math.isnan(value) or math.isinf(value):
        raise ValueError('invalid %s' % label)
    return value


def _pose(value, label):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError('invalid %s' % label)
    result = tuple(_finite(item, label) for item in value)
    if max(abs(item) for item in result[:2]) > 20.0:
        raise ValueError('%s outside local area' % label)
    return result[0], result[1], wrap(result[2])


def _slot_pose(row):
    if not isinstance(row, dict):
        raise ValueError('invalid parking slot')
    if isinstance(row.get('pose'), (list, tuple)):
        return _pose(row['pose'], 'parking slot pose')
    return (_finite(row.get('x'), 'parking slot x'),
            _finite(row.get('y'), 'parking slot y'),
            wrap(_finite(row.get('yaw'), 'parking slot yaw')))


def _parallel_heading(yaw):
    """Choose the representative of a line heading parallel to the car."""
    yaw = wrap(yaw)
    alternatives = (yaw, wrap(yaw + math.pi))
    return min(alternatives, key=lambda value: abs(value))


def invert_relative_pose(relative):
    """Return the vehicle pose in a frame whose origin is the observed bay.

    ``relative`` is the bay pose in the vehicle frame.  Keeping this helper
    public makes the frame convention directly testable and avoids hiding a
    sign error in the camera callback.
    """
    x, y, yaw = _pose(relative, 'relative parking slot pose')
    c, s = math.cos(yaw), math.sin(yaw)
    return (-c * x - s * y, s * x - c * y, wrap(-yaw))


def _normalise_id(value):
    if not isinstance(value, _string_types):
        return None
    value = value.strip().upper()
    return value or None


def _requested_target(cfg, data, target_id):
    for value in (target_id,
                  data.get('target_id') if isinstance(data, dict) else None,
                  data.get('selected_slot_id') if isinstance(data, dict) else None,
                  data.get('slot_id') if isinstance(data, dict) else None,
                  cfg.get('parking_slot')):
        value = _normalise_id(value)
        if value is None:
            continue
        if value in ('AUTO', 'ANY', 'NONE', 'NULL'):
            return None
        return value
    return None


def _profile(cfg, slot_id, candidate):
    profiles = cfg.get('slots', {}) if isinstance(cfg, dict) else {}
    row = profiles.get(slot_id, {}) if isinstance(profiles, dict) else {}
    kind = candidate.get('kind')
    if kind not in _BAY_KINDS:
        kind = row.get('kind', 'parallel')
    if kind not in _BAY_KINDS:
        return None
    if not row and isinstance(profiles, dict):
        # AUTO frames can omit a semantic id while still labelling the bay
        # kind.  P1--P3 share one nominal and P4--P5 share the other.
        row = next((value for value in profiles.values()
                    if isinstance(value, dict) and value.get('kind') == kind),
                   {})
    if slot_id is not None and row.get('kind') in _BAY_KINDS and kind != row['kind']:
        return None
    default_length, default_width = ((.45, .38) if kind == 'perpendicular'
                                     else (.70, .36))
    try:
        length = _finite(candidate.get('length', row.get('length',
                                                           default_length)),
                         'parking slot length')
        width = _finite(candidate.get('width', row.get('width',
                                                        default_width)),
                        'parking slot width')
    except ValueError:
        return None
    if not 0 < length <= 5 or not 0 < width <= 5:
        return None
    nominal_length, nominal_width = default_length, default_width
    if row:
        try:
            tolerance = float(cfg.get('parallel_parking_slot_size_tolerance_m',
                                      .05))
            nominal_length = _finite(row.get('length', length),
                                     'nominal parking slot length')
            nominal_width = _finite(row.get('width', width),
                                    'nominal parking slot width')
        except (TypeError, ValueError):
            return None
        if (abs(length - nominal_length) > tolerance or
                abs(width - nominal_width) > tolerance):
            return None
    # Detector dimensions are evidence for validation only.  The planner
    # receives the configured nominal bay dimensions so a within-tolerance
    # vision bias cannot enlarge the legal parking target.
    return dict(length=nominal_length, width=nominal_width, kind=kind)


def _candidate(row, index, cfg, target_hint=None):
    if not isinstance(row, dict):
        return None
    if row.get('kind') not in ((None,) + _BAY_KINDS):
        return None
    try:
        pose = _slot_pose(row)
    except ValueError:
        return None
    raw_id = None
    for key in ('id', 'slot_id', 'target_id', 'designated_id'):
        raw_id = _normalise_id(row.get(key))
        if raw_id is not None:
            break
    nominal_id = raw_id if raw_id is not None else (
        _normalise_id(target_hint) or _normalise_id(cfg.get('parking_slot')))
    nominal = cfg.get('slots', {}).get(nominal_id, {})
    kind = row.get('kind') if row.get('kind') in _BAY_KINDS else nominal.get('kind')
    heading = _parallel_heading(pose[2])
    if cfg.get('parking_mode') == 'forward_plan' and kind == 'perpendicular':
        # The two headings describe the same bay axis.  Forward entry needs
        # the representative whose axis points from the vehicle to the bay;
        # equivalently, the vehicle is on the bay-frame negative x side.
        alternatives = (wrap(pose[2]), wrap(pose[2] + math.pi))
        heading = max(alternatives,
                      key=lambda value: pose[0] * math.cos(value) +
                                        pose[1] * math.sin(value))
    return dict(index=index, pose=(pose[0], pose[1], heading),
                id=raw_id, raw=row,
                profile=_profile(cfg, nominal_id, row))


def _ordered_candidates(data, cfg, target_hint=None):
    rows = data.get('slots', []) if isinstance(data, dict) else data
    if not isinstance(rows, list):
        return []
    candidates = [item for index, row in enumerate(rows)
                  for item in [_candidate(row, index, cfg, target_hint)]
                  if item is not None]
    # x is the longitudinal bay order in the camera frame.  The configured
    # target-family rank is applied only when the detector did not provide IDs.
    return sorted(candidates, key=lambda item: (item['pose'][0], item['pose'][1]))


def _target_ids(cfg):
    return _FORWARD_IDS if cfg.get('parking_mode') == 'forward_plan' else _PARALLEL_IDS


def _candidate_kind(item, cfg, slot_id=None):
    raw_kind = item.get('raw', {}).get('kind')
    if raw_kind in _BAY_KINDS:
        return raw_kind
    profile = cfg.get('slots', {}).get(slot_id, {}) if slot_id else {}
    return profile.get('kind') if profile.get('kind') in _BAY_KINDS else None


def _select_target(candidates, requested, cfg):
    target_ids = _target_ids(cfg)
    if requested is not None:
        exact = [item for item in candidates if item['id'] == requested]
        if exact:
            return exact[0] if len(exact) == 1 else None
        if any(item['id'] is not None for item in candidates):
            # Once the detector supplies semantic identities, a mismatching
            # frame cannot be re-ranked into the requested physical bay.
            return None
        # Existing detector frames contain geometry but no semantic ID.  A
        # concrete target may use its configured rank, provided the complete
        # same-kind target family is visible.  This is deterministic and does
        # not silently fall back to the nearest bay.
        profile = cfg.get('slots', {}).get(requested, {})
        expected_kind = profile.get('kind')
        if expected_kind not in _BAY_KINDS:
            return None
        family = [item for item in candidates
                  if _candidate_kind(item, cfg, requested) == expected_kind]
        rank = profile.get('rank')
        if rank is None:
            rank = target_ids.index(requested) if requested in target_ids else None
        try:
            rank = int(rank)
        except (TypeError, ValueError):
            rank = None
        try:
            minimum = int(profile.get('min_candidates', 3))
        except (TypeError, ValueError):
            minimum = 3
        if cfg.get('parking_mode') == 'forward_plan' and expected_kind == 'perpendicular':
            minimum = max(minimum, 2)
        if rank is not None and rank < 0:
            rank += len(family)
        if (rank is not None and rank >= 0 and minimum > 0 and
                len(family) >= minimum and len(family) > rank):
            selected = family[rank]
            selected = dict(selected, id=requested)
            return selected
        return None

    # AUTO is safe only when the observation itself names one target.  A
    # single unlabelled visible bay is still ambiguous: it may be a partial
    # view of one of the designated bays.
    labelled = [item for item in candidates
                if item['id'] in target_ids]
    return labelled[0] if len(labelled) == 1 else None


def _scan_values(scan):
    if scan is None:
        return [], None, None, 0
    if isinstance(scan, dict):
        points = scan.get('obstacles', scan.get('points', []))
        pose = scan.get('pose')
        stamp = scan.get('stamp')
        valid = scan.get('valid_rays', scan.get('valid', 0))
    else:
        points = getattr(scan, 'obstacles', [])
        pose = getattr(scan, 'pose', None)
        stamp = getattr(scan, 'stamp', None)
        valid = getattr(scan, 'valid_rays', 0)
    if not isinstance(points, (list, tuple)):
        points = []
    clean = []
    for point in points:
        try:
            if isinstance(point, dict):
                point = (point['x'], point['y'])
            if not isinstance(point, (list, tuple)) or len(point) < 2:
                continue
            clean.append((_finite(point[0], 'lidar x'),
                          _finite(point[1], 'lidar y')))
        except (KeyError, TypeError, ValueError):
            continue
    try:
        valid = int(valid)
    except (TypeError, ValueError):
        valid = 0
    if pose is not None:
        try:
            pose = _pose(pose, 'lidar pose')
        except ValueError:
            pose = None
    try:
        stamp = _finite(stamp, 'lidar stamp') if stamp is not None else None
    except ValueError:
        stamp = None
    return clean, pose, stamp, max(0, valid)


def _lidar_extrinsic(cfg):
    lidar_cfg = cfg.get('lidar') if isinstance(cfg, dict) else None
    if not isinstance(lidar_cfg, dict):
        return None
    try:
        # Do not silently turn a missing or malformed calibration into an
        # identity transform.  The real Scan always carries this pose.
        return (_finite(lidar_cfg['x'], 'lidar x'),
                _finite(lidar_cfg['y'], 'lidar y'),
                wrap(_finite(lidar_cfg['yaw'], 'lidar yaw')))
    except (KeyError, TypeError, ValueError):
        return None


def _scan_transform_valid(scan, cfg):
    if scan is None:
        return False
    unused_points, scan_pose, unused_stamp, unused_valid = _scan_values(scan)
    del unused_points, unused_stamp, unused_valid
    return scan_pose is not None and _lidar_extrinsic(cfg) is not None


def _vehicle_lidar_points(scan, current_pose, scene_pose, source, cfg):
    points, scan_pose, unused_stamp, unused_valid = _scan_values(scan)
    extrinsic = _lidar_extrinsic(cfg)
    if not points or scan_pose is None or extrinsic is None:
        return []
    if source in ('odom', 'fused', 'lidar'):
        # Scan.obstacles are already in its construction-time world frame.
        # Plain dictionaries used by replay/tests may explicitly identify
        # base_link points instead.
        frame = scan.get('frame') if isinstance(scan, dict) else None
        if frame in ('base_link', 'vehicle'):
            return [world(current_pose, point) for point in points]
        return points

    # Vision frame: remove the sensor/world transform first, then apply the
    # measured vehicle pose in the bay frame.  A scan pose is preferred; a
    # plain list is already interpreted as vehicle-relative points.
    # Scan.obstacles are already in the shared world frame.  Align them to
    # the camera timestamp before expressing them in the fixed measured-bay
    # frame; scan and camera callbacks can legitimately straddle one motion
    # step.  Missing camera-time pose evidence fails closed.
    if current_pose is None or scene_pose is None:
        return []
    relative = [local(current_pose, point) for point in points]
    return [world(scene_pose, point) for point in relative]


def _slot_samples(slot_pose, length, width):
    """Small fixed grid used only for the occupancy coverage gate."""
    return [world(slot_pose, (x, y))
            for x in (-length / 2.0, 0.0, length / 2.0)
            for y in (-width / 2.0, 0.0, width / 2.0)]


def _camera_scan_query_points(points, scene_pose, camera_pose):
    """Map measured-bay samples into the shared scan world frame.

    ``camera_pose`` is the pose at the ground image timestamp.  Scan.classify
    then applies its own scan-time sensor pose and extrinsic calibration when
    judging each world query point.
    """
    if scene_pose is None or camera_pose is None:
        return None
    return [world(camera_pose, local(scene_pose, point)) for point in points]


def _slot_coverage(slot_pose, length, width, scan, scene_pose, source, cfg,
                   camera_pose=None):
    """Return measured FREE fraction, or None when no coverage API exists."""
    if not _scan_transform_valid(scan, cfg):
        return None
    samples = _slot_samples(slot_pose, length, width)
    if source in ('odom', 'fused', 'lidar'):
        query = samples
    else:
        query = _camera_scan_query_points(samples, scene_pose, camera_pose)
        if query is None:
            return None
    if isinstance(scan, dict):
        value = scan.get('coverage')
        try:
            value = float(value)
            return value if 0.0 <= value <= 1.0 else None
        except (TypeError, ValueError):
            return None
    coverage = getattr(scan, 'coverage', None)
    if callable(coverage):
        try:
            value = float(coverage(query))
            return value if 0.0 <= value <= 1.0 else None
        except (TypeError, ValueError):
            return None
    classify = getattr(scan, 'classify', None)
    if callable(classify):
        try:
            values = [classify(point) == 'FREE' for point in query]
            return sum(values) / float(len(values))
        except (TypeError, ValueError):
            return None
    return None


def _slot_status(slot_pose, length, width, points, scan, scene_pose,
                 source, cfg, stamp, camera_pose=None):
    margin = max(0.0, float(cfg.get('parallel_parking_occupancy_margin_m',
                                  cfg.get('obstacle_margin', .02))))
    occupied = any(abs(local(slot_pose, point)[0]) <= length / 2.0 + margin and
                   abs(local(slot_pose, point)[1]) <= width / 2.0 + margin
                   for point in points)
    scan_points, unused_pose, scan_stamp, valid = _scan_values(scan)
    del scan_points, unused_pose
    timeout = float(cfg.get('lidar_timeout', cfg.get('sensor_timeout', .5)))
    try:
        # A coverage callback alone is not evidence when the scan carried no
        # valid rays.  Production config supplies a larger threshold; keep a
        # nonzero floor for small replay configs too.
        minimum_rays = int(cfg.get('lidar', {}).get('min_rays', 1))
    except (AttributeError, TypeError, ValueError):
        minimum_rays = 0
    sync_ok = (scan_stamp is not None and
               abs(stamp - scan_stamp) <= timeout)
    camera_pose_ok = (source != 'vision' or camera_pose is not None)
    fresh = (_scan_transform_valid(scan, cfg) and sync_ok and camera_pose_ok and
             valid >= max(0, minimum_rays))
    if occupied and fresh:
        return 'OCCUPIED', dict(source='lidar', obstacle_points=True,
                                valid_rays=valid, fresh=fresh)
    if occupied:
        # A point from a scan outside the camera/scan synchronization window
        # cannot be placed at the image timestamp.  Preserve the safe stop
        # through UNKNOWN instead of treating stale geometry as current.
        return 'UNKNOWN', dict(source='lidar', obstacle_points=True,
                                valid_rays=valid, fresh=fresh)
    coverage = _slot_coverage(slot_pose, length, width, scan, scene_pose,
                              source, cfg, camera_pose=camera_pose)
    minimum_coverage = float(cfg.get('parallel_parking_min_coverage',
                                     cfg.get('slot_min_coverage', .80)))
    if (fresh and coverage is not None and
            coverage >= minimum_coverage):
        return 'FREE', dict(source='lidar', obstacle_points=False,
                            valid_rays=valid, fresh=fresh,
                            coverage=coverage)
    return 'UNKNOWN', dict(source='lidar', obstacle_points=False,
                            valid_rays=valid, fresh=fresh,
                            coverage=coverage)


def _region(scene_pose, slot, cfg):
    """Return the approach road and exact bay rectangles in scene coordinates."""
    road_half_width = float(cfg.get('parallel_parking_road_half_width',
                                    cfg.get('parking_road_half_width', .60)))
    road = 2.0 * road_half_width
    setup = float(cfg.get('parallel_parking_max_setup_m', 2.0))
    slot_pose = slot['pose']
    relative = local(slot_pose, (scene_pose[0], scene_pose[1]))
    if slot.get('kind') == 'perpendicular':
        # A forward entry approaches the bay through its negative-length end.
        # The configured 0.60 m half-width describes a 1.20 m full road
        # corridor along the bay axis; its lateral extent is the measured
        # initial vehicle position plus the bounded setup margin.  Do not
        # extend the road through the bay or use an unmeasured map opening.
        mouth = -slot['length'] / 2.0
        xlo, xhi = mouth - road, mouth
        ylo, yhi = relative[1] - setup, relative[1] + setup
    else:
        # The configured value is a half width around the lane centre.  The
        # side-parking corridor therefore covers the full 1.20 m outside the
        # bay mouth with the competition defaults.
        side = 1.0 if relative[1] >= 0.0 else -1.0
        side_a = slot['width'] / 2.0
        side_b = side_a + road
        ylo, yhi = sorted((side * side_a, side * side_b))
        xlo = min(-slot['length'] / 2.0, relative[0] - setup)
        xhi = max(slot['length'] / 2.0, relative[0] + setup)
    if xhi - xlo < .10 or yhi - ylo < .10:
        return []

    def to_scene(rows):
        return [world(slot_pose, point) for point in rows]

    road_polygon = to_scene([(xlo, ylo), (xhi, ylo),
                             (xhi, yhi), (xlo, yhi)])
    return [road_polygon, _slot_polygon(slot_pose, slot['length'],
                                         slot['width'])]


def _slot_polygon(slot_pose, length, width):
    """Return one exact nominal bay rectangle in scene coordinates."""
    half_length, half_width = length / 2.0, width / 2.0
    return [world(slot_pose, point)
            for point in [(-half_length, -half_width),
                          (half_length, -half_width),
                          (half_length, half_width),
                          (-half_length, half_width)]]


def _previous_ready(previous):
    return isinstance(previous, dict) and previous.get('ready') is True


def _previous_locked(previous, cfg):
    """Return true after target geometry has been confirmed independently of occupancy."""
    if not isinstance(previous, dict):
        return False
    target_id = _normalise_id(previous.get('target_id'))
    if target_id is None and isinstance(previous.get('slot'), dict):
        target_id = _normalise_id(previous['slot'].get('id'))
    try:
        confirmations = int(previous.get('confirmations', 0))
        required = int(cfg.get('parallel_parking_confirm_frames', 3))
    except (TypeError, ValueError):
        return False
    association_pose = _previous_association_pose(previous)
    return (target_id is not None and confirmations >= max(1, required) and
            association_pose is not None)


def _previous_relative_slot(previous):
    """Recover the camera-relative bay pose after a public scene round trip."""
    if not isinstance(previous, dict):
        return None
    private = previous.get('_relative_slot')
    if isinstance(private, (list, tuple)) and len(private) == 3:
        return private
    slot = previous.get('slot')
    pose = previous.get('pose')
    if not isinstance(slot, dict):
        return None
    try:
        slot_pose = _pose(slot.get('pose'), 'previous slot pose')
        vehicle_pose = _pose(pose, 'previous vehicle pose')
        if previous.get('frame') == 'measured_bay':
            return invert_relative_pose(vehicle_pose)
        return tuple(local(vehicle_pose, slot_pose))
    except ValueError:
        return None


def _same_target(previous, target_id):
    if not isinstance(previous, dict):
        return True
    old = _normalise_id(previous.get('target_id'))
    if old is None and isinstance(previous.get('slot'), dict):
        old = _normalise_id(previous['slot'].get('id'))
    return old is None or old == target_id


def _observation_source(data):
    """Return the camera source that produced a slot frame."""
    if not isinstance(data, dict):
        return None
    source = _normalise_id(data.get('source', 'front'))
    return source.lower() if source is not None else None


def _previous_association_pose(previous):
    """Read the time-aligned vehicle pose used only for cross-camera matching."""
    if not isinstance(previous, dict):
        return None
    for key in ('_association_pose', 'association_pose'):
        value = previous.get(key)
        if value is None:
            continue
        try:
            return _pose(value, 'previous association pose')
        except ValueError:
            return None
    return None


def _previous_target_world(previous):
    """Recover the locked bay pose in the shared pose frame, when possible."""
    if not isinstance(previous, dict):
        return None
    slot = previous.get('slot')
    if not isinstance(slot, dict):
        return None
    try:
        if previous.get('frame') == 'measured_bay':
            association_pose = _previous_association_pose(previous)
            relative_slot = _previous_relative_slot(previous)
            if association_pose is None or relative_slot is None:
                return None
            relative_slot = _pose(relative_slot, 'previous relative slot pose')
            return tuple(world(association_pose, relative_slot)) + (
                wrap(association_pose[2] + relative_slot[2]),)
        return _pose(slot.get('pose'), 'previous slot pose')
    except ValueError:
        return None


def _associate_locked_target(candidates, previous, requested, current_pose,
                             cfg):
    """Associate a current partial frame with one already locked bay.

    ID-less rear frames cannot rank the visible bays again: after the vehicle
    moves, that rank is camera-dependent and can select a neighbouring bay.
    Use the shared pose frame to require exactly one nearby target with a
    bounded heading.  An explicit identity remains subject to the locked
    identity gate but does not need a second semantic detector result.
    """
    if not _previous_locked(previous, cfg):
        return None
    previous_id = _normalise_id(previous.get('target_id'))
    if previous_id is None and isinstance(previous.get('slot'), dict):
        previous_id = _normalise_id(previous['slot'].get('id'))
    if previous_id is None or (requested is not None and requested != previous_id):
        return None

    named = [item for item in candidates if item['id'] is not None]
    if named:
        exact = [item for item in named if item['id'] == previous_id]
        if len(exact) != 1:
            return None
        return exact[0]

    # A partial unlabelled frame must be related to the previous locked
    # geometry in a shared pose frame.  Missing pose evidence fails closed.
    if current_pose is None:
        return None
    previous_pose = _previous_association_pose(previous)
    target_world = _previous_target_world(previous)
    if previous_pose is None or target_world is None:
        return None
    max_pose_step = float(cfg.get('parallel_parking_slot_drift_m', .12))
    max_heading_step = float(cfg.get('parking_yaw_match_tolerance',
                                      math.radians(15.0)))
    if (math.hypot(current_pose[0] - previous_pose[0],
                  current_pose[1] - previous_pose[1]) > max_pose_step or
            abs(wrap(current_pose[2] - previous_pose[2])) > max_heading_step):
        return None

    max_distance = float(cfg.get('parallel_parking_slot_drift_m', .12))
    max_yaw = float(cfg.get('parking_yaw_match_tolerance',
                            math.radians(15.0)))
    matches = []
    for item in candidates:
        candidate_world = tuple(world(current_pose, item['pose'])) + (
            wrap(current_pose[2] + item['pose'][2]),)
        if (math.hypot(candidate_world[0] - target_world[0],
                       candidate_world[1] - target_world[1]) <= max_distance and
                abs(wrap(candidate_world[2] - target_world[2])) <= max_yaw):
            matches.append(item)
    if len(matches) != 1:
        # Zero matches means the rear cannot see the locked bay; multiple
        # matches are ambiguous and must never switch to a neighbour.
        return None
    return dict(matches[0], id=previous_id)


def _line_measure(row):
    """Normalize one finite parking-line segment from the front camera."""
    if isinstance(row, dict):
        row = row.get('points', row.get('line'))
    if not isinstance(row, (list, tuple)) or len(row) != 2:
        return None
    try:
        first = (_finite(row[0][0], 'parking line x'),
                 _finite(row[0][1], 'parking line y'))
        second = (_finite(row[1][0], 'parking line x'),
                  _finite(row[1][1], 'parking line y'))
    except (IndexError, KeyError, TypeError, ValueError):
        return None
    dx, dy = second[0] - first[0], second[1] - first[1]
    length = math.hypot(dx, dy)
    if length < .05:
        return None
    return dict(first=first, second=second,
                midpoint=((first[0] + second[0]) / 2.0,
                          (first[1] + second[1]) / 2.0),
                unit=(dx / length, dy / length), length=length)


def _partial_slot_from_lines(previous, lines, cfg):
    """Infer a locked bay from two side edges and its far cross edge.

    This intentionally supports one small, bounded partial case.  The mouth
    edge is never invented: the far edge fixes the known end and the nominal
    length places the bay centre behind it.  The result has no semantic id so
    ``_associate_locked_target`` must match it to the already locked target in
    the shared pose frame.
    """
    if not _previous_locked(previous, cfg):
        return None
    slot = previous.get('slot') if isinstance(previous, dict) else None
    if not isinstance(slot, dict) or slot.get('kind') not in _BAY_KINDS:
        return None
    try:
        length = _finite(slot['length'], 'parking slot length')
        width = _finite(slot['width'], 'parking slot width')
    except (KeyError, TypeError, ValueError):
        return None
    measured = [item for item in (_line_measure(row) for row in (lines or []))
                if item is not None]
    if len(measured) < 3:
        return None
    side_min_length = max(.12, length * .30)
    side_angle = math.cos(math.radians(20.0))
    cross_angle = math.sin(math.radians(25.0))
    separation_tolerance = max(.08, width * .35)
    cross_min, cross_max = max(.12, width * .35), width * 1.8
    best = None
    for first_index, first in enumerate(measured):
        if first['length'] < side_min_length:
            continue
        for second_index in range(first_index + 1, len(measured)):
            second = measured[second_index]
            if (second['length'] < side_min_length or
                    abs(first['unit'][0] * second['unit'][0] +
                        first['unit'][1] * second['unit'][1]) < side_angle):
                continue
            ux, uy = first['unit']
            normal = (-uy, ux)
            delta = (second['midpoint'][0] - first['midpoint'][0],
                     second['midpoint'][1] - first['midpoint'][1])
            separation = abs(delta[0] * normal[0] + delta[1] * normal[1])
            if abs(separation - width) > separation_tolerance:
                continue
            first_projection = [point[0] * ux + point[1] * uy
                                for point in (first['first'], first['second'])]
            second_projection = [point[0] * ux + point[1] * uy
                                 for point in (second['first'], second['second'])]
            overlap_start = max(min(first_projection), min(second_projection))
            overlap_end = min(max(first_projection), max(second_projection))
            overlap = overlap_end - overlap_start
            if overlap < side_min_length:
                continue
            side_min = max(min(first_projection), min(second_projection))
            side_max = min(max(first_projection), max(second_projection))
            for cross_index, cross in enumerate(measured):
                if cross_index in (first_index, second_index):
                    continue
                if not cross_min <= cross['length'] <= cross_max:
                    continue
                cross_dot = abs(ux * cross['unit'][0] +
                                uy * cross['unit'][1])
                if cross_dot > cross_angle:
                    continue
                cross_projection = (cross['midpoint'][0] * ux +
                                    cross['midpoint'][1] * uy)
                endpoint = None
                for candidate_end in (side_max, side_min):
                    if abs(cross_projection - candidate_end) <= max(
                            .10, length * .30):
                        endpoint = candidate_end
                        break
                if endpoint is None:
                    continue
                axis = (ux, uy) if endpoint == side_max else (-ux, -uy)
                axis_normal = (-axis[1], axis[0])
                lateral = .5 * (
                    first['midpoint'][0] * axis_normal[0] +
                    first['midpoint'][1] * axis_normal[1] +
                    second['midpoint'][0] * axis_normal[0] +
                    second['midpoint'][1] * axis_normal[1])
                far_axis = (cross['midpoint'][0] * axis[0] +
                            cross['midpoint'][1] * axis[1])
                far_point = (axis[0] * far_axis + axis_normal[0] * lateral,
                             axis[1] * far_axis + axis_normal[1] * lateral)
                centre = (far_point[0] - axis[0] * length / 2.0,
                          far_point[1] - axis[1] * length / 2.0)
                # Forward-plan partial recovery cannot resurrect a bay behind
                # the vehicle.  This also rejects the opposite line-axis sign.
                if centre[0] * axis[0] + centre[1] * axis[1] <= .02:
                    continue
                score = (abs(separation - width) / max(width, .01) +
                         abs(overlap - length) / max(length, .01) +
                         abs(cross['length'] - width) / max(width, .01) +
                         abs(cross_projection - endpoint) /
                         max(length, .01))
                candidate = (score, dict(x=centre[0], y=centre[1],
                                        yaw=math.atan2(axis[1], axis[0]),
                                        kind=slot['kind'], length=length,
                                        width=width,
                                        geometry_status='supported_partial'))
                if best is None or candidate[0] < best[0]:
                    best = candidate
    return best[1] if best is not None else None


def _scene_slot_pose(relative_pose, current_pose, vehicle_pose, source):
    """Convert a camera candidate into the scene frame used by the builder."""
    base = current_pose if source in ('odom', 'fused', 'lidar') else vehicle_pose
    if base is None:
        return None
    return tuple(world(base, relative_pose)) + (
        wrap(base[2] + relative_pose[2]),)


def _neighbour_candidates(candidates, selected, target_profile, source,
                         current_pose, vehicle_pose, obstacles, scan, cfg,
                         stamp):
    """Cache same-frame exact geometry; occupancy alone admits driveable bays."""
    entries, evidence = [], []
    selected_index = selected.get('index')
    for candidate in candidates:
        if candidate.get('index') == selected_index:
            continue
        raw = candidate.get('raw', {})
        if raw.get('geometry_status', 'complete') != 'complete':
            continue
        profile = candidate.get('profile')
        if (not isinstance(profile, dict) or
                profile.get('kind') != target_profile.get('kind')):
            continue
        candidate_pose = _scene_slot_pose(
            candidate['pose'], current_pose, vehicle_pose, source)
        if candidate_pose is None:
            continue
        status, detail = _slot_status(
            candidate_pose, profile['length'], profile['width'], obstacles,
            scan, vehicle_pose, source, cfg, stamp, camera_pose=current_pose)
        item = dict(id=candidate.get('id'), index=candidate.get('index'),
                    pose=candidate_pose, length=profile['length'],
                    width=profile['width'], kind=profile['kind'])
        summary = dict(id=item['id'], kind=item['kind'], pose=item['pose'],
                       length=item['length'], width=item['width'],
                       occupancy=status, included=status == 'FREE',
                       fresh=bool(detail.get('fresh', False)),
                       valid_rays=detail.get('valid_rays', 0))
        if detail.get('coverage') is not None:
            summary['coverage'] = detail['coverage']
        evidence.append(summary)
        # Geometry and current free-space evidence have separate lifetimes.
        # Keep a completely measured bay even when the first scan is unknown;
        # a later scan may clear it, but caching never grants driveable space.
        entries.append(item)
    return entries, evidence


def _cached_neighbour_regions(neighbours, source, current_pose, vehicle_pose,
                              obstacles, scan, cfg, stamp):
    """Recheck fixed neighbour poses; UNKNOWN/OCCUPIED stays outside regions."""
    regions, evidence = [], []
    for item in neighbours:
        try:
            pose = _pose(item['pose'], 'neighbour slot pose')
            length = _finite(item['length'], 'neighbour slot length')
            width = _finite(item['width'], 'neighbour slot width')
            kind = item['kind']
        except (KeyError, TypeError, ValueError):
            continue
        if kind not in _BAY_KINDS:
            continue
        status, detail = _slot_status(
            pose, length, width, obstacles, scan, vehicle_pose, source, cfg,
            stamp, camera_pose=current_pose)
        summary = dict(id=item.get('id'), kind=kind, pose=pose,
                       length=length, width=width, occupancy=status,
                       included=status == 'FREE',
                       fresh=bool(detail.get('fresh', False)),
                       valid_rays=detail.get('valid_rays', 0))
        if detail.get('coverage') is not None:
            summary['coverage'] = detail['coverage']
        evidence.append(summary)
        if status == 'FREE':
            regions.append(_slot_polygon(pose, length, width))
    return regions, evidence


def build_parallel_scene(cfg, previous, pose, data, stamp, scan=None,
                         target_id=None, pose_source=None):
    """Build one scene dictionary or return ``None`` for unusable geometry.

    ``previous`` is the last builder output.  It is used only for target
    identity, repeated-frame confirmation, and the fixed odom bay anchor; it
    is never treated as a command-model pose.
    """
    if not isinstance(cfg, dict) or not isinstance(data, dict):
        return None
    try:
        stamp = _finite(stamp, 'parking scene stamp')
    except ValueError:
        return None
    if isinstance(previous, dict):
        try:
            if stamp <= _finite(previous.get('stamp'), 'previous scene stamp'):
                return None
        except ValueError:
            pass
    observation_source = _observation_source(data)
    if observation_source not in ('front', 'rear'):
        return None
    forward_plan = cfg.get('parking_mode') == 'forward_plan'
    if forward_plan and observation_source != 'front':
        return None
    if (observation_source == 'rear' and
            (data.get('frame', 'base_link') != 'base_link' or
             not _previous_locked(previous, cfg))):
        # Front is the only source allowed to establish the initial target
        # and its complete-candidate confirmation latch.
        return None
    try:
        current_pose = _pose(pose, 'vehicle pose') if pose is not None else None
    except ValueError:
        current_pose = None
    requested = _requested_target(cfg, data, target_id)
    candidates = _ordered_candidates(data, cfg, requested)
    if _previous_locked(previous, cfg):
        selected = _associate_locked_target(candidates, previous, requested,
                                             current_pose, cfg)
    else:
        selected = _select_target(candidates, requested, cfg)
    if selected is None:
        return None
    selected_id = selected['id'] or requested
    if selected_id is None:
        return None
    if not _same_target(previous, selected_id):
        return None
    # Re-resolve after rank association so an ID-less detector row uses the
    # configured dimensions and kind of its designated target.
    profile = _profile(cfg, selected_id, selected['raw'])
    if profile is None:
        return None

    source = _normalise_id(pose_source)
    configured_source = _normalise_id(
        cfg.get('parallel_parking_pose_source'))
    mode = _normalise_id(cfg.get('pose_mode'))
    if source == 'COMMAND_MODEL':
        # A command-model trajectory is not a measurement source.  The
        # caller may still omit pose_source and use the independent camera
        # relative slot pose, which is handled by the vision branch below.
        return None
    if source is None:
        source = configured_source.lower() if (
            configured_source is not None and
            configured_source.lower() in _POSE_SOURCES) else (
            'odom' if mode == 'ODOM' else 'vision')
    else:
        source = source.lower()
    # A caller cannot relabel a command-model pose as odom/fused/lidar.  Odom
    # is trusted only when the configured pose stream is odom; the optional
    # fused/lidar modes likewise require an explicit matching producer mode.
    if source != 'vision':
        if mode == 'COMMAND_MODEL':
            return None
        if source == 'odom' and mode != 'ODOM':
            return None
        if source in ('fused', 'lidar') and configured_source != source.upper():
            return None
    if source not in _POSE_SOURCES:
        return None
    relative_slot = selected['pose']
    if isinstance(previous, dict) and not _previous_locked(previous, cfg):
        old_relative = _previous_relative_slot(previous)
        if isinstance(old_relative, (list, tuple)) and len(old_relative) == 3:
            try:
                old_relative = _pose(old_relative, 'previous relative slot pose')
                max_jump = float(cfg.get(
                    'parallel_parking_relative_jump_m', .35))
                if math.hypot(relative_slot[0] - old_relative[0],
                             relative_slot[1] - old_relative[1]) > max_jump:
                    return None
                if abs(wrap(relative_slot[2] - old_relative[2])) > math.radians(30):
                    return None
            except (TypeError, ValueError):
                return None
    if source in ('odom', 'fused', 'lidar'):
        if current_pose is None:
            return None
        measured_slot = tuple(world(current_pose, relative_slot)) + (
            wrap(current_pose[2] + relative_slot[2]),)
        old_slot = previous.get('slot', {}).get('pose') if isinstance(previous, dict) else None
        if _previous_ready(previous) and isinstance(old_slot, (list, tuple)):
            try:
                old_slot = _pose(old_slot, 'previous slot pose')
                drift = math.hypot(measured_slot[0] - old_slot[0],
                                   measured_slot[1] - old_slot[1])
                max_drift = float(cfg.get('parallel_parking_slot_drift_m', .12))
                if drift > max_drift:
                    return None
                slot_pose = old_slot
            except (TypeError, ValueError):
                slot_pose = measured_slot
        else:
            slot_pose = measured_slot
        vehicle_pose = current_pose
        frame = 'odom'
    else:
        # The relative slot itself supplies the measured visual pose.  The
        # slot is the origin of the fixed bay frame; vehicle odometry is not
        # consulted for this path.
        vehicle_pose = invert_relative_pose(relative_slot)
        old_slot = previous.get('slot', {}).get('pose') if isinstance(previous, dict) else None
        slot_pose = (0.0, 0.0, 0.0)
        if isinstance(old_slot, (list, tuple)) and len(old_slot) == 3:
            try:
                old_slot = _pose(old_slot, 'previous slot pose')
                if math.hypot(old_slot[0], old_slot[1]) > .05 or abs(old_slot[2]) > .05:
                    return None
            except ValueError:
                return None
        frame = 'measured_bay'

    raw_status = selected['raw'].get('geometry_status', 'complete')
    if raw_status not in ('complete', 'supported_partial'):
        return None
    slot = dict(id=selected_id, pose=slot_pose,
                length=profile['length'], width=profile['width'],
                kind=profile['kind'])
    obstacles = _vehicle_lidar_points(scan, current_pose, vehicle_pose,
                                      source, cfg)
    occupancy, occupancy_detail = _slot_status(
        slot_pose, slot['length'], slot['width'], obstacles, scan,
        vehicle_pose, source, cfg, stamp, camera_pose=current_pose)
    previous_count = 0
    if isinstance(previous, dict):
        previous_count = previous.get('_confirmations', previous.get('confirmations', 0))
        try:
            previous_count = int(previous_count)
        except (TypeError, ValueError):
            previous_count = 0
    if isinstance(previous, dict) and _normalise_id(previous.get('target_id')) == selected_id:
        confirmations = previous_count + 1
    else:
        confirmations = 1
    required = int(cfg.get('parallel_parking_confirm_frames', 3))
    ready = confirmations >= max(1, required) and occupancy == 'FREE'
    geometry_locked = (current_pose is not None and
                       confirmations >= max(1, required) and
                       selected_id is not None)
    regions = _region(vehicle_pose, slot, cfg)
    if (forward_plan and profile['kind'] == 'perpendicular' and
            isinstance(previous, dict) and _previous_locked(previous, cfg) and
            isinstance(previous.get('regions'), list) and previous['regions']):
        # Once the target is geometrically locked, retain the original
        # measured approach corridor while the vehicle moves.  Re-centering
        # it on every partial frame would enlarge the planner's legal area
        # from unmeasured space.
        base_regions = previous.get('_base_regions')
        if not isinstance(base_regions, list) or not base_regions:
            base_regions = previous['regions']
        regions = list(base_regions)
    else:
        base_regions = list(regions)
    neighbour_slots = []
    region_evidence = []
    if forward_plan:
        cached = (previous.get('_neighbour_slots')
                  if isinstance(previous, dict) else None)
        if isinstance(cached, list):
            neighbour_slots = cached
            neighbour_regions, region_evidence = _cached_neighbour_regions(
                neighbour_slots, source, current_pose, vehicle_pose,
                obstacles, scan, cfg, stamp)
            regions.extend(neighbour_regions)
        elif previous is None and raw_status == 'complete':
            # Only the first complete slot frame may establish the static
            # neighbour cache.  A partial target frame can never invent one.
            neighbour_slots, region_evidence = _neighbour_candidates(
                candidates, selected, profile, source, current_pose,
                vehicle_pose, obstacles, scan, cfg, stamp)
            regions.extend([_slot_polygon(item['pose'], item['length'],
                                           item['width'])
                            for item, detail in zip(neighbour_slots,
                                                    region_evidence)
                            if detail['included']])
    scene = dict(frame=frame, pose=vehicle_pose, stamp=stamp,
                 pose_source=source, regions=regions,
                 obstacles=obstacles, slot=slot, target_id=selected_id,
                 observation_source=observation_source,
                 geometry_status=raw_status,
                 region_evidence=region_evidence,
                 occupancy=occupancy, occupancy_detail=occupancy_detail,
                 confirmations=confirmations, ready=ready,
                 geometry_locked=geometry_locked,
                 _confirmations=confirmations, _relative_slot=relative_slot,
                 _base_regions=base_regions,
                 # This is association metadata only.  ``pose`` remains the
                 # independent camera-measured vehicle pose in measured_bay.
                 _association_pose=current_pose,
                 association_pose=current_pose,
                 _neighbour_slots=neighbour_slots)
    return scene


def public_scene(scene):
    """Drop builder bookkeeping before a scene is sent over ROS."""
    if not isinstance(scene, dict):
        return None
    return dict((key, value) for key, value in scene.items()
                if not key.startswith('_'))


class ParallelParkingScene(object):
    """Small stateful wrapper convenient for camera/replay callers."""
    def __init__(self, cfg):
        self.cfg = dict(cfg)
        self.scene = None

    def observe(self, pose, data, stamp, scan=None, target_id=None,
                pose_source=None):
        result = build_parallel_scene(self.cfg, self.scene, pose, data, stamp,
                                      scan=scan, target_id=target_id,
                                      pose_source=pose_source)
        if result is not None:
            self.scene = result
        elif (isinstance(self.scene, dict) and
              not _previous_locked(self.scene, self.cfg)):
            # Invalid identity/jump/duplicate input resets only the pending
            # confirmation latch; a confirmed scene remains available until a
            # newer valid observation replaces it.
            self.scene = None
        return result
