"""One persistent white divider before the farthest visible P board.

Camera candidates are measured in the body frame. Image-time pose samples
transport the same physical divider between frames; a nearby observation can
correct command-odometry drift, but a missing target is never replaced.
"""
from __future__ import division
import math
from collections import OrderedDict
import numpy as np
from robot.common.contracts import number, boolean
from robot.common.geometry import world, local, wrap


class LineCoordinates(object):
    """Map parking-only BEV geometry into the production body calibration.

Both homographies consume the same rectified image, so ground points can be
converted exactly. The empirically fitted parking H must not be decomposed
as a physical camera pose for elevated P-board ranging.
"""
    def __init__(self, parking_camera, body_camera):
        def metric(camera):
            ppm=camera['pixels_per_m']
            return np.array([[0.,-ppm,camera['origin_u']],
                             [-ppm,0.,camera['origin_v']],[0.,0.,1.]])
        a,b=metric(parking_camera),metric(body_camera)
        h=np.asarray(parking_camera['H']).reshape(3,3)
        k=np.asarray(body_camera['H']).reshape(3,3)
        self.matrix=np.dot(np.linalg.inv(b),np.dot(k,np.dot(np.linalg.inv(h),a)))
        self.inverse=np.linalg.inv(self.matrix)

    def apply(self,points,matrix):
        points=np.asarray(points,dtype=float)
        rows=np.dot(np.column_stack((points,np.ones(len(points)))),matrix.T)
        if not np.all(np.isfinite(rows)) or np.any(abs(rows[:,2])<1e-8):
            raise ValueError('parking/body ground projection singular')
        return (rows[:,:2]/rows[:,2,None]).tolist()

    def to_body(self,points):return self.apply(points,self.matrix)
    def to_parking(self,points):return self.apply(points,self.inverse)


class FramePoses(object):
    def __init__(self):
        self.rows = OrderedDict()

    def update(self, samples):
        if not isinstance(samples, (list, tuple)) or len(samples) > 40:
            raise ValueError('invalid parking pose samples')
        parsed = []
        for row in samples:
            if not isinstance(row, (list, tuple)) or len(row) != 2 or len(row[1]) != 3:
                raise ValueError('invalid parking pose sample')
            parsed.append((number(row[0]),tuple(number(v) for v in row[1])))
        for stamp,pose in parsed:
            self.rows[stamp] = pose
        self.rows = OrderedDict(sorted(self.rows.items())[-100:])

    def at(self, stamp):
        rows = list(self.rows.items())
        if not rows:
            return None
        nearest = min(rows, key=lambda row: abs(row[0]-stamp))
        if abs(nearest[0]-stamp) > .15:
            return None
        for (a, p), (b, q) in zip(rows, rows[1:]):
            if a <= stamp <= b and 0 < b-a <= .3:
                t = (stamp-a)/(b-a)
                return (p[0]+t*(q[0]-p[0]), p[1]+t*(q[1]-p[1]),
                        wrap(p[2]+t*wrap(q[2]-p[2])))
        return nearest[1]


def road_points(points, heading):
    c, s = math.cos(heading), math.sin(heading)
    return np.dot(np.asarray(points), np.array([[c, -s], [s, c]]))


def line_cost(a, b, distance_m, angle_deg):
    """Normal distance and longitudinal overlap tolerate a clipped fragment."""
    a, b = np.asarray(a), np.asarray(b)
    axis, other = a[1]-a[0], b[1]-b[0]
    length, other_length = np.linalg.norm(axis), np.linalg.norm(other)
    if min(length, other_length) < .001:
        return None
    axis, other = axis/length, other/other_length
    angle = math.acos(min(1., abs(float(np.dot(axis, other)))))
    if angle > math.radians(angle_deg):
        return None
    normal = np.array([-axis[1], axis[0]])
    distance = abs(float(np.dot(np.mean(b, axis=0)-np.mean(a, axis=0), normal)))
    aa, bb = np.dot(a, axis), np.dot(b, axis)
    overlap = min(max(aa), max(bb))-max(min(aa), min(bb))
    if distance > distance_m or overlap < .04:
        return None
    return distance+.1*angle


def refresh_line(previous, measured):
    # A newly clipped fragment updates position without discarding the full
    # previously observed extent; a later fragment at the other end still matches.
    previous, measured = np.asarray(previous), np.asarray(measured)
    old_axis,axis = previous[1]-previous[0],measured[1]-measured[0]
    old_axis,axis = old_axis/np.linalg.norm(old_axis),axis/np.linalg.norm(axis)
    if np.dot(old_axis,axis) < 0:
        axis = -axis
    normal = np.array([-axis[1],axis[0]])
    center = np.mean(previous,axis=0)
    center = center+normal*np.dot(np.mean(measured,axis=0)-center,normal)
    axis = .7*old_axis+.3*axis
    axis = axis/np.linalg.norm(axis)
    extents = np.dot(np.vstack((previous,measured))-center,axis)
    return [list(center+min(extents)*axis),list(center+max(extents)*axis)]


def p_candidates(data, threshold):
    rows = data.get('route_detections', data.get('detections'))
    if rows is None:
        rows = [dict(label=data.get('label'), confidence=data.get('confidence'),
                     bounds=data.get('sign_bounds'))]
    if not isinstance(rows, list):
        return []
    result = []
    for row in rows:
        if not isinstance(row, dict) or str(row.get('label', '')).upper() not in ('P', 'PARK', 'PARKING'):
            continue
        try:
            bounds = row.get('bounds')
            if (number(row.get('confidence')) >= threshold and
                    isinstance(bounds, (list, tuple)) and len(bounds) == 4):
                bounds = [number(v) for v in bounds]
                if bounds[2] > 0 and bounds[3] > 0:
                    result.append(bounds)
        except (ValueError, TypeError):
            continue
    return result


class TargetLine(object):
    def __init__(self, cfg):
        options = cfg['parking_timed']
        self.threshold = cfg['sign_confidence']
        self.frames = options['seen_frames']
        self.distance_m = options.get('target_match_m', .12)
        self.angle_deg = options.get('target_angle_deg', 15.)
        self.reset(None)

    def reset(self, run):
        self.run, self.last_stamp = run, None
        self.sign_world, self.sign_stamp, self.sign_bounds = None, None, None
        self.line_world, self.candidate = None, None
        self.votes, self.locked_at = 0, None

    def acquire_sign(self, data, source_pose, image_pose, projector, heading):
        if self.sign_world is not None or source_pose is None or image_pose is None:
            return
        choices = []
        for bounds in p_candidates(data, self.threshold):
            point = projector.position(bounds)
            if point is None:
                continue
            anchor = world(source_pose, point)
            forward = float(road_points([local(image_pose, anchor)], heading)[0, 0])
            if forward > 0:
                choices.append((forward, anchor, bounds))
        if choices:
            _, self.sign_world, self.sign_bounds = max(choices, key=lambda row: row[0])
            self.sign_stamp = data['stamp']

    def result(self, pose, valid, visible, reason):
        locked = self.line_world is not None
        return dict(run=self.run, id='before_p_white' if locked else None,
                    valid=valid, locked=locked, visible=visible, reason=reason,
                    votes=self.votes, locked_at=self.locked_at,
                    line=[list(local(pose, p)) for p in self.line_world]
                         if locked and pose is not None else None,
                    sign=list(local(pose, self.sign_world))
                         if self.sign_world is not None and pose is not None else None,
                    sign_stamp=self.sign_stamp, sign_bounds=self.sign_bounds)

    def observe(self, lines, stamp, pose, heading):
        if pose is None:
            # Missing pose alignment is UNKNOWN, never evidence of disappearance.
            if self.line_world is None:
                self.candidate, self.votes = None, 0
            return self.result(pose, False, None, 'WAIT_IMAGE_POSE')
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return self.result(pose, False, None, 'DUPLICATE_TARGET_FRAME')
        self.last_stamp = stamp
        if self.sign_world is None:
            return self.result(pose, True, None, 'WAIT_P_ANCHOR')
        incoming = [[world(pose, p) for p in line] for line in lines]
        if self.line_world is not None:
            choices = []
            for row in incoming:
                cost = line_cost(self.line_world, row, self.distance_m, self.angle_deg)
                if cost is not None:
                    choices.append((cost, row))
            if choices:
                self.line_world = refresh_line(self.line_world,min(choices,key=lambda row: row[0])[1])
                return self.result(pose, True, True, 'TARGET_VISIBLE')
            # Preserve the locked identity, even if other bay lines remain.
            return self.result(pose, True, False, 'TARGET_MISSING')
        sign_x = road_points([local(pose, self.sign_world)], heading)[0, 0]
        choices = []
        for line, measured in zip(lines, incoming):
            x = float(np.mean(road_points(line, heading), axis=0)[0])
            if 0 < x < sign_x-.02:
                choices.append((x, measured))
        if not choices:
            self.candidate, self.votes = None, 0
            return self.result(pose, True, None, 'WAIT_LINE_BEFORE_P')
        measured = max(choices, key=lambda row: row[0])[1]
        matched = (self.candidate is not None and
                   line_cost(self.candidate, measured, self.distance_m, self.angle_deg) is not None)
        self.votes = self.votes+1 if matched else 1
        self.candidate = measured
        if self.votes >= self.frames:
            self.line_world, self.locked_at = measured, stamp
            return self.result(pose, True, True, 'TARGET_VISIBLE')
        return self.result(pose, True, None, 'LOCKING_TARGET_LINE')


def validate_target(data):
    if not isinstance(data, dict):
        raise ValueError('invalid parking target')
    number(data.get('run'))
    valid, locked = boolean(data.get('valid')), boolean(data.get('locked'))
    visible = data.get('visible')
    if visible is not None:
        boolean(visible)
    if visible is not None and (not valid or not locked):
        raise ValueError('target visibility requires a valid locked target')
    if locked:
        if data.get('id') != 'before_p_white':
            raise ValueError('invalid parking target identity')
        number(data.get('locked_at'))
        if valid:
            line = data.get('line')
            if not isinstance(line, list) or len(line) != 2:
                raise ValueError('missing locked target geometry')
            for p in line:
                if not isinstance(p, (list, tuple)) or len(p) != 2:
                    raise ValueError('invalid target line point')
                for v in p:
                    if abs(number(v)) > 20:
                        raise ValueError('unbounded target line point')
    return data
