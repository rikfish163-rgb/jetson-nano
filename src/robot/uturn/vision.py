"""Ground-image relative pose for the adjacent-lane U-turn.

Pose is measured from image motion, never from issued speed/steering. The
corridor is a configured local map, NOT a claim that unseen ground was observed.
Loss of tracking invalidates this maneuver's frame; no silent reinitialization.
"""
from __future__ import division
import math
import cv2
import numpy as np
from robot.common.geometry import world
from robot.common.geometry import local
from robot.common.geometry import wrap
from robot.camera.landmarks import BlueLandmarks
from robot.camera.landmarks import image_segments


def corridor_regions(side, spacing, opening_start, opening_end, x_min, x_max, margin):
    values = (spacing, opening_start, opening_end, x_min, x_max, margin)
    if not all(np.isfinite(v) for v in values):
        raise ValueError('invalid corridor numbers')
    if (side not in ('LEFT', 'RIGHT') or not .1 <= spacing <= 2 or
            not -3 <= x_min < opening_start < opening_end < x_max <= 3 or
            not 0 < margin < spacing/4 or opening_end-opening_start <= 2*margin):
        raise ValueError('invalid corridor geometry')
    direction = -1 if side == 'LEFT' else 1
    def rect(x0, x1, y0, y1):
        return [(x0, direction*y0), (x1, direction*y0),
                (x1, direction*y1), (x0, direction*y1)]
    # A narrow forbidden strip protects the middle white line outside its
    # opening. Outer white boundaries are inset by the map uncertainty margin.
    return [rect(x_min, x_max, -spacing/2+margin, spacing/2-margin),
            rect(x_min, x_max, spacing/2+margin, 1.5*spacing-margin),
            rect(opening_start+margin, opening_end-margin,
                 -spacing/2+margin, 1.5*spacing-margin)]


def _rigid(a, b):
    ac, bc = a.mean(axis=0), b.mean(axis=0)
    u, _, vt = np.linalg.svd(np.dot((a-ac).T, b-bc))
    rotation = np.dot(vt.T, u.T)
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = np.dot(vt.T, u.T)
    return rotation, bc-np.dot(rotation, ac)


def rigid_motion(a, b):
    """RANSAC SE(2) fit in metres; reject collinear/poorly spread support."""
    if len(a) < 8:
        raise ValueError('vision_too_few_tracks')
    rng = np.random.RandomState(12)
    best = np.zeros(len(a), dtype=bool)
    for unused in range(60):
        ids = rng.choice(len(a), 2, replace=False)
        if np.linalg.norm(a[ids[0]]-a[ids[1]]) < .12:
            continue
        rotation, shift = _rigid(a[ids], b[ids])
        good = np.linalg.norm(np.dot(a, rotation.T)+shift-b, axis=1) < .012
        if good.sum() > best.sum(): best = good
    if best.sum() < 8 or best.mean() < .65:
        raise ValueError('vision_inconsistent_ground_motion')
    if np.linalg.eigvalsh(np.cov(a[best].T))[0] < .001:
        raise ValueError('vision_ground_features_not_spread')
    rotation, shift = _rigid(a[best], b[best])
    return rotation, shift, int(best.sum())


class GroundMotion(object):
    def __init__(self, camera):
        self.camera = camera
        self.previous = self.points = None
        self.stamp = None
        self.pose = (0., 0., 0.)
        self.inliers = 0

    def metric(self, points):
        c = self.camera
        return np.column_stack(((c['origin_v']-points[:, 1])/c['pixels_per_m'],
                                (c['origin_u']-points[:, 0])/c['pixels_per_m']))

    def update(self, gray, mask, stamp):
        valid = mask.copy()
        valid[:12] = valid[-12:] = 0
        valid[:, :12] = valid[:, -12:] = 0
        # Blue paint has lower grayscale contrast than white paint. Retain its
        # corners; forward/backward flow and rigid RANSAC still reject bad tracks.
        points = cv2.goodFeaturesToTrack(gray, maxCorners=140, qualityLevel=.005,
                                         minDistance=8, mask=valid, blockSize=5)
        if points is None or len(points) < 8:
            raise ValueError('vision_ground_features_missing')
        pose = self.pose
        if self.previous is not None:
            dt = stamp-self.stamp
            if not 0 < dt <= .4:
                raise ValueError('vision_frame_gap')
            nxt, good, _ = cv2.calcOpticalFlowPyrLK(self.previous, gray, self.points, None,
                                                  winSize=(21, 21), maxLevel=3)
            if nxt is None: raise ValueError('vision_flow_missing')
            back, back_good, _ = cv2.calcOpticalFlowPyrLK(gray, self.previous, nxt, None,
                                                        winSize=(21, 21), maxLevel=3)
            if back is None: raise ValueError('vision_flow_missing')
            a, b = self.points.reshape(-1, 2), nxt.reshape(-1, 2)
            keep = ((good.ravel() != 0) & (back_good.ravel() != 0) &
                    (np.linalg.norm(back.reshape(-1, 2)-a, axis=1) < 1.))
            keep &= (b[:, 0] >= 12) & (b[:, 0] < gray.shape[1]-12)
            keep &= (b[:, 1] >= 12) & (b[:, 1] < gray.shape[0]-12)
            rotation, shift, self.inliers = rigid_motion(self.metric(a[keep]), self.metric(b[keep]))
            # p_current = R * p_previous + t describes the stationary floor.
            # Its inverse is the car displacement expressed in the previous car frame.
            delta = -np.dot(rotation.T, shift)
            angle = -math.atan2(rotation[1, 0], rotation[0, 0])
            if np.linalg.norm(delta) > .65*dt+.008 or abs(angle) > 1.6*dt+.015:
                raise ValueError('vision_motion_jump')
            pose = tuple(world(self.pose, delta))+(wrap(self.pose[2]+angle),)
        self.previous, self.points, self.stamp = gray.copy(), points, stamp
        self.pose = pose
        return pose


class VisionUturnScene(object):
    def __init__(self, cfg):
        self.cfg = cfg
        self.active = False
        self.requested = False
        self.blue_enabled = cfg.get('uturn_blue_landmarks_enabled',False)
        self.side = None
        self.side_stamp = -1.
        self.reset()

    def reset(self):
        camera=dict(self.cfg['front_camera'])
        if self.blue_enabled: camera['origin_u']+=360
        self.motion = GroundMotion(camera)
        self.blue_landmarks = BlueLandmarks(self.cfg) if self.blue_enabled else None
        self.regions = None
        self.frame = None
        self.failed = False
        self.reason = 'vision_wait_uturn_blue'
        self.locked_side = None
        self.initial_pose = (0., 0., 0.)
        self.action_reference = None
        self.last_visual_pose = None

    def observe_lane(self, side, stamp):
        configured=self.cfg.get('uturn_followed_boundary','AUTO')
        if configured not in ('AUTO','LEFT','RIGHT'):
            raise ValueError('invalid uturn_followed_boundary')
        if configured != 'AUTO': side=configured
        if not self.active and stamp > self.side_stamp:
            if self.blue_enabled and self.frame is not None and side!=self.side:
                self.reset()
            # BOTH/UNKNOWN explicitly invalidates the choice, not "assume right".
            self.side = side if side in ('LEFT', 'RIGHT') else None
            self.side_stamp = stamp

    def set_active(self, active):
        if not active and self.active: self.reset()
        if not active and not self.requested and self.frame is not None: self.reset()
        self.active = active

    def update(self, bev, white, blue, stamp):
        if not (self.active or (self.blue_enabled and self.requested)) or self.failed: return None
        try:
            if self.regions is None:
                configured=self.cfg.get('uturn_followed_boundary','AUTO')
                if configured in ('LEFT','RIGHT'):
                    # Explicit reference is useful for startup-before-lane.
                    # A matching measured white boundary is still required below.
                    self.side,self.side_stamp=configured,stamp
                if self.side is None or not 0 <= stamp-self.side_stamp <= 1.:
                    self.reason = 'vision_followed_boundary_unknown'
                    return None
                spacing = self.cfg.get('uturn_lane_spacing', .6)
                # Require a straight, centered approach before attaching the
                # configured map to the rear axle. Curves need a different map.
                lines = cv2.HoughLinesP(white, 1, np.pi/180, 35,
                                       minLineLength=100, maxLineGap=10)
                matches = []
                for row in (() if lines is None else lines[:, 0]):
                    a, b = self.motion.metric(np.array(row, float).reshape(2, 2))
                    if b[0] < a[0]: a, b = b, a
                    dx, dy = b-a
                    if dx < .25 or abs(math.atan2(dy, dx)) > math.radians(10): continue
                    intercept = a[1]-a[0]*dy/dx
                    expected = spacing/2*(1 if self.side == 'LEFT' else -1)
                    if abs(intercept-expected) <= .06:
                        heading = math.atan2(dy,dx)
                        center = intercept*math.cos(heading)-expected
                        matches.append((heading,center))
                if not matches:
                    self.reason = 'vision_wait_straight_boundary_alignment'
                    return None
                heading, center = np.median(matches,axis=0)
                self.initial_pose = (0., -float(center), -float(heading))
                regions = corridor_regions(self.side, spacing,
                    self.cfg.get('uturn_opening_start_m', -.1),
                    self.cfg.get('uturn_opening_end_m', 1.2),
                    self.cfg.get('uturn_corridor_min_x_m', -.5),
                    self.cfg.get('uturn_corridor_max_x_m', 1.6),
                    self.cfg.get('uturn_boundary_margin_m', .03))
            else:
                regions = self.regions
            gray = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
            mask = cv2.bitwise_or(white, blue)
            segments=(image_segments(blue,self.motion.metric,self.motion.camera['pixels_per_m'],
                                     self.blue_landmarks.length+.08)
                      if self.blue_enabled else [])
            direct_blue=False
            try:
                measured = self.motion.update(gray, mask, stamp)
                pose = tuple(world(self.initial_pose,measured))+(wrap(self.initial_pose[2]+measured[2]),)
            except ValueError:
                if not self.blue_enabled: raise
                predicted=self.last_visual_pose or self.initial_pose
                if self.blue_landmarks.landmarks is None:
                    # Without flow there is no transform for older observations.
                    # Only the current complete pattern can initialize a map.
                    self.blue_landmarks.history=[]
                    self.blue_landmarks.update(segments,predicted,stamp)
                pose=self.blue_landmarks.update(segments,predicted,stamp)
                if not self.blue_landmarks.corrected: raise
                # Direct geometric localization retains the SAME fixed frame.
                # Restarting feature detection does not reset the vehicle pose.
                self.motion.previous=self.motion.points=None
                direct_blue=True
            if self.regions is None:
                self.regions = regions
                self.frame = 'uturn_vision_%.6f' % stamp
                self.locked_side = self.side
            if self.blue_enabled:
                if not direct_blue: pose=self.blue_landmarks.update(segments,pose,stamp)
                if self.blue_landmarks.corrected:
                    xy=local(self.initial_pose,pose)
                    self.motion.pose=tuple(xy)+(wrap(pose[2]-self.initial_pose[2]),)
                self.last_visual_pose=pose
                if not self.active:
                    self.reason='vision_collecting_'+self.blue_landmarks.reason
                    return None
                if self.blue_landmarks.landmarks is None:
                    self.reason=self.blue_landmarks.reason
                    return None
                if self.action_reference is None:
                    # The approach may have moved while collecting the map.
                    # Freeze the maneuver origin only after its blue trigger.
                    self.action_reference=[float(pose[0]),0.,0.]
                    self.regions=[[(x+pose[0],y) for x,y in region] for region in self.regions]
            # Observed white paint also vetoes paths through a presumed opening.
            # Voxel sampling bounds message size while retaining line continuity.
            v, u = np.nonzero(white)
            points = self.motion.metric(np.column_stack((u, v)))
            cells = np.unique(np.floor(points/.025).astype(np.int32), axis=0)
            if len(cells) > 2000: raise ValueError('vision_white_area_ambiguous')
            obstacles = [tuple(world(pose, (cell+.5)*.025)) for cell in cells]
            self.reason = 'vision_tracking'
            return dict(stamp=stamp, frame=self.frame, pose=list(pose), pose_source='vision',
                        lane_reference=self.action_reference or [0.,0.,0.],
                        map_source='configured_corridor_with_observed_white_veto',
                        followed_boundary=self.locked_side, regions=self.regions,
                        obstacles=obstacles, vision_inliers=self.motion.inliers)
        except (ValueError, cv2.error, np.linalg.LinAlgError) as exc:
            self.reason = str(exc)
            # Once a frame exists, never hide missing motion by resetting pose.
            if self.blue_enabled and not self.active:
                self.reset()
                self.reason='vision_approach_retry_'+str(exc)
            else:
                self.failed = self.frame is not None
            return None
