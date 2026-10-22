"""Front-camera transverse white-line observation and one-shot stop decision."""
from __future__ import division
import math
import cv2
import numpy as np
from robot.parallel_parking.reference_vision import ReferenceVision, paint_mask


class LineVision(object):
    def __init__(self, cfg, side='right', min_length=.08, angle_deg=25., acquire_length=.12,
                 white_v_min=190):
        reference = ReferenceVision(cfg)
        self.detector, self.camera = reference.detector, reference.camera
        self.cfg, self.side = cfg, side
        self.white_cfg = dict(cfg['white'], v_min=white_v_min)
        self.min_length, self.angle_deg = min_length, angle_deg
        self.acquire_length = acquire_length
        self.history = []

    def segments(self, mask):
        c = self.camera
        ppm = c['pixels_per_m']
        rows = cv2.HoughLinesP(mask, 1, np.pi/180, max(8, int(ppm*.045)),
                              minLineLength=max(5, int(ppm*self.min_length)),
                              maxLineGap=max(2, int(ppm*.025)))
        accepted = []
        if rows is None:
            return accepted
        for u, v, w, z in rows[:, 0]:
            a = np.array([(c['origin_v']-v)/ppm, (c['origin_u']-u)/ppm])
            b = np.array([(c['origin_v']-z)/ppm, (c['origin_u']-w)/ppm])
            dx, dy = b-a
            if np.linalg.norm(b-a) < self.min_length:
                continue
            if abs(dx) > abs(dy)*math.tan(math.radians(self.angle_deg)):
                continue
            mid = (a+b)/2
            # x forward, y left. Restrict to the adjacent parking strip.
            lateral = -mid[1] if self.side == 'right' else mid[1]
            if not 0 < mid[0] <= 3.0 or not .08 <= lateral <= 1.30:
                continue
            if any(abs(mid[0]-np.mean(line, axis=0)[0]) < .04 and
                   abs(mid[1]-np.mean(line, axis=0)[1]) < .20 for line in accepted):
                continue
            accepted.append([a, b])
        return accepted

    def filter_lines(self, candidates):
        accepted = []
        previous = [line for rows in self.history for line in rows]
        for line in candidates:
            a, b = np.asarray(line)
            length = np.linalg.norm(b-a)
            if length > .70:
                continue
            known = False
            for old in previous:
                center = np.mean(old, axis=0)
                delta_x = (a[0]+b[0])/2-center[0]
                overlap = min(max(a[1], b[1]), max(p[1] for p in old))-max(min(a[1], b[1]), min(p[1] for p in old))
                if -.20 <= delta_x <= .08 and overlap >= .04:
                    known = True
                    break
            if length >= self.acquire_length or known:
                accepted.append(line)
        self.history = (self.history+[accepted])[-3:]
        return accepted

    def paint_supported(self, line, mask):
        a, b = np.asarray(line)
        delta = b-a
        length = np.linalg.norm(delta)
        normal = np.array([-delta[1], delta[0]])/length
        ppm = self.camera['pixels_per_m']
        centers = a+np.linspace(.1, .9, 25)[:, None]*delta
        samples = centers[:, None, :]+np.arange(-.08, .081, 1./ppm)[None, :, None]*normal
        u = np.rint(self.camera['origin_u']-samples[:, :, 1]*ppm).astype(int)
        v = np.rint(self.camera['origin_v']-samples[:, :, 0]*ppm).astype(int)
        inside = (u >= 0) & (u < mask.shape[1]) & (v >= 0) & (v < mask.shape[0])
        paint = np.zeros(u.shape, bool)
        paint[inside] = mask[v[inside], u[inside]] > 0
        widths = paint.sum(axis=1)/ppm
        return .008 <= np.median(widths) <= .065 and np.mean(widths >= .008) >= .8

    def observe(self, frame):
        bev = self.detector.bev(frame, parking=True)
        mask = paint_mask(bev, self.white_cfg, self.camera['pixels_per_m'])
        mask[self.detector.parking_valid == 0] = 0
        # Crop only the lateral parking strip; preserve near-image fragments.
        columns = (self.camera['origin_u']-np.arange(mask.shape[1]))/self.camera['pixels_per_m']
        lateral = -columns if self.side == 'right' else columns
        mask[:, (lateral < .08) | (lateral > 1.30)] = 0
        candidates = [line for line in self.segments(mask) if self.paint_supported(line, mask)]
        lines = self.filter_lines(candidates)
        for line in lines:
            points = [(int(self.camera['origin_u']-p[1]*self.camera['pixels_per_m']),
                       int(self.camera['origin_v']-p[0]*self.camera['pixels_per_m'])) for p in line]
            cv2.line(bev, points[0], points[1], (0, 0, 255), 3)
        cv2.putText(bev, 'TRANSVERSE WHITE LINES: %d' % len(lines), (15, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 255), 2)
        return len(lines), bev


class StopRun(object):
    def __init__(self, speed=30, lost_seconds=.3, seen_frames=2,
                 seek_seconds=5., max_seconds=20., camera_timeout=.5, steering=-5,
                 lost_frames=3, end_verify_seconds=2.):
        self.speed, self.lost_seconds = speed, lost_seconds
        self.steering = steering
        self.seen_frames = seen_frames
        self.lost_frames = lost_frames
        self.seek_seconds, self.max_seconds = seek_seconds, max_seconds
        self.camera_timeout = camera_timeout
        self.started = None
        self.stamp, self.received = None, None
        self.seen_count, self.missing_count = 0, 0
        self.seen, self.missing_since = False, None
        self.both_curved, self.line_count = False, None
        self.stop_candidate_since = None
        self.end_verify_seconds = end_verify_seconds
        self.finished, self.reason = False, 'READY'

    def start(self, now):
        self.started = now

    def abort(self, reason):
        self.finished, self.reason = True, reason
        return (0, 0)

    def observe(self, count, stamp, received, both_curved=False):
        if self.finished or (self.stamp is not None and stamp <= self.stamp):
            return
        self.stamp, self.received = stamp, received
        self.line_count, self.both_curved = count, both_curved is True
        if self.started is None:
            # Countdown observations can establish visibility, but cannot stop
            # a run that has not started. Blank countdown frames disarm it.
            self.seen_count = self.seen_count+1 if count > 0 else 0
            self.seen = self.seen_count >= self.seen_frames
            return
        # Brake at the first jointly plausible end. Confirmation is performed
        # while stationary, and ambiguous observations must NEVER re-arm motion.
        if self.seen and count == 0 and self.both_curved and self.stop_candidate_since is None:
            self.stop_candidate_since = received
        if count > 0:
            self.seen_count += 1
            self.seen = self.seen or self.seen_count >= self.seen_frames
            self.missing_count, self.missing_since = 0, None
        else:
            self.seen_count = 0
            if self.seen and self.both_curved:
                self.missing_count += 1
                if self.missing_since is None:
                    self.missing_since = received
                if (self.missing_count >= self.lost_frames and
                        received-self.missing_since >= self.lost_seconds):
                    self.abort('BAY_END_CONFIRMED')
            else:
                self.missing_count, self.missing_since = 0, None

    def tick(self, now, ros_now):
        if self.finished:
            return (0, 0)
        if (self.received is None or not 0 <= now-self.received <= self.camera_timeout or
                self.stamp is None or not 0 <= ros_now-self.stamp <= self.camera_timeout):
            return self.abort('CAMERA_TIMEOUT')
        if self.started is None:
            return (0, 0)
        if self.stop_candidate_since is not None:
            if now-self.stop_candidate_since >= self.end_verify_seconds:
                return self.abort('END_UNCONFIRMED_STOPPED')
            self.reason = 'STOP_VERIFY_END'
            return (0, 0)
        if now-self.started >= self.max_seconds:
            return self.abort('MAX_RUN_TIMEOUT')
        if not self.seen and now-self.started >= self.seek_seconds:
            return self.abort('NO_LINES_SEEN')
        self.reason = 'FORWARD_SEEN_LINES' if self.seen else 'FORWARD_SEEK_LINES'
        if self.seen and self.line_count == 0:
            self.reason = 'FORWARD_WAIT_BOTH_CURVES'
        return (self.speed, self.steering)
