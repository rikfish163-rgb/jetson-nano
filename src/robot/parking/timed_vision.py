"""Same-frame bay lines and measured road bends; reuses calibrated test geometry."""
from __future__ import division
import math
import cv2
import numpy as np
from collections import OrderedDict
from robot.parallel_parking.reference_vision import ReferenceVision,paint_mask
from robot.parking.timed_core import near_lane_geometry

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


class BayLineVision(LineVision):
    def __init__(self, cfg, args):
        LineVision.__init__(self,cfg,args.side,args.min_line_m,args.angle_deg,
                            args.acquire_line_m,args.white_v_min)
        self.heading = 0.
        self.last_heading_stamp = None

    def set_lane_observation(self, observation):
        geometry = near_lane_geometry(observation)
        if geometry['valid']:
            self.heading = math.radians(geometry['heading_deg'])
            self.last_heading_stamp = observation['stamp']
        elif (observation and self.last_heading_stamp is not None and
              observation['stamp']-self.last_heading_stamp > .5):
            self.heading = 0.

    def road_axes(self, points):
        c,s = math.cos(self.heading),math.sin(self.heading)
        return np.dot(np.asarray(points),np.array([[c,-s],[s,c]]))

    def accepts(self, a, b):
        road = self.road_axes([a,b])
        dx,dy = road[1]-road[0]
        if np.linalg.norm(road[1]-road[0]) < self.min_length:
            return False
        if abs(dx) > abs(dy)*math.tan(math.radians(self.angle_deg)):
            return False
        middle = np.mean(road,axis=0)
        lateral = -middle[1] if self.side == 'right' else middle[1]
        return 0 < middle[0] <= 3. and .08 <= lateral <= 1.30

    def segments(self, mask):
        camera = self.camera
        ppm = camera['pixels_per_m']
        rows = cv2.HoughLinesP(mask,1,np.pi/180,max(8,int(ppm*.045)),
                              minLineLength=max(5,int(ppm*self.min_length)),
                              maxLineGap=max(2,int(ppm*.025)))
        accepted = []
        if rows is None:
            return accepted
        for u,v,w,z in rows[:,0]:
            a = np.array([(camera['origin_v']-v)/ppm,(camera['origin_u']-u)/ppm])
            b = np.array([(camera['origin_v']-z)/ppm,(camera['origin_u']-w)/ppm])
            if not self.accepts(a,b):
                continue
            middle = np.mean([a,b],axis=0)
            if any(abs(middle[0]-np.mean(line,axis=0)[0]) < .04 and
                   abs(middle[1]-np.mean(line,axis=0)[1]) < .20 for line in accepted):
                continue
            accepted.append([a,b])
        return accepted

    def paint_roi(self, bev):
        # Apply costly HSV/contrast operations only to the rotated bay strip.
        # Metric coordinates, scale, camera calibration and support checks stay unchanged.
        side = -1. if self.side == 'right' else 1.
        corners = np.array([[x,side*y] for x in (0.,3.) for y in (.08,1.30)])
        c,s = math.cos(self.heading),math.sin(self.heading)
        corners = np.dot(corners,np.array([[c,s],[-s,c]]))
        ppm = self.camera['pixels_per_m']
        uv = np.column_stack((self.camera['origin_u']-corners[:,1]*ppm,
                              self.camera['origin_v']-corners[:,0]*ppm))
        lo = np.maximum(0,np.floor(uv.min(axis=0)-.10*ppm).astype(int))
        hi = np.minimum([bev.shape[1],bev.shape[0]],np.ceil(uv.max(axis=0)+.10*ppm).astype(int))
        mask = np.zeros(bev.shape[:2],dtype=np.uint8)
        if np.all(hi > lo):
            mask[lo[1]:hi[1],lo[0]:hi[0]] = paint_mask(
                bev[lo[1]:hi[1],lo[0]:hi[0]],self.white_cfg,ppm)
        mask[self.detector.parking_valid == 0] = 0
        return mask

    def observe(self, frame):
        bev = self.detector.bev(frame,parking=True)
        mask = self.paint_roi(bev)
        candidates = [line for line in self.segments(mask) if self.paint_supported(line,mask)]
        lines = self.filter_lines(candidates)
        for line in lines:
            pts = [(int(self.camera['origin_u']-p[1]*self.camera['pixels_per_m']),
                    int(self.camera['origin_v']-p[0]*self.camera['pixels_per_m'])) for p in line]
            cv2.line(bev,pts[0],pts[1],(0,0,255),3)
        cv2.putText(bev,'P3 BAY LINES: %d; road yaw %.1f deg' %
                    (len(lines),math.degrees(self.heading)),(15,28),
                    cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,255),2)
        return len(lines),bev


def boundary_shape(points, min_curvature=.45, min_turn_deg=10.):
    result = dict(kind='UNKNOWN', curvature=None, turn_deg=None, span_m=0.)
    xy = np.asarray(points, dtype=float)
    if xy.ndim != 2 or xy.shape[1] != 2 or not np.all(np.isfinite(xy)):
        return result
    xy = xy[(xy[:, 0] >= .45) & (xy[:, 0] <= 1.5)]
    xy = xy[np.argsort(xy[:, 0])]
    if len(xy) < 5 or len(set(xy[:, 0])) != len(xy):
        return result
    span = float(xy[-1, 0]-xy[0, 0])
    result['span_m'] = span
    if span < .35:
        return result
    center = float(np.mean(xy[:, 0]))
    x = xy[:, 0]-center
    a, b, c = np.polyfit(x, xy[:, 1], 2)
    fit = a*x*x+b*x+c
    residual = float(np.sqrt(np.mean((xy[:, 1]-fit)**2)))
    result['residual_m'] = residual
    if residual > .02 or np.max(abs(xy[:, 1]-fit)) > .045:
        return result
    curvature = float(2*a/(1+b*b)**1.5)
    turn = math.degrees(math.atan(2*a*x[-1]+b)-math.atan(2*a*x[0]+b))
    sag = abs(float(a))*span*span/4.
    curved = abs(curvature) >= min_curvature and abs(turn) >= min_turn_deg and sag >= .012
    result.update(kind='CURVE' if curved else 'STRAIGHT', curvature=curvature,
                  turn_deg=turn, sag_m=sag, coefficients=[float(a), float(b), float(c)],
                  center_x=center, min_x=float(xy[0, 0]), max_x=float(xy[-1, 0]))
    return result


def both_boundary_curves(boundaries, min_curvature=.45, min_turn_deg=10.):
    left = boundary_shape(boundaries.get('LEFT', []), min_curvature, min_turn_deg)
    right = boundary_shape(boundaries.get('RIGHT', []), min_curvature, min_turn_deg)
    result = dict(left=left, right=right, both_curved=False, reason='need_two_curves')
    if left['kind'] != 'CURVE' or right['kind'] != 'CURVE':
        return result
    if left['curvature']*right['curvature'] <= 0:
        result['reason'] = 'opposite_bend_directions'
        return result
    lo, hi = max(left['min_x'], right['min_x']), min(left['max_x'], right['max_x'])
    if hi-lo < .30:
        result['reason'] = 'insufficient_shared_span'
        return result
    xs = np.linspace(lo, hi, 9)
    def at(row):
        a, b, c = row['coefficients']
        x = xs-row['center_x']
        return a*x*x+b*x+c
    width = at(left)-at(right)
    if np.any(width < .35) or np.any(width > 1.05):
        result['reason'] = 'boundary_pair_width_invalid'
        return result
    result.update(both_curved=True, reason='both_boundaries_curve_same_way')
    return result


class PairBuffer(object):
    """Pair by the ORIGINAL image stamp; never combine adjacent camera frames."""
    def __init__(self,limit=20):
        self.images,self.lanes = OrderedDict(),OrderedDict()
        self.limit,self.last = limit,-1.
    def put(self,channel,stamp,value):
        rows = self.images if channel == 'image' else self.lanes
        rows[stamp] = value
        while len(rows) > self.limit:
            rows.popitem(last=False)
    def newest(self,now,timeout):
        stamps = [t for t in self.images if t in self.lanes and t > self.last and 0 <= now-t <= timeout]
        if not stamps:
            return None
        stamp = max(stamps)
        self.last = stamp
        return (self.images[stamp],self.lanes[stamp])

