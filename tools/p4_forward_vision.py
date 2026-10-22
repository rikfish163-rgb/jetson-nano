# -*- coding: utf-8 -*-
"""Measured front-facing U-shaped bay, using the existing front calibration."""
from __future__ import division
import math
import os
import cv2
import numpy as np
from robot.common.config import read_mapping
from robot.camera.vision import GroundDetector
from robot.parallel_parking.reference_vision import paint_mask


def merge_segments(rows, tolerance=.025):
    groups = []
    rows = sorted(rows, key=lambda ab: np.linalg.norm(ab[1]-ab[0]), reverse=True)
    for a, b in rows[:100]:
        length = np.linalg.norm(b-a)
        if length < .02:
            continue
        axis = (b-a)/length
        for g in groups:
            p, q, u = g
            normal = np.array([-u[1], u[0]])
            if abs(np.dot(axis, u)) < math.cos(math.radians(5)):
                continue
            if max(abs(np.dot(a-p, normal)), abs(np.dot(b-p, normal))) > tolerance:
                continue
            lo, hi = sorted((np.dot(a-p, u), np.dot(b-p, u)))
            span = np.dot(q-p, u)
            if lo > span+.06 or hi < -.06:
                continue
            g[0], g[1] = p+min(0, lo)*u, p+max(span, hi)*u
            break
        else:
            if len(groups) < 35:
                groups.append([a.copy(), b.copy(), axis])
    return [(a, b) for a, b, u in groups]


def geometry(segments, cfg, support):
    """Select two observed side rails and an observed continuous terminating edge."""
    sides, transverse = [], []
    for a, b in segments:
        dx, dy = b-a
        if abs(dx) >= cfg['min_side_m']:
            a, b = sorted((a, b), key=lambda p: p[0])
            slope = (b[1]-a[1])/(b[0]-a[0])
            yaw = math.atan(slope)
            if abs(yaw) <= math.radians(cfg['max_heading_deg']) and support(a, b) >= cfg['paint_support']:
                sides.append((a, b, slope, a[1]-slope*a[0], yaw))
        if abs(dy) >= cfg['min_bottom_m'] and abs(dx) <= abs(dy)*math.tan(math.radians(40)):
            transverse.append((a, b))
    candidates = []
    for i, left in enumerate(sides):
        for right in sides[i+1:]:
            a, b, k1, c1, yaw1 = left
            d, e, k2, c2, yaw2 = right
            if abs(yaw1-yaw2) > math.radians(cfg['parallel_tolerance_deg']):
                continue
            lo, hi = max(a[0], d[0]), min(b[0], e[0])
            if hi-lo < cfg['min_side_m']:
                continue
            x = (lo+hi)/2
            width = abs((k1*x+c1)-(k2*x+c2))*math.cos((yaw1+yaw2)/2)
            if abs(width-cfg['bay_width_m']) > cfg['width_tolerance_m']:
                continue
            heading = (yaw1+yaw2)/2
            center = ((k1+k2)*cfg['lookahead_m']+c1+c2)/2
            if abs(center) > .65:
                continue
            bottoms = []
            for p, q in transverse:
                if abs(q[1]-p[1]) < 1e-6:
                    continue
                k = (q[0]-p[0])/(q[1]-p[1])
                c = p[0]-k*p[1]
                if min(abs(1-k*k1), abs(1-k*k2)) < .3:
                    continue
                x1, x2 = (k*c1+c)/(1-k*k1), (k*c2+c)/(1-k*k2)
                p1, p2 = np.array([x1, k1*x1+c1]), np.array([x2, k2*x2+c2])
                if not (a[0]-.07 <= x1 <= b[0]+.07 and d[0]-.07 <= x2 <= e[0]+.07):
                    continue
                # The far ends of BOTH side rails must meet the bottom. An
                # entrance line or arbitrary road cross-line is not a bottom.
                if max(abs(x1-b[0]), abs(x2-e[0])) > .12:
                    continue
                ylo, yhi = sorted((p[1], q[1]))
                if min(p1[1], p2[1]) < ylo-.05 or max(p1[1], p2[1]) > yhi+.05:
                    continue
                if np.mean([x1, x2]) <= .05:
                    continue
                axis = np.array([math.cos(heading), math.sin(heading)])
                if abs(np.dot((q-p)/np.linalg.norm(q-p), axis)) > math.sin(math.radians(15)):
                    continue
                if support(p1, p2) < cfg['paint_support']:
                    continue
                bottoms.append(((p1+p2)/2, p1, p2))
            selected = max(bottoms, key=lambda row: row[0][0]) if bottoms else None
            bottom = selected[0] if selected is not None else None
            candidates.append(dict(sides_valid=True, bottom_valid=bottom is not None,
                                   center_y_m=float(center), heading_rad=float(heading),
                                   heading_deg=float(math.degrees(heading)), width_m=float(width),
                                   visible_side_m=float(hi-lo),
                                   bottom_midpoint_m=bottom.tolist() if bottom is not None else None,
                                   bottom_y_m=float(bottom[1]) if bottom is not None else None,
                                   bottom_line_m=[selected[1].tolist(), selected[2].tolist()] if selected is not None else None,
                                   rails=[[a.tolist(), b.tolist()], [d.tolist(), e.tolist()]]))
    return candidates


class BayVision(object):
    def __init__(self, ignored_cfg, args):
        self.cfg = args.p4_config
        # Use CURRENT production intrinsics/pose, not the separate old P1 trial H
        # loaded by the shared runner. Half-size canvas changes no metric scale.
        root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        base = read_mapping(os.path.join(root, 'src/robot/config/competition.yaml'))
        camera = dict(base['front_camera'])
        camera['H'] = np.dot(np.diag([.5, .5, 1.]), np.asarray(camera['H']).reshape(3, 3)).reshape(-1).tolist()
        for key in ('bev_width', 'bev_height', 'origin_u', 'origin_v', 'pixels_per_m'):
            camera[key] *= .5
        camera['bev_width'], camera['bev_height'] = int(camera['bev_width']), int(camera['bev_height'])
        self.detector = GroundDetector(dict(base, front_camera=camera))
        self.camera = dict(camera, origin_u=camera['origin_u']+360)
        self.white_cfg = dict(base['white'], v_min=self.cfg['white_v_min'], s_max=self.cfg['white_s_max'])
        self.inverse = np.linalg.inv(np.asarray(camera['H']).reshape(3, 3))
        self.data = {}
        self.previous = None
        self.previous_sides = None

    def scene_data(self):
        return dict(self.data)

    def analyze_mask(self, mask, valid, image_height):
        c, cfg = self.camera, self.cfg
        ppm = c['pixels_per_m']
        mask = mask.copy()
        mask[valid == 0] = 0
        xs = (c['origin_v']-np.arange(mask.shape[0]))/ppm
        ys = (c['origin_u']-np.arange(mask.shape[1]))/ppm
        mask[(xs < .02) | (xs > 1.5), :] = 0
        mask[:, abs(ys) > .95] = 0
        tracking = (self.previous is not None and
                    self.previous.get('bottom_row', 0) >= cfg['bottom_near_row']-.20)
        short = cfg['tracking_min_side_m'] if tracking else cfg['min_side_m']
        raw = cv2.HoughLinesP(mask, 1, np.pi/180, 5 if tracking else 16,
                              minLineLength=max(4, int(ppm*short)), maxLineGap=max(2, int(ppm*.025)))
        rows = []
        if raw is not None:
            for u, v, w, z in raw[:, 0]:
                rows.append((np.array([(c['origin_v']-v)/ppm, (c['origin_u']-u)/ppm]),
                             np.array([(c['origin_v']-z)/ppm, (c['origin_u']-w)/ppm])))
        painted = cv2.dilate(mask, np.ones((5, 5), np.uint8))
        def support(a, b):
            points = a+(b-a)*np.linspace(.02, .98, 51)[:, None]
            us = np.rint(c['origin_u']-points[:, 1]*ppm).astype(int)
            vs = np.rint(c['origin_v']-points[:, 0]*ppm).astype(int)
            inside = (us >= 0) & (us < mask.shape[1]) & (vs >= 0) & (vs < mask.shape[0])
            values = np.zeros(len(us), bool)
            values[inside] = (painted[vs[inside], us[inside]] > 0) & (valid[vs[inside], us[inside]] > 0)
            return float(values.mean())
        candidates = geometry(merge_segments(rows), dict(cfg, min_side_m=short), support)
        if not candidates:
            scene = dict(sides_valid=False, bottom_valid=False, image_height=image_height)
        else:
            # Follow the same rail pair as the bottom enters view. A bottom in
            # an adjacent bay must not take priority over the acquired sides.
            old_sides = self.previous_sides
            center = old_sides['center_y_m'] if old_sides else 0.
            heading = old_sides['heading_rad'] if old_sides else 0.
            def distance(s):
                return abs(s['center_y_m']-center) + cfg['lookahead_m']*abs(s['heading_rad']-heading)
            closest = min(distance(s) for s in candidates)
            # Multiple Hough edges can describe the same painted rail. Within
            # the segment merge tolerance, prefer the complete observed U.
            matching = [s for s in candidates if distance(s) <= closest+.025]
            scene = min(matching, key=lambda s: (not s['bottom_valid'], distance(s)))
            self.previous_sides = dict(scene)
        # Near the edge, the terminal side tips can disappear before the actual
        # cross-line. Keep tracking the OBSERVED cross-line, never infer it from
        # vanished paint. Core bounds this fallback by recent measured rails and
        # holds the last steering while only the bottom is observable.
        if tracking and not scene.get('bottom_valid'):
            old = self.previous
            ox, oy = old['bottom_midpoint_m']
            possible = []
            for a, b in merge_segments(rows):
                delta = b-a
                length = np.linalg.norm(delta)
                if length < cfg['min_bottom_m']:
                    continue
                axis = np.array([math.cos(old['heading_rad']), math.sin(old['heading_rad'])])
                if abs(np.dot(delta/length, axis)) > math.sin(math.radians(15)):
                    continue
                middle = (a+b)/2
                if not -.12 <= middle[0]-ox <= .04 or abs(middle[1]-oy) > .06:
                    continue
                if abs(length-old['width_m']) > cfg['width_tolerance_m'] or support(a, b) < cfg['paint_support']:
                    continue
                possible.append((middle, a, b))
            if possible:
                selected = min(possible, key=lambda row: np.linalg.norm(row[0]-[ox, oy]))
                bottom = selected[0]
                scene = dict(old, sides_valid=False, bottom_valid=True, tracking_bottom=True,
                             bottom_midpoint_m=bottom.tolist(), bottom_y_m=float(bottom[1]),
                             bottom_line_m=[selected[1].tolist(), selected[2].tolist()])
        scene['image_height'] = image_height
        if scene['bottom_valid']:
            x, y = scene['bottom_midpoint_m']
            p = np.dot(self.inverse, [c['origin_u']-360-y*ppm, c['origin_v']-x*ppm, 1.])
            if abs(p[2]) < 1e-8:
                scene['bottom_valid'] = False
            else:
                scene['bottom_row'] = float(p[1]/p[2]/image_height)
                if not 0 <= scene['bottom_row'] <= 1:
                    scene['bottom_valid'] = False
        if scene['bottom_valid']:
            self.previous = dict(scene)
        return scene

    def observe(self, frame):
        bev = self.detector.bev(frame, parking=True)
        mask = paint_mask(bev, self.white_cfg, self.camera['pixels_per_m'])
        self.data = self.analyze_mask(mask, self.detector.parking_valid, frame.shape[0])
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.data['image_ok'] = bool(5 < gray.mean() < 250 and gray.std() > 3)
        count = int(self.data.get('sides_valid', False))*2 + int(self.data.get('bottom_valid', False))
        for line in (self.data.get('rails', []) if self.data.get('sides_valid') else []):
            points = [(int(self.camera['origin_u']-y*self.camera['pixels_per_m']),
                       int(self.camera['origin_v']-x*self.camera['pixels_per_m'])) for x, y in line]
            cv2.line(bev, points[0], points[1], (0, 255, 0), 2)
        bottom_line = self.data.get('bottom_line_m')
        if self.data.get('bottom_valid') and bottom_line:
            points = [(int(self.camera['origin_u']-y*self.camera['pixels_per_m']),
                       int(self.camera['origin_v']-x*self.camera['pixels_per_m'])) for x, y in bottom_line]
            cv2.line(bev, points[0], points[1], (0, 255, 255), 3)
        midpoint = self.data.get('bottom_midpoint_m')
        if midpoint is not None and self.data.get('bottom_valid'):
            x, y = midpoint
            cv2.circle(bev, (int(self.camera['origin_u']-y*self.camera['pixels_per_m']),
                            int(self.camera['origin_v']-x*self.camera['pixels_per_m'])), 5, (0, 0, 255), -1)
        cv2.putText(bev, 'P4 sides=%s bottom=%s' % (self.data.get('sides_valid'), self.data.get('bottom_valid')),
                    (15, 28), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 2)
        return count, bev

    def image_point(self, point):
        x, y = point
        c = self.camera
        projected = np.dot(self.inverse, [c['origin_u']-360-y*c['pixels_per_m'],
                                          c['origin_v']-x*c['pixels_per_m'], 1.])
        if abs(projected[2]) < 1e-8:
            return None
        pixel = projected[:2]/projected[2]
        if not np.all(np.isfinite(pixel)) or np.max(abs(pixel)) > 10000:
            return None
        return tuple(int(round(value)) for value in pixel)

    def overlay(self, frame):
        """Draw measured geometry on a rectified live front image, no control."""
        out = cv2.remap(frame, self.detector.maps[0], self.detector.maps[1], cv2.INTER_LINEAR)
        scene = self.data
        def line(points, color, width=2):
            pixels = [self.image_point(p) for p in points]
            if all(p is not None for p in pixels):
                cv2.line(out, pixels[0], pixels[1], color, width)
        if scene.get('sides_valid'):
            for rail in scene.get('rails', []):
                line(rail, (0, 255, 0), 3)
        if scene.get('bottom_valid'):
            bottom = scene.get('bottom_line_m')
            if bottom:
                line(bottom, (0, 255, 255), 3)
            middle = self.image_point(scene['bottom_midpoint_m'])
            if middle is not None:
                cv2.circle(out, middle, 6, (0, 0, 255), -1)
                cv2.putText(out, 'BOTTOM MID', (middle[0]+8, middle[1]-8),
                            cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 0, 255), 1)
        row = int(round(self.cfg['bottom_near_row']*out.shape[0]))
        cv2.line(out, (0, row), (out.shape[1]-1, row), (255, 170, 0), 1)
        cv2.putText(out, 'P4 SIDES:%s BOTTOM:%s' % (scene.get('sides_valid', False), scene.get('bottom_valid', False)),
                    (12, 26), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 255, 255), 2)
        return out


class NoRoadVision(object):
    """Shared runner adapter; this trial never performs road lane following."""
    def __init__(self, *args):
        pass

    def observe(self, frame, stamp):
        return dict(both_curved=False), None
