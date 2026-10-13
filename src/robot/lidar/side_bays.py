"""Shared right-front reference line and box 1/2 geometry, Python 2/3."""
from __future__ import division
import math
import random

# Rectangles keep vehicle axes; only their origin moves to the line midpoint.
BOXES = [('1', (-0.7, 0.0, -0.5, -0.1)),
         ('2', (-1.4, -0.7, -0.5, -0.1))]


def fit_segment(points):
    """Orthogonal least squares and endpoints of the observed point segment."""
    cx = sum(p[0] for p in points) / len(points)
    cy = sum(p[1] for p in points) / len(points)
    xx = sum((p[0]-cx)**2 for p in points)
    yy = sum((p[1]-cy)**2 for p in points)
    xy = sum((p[0]-cx)*(p[1]-cy) for p in points)
    angle = 0.5 * math.atan2(2*xy, xx-yy)
    ux, uy = math.cos(angle), math.sin(angle)
    projections = [(p[0]-cx)*ux + (p[1]-cy)*uy for p in points]
    lo, hi = min(projections), max(projections)
    ends = [[cx+v*ux, cy+v*uy] for v in (lo, hi)]
    residual = math.sqrt(sum((-(p[0]-cx)*uy+(p[1]-cy)*ux)**2
                             for p in points) / len(points))
    return dict(endpoints=ends, midpoint=[(ends[0][i]+ends[1][i])/2 for i in (0, 1)],
                length=hi-lo, rms=residual, count=len(points))


def detect_line(points, radius=1.0, tolerance=0.02, min_points=8,
                min_length=0.12, max_gap=0.10):
    """Bounded deterministic RANSAC, splitting disconnected collinear objects."""
    roi = [p for p in points if p[0] > 0 and p[1] < 0
           and p[0]*p[0]+p[1]*p[1] <= radius*radius]
    if len(roi) < min_points:
        return None
    rng = random.Random(0)
    best, best_score = None, (0, 0)
    for _ in range(160):
        a, b = rng.sample(roi, 2)
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(dx, dy)
        if length < min_length/2:
            continue
        ux, uy = dx/length, dy/length
        hits = sorted(((p[0]*ux+p[1]*uy, p) for p in roi
                       if abs((p[0]-a[0])*uy-(p[1]-a[1])*ux) <= tolerance),
                      key=lambda row: row[0])
        groups = []
        for projection, point in hits:
            if not groups or projection-previous > max_gap:
                groups.append([])
            groups[-1].append(point)
            previous = projection
        for group in groups:
            if len(group) < min_points:
                continue
            segment = fit_segment(group)
            # Require every retained point to remain within the final fit band.
            e0, e1 = segment['endpoints']
            span = segment['length']
            if span < min_length:
                continue
            kept = [p for p in group if abs((p[0]-e0[0])*(e1[1]-e0[1]) -
                    (p[1]-e0[1])*(e1[0]-e0[0]))/span <= tolerance]
            if len(kept) < min_points:
                continue
            segment = fit_segment(kept)
            score = (segment['count'], segment['length'])
            if segment['length'] >= min_length and segment['rms'] <= tolerance and score > best_score:
                best, best_score = segment, score
    return best


def measure_boxes(points, line):
    boxes = []
    if line is not None:
        mx, my = line['midpoint']
        for name, bounds in BOXES:
            x0, x1, y0, y1 = bounds
            # Half-open X intervals assign the shared x=-0.7 edge to box 1.
            count = sum(1 for x, y in points if x0 <= x-mx < x1
                        and y0 <= y-my <= y1)
            if name == '1':
                count += sum(1 for x, y in points if x-mx == x1 and y0 <= y-my <= y1)
            boxes.append(dict(name=name, relative=list(bounds),
                              bounds=[mx+x0, mx+x1, my+y0, my+y1], count=count))
    return dict(line=line, boxes=boxes,
                difference=boxes[0]['count']-boxes[1]['count'] if boxes else None)


def measure(points):
    return measure_boxes(points, detect_line(points))
