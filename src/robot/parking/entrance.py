"""Infer a side-facing bay from visible entrance endpoints, not a visible back.

Entrance position/orientation plus a known depth define the candidate rectangle.
This is a geometric implementation, not a downloaded/trained detector. See:
https://cslinzhang.github.io/ps/ (entrance endpoints and inferred other corners).
"""
from __future__ import division
import math
import numpy as np


def from_entrance(a, b, c, d, u, n, na, nc, metric, ppm, mask,
                  valid_mask, cfg, coverage):
    """Return (candidate, rejection) for an already width/parallel-checked pair.

    Both mouth endpoints must be observed, aligned and followed by white side
    segments into the bay. The direction before the mouth must be observable
    and mostly unpainted. Thus cropped ends and arbitrary interior fragments
    cannot become mouths just because their separation matches the bay width.
    """
    if valid_mask is None:
        return None, 'entrance_view_unknown'
    lo1, hi1 = sorted(float(np.dot(p, u)) for p in (a, b))
    lo2, hi2 = sorted(float(np.dot(p, u)) for p in (c, d))
    ends1=sorted((a,b),key=lambda p:float(np.dot(p,u)))
    ends2=sorted((c,d),key=lambda p:float(np.dot(p,u)))
    vectors=[ends[1]-ends[0] for ends in (ends1,ends2)]
    if any(np.linalg.norm(v)<1 for v in vectors):
        return None,'entrance_short_sides'
    vectors=[v/np.linalg.norm(v) for v in vectors]
    axis=vectors[0]+vectors[1]
    axis=axis/np.linalg.norm(axis)
    width = cfg['slots']['P4']['width']
    depth = cfg['slots']['P4']['length']
    minimum = cfg.get('parking_visible_side_m', .18)*ppm
    h, w = mask.shape

    def observed(p):
        x, y = int(round(p[0])), int(round(p[1]))
        # Require a real neighborhood, not just one interpolated edge pixel.
        radius = max(1, int(round(.0125*ppm)))
        if x-radius < 0 or y-radius < 0 or x+radius >= w or y+radius >= h:
            return False
        return (valid_mask is None or
                bool(np.all(valid_mask[y-radius:y+radius+1,
                                       x-radius:x+radius+1])))

    reason = 'entrance_not_aligned'
    for t1, t2, direction in ((lo1, lo2, 1), (hi1, hi2, -1)):
        if abs(t1-t2) > .06*ppm:
            continue
        entrance = [ends[0 if direction==1 else 1] for ends in (ends1,ends2)]
        mid = (entrance[0]+entrance[1])/2
        mx, my = metric(*mid)
        ix, iy = metric(*(mid+direction*axis*ppm))
        dx, dy = ix-mx, iy-my
        # This fallback is for perpendicular bays on the vehicle's side.
        # The endpoint nearer the road must point outward into that same side.
        if (abs(my) < .05 or dy*my <= 0 or
                abs(dy) < abs(dx) or mx <= 0):
            if reason == 'entrance_not_aligned':
                reason = 'entrance_not_side_facing'
            continue
        lengths = ((hi1-t1, hi2-t2) if direction == 1 else
                   (t1-lo1, t2-lo2))
        if min(lengths) < minimum:
            reason = 'entrance_short_sides'
            continue
        # A painted line spanning much more than the known depth is unlikely
        # to be this bay's side. Width alone is insufficient evidence.
        if max(lengths) > (depth+.12)*ppm:
            reason = 'entrance_sides_too_long'
            continue
        probe = [p-direction*v*q*ppm for p,v in zip(entrance,vectors)
                 for q in (.04, .08, .12)]
        if not all(observed(p) for p in entrance+probe):
            reason = 'entrance_outside_view'
            continue
        if any(coverage(p-direction*v*.04*ppm,
                        p-direction*v*.12*ppm) > .25 for p,v in zip(entrance,vectors)):
            reason = 'entrance_line_continues'
            continue
        if min(coverage(p, p+direction*v*minimum)
               for p,v in zip(entrance,vectors)) < .65:
            reason = 'entrance_side_coverage'
            continue
        center = mid+direction*axis*depth*ppm/2
        x, y = metric(*center)
        return dict(x=x, y=y, yaw=math.atan2(dy, dx), kind='perpendicular',
                    length=depth, width=width, evidence='entrance',
                    bottom_inferred=True,
                    observed_sides=[[list(metric(*a)),list(metric(*b))],
                                    [list(metric(*c)),list(metric(*d))]],
                    entrance=[list(metric(*p)) for p in entrance]), None
    return None, reason
