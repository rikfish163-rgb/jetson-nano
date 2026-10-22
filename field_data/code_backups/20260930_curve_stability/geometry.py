"""Planar bicycle geometry. All poses refer to the rear axle, in metres/radians."""
from __future__ import division

import math

try:
    _isfinite = math.isfinite
except AttributeError:
    def _isfinite(value):
        return not math.isinf(value) and not math.isnan(value)

_INF = float('inf')


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def world(pose, point):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (pose[0] + c * point[0] - s * point[1],
            pose[1] + s * point[0] + c * point[1])


def local(pose, point):
    dx, dy = point[0] - pose[0], point[1] - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (c * dx + s * dy, -s * dx + c * dy)


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def bicycle(pose, ds, steering, wheelbase):
    k = math.tan(steering) / wheelbase
    yaw = pose[2]
    if abs(k) < 1e-8:
        return tuple(world(pose, (ds, 0))) + (yaw,)
    return (pose[0] + (math.sin(yaw + ds*k) - math.sin(yaw))/k,
            pose[1] + (math.cos(yaw) - math.cos(yaw + ds*k))/k,
            wrap(yaw + ds*k))


def tires(pose, cfg):
    return [world(pose, (x, y)) for x in (0, cfg['wheelbase'])
            for y in (-cfg['track']/2, cfg['track']/2)]


def inside_slot(pose, slot, cfg):
    # Include finite tyre contact radius. Body overhang is an obstacle check,
    # not the four-wheels-inside scoring criterion.
    r = cfg['tire_radius']
    return all(abs(local(slot['pose'], p)[0]) <= slot['length']/2-r and
               abs(local(slot['pose'], p)[1]) <= slot['width']/2-r
               for p in tires(pose, cfg))


def _footprint_offsets(cfg, spacing=0.10):
    lo, hi = -cfg['rear_overhang'], cfg['wheelbase']+cfg['front_overhang']
    half = cfg['body_width']/2
    nx = max(1, int(math.ceil((hi-lo)/spacing)))
    ny = max(1, int(math.ceil(2*half/spacing)))
    return [(lo+(hi-lo)*i/nx, -half+2*half*j/ny)
            for i in range(nx+1) for j in range(ny+1)]


def footprint(pose, cfg, spacing=0.10):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return [(pose[0] + c*x - s*y, pose[1] + s*x + c*y)
            for x, y in _footprint_offsets(cfg, spacing)]


def collision(pose, obstacles, cfg):
    margin = cfg['obstacle_margin']
    c,s = math.cos(pose[2]),math.sin(pose[2])
    lo,hi = -cfg['rear_overhang']-margin,cfg['wheelbase']+cfg['front_overhang']+margin
    half = cfg['body_width']/2+margin
    for p in obstacles:
        dx,dy = p[0]-pose[0],p[1]-pose[1]
        x,y = c*dx+s*dy,-s*dx+c*dy
        if lo <= x <= hi and abs(y) <= half:
            return True
    return False


def in_box(point, box):
    return box[0] <= point[0] <= box[1] and box[2] <= point[1] <= box[3]


def slot_samples(slot):
    return [world(slot['pose'], (slot['length']*(i/6-0.5), slot['width']*(j/4-0.5)))
            for i in range(7) for j in range(5)]
