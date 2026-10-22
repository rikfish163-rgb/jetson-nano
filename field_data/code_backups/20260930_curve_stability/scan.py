"""LaserScan processing, occupancy and obstacle-shape evidence. No ROS I/O."""
from __future__ import division
import math
from robot.common.geometry import _isfinite
from robot.common.geometry import _INF
from robot.common.geometry import world
from robot.common.geometry import local
from robot.common.geometry import wrap
from robot.common.geometry import distance


def classify_scan_cluster(points, cfg):
    """Compare line, corner and circular cross-section models; no semantic claim."""
    import numpy as np
    row = dict(kind='unknown',reason='too_few_points',point_count=len(points),semantic_verified=False,
               motion_target=False)
    if len(points) < max(6,cfg.get('cluster_min_points',6)):
        return row
    xy = np.asarray(points,dtype=float)
    if xy.ndim != 2 or xy.shape[1] != 2 or not np.all(np.isfinite(xy)):
        row['reason'] = 'invalid_points'
        return row
    origin = np.mean(xy,axis=0)
    q = xy-origin
    values,vectors = np.linalg.eigh(np.dot(q.T,q)/len(q))
    line_error = math.sqrt(max(0,float(values[0])))
    width = float(np.ptp(np.dot(q,vectors[:,1])))
    row.update(center=(float(origin[0]),float(origin[1])),span_m=width,line_error_m=line_error)
    if width < .002:
        row['reason'] = 'degenerate_cluster'
        return row
    flat_error = cfg.get('flat_line_error',.0025)
    if cfg.get('shape_mode','line_reject') == 'line_reject':
        # Orthogonal distance, not y=f(x): invariant to board orientation.
        residual = np.abs(np.dot(q,vectors[:,0]))
        ratio = line_error/width
        inliers = float(np.mean(residual <= 2*flat_error))
        row.update(line_ratio=ratio,line_inlier_fraction=inliers)
        if width < cfg.get('compact_min_span',.015):
            row['reason'] = 'shape_below_resolution'
            return row
        # Split along the principal axis, not input order or world X/Y.
        ordered = q[np.argsort(np.dot(q,vectors[:,1]))]
        axes, reliable = [], True
        for part in (ordered[:len(q)//2],ordered[len(q)//2:]):
            centered = part-np.mean(part,axis=0)
            vals,vecs = np.linalg.eigh(np.dot(centered.T,centered)/len(part))
            major = math.sqrt(max(0,float(vals[1])))
            minor = math.sqrt(max(0,float(vals[0])))
            reliable = reliable and major >= flat_error and minor <= .35*major
            axes.append(vecs[:,1])
        bend = math.degrees(math.acos(min(1.,abs(float(np.dot(axes[0],axes[1]))))))
        row.update(segment_bend_deg=bend,segment_directions_reliable=bool(reliable))
        if (reliable and bend <= cfg.get('line_max_bend_deg',10) and line_error <= flat_error and
                ratio <= cfg.get('line_max_ratio',.025) and
                inliers >= cfg.get('line_inlier_ratio',.9)):
            row.update(kind='wall_candidate' if width >= cfg.get('wall_min_width',.55) else 'flat_board_candidate',
                       reason='supported_straight_shape')
        elif width > cfg.get('compact_max_span',.50):
            row.update(kind='other',reason='oversize_non_line')
        elif line_error > flat_error and ratio >= cfg.get('obstacle_min_ratio',.05):
            row.update(kind='obstacle_candidate',reason='supported_irregular_shape',motion_target=True)
        elif (reliable and bend >= cfg.get('obstacle_min_bend_deg',25) and
                line_error > .5*flat_error):
            row.update(kind='obstacle_candidate',reason='supported_curved_shape',motion_target=True)
        else:
            # Failure to confirm a board is not positive evidence of an obstacle.
            row['reason'] = 'ambiguous_shape'
        return row
    if line_error <= flat_error and width >= cfg.get('flat_min_width',.04):
        row.update(kind='wall_candidate' if width >= cfg.get('wall_min_width',.55) else 'flat_board_candidate',
                   reason='linear_surface')
        return row
    if width > 2*cfg.get('round_radius_max',.25)+cfg.get('cluster_gap',.045):
        row.update(kind='other',reason='oversize_non_round')
        return row
    matrix = np.column_stack((2*q[:,0],2*q[:,1],np.ones(len(q))))
    fit,unused,rank,unused_s = np.linalg.lstsq(matrix,np.sum(q*q,axis=1),rcond=-1)
    squared = float(fit[2]+np.dot(fit[:2],fit[:2]))
    if rank < 3 or not _isfinite(squared) or squared <= 0:
        row['reason'] = 'circle_fit_degenerate'
        return row
    center,radius = fit[:2],math.sqrt(squared)
    radial = np.sqrt(np.sum((q-center)**2,axis=1))
    error = float(np.sqrt(np.mean((radial-radius)**2)))
    angles = np.unwrap(np.arctan2(q[:,1]-center[1],q[:,0]-center[0]))
    arc = math.degrees(float(np.ptp(angles)))
    row.update(radius_m=radius,fit_error_m=error,arc_deg=arc)
    # A box corner can also have a small circle residual. Compare its two faces.
    if error > .0015:
        for fraction in (.33,.50,.67):
            split = int(len(q)*fraction)
            if min(split,len(q)-split)<3:
                continue
            axes,errors = [],[]
            for part in (q[:split],q[split:]):
                d = part-np.mean(part,axis=0)
                vals,vecs = np.linalg.eigh(np.dot(d.T,d)/len(d))
                axes.append(vecs[:,1])
                errors.append(math.sqrt(max(0,float(vals[0]))))
            bend = math.degrees(math.acos(min(1.,abs(float(np.dot(axes[0],axes[1]))))))
            if bend >= 25 and max(errors) <= flat_error and error > 2*max(errors):
                row.update(kind='corner_candidate',reason='two_flat_faces')
                return row
    if error > cfg.get('round_fit_error',.008):
        row.update(kind='other',reason='poor_circle_fit')
    elif arc < cfg.get('round_min_arc_deg',40):
        row['reason'] = 'arc_too_short'
    elif radius < cfg.get('round_radius_min',.025):
        row.update(kind='thin_post_candidate',reason='below_target_radius')
    elif radius > cfg.get('round_radius_max',.25):
        row.update(kind='other',reason='above_target_radius')
    else:
        c = origin+center
        row.update(kind='round_candidate',reason='compact_curved_cross_section',motion_target=True,
                   center=(float(c[0]),float(c[1])))
    return row


def round_scan_clusters(ranges, angle_min, increment, range_min, range_max, cfg, diagnostics=None, motion_points=None):
    """Shared clustering for line rejection or legacy circle fit; no semantic labels."""
    groups, current = [], []
    gap = cfg.get('cluster_gap',.045)
    for i,r in enumerate(ranges):
        if not _isfinite(r) or not range_min <= r < range_max:
            if current:
                groups.append(current)
                current = []
            continue
        a = angle_min+i*increment
        p = (r*math.cos(a),r*math.sin(a))
        if current and distance(current[-1],p) > gap+2*r*abs(increment):
            groups.append(current)
            current = []
        current.append(p)
    if current:
        groups.append(current)
    # A full-circle scan can split an object at its first/last beam.
    if (len(groups)>1 and abs(abs(increment)*len(ranges)-2*math.pi) < 2*abs(increment) and
            _isfinite(ranges[0]) and range_min <= ranges[0] < range_max and
            _isfinite(ranges[-1]) and range_min <= ranges[-1] < range_max and
            distance(groups[-1][-1],groups[0][0]) <= gap+2*min(ranges[0],ranges[-1])*abs(increment)):
        groups[0] = groups[-1]+groups[0]
        groups.pop()
    if len(groups) > cfg.get('shape_max_clusters',128):
        raise ValueError('scan shape complexity exceeded')
    results = []
    for points in groups:
        row = classify_scan_cluster(points,cfg)
        if diagnostics is not None:
            diagnostics.append(row)
        if motion_points is not None and row['motion_target']:
            motion_points.extend(points)
        if row['kind'] == 'round_candidate':
            results.append(row)
    return results


class Scan:
    """Full LaserScan rays in a fixed local-world snapshot; no nearest-only JSON.

    FREE requires a ray to have seen beyond the query. Occlusion/no ray is
    UNKNOWN, not FREE. +inf is a valid no-return up to range_max, NaN/0 are not.
    """
    def __init__(self, ranges, angle_min, increment, range_min, range_max,
                 vehicle_pose, extrinsic, stamp):
        if (not all(_isfinite(v) for v in (angle_min,increment,range_min,range_max,stamp)) or
                not increment or not 0 <= range_min < range_max or len(ranges) < 2 or
                not all(_isfinite(v) for v in tuple(vehicle_pose)+(extrinsic['x'],extrinsic['y'],extrinsic['yaw'],extrinsic['range_cap'])) or
                extrinsic['range_cap'] <= range_min):
            raise ValueError('invalid scan geometry')
        self.ranges = list(ranges)
        self.angle_min, self.increment = angle_min, increment
        self.range_min = range_min
        self.range_max = min(range_max, extrinsic['range_cap'])
        self.pose = tuple(world(vehicle_pose, (extrinsic['x'], extrinsic['y']))) + \
                    (wrap(vehicle_pose[2]+extrinsic['yaw']),)
        self.stamp = stamp
        self.obstacles = []
        self.valid_rays = 0
        for i, r in enumerate(self.ranges):
            if r >= self.range_min and (_isfinite(r) or r == _INF):
                self.valid_rays += 1
            if _isfinite(r) and self.range_min <= r < self.range_max:
                a = angle_min+i*increment
                self.obstacles.append(world(self.pose, (r*math.cos(a), r*math.sin(a))))
        self.shape_filter = extrinsic.get('shape_filter',False)
        self.shape_mode = extrinsic.get('shape_mode','line_reject')
        self.round_clusters = []
        self.cluster_diagnostics = []
        self.motion_obstacles = self.obstacles
        if self.shape_filter:
            motion_points = []
            self.round_clusters = round_scan_clusters(self.ranges,angle_min,increment,
                                                      self.range_min,self.range_max,extrinsic,self.cluster_diagnostics,motion_points)
            self.cluster_diagnostics.sort(key=lambda row: math.hypot(*row['center']) if 'center' in row else _INF)
            self.motion_obstacles = []
            if self.shape_mode == 'line_reject':
                self.motion_obstacles = [world(self.pose,p) for p in motion_points]
            for cluster in self.round_clusters:
                x,y = cluster['center']
                radius = cluster['radius_m']
                # Use the fitted disk perimeter, not only its visible front arc.
                self.motion_obstacles.append(world(self.pose,(x,y)))
                self.motion_obstacles.extend(world(self.pose,(x+radius*math.cos(i*math.pi/12),
                                                               y+radius*math.sin(i*math.pi/12))) for i in range(24))

    def classify(self, point, margin=0.02):
        x, y = local(self.pose, point)
        d = math.hypot(x, y)
        if d < self.range_min:
            return 'UNKNOWN'
        a = math.atan2(y, x)
        indices = [int(round((a+k*2*math.pi-self.angle_min)/self.increment)) for k in (-1,0,1)]
        indices = [i for i in indices if 0 <= i < len(self.ranges)]
        if not indices or d+margin > self.range_max:
            return 'UNKNOWN'
        r = self.ranges[indices[0]]
        if math.isnan(r) or r < self.range_min or r == -_INF:
            return 'UNKNOWN'
        if abs(r-d) <= margin:
            return 'OCCUPIED'
        return 'FREE' if min(r, self.range_max) > d+margin else 'UNKNOWN'

    def coverage(self, points):
        return sum(self.classify(p) == 'FREE' for p in points)/float(max(1, len(points)))

    def evidence(self, point, margin=0.02):
        """Explain a queried ray only on a stop; do not burden every sweep point."""
        x,y = local(self.pose,point)
        d,a = math.hypot(x,y),math.atan2(y,x)
        indices = [int(round((a+k*2*math.pi-self.angle_min)/self.increment)) for k in (-1,0,1)]
        indices = [i for i in indices if 0 <= i < len(self.ranges)]
        i = indices[0] if indices else None
        r = self.ranges[i] if i is not None else None
        if d < self.range_min:
            cause = 'inside_min_range'
        elif i is None:
            cause = 'outside_scan_angles'
        elif d+margin > self.range_max:
            cause = 'beyond_range_cap'
        elif math.isnan(r) or r < self.range_min or r == -_INF:
            cause = 'invalid_ray'
        elif abs(r-d) <= margin:
            cause = 'echo_at_query'
        elif min(r,self.range_max) <= d+margin:
            cause = 'occluded'
        else:
            cause = 'clear_ray'
        return dict(classification=self.classify(point,margin),cause=cause,
                    beam_index=i,beam_angle_deg=math.degrees(wrap(self.angle_min+i*self.increment)) if i is not None else None,
                    range_m=r if r is not None and _isfinite(r) else None,
                    query_distance_m=d,valid_rays=self.valid_rays,total_rays=len(self.ranges))

