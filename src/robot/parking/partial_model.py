"""Visibility-aware two-bay fitting for stationary front-camera inspection.

Image-agreement scores are not calibrated probabilities. Unobserved boundaries
remain inferred. No neural-network weights or driving commands are used here.
"""
from __future__ import division
import itertools
import math
import time
import cv2
import numpy as np


class PartialBayModel(object):
    def __init__(self, cfg, metric, ppm):
        parameters = cfg.get('parking_partial', {})
        self.width = float(parameters.get('bay_width_m', .38))
        self.depth = float(parameters.get('bay_depth_m', .45))
        self.ppm = float(ppm)
        if min(self.width, self.depth, self.ppm) <= 0:
            raise ValueError('parking dimensions and pixels per metre must be positive')
        self.metric = metric
        self.origin = np.asarray(metric(0, 0), dtype=float)
        basis = np.column_stack((np.asarray(metric(1, 0)) - self.origin,
                                 np.asarray(metric(0, 1)) - self.origin))
        self.inverse = np.linalg.inv(basis)

    def _prepare(self, white, valid):
        if white.ndim != 2 or white.dtype != np.uint8:
            raise ValueError('parking white mask must be uint8 single channel')
        if valid is None or valid.shape != white.shape:
            raise ValueError('parking visibility mask must match white mask')
        self.white = np.where((white > 0) & (valid > 0), 255, 0).astype(np.uint8)
        self.valid = valid > 0
        self.height, self.image_width = white.shape
        self.distance = cv2.distanceTransform(255 - self.white, cv2.DIST_L2, 3) / self.ppm

    def _sample(self, points):
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        pixels = np.rint(np.dot(points - self.origin, self.inverse.T)).astype(int)
        inside = ((pixels[:, 0] >= 0) & (pixels[:, 0] < self.image_width) &
                  (pixels[:, 1] >= 0) & (pixels[:, 1] < self.height))
        u = np.clip(pixels[:, 0], 0, self.image_width - 1)
        v = np.clip(pixels[:, 1], 0, self.height - 1)
        return self.distance[v, u], inside & self.valid[v, u]

    def _part(self, a, b):
        a, b = np.asarray(a), np.asarray(b)
        length = float(np.linalg.norm(b - a))
        count = max(8, int(math.ceil(length / .015)) + 1)
        points = a + np.linspace(0, 1, count)[:, None] * (b - a)
        distances, visible = self._sample(points)
        agreement = np.clip(1 - distances / .035, 0, 1)
        visible_agreement = np.sort(agreement[visible])
        raw_mean = float(visible_agreement.mean()) if visible.any() else 0.
        retained = max(1, int(math.ceil(len(visible_agreement) * .70)))
        mean = float(visible_agreement[-retained:].mean()) if visible.any() else 0.
        normal = np.array([-(b - a)[1], (b - a)[0]]) / length
        offsets = []
        for sign in (-1, 1):
            other, other_visible = self._sample(points + sign * .05 * normal)
            usable = visible & other_visible
            if usable.any():
                offsets.extend(np.clip(1 - other[usable] / .035, 0, 1).tolist())
        contrast = max(0., mean - float(np.mean(offsets))) if offsets else 0.
        good = visible & (distances <= .025)
        visible_length = float(visible.mean()) * length
        matched_length = float(good.mean()) * length
        segments, start = [], None
        for index, matched in enumerate(list(good) + [False]):
            if matched and start is None:
                start = index
            if not matched and start is not None:
                if index - start >= 2:
                    segments.append([points[start].tolist(), points[index - 1].tolist()])
                start = None
        supported = mean >= .52 and contrast >= .12 and matched_length >= .10
        quality = mean * min(1., visible_length / .18) * min(1., contrast / .30)
        return dict(agreement=mean, raw_agreement=raw_mean, contrast=contrast, visible_length_m=visible_length,
                    matched_length_m=matched_length, visible_fraction=float(visible.mean()),
                    supported=bool(supported), quality=float(quality), segments=segments)

    def _dash_runs(self, mouth, along):
        # A 1-D entrance profile avoids joining all dashes into one component
        # when paint touches a transverse separator. No fixed dash gap is assumed.
        length = 2 * self.width
        count = max(2, int(math.ceil(length / .005)) + 1)
        locations = np.linspace(0, length, count)
        distances, visible = self._sample(mouth + locations[:, None] * along)
        painted = visible & (distances <= .009)
        runs, start = [], None
        for index, match in enumerate(list(painted) + [False]):
            if match and start is None:
                start = index
            if not match and start is not None:
                bounded = start > 0 and index < count
                bounded = bounded and visible[start - 1] and visible[index]
                run_length = (index - start) * length / (count - 1)
                if bounded and .030 <= run_length <= .085:
                    runs.append((mouth + .5 * (locations[start] +
                                              locations[index - 1]) * along).tolist())
                start = None
        return runs

    def _fit(self, pose):
        mouth = np.asarray(pose[:2], dtype=float)
        into = np.asarray(pose[2:4], dtype=float)
        if not np.isfinite(pose).all() or np.linalg.norm(into) < 1e-6:
            raise ValueError('parking pose needs a finite nonzero direction')
        into = into / np.linalg.norm(into)
        along = np.array([-into[1], into[0]])
        if along[0] < 0:
            along *= -1
        entrances = [mouth + i * self.width * along for i in range(3)]
        sides = [self._part(p, p + self.depth * into) for p in entrances]
        back = self._part(entrances[0] + self.depth * into, entrances[2] + self.depth * into)
        dashes = self._dash_runs(mouth, along)
        anchors = 0
        for point, side in zip(entrances, sides):
            if not side['supported']:
                continue
            distances, seen = self._sample([point - .06 * into, point, point + .06 * into])
            if seen.all() and distances[0] > .04 and max(distances[1:]) < .025:
                anchors += 1
        depth_supported = back['supported'] or len(dashes) >= 2 or anchors >= 2
        quality = sorted([s['quality'] for s in sides], reverse=True)
        # Independent evidence from the third separator distinguishes a full pair
        # from a shifted explanation containing only one supported bay.
        score = (.46 * np.mean(quality[:2]) + .14 * quality[2] + .18 * back['quality'] +
                 .12 * min(len(dashes) / 3., 1.) + .10 * min(anchors / 2., 1.))
        score -= .08 * sum(s['visible_fraction'] * (1 - s['raw_agreement']) for s in sides) / 3.
        if back['visible_fraction'] > .5 and back['raw_agreement'] < .2:
            score -= .05 * back['visible_fraction']
        score = float(np.clip(score, 0, 1))
        pair_supported = sides[0]['supported'] and sides[2]['supported']
        candidates = []
        if depth_supported and sum(s['supported'] for s in sides) >= 2 and score >= .52:
            for i in range(2):
                if not (pair_supported or (sides[i]['supported'] and sides[i + 1]['supported'])):
                    continue
                centre = mouth + (i + .5) * self.width * along + .5 * self.depth * into
                if centre[0] <= 0 or abs(centre[1]) < .05:
                    continue
                candidates.append(dict(
                    x=float(centre[0]), y=float(centre[1]), yaw=float(math.atan2(into[1], into[0])),
                    length=self.depth, width=self.width, kind='perpendicular',
                    evidence='partial_template', model_score=score,
                    geometry_status='supported_partial', geometry_inferred=True,
                    bottom_inferred=not back['supported'],
                    adjacent_group='partial_pair' if pair_supported else None,
                    observed_sides=sides[i]['segments'] + sides[i + 1]['segments'],
                    observed_back_segments=back['segments'], entrance_dashes=dashes))
        parts = dict(('side_%d' % i, s) for i, s in enumerate(sides))
        parts['back'] = back
        polygon = [entrances[0], entrances[2], entrances[2] + self.depth * into,
                   entrances[0] + self.depth * into]
        return dict(pose=mouth.tolist() + into.tolist(), score=score, candidates=candidates,
                    depth_supported=bool(depth_supported),
                    pair_identity_supported=bool(pair_supported and len(candidates) == 2),
                    mouth_anchors=anchors, dash_count=len(dashes), parts=parts,
                    polygon=[p.tolist() for p in polygon],
                    visible_fraction=float(np.mean([p['visible_fraction'] for p in parts.values()])))

    def evaluate(self, white, valid, pose):
        self._prepare(white, valid)
        return self._fit(pose)

    def _entrance_guides(self):
        """Use multiple short painted dashes to anchor the entrance line.

        Neither a dash's phase in the seven-dash pattern nor an obstacle's
        outline supplies a bay identity. Side-line intersections supply that.
        """
        result = cv2.findContours(self.white.copy(), cv2.RETR_EXTERNAL,
                                  cv2.CHAIN_APPROX_SIMPLE)
        components = []
        for contour in result[-2]:
            if cv2.contourArea(contour) < max(4, .00010 * self.ppm ** 2):
                continue
            rectangle = cv2.minAreaRect(contour)
            long_px, short_px = max(rectangle[1]), min(rectangle[1])
            if not (.030 <= long_px / self.ppm <= .085 and
                    .003 <= short_px / self.ppm <= .035 and long_px >= 1.8 * short_px):
                continue
            corners = cv2.boxPoints(rectangle)
            edges = [corners[(i + 1) % 4] - corners[i] for i in range(4)]
            edge = max(edges, key=lambda p: np.linalg.norm(p))
            centre = np.asarray(self.metric(*rectangle[0]))
            tip = np.asarray(self.metric(*(np.asarray(rectangle[0]) + edge)))
            direction = tip - centre
            direction /= np.linalg.norm(direction)
            if direction[0] < 0:
                direction = -direction
            if direction[0] < .60 or abs(centre[1]) < .05:
                continue
            # Reject regions at the field-of-view boundary: their apparent
            # length could be a clipped solid line rather than a complete dash.
            outside = np.asarray([self.metric(*(np.asarray(rectangle[0]) - .75 * edge)),
                                  self.metric(*(np.asarray(rectangle[0]) + .75 * edge))])
            distances, seen = self._sample(outside)
            if not seen.all() or min(distances) < .008:
                continue
            components.append((centre, direction))
        guides = []
        for centre, direction in components[:80]:
            normal = np.array([-direction[1], direction[0]])
            members = [p for p, axis in components[:80]
                       if (p[1] * centre[1] > 0 and
                           abs(np.dot(p - centre, normal)) <= .022 and
                           np.dot(axis, direction) >= math.cos(math.radians(15)))]
            if len(members) < 2:
                continue
            offsets = [float(np.dot(p, direction)) for p in members]
            if max(offsets) - min(offsets) < .10:
                continue
            anchor = np.mean(members, axis=0)
            if any(abs(np.dot(anchor - np.asarray(g['point']), normal)) < .025 and
                   np.dot(direction, g['direction']) > math.cos(math.radians(8))
                   for g in guides):
                continue
            guides.append(dict(point=anchor.tolist(), direction=direction.tolist(),
                               dash_centres=[p.tolist() for p in members],
                               count=len(members), span_m=max(offsets) - min(offsets)))
        guides.sort(key=lambda g: (-g['count'], -g['span_m']))
        return guides[:4]

    def _seeds(self):
        self.entrance_guides = self._entrance_guides()
        lines = cv2.HoughLinesP(self.white, 1, np.pi / 180., 16,
                               minLineLength=max(8, int(.07 * self.ppm)),
                               maxLineGap=max(2, int(.025 * self.ppm)))
        raw = [] if lines is None else list(lines.reshape(-1, 4))
        raw.sort(key=lambda l: -((l[2] - l[0]) ** 2 + (l[3] - l[1]) ** 2))
        groups = []
        for u0, v0, u1, v1 in raw[:64]:
            a = np.asarray(self.metric(float(u0), float(v0)))
            b = np.asarray(self.metric(float(u1), float(v1)))
            length = float(np.linalg.norm(b - a))
            middle = .5 * (a + b)
            if not .07 <= length <= self.depth + .20 or abs(middle[1]) < .05:
                continue
            into = (b - a) / length
            if abs(into[1]) < .45:
                continue
            if into[1] * middle[1] < 0:
                into = -into
            duplicate = False
            for other in groups:
                normal = np.array([-into[1], into[0]])
                if (np.dot(into, other[2]) > .99 and
                        abs(np.dot(middle - other[3], normal)) < .015 and
                        np.linalg.norm(middle - other[3]) < .22):
                    duplicate = True
                    break
            if not duplicate:
                groups.append((a, b, into, middle))
            if len(groups) >= 32:
                break
        poses, keys = [], set()
        for first, second in itertools.combinations(groups, 2):
            if np.dot(first[2], second[2]) < math.cos(math.radians(12)):
                continue
            into = first[2] + second[2]
            into /= np.linalg.norm(into)
            along = np.array([-into[1], into[0]])
            if along[0] < 0:
                along = -along
            ordered = sorted((first, second), key=lambda g: np.dot(g[3], along))
            separation = float(np.dot(ordered[1][3] - ordered[0][3], along))
            gap = int(round(separation / self.width))
            if gap not in (1, 2) or abs(separation - gap * self.width) > .08:
                continue
            normal_origin = .5 * (np.dot(ordered[0][3], along) +
                                  np.dot(ordered[1][3], along) - gap * self.width)
            endpoints = [np.dot(p, into) for g in ordered for p in g[:2]]
            depths = endpoints + [d - self.depth for d in endpoints]
            for first_index in range(3 - gap):
                base = (normal_origin - first_index * self.width) * along
                entrance_depths = []
                for guide in self.entrance_guides:
                    if guide['point'][1] * into[1] <= 0:
                        continue
                    guide_normal = np.array([-guide['direction'][1], guide['direction'][0]])
                    denominator = np.dot(guide_normal, into)
                    if abs(denominator) < math.cos(math.radians(20)):
                        continue
                    entrance_depths.append(float(np.dot(
                        guide_normal, np.asarray(guide['point']) - base) / denominator))
                for offset in entrance_depths + depths:
                    mouth = ((normal_origin - first_index * self.width) * along + offset * into)
                    if mouth[0] < -.3 or abs(mouth[1]) < .05 or mouth[1] * into[1] <= 0:
                        continue
                    key = (int(round(mouth[0] / .02)), int(round(mouth[1] / .02)),
                           int(round(math.atan2(into[1], into[0]) / .05)))
                    if key in keys:
                        continue
                    keys.add(key)
                    poses.append(mouth.tolist() + into.tolist())
                    if len(poses) >= 192:
                        return poses, len(raw), len(groups)
        return poses, len(raw), len(groups)

    def _rank_poses(self, poses):
        """Score all poses in array operations; decode geometry only for finalists.

        This uses the same samples and thresholds as _fit, but avoids thousands
        of tiny Python/Numpy calls and observed-segment allocations per frame.
        """
        if not poses:
            return []
        poses = np.asarray(poses, dtype=float)
        mouth = poses[:, :2]
        into = poses[:, 2:4]
        into = into / np.linalg.norm(into, axis=1)[:, None]
        along = np.column_stack((-into[:, 1], into[:, 0]))
        along[along[:, 0] < 0] *= -1
        entrances = mouth[:, None, :] + np.arange(3)[None, :, None] * self.width * along[:, None, :]
        starts = np.concatenate((entrances, (entrances[:, :1] + self.depth * into[:, None])), axis=1)
        vectors = np.concatenate((np.repeat(self.depth * into[:, None], 3, axis=1),
                                  2 * self.width * along[:, None]), axis=1)
        lengths = np.array([self.depth] * 3 + [2 * self.width])
        counts = np.maximum(8, np.ceil(lengths / .015).astype(int) + 1)
        steps = np.arange(int(counts.max()))
        active = steps[None, :] < counts[:, None]
        fractions = np.minimum(1., steps[None, :] / (counts[:, None] - 1))
        points = starts[:, :, None, :] + fractions[None, :, :, None] * vectors[:, :, None, :]
        shape = points.shape[:-1]
        distance, seen = self._sample(points)
        distance, seen = distance.reshape(shape), seen.reshape(shape) & active[None]
        agreement = np.clip(1 - distance / .035, 0, 1)
        raw_mean = (agreement * seen).sum(axis=2) / np.maximum(1, seen.sum(axis=2))
        ordered = np.sort(np.where(seen, agreement, -1.), axis=2)
        retained = np.maximum(1, np.ceil(seen.sum(axis=2) * .70).astype(int))
        selected = steps[None, None, :] >= (len(steps) - retained[:, :, None])
        on_mean = np.where(seen.any(axis=2), (ordered * selected).sum(axis=2) / retained, 0.)
        normal = np.stack((-vectors[:, :, 1], vectors[:, :, 0]), axis=2) / lengths[None, :, None]
        off_sum = np.zeros(on_mean.shape)
        off_count = np.zeros(on_mean.shape)
        for sign in (-1, 1):
            other, other_seen = self._sample(points + sign * .05 * normal[:, :, None])
            other, other_seen = other.reshape(shape), other_seen.reshape(shape)
            usable = seen & other_seen
            off_sum += (np.clip(1 - other / .035, 0, 1) * usable).sum(axis=2)
            off_count += usable.sum(axis=2)
        contrast = np.maximum(0., on_mean - off_sum / np.maximum(1, off_count))
        contrast[off_count == 0] = 0
        visible_fraction = seen.sum(axis=2) / counts[None, :]
        matched_length = ((seen & (distance <= .025)).sum(axis=2) /
                          counts[None, :]) * lengths[None, :]
        supported = (on_mean >= .52) & (contrast >= .12) & (matched_length >= .10)
        quality = (on_mean * np.minimum(1., visible_fraction * lengths[None, :] / .18) *
                   np.minimum(1., contrast / .30))
        anchor_points = entrances[:, :, None, :] + np.array([-.06, 0., .06])[None, None, :, None] * into[:, None, None, :]
        anchor_distance, anchor_seen = self._sample(anchor_points)
        anchor_distance = anchor_distance.reshape(-1, 3, 3)
        anchor_seen = anchor_seen.reshape(-1, 3, 3)
        anchors = (supported[:, :3] & anchor_seen.all(axis=2) &
                   (anchor_distance[:, :, 0] > .04) &
                   (anchor_distance[:, :, 1:].max(axis=2) < .025)).sum(axis=1)
        length = 2 * self.width
        dash_samples = max(2, int(math.ceil(length / .005)) + 1)
        locations = np.linspace(0, length, dash_samples)
        dash_distance, dash_seen = self._sample(mouth[:, None, :] + locations[None, :, None] * along[:, None, :])
        dash_distance = dash_distance.reshape(-1, dash_samples)
        dash_seen = dash_seen.reshape(-1, dash_samples)
        painted = dash_seen & (dash_distance <= .009)
        dash_counts = np.zeros(len(poses), dtype=int)
        for index, row in enumerate(painted):
            changes = np.diff(np.concatenate(([0], row.astype(np.int8), [0])))
            beginnings = np.flatnonzero(changes == 1)
            endings = np.flatnonzero(changes == -1)
            for start, end in zip(beginnings, endings):
                if (start > 0 and end < dash_samples and dash_seen[index, start - 1] and
                        dash_seen[index, end] and .030 <= (end - start) * length / (dash_samples - 1) <= .085):
                    dash_counts[index] += 1
        side_quality = np.sort(quality[:, :3], axis=1)
        scores = (.46 * side_quality[:, 1:].mean(axis=1) + .14 * side_quality[:, 0] +
                  .18 * quality[:, 3] + .12 * np.minimum(dash_counts / 3., 1.) +
                  .10 * np.minimum(anchors / 2., 1.))
        scores -= .08 * (visible_fraction[:, :3] * (1 - raw_mean[:, :3])).mean(axis=1)
        missing_back = (visible_fraction[:, 3] > .5) & (raw_mean[:, 3] < .2)
        scores -= .05 * visible_fraction[:, 3] * missing_back
        scores = np.clip(scores, 0, 1)
        depth_supported = supported[:, 3] | (dash_counts >= 2) | (anchors >= 2)
        eligible = depth_supported & (supported[:, :3].sum(axis=1) >= 2) & (scores >= .52)
        return [dict(pose=pose.tolist(), score=float(score), eligible=bool(ok))
                for pose, score, ok in zip(poses, scores, eligible)]

    @staticmethod
    def _same_pose(a, b):
        return (np.linalg.norm(np.asarray(a['pose'][:2]) - b['pose'][:2]) < .10 and
                np.dot(a['pose'][2:], b['pose'][2:]) > math.cos(math.radians(8)))

    def detect(self, white, valid):
        started = time.time()
        model = dict(reason='no_hypothesis', evaluated=0, hypotheses=[],
                     revision='entrance_batch_v2')
        diagnostic = dict(algorithm='partial_model_v1', raw_segments=0,
                          merged_segments=0, accepted=0, model=model)
        if valid is None or valid.shape != white.shape:
            model['reason'] = 'visibility_unavailable'
            return [], diagnostic
        self._prepare(white, valid)
        poses, raw, merged = self._seeds()
        model['entrance_guides'] = self.entrance_guides
        diagnostic.update(raw_segments=raw, merged_segments=merged)
        reports = self._rank_poses(poses)
        reports.sort(key=lambda r: -r['score'])
        seeds = []
        for report in reports:
            if not any(self._same_pose(report, old) for old in seeds):
                seeds.append(report)
            if len(seeds) == 4:
                break
        refinement = []
        for seed in seeds:
            into = np.asarray(seed['pose'][2:])
            along = np.array([-into[1], into[0]])
            for lateral, depth, angle in itertools.product(
                    (-.02, 0., .02), (-.02, 0., .02), (-3., 0., 3.)):
                centre = np.asarray(seed['pose'][:2]) + lateral * along + depth * into
                theta = math.radians(angle)
                rotated = [into[0] * math.cos(theta) - into[1] * math.sin(theta),
                           into[0] * math.sin(theta) + into[1] * math.cos(theta)]
                refinement.append(centre.tolist() + rotated)
        reports.extend(self._rank_poses(refinement))
        reports.sort(key=lambda r: -r['score'])
        model['scored_poses'] = len(reports)
        if not reports:
            model.update(evaluated=0, decoded_poses=0,
                         processing_ms=1000 * (time.time() - started))
            return [], diagnostic
        # Decode supported hypotheses first. Keep unresolved geometry visible
        # when none qualifies; it must never become a confirmed bay by fallback.
        shortlist = []
        ranked = [r for r in reports if r['eligible']] or reports
        for report in ranked:
            if report['score'] < ranked[0]['score'] - .08:
                break
            if not any(self._same_pose(report, old) for old in shortlist):
                shortlist.append(report)
            if len(shortlist) >= 8:
                break
        decoded = [self._fit(r['pose']) for r in shortlist]
        model.update(evaluated=len(reports) + len(decoded), decoded_poses=len(decoded),
                     processing_ms=1000 * (time.time() - started), scoring='batched_v2')
        reports = sorted(decoded, key=lambda r: -r['score'])
        best = next((r for r in reports if r['candidates'] and
                     r['score'] >= reports[0]['score'] - .08), reports[0])
        alternatives = []
        for report in reports:
            if not report['candidates'] or report['score'] < best['score'] - .08:
                continue
            if any(self._same_pose(report, r) for r in [best] + alternatives):
                continue
            alternatives.append(report)
            if len(alternatives) == 2:
                break
        candidates = [dict(c) for c in best['candidates']]
        if alternatives:
            candidates = [c for c in candidates if all(any(
                math.hypot(c['x'] - other['x'], c['y'] - other['y']) < .12
                for other in alternative['candidates']) for alternative in alternatives)]
            for candidate in candidates:
                candidate['adjacent_group'] = None
                candidate['geometry_status'] = 'identity_ambiguous'
        model.update(reason='supported_geometry' if candidates else 'partial_geometry_unresolved',
                     score=best['score'], ambiguous=bool(alternatives),
                     visible_fraction=best['visible_fraction'], parts=best['parts'],
                     dash_count=best['dash_count'], mouth_anchors=best['mouth_anchors'],
                     pair_identity_supported=bool(best['pair_identity_supported'] and not alternatives),
                     hypotheses=[dict(score=r['score'], polygon=r['polygon'],
                                      visible_fraction=r['visible_fraction'],
                                      supported_bays=len(r['candidates']))
                                 for r in [best] + alternatives])
        diagnostic['accepted'] = len(candidates)
        return candidates, diagnostic
