# -*- coding: utf-8 -*-
"""One-shot P4 forward entry state machine; no ROS or hardware imports."""
from __future__ import division
import math


LIMITS = {
    'countdown_s': (2, 10, False), 'prepare_speed_raw': (1, 30, True),
    'prepare_steering_raw': (-22, 22, True), 'prepare_s': (.05, 5, False),
    'align_speed_raw': (1, 30, True), 'align_timeout_s': (1, 60, False),
    'side_align_timeout_s': (1, 60, False), 'bottom_seen_frames': (3, 20, True),
    'final_speed_raw': (1, 30, True), 'final_s': (.05, 3, False),
    'search_timeout_s': (.5, 15, False), 'seen_frames': (3, 20, True),
    'lost_frames': (3, 20, True), 'lost_s': (.2, 1, False),
    'verify_timeout_s': (.3, 3, False), 'camera_timeout_s': (.1, .8, False),
    'control_gap_s': (.1, .25, False), 'steering_limit_raw': (1, 22, True),
    'lookahead_m': (.1, 1, False), 'position_gain': (1, 100, False),
    'heading_gain': (0, 50, False), 'bottom_weight': (0, 1, False),
    'bottom_near_row': (.65, .98, False), 'near_frames': (2, 10, True),
    'near_motion_pixels': (1, 30, False), 'near_recent_s': (.1, 1, False),
    'bay_width_m': (.25, .8, False), 'width_tolerance_m': (.02, .12, False),
    'max_heading_deg': (5, 45, False), 'parallel_tolerance_deg': (2, 15, False),
    'min_side_m': (.1, .5, False), 'min_bottom_m': (.1, .5, False),
    'tracking_min_side_m': (.02, .08, False),
    'paint_support': (.5, 1, False), 'white_v_min': (100, 250, True),
    'white_s_max': (10, 100, True), 'target_jump_m': (.03, .3, False),
    'target_heading_jump_deg': (3, 25, False),
}


def validate(cfg):
    if set(cfg) != set(LIMITS):
        raise ValueError('config keys missing/unknown: %s' % sorted(set(cfg) ^ set(LIMITS)))
    result = {}
    for key, (lo, hi, integer) in LIMITS.items():
        v = cfg[key]
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise ValueError(key + ' must be numeric')
        if math.isnan(v) or math.isinf(v) or not lo <= v <= hi or (integer and int(v) != v):
            raise ValueError('%s outside [%s, %s]' % (key, lo, hi))
        result[key] = int(v) if integer else float(v)
    if result['verify_timeout_s'] <= result['lost_s']:
        raise ValueError('verify_timeout_s must exceed lost_s')
    if abs(result['prepare_steering_raw']) > result['steering_limit_raw']:
        raise ValueError('prepare steering exceeds steering limit')
    return result


class ForwardRun(object):
    def __init__(self, cfg):
        self.cfg = validate(cfg)
        self.started = self.last_tick = self.stamp = self.received = None
        self.phase, self.reason = 'READY', 'READY'
        self.finished = self.seen = False
        self.seen_count = self.missing_count = self.near_count = 0
        self.stop_candidate_since = None
        self.deadline = 0.
        self.scene = {}
        self.latest_scene = {}
        self.last_steering = self.frozen_steering = 0
        self.near_armed = False
        self.first_bottom_row = self.last_near_at = None
        self.last_target = None
        self.last_sides_at = None
        self.bottom_seen_count = 0
        self.bottom_candidate_row = self.bottom_candidate_y = None

    def start(self, now):
        self.started = self.last_tick = now
        self.phase = self.reason = 'PREPARE'
        self.deadline = now + self.cfg['prepare_s']

    def abort(self, reason):
        self.finished, self.reason, self.phase = True, reason, 'FINISHED'
        return 0, 0

    def observe_scene(self, scene):
        self.latest_scene = dict(scene)

    def steering(self, scene, use_bottom=True):
        c = self.cfg
        error = scene['center_y_m']
        if use_bottom and scene.get('bottom_y_m') is not None:
            error = ((1-c['bottom_weight'])*error +
                     c['bottom_weight']*scene['bottom_y_m'])
        raw = (c['position_gain']*math.atan2(error, c['lookahead_m']) +
               c['heading_gain']*scene['heading_rad'])
        return int(round(max(-c['steering_limit_raw'], min(c['steering_limit_raw'], raw))))

    def same_target(self, scene):
        if self.last_target is None:
            return True
        return (abs(scene['center_y_m']-self.last_target[0]) <= self.cfg['target_jump_m'] and
                abs(scene['heading_rad']-self.last_target[1]) <=
                math.radians(self.cfg['target_heading_jump_deg']))

    def observe(self, count, stamp, received, both_curved=False):
        if self.finished or (self.stamp is not None and stamp <= self.stamp):
            return
        self.stamp, self.received = stamp, received
        self.scene = s = dict(self.latest_scene)
        if self.started is not None and s.get('image_ok') is False:
            self.abort('IMAGE_INVALID')
            return
        if self.phase not in ('SEARCH', 'SIDE_ALIGN', 'ALIGN', 'VERIFY'):
            return
        if received+1e-9 >= self.deadline:
            self.abort({'SEARCH': 'SEARCH_TIMEOUT', 'SIDE_ALIGN': 'SIDE_ALIGN_TIMEOUT', 'ALIGN': 'ALIGN_TIMEOUT',
                        'VERIFY': 'DISAPPEARANCE_UNCONFIRMED'}[self.phase])
            return
        sides, bottom = s.get('sides_valid') is True, s.get('bottom_valid') is True
        if self.phase == 'SEARCH':
            if (sides and s.get('visible_side_m', 0) >= self.cfg['min_side_m'] and
                    self.same_target(s)):
                self.seen_count += 1
                self.last_target = (s['center_y_m'], s['heading_rad'])
            else:
                self.seen_count = 0
                self.last_target = None
            if self.seen_count >= self.cfg['seen_frames']:
                self.seen = True
                self.phase = self.reason = 'SIDE_ALIGN'
                self.deadline = received + self.cfg['side_align_timeout_s']
                self.last_steering = self.steering(s, use_bottom=False)
                self.last_sides_at = received
            return
        if self.phase == 'SIDE_ALIGN':
            if not sides or not self.same_target(s):
                self.abort('SIDE_GEOMETRY_LOST_OR_TARGET_JUMP')
                return
            self.last_target = (s['center_y_m'], s['heading_rad'])
            self.last_sides_at = received
            self.last_steering = self.steering(s, use_bottom=False)
            if bottom:
                if (self.bottom_candidate_y is not None and
                        abs(s['bottom_y_m']-self.bottom_candidate_y) > self.cfg['target_jump_m']):
                    self.bottom_seen_count = 0
                if self.bottom_seen_count == 0:
                    self.bottom_candidate_row = s['bottom_row']
                self.bottom_seen_count += 1
                self.bottom_candidate_y = s['bottom_y_m']
            else:
                self.bottom_seen_count = 0
                self.bottom_candidate_row = self.bottom_candidate_y = None
            if self.bottom_seen_count < self.cfg['bottom_seen_frames']:
                return
            self.phase = self.reason = 'ALIGN'
            self.deadline = received + self.cfg['align_timeout_s']
            self.first_bottom_row = self.bottom_candidate_row
            # Continue below to apply bottom correction and near-edge evidence.
        # At the image edge the terminating U can leave the frame as a whole.
        # Loss of both rails is allowed ONLY after stable near-edge motion;
        # an observed different bay is still a hard stop.
        tracked_bottom = (s.get('tracking_bottom') is True and self.last_sides_at is not None and
                          received-self.last_sides_at <= self.cfg['near_recent_s'])
        if (sides and not self.same_target(s)) or (bottom and not sides and not tracked_bottom):
            self.abort('GEOMETRY_LOST_OR_TARGET_JUMP')
            return
        if bottom:
            if self.phase == 'VERIFY':
                self.abort('BOTTOM_REAPPEARED_DURING_CONFIRMATION')
                return
            if sides:
                self.last_target = (s['center_y_m'], s['heading_rad'])
                self.last_steering = self.steering(s)
                self.last_sides_at = received
            if self.first_bottom_row is None:
                self.first_bottom_row = s['bottom_row']
            motion = s['bottom_row']-self.first_bottom_row
            if s['bottom_row'] >= self.cfg['bottom_near_row']:
                self.near_count += 1
                self.last_near_at = received
                if (self.near_count >= self.cfg['near_frames'] and
                        motion * s['image_height'] >= self.cfg['near_motion_pixels']):
                    self.near_armed = True
            else:
                self.near_count, self.near_armed = 0, False
            return
        if self.phase == 'ALIGN':
            if (not self.near_armed or self.last_near_at is None or
                    received-self.last_near_at > self.cfg['near_recent_s']):
                self.abort('BOTTOM_LOST_BEFORE_IMAGE_EDGE')
                return
            self.phase = self.reason = 'VERIFY'
            self.stop_candidate_since = received
            self.frozen_steering = self.last_steering
            self.missing_count = 1
            self.deadline = received + self.cfg['verify_timeout_s']
        else:
            self.missing_count += 1

    def tick(self, now, ros_now):
        if self.finished:
            return 0, 0
        if self.started is None:
            return 0, 0
        dt = now-self.last_tick
        if not 0 <= dt <= self.cfg['control_gap_s']:
            return self.abort('CONTROL_GAP')
        self.last_tick = now
        if (self.stamp is None or self.received is None or
                not 0 <= ros_now-self.stamp <= self.cfg['camera_timeout_s'] or
                not 0 <= now-self.received <= self.cfg['camera_timeout_s']):
            return self.abort('CAMERA_TIMEOUT')
        if self.phase == 'PREPARE':
            if now+1e-9 < self.deadline:
                return self.cfg['prepare_speed_raw'], self.cfg['prepare_steering_raw']
            self.phase = self.reason = 'SEARCH'
            self.deadline = now+self.cfg['search_timeout_s']
            self.seen_count, self.last_target = 0, None
            return 0, 0
        if self.phase == 'SEARCH':
            if now+1e-9 >= self.deadline:
                return self.abort('SEARCH_TIMEOUT')
            return 0, 0
        if self.phase in ('SIDE_ALIGN', 'ALIGN'):
            if now+1e-9 >= self.deadline:
                return self.abort('SIDE_ALIGN_TIMEOUT' if self.phase == 'SIDE_ALIGN' else 'ALIGN_TIMEOUT')
            return self.cfg['align_speed_raw'], self.last_steering
        if self.phase == 'VERIFY':
            if now+1e-9 >= self.deadline:
                return self.abort('DISAPPEARANCE_UNCONFIRMED')
            if (self.missing_count >= self.cfg['lost_frames'] and
                    self.received-self.stop_candidate_since+1e-9 >= self.cfg['lost_s']):
                self.phase = self.reason = 'FINAL'
                self.deadline = now+self.cfg['final_s']
                return self.cfg['final_speed_raw'], self.frozen_steering
            return 0, 0
        if self.phase == 'FINAL':
            if now+1e-9 >= self.deadline:
                return self.abort('P4_COMPLETE')
            return self.cfg['final_speed_raw'], self.frozen_steering
        return self.abort('INVALID_PHASE')

    def diagnostics(self, now):
        return dict(sides_valid=self.scene.get('sides_valid', False),
                    bottom_valid=self.scene.get('bottom_valid', False),
                    bottom_midpoint_m=self.scene.get('bottom_midpoint_m'),
                    bottom_row=self.scene.get('bottom_row'),
                    center_y_m=self.scene.get('center_y_m'),
                    heading_deg=self.scene.get('heading_deg'),
                    guidance='SIDES' if self.phase == 'SIDE_ALIGN' else 'BOTTOM' if self.phase == 'ALIGN' else self.phase,
                    bottom_seen_count=self.bottom_seen_count,
                    seen_count=self.seen_count, near_armed=self.near_armed,
                    missing_count=self.missing_count, frozen_steering=self.frozen_steering)

    def format_diagnostics(self, now):
        return ('P4 guidance=%s center_y=%s m heading=%s deg bottom=%s row=%s '
                'bottom_seen=%d/%d '
                'seen=%d/%d near_armed=%s missing=%d/%d frozen_steering=%d' %
                (self.phase, self.scene.get('center_y_m'), self.scene.get('heading_deg'),
                 self.scene.get('bottom_midpoint_m'), self.scene.get('bottom_row'),
                 self.bottom_seen_count, self.cfg['bottom_seen_frames'],
                 self.seen_count, self.cfg['seen_frames'], self.near_armed,
                 self.missing_count, self.cfg['lost_frames'], self.frozen_steering))
