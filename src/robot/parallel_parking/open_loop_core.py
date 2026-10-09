# -*- coding: utf-8 -*-
"""Bounded, one-shot parallel parking experiment. No ROS or hardware imports."""
from __future__ import division
import math
from collections import namedtuple

Stage = namedtuple('Stage', 'name seconds speed steering')
DEFAULTS = dict(side='right', speed_raw=12, first_steering_raw=22,
                forward_speed_raw=12, forward_seconds=0.5, forward_only=False,
                first_seconds=0.5, second_steering_raw=22, second_seconds=0.0,
                repeat_bay_steering_raw=22, repeat_bay_seconds=0.0,
                repeat_counter_steering_raw=22, repeat_counter_seconds=0.0,
                straight_seconds=0.0, pause_seconds=0.7, countdown_seconds=3.0)

SLOT_REFERENCE = dict(reference_offset_m=0.85, slot_spacing_m=0.70,
                      forward_mps_per_raw=0.008, reverse_mps_per_raw=0.007)
SLOT_MOTION = dict(speed_raw=30, position_speed_raw=30)


def finite(value, key, low, high, integer=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError('%s must be a number' % key)
    if math.isnan(value) or math.isinf(value) or not low <= value <= high:
        raise ValueError('%s must be within [%s, %s]' % (key, low, high))
    if integer and int(value) != value:
        raise ValueError('%s must be an integer' % key)
    return int(value) if integer else float(value)


def make_plan(overrides=None):
    values = dict(DEFAULTS)
    overrides = overrides or {}
    unknown = set(overrides) - set(values)
    if unknown:
        raise ValueError('unknown parameters: '+', '.join(sorted(unknown)))
    values.update(overrides)
    if values['side'] not in ('left', 'right'):
        raise ValueError('side must be left or right')
    speed = finite(values['speed_raw'], 'speed_raw', 1, 30, True)
    forward_speed = finite(values['forward_speed_raw'], 'forward_speed_raw', 1, 30, True)
    forward = finite(values['forward_seconds'], 'forward_seconds', 0, 5)
    forward_only = values['forward_only']
    if not isinstance(forward_only, bool):
        raise ValueError('forward_only must be a boolean')
    if forward_only and forward == 0:
        raise ValueError('forward_only requires forward_seconds > 0')
    first = finite(values['first_seconds'], 'first_seconds', 0.05, 5)
    second = finite(values['second_seconds'], 'second_seconds', 0, 5)
    repeat_bay = finite(values['repeat_bay_seconds'], 'repeat_bay_seconds', 0, 5)
    repeat_counter = finite(values['repeat_counter_seconds'], 'repeat_counter_seconds', 0, 5)
    straight = finite(values['straight_seconds'], 'straight_seconds', 0, 5)
    if repeat_bay > 0 and second == 0:
        raise ValueError('repeat_bay_seconds requires second_seconds > 0')
    if repeat_counter > 0 and repeat_bay == 0:
        raise ValueError('repeat_counter_seconds requires repeat_bay_seconds > 0')
    a = finite(values['first_steering_raw'], 'first_steering_raw', 1, 22, True)
    b = finite(values['second_steering_raw'], 'second_steering_raw', 1, 22, True)
    repeat_a = finite(values['repeat_bay_steering_raw'], 'repeat_bay_steering_raw', 1, 22, True)
    repeat_b = finite(values['repeat_counter_steering_raw'], 'repeat_counter_steering_raw', 1, 22, True)
    pause = finite(values['pause_seconds'], 'pause_seconds', 0.3, 3)
    countdown = finite(values['countdown_seconds'], 'countdown_seconds', 2, 10)
    sign = -1 if values['side'] == 'right' else 1
    stages = [Stage('COUNTDOWN', countdown, 0, 0)]
    if forward > 0:
        stages.append(Stage('FORWARD_PREPARE', forward, forward_speed, 0))
        if not forward_only:
            stages.append(Stage('PAUSE_BEFORE_REVERSE', pause, 0, 0))
    if not forward_only:
        stages.append(Stage('REVERSE_INTO_BAY', first, -speed, sign*a))
    if not forward_only and second > 0:
        stages += [Stage('PAUSE', pause, 0, 0),
                   Stage('REVERSE_COUNTERSTEER', second, -speed, -sign*b)]
    if not forward_only and repeat_bay > 0:
        stages += [Stage('PAUSE', pause, 0, 0),
                   Stage('REVERSE_INTO_BAY_2', repeat_bay, -speed, sign*repeat_a)]
    if not forward_only and repeat_counter > 0:
        stages += [Stage('PAUSE', pause, 0, 0),
                   Stage('REVERSE_COUNTERSTEER_2', repeat_counter, -speed, -sign*repeat_b)]
    if not forward_only and straight > 0:
        stages += [Stage('PAUSE', pause, 0, 0),
                   Stage('REVERSE_STRAIGHT', straight, -speed, 0)]
    stages += [Stage('FINAL_STOP', max(0.5, pause), 0, 0)]
    return stages


def slot_distance(slot, reference_offset_m=0.85, slot_spacing_m=0.70):
    """Signed travel from the front axle aligned with P1's front boundary."""
    if slot not in ('P1', 'P2', 'P3'):
        raise ValueError('slot must be P1, P2, or P3')
    offset = finite(reference_offset_m, 'reference_offset_m', -2, 2)
    spacing = finite(slot_spacing_m, 'slot_spacing_m', 0.1, 2)
    return offset - ('P1', 'P2', 'P3').index(slot)*spacing


def make_slot_plan(slot, speed_raw=30, position_speed_raw=30,
                   reference_offset_m=0.85, slot_spacing_m=0.70,
                   forward_mps_per_raw=0.008, reverse_mps_per_raw=0.007,
                   first_seconds=1.6, second_seconds=1.4,
                   pause_seconds=0.7, countdown_seconds=3.0):
    """Reference positioning, then continuous left/right reverse full lock.

    Distances use a command-speed model, not encoder feedback. Positioning
    is bounded to 15 seconds; existing manual stage limits remain unchanged.
    """
    distance = slot_distance(slot, reference_offset_m, slot_spacing_m)
    speed = finite(speed_raw, 'speed_raw', 1, 30, True)
    position_speed = finite(position_speed_raw, 'position_speed_raw', 1, 30, True)
    forward = finite(forward_mps_per_raw, 'forward_mps_per_raw', .0001, .1)
    reverse = finite(reverse_mps_per_raw, 'reverse_mps_per_raw', .0001, .1)
    first = finite(first_seconds, 'first_seconds', .05, 5)
    second = finite(second_seconds, 'second_seconds', .05, 5)
    pause = finite(pause_seconds, 'pause_seconds', .3, 3)
    countdown = finite(countdown_seconds, 'countdown_seconds', 2, 10)
    stages = [Stage('COUNTDOWN', countdown, 0, 0)]
    if abs(distance) > 1e-9:
        coefficient = forward if distance > 0 else reverse
        seconds = finite(abs(distance)/(position_speed*coefficient),
                         'position_seconds', 0.05, 15)
        stages += [Stage('POSITION_FROM_REFERENCE', seconds,
                         position_speed if distance > 0 else -position_speed, 0),
                   Stage('PAUSE_BEFORE_REVERSE', pause, 0, 0)]
    stages += [Stage('REVERSE_LEFT_LOCK', first, -speed, 22),
               Stage('REVERSE_RIGHT_LOCK', second, -speed, -22),
               Stage('FINAL_STOP', max(.5, pause), 0, 0)]
    return stages


class OneShot(object):
    """Never resumes after done/abort; a control-loop gap aborts the run."""
    def __init__(self, stages):
        self.stages = list(stages)
        self.index = 0
        self.last = None
        self.deadline = None
        self.reason = 'NOT_STARTED'
        self.finished = False

    def abort(self, reason):
        self.finished = True
        self.reason = reason
        return 0, 0

    def tick(self, now):
        if self.finished:
            return 0, 0
        if math.isnan(now) or math.isinf(now):
            return self.abort('INVALID_CLOCK')
        if self.last is not None and not 0 <= now-self.last <= 0.25:
            return self.abort('CONTROL_LOOP_GAP')
        self.last = now
        if self.deadline is None:
            self.deadline = now + self.stages[0].seconds
        if now >= self.deadline:
            self.index += 1
            if self.index == len(self.stages):
                return self.abort('COMPLETE')
            self.deadline = now + self.stages[self.index].seconds
        stage = self.stages[self.index]
        self.reason = stage.name
        return stage.speed, stage.steering


def require_ownership(publishers, subscribers, own_name,
                      command='/parallel_parking/open_loop_cmd',
                      bridge='/parallel_open_loop_bridge'):
    """Reject other command owners, even idle teleoperation publishers."""
    ack = '/ackermann_cmd'
    if set(publishers.get(command, [])) - {own_name}:
        raise ValueError('another open-loop test is running')
    if set(publishers.get(ack, [])) != {bridge}:
        raise ValueError('require only '+bridge+' on /ackermann_cmd')
    for topic in ('/control/cmd', '/keyboard/control_cmd', '/joystick/control_cmd'):
        if publishers.get(topic):
            raise ValueError('stop other controller/teleop publisher: '+topic)
    if set(subscribers.get(command, [])) != {bridge}:
        raise ValueError('dedicated bridge input is not connected')
    if '/base_controller' not in subscribers.get(ack, []):
        raise ValueError('base_controller is not connected')
    if set(publishers.get('/base_controller/status', [])) != {'/base_controller'}:
        raise ValueError('base status publisher is missing or ambiguous')


def require_base(status, received_at, now):
    if status is None or not 0 <= now-received_at <= 0.5:
        raise ValueError('base status missing/stale')
    if (status.get('serial_open') is not True or
            status.get('command_received') is not True or
            status.get('timed_out') is not False or status.get('serial_bytes') != 11):
        raise ValueError('base serial/command is not ready')
