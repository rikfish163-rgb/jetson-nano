"""parallel_parking module: explicit context input; coordinator applies returned updates."""
from __future__ import division

from robot.common.contracts import model_to_command_steering
from robot.parallel_parking.planner import ForwardParking
from robot.parallel_parking.planner import ParallelParking
from robot.parallel_parking.planner import decode_parallel_scene



FIELDS = ('action_started', 'cfg', 'executor', 'parallel_future', 'parallel_parking',
 'parallel_scene', 'reason', 'state', 'wait_until')
CALLS = (('motion', 'stop'), ('obstacle', 'checked_command'))
OPERATIONS = ('observe_parallel_scene', 'parallel_parking_tick')


def _scene_measurement_ready(scene):
    """Require the camera producer's explicit readiness and occupancy gate."""
    if not isinstance(scene, dict):
        return False
    if 'ready' not in scene or 'occupancy' not in scene:
        return False
    if scene.get('ready') is not True:
        return False
    if scene.get('occupancy') != 'FREE':
        return False
    return True


def _forward_scene_ready(ctx, scene):
    return (ctx.cfg.get('parking_mode') != 'forward_plan' or
            scene.get('observation_source') == 'front')


def observe_parallel_scene(ctx, data, now):
    forward = ctx.cfg.get('parking_mode') == 'forward_plan'
    scene = decode_parallel_scene(data, allow_perpendicular=forward)
    if not 0 <= now-scene['stamp'] <= ctx.cfg['sensor_timeout']:
        raise ValueError('parallel parking scene stale')
    if ctx.parallel_scene and scene['stamp'] <= ctx.parallel_scene['stamp']:
        return
    if ctx.parallel_parking:
        ctx.parallel_parking.observe(scene)
    ctx.parallel_scene = scene


def parallel_parking_tick(ctx, now):
    if now-ctx.action_started > ctx.cfg.get('parallel_parking_timeout_s',60.0):
        ctx.state = 'FAULT'
        return ctx.call('motion', 'stop', 'parallel_parking_timeout')
    if now < ctx.wait_until:
        return ctx.call('motion', 'stop', 'parallel_parking_blue_stop')
    if ctx.parallel_parking is None:
        scene = ctx.parallel_scene
        if (scene is None or scene['stamp'] < ctx.action_started or
                not 0 <= now-scene['stamp'] <= ctx.cfg['sensor_timeout']):
            return ctx.call('motion', 'stop', 'parallel_parking_scene_missing')
        if not _scene_measurement_ready(scene) or not _forward_scene_ready(ctx, scene):
            return ctx.call('motion', 'stop', 'parallel_parking_scene_unconfirmed')
        requested = ctx.cfg['parking_slot']
        slot = scene['slot']
        if (requested != 'AUTO' and slot['id'] != requested):
            return ctx.call('motion', 'stop', 'parallel_parking_requested_slot_missing')
        nominal = ctx.cfg.get('slots',{}).get(slot['id'])
        allowed_kinds = ('parallel', 'perpendicular') if (
            ctx.cfg.get('parking_mode') == 'forward_plan') else ('parallel',)
        if (nominal is None or nominal.get('kind') not in allowed_kinds or
                slot.get('kind') != nominal.get('kind')):
            return ctx.call('motion', 'stop', 'parallel_parking_wrong_slot_kind')
        size_tolerance = ctx.cfg.get('parallel_parking_slot_size_tolerance_m',.05)
        if any(abs(slot[key]-nominal[key]) > size_tolerance for key in ('length','width')):
            return ctx.call('motion', 'stop', 'parallel_parking_slot_size_mismatch')
        task_class = (ForwardParking if ctx.cfg.get('parking_mode') ==
                      'forward_plan' else ParallelParking)
        ctx.parallel_parking = task_class(ctx.cfg,scene)
        ctx.parallel_future = ctx.executor.submit(ctx.parallel_parking.plan)
        return ctx.call('motion', 'stop', 'parallel_parking_planning')
    task = ctx.parallel_parking
    # A confirmed task must stop when the newest measured scene loses
    # readiness or occupancy clearance, including a missing production gate.
    if (not _scene_measurement_ready(ctx.parallel_scene) or
            not _forward_scene_ready(ctx, ctx.parallel_scene)):
        return ctx.call('motion', 'stop', 'parallel_parking_scene_unconfirmed')
    if task.phase == 'PLAN':
        if not ctx.parallel_future.done():
            return ctx.call('motion', 'stop', 'parallel_parking_planning')
        try:
            task.accept_plan(ctx.parallel_future.result())
        except Exception as exc:
            ctx.state = 'FAULT'
            return ctx.call('motion', 'stop', 'parallel_parking_planner_error:'+str(exc))
    speed, physical = task.command(now)
    command = speed, model_to_command_steering(physical,ctx.cfg)
    if task.phase == 'DONE':
        ctx.state = 'FINISHED'
        return ctx.call('motion', 'stop', 'parallel_parking_complete')
    ctx.reason = task.reason
    return ctx.call('obstacle', 'checked_command', command,now,False)
