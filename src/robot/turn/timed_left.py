"""Production two-stage LEFT after stopping the front axle on blue.
The two durations count emitted commands, not measured wheel motion.
"""


def tick(ctx, now):
    cfg, task = ctx.cfg, ctx.left_lock
    if task.get('phase') == 'WAIT_LANE':
        if ctx.call('turn','handoff_lane_confirmed',now):
            ctx.call('mission','resume_lane')
            return ctx.call('motion','stop','left_timed_complete_lane')
        return ctx.call('motion','stop','left_timed_wait_lane')
    if task.get('timed_started') is None:
        task.update(phase='TIMED_ENTRY',timed_started=now,timed_last=now)
    dt=now-task['timed_last']
    if not 0 <= dt <= .5:
        ctx.state='FAULT'
        return ctx.call('motion','stop','left_timed_control_gap')
    # The bridge stops expired commands after .25 s. Recover brief scheduler
    # delays, but exclude that stopped interval from both motion durations.
    task['timed_started']+=max(0.,dt-.25)
    task['timed_last']=now
    duration=cfg.get('left_timed_entry_s',1.5) if task['phase']=='TIMED_ENTRY' else cfg.get('left_timed_turn_s',3.5)
    if now-task['timed_started'] >= duration:
        if task['phase']=='TIMED_ENTRY':
            task.update(phase='TIMED_TURN',timed_started=now)
        else:
            task['phase']='WAIT_LANE'
            ctx.exit_count,ctx.exit_stamp=0,-1.
            return ctx.call('motion','stop','left_timed_complete_wait_lane')
    entry=task['phase']=='TIMED_ENTRY'
    speed=cfg.get('left_timed_entry_speed_raw',30) if entry else cfg.get('left_timed_turn_speed_raw',30)
    raw=cfg.get('left_timed_entry_steering_raw',0) if entry else cfg.get('left_timed_turn_steering_raw',22)
    scale=cfg.get('steering_command_scale_rad',cfg['max_steer'])
    command=(float(speed)/cfg['speed_sign'],float(raw)/cfg['steering_raw_limit']*scale/cfg['steering_sign'])
    ctx.lane_source='left_timed_entry' if entry else 'left_timed_turn'
    result=ctx.call('obstacle','checked_command',command,now,False)
    if result != command:
        ctx.state='FAULT'
        return ctx.call('motion','stop','left_timed_guard:'+ctx.reason)
    ctx.reason=ctx.lane_source
    return result
