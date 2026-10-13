"""Production two-stage LEFT after stopping the front axle on blue.
The two durations count emitted commands, not measured wheel motion.
"""


def tick(ctx, now):
    cfg, task = ctx.cfg, ctx.left_lock
    if task.get('phase') == 'WAIT_LANE':
        if ctx.call('turn','handoff_lane_confirmed',now):
            ctx.call('mission','resume_lane')
            return ctx.call('lane','search_command',now,'left_timed_complete_lane')
        return ctx.call('lane','search_command',now,'left_timed_lane_search')
    if task.get('bay_profile'):
        return bay_tick(ctx,now)
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
            task.update(phase='WAIT_LANE',exit_started=now)
            ctx.exit_count,ctx.exit_stamp=0,-1.
            return ctx.call('lane','search_command',now,'left_timed_lane_search')
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


def bay_tick(ctx,now):
    """Selected bay maneuver; the final straight is never a visual search."""
    cfg,task=ctx.cfg,ctx.left_lock
    p=task['bay_profile']
    if task.get('phase')=='FINISHED':
        return ctx.call('motion','stop','left_bay_'+p['target']+'_complete')
    stages=[('TIMED_ENTRY',p['entry_s'],0),('TIMED_TURN',p['turn_s'],22),
            ('TIMED_STRAIGHT',p['straight_s'],0),('BAY_SETTLE',p['settle_s'],0)]
    stages=[s for s in stages if s[1]>0]
    if task.get('bay_index') is None:
        task.update(bay_index=0,phase=stages[0][0],timed_started=now,timed_last=now)
    dt=now-task['timed_last']
    if not 0<=dt<=.5:
        ctx.state='FAULT'
        return ctx.call('motion','stop','left_bay_control_gap')
    if task['phase']!='BAY_SETTLE':
        task['timed_started']+=max(0.,dt-.25)
    task['timed_last']=now
    stage=stages[task['bay_index']]
    if now-task['timed_started']>=stage[1]-1e-9:
        task['bay_index']+=1
        if task['bay_index']==len(stages):
            ctx.parking_completed=True
            ctx.turn_reverse_watch=None
            ctx.state='FINISHED'
            task.update(phase='FINISHED',completed_at=now)
            return ctx.call('motion','stop','left_bay_'+p['target']+'_complete')
        stage=stages[task['bay_index']]
        task.update(phase=stage[0],timed_started=now)
    if stage[0]=='BAY_SETTLE':
        return ctx.call('motion','stop','left_bay_'+p['target']+'_settle')
    scale=cfg.get('steering_command_scale_rad',cfg['max_steer'])
    command=(30./cfg['speed_sign'],float(stage[2])/cfg['steering_raw_limit']*scale/cfg['steering_sign'])
    result=ctx.call('obstacle','checked_command',command,now,False)
    if result!=command:
        ctx.state='FAULT'
        return ctx.call('motion','stop','left_bay_guard:'+ctx.reason)
    ctx.lane_source='left_bay_'+p['target']+'_'+stage[0].lower()
    ctx.reason=ctx.lane_source
    return result
