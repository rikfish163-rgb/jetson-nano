"""Shared image-row trigger and fixed-time approach for tests and production.
No ROS, pose integration or distance-to-stop calculation.
"""
from __future__ import division
import math
from robot.common.contracts import number
from robot.camera.alignment import steering_for_heading
from robot.common.geometry import distance


DIRECTION_ACTIONS = ('LEFT','RIGHT','STRAIGHT','UTURN')


def pending_approach(ctx):
    return (ctx.cfg.get('blue_timed_enabled',False) and ctx.action is None and
            ctx.pending in DIRECTION_ACTIONS and ctx.state in ('LANE','GAP','WAIT_OBSTACLE'))


def validate_settings(settings, cfg):
    limits=dict(speed_raw=(1,min(30,cfg['speed_raw_limit'])),trigger_row_ratio=(.05,.9),
                trigger_band_ratio=(.01,.3),forward_seconds=(0,10),confirm_frames=(1,10),
                heading_tolerance_deg=(1,30),max_search_seconds=(1,60),camera_timeout_s=(.1,2),
                align_speed_raw=(1,min(30,cfg['speed_raw_limit'])),
                align_max_steering_raw=(1,cfg['steering_raw_limit']),
                align_tolerance_deg=(1,15),align_confirm_frames=(1,10),align_timeout_s=(.1,30),
                align_recheck_s=(0,3))
    result={}
    for key,(low,high) in limits.items():
        value=number(settings.get(key,0) if key=='align_recheck_s' else settings[key])
        if not low<=value<=high:raise ValueError('invalid blue stop '+key)
        result[key]=value
    if result['trigger_row_ratio']+result['trigger_band_ratio']>1:
        raise ValueError('trigger band outside image')
    if result['align_tolerance_deg']>result['heading_tolerance_deg']:
        raise ValueError('alignment tolerance exceeds trigger tolerance')
    for key in ('speed_raw','confirm_frames','align_speed_raw','align_max_steering_raw','align_confirm_frames'):
        if result[key]!=int(result[key]):raise ValueError('integer required: '+key)
        result[key]=int(result[key])
    return result


def row_candidate(lines,cfg):
    camera=cfg['front_camera'];candidates=[]
    for line in lines:
        x,y,yaw,length=[number(line[k]) for k in ('x','y','yaw','length')]
        heading=(yaw+math.pi/2)%math.pi-math.pi/2
        if x<=0 or length<cfg['blue']['long_min'] or abs(heading)>math.radians(60):continue
        if abs(y/math.cos(heading))>length/2:continue
        cross_x=x+y*math.tan(heading)
        row=(camera['origin_v']-cross_x*camera['pixels_per_m'])/(camera['bev_height']-1.)
        if 0<=row<=1:candidates.append((row,math.degrees(heading)))
    return max(candidates) if candidates else None


def advance(state,now,stamp,lines,cfg,p,allow_trigger=True):
    """Align before image trigger, then run the calibrated straight interval."""
    if state.get('done'):return 'done'
    if state.get('fault'):return state['fault']
    def fail(reason):
        state['fault']=reason
        return reason
    if not 0<=now-stamp<=p['camera_timeout_s']:return fail('sensor_lost')
    gap=now-state.get('last',now)
    if not 0<=gap<=.5:return fail('control_gap')
    if state.get('trigger') is not None:
        # The bridge stops expired commands after .25 s. Recover brief
        # scheduler delays without counting that stopped time as forward motion.
        state['paused_s']=state.get('paused_s',0.)+max(0.,gap-.25)
    state['last']=now
    state.setdefault('start',now)
    if state.get('trigger') is None:
        if now-state['start']>=p['max_search_seconds']:return fail('search_timeout')
        if (state.get('recheck_started') is not None and
                now-state['recheck_started']>=p.get('align_recheck_s',0)):
            return fail('alignment_recheck_timeout')
        if (stamp>state.get('processed',-1.) and
                (state.get('recheck_started') is None or stamp>state['recheck_started'])):
            state['processed']=stamp
            candidate=row_candidate(lines,cfg)
            if candidate is None:
                if state.get('seen_blue'):return fail('blue_lost_before_trigger')
                state.update(confirmations=0,previous_row=None,debug=dict(row_ratio=None,phase='search'))
            else:
                row,angle=candidate
                state['in_trigger_band']=row>=p['trigger_row_ratio']
                state['seen_blue']=True
                state['heading_deg']=angle
                state.setdefault('align_started',now)
                state['debug']=dict(row_ratio=row,heading_deg=angle,trigger_row_ratio=p['trigger_row_ratio'])
                if row>p['trigger_row_ratio']+p['trigger_band_ratio']:return fail('past_trigger_band')
                tolerance=(p['heading_tolerance_deg'] if state['in_trigger_band']
                           else p['align_tolerance_deg'])
                if abs(angle)<=tolerance:
                    state['align_frames']=state.get('align_frames',0)+1
                    if state['align_frames']>=p['align_confirm_frames']:state['aligned']=True
                else:
                    state['align_frames']=0
                    if abs(angle)>p['heading_tolerance_deg']:
                        if state.get('aligned'):state['align_started']=now
                        state['aligned']=False
                state['debug'].update(align_frames=state['align_frames'],aligned=state.get('aligned',False))
                if row>=p['trigger_row_ratio']:
                    # A physically straight car may reach the trigger while
                    # the last alignment votes are still arriving. Confirm
                    # alignment and row concurrently with neutral steering;
                    # do not fault solely because those votes are unfinished.
                    if abs(angle)>p['heading_tolerance_deg']:
                        if p.get('align_recheck_s',0)<=0:
                            return fail('alignment_distance_insufficient')
                        state.setdefault('recheck_started',now)
                        state.update(confirmations=0,aligned=False,align_frames=0)
                    elif allow_trigger:
                        previous=state.get('previous_row')
                        if previous is not None and abs(row-previous)>p['trigger_band_ratio']:state['confirmations']=0
                        state['confirmations']=state.get('confirmations',0)+1
                        state['previous_row']=row
                        if state.get('aligned') and state['confirmations']>=p['confirm_frames']:
                            state['trigger']=now
                else:state.update(confirmations=0,previous_row=row)
        if (state.get('seen_blue') and not state.get('aligned') and
                now-state['align_started']>=p['align_timeout_s']):return fail('alignment_timeout')
        if state.get('recheck_started') is not None and state.get('trigger') is None:
            elapsed=now-state['recheck_started']
            state['debug'].update(phase='recheck',recheck_elapsed_s=elapsed,
                                  trigger_heading_tolerance_deg=p['heading_tolerance_deg'])
            if elapsed>=p.get('align_recheck_s',0):return fail('alignment_recheck_timeout')
            return 'recheck'
    if state.get('trigger') is not None:
        paused=state.get('paused_s',0.)
        elapsed=now-state['trigger']-paused
        state.setdefault('debug',{}).update(elapsed_s=elapsed,paused_s=paused,
                                            control_gap_s=gap)
        if elapsed>=p['forward_seconds']:
            state['done']=True
            return 'done'
        state['debug']['phase']='timed'
        return 'timed'
    if state.get('seen_blue') and not state.get('in_trigger_band') and (not state.get('aligned') or
            abs(state['heading_deg'])>p['align_tolerance_deg']):
        state['debug']['phase']='align'
        return 'align'
    state.setdefault('debug',{})['phase']='approach'
    return 'approach'


def approach_command(state,result,cfg,p):
    """Shared command conversion, including steering polarity and raw limits."""
    speed=p['speed_raw'];steering=0.
    if result=='recheck':return 0.,0.
    if result=='align':
        speed=p['align_speed_raw']
        angle=state['heading_deg']
        if abs(angle)>p['align_tolerance_deg']:
            steering=steering_for_heading(math.radians(angle),cfg)
            limit=(cfg.get('steering_command_scale_rad',cfg['max_steer'])*
                   p['align_max_steering_raw']/float(cfg['steering_raw_limit']))
            steering=max(-limit,min(limit,steering))
    return float(speed)/cfg['speed_sign'],steering


def prealign_tick(ctx,now):
    """Steer during junction voting; never consume a line or dispatch an action.

    Only the geometrically filtered, currently observed marker candidate may
    steer. Its three-frame confirmation still belongs to camera.observations.
    """
    candidate=ctx.marker_candidate
    p=dict(ctx.cfg.get('blue_stop_test',{}))
    p['camera_timeout_s']=ctx.cfg.get('ground_timeout',1.25)
    if (not pending_approach(ctx) or ctx.blue_consumed or candidate is None or
            candidate['stamp']!=ctx.front_marker_stamp or
            not 0<=now-candidate['stamp']<=p['camera_timeout_s']):
        ctx.blue_prealign=None
        return None
    # Both lists were captured by the same observation callback. Comparing
    # their saved points avoids mixing a delayed image with the current pose.
    raw_front=[line for line in ctx.front_blue_image_lines if line['x']>0]
    lines=[line for line,measured in zip(raw_front,ctx.front_blue_lines)
           if distance(measured['point'],candidate['point'])<.05]
    if row_candidate(lines,ctx.cfg) is None:
        ctx.blue_prealign=None
        return None
    task=ctx.blue_prealign
    if (task is None or task['action']!=ctx.pending or
            distance(task['point'],candidate['point'])>.15 or
            (candidate['stamp']>task['stamp'] and candidate['count']==1)):
        task=dict(action=ctx.pending,point=candidate['point'],stamp=candidate['stamp'],sequence={})
    task.update(point=candidate['point'],stamp=candidate['stamp'])
    ctx.blue_prealign=task
    result=advance(task['sequence'],now,ctx.front_marker_stamp,lines,ctx.cfg,p,allow_trigger=False)
    if result not in ('align','approach','recheck'):
        ctx.state='FAULT'
        return ctx.call('motion','stop','blue_prealign_'+result)
    command=approach_command(task['sequence'],result,ctx.cfg,p)
    output=ctx.call('obstacle','checked_command',command,now,False)
    if output!=command:
        ctx.state='FAULT'
        return ctx.call('motion','stop','blue_prealign_guard:'+ctx.reason)
    ctx.lane_curve_lock=None
    ctx.lane_source='blue_image_prealign'
    ctx.reason=ctx.pending.lower()+('_blue_recheck' if result=='recheck' else '_blue_prealign')
    return output


def production_tick(ctx,now):
    task=ctx.blue_approach
    # Production already has a shared front-image freshness budget; the
    # standalone blue-stop test keeps its own stricter timeout setting.
    p=dict(ctx.cfg['blue_stop_test'])
    p['camera_timeout_s']=ctx.cfg.get('ground_timeout',1.25)
    if task['phase']=='STOP':
        if now<task['stop_until']:
            return ctx.call('motion','stop','blue_stop_wait_'+ctx.action.lower())
        if not 0<=now-ctx.front_marker_stamp<=p['camera_timeout_s']:
            ctx.state='FAULT'
            return ctx.call('motion','stop','blue_timed_sensor_lost')
        return ctx.call('mission','begin_blue_action',now)
    sequence=task.setdefault('image_timing',{})
    result=advance(sequence,now,ctx.front_marker_stamp,ctx.front_blue_image_lines,ctx.cfg,p)
    task['image_debug']=sequence.get('debug',{})
    if result=='done':
        task.update(phase='STOP',stop_until=now+ctx.cfg.get('intersection_wait_s',1.),stop_pose=ctx.pose)
        ctx.state='BLUE_STOP'
        return ctx.call('motion','stop','blue_stop_wait_'+ctx.action.lower())
    if result not in ('align','approach','timed','recheck'):
        ctx.state='FAULT'
        return ctx.call('motion','stop','blue_timed_'+result)
    command=approach_command(sequence,result,ctx.cfg,p)
    output=ctx.call('obstacle','checked_command',command,now,False)
    if output!=command:
        ctx.state='FAULT'
        return ctx.call('motion','stop','blue_timed_guard:'+ctx.reason)
    ctx.lane_source='blue_image_'+result
    ctx.reason=ctx.action.lower()+'_blue_'+result
    return output
