"""Explicit open-loop calibration maneuver; distances are command estimates."""
from __future__ import division
import math


def sequence_steps(cfg,forward,vf,entry,pause):
    rows=cfg['uturn_trial_sequence']
    if not isinstance(rows,list) or not 1<=len(rows)<=20:
        raise ValueError('trial sequence must contain 1..20 segments')
    steps=[('ENTRY',entry/vf,(int(forward),0.0))] if entry else []
    previous=0
    for i,row in enumerate(rows):
        if not isinstance(row,dict):raise ValueError('invalid trial segment')
        speed,raw,seconds=(row.get(k) for k in ('speed','steering','seconds'))
        for v in (speed,raw,seconds):
            if isinstance(v,bool) or not isinstance(v,(int,float)) or math.isnan(v) or math.isinf(v):
                raise ValueError('invalid trial segment number')
        if speed!=int(speed) or not 1<=abs(speed)<=30 or raw not in (-22,0,22) or not 0<seconds<=10:
            raise ValueError('trial segment outside limits')
        if i==0 and speed<0:raise ValueError('trial must start forward')
        angle=raw/cfg['steering_raw_limit']/cfg['steering_sign']*cfg.get('steering_command_scale_rad',cfg['max_steer'])
        if previous*speed<=0:steps.append(('GEAR_PAUSE_%d'%(i+1),pause,(0,angle)))
        steps.append(('SEGMENT_%d'%(i+1),seconds,(int(speed/cfg['speed_sign']),angle)))
        previous=speed
    if previous<0:raise ValueError('trial must finish forward')
    steps.append(('SETTLE',pause,(0,0.0)))
    return steps


class TimedUturn(object):
    def __init__(self, cfg):
        def value(key, default, low, high):
            if isinstance(cfg.get(key,default),bool):raise ValueError('invalid '+key)
            v=float(cfg.get(key,default))
            if math.isnan(v) or math.isinf(v) or not low <= v <= high:
                raise ValueError('invalid '+key)
            return v
        side=cfg.get('uturn_trial_side','left')
        if side not in ('left','right'):raise ValueError('uturn_trial_side must be left or right')
        self.side=side
        direction=1 if side=='left' else -1
        radius=value('uturn_trial_radius_m',.65,.2,2)
        spacing=value('uturn_trial_spacing_m',.6,.1,1.2)
        if spacing>2*radius:raise ValueError('spacing exceeds three-arc geometry')
        alpha=math.acos((2*radius-spacing)/(4*radius))
        beta=math.pi-2*alpha
        forward=value('uturn_trial_forward_raw',26,1,30)
        reverse=value('uturn_trial_reverse_raw',26,1,30)
        raw=value('uturn_trial_steering_raw',22,1,min(22,cfg['steering_raw_limit']))
        if any(v!=int(v) for v in (forward,reverse,raw)):raise ValueError('RAW values must be integers')
        # Encode the requested RAW magnitude through the existing command scale.
        steer=direction*raw/cfg['steering_raw_limit']*cfg.get('steering_command_scale_rad',cfg['max_steer'])
        vf=value('uturn_trial_forward_mps',.208,.02,1)
        vr=value('uturn_trial_reverse_mps',.182,.02,1)
        pause=value('uturn_trial_pause_s',.6,.3,3)
        entry=value('uturn_trial_entry_m',.12,0,.8)
        first=value('uturn_trial_first_gain',1,.4,1.6)
        back=value('uturn_trial_reverse_gain',1,.4,1.6)
        last=value('uturn_trial_last_gain',1,.4,1.6)
        self.steps=[('ENTRY',entry/vf,(int(forward),0.0)),
                    ('FORWARD_FIRST',radius*alpha*first/vf,(int(forward),steer)),
                    ('BRAKE_REVERSE',pause,(0,0.0)),
                    ('REVERSE_ARC',radius*beta*back/vr,(-int(reverse),-steer)),
                    ('BRAKE_FORWARD',pause,(0,0.0)),
                    ('FORWARD_LAST',radius*alpha*last/vf,(int(forward),steer)),
                    ('SETTLE',pause,(0,0.0))]
        if 'uturn_trial_sequence' in cfg:
            self.steps=sequence_steps(cfg,forward,vf,entry,pause)
        self.index,self.elapsed,self.last=0,0.0,None
        self.total_s=sum(step[1] for step in self.steps)
        if self.total_s+cfg.get('intersection_wait_s',1)+1 >= cfg['action_timeout']:
            raise ValueError('trial duration exceeds action_timeout')

    def pause(self):
        self.last=None

    def command(self, now):
        dt=0 if self.last is None else now-self.last
        self.last=now
        if not 0<=dt<=.5:raise ValueError('trial control clock gap')
        self.elapsed+=dt
        if self.index < len(self.steps) and self.elapsed>=self.steps[self.index][1]:
            self.index+=1
            self.elapsed=0.0  # Never skip a braking phase after a delayed tick.
        if self.index==len(self.steps):return (0,0.0)
        return self.steps[self.index][2]

    @property
    def done(self):return self.index==len(self.steps)

    def status(self):
        return dict(phase='DONE' if self.done else self.steps[self.index][0],
                    elapsed_s=self.elapsed,total_estimated_s=self.total_s,
                    side=self.side,evidence='open_loop_command_time')
