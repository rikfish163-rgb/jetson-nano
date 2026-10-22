#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Interactive forward/reverse RAW 15/20/25/30 calibration. Preview by default."""
from __future__ import division,print_function
import argparse
import csv
import json
import os
import subprocess
import sys
import time
from open_loop_core import Stage,finite
from parallel_open_loop_test import execute

ROOT=os.path.abspath(os.path.join(os.path.dirname(__file__),'../../..'))
SPEEDS=(15,20,25,30)
DIRECTIONS=('forward','reverse')
try:read_input=raw_input
except NameError:read_input=input


def make_trials(speeds,durations,directions,repeats):
    if not speeds or not durations or not directions:raise ValueError('empty calibration plan')
    speeds=[finite(s,'speed',15,30,True) for s in speeds]
    if any(s not in SPEEDS for s in speeds):raise ValueError('speeds must be 15,20,25,30')
    durations=[finite(t,'duration',.25,5.) for t in durations]
    repeats=finite(repeats,'repeats',1,5,True)
    if any(d not in DIRECTIONS for d in directions):raise ValueError('invalid direction')
    if any(len(set(v))!=len(v) for v in (speeds,durations,directions)):
        raise ValueError('duplicate speeds/durations/directions; use --repeats instead')
    rows=[]
    for direction in directions:
        for speed in speeds:
            for seconds in durations:
                for repeat in range(1,repeats+1):
                    rows.append(dict(index=len(rows)+1,direction=direction,speed_raw=speed,
                        signed_speed_raw=speed if direction=='forward' else -speed,
                        steering_raw=0,requested_seconds=seconds,repeat=repeat,
                        status='pending',attempt=0,command_seconds=None,distance_cm=None))
    return rows


def new_session(speeds,durations,directions,repeats):
    return dict(version=1,created_at=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
        config=dict(speeds=list(speeds),durations=list(durations),directions=list(directions),repeats=repeats),
        complete=False,trials=make_trials(speeds,durations,directions,repeats))


def measurement(value):
    return finite(float(value),'distance_cm',0,1000.)


class CommandRecorder(object):
    def __init__(self,speed):
        self.speed=speed;self.started=self.stopped=None;self.events=[]

    def __call__(self,speed,steering,now):
        if steering!=0 or speed not in (0,self.speed):raise ValueError('unexpected calibration command')
        self.events.append(dict(monotonic_s=now,speed_raw=speed,steering_raw=steering))
        if speed:
            if self.stopped is not None:raise ValueError('unexpected motion restart')
            if self.started is None:self.started=now
        elif self.started is not None and self.stopped is None:self.stopped=now

    def seconds(self):
        if self.started is None or self.stopped is None:raise ValueError('no completed motion interval')
        return finite(self.stopped-self.started,'command_seconds',.001,5.25)


def fit_models(trials):
    groups={}
    for row in trials:
        if row['status']=='measured':
            groups.setdefault((row['direction'],row['speed_raw']),[]).append(row)
    result={}
    for (direction,speed),rows in sorted(groups.items()):
        times=[finite(r['command_seconds'],'command_seconds',.001,5.25) for r in rows]
        distances=[measurement(r['distance_cm'])/100. for r in rows]
        model=dict(sample_count=len(rows),duration_range_s=[min(times),max(times)],
                   usable=True,method='effective_average',offset_m=0.)
        if max(distances)==0:
            model.update(usable=False,method='no_motion',speed_mps=0.,mps_per_raw=0.)
        else:
            velocity=sum(distances)/sum(times)
            mean_t=sum(times)/len(times);mean_d=sum(distances)/len(distances)
            denominator=sum((t-mean_t)**2 for t in times)
            if len(set(r['requested_seconds'] for r in rows))>=2 and denominator>1e-6:
                velocity=sum((t-mean_t)*(d-mean_d) for t,d in zip(times,distances))/denominator
                model.update(method='affine',offset_m=mean_d-velocity*mean_t)
            model.update(speed_mps=velocity,mps_per_raw=velocity/speed,usable=velocity>0)
            if velocity<=0:model['method']='inconsistent'
            model['rms_error_m']=(sum((d-max(0.,velocity*t+model['offset_m']))**2
                                      for t,d in zip(times,distances))/len(times))**.5
        result.setdefault(direction,{})[str(speed)]=model
    return result


def atomic_text(path,text):
    temporary=path+'.tmp'
    with open(temporary,'w') as stream:stream.write(text)
    os.rename(temporary,path)


def save_session(session,folder):
    import yaml
    if not os.path.isdir(folder):os.makedirs(folder)
    session['complete']=all(r['status']=='measured' for r in session['trials'])
    session['models']=fit_models(session['trials'])
    atomic_text(os.path.join(folder,'results.json'),json.dumps(session,indent=2,allow_nan=False)+'\n')
    table=dict(version=1,complete=session['complete'],models=session['models'],
        formula='distance_m = max(0, speed_mps * command_seconds + offset_m)',
        measurement='Stopped axle-to-axle displacement; includes starting and stopping effects.')
    atomic_text(os.path.join(folder,'speed_table.yaml'),yaml.safe_dump(table,default_flow_style=False))
    columns=['index','direction','speed_raw','requested_seconds','command_seconds','distance_cm','status']
    temporary=os.path.join(folder,'measurements.csv.tmp')
    with open(temporary,'w') as stream:
        writer=csv.DictWriter(stream,fieldnames=columns,extrasaction='ignore')
        writer.writeheader();writer.writerows(session['trials'])
    os.rename(temporary,os.path.join(folder,'measurements.csv'))


def load_session(folder):
    with open(os.path.join(folder,'results.json')) as stream:session=json.load(stream)
    if session.get('version')!=1:raise ValueError('unsupported calibration file version')
    cfg=session['config'];expected=make_trials(cfg['speeds'],cfg['durations'],cfg['directions'],cfg['repeats'])
    if len(expected)!=len(session['trials']):raise ValueError('calibration plan mismatch')
    for row,template in zip(session['trials'],expected):
        for key in ('index','direction','speed_raw','signed_speed_raw','steering_raw','requested_seconds','repeat'):
            if row[key]!=template[key]:raise ValueError('calibration trial mismatch: '+key)
        if row['status'] not in ('pending','moving','awaiting_measurement','measured','failed'):
            raise ValueError('invalid trial status')
        finite(row['attempt'],'attempt',0,1000,True)
        if row['status'] in ('awaiting_measurement','measured'):
            finite(row['command_seconds'],'command_seconds',.001,row['requested_seconds']+.25)
        if row['status']=='measured':measurement(row['distance_cm'])
    return session


def run_child(speed,seconds,output):
    speed=finite(speed,'signed speed',-30,30,True)
    if abs(speed) not in SPEEDS:raise ValueError('invalid calibration speed')
    seconds=finite(seconds,'seconds',.25,5.)
    recorder=CommandRecorder(speed)
    record=dict(status='failed',speed_raw=speed,requested_seconds=seconds,
                command_seconds=None,started_at=time.time())
    try:
        stages=[Stage('COUNTDOWN',3.,0,0),Stage('CALIBRATE_STRAIGHT',seconds,speed,0),
                Stage('FINAL_STOP',.7,0,0)]
        result=execute(stages,on_command=recorder)
        record.update(status='complete',command_seconds=recorder.seconds())
        return result
    except (ValueError,RuntimeError,KeyboardInterrupt) as exc:
        record['error']=str(exc);print('STOP: '+str(exc));return 1
    finally:
        record.update(finished_at=time.time(),commands=recorder.events)
        atomic_text(output,json.dumps(record,indent=2,allow_nan=False)+'\n')


def subprocess_trial(row,path):
    command=[sys.executable,'-B',os.path.abspath(__file__),'--execute',
        '--worker-speed',str(row['signed_speed_raw']),'--worker-seconds',str(row['requested_seconds']),
        '--worker-output',path]
    process=subprocess.Popen(command)
    try:return process.wait()
    finally:
        if process.poll() is None:
            process.terminate();process.wait()


def print_models(session):
    print('\n方向       RAW  速度(cm/s)  综合偏差(cm)  样本数  模型')
    for direction,models in sorted(session.get('models',{}).items()):
        for speed,model in sorted(models.items(),key=lambda pair:int(pair[0])):
            print('%-10s %2s %11.2f %13.2f %7d  %s' % (direction,speed,
                model['speed_mps']*100,model['offset_m']*100,model['sample_count'],model['method']))


def run_session(session,folder,runner=subprocess_trial,input_fn=read_input):
    save_session(session,folder)
    try:
        for row in session['trials']:
            if row['status']=='measured':continue
            while row['status']!='measured':
                if row['status']!='awaiting_measurement':
                    row['attempt']+=1;row['status']='moving'
                    save_session(session,folder)
                    path=os.path.join(folder,'trial_%03d_attempt_%02d.json'%(row['index'],row['attempt']))
                    print('\n[%d/%d] %s RAW=%+d 转向=0，运行 %.2f 秒。' %
                          (row['index'],len(session['trials']),str(row['direction']),row['signed_speed_raw'],row['requested_seconds']))
                    print('标记当前前轮轴起点，准备好后按 g；倒计时 3 秒后运行，随后自动停车。')
                    sys.stdout.flush()
                    code=runner(row,path)
                    if code!=0:
                        row.update(status='failed',error='trial process returned %d'%code)
                        print('本轮未完成，保存进度并退出。');return 1
                    with open(path) as stream:record=json.load(stream)
                    if record.get('status')!='complete' or record.get('speed_raw')!=row['signed_speed_raw']:
                        raise ValueError('invalid trial execution record')
                    row['command_seconds']=finite(record['command_seconds'],'command_seconds',.001,row['requested_seconds']+.25)
                    row['status']='awaiting_measurement';row['execution_record']=os.path.basename(path)
                    save_session(session,folder)
                print('实际发送运动指令 %.3f 秒；量停车后的前轮轴单段位移。'%row['command_seconds'])
                try:answer=input_fn('距离（厘米；0=没动，r=重做本轮，q=保存退出）：').strip().lower()
                except EOFError:answer='q'
                if answer=='q':return 0
                if answer=='r':
                    row.update(status='pending',command_seconds=None,distance_cm=None)
                    save_session(session,folder);continue
                try:distance=measurement(answer)
                except (ValueError,TypeError):
                    print('请输入 0～1000 的厘米数，或 r/q。');continue
                row.update(status='measured',distance_cm=distance)
                save_session(session,folder)
                print('已保存：%.2f cm，平均 %.2f cm/s。'%(distance,distance/row['command_seconds']))
        print_models(session)
        return 0
    finally:save_session(session,folder)


def main(argv=None):
    parser=argparse.ArgumentParser(description='前后 RAW 15/20/25/30 标定；默认只预览。')
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--speeds',nargs='+',type=int,choices=SPEEDS,default=list(SPEEDS))
    parser.add_argument('--durations',nargs='+',type=float,default=[2.])
    parser.add_argument('--directions',nargs='+',choices=DIRECTIONS,default=list(DIRECTIONS))
    parser.add_argument('--repeats',type=int,default=1)
    parser.add_argument('--output',help='new result directory')
    parser.add_argument('--resume',help='resume an existing result directory')
    parser.add_argument('--worker-speed',type=int,help=argparse.SUPPRESS)
    parser.add_argument('--worker-seconds',type=float,help=argparse.SUPPRESS)
    parser.add_argument('--worker-output',help=argparse.SUPPRESS)
    args=parser.parse_args(argv)
    try:
        if args.worker_speed is not None:
            if not args.execute or not args.worker_output or args.worker_seconds is None:
                raise ValueError('worker requires --execute, duration and result path')
            return run_child(args.worker_speed,args.worker_seconds,args.worker_output)
        if args.resume and args.output:raise ValueError('choose --resume OR --output')
        if args.resume:
            folder=os.path.abspath(args.resume);session=load_session(folder)
        else:
            session=new_session(args.speeds,args.durations,args.directions,args.repeats)
            folder=os.path.abspath(args.output or os.path.join(ROOT,'field_data','calibration',
                'speed_%s_%d'%(time.strftime('%Y%m%d_%H%M%S'),os.getpid())))
        print('标定顺序：')
        for row in session['trials']:
            print('%2d  %-7s RAW=%+3d steering=0  %.2fs  %s'%
                  (row['index'],row['direction'],row['signed_speed_raw'],row['requested_seconds'],row['status']))
        if not args.execute:
            print('PREVIEW ONLY: no ROS nodes or vehicle commands. Add --execute to calibrate.');return 0
        if not sys.stdin.isatty():raise ValueError('execute requires a foreground interactive SSH terminal')
        if not args.resume and any(os.path.exists(os.path.join(folder,name)) for name in
                                   ('results.json','measurements.csv','speed_table.yaml')):
            raise ValueError('results exist; use --resume or choose another directory')
        print('结果目录：'+folder)
        print('每轮单独启动；测量输入不限时；空格/x/q/Esc/Ctrl+C 可中止运动。')
        sys.stdout.flush()
        result=run_session(session,folder)
        print('已保存：'+os.path.join(folder,'results.json'))
        print('续测：python2 -B %s --execute --resume %s'%(os.path.abspath(__file__),folder))
        return result
    except (ValueError,TypeError,KeyError,OSError,RuntimeError,KeyboardInterrupt) as exc:
        print('STOP / NOT STARTED: '+str(exc),file=sys.stderr);return 1


if __name__=='__main__':sys.exit(main())
