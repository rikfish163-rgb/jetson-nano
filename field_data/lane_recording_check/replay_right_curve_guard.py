"""No ROS publishers: replay recorded vehicle-frame targets through the controller."""
from __future__ import print_function
import glob, json, os
from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.master.controller import Controller

root='/home/nano/robocup_ws'
out=root+'/field_data/lane_recording_check/right_curve_guard'
if not os.path.isdir(out):os.makedirs(out)
report=dict(method='Recorded current-vehicle-frame paths; fixed replay pose. Not a new physical trajectory.',runs=[])
for run in sorted(glob.glob(root+'/field_data/lane_runs/*')):
    path=run+'/target.jsonl'
    if not os.path.isfile(path):continue
    cfg=load_config(root+'/src/robot/config')
    cfg.update(wait_green=False,lidar_enabled=False,steering_command_scale_rad=.03)
    core=Controller(cfg)
    counts=dict(run=os.path.basename(run),ticks=0,locks=0,releases=0,guarded=0,
                positive_while_locked=0,gap_positive_while_locked=0,stops=0)
    stream=open(out+'/'+counts['run']+'.jsonl','w')
    for line in open(path):
        row=json.loads(line)['data']
        if row['state'] not in ('LANE','GAP'):
            core.lane_curve_lock=None
            core.gap_origin=None
            core.state='LANE'
            continue
        selection=row.get('selection')
        core.lane=[tuple(p) for p in selection['path']] if selection else []
        core.lane_stamp=row['source_stamp']
        core.lane_target=None
        before=core.lane_curve_lock is not None
        speed,steer=core.lane_command(row['stamp'])
        core.issued_steer=steer
        raw=encode_command(speed,steer,cfg,0)['steering_raw']
        locked=core.lane_curve_lock is not None
        guarded=bool(core.lane_target and core.lane_target['selection']=='right_curve_hold')
        counts['ticks']+=1
        counts['locks']+=locked and not before
        counts['releases']+=before and not locked
        counts['guarded']+=guarded
        counts['positive_while_locked']+=locked and raw>0
        counts['gap_positive_while_locked']+=locked and core.state=='GAP' and raw>0
        counts['stops']+=speed==0
        stream.write(json.dumps(dict(stamp=row['stamp'],source_stamp=row['source_stamp'],
            original_raw=row['command']['steering_raw'],raw=raw,speed=speed,
            state=core.state,guarded=guarded,lock=core.lane_curve_lock,
            selection=core.lane_target))+'\n')
    stream.close()
    core.close()
    if counts['ticks']:
        report['runs'].append(counts)
        print(json.dumps(counts))
        assert counts['positive_while_locked']==0 and counts['gap_positive_while_locked']==0
with open(out+'/summary.json','w') as f:json.dump(report,f,indent=2)
