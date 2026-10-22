"""Offline audit of recorded stop commands; no ROS publishers or motion."""
from __future__ import print_function
import collections
import copy
import json
import os
from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.master.controller import Controller

ROOT = '/home/nano/robocup_ws'
RUN = ROOT+'/field_data/lane_runs/20260930_025754_5783'
OUT = ROOT+'/field_data/lane_recording_check/latest_025754_audit'
rows = [json.loads(line)['data'] for line in open(RUN+'/target.jsonl')]
first = [r for r in rows if r.get('curve_lock') and
         r['curve_lock']['started'] == 1790708315.434699]
episodes = []
for i, row in enumerate(first):
    if row['command']['speed_raw'] != 0:
        continue
    end = first[i+1]['stamp'] if i+1 < len(first) else row['stamp']
    if (episodes and episodes[-1]['end'] == row['stamp'] and
            episodes[-1]['reason'] == row['reason']):
        episodes[-1]['end'] = end
        episodes[-1]['ticks'] += 1
    else:
        episodes.append(dict(start=row['stamp'],end=end,ticks=1,
                             reason=row['reason']))

cfg = load_config(ROOT+'/src/robot/config')
cfg.update(wait_green=False,steering_command_scale_rad=.03)
c = Controller(cfg)
replayed = []
try:
    for row in rows:
        if row['command']['speed_raw'] != 0:
            continue
        c.state = row['state']
        c.action = None
        c.lane_curve_lock = copy.deepcopy(row.get('curve_lock'))
        speed, steer = c.stop(row['reason'])
        new = encode_command(speed,steer,cfg,0)
        expected = (-22 if row.get('curve_lock') and
                    row['state'] in ('LANE','GAP') and
                    row['reason'] in ('lane_stream_stale','scan_missing_or_stale')
                    else 0)
        assert new['speed_raw'] == 0
        assert new['steering_raw'] == expected
        replayed.append(dict(stamp=row['stamp'],reason=row['reason'],
                             original=row['command'],after=new))
finally:
    c.close()

summary = dict(
    run=os.path.basename(RUN), first_lock_ticks=len(first),
    first_lock_start=first[0]['curve_lock']['started'],
    first_lock_last_tick=first[-1]['stamp'],
    first_lock_commands=dict(collections.Counter(
        '%s/%s'%(r['command']['speed_raw'],r['command']['steering_raw']) for r in first)),
    first_lock_zero_episodes=episodes,
    first_lock_zero_duration_s=sum(e['end']-e['start'] for e in episodes),
    stop_snapshots_checked=len(replayed),
    stop_snapshots_changed=sum(r['after']['steering_raw'] !=
                              r['original']['steering_raw'] for r in replayed),
    evidence_limit='Recorded stop-state snapshots only. No new trajectory or measured wheel angle.',
    initial_motion_tick=next(r['stamp'] for r in rows if r['command']['speed_raw'] > 0))
with open(OUT+'/summary.json','w') as f:json.dump(summary,f,indent=2,sort_keys=True)
with open(OUT+'/stop_replay.jsonl','w') as f:
    for row in replayed:f.write(json.dumps(row)+'\n')
print(json.dumps(summary,indent=2,sort_keys=True))
