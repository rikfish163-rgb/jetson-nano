"""Replay recorded local paths, not counterfactual vehicle trajectories."""
from __future__ import print_function
import imp
import json
import os
from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.master.controller import Controller

root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
recording = os.path.join(root,'field_data/lane_runs/20260930_040314_16505')
old_lane = imp.load_source('old_lane_replay', os.path.join(root,
    'field_data/code_backups/20260930_curve_stability/src/robot/lane/controller.py'))
cfg = load_config(os.path.join(root,'src/robot/config'))
cfg.update(wait_green=False,lidar_enabled=False,steering_command_scale_rad=.03,
           lane_curvature_preview=False,lane_curve_speed_raw=12)
before = Controller(cfg)
before._runtime.modules['lane'] = old_lane
cfg = dict(cfg, lane_curvature_preview=True)
after = Controller(cfg)
observations = {}
with open(os.path.join(recording,'observation.jsonl')) as stream:
    for line in stream:
        observation = json.loads(line)['data']
        observations[observation['stamp']] = observation
rows = []
rendered_sources = set()
events = 0
with open(os.path.join(recording,'target.jsonl')) as stream:
    for line in stream:
        event = json.loads(line)['data']
        events += 1
        selection = event.get('selection') or {}
        if selection.get('path'): rendered_sources.add(event['source_stamp'])
        if event['state'] not in ('LANE','GAP') or not selection.get('path'):
            continue
        stamp,now = event['source_stamp'],event['stamp']
        observation = observations.get(stamp)
        if observation is None:
            continue
        row = dict(stamp=now, source_stamp=stamp, confidence=observation['confidence'],
                   recorded_speed=event['command']['speed_raw'],recorded_raw=event['command']['steering_raw'])
        for label,controller in (('before',before),('after',after)):
            controller.observe_lane(selection['path'],observation['confidence'],stamp)
            speed,steer = controller.lane_command(now)
            controller.issued_steer = steer
            row[label+'_speed'] = speed
            row[label+'_raw'] = encode_command(speed,steer,controller.cfg,0)['steering_raw']
        row['preview'] = after.lane_preview
        rows.append(row)
before.close()
after.close()
with open(os.path.join(os.path.dirname(__file__),'replay.jsonl'),'w') as stream:
    for row in rows: stream.write(json.dumps(row)+'\n')
entry = min(rows,key=lambda row:abs(row['stamp']-1790712276.010137))
summary = dict(scope='same recorded local paths; no simulated physical trajectory or lidar freshness',
               rows=len(rows), numeric_target_events=events,
               old_target_images=events, new_target_images=len(rendered_sources),
               recorded_entry=entry)
with open(os.path.join(os.path.dirname(__file__),'replay_summary.json'),'w') as stream:
    json.dump(summary,stream,indent=2)
print(json.dumps(summary,indent=2))
