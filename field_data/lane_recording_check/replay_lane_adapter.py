from __future__ import print_function
import os,json,imp,threading,collections
from robot.common.config import load_config
from robot.master.controller import Controller
from test_ros_adapter import _Clock,_FakeRospy,_BoolMessage

root='/home/nano/robocup_ws'
sources=[('before','/home/nano/robocup_cleanup_backups/20260929/before_lane_adapter_fix/controller_node.py'),
         ('after',root+'/src/robot/master/controller_node.py')]
results=[]
for run in ('20260929_224831_28947','20260929_225212_31193'):
    folder=root+'/field_data/lane_runs/'+run
    observations=[json.loads(s) for s in open(folder+'/observation.jsonl')]
    ticks=[json.loads(s)['data'] for s in open(folder+'/target.jsonl')]
    events=[(r['received'],0,r['data']) for r in observations]
    events += [(r['stamp'],1,r) for r in ticks if r['state'] in ('LANE','GAP')]
    events.sort(key=lambda e:(e[0],e[1]))
    result=dict(run=run,observations=len(observations),versions={})
    for name,path in sources:
        module=imp.load_source('adapter_replay_'+name,path)
        clock=_Clock(0.)
        fake=_FakeRospy(clock)
        module.rospy=fake
        node=module.Node.__new__(module.Node)
        node.cfg=load_config(root+'/src/robot/config')
        node.core=Controller(node.cfg)
        node.lock=threading.RLock()
        node.source_pose=lambda stamp:(0.,0.,0.)
        counts=dict(accepted=0,rejected=0,normal_ticks=0,stale_ticks=0,reasons={})
        reasons=collections.Counter()
        for now,kind,data in events:
            clock.value=now
            if kind==0:
                previous=len(fake.warnings)
                node.lane_observation(_BoolMessage(json.dumps(data)))
                if node.core.lane_stamp==data['stamp']:
                    counts['accepted']+=1
                else:
                    counts['rejected']+=1
                    if len(fake.warnings)>previous:
                        reasons[str(fake.warnings[-1])]+=1
            else:
                counts['normal_ticks']+=1
                if not 0<=now-node.core.lane_stamp<=node.cfg['sensor_timeout']:
                    counts['stale_ticks']+=1
        counts['reasons']=dict(reasons)
        node.core.close()
        result['versions'][name]=counts
    results.append(result)
report=dict(note='Recorder receive times approximate adapter arrival; normal LANE/GAP ticks only. No motion.',runs=results)
with open(root+'/field_data/lane_recording_check/lane_adapter_replay.json','w') as f:json.dump(report,f,indent=2)
print(json.dumps(report,indent=2))
