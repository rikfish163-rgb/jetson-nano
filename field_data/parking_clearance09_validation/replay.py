"""Saved images + interpolated command poses; perception comparison, no motion."""
import bisect
import glob
import json
import os
import tarfile
from collections import Counter
import cv2
from robot.common.config import load_config
from robot.common.geometry import world, wrap
from robot.camera.vision import GroundDetector
from robot.parking.forward import ForwardParking

os.chdir('/home/nano/robocup_ws')
out='field_data/parking_clearance09_validation'
rows=json.load(open(out+'/recorded_states.json'))
rows.sort(key=lambda r:r['stamp'])
stamps=[r['stamp'] for r in rows]
seed=next(r for r in rows if r.get('parking_entry'))
end=next(r for r in rows if r['reason']=='parking_at_bottom_clearance')

def pose_at(t):
    i=max(1,min(len(rows)-1,bisect.bisect_left(stamps,t)))
    a,b=rows[i-1],rows[i]
    f=max(0.,min(1.,(t-a['stamp'])/(b['stamp']-a['stamp'])))
    return tuple(a['pose'][j]+f*(b['pose'][j]-a['pose'][j]) for j in (0,1))+(a['pose'][2]+f*wrap(b['pose'][2]-a['pose'][2]),)

scope={}
with tarfile.open('/home/nano/refactor_backups/before-parking-clearance09-2e0sAf/before.tar.gz') as archive:
    exec compile(archive.extractfile('src/robot/parking/forward.py').read(),'before_forward.py','exec') in scope
cfg=load_config('src/robot/config')
detector=GroundDetector(cfg)
# Keep speeds/clearance equal here to isolate pair association, not emulate a
# new driven trajectory from pictures recorded at the old speed.
cfg.update(parking_final_speed_raw=16,parking_bottom_clearance_m=.04)
models=[]
for cls in (scope['ForwardParking'],ForwardParking):
    p=cls(dict(cfg),seed['pose'],seed['stamp'])
    p.sign_anchor=world(seed['pose'],seed['parking_entry']['sign_anchor_local'])
    models.append(p)
events=[]
for path in sorted(glob.glob('field_data/sign_capture/20260920_234812_23760/*_PARKING_frame.png')):
    t=float(os.path.basename(path).split('_')[0])
    if not seed['stamp']<t<=end['stamp']:continue
    bev=detector.bev(cv2.imread(path),parking=True)
    white=cv2.inRange(cv2.cvtColor(bev,cv2.COLOR_BGR2HSV),(0,0,cfg['white']['v_min']),(179,cfg['white']['s_max'],255))
    lines=detector.parking_lines(white,u_offset=360)
    pose=pose_at(t)
    event=dict(stamp=t,frame=path)
    for name,p in zip(('before','after'),models):
        p.observe([[world(pose,q) for q in line] for line in lines],t,pose)
        command=p.command(t,pose)
        event[name]=dict(command=command,reason=p.reason,view=p.view,debug=p.debug)
    events.append(event)
result=dict(kind='saved_images_interpolated_command_poses_not_counterfactual_motion',events=events)
for name in ('before','after'):
    result[name]=dict(Counter(e[name]['reason'] for e in events))
with open(out+'/replay.json','w') as stream:json.dump(result,stream,indent=2)
print(json.dumps(dict(frames=len(events),before=result['before'],after=result['after'])))
