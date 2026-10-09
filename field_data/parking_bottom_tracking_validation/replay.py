"""Nano Python 2: saved images with interpolated recorded command poses, no ROS nodes."""
import bisect
import glob
import json
import os
import tarfile
import cv2
from robot.common.config import load_config
from robot.common.geometry import world, wrap
from robot.camera.vision import GroundDetector
from robot.parking.forward import ForwardParking

ROOT='/home/nano/robocup_ws'
os.chdir(ROOT)
rows=[]
log='/home/nano/.ros/log/a9c4a128-b4ed-11f1-b49d-380025ec2b29/competition_controller-7.log'
for line in open(log):
    if 'competition_state {' not in line:continue
    try:r=json.JSONDecoder().raw_decode(line.split('competition_state ',1)[1])[0]
    except ValueError:continue
    rows.append(r)
stamps=[r['stamp'] for r in rows]
seed=next(r for r in rows if r.get('parking_entry') and r['parking_entry'].get('bottom_local'))
end=next(r for r in rows if r['reason']=='parking_at_bottom_clearance')
start=seed['stamp']-seed['parking_entry']['bottom_age_s']

def pose_at(t):
    i=max(1,min(len(rows)-1,bisect.bisect_left(stamps,t)))
    a,b=rows[i-1],rows[i]
    f=max(0.,min(1.,(t-a['stamp'])/(b['stamp']-a['stamp'])))
    return tuple(a['pose'][j]+f*(b['pose'][j]-a['pose'][j]) for j in (0,1))+(a['pose'][2]+f*wrap(b['pose'][2]-a['pose'][2]),)

scope={}
with tarfile.open('/home/nano/refactor_backups/before-parking-bottom-tracking-BIUMPc/before.tar.gz') as archive:
    exec compile(archive.extractfile('src/robot/parking/forward.py').read(),'before_forward.py','exec') in scope
cfg=load_config('src/robot/config')
detector=GroundDetector(cfg)
models=[]
for cls in (scope['ForwardParking'],ForwardParking):
    p=cls(cfg,pose_at(start),start)
    p.phase='CENTER'
    p.sign_anchor=world(seed['pose'],seed['parking_entry']['sign_anchor_local'])
    p.bottom=[world(seed['pose'],q) for q in seed['parking_entry']['bottom_local']]
    p.bottom_stamp=p.stamp=start
    p.bottom_source='p_near_cross'
    if hasattr(p,'remember_bottom'):p.remember_bottom(p.bottom,start,p.bottom_source)
    models.append(p)
events=[]
for path in sorted(glob.glob('field_data/sign_capture/20260920_202042_1547/*_PARKING_frame.png')):
    t=float(os.path.basename(path).split('_')[0])
    if not start<t<=end['stamp']:continue
    bev=detector.bev(cv2.imread(path),parking=True)
    white=cv2.inRange(cv2.cvtColor(bev,cv2.COLOR_BGR2HSV),(0,0,cfg['white']['v_min']),(179,cfg['white']['s_max'],255))
    lines=detector.parking_lines(white,u_offset=360)
    pose=pose_at(t)
    event=dict(stamp=t,frame=path)
    for name,p in zip(('before','after'),models):
        p.observe([[world(pose,q) for q in line] for line in lines],t,pose)
        command=p.command(t,pose)
        event[name]=dict(command=command,status=p.status,bottom_age_s=t-p.bottom_stamp,
                         clearance_m=p.debug.get('bumper_clearance_m'),source=p.bottom_source)
    events.append(event)
# Also test the actual recorded finish tick, just after the last image.
result=dict(kind='saved_images_with_interpolated_command_model_poses',frames=len(events),events=events,
    recorded_finish=dict(stamp=end['stamp'],state=end['state'],parking_entry=end['parking_entry']))
for name,p in zip(('before','after'),models):
    command=p.command(end['stamp'],end['pose'])
    result[name]=dict(command=command,status=p.status,debug=p.debug)
json.dump(result,open('field_data/parking_bottom_tracking_validation/replay.json','w'),indent=2)
print(json.dumps(dict(frames=len(events),before=result['before'],after=result['after'])))
# Sign captures and interpolated poses are not the original ground stream;
# they reproduce stale association, not the exact original finish tick.
assert end['state']=='FINISHED'
assert end['stamp']-models[0].bottom_stamp>3.
assert result['before']['debug']['bumper_clearance_m']<.15
assert result['after']['status'] is None
assert result['after']['command'][0]>0
assert result['after']['debug']['bumper_clearance_m']>.20
assert end['stamp']-models[1].bottom_stamp<.25
