from __future__ import print_function
import glob, imp, json, os
import cv2
import numpy as np

root='/home/nano/robocup_ws'
run=root+'/field_data/lane_runs/20260929_230116_1410'
out=root+'/field_data/lane_recording_check/exit_left_replay'
if not os.path.isdir(out):os.makedirs(out)
sources=[('/home/nano/robocup_cleanup_backups/20260929/before_dash_gap_fix/camera_yihan_web.py','before'),
         (root+'/src/ros/camera/scripts/camera_yihan_web.py','after')]
cal=json.loads(open(run+'/calibration.jsonl').readline())['data']
cameras=[]
for path,name in sources:
    c=imp.load_source('exit_'+name,path)
    for key in ('K','D','H'):
        if key in cal:setattr(c,key,np.asarray(cal[key],dtype=np.float64))
    c.init_undistort_maps()
    cameras.append(c)
counts=dict(frames=0,unreadable=0,before_no_path=0,after_no_path=0,
            before_right_reference=0,after_right_reference=0,changed=0)
rows=open(out+'/frames.jsonl','w')
for path in sorted(glob.glob(run+'/*_front.jpg')):
    image=cv2.imread(path)
    if image is None:
        counts['unreadable']+=1
        continue
    stamp=int(os.path.basename(path).split('_')[0])/1e9
    mask=cameras[0].make_metric_bev(cameras[0].detect_white_line(image))
    row=dict(stamp=stamp,file=os.path.basename(path))
    for c,name in zip(cameras,('before','after')):
        tracked=c.track_metric_lane(mask,source_stamp=stamp)
        selected=c.select_nearest_lane_path_segment(tracked['center_segments'])
        points=[c.bev_point_to_vehicle_m(p['x'],p['y']) for p in selected]
        target=max(points,key=lambda p:p[0]) if points else None
        ref=next((p.get('reference_side') for p in tracked['center_points'] if p),None)
        row[name]=dict(points=points,target=target,reference=ref,dashed=tracked['dashed_left'],
                       raw=int(round(22*max(-1.,min(1.,target[1]/.075)))) if target else None)
        counts[name+'_no_path']+=not bool(points)
        counts[name+'_right_reference']+=ref=='right'
    counts['frames']+=1
    counts['changed']+=row['before']!=row['after']
    rows.write(json.dumps(row)+'\n')
    if abs(stamp-1790694132.4520724)<.001:print(json.dumps(row))
rows.close()
with open(out+'/summary.json','w') as f:json.dump(counts,f,indent=2)
print(json.dumps(counts))
