import os, glob, imp, json
import cv2
root='/home/nano/robocup_ws'
old=imp.load_source('before_unified','/home/nano/robocup_cleanup_backups/20260929/unified_center_path/src/ros/camera/scripts/camera_yihan_web.py')
new=imp.load_source('after_unified',root+'/src/ros/camera/scripts/camera_yihan_web.py')
new.init_undistort_maps()
report=dict(frames=0,skipped=0,before_valid=0,after_valid=0,recovered=[],regressions=[],invalid=[])
for name in sorted(glob.glob(root+'/field_data/lane_runs/20260929_205852_2178/*_front.jpg')):
    frame=cv2.imread(name)
    if frame is None:
        report['skipped']+=1
        continue
    stamp=int(os.path.basename(name).split('_')[0])/1e9
    mask=new.make_metric_bev(new.detect_white_line(frame))
    outcomes=[]
    for module in (old,new):
        r=module.track_metric_lane(mask,source_stamp=stamp)
        d={}
        p=module.select_nearest_lane_path_segment(r['center_segments'],d)
        outcomes.append(dict(valid=len(p)>=3 and r['tracking_confidence']>=.35,
            diagnostic=d,confidence=r['tracking_confidence'],
            observed={side:[[q['x'],q['y']] for q in r[side+'_points']
                if module.boundary_point_is_reliable(q,True)] for side in ('left','right')}))
        if module is new and abs(stamp-1790686762.8086748)<.001:
            cv2.imwrite(root+'/field_data/lane_recording_check/unified_after.jpg',
                new.make_lane_tracking_debug(mask,r))
            report['example']=dict(stamp=stamp,points=[[v['x'],v['y']] for v in p],
                diagnostic=d,reference_side=next((v['reference_side'] for v in p),None))
    a,b=outcomes
    report['frames']+=1
    report['before_valid']+=int(a['valid'])
    report['after_valid']+=int(b['valid'])
    if a['valid'] and not b['valid']:report['regressions'].append(dict(stamp=stamp,before=a,after=b))
    if b['valid'] and not a['valid']:report['recovered'].append(stamp)
    if not b['valid']:report['invalid'].append(dict(stamp=stamp,after=b))
with open(root+'/field_data/lane_recording_check/unified_replay.json','w') as f:json.dump(report,f,indent=2)
print(json.dumps(report,indent=2))
