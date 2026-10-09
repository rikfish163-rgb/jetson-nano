from __future__ import print_function, division
import json
import os
import sys
import time
import cv2
import numpy as np
from yolo_detector import YoloDetector, preprocess, decode, LABELS

directory=sys.argv[1]
reference_mode=len(sys.argv)>2 and sys.argv[2]=='reference'
manifest=json.load(open(os.path.join(directory,'smoke_manifest.json')))
if reference_mode:
    import torch
    root=os.path.dirname(directory)
    sys.path.insert(0,os.path.join(root,'src/yolov5'))
    from models.experimental import attempt_load
    original=torch.load
    checkpoint=os.path.join(root,'yolo_fast/weights/best.pt')
    def trusted(path,*args,**kwargs):
        assert os.path.realpath(path)==os.path.realpath(checkpoint)
        kwargs['weights_only']=False
        return original(path,*args,**kwargs)
    torch.load=trusted
    torch.set_num_threads(2)
    model=attempt_load(checkpoint,device=torch.device('cuda:0'),inplace=False,fuse=True).eval()
    def raw(blob):
        with torch.no_grad():return model(torch.from_numpy(blob).cuda())[0].cpu().numpy()
else:
    detector=YoloDetector(os.path.join(directory,'yolov5s_640_fp16.engine'))
    raw=detector.predict_raw
    # Warm up the real fixed-size engine before measuring steady-state latency.
    for _ in range(5):raw(np.zeros((1,3,640,640),np.float32))
rows=[];times=[]
for item in manifest:
    frame=cv2.imread(os.path.join(directory,'smoke_images',item['file']))
    if frame is None:raise RuntimeError('Missing image '+item['file'])
    started=time.time()
    blob,ratio,padding=preprocess(frame)
    output=raw(blob)
    detections=decode(output,frame.shape,ratio,padding)
    elapsed=1000*(time.time()-started);times.append(elapsed)
    rows.append(dict(file=item['file'],detections=detections,elapsed_ms=elapsed,
                     truth=[LABELS[a['category_id']] for a in item['annotations']]))
    print(item['file'],[(d['label'],round(d['confidence'],4)) for d in detections],round(elapsed,2),flush=True) if sys.version_info[0]>=3 else None
report=dict(rows=rows,mean_ms=float(np.mean(times)),p95_ms=float(np.percentile(times,95)),
            measured_images=len(rows),includes='preprocess + synchronized inference + copy + decode/NMS; excludes disk and ROS')
if not reference_mode:
    references=json.load(open(os.path.join(directory,'reference.json')))['rows']
    confidence_diff=[];coordinate_diff=[]
    for a,b in zip(rows,references):
        assert a['file']==b['file']
        aa=sorted(a['detections'],key=lambda d:(d['label'],d['bounds'][0]))
        bb=sorted(b['detections'],key=lambda d:(d['label'],d['bounds'][0]))
        assert len(aa)==len(bb),(a['file'],len(aa),len(bb))
        for x,y in zip(aa,bb):
            assert x['label']==y['label'],a['file']
            confidence_diff.append(abs(x['confidence']-y['confidence']))
            coordinate_diff.extend(abs(v-w) for v,w in zip(x['bounds'],y['bounds']))
    report['max_confidence_difference']=max(confidence_diff or [0])
    report['max_bound_difference_pixels']=max(coordinate_diff or [0])
    report['matching_detection_counts_and_labels']=True
    assert report['max_confidence_difference']<.03,report['max_confidence_difference']
    assert report['max_bound_difference_pixels']<=3,report['max_bound_difference_pixels']
    detector.close()
name='reference.json' if reference_mode else 'nano_benchmark.json'
with open(os.path.join(directory,name),'w') as stream:json.dump(report,stream,indent=2)
print('REPORT',json.dumps(dict((k,v) for k,v in report.items() if k!='rows')))
