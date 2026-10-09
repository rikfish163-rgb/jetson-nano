"""Read-only OpenCV ONNX check of a replay case manifest, Python 2/3."""
from __future__ import print_function
import argparse
import json
import os
import sys
import time
import cv2
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src/ros/signs/scripts')))
from board_roi import extract_board_roi
from sign_classifier_cv import SignClassifier

p=argparse.ArgumentParser()
p.add_argument('--manifest',required=True)
p.add_argument('--model-dir',required=True)
p.add_argument('--output',required=True)
args=p.parse_args()
cv2.setNumThreads(1)
model=SignClassifier(os.path.join(args.model_dir,'resnet18_candidate.onnx'),
                     os.path.join(args.model_dir,'labels.json'))
with open(args.manifest) as stream: cases=json.load(stream)
results=[]
for case in cases:
    frame=cv2.imread(case['path'])
    if frame is None: raise RuntimeError('Unreadable case: '+case['path'])
    start=time.time()
    candidate=extract_board_roi(frame,True)
    crop_ms=(time.time()-start)*1000
    pred,score='no_candidate',0.
    if candidate is not None:
        scores=model.classify_scores(candidate[0])
        pred=max(scores,key=scores.get)
        score=scores[pred]
    accepted=pred if score>=.8 and pred not in ('background','no_candidate') else ''
    expected=case['pred'] if case.get('score',0)>=.8 and case['pred'] not in ('background','no_candidate') else ''
    results.append(dict(path=case['path'],label=case['label'],pred=pred,score=score,
                        accepted=accepted,expected=expected,match=accepted==expected,
                        crop_ms=crop_ms,total_ms=(time.time()-start)*1000))
summary=dict(cases=len(results),matches=sum(r['match'] for r in results),
             crop_ms_median=float(np.median([r['crop_ms'] for r in results])),
             total_ms_median=float(np.median([r['total_ms'] for r in results])))
with open(args.output,'w') as stream: json.dump(dict(summary=summary,results=results),stream,indent=2)
print(json.dumps(summary))
sys.exit(0 if summary['matches']==len(results) else 1)
