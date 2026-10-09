"""Replay full saved frames with candidate extraction and an eight-class model."""
import argparse
import glob
import json
import os
import sys
import time
import cv2
import numpy as np
import torch
from torchvision.models import resnet18
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src/ros/signs/scripts')))
from board_roi import extract_board_roi

p=argparse.ArgumentParser()
p.add_argument('--run',required=True)
p.add_argument('--output',required=True)
args=p.parse_args()
cv2.setNumThreads(1)
torch.set_num_threads(2)
c=torch.load(args.run+'/model/best.pt',map_location='cpu',weights_only=True)
labels=c['labels']
model=resnet18(weights=None)
model.fc=torch.nn.Linear(model.fc.in_features,len(labels))
model.load_state_dict(c['state_dict'])
model.cuda().eval()
report={}
details=[]
for split in ('val','test'):
    for label in labels:
        row=dict(n=0,no_candidate=0,correct=0,low=0,background=0,wrong={})
        batch=[]
        sources=[]
        for path in sorted(glob.glob(args.run+'/data/'+split+'/'+label+'/*/frames/*.jpg')):
            row['n']+=1
            candidate=extract_board_roi(cv2.imread(path),True)
            if candidate is None:
                row['no_candidate']+=1
                details.append(dict(path=path,label=label,pred='no_candidate'))
                continue
            crop,bounds=candidate
            rgb=cv2.cvtColor(cv2.resize(crop,(224,224),interpolation=cv2.INTER_AREA),cv2.COLOR_BGR2RGB).astype(np.float32)/255
            batch.append(((rgb-np.array([.485,.456,.406],np.float32))/np.array([.229,.224,.225],np.float32)).transpose(2,0,1))
            sources.append((path,bounds))
        for start in range(0,len(batch),64):
            with torch.no_grad():
                probs=model(torch.tensor(np.array(batch[start:start+64])).cuda()).softmax(1).cpu().numpy()
            for scores,(path,bounds) in zip(probs,sources[start:start+64]):
                pred=labels[int(scores.argmax())]
                score=float(scores.max())
                details.append(dict(path=path,label=label,pred=pred,score=score,bounds=bounds))
                if score<.8:row['low']+=1
                elif pred=='background':row['background']+=1
                elif pred==label:row['correct']+=1
                else:row['wrong'][pred]=row['wrong'].get(pred,0)+1
        report[split+'/'+label]=row
        print(split,label,row,flush=True)
with open(args.output,'w') as f:
    json.dump(dict(summary=report,details=details),f,indent=2)
