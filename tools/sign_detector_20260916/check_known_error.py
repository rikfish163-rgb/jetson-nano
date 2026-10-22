"""Evaluate a previously observed LEFT->PARK mistake; never used for fitting."""
import json
from pathlib import Path
import sys
import cv2
import numpy as np

root=Path(sys.argv[1]);image=cv2.imread(str(root/'known_left_crop.png'))
assert image is not None
labels=['red','green','straight','left','right','uturn','park','background']
rgb=cv2.cvtColor(cv2.resize(image,(224,224),interpolation=cv2.INTER_AREA),cv2.COLOR_BGR2RGB).astype(np.float32)/255
x=((rgb-np.array([.485,.456,.406],np.float32))/np.array([.229,.224,.225],np.float32)).transpose(2,0,1)[None]
cv2.setNumThreads(1)
result={'truth':'left','purpose':'known-error regression, not independent accuracy','models':{}}
for name,path in [('old','/home/hts/robodata/runs/20260915_10000/model/resnet18_candidate.onnx'),('refined',str(root/'refiner/resnet18_candidate.onnx'))]:
    net=cv2.dnn.readNetFromONNX(path);net.setInput(np.ascontiguousarray(x));logits=net.forward().reshape(-1)
    scores=np.exp(logits-logits.max());scores/=scores.sum()
    result['models'][name]={'prediction':labels[int(scores.argmax())],'scores':dict(zip(labels,map(float,scores)))}
(root/'known_error_regression.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
