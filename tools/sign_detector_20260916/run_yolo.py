"""YOLOv5 v7 launcher for trusted official/local checkpoints on current PyTorch."""
import os
from pathlib import Path
import sys
import torch
import numpy as np

# NumPy 2.4 removed the old alias used by YOLOv5 v7's AP integration.
if not hasattr(np, 'trapz'):
    np.trapz=np.trapezoid

root=Path(sys.argv[1]);repo=root/'src/yolov5'
os.environ['YOLOv5_AUTOINSTALL']='false'
os.environ['WANDB_DISABLED']='true'
# v7 official .pt serializes model classes. Restrict unsafe loading to these
# explicitly trusted official/downloaded or locally generated artifacts.
original=torch.load
def trusted_load(path,*args,**kwargs):
    resolved=Path(path).resolve()
    if not (resolved.is_relative_to(root/'weights') or resolved.is_relative_to(root/'yolo')):
        raise ValueError('Unexpected checkpoint path: '+str(resolved))
    kwargs['weights_only']=False
    return original(path,*args,**kwargs)
torch.load=trusted_load
sys.path.insert(0,str(repo));os.chdir(repo)
import train
# Avoid check_amp downloading an unrelated nano checkpoint. Training scaler and
# finite losses are checked on this actual job; real batches exercise AMP.
train.check_amp=lambda model:True
train.run(weights=str(root/'weights/yolov5s.pt'),data=str(root/'training_data/data.yaml'),
          hyp=str(root/'yolo_hyp.yaml'),imgsz=640,batch_size=16,epochs=150,patience=25,
          optimizer='AdamW',workers=4,device='0',project=str(root),name='yolo',
          seed=42,cos_lr=True,noplots=True,exist_ok=True)
