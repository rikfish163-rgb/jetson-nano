"""Resume YOLOv5 optimizer/EMA under DDP without main() restoring old device options."""
import argparse
import datetime
import os
from pathlib import Path
import sys
import warnings
import numpy as np
import torch
import torch.distributed as dist
import yaml

root=Path(sys.argv[1]);repo=root/'src/yolov5'
os.environ['YOLOv5_AUTOINSTALL']='false';os.environ['WANDB_DISABLED']='true'
warnings.filterwarnings('ignore',category=FutureWarning,message='.*torch.cuda.amp.*')
if not hasattr(np,'trapz'):np.trapz=np.trapezoid
original=torch.load
def trusted_load(path,*args,**kwargs):
    path=Path(path).resolve()
    if not any(path.is_relative_to(root/d) for d in ('weights','yolo','yolo_ddp')):
        raise ValueError('Unexpected checkpoint path: '+str(path))
    kwargs['weights_only']=False
    return original(path,*args,**kwargs)
torch.load=trusted_load
sys.path.insert(0,str(repo));os.chdir(repo)
import train
from utils.callbacks import Callbacks
train.check_amp=lambda model:True
local=int(os.environ['LOCAL_RANK']);torch.cuda.set_device(local)
torch.set_num_threads(2)
dist.init_process_group('nccl',timeout=datetime.timedelta(minutes=5),device_id=torch.device('cuda',local))
options=yaml.safe_load((root/'yolo/opt.yaml').read_text())
options.update(weights=str(root/'yolo_ddp/weights/last.pt'),resume=True,cfg='',batch_size=32,
               device='0,1',workers=2,save_dir=str(root/'yolo_ddp'),project=str(root),name='yolo_ddp',
               exist_ok=True,noplots=True)
opt=argparse.Namespace(**options)
if dist.get_rank()==0:print('DDP resume: 2 ranks; global batch 32; effective optimizer batch 64; preserved checkpoint optimizer and EMA',flush=True)
try:
    train.train(opt.hyp,opt,torch.device('cuda',local),Callbacks())
finally:
    dist.destroy_process_group()
