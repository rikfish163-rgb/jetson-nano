"""Checkpoint-verified handoff of only this experiment's two running process groups."""
import json
import os
from pathlib import Path
import shutil
import signal
import sys
import time
import torch
import yaml

root=Path(sys.argv[1]);sys.path.insert(0,str(root/'src/yolov5'))
dest=root/'ddp_migration';dest.mkdir(exist_ok=False)
records={n:json.loads((root/(n+'_job.json')).read_text()) for n in ('yolo','dfine')}
for name,record in records.items():
    pid=record['pid'];cmd=Path('/proc')/str(pid)/'cmdline'
    if not cmd.exists() or str(root).encode() not in cmd.read_bytes() or os.getpgid(pid)!=pid:
        raise RuntimeError('Refuse to signal unexpected supervisor '+str(pid))
validated={}
frozen=[]
try:
    for name,r in records.items():
        os.killpg(r['pid'],signal.SIGSTOP);frozen.append(r['pid'])
    # If a write was in progress, resume both rather than destroy a partial checkpoint.
    for name,source in [('yolo',root/'yolo/weights/last.pt'),('dfine',root/'dfine/last.pth')]:
        checkpoint=torch.load(source,map_location='cpu',weights_only=False if name=='yolo' else True)
        key='epoch' if name=='yolo' else 'last_epoch'
        assert checkpoint[key]>=0 and checkpoint.get('optimizer') is not None
        validated[name]={'last_epoch':checkpoint[key],'source':str(source),'old_supervisor':records[name]['pid']}
        shutil.copy2(source,dest/source.name)
        del checkpoint
    yolo_out=root/'yolo_ddp';(yolo_out/'weights').mkdir(parents=True,exist_ok=False)
    for file in ('last.pt','best.pt'):
        shutil.copy2(root/'yolo/weights'/file,yolo_out/'weights'/file)
    shutil.copy2(root/'yolo/results.csv',yolo_out/'results.csv')
    cfg=yaml.safe_load((root/'dfine.yaml').read_text())
    cfg['output_dir']=str(root/'dfine_ddp')
    cfg['train_dataloader']['total_batch_size']=32
    cfg['train_dataloader']['num_workers']=2
    cfg['val_dataloader']['total_batch_size']=128
    cfg['val_dataloader']['num_workers']=2
    cfg['print_freq']=20
    (root/'dfine_ddp.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    (root/'dfine_ddp').mkdir(exist_ok=False)
    shutil.copy2(root/'dfine/best_stg1.pth',dest/'dfine_best_single.pth')
    shutil.copy2(dest/'dfine_best_single.pth',root/'dfine_ddp/best_stg1.pth')
    # Keep old best in its original run. New run metrics/best remain separate.
    validated['configuration']={'yolo_gpus':[1,2],'yolo_global_batch':32,'yolo_effective_batch':64,
                                'dfine_gpus':[5,6,7,8],'dfine_global_batch_before':8,'dfine_global_batch_after':32,
                                'learning_rate_policy':'resume saved optimizer; no automatic LR scaling'}
    (dest/'handoff.json').write_text(json.dumps(validated,indent=2))
except BaseException:
    for pid in frozen:os.killpg(pid,signal.SIGCONT)
    raise
for pid in frozen:
    os.killpg(pid,signal.SIGTERM);os.killpg(pid,signal.SIGCONT)
for _ in range(40):
    alive=[]
    for pid in frozen:
        p=Path('/proc')/str(pid)/'stat'
        if p.exists() and p.read_text().rsplit(')',1)[1].split()[0]!='Z':alive.append(pid)
    if not alive:break
    time.sleep(.25)
if alive:
    raise RuntimeError('Supervisors still alive; do not launch duplicate workers: '+str(alive))
print(json.dumps(validated,indent=2),flush=True)
