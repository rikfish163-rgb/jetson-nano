"""Launch/checkpoint continuation under torchrun, with logs and exit records."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

root=Path(sys.argv[1]);name=sys.argv[2];py=str(root/'venv/bin/python')
settings={
    'yolo_ddp':([1,2], [str(root/'tools/run_yolo_ddp.py'),str(root)],root),
    'dfine_ddp':([5,6,7,8], ['train.py','-c',str(root/'dfine_ddp.yaml'),'--use-amp','--seed=42','-r',str(root/'ddp_migration/last.pth')],root/'src/D-FINE')}
gpus,args,cwd=settings[name]
cmd=[py,'-u','-m','torch.distributed.run','--standalone','--nproc_per_node='+str(len(gpus))]+args
env=dict(os.environ,CUDA_VISIBLE_DEVICES=','.join(map(str,gpus)),OMP_NUM_THREADS='2',MKL_NUM_THREADS='2',PYTHONUNBUFFERED='1',NCCL_DEBUG='WARN')
record=root/(name+'_job.json')
if '--worker' in sys.argv:
    result=subprocess.run(cmd,cwd=cwd,env=env)
    (root/(name+'.exit')).write_text(str(result.returncode)+'\n')
    sys.exit(result.returncode)
if record.exists():
    old=json.loads(record.read_text());p=Path('/proc')/str(old['pid'])/'stat'
    if p.exists() and p.read_text().rsplit(')',1)[1].split()[0]!='Z':raise RuntimeError('Job already active')
exitfile=root/(name+'.exit')
if exitfile.exists():exitfile.rename(root/(name+'.exit.previous.'+str(int(time.time()))))
with (root/(name+'.log')).open('ab',buffering=0) as log:
    proc=subprocess.Popen([py,__file__,str(root),name,'--worker'],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,cwd=root,env=env)
info=dict(name=name,pid=proc.pid,gpus=gpus,command=cmd,start_time=time.time(),candidate_only=True)
record.write_text(json.dumps(info,indent=2));print(json.dumps(info))
