"""Detached jobs with persisted command, PID, log and exit status."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

root=Path(sys.argv[1]);name=sys.argv[2];py=str(root/'venv/bin/python')
jobs={
    'refiner_area':([py,'-u',str(root/'tools/train_refiner.py'),'--root',str(root),'--checkpoint','/home/hts/robodata/runs/20260915_10000/model/best.pt','--out-name','refiner_area'],1,root),
    'refiner':([py,'-u',str(root/'tools/train_refiner.py'),'--root',str(root),'--checkpoint','/home/hts/robodata/runs/20260915_10000/model/best.pt'],1,root),
    'yolo':([py,'-u',str(root/'tools/run_yolo.py'),str(root)],0,root),
    'dfine':([py,'-u','train.py','-c',str(root/'dfine.yaml'),'--use-amp','--seed=42','-t',str(root/'weights/dfine_l_coco.pth')],2,root/'src/D-FINE')}
cmd,gpu,cwd=jobs[name]
env=dict(os.environ,CUDA_VISIBLE_DEVICES=str(gpu),OMP_NUM_THREADS='4',MKL_NUM_THREADS='4',PYTHONUNBUFFERED='1')
record=root/(name+'_job.json')
if '--worker' in sys.argv:
    result=subprocess.run(cmd,cwd=cwd,env=env)
    (root/(name+'.exit')).write_text(str(result.returncode)+'\n')
    sys.exit(result.returncode)
if record.exists():
    old=json.loads(record.read_text())
    try:os.kill(old['pid'],0)
    except ProcessLookupError:pass
    else:raise RuntimeError('Existing supervisor is still alive: '+str(old['pid']))
exitfile=root/(name+'.exit')
if exitfile.exists():exitfile.rename(root/(name+'.exit.previous.'+str(int(time.time()))))
with (root/(name+'.log')).open('ab',buffering=0) as log:
    process=subprocess.Popen([py,__file__,str(root),name,'--worker'],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,cwd=root,env=env)
info=dict(name=name,pid=process.pid,gpu=gpu,command=cmd,start_time=time.time(),status='started_unverified',candidate_only=True)
record.write_text(json.dumps(info,indent=2));print(json.dumps(info))
