"""Report supervisors and epoch evidence without touching running jobs."""
import csv
import json
from pathlib import Path
import sys
import time

root=Path(sys.argv[1]);result={'checked_at':time.time(),'jobs':{}}
for name in ('yolo','dfine','refiner_area'):
    active=next((n for n in (name+'_fast',name+'_ddp',name) if (root/(n+'_job.json')).exists()),name)
    job=json.loads((root/(active+'_job.json')).read_text())
    proc=Path('/proc')/str(job['pid'])/'stat'
    alive=proc.exists() and proc.read_text().rsplit(')',1)[1].split()[0]!='Z'
    exitfile=root/(active+'.exit')
    code=int(exitfile.read_text()) if exitfile.exists() and exitfile.stat().st_mtime>=job['start_time'] else None
    state='running' if alive else 'complete' if code==0 else 'failed' if code is not None else 'not_running_unknown'
    epochs=None
    if name=='yolo' and (root/active/'results.csv').exists():
        rows=list(csv.DictReader((root/active/'results.csv').open()))
        if rows:epochs=int(float(next(iter(rows[-1].values()))))+1
    elif name=='dfine' and (root/active/'log.txt').exists():
        rows=(root/active/'log.txt').read_text().splitlines()
        if rows:epochs=json.loads(rows[-1])['epoch']+1
    elif name=='refiner_area' and (root/'refiner_area/metrics.jsonl').exists():
        rows=(root/'refiner_area/metrics.jsonl').read_text().splitlines()
        if rows:epochs=json.loads(rows[-1])['epoch']
    migration='fast_migration' if active.endswith('_fast') else 'ddp_migration'
    if epochs is None and active!=name and (root/migration/'handoff.json').exists():
        epochs=json.loads((root/migration/'handoff.json').read_text())[name]['last_epoch']+1
    result['jobs'][name]={'state':state,'active_run':active,'pid':job['pid'],'gpus':job.get('gpus',[job.get('gpu')]),'completed_epochs':epochs,'exit_code':code}
(root/'status.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
