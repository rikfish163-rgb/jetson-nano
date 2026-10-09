"""Bounded NCCL correctness and throughput test, no dataset or model changes."""
import datetime
import json
import os
import sys
import time
import torch
import torch.distributed as dist

local=int(os.environ['LOCAL_RANK'])
torch.cuda.set_device(local)
backend='cpu:gloo,cuda:nccl' if '--dual' in sys.argv else 'nccl'
dist.init_process_group(backend,timeout=datetime.timedelta(seconds=60),device_id=torch.device('cuda',local))
world=dist.get_world_size()
if '--dual' in sys.argv:
    sizes=[torch.zeros(1,dtype=torch.int64) for _ in range(world)]
    dist.all_gather(sizes,torch.tensor([dist.get_rank()],dtype=torch.int64))
    assert [int(s.item()) for s in sizes]==list(range(world))
x=torch.ones(8*1024*1024,device='cuda')
dist.all_reduce(x)
assert float(x[0])==world
for _ in range(3):
    x.fill_(1);dist.all_reduce(x)
torch.cuda.synchronize();start=time.perf_counter()
for _ in range(10):
    x.fill_(1);dist.all_reduce(x)
torch.cuda.synchronize()
elapsed=time.perf_counter()-start
if dist.get_rank()==0:print(json.dumps({'world_size':world,'all_reduce_32MiB_ms':elapsed*100,'correct':True}),flush=True)
dist.destroy_process_group()
