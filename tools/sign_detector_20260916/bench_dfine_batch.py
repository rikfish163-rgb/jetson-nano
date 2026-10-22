"""Bounded disposable throughput/memory probe; never saves training weights."""
import json
import os
from pathlib import Path
import sys
import time

import torch

root = Path(sys.argv[1])
batch = int(sys.argv[2])
sys.path.insert(0, str(root / 'src/D-FINE'))
from src.core import YAMLConfig

torch.set_num_threads(2)
torch.manual_seed(42)
cfg = YAMLConfig(str(root / 'dfine_ddp.yaml'), use_amp=True)
cfg.yaml_cfg['train_dataloader']['total_batch_size'] = batch
cfg.yaml_cfg['train_dataloader']['num_workers'] = 0
cfg.yaml_cfg['train_dataloader']['collate_fn']['base_size'] = 800
cfg.yaml_cfg['train_dataloader']['collate_fn']['base_size_repeat'] = None
model = cfg.model.cuda().train()
checkpoint = torch.load(root / 'ddp_migration/last.pth', map_location='cpu', weights_only=True)
model.load_state_dict(checkpoint['model'])
criterion = cfg.criterion.cuda().train()
optimizer = cfg.optimizer
scaler = torch.amp.GradScaler('cuda')
samples, targets = next(iter(cfg.train_dataloader))
samples = torch.nn.functional.interpolate(samples.cuda(), size=(800, 800), mode='bilinear', align_corners=False)
targets = [{k: v.cuda() if isinstance(v, torch.Tensor) else v for k, v in t.items()} for t in targets]
assert tuple(samples.shape) == (batch, 3, 800, 800), samples.shape
times = []
torch.cuda.reset_peak_memory_stats()
for step in range(10):
    torch.cuda.synchronize()
    start = time.perf_counter()
    optimizer.zero_grad(set_to_none=True)
    with torch.autocast('cuda'):
        outputs = model(samples, targets=targets)
    losses = criterion(outputs, targets, epoch=16, step=step, global_step=4000+step, epoch_step=100)
    loss = sum(losses.values())
    assert torch.isfinite(loss).item(), loss
    scaler.scale(loss).backward()
    scaler.unscale_(optimizer)
    torch.nn.utils.clip_grad_norm_(model.parameters(), 0.1)
    scaler.step(optimizer)
    scaler.update()
    torch.cuda.synchronize()
    if step >= 3:
        times.append(time.perf_counter() - start)
result = dict(batch=batch, size=800, seconds_per_step=sum(times)/len(times),
              images_per_second=batch/(sum(times)/len(times)),
              peak_allocated_gib=torch.cuda.max_memory_allocated()/2**30,
              peak_reserved_gib=torch.cuda.max_memory_reserved()/2**30,
              measured_steps=len(times), includes_ddp=False)
(root / ('bench_dfine_batch%d.json' % batch)).write_text(json.dumps(result, indent=2))
print('BENCHMARK ' + json.dumps(result), flush=True)
