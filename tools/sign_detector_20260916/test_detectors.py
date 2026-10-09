"""Evaluate fixed validation-selected weights on the existing test split."""
import json
import os
from pathlib import Path
import sys
import time

import numpy as np
import torch

root = Path(sys.argv[1]).resolve()
name = sys.argv[2]
out = root / ('test_' + name)
out.mkdir(exist_ok=False)
torch.set_num_threads(2)
start = time.time()
annotations = json.loads((root / 'training_data/test.json').read_text())
predictions = []

if name == 'yolo':
    repo = root / 'src/yolov5'
    os.environ['YOLOv5_AUTOINSTALL'] = 'false'
    os.environ['WANDB_DISABLED'] = 'true'
    if not hasattr(np, 'trapz'):
        np.trapz = np.trapezoid
    checkpoint = root / 'yolo_fast/weights/best.pt'
    original_load = torch.load
    def trusted_load(path, *args, **kwargs):
        if Path(path).resolve() != checkpoint:
            raise ValueError('Unexpected checkpoint: ' + str(path))
        kwargs['weights_only'] = False
        return original_load(path, *args, **kwargs)
    torch.load = trusted_load
    sys.path.insert(0, str(repo))
    os.chdir(repo)
    import val
    result, per_class, timing = val.run(
        data=str(root/'training_data/data.yaml'), weights=str(checkpoint),
        task='test', batch_size=32, imgsz=640, device='0', workers=2,
        half=False, save_json=True, plots=False, verbose=True,
        project=str(root), name=out.name, exist_ok=True)
    raw = json.loads((out/'best_predictions.json').read_text())
    image_ids = {Path(i['file_name']).stem: i['id'] for i in annotations['images']}
    for p in raw:
        p['image_id'] = image_ids[str(p['image_id'])]
        predictions.append(p)
    native = dict(metrics=list(result), per_class_map=list(per_class), server_batch_timing_ms=list(timing))
else:
    assert name == 'dfine'
    sys.path.insert(0, str(root/'src/D-FINE'))
    from src.core import YAMLConfig
    from src.solver import TASKS
    from src.solver.det_engine import evaluate
    checkpoint = root/'dfine_fast/best_stg2.pth'
    cfg = YAMLConfig(str(root/'dfine_fast.yaml'), resume=str(checkpoint),
                     output_dir=str(out), device='cuda', use_amp=False)
    loader_cfg = cfg.yaml_cfg['val_dataloader']
    loader_cfg['total_batch_size'] = 16
    loader_cfg['num_workers'] = 2
    loader_cfg['dataset']['img_folder'] = str(root/'training_data/images/test')
    loader_cfg['dataset']['ann_file'] = str(root/'training_data/test.json')
    solver = TASKS[cfg.yaml_cfg['task']](cfg)
    solver.eval()
    original_update = solver.evaluator.update
    def update_and_capture(results):
        for image_id, result in results.items():
            for box, score, label in zip(result['boxes'].detach().cpu().tolist(),
                                         result['scores'].detach().cpu().tolist(),
                                         result['labels'].detach().cpu().tolist()):
                x1,y1,x2,y2 = box
                predictions.append(dict(image_id=int(image_id), category_id=int(label),
                                        bbox=[x1,y1,x2-x1,y2-y1], score=float(score)))
        original_update(results)
    solver.evaluator.update = update_and_capture
    model = solver.ema.module if solver.ema else solver.model
    stats, evaluator = evaluate(model, solver.criterion, solver.postprocessor,
                                solver.val_dataloader, solver.evaluator, solver.device,
                                epoch=-1, use_wandb=False)
    native = {k: list(v) if isinstance(v, np.ndarray) else v for k,v in stats.items()}
    solver.cleanup()

# Use the same COCO evaluator for both detectors, rather than comparing two AP implementations.
from faster_coco_eval import COCO, COCOeval_faster
gt = COCO(str(root/'training_data/test.json'))
dt = gt.loadRes(predictions)
ev = COCOeval_faster(gt, dt, 'bbox')
ev.evaluate()
ev.accumulate()
ev.summarize()
per_class = {}
for k, category in enumerate(annotations['categories']):
    ap = ev.eval['precision'][:,:,k,0,-1]
    ap50 = ap[0]
    per_class[category['name']] = dict(map50_95=float(ap[ap>=0].mean()), map50=float(ap50[ap50>=0].mean()))

# Fixed operating point, not tuned on test data: confidence >= .5, box IoU >= .5.
def iou(a,b):
    x=max(a[0],b[0]);y=max(a[1],b[1]);xx=min(a[0]+a[2],b[0]+b[2]);yy=min(a[1]+a[3],b[1]+b[3])
    inter=max(0,xx-x)*max(0,yy-y)
    return inter/max(a[2]*a[3]+b[2]*b[3]-inter,1e-9)
gts={i['id']:[] for i in annotations['images']}
preds={i['id']:[] for i in annotations['images']}
for a in annotations['annotations']:gts[a['image_id']].append(a)
for p in predictions:
    if p['score']>=.5:preds[p['image_id']].append(p)
counts={c['id']:dict(tp=0,fp=0,fn=0) for c in annotations['categories']}
background_fp_images=0
for image_id, truth in gts.items():
    matched=set()
    if not truth and preds[image_id]:background_fp_images+=1
    for p in sorted(preds[image_id],key=lambda d:d['score'],reverse=True):
        candidates=[(iou(p['bbox'],a['bbox']),j) for j,a in enumerate(truth)
                    if j not in matched and a['category_id']==p['category_id']]
        best=max(candidates,default=(0,-1))
        if best[0]>=.5:
            counts[p['category_id']]['tp']+=1;matched.add(best[1])
        else:counts[p['category_id']]['fp']+=1
    for j,a in enumerate(truth):
        if j not in matched:counts[a['category_id']]['fn']+=1
for c in annotations['categories']:
    n=counts[c['id']]
    per_class[c['name']]['at_confidence_0_5']=dict(n,precision=n['tp']/max(n['tp']+n['fp'],1),recall=n['tp']/max(n['tp']+n['fn'],1))
report=dict(model=name,checkpoint=str(checkpoint),images=len(annotations['images']),
            objects=len(annotations['annotations']),common_coco_metrics=ev.stats.tolist(),
            per_class=per_class,background_images=sum(not a for a in gts.values()),
            background_false_positive_images=background_fp_images,native=native,
            elapsed_seconds=time.time()-start,
            limits='Existing development-source test split; provisional boxes; not an independent field test. Server timings are not Nano latency.')
(out/'report.json').write_text(json.dumps(report,indent=2))
(out/'predictions_coco.json').write_text(json.dumps(predictions))
print('TEST_REPORT '+json.dumps(report),flush=True)
