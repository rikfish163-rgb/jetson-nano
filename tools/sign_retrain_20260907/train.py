#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ResNet18 fine-tuning; optional eighth background class, no installation."""
import argparse
import csv
import json
import os
import random
from common import LABELS as BASE_LABELS, samples, legacy_samples, check_overlap, metrics, file_sha256, early_stop_due


def main():
    p = argparse.ArgumentParser(description='七类 ResNet18 训练与旧版 OpenCV ONNX 导出')
    p.add_argument('--data',required=True)
    p.add_argument('--out',required=True)
    p.add_argument('--include-background',action='store_true')
    p.add_argument('--extra-train',help='旧的 label/*.jpg 数据，仅加入训练，不随机拆到验证集')
    p.add_argument('--checkpoint',help='仅使用本工具生成且可信的 best.pt；不是 ONNX')
    p.add_argument('--epochs',type=int,default=30)
    p.add_argument('--patience',type=int,default=15,help='连续多少轮val宏平均召回率未提升则停止；0禁用')
    p.add_argument('--export-only',action='store_true',help='评估并导出本次out目录已有best.pt，不训练')
    p.add_argument('--batch-size',type=int,default=32)
    p.add_argument('--lr',type=float,default=0.0001)
    p.add_argument('--workers',type=int,default=2)
    p.add_argument('--device',choices=['cuda','cpu'],default='cuda')
    p.add_argument('--seed',type=int,default=42)
    p.add_argument('--check-data-only',action='store_true')
    args = p.parse_args()
    LABELS = BASE_LABELS + ['background'] if args.include_background else BASE_LABELS
    groups = {s:samples(args.data,s,LABELS) for s in ('train','val','test')}
    if args.extra_train:
        groups['train'] += legacy_samples(args.extra_train)
    check_overlap(groups)
    print('固定类别顺序:',LABELS)
    for split,rows in groups.items():
        print(split,{label:sum(y==i for _,y in rows) for i,label in enumerate(LABELS)})
    if args.check_data_only:
        print('目录、类别覆盖、跨集合文件重复检查通过；仍需人工检查 ROI 和相似视频帧。')
        return
    if args.epochs < 1 or args.batch_size < 2 or args.lr <= 0 or args.workers < 0 or args.patience < 0:
        p.error('epochs>=1, batch-size>=2, lr>0, workers>=0 required')
    import cv2
    import numpy as np
    import torch
    from torch import nn
    from torch.utils.data import Dataset,DataLoader,WeightedRandomSampler
    from torchvision.models import resnet18,ResNet18_Weights
    import onnx
    cv2.setNumThreads(1)
    torch.set_num_threads(4)
    random.seed(args.seed); np.random.seed(args.seed); torch.manual_seed(args.seed)
    if args.device == 'cuda' and not torch.cuda.is_available():
        raise RuntimeError('CUDA 不可用；先检查环境，或明确指定 --device cpu')
    if args.export_only:
        if not os.path.isfile(os.path.join(args.out,'best.pt')): raise ValueError('Missing best.pt')
    else:
        os.makedirs(args.out,exist_ok=False)
    mean = np.array([.485,.456,.406],np.float32)
    std = np.array([.229,.224,.225],np.float32)

    class Images(Dataset):
        def __init__(self,rows,augment):
            self.rows,self.augment = rows,augment
        def __len__(self):
            return len(self.rows)
        def __getitem__(self,index):
            path,label = self.rows[index]
            bgr = cv2.imread(path)
            if bgr is None:
                raise ValueError('Unreadable image: '+path)
            bgr = cv2.resize(bgr,(224,224),interpolation=cv2.INTER_AREA)
            if self.augment:
                # No horizontal flip: left/right arrows must retain semantics.
                matrix = cv2.getRotationMatrix2D((112,112),random.uniform(-8,8),random.uniform(.95,1.05))
                bgr = cv2.warpAffine(bgr,matrix,(224,224),borderMode=cv2.BORDER_REPLICATE)
                bgr = np.clip(bgr.astype(np.float32)*random.uniform(.85,1.15)+random.uniform(-8,8),0,255).astype(np.uint8)
            rgb = cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB).astype(np.float32)/255
            tensor = torch.from_numpy(np.ascontiguousarray(((rgb-mean)/std).transpose(2,0,1)))
            return tensor,label

    counts = np.bincount([y for _,y in groups['train']],minlength=len(LABELS))
    sampler = WeightedRandomSampler([1.0/counts[y] for _,y in groups['train']],len(groups['train']),replacement=True)
    loaders = {s:DataLoader(Images(rows,s=='train'),batch_size=args.batch_size,
                           sampler=sampler if s=='train' else None,
                           num_workers=args.workers,pin_memory=args.device=='cuda',
                           drop_last=s=='train') for s,rows in groups.items()}
    model = resnet18(weights=None if args.checkpoint or args.export_only else ResNet18_Weights.IMAGENET1K_V1)
    model.fc = nn.Linear(model.fc.in_features,len(LABELS))
    if args.checkpoint:
        checkpoint = torch.load(args.checkpoint,map_location='cpu',weights_only=True)
        if checkpoint['labels'] != LABELS:
            raise ValueError('Checkpoint label order mismatch')
        model.load_state_dict(checkpoint['state_dict'])
    model.to(args.device)
    optimizer = torch.optim.AdamW(model.parameters(),lr=args.lr,weight_decay=.0001)
    criterion = nn.CrossEntropyLoss()
    def evaluate(split):
        matrix = [[0]*len(LABELS) for _ in LABELS]
        model.eval()
        with torch.no_grad():
            for x,y in loaders[split]:
                pred = model(x.to(args.device)).argmax(1).cpu().tolist()
                for truth,guess in zip(y.tolist(),pred):matrix[truth][guess] += 1
        return metrics(matrix,LABELS)
    best = -1
    best_epoch = 0
    print('开始训练：每轮显示各类召回率；只用 val 选模型，test 最后评估一次。')
    for epoch in range(1,1 if args.export_only else args.epochs+1):
        model.train(); loss_sum,batches = 0,0
        for x,y in loaders['train']:
            optimizer.zero_grad(set_to_none=True)
            loss = criterion(model(x.to(args.device)),y.to(args.device))
            loss.backward(); optimizer.step()
            loss_sum += loss.item(); batches += 1
        if not batches:raise ValueError('Training set smaller than batch size')
        report = evaluate('val')
        print('Epoch %d/%d loss=%.4f macro_recall=%.4f P->U=%.4f U->P=%.4f' %
              (epoch,args.epochs,loss_sum/batches,report['macro_recall'],report['park_to_uturn'],report['uturn_to_park']),flush=True)
        print(report['recall'],flush=True)
        if report['macro_recall'] > best:
            best = report['macro_recall']
            best_epoch = epoch
            torch.save(dict(labels=LABELS,state_dict=model.state_dict()),os.path.join(args.out,'best.pt'))
        if early_stop_due(epoch,best_epoch,args.patience):
            print('Early stopping: epoch=%d best_epoch=%d patience=%d' %
                  (epoch,best_epoch,args.patience),flush=True)
            break
    model.load_state_dict(torch.load(os.path.join(args.out,'best.pt'),map_location=args.device,weights_only=True)['state_dict'])
    final = {s:evaluate(s) for s in ('val','test')}
    for split,report in final.items():
        with open(os.path.join(args.out,split+'_confusion.csv'),'w',newline='') as f:
            writer=csv.writer(f);writer.writerow(['true/pred']+LABELS)
            writer.writerows([label]+row for label,row in zip(LABELS,report['confusion']))
    print('导出候选 ONNX：固定 batch=1，opset=11，input/logits，不替换在线文件。')
    model.cpu().eval()
    dummy,_ = Images(groups['val'],False)[0]
    dummy = dummy.unsqueeze(0)
    output = os.path.join(args.out,'resnet18_candidate.onnx')
    torch.onnx.export(model,dummy,output,input_names=['input'],output_names=['logits'],
                      opset_version=11,do_constant_folding=True,dynamo=False)
    graph = onnx.load(output);onnx.checker.check_model(graph)
    meta=graph.metadata_props.add();meta.key='labels';meta.value=json.dumps(LABELS)
    onnx.save(graph,output)
    net=cv2.dnn.readNetFromONNX(output);net.setInput(dummy.numpy(),'input')
    actual=net.forward('logits')
    with torch.no_grad():expected=model(dummy).numpy()
    if actual.shape != (1,len(LABELS)) or not np.allclose(actual,expected,atol=.001,rtol=.001):
        raise RuntimeError('OpenCV/torch logits parity failed; do not install candidate')
    final.update(labels=LABELS,args=vars(args),onnx_sha256=file_sha256(output),
                 onnx_max_abs_error=float(np.max(np.abs(actual-expected))))
    with open(os.path.join(args.out,'report.json'),'w') as f:json.dump(final,f,indent=2)
    print('完成。测试结果:',final['test'])
    print('候选模型:',output,'；仍需 Nano 旧版 OpenCV 验证。')


if __name__ == '__main__':
    main()
