"""Fine-tune existing ResNet18 on deployment-like crops. Candidate only."""
import argparse
import collections
import json
from pathlib import Path
import random
import time
import cv2
import numpy as np
import torch
from torch.utils.data import Dataset,DataLoader,WeightedRandomSampler
from torchvision.models import resnet18

LABELS=['red','green','straight','left','right','uturn','park','background']

def crop_image(image,box,margin):
    if box is None:return image
    x,y,w,h=box;px=max(2,round(w*margin));py=max(2,round(h*margin))
    return image[max(0,y-py):min(image.shape[0],y+h+py),max(0,x-px):min(image.shape[1],x+w+px)]

class Images(Dataset):
    def __init__(self,rows,augment,stress=False):self.rows,self.augment,self.stress=rows,augment,stress
    def __len__(self):return len(self.rows)
    def __getitem__(self,i):
        r=self.rows[i];im=cv2.imread(r['image'])
        if im is None:raise IOError(r['image'])
        box=r['box']
        if box is None and r['candidates']:
            # Background hard negatives from actual cone/blue-floor proposals.
            boxes=[c['box'] for c in r['candidates']]
            box=random.choice(boxes) if self.augment else boxes[0]
        margin=random.uniform(.04,.30) if self.augment else ([.04,.15,.30][i%3] if self.stress else .15)
        im=crop_image(im,box,margin)
        if self.stress or (self.augment and random.random()<.6):
            size=[16,24,32][i%3] if self.stress else random.randint(16,48)
            h,w=im.shape[:2]
            im=cv2.resize(im,(max(4,round(w*size/min(h,w))),max(4,round(h*size/min(h,w)))),interpolation=cv2.INTER_AREA)
            if self.augment and random.random()<.5:im=cv2.GaussianBlur(im,(3,3),random.uniform(.3,.8))
        # Match production INTER_AREA exactly at validation/inference. Train on
        # both interpolations to avoid another small-image interpolation shortcut.
        interpolation=cv2.INTER_LINEAR if self.augment and random.random()<.5 else cv2.INTER_AREA
        im=cv2.resize(im,(224,224),interpolation=interpolation)
        if self.augment:
            # Small geometric jitter; never mirror direction signs.
            m=cv2.getRotationMatrix2D((112,112),random.uniform(-8,8),random.uniform(.95,1.05))
            im=cv2.warpAffine(im,m,(224,224),borderMode=cv2.BORDER_REPLICATE)
            if random.random()<.35:
                src=np.float32([[0,0],[223,0],[223,223],[0,223]])
                dst=src+np.random.uniform(-10,10,(4,2)).astype(np.float32)
                im=cv2.warpPerspective(im,cv2.getPerspectiveTransform(src,dst),(224,224),borderMode=cv2.BORDER_REPLICATE)
            im=np.clip(im.astype(np.float32)*random.uniform(.75,1.2)+random.uniform(-10,10),0,255).astype(np.uint8)
        x=im[:,:,::-1].astype(np.float32)/255
        x=(x-np.array([.485,.456,.406],np.float32))/np.array([.229,.224,.225],np.float32)
        return torch.from_numpy(np.ascontiguousarray(x.transpose(2,0,1))),LABELS.index(r['label'])

def report(matrix):
    recall=np.diag(matrix)/np.maximum(matrix.sum(1),1)
    precision=np.diag(matrix)/np.maximum(matrix.sum(0),1)
    return dict(confusion=matrix.tolist(),recall=dict(zip(LABELS,recall.tolist())),precision=dict(zip(LABELS,precision.tolist())),macro_recall=float(recall.mean()),false_park=int(matrix[:,6].sum()-matrix[6,6]))

def main():
    p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--checkpoint',required=True);p.add_argument('--epochs',type=int,default=60);p.add_argument('--patience',type=int,default=12);p.add_argument('--out-name',default='refiner');a=p.parse_args()
    root=Path(a.root);out=root/a.out_name;out.mkdir(exist_ok=False)
    cv2.setNumThreads(1);torch.set_num_threads(4)
    random.seed(42);np.random.seed(42);torch.manual_seed(42)
    rows=json.loads((root/'training_data/manifest.json').read_text())
    groups={s:[r for r in rows if r['split']==s] for s in ('train','val','test')}
    sizes=collections.Counter(r['group'] for r in groups['train'])
    class_groups={l:len(set(r['group'] for r in groups['train'] if r['label']==l)) for l in LABELS}
    weights=[1/(sizes[r['group']]*class_groups[r['label']]) for r in groups['train']]
    sampler=WeightedRandomSampler(weights,len(weights),replacement=True)
    loaders={s:DataLoader(Images(rr,s=='train'),batch_size=64,num_workers=4,pin_memory=True,sampler=sampler if s=='train' else None) for s,rr in groups.items()}
    loaders['val_stress']=DataLoader(Images(groups['val'],False,True),batch_size=64,num_workers=4,pin_memory=True)
    model=resnet18(weights=None);model.fc=torch.nn.Linear(512,8)
    ckpt=torch.load(a.checkpoint,map_location='cpu',weights_only=True)
    assert ckpt['labels']==LABELS
    model.load_state_dict(ckpt['state_dict']);model.cuda()
    optimizer=torch.optim.AdamW(model.parameters(),lr=2e-5,weight_decay=1e-4)
    scheduler=torch.optim.lr_scheduler.CosineAnnealingLR(optimizer,a.epochs,eta_min=2e-6)
    criterion=torch.nn.CrossEntropyLoss();scaler=torch.amp.GradScaler('cuda')
    def evaluate(split):
        model.eval();matrix=np.zeros((8,8),dtype=np.int64)
        with torch.no_grad():
            for x,y in loaders[split]:
                pred=model(x.cuda(non_blocking=True)).argmax(1).cpu().numpy()
                np.add.at(matrix,(y.numpy(),pred),1)
        return report(matrix)
    baseline={'clean':evaluate('val'),'synthetic_stress':evaluate('val_stress')};(out/'baseline_val.json').write_text(json.dumps(baseline,indent=2))
    print('Baseline',json.dumps(baseline),flush=True)
    best=-1;best_epoch=0
    for epoch in range(1,a.epochs+1):
        start=time.time();model.train();loss_sum=0
        for x,y in loaders['train']:
            optimizer.zero_grad(set_to_none=True)
            with torch.autocast('cuda',dtype=torch.float16):loss=criterion(model(x.cuda(non_blocking=True)),y.cuda(non_blocking=True))
            if not torch.isfinite(loss):raise RuntimeError('Nonfinite loss')
            scaler.scale(loss).backward();scaler.step(optimizer);scaler.update();loss_sum+=float(loss.detach())
        scheduler.step();r=evaluate('val');stress=evaluate('val_stress')
        score=.6*r['macro_recall']+.4*stress['macro_recall']-.5*(r['false_park']+stress['false_park'])/(2*len(groups['val']))
        row=dict(epoch=epoch,loss=loss_sum/len(loaders['train']),seconds=time.time()-start,score=score,synthetic_stress=stress,**r)
        with (out/'metrics.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
        print(json.dumps(row),flush=True)
        if score>best:
            best=score;best_epoch=epoch
            torch.save(dict(labels=LABELS,state_dict=model.state_dict(),epoch=epoch),out/'best.pt')
        if epoch-best_epoch>=a.patience:break
    model.load_state_dict(torch.load(out/'best.pt',weights_only=True)['state_dict'])
    result=dict(val=evaluate('val'),synthetic_stress=evaluate('val_stress'),test=evaluate('test'),best_epoch=best_epoch,initial_checkpoint=a.checkpoint,evaluation_status='reused development images and synthetic stress, NOT independent field accuracy')
    (out/'report.json').write_text(json.dumps(result,indent=2))
    model.cpu().eval();example=next(iter(loaders['val']))[0][:1]
    torch.onnx.export(model,example,out/'resnet18_candidate.onnx',input_names=['input'],output_names=['logits'],opset_version=11,dynamo=False)
    import onnx
    net=onnx.load(out/'resnet18_candidate.onnx');onnx.checker.check_model(net)
    dnn=cv2.dnn.readNetFromONNX(str(out/'resnet18_candidate.onnx'));dnn.setInput(example.numpy());actual=dnn.forward()
    with torch.no_grad():expected=model(example).numpy()
    assert np.allclose(actual,expected,atol=.001,rtol=.001)
    print('FINISHED; candidate only; no robot deployment',flush=True)

if __name__=='__main__':main()
