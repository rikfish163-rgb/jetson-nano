"""Synthetic position/scale stress test using real held-out sign crops.

This is a regression check, not an independent estimate of field accuracy.
Run on the evaluation GPU; never starts ROS or opens a camera.
"""
import argparse
import importlib.util
import json
import cv2
import numpy as np
import torch
from torchvision.models import resnet18


def load_roi(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.extract_board_roi


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run', required=True)
    p.add_argument('--before', required=True)
    p.add_argument('--after', required=True)
    p.add_argument('--output', required=True)
    args = p.parse_args()
    cv2.setNumThreads(1)
    torch.set_num_threads(2)
    before = load_roi(args.before, 'roi_before')
    after = load_roi(args.after, 'roi_after')
    checkpoint = torch.load(args.run+'/model/best.pt', map_location='cpu', weights_only=True)
    labels = checkpoint['labels']
    model = resnet18(weights=None)
    model.fc = torch.nn.Linear(model.fc.in_features, len(labels))
    model.load_state_dict(checkpoint['state_dict'])
    model.cuda().eval()
    baseline = json.load(open(args.run+'/board_replay_v2.json'))['details']
    sources = []
    for label in labels:
        if label == 'background':
            continue
        good = [r for r in baseline if '/val/' in r['path'] and
                r['label'] == label and r['pred'] == label and r.get('score', 0) >= .95]
        sources.extend([good[0], good[len(good)//2]])
    summary, details, batch, rows = {}, [], [], []
    for source in sources:
        path = source['path']
        frame = cv2.imread(path)
        original = before(frame)
        for size in (24, 48, 96, 256):
            ratio = float(size)/max(original.shape[:2])
            patch = cv2.resize(original, (max(1, round(original.shape[1]*ratio)),
                                         max(1, round(original.shape[0]*ratio))),
                               interpolation=cv2.INTER_AREA)
            h, w = patch.shape[:2]
            for row, y in enumerate((0, (360-h)//2, 360-h)):
                for col, x in enumerate((0, (640-w)//2, 640-w)):
                    image = np.full((360, 640, 3), 90, np.uint8)
                    image[y:y+h, x:x+w] = patch
                    key = '%dpx/row%d' % (size, row)
                    result = summary.setdefault(key, dict(n=0, before_found=0, after_found=0,
                                                          accepted_correct=0, wrong=0, rejected=0))
                    result['n'] += 1
                    result['before_found'] += int(before(image) is not None)
                    crop = after(image)
                    result['after_found'] += int(crop is not None)
                    item = dict(source=path, label=source['label'], size=size, row=row, col=col,
                                found=crop is not None)
                    details.append(item)
                    if crop is None:
                        result['rejected'] += 1
                        continue
                    rgb = cv2.cvtColor(cv2.resize(crop, (224, 224), interpolation=cv2.INTER_AREA),
                                       cv2.COLOR_BGR2RGB).astype(np.float32)/255
                    batch.append(((rgb-np.array([.485,.456,.406], np.float32))/
                                  np.array([.229,.224,.225], np.float32)).transpose(2,0,1))
                    rows.append((result, item))
    for start in range(0, len(batch), 64):
        with torch.no_grad():
            probs = model(torch.tensor(np.array(batch[start:start+64])).cuda()).softmax(1).cpu().numpy()
        for scores, (result, item) in zip(probs, rows[start:start+64]):
            label = labels[int(scores.argmax())]
            score = float(scores.max())
            item.update(pred=label, score=score)
            status = ('rejected' if label == 'background' or score < .8 else
                      'accepted_correct' if label == item['label'] else 'wrong')
            result[status] += 1
    with open(args.output, 'w') as stream:
        json.dump(dict(summary=summary, details=details), stream, indent=2)
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
