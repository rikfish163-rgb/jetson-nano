"""Shared data contract; compatible with Nano Python 2 and training Python 3."""
from __future__ import division
import glob
import hashlib
import json
import os

LABELS = ['red','green','straight','left','right','uturn','park']
EXTENSIONS = ('.jpg','.jpeg','.png')


def early_stop_due(epoch, best_epoch, patience):
    return patience > 0 and epoch - best_epoch >= patience


def dataset_digest(root, labels=LABELS):
    """Bind operator review to the exact ROI bytes and their split/label/session."""
    manifest = []
    for split in ('train', 'val', 'test'):
        for label in labels:
            for path in sorted(glob.glob(os.path.join(root, split, label, '*', 'roi', '*'))):
                if not path.lower().endswith(EXTENSIONS): continue
                if os.path.islink(path) or not os.path.isfile(path):
                    raise ValueError('ROI must be a regular file: '+path)
                manifest.append([os.path.relpath(path, root), file_sha256(path)])
    payload = json.dumps(manifest, ensure_ascii=True, separators=(',', ':')).encode('ascii')
    return hashlib.sha256(payload).hexdigest()


def file_sha256(path):
    h = hashlib.sha256()
    with open(path,'rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):
            h.update(block)
    return h.hexdigest()


def samples(root, split, labels=LABELS):
    rows = []
    for index,label in enumerate(labels):
        paths = sorted(glob.glob(os.path.join(root,split,label,'*','roi','*')))
        paths = [p for p in paths if p.lower().endswith(EXTENSIONS)]
        if not paths:
            raise ValueError('Missing class: %s/%s (collect it before training)' % (split,label))
        rows.extend((p,index) for p in paths)
    return rows


def legacy_samples(root):
    return [(p,i) for i,label in enumerate(LABELS)
            for p in sorted(glob.glob(os.path.join(root,label,'*')))
            if p.lower().endswith(EXTENSIONS)]


def check_overlap(groups):
    """Reject byte-identical images crossing splits. Sessions must also differ."""
    seen = {}
    for split,rows in groups.items():
        for path,_ in rows:
            digest = file_sha256(path)
            if digest in seen and seen[digest][0] != split:
                raise ValueError('Split leakage: %s and %s' % (seen[digest][1],path))
            seen[digest] = (split,path)


def metrics(matrix, labels=LABELS):
    size = len(labels)
    if len(matrix) != size or any(len(row) != size for row in matrix):
        raise ValueError('Confusion matrix shape does not match labels')
    totals = [sum(row) for row in matrix]
    recall = [matrix[i][i]/float(max(1,totals[i])) for i in range(size)]
    total = sum(totals)
    return dict(labels=labels,confusion=matrix,samples=totals,
                accuracy=sum(matrix[i][i] for i in range(size))/float(max(1,total)),
                recall=dict(zip(labels,recall)),macro_recall=sum(recall)/float(size),
                park_to_uturn=matrix[6][5]/float(max(1,totals[6])),
                uturn_to_park=matrix[5][6]/float(max(1,totals[5])))
