"""Create traceable provisional labels; original data and models are read-only."""
import argparse
import collections
import hashlib
import importlib.util
import json
from pathlib import Path
import random

import cv2
import numpy as np
from PIL import Image, ImageDraw

LABELS = ['red', 'green', 'straight', 'left', 'right', 'uturn', 'park', 'background']


def assign_groups(rows, gap=60):
    """Keep contiguous capture bursts together, including old split boundaries."""
    rng = random.Random(42)
    for label in LABELS:
        selected = sorted([r for r in rows if r['label'] == label], key=lambda r: r['stamp'])
        groups = []
        for row in selected:
            if not groups or row['stamp'] - groups[-1][-1]['stamp'] > gap:
                groups.append([])
            groups[-1].append(row)
        if len(groups) < 3:
            raise ValueError('Need three independent capture groups for ' + label)
        total = len(selected)
        best = None
        for _ in range(2000):
            choices = [rng.choices(range(3), [.7, .15, .15])[0] for g in groups]
            counts = [sum(len(g) for g, s in zip(groups, choices) if s == i) for i in range(3)]
            if min(counts) == 0:
                continue
            error = sum(abs(n / total - target) for n, target in zip(counts, [.7, .15, .15]))
            if best is None or error < best[0]:
                best = error, choices
        if best is None:
            raise ValueError('Unable to split ' + label)
        for i, (group, split) in enumerate(zip(groups, best[1])):
            for row in group:
                row.update(group=label + '_%03d' % i, split=['train', 'val', 'test'][split])


def crop(frame, box, fraction=.15):
    x, y, w, h = box
    px, py = max(2, round(w * fraction)), max(2, round(h * fraction))
    return frame[max(0, y-py):min(frame.shape[0], y+h+py), max(0, x-px):min(frame.shape[1], x+w+px)]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--data', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--board-module', required=True)
    a = p.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=False)
    spec = importlib.util.spec_from_file_location('board', a.board_module)
    board = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(board)
    cv2.setNumThreads(1)
    rows = []
    for path in sorted(Path(a.data).glob('*/*/*/frames/*')):
        if path.suffix.lower() not in ('.jpg', '.png', '.jpeg'):
            continue
        old_split, label, session, _, name = path.relative_to(a.data).parts
        frame = cv2.imread(str(path))
        if frame is None:
            raise ValueError('Unreadable: ' + str(path))
        candidates = board.board_candidates(frame)
        expected = label if label in ('red', 'green') else 'blue'
        matches = [c for c in candidates if c['color'] == expected]
        box = list(matches[0]['bounds']) if matches and label != 'background' else None
        flags = []
        if label != 'background' and box is None:
            flags.append('missing_box')
        if len(candidates) > (0 if label == 'background' else 1):
            flags.append('extra_candidates')
        rows.append(dict(path=str(path), label=label, session=session, old_split=old_split,
                         stamp=float(path.stem), box=box, width=frame.shape[1], height=frame.shape[0],
                         candidates=[dict(box=list(c['bounds']), color=c['color']) for c in candidates],
                         sha256=hashlib.sha256(path.read_bytes()).hexdigest(), flags=flags,
                         annotation_status='rule_proposal_not_ground_truth'))
    assign_groups(rows)
    # Exclude byte duplicates across groups from all training/evaluation exports.
    by_hash = collections.defaultdict(list)
    for row in rows:
        by_hash[row['sha256']].append(row)
    for members in by_hash.values():
        if len(set(r['split'] for r in members)) > 1:
            for row in members:
                row['flags'].append('cross_split_duplicate')
    (out/'manifest.json').write_text(json.dumps(rows, indent=2))
    stats = dict(total=len(rows), annotation_status='PROVISIONAL; requires visual audit',
                 evaluation_status='development split; source images previously used',
                 splits={}, flags=dict(collections.Counter(f for r in rows for f in r['flags'])))
    for split in ('train', 'val', 'test'):
        selected = [r for r in rows if r['split'] == split]
        stats['splits'][split] = dict(classes=dict(collections.Counter(r['label'] for r in selected)),
                                     groups=sorted(set(r['group'] for r in selected)))
    stats['size_bins'] = {label: dict(collections.Counter(
        '<16' if min(r['box'][2:]) < 16 else '16-31' if min(r['box'][2:]) < 32 else
        '32-63' if min(r['box'][2:]) < 64 else '64-127' if min(r['box'][2:]) < 128 else '128+'
        for r in rows if r['label'] == label and r['box'])) for label in LABELS[:-1]}
    (out/'stats.json').write_text(json.dumps(stats, indent=2))
    sessions = collections.defaultdict(list)
    for row in rows:
        sessions[row['session']].append(row)
    review = out/'review'
    review.mkdir()
    items = sorted(sessions.items())
    index = []
    for page in range((len(items)+19)//20):
        sheet = Image.new('RGB', (1200, 1100), 'white')
        draw = ImageDraw.Draw(sheet)
        for k, (session, members) in enumerate(items[page*20:(page+1)*20]):
            members.sort(key=lambda r:r['stamp'])
            r = members[len(members)//2]
            frame = cv2.imread(r['path'])
            for c in r['candidates']:
                x,y,w,h = c['box']
                cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,255), 2)
            x, y = (k%4)*300, (k//4)*220
            sheet.paste(Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)).resize((296,166)), (x,y+30))
            draw.text((x+3,y+2), '%03d %s %s n=%d' % (page*20+k,r['label'],r['split'],len(members)), fill='black')
            draw.text((x+3,y+16), session, fill='black')
            draw.text((x+3,y+200), 'flags: '+','.join(sorted(set(f for rr in members for f in rr['flags']))), fill='black')
            index.append(dict(id=page*20+k, session=session, label=r['label'], count=len(members)))
        sheet.save(str(review/('page_%02d.jpg'%page)))
    (review/'index.json').write_text(json.dumps(index,indent=2))
    print(json.dumps(stats,indent=2), flush=True)


if __name__ == '__main__':
    main()
