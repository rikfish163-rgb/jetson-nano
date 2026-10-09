#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Guided, resumable 500-image/class capture. Never publishes vehicle commands."""
from __future__ import print_function
import argparse
import fcntl
import glob
import json
import os
import subprocess
import sys
import time
from common import LABELS, EXTENSIONS, dataset_digest

TARGETS = [('train', 350), ('val', 100), ('test', 50)]
NAMES = ['红灯', '绿灯', '直行', '左转', '右转', '掉头', '停车 P']
HINTS = ['正面，中距离，缓慢左右移动牌', '稍远距离，保持牌清晰完整',
         '较近距离，不要让牌出画面', '牌向左倾斜约 10–25 度',
         '牌向右倾斜约 10–25 度', '改变背景与高度，缓慢移动',
         '改变光照，避免全部是同一个背景']


def capture_target(label, target, per_class=None):
    if per_class is not None:
        return target * per_class // 500
    return target * 3 if label == 'background' else target


def inventory(root, labels=LABELS):
    return {s: {label: len([p for p in glob.glob(os.path.join(root, s, label, '*', 'roi', '*'))
                           if p.lower().endswith(EXTENSIONS)]) for label in labels}
            for s, _ in TARGETS}


def next_batch(current, target):
    if not 0 <= current <= target:
        raise ValueError('Sample count exceeds target; inspect dataset before continuing')
    return min(target-current, 50-current % 50)


def main():
    p = argparse.ArgumentParser(description='七类逐类提示采集，每类500张，可原命令续采')
    p.add_argument('--root', default='/home/nano/robodata/signs_20260914')
    p.add_argument('--include-background', action='store_true',
                   help='另采非路牌1500张；须使用支持八分类的新训练流程')
    p.add_argument('--hz', type=float, default=2)
    p.add_argument('--per-class', type=int, help='每类总数（含非路牌），按70/20/10划分；须为10的倍数')
    p.add_argument('--topic', default='/front/usb_cam/image_raw')
    p.add_argument('--preview', action='store_true', help='Nano桌面显示实时原图与ROI；空格暂停，q退出')
    p.add_argument('--status', action='store_true')
    p.add_argument('--check-complete', action='store_true')
    p.add_argument('--check-ready', action='store_true', help='核实READY后数据没有变化')
    args = p.parse_args()
    if not 0 < args.hz <= 20: p.error('hz must be in (0,20]')
    if args.per_class is not None and (args.per_class <= 0 or args.per_class % 10):
        p.error('per-class must be a positive multiple of 10')
    labels = LABELS + ['background'] if args.include_background else LABELS
    names = NAMES + ['非路牌（锥桶、蓝线、背景）'] if args.include_background else NAMES
    counts = inventory(args.root, labels)
    for label, name in zip(labels, names):
        print('%s (%s): %s / %d' % (name, label, sum(counts[s][label] for s, _ in TARGETS),
                                    capture_target(label, 500, args.per_class)))
    complete = all(counts[s][label] == capture_target(label, n, args.per_class) for s, n in TARGETS for label in labels)
    if args.check_ready:
        if not complete: return 2
        with open(os.path.join(args.root, 'CAPTURE_COMPLETE.json')) as f: reviewed = json.load(f)
        if (reviewed.get('labels') != labels or reviewed.get('counts') != counts or
                reviewed.get('dataset_sha256') != dataset_digest(args.root, labels)):
            raise ValueError('Dataset changed after READY; review and run capture command again')
        return 0
    if args.check_complete: return 0 if complete else 2
    if args.status: return 0
    if not os.path.isdir(args.root): os.makedirs(args.root)
    lock = open(os.path.join(args.root, '.capture.lock'), 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    marker = os.path.join(args.root, 'CAPTURE_COMPLETE.json')
    if os.path.exists(marker): os.unlink(marker)
    try: ask = raw_input
    except NameError: ask = input
    print('车辆保持静止，手持牌改变位置。一次只放当前类别；错裁图需清理后续采。')
    print('每组50张，目标 %.1f Hz，理想采集时间约 %.1f 秒（不含启动及处理开销）。' % (args.hz, 50.0/args.hz))
    print('训练/验证/测试需重新摆场景，不要保持原姿势连拍。')
    for split, base_target in TARGETS:
        for label, name in zip(labels, names):
            target = capture_target(label, base_target, args.per_class)
            while True:
                current = inventory(args.root, labels)[split][label]
                count = next_batch(current, target)
                if count == 0: break
                hint = HINTS[(current//50) % len(HINTS)] if split == 'train' else '换背景、光照和距离后重新摆牌，覆盖远中近与左右角度'
                if label == 'background':
                    hint = '移走所有真实路牌；轮换锥桶底座、蓝线、红绿蓝杂物，改变角度、背景与光照；验证/测试换场景'
                print('\n\a当前：%s [%s]  %s %d/%d；下一组%d张\n提示：%s' %
                      (name, label, split, current, target, count, hint))
                if ask('摆好当前牌后回车开始；输入 q 保存进度退出：').strip().lower() == 'q': return 2
                command = [sys.executable, '-u', os.path.join(os.path.dirname(__file__), 'collect.py'),
                           '--root', args.root, '--label', label, '--split', split,
                           '--count', str(count), '--hz', str(args.hz), '--duration', '600', '--topic', args.topic]
                if args.preview: command.append('--preview')
                result = subprocess.call(command)
                if result != 0:
                    print('本组中止或未采满，已保存进度；原命令可继续。')
                    return 2
    print('\n%d张已采满。检查各组 roi/，剔除错标签和不合格图片后可用原命令补足。' %
          sum(capture_target(label, 500, args.per_class) for label in labels))
    if ask('确认样本检查完成，输入 READY 标记可训练；其他输入稍后检查：').strip() != 'READY': return 2
    with open(marker+'.tmp', 'w') as f:
        json.dump(dict(labels=labels, counts=inventory(args.root, labels), completed_at=time.time(),
                       dataset_sha256=dataset_digest(args.root, labels)), f, indent=2)
    os.rename(marker+'.tmp', marker)
    print('采集完成并标记可训练：'+marker)
    return 0


if __name__ == '__main__':
    try: sys.exit(main())
    except (KeyboardInterrupt, EOFError):
        print('\n已停止，原命令可断点续采。')
        sys.exit(2)
