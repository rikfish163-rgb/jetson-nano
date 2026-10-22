#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Collect a separate, resumable supplement; never publish motion commands."""
from __future__ import print_function
import argparse
import fcntl
import glob
import os
import subprocess
import sys
from common import EXTENSIONS

PLAN = [('right', 500), ('park', 500), ('background', 600),
        ('left', 200), ('uturn', 200)]
SPLITS = [('train', 7), ('val', 2), ('test', 1)]
NAMES = dict(right='右转', park='停车 P', background='非路牌', left='左转', uturn='掉头')


def count_images(root, split, label):
    return sum(p.lower().endswith(EXTENSIONS)
               for p in glob.glob(os.path.join(root, split, label, '*', 'roi', '*')))


def main():
    parser = argparse.ArgumentParser(description='追加2000张，独立目录，可断点续采')
    parser.add_argument('--root', default='/home/nano/robodata/signs_supplement_2000')
    parser.add_argument('--hz', type=float, default=20)
    parser.add_argument('--preview', action='store_true')
    parser.add_argument('--status', action='store_true')
    args = parser.parse_args()
    if not 0 < args.hz <= 20:
        parser.error('hz must be in (0,20]')
    for label, total in PLAN:
        print('%s [%s]: %d/%d' % (NAMES[label], label,
              sum(count_images(args.root, split, label) for split, _ in SPLITS), total))
    if args.status:
        return 0
    if not os.path.isdir(args.root):
        os.makedirs(args.root)
    lock = open(os.path.join(args.root, '.capture.lock'), 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    try:
        ask = raw_input
    except NameError:
        ask = input
    print('仅补采，原8000张不变。每组最多50张；每组换距离、角度或背景。')
    for split, ratio in SPLITS:
        print('\n%s：验证/测试必须换场景，不能沿用训练时的连续帧。' % split)
        for label, total in PLAN:
            target = total * ratio // 10
            while True:
                current = count_images(args.root, split, label)
                if current > target:
                    raise ValueError('Count exceeds target: %s/%s' % (split, label))
                if current == target:
                    break
                count = min(50 - current % 50, target - current)
                print('\n%s [%s] %s %d/%d，下一组%d张。' %
                      (NAMES[label], label, split, current, target, count))
                if label == 'background':
                    print('移走所有真路牌；轮换锥桶、蓝线和红绿蓝杂物。')
                else:
                    print('只放当前类别，牌面完整；轮换远中近、倾斜角度和背景。')
                if ask('摆好后回车开始；q退出保存进度：').strip().lower() == 'q':
                    return 2
                command = [sys.executable, '-u', os.path.join(os.path.dirname(__file__), 'collect.py'),
                           '--root', args.root, '--label', label, '--split', split,
                           '--count', str(count), '--hz', str(args.hz), '--duration', '600']
                if args.preview:
                    command.append('--preview')
                if subprocess.call(command) != 0:
                    print('本组未采满；原命令可以续采。')
                    return 2
    print('补采2000张完成。请检查ROI，数量齐全不代表图片质量已审核。')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (KeyboardInterrupt, EOFError):
        print('\n采集停止，原命令可以续采。')
        sys.exit(2)
