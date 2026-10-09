#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Nano OpenCV verification; replacement needs --install and typed confirmation."""
from __future__ import print_function
import argparse
import fcntl
import json
import os
import shutil
import sys
import tempfile
import time
from common import LABELS, samples, metrics, file_sha256


def main():
    p = argparse.ArgumentParser(description='Nano 验证候选 ONNX；默认不替换')
    p.add_argument('--candidate',required=True)
    p.add_argument('--data',required=True)
    p.add_argument('--split',choices=['val','test'],default='val')
    p.add_argument('--runtime-scripts',default='/home/nano/robocup_ws/src/ros/signs/scripts')
    p.add_argument('--target',default='/home/nano/robocup_ws/src/ros/signs/scripts/resnet18.onnx')
    p.add_argument('--min-recall',type=float,default=.80)
    p.add_argument('--max-pair-error',type=float,default=.10)
    p.add_argument('--max-regression',type=float,default=.03)
    p.add_argument('--install',action='store_true')
    p.add_argument('--yes',action='store_true',help='已授权的自动流程，跳过交互确认；仍执行全部评估')
    args = p.parse_args()
    for v in (args.min_recall,args.max_pair_error,args.max_regression):
        if not 0 <= v <= 1:p.error('thresholds must be in [0,1]')
    if os.path.realpath(args.candidate)==os.path.realpath(args.target):
        p.error('candidate must be separate from current model')
    import cv2
    import numpy as np
    cv2.setNumThreads(1)
    sys.path.insert(0,args.runtime_scripts)
    from sign_actions import LABELS as runtime_labels
    from sign_classifier_cv import SignClassifier
    if runtime_labels != LABELS:raise ValueError('Runtime label order changed')
    report_path=os.path.join(os.path.dirname(args.candidate),'report.json')
    with open(report_path) as f:training=json.load(f)
    candidate_hash=file_sha256(args.candidate)
    if training['labels'] != LABELS or training['onnx_sha256'] != candidate_hash:
        raise ValueError('Candidate does not match training report')
    target_hash=file_sha256(args.target)
    rows=samples(args.data,args.split)
    print('只运行分类，不发布控制命令。将对新旧模型使用同一批 ROI。')
    models=[SignClassifier(args.target),SignClassifier(args.candidate)]
    matrices=[[[0]*7 for _ in LABELS] for _ in models]
    for n,(path,truth) in enumerate(rows,1):
        frame=cv2.imread(path)
        if frame is None:raise ValueError('Unreadable image: '+path)
        for model,matrix in zip(models,matrices):
            guess,score,label=model.classify_sign(frame)
            if not np.isfinite(score):raise ValueError('Nonfinite output')
            matrix[truth][guess]+=1
        if n%25==0:print('已验证 %d/%d' % (n,len(rows)))
    old,new=[metrics(m) for m in matrices]
    result=dict(old=old,new=new,split=args.split,candidate_sha256=candidate_hash,target_sha256=target_hash)
    print('旧模型召回率:',old['recall'])
    print('新模型召回率:',new['recall'])
    print('新模型 park->uturn=%.3f，uturn->park=%.3f' % (new['park_to_uturn'],new['uturn_to_park']))
    failures=[]
    for label in LABELS:
        if new['recall'][label]<args.min_recall:failures.append(label+' recall below minimum')
        if new['recall'][label]+args.max_regression<old['recall'][label]:failures.append(label+' regressed')
    if max(new['park_to_uturn'],new['uturn_to_park'])>args.max_pair_error:failures.append('park/uturn confusion too high')
    result['failures']=failures
    with open(args.candidate+'.nano_eval.json','w') as f:json.dump(result,f,indent=2)
    if failures:
        print('未达到替换条件:',failures)
        return 2
    if not args.install:
        print('验证通过，但没有替换模型。确认结果后使用同一命令加 --install。')
        return 0
    print('请先停止使用该模型的识别节点；不要同时运行自动驾驶。')
    print('将备份并替换:',args.target)
    try:ask=raw_input
    except NameError:ask=input
    if not args.yes and ask('输入 REPLACE 确认替换，其他输入取消：').strip()!='REPLACE':
        print('已取消，当前模型未改。');return 0
    install_lock = open(args.target+'.install.lock', 'a')
    fcntl.flock(install_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    if file_sha256(args.target)!=target_hash:raise ValueError('Current model changed during verification; retry after coordination')
    backup=args.target+'.backup.'+time.strftime('%Y%m%d_%H%M%S')
    if os.path.exists(backup):raise ValueError('Backup already exists')
    fd,staged=tempfile.mkstemp(prefix='.sign_candidate_',suffix='.onnx',dir=os.path.dirname(os.path.abspath(args.target)))
    os.close(fd)
    try:
        shutil.copy2(args.candidate,staged)
        if file_sha256(staged)!=candidate_hash:raise ValueError('Candidate changed during verification')
        shutil.copy2(args.target,backup)
        with open(staged,'rb') as f:os.fsync(f.fileno())
        os.rename(staged,args.target)
        if file_sha256(args.target)!=candidate_hash:raise ValueError('Installed model hash mismatch')
    finally:
        if os.path.exists(staged):os.unlink(staged)
    print('替换完成，备份:',backup)
    print('新 SHA256:',file_sha256(args.target))
    print('已运行的节点仍使用内存中的旧模型；下次重新启动识别节点才加载新模型。')
    try:from shlex import quote
    except ImportError:from pipes import quote
    print('如需恢复，先停止识别节点，再执行以下完整命令：')
    print('cd /home/nano/robocup_ws\nflock %s cp -p %s %s' %
          (quote(args.target+'.install.lock'),quote(backup),quote(args.target)))
    return 0


if __name__=='__main__':
    sys.exit(main())
