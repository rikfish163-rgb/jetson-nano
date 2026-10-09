#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Offline synthetic maneuver bench: files only, no ROS imports or publishers."""
from __future__ import print_function, division
import argparse
import copy
import json
import math
import os
import sys
import time
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PACKAGE = os.path.join(ROOT, 'src', 'robot')
sys.path.insert(0, os.path.dirname(PACKAGE))
from robot.common.contracts import validate_config
from robot.common.contracts import encode_command
from robot.common.contracts import model_to_command_steering
from robot.uturn.relative import RelativeUturn
from robot.uturn.relative import decode_scene
from robot.parallel_parking.planner import ParallelParking


def rectangle(x0, y0, x1, y1):
    return [[x0,y0],[x1,y0],[x1,y1],[x0,y1]]


def examples(cfg):
    cases = []
    spacing = cfg.get('uturn_lane_spacing', .60)
    for side, sign in [('RIGHT',1), ('LEFT',-1)]:
        low, high = sorted([-.30*sign, (spacing+.30)*sign])
        scene = dict(stamp=1.,frame='synthetic_opening',pose=[0,0,0],
                     pose_source='vision',followed_boundary=side,
                     regions=[rectangle(-.80,low,1.10,high)],obstacles=[])
        cases.append(('uturn_follow_'+side,'uturn',scene,True))
    blocked = copy.deepcopy(cases[0][2])
    blocked['regions'] = [rectangle(-.8,-.3,1.1,.3)]
    cases.append(('uturn_insufficient_width','uturn',blocked,False))
    nominal = cfg['slots']['P1']
    length, width = nominal['length'], nominal['width']
    for side, sign in [('left',1), ('right',-1)]:
        bay_low, bay_high = sorted([.30*sign,(.30+width)*sign])
        scene = dict(stamp=1.,frame='synthetic_parking',pose=[-.70,0,0],
            pose_source='vision',obstacles=[],
            regions=[rectangle(-1.5,-.3,2.,.3),
                     rectangle(-length/2,bay_low,length/2,bay_high)],
            slot=dict(id='P1',pose=[0,sign*(.30+width/2),0],length=length,width=width))
        cases.append(('parallel_'+side,'parallel',scene,True))
    narrow = copy.deepcopy(cases[-1][2])
    narrow['slot']['width'] = .18
    cases.append(('parallel_too_narrow','parallel',narrow,False))
    return cases


def escape(text):
    return str(text).replace('&','&amp;').replace('<','&lt;').replace('>','&gt;').replace('"','&quot;')


def svg(result, cfg):
    regions, path = result['scene']['regions'], result['path']
    all_xy = [p for poly in regions for p in poly]+[p[:2] for p in path]
    lo = [min(p[i] for p in all_xy)-.15 for i in (0,1)]
    hi = [max(p[i] for p in all_xy)+.15 for i in (0,1)]
    scale = min(650/(hi[0]-lo[0]),340/(hi[1]-lo[1]))
    def xy(p): return '%.1f,%.1f' % (25+(p[0]-lo[0])*scale,365-(p[1]-lo[1])*scale)
    parts = ['<svg viewBox="0 0 710 390" role="img" aria-label="planned vehicle path">']
    for poly in regions:
        parts.append('<polygon points="%s" fill="#eef2f6" stroke="#334155"/>' % ' '.join(xy(p) for p in poly))
    for a,b in zip(path,path[1:]):
        color = '#16803b' if b[3]>0 else '#d97706'
        parts.append('<polyline points="%s %s" fill="none" stroke="%s" stroke-width="3"/>' % (xy(a),xy(b),color))
    stride = max(1,len(path)//16)
    for p in path[::stride]:
        # Four rectangle corners from the same rear-axle vehicle convention.
        c,s=math.cos(p[2]),math.sin(p[2])
        body=[(p[0]+c*x-s*y,p[1]+s*x+c*y) for x,y in
              [(-cfg['rear_overhang'],-cfg['body_width']/2),
               (cfg['wheelbase']+cfg['front_overhang'],-cfg['body_width']/2),
               (cfg['wheelbase']+cfg['front_overhang'],cfg['body_width']/2),
               (-cfg['rear_overhang'],cfg['body_width']/2)]]
        parts.append('<polygon points="%s" fill="none" stroke="#64748b" opacity=".45"/>' % ' '.join(xy(q) for q in body))
    for name,p,color in [('START',result['scene']['pose'],'#1d4ed8'),('GOAL',result['goal'],'#be123c')]:
        x,y=xy(p).split(',')
        tip=(p[0]+.15*math.cos(p[2]),p[1]+.15*math.sin(p[2]))
        parts.append('<circle cx="%s" cy="%s" r="5" fill="%s"/><polyline points="%s %s" stroke="%s" stroke-width="3"/><text x="%s" y="%s" dy="-10" font-size="12">%s</text>' % (x,y,color,xy(p),xy(tip),color,x,y,name))
    parts.append('</svg>')
    return ''.join(parts)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',default=os.path.join(PACKAGE,'config','competition.yaml'))
    parser.add_argument('--maneuver-config',default=os.path.join(PACKAGE,'config','maneuvers.yaml'))
    parser.add_argument('--output',default=os.path.join(ROOT,'docs','verification','maneuver-preview'))
    args=parser.parse_args()
    with open(args.config) as f: cfg=yaml.safe_load(f)
    if args.maneuver_config:
        with open(args.maneuver_config) as f: cfg.update(yaml.safe_load(f) or {})
    validate_config(cfg)
    results=[]
    for name,kind,scene,expected in examples(cfg):
        started=time.time()
        task=RelativeUturn(cfg,decode_scene(scene)) if kind=='uturn' else ParallelParking(cfg,scene)
        path,reason=task.plan()
        task.accept_plan((path,reason))
        accepted=task.phase=='TRACK'
        control_ms={}
        if accepted:
            for label,stamp in [('first',1.01),('same_observation',1.02),('fresh_observation',1.03)]:
                if label=='fresh_observation':
                    fresh=dict(scene,stamp=stamp)
                    task.observe(decode_scene(fresh) if kind=='uturn' else fresh)
                before=time.time()
                task.command(stamp)
                control_ms[label]=round(1000*(time.time()-before),3)
        raw=[abs(encode_command(0,model_to_command_steering(p[4],cfg),cfg,0)['steering_raw']) for p in path]
        result=dict(name=name,kind=kind,synthetic=True,scene=scene,goal=task.goal,
                    path=path,reason=task.reason,planner_reason=reason,
                    control_ms=control_ms,planned=accepted,expected=expected,
                    seconds=round(time.time()-started,3),max_abs_steering_raw=max(raw or [0]),
                    cusps=sum(a[3]!=b[3] for a,b in zip(path,path[1:])))
        results.append(result)
        print(name, 'PLANNED' if accepted else 'BLOCKED', reason, result['seconds'],control_ms)
    passed=all(r['planned']==r['expected'] and r['max_abs_steering_raw']<=22 for r in results)
    report=dict(evidence='synthetic geometry only; no sensors, no ROS, no physical validation',
                passed=passed,vehicle={k:cfg[k] for k in ('wheelbase','body_width','max_steer','steering_raw_limit')},results=results)
    parent=os.path.dirname(os.path.abspath(args.output))
    if not os.path.isdir(parent):os.makedirs(parent)
    with open(args.output+'.json','w') as f:json.dump(report,f,indent=2,sort_keys=True)
    cards=[]
    for result in results:
        cards.append('<article><h2>%s</h2><p>%s | cusps: %d | max raw: %d | %.3f s</p>%s</article>' %
                     (escape(result['name']),escape(result['planner_reason']),result['cusps'],result['max_abs_steering_raw'],result['seconds'],svg(result,cfg)))
    html='''<!doctype html><html lang="en"><meta charset="utf-8"><title>Maneuver geometry bench</title>
<style>body{font:16px system-ui;margin:30px;background:#fafafa;color:#172033}main{display:grid;grid-template-columns:repeat(auto-fit,minmax(440px,1fr));gap:20px}article{background:white;padding:18px;border:1px solid #ddd}h2{font-size:19px}svg{width:100%}</style>
<h1>U-turn and parallel parking: offline geometry bench</h1>
<p>SYNTHETIC opening and slot polygons. No live sensors, no ROS commands, no hardware success claim.</p>
<p>Green: forward. Orange: reverse. Outlines: sampled vehicle body. X: forward at entry; Y: left. Units: metres.</p>
<main>'''+''.join(cards)+'</main></html>'
    with open(args.output+'.html','w') as f:f.write(html)
    print('OFFLINE_MANEUVER_BENCH', 'PASS' if passed else 'FAIL')
    return 0 if passed else 1


if __name__=='__main__':
    sys.exit(main())
