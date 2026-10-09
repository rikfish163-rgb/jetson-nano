"""Jointly fit two adjacent perpendicular bays from three observed separators."""
from __future__ import division
import itertools
import math
import numpy as np
import cv2


def adjacent_bays(groups, mask, valid, metric, ppm, cfg):
    diag=dict(triples=0,reject_parallel=0,reject_spacing=0,
              reject_mouth=0,reject_view=0,reject_support=0,accepted_pairs=0)
    # Blank pixels only prove an entrance when the camera actually observed them.
    # Without a validity mask, retain the anchored-edge detector instead.
    if valid is None:
        diag['reject_view'] = 1
        return [], diag
    width,depth=cfg['slots']['P4']['width'],cfg['slots']['P4']['length']
    tol=min(.06,cfg['white']['dimension_tolerance'])
    dilation=cv2.dilate(mask,np.ones((5,5),np.uint8))
    h,w=mask.shape
    def visible(p):
        x,y=int(round(p[0])),int(round(p[1]))
        r=max(1,int(.0125*ppm))
        return (r<=x<w-r and r<=y<h-r and
                (valid is None or bool(np.all(valid[y-r:y+r+1,x-r:x+r+1]))))
    def support(a,b):
        pts=np.array([a+(b-a)*i/24 for i in range(25)]).astype(int)
        return sum(0<=x<w and 0<=y<h and bool(dilation[y,x]) for x,y in pts)/25.
    lines=[]
    for a,b,unused in groups:
        wa,wb=np.array(metric(*a)),np.array(metric(*b))
        if abs(wa[1])>abs(wb[1]):
            a,b,wa,wb=b,a,wb,wa
        size=float(np.linalg.norm(wb-wa))
        if not .12<=size<=depth+.12 or abs(wa[1])<.05 or wa[0]<=0:
            continue
        v=(wb-wa)/size
        if v[1]*wa[1]<=0 or abs(v[1])<abs(v[0]):
            continue
        lines.append(dict(a=a,b=b,mouth=wa,axis=v,size=size,
                          pixel_axis=(b-a)/np.linalg.norm(b-a),side=1 if wa[1]>0 else -1))
    proposals=[]
    for triple in itertools.combinations(lines,3):
        if len(set(line['side'] for line in triple))!=1:
            continue
        diag['triples']+=1
        if any(np.dot(a['axis'],b['axis'])<math.cos(math.radians(12))
               for a,b in itertools.combinations(triple,2)):
            diag['reject_parallel']+=1
            continue
        axis=sum((line['axis'] for line in triple),np.zeros(2))
        axis/=np.linalg.norm(axis)
        along=np.array([-axis[1],axis[0]])
        if along[0]<0:
            along=-along
        ordered=sorted(triple,key=lambda line:float(np.dot(line['mouth'],along)))
        q=[float(np.dot(line['mouth'],along)) for line in ordered]
        spacing=[q[1]-q[0],q[2]-q[1]]
        if any(abs(gap-width)>tol for gap in spacing):
            diag['reject_spacing']+=1
            continue
        t=[float(np.dot(line['mouth'],axis)) for line in ordered]
        if max(t)-min(t)>.06:
            diag['reject_mouth']+=1
            continue
        if not all(visible(line['a']-line['pixel_axis']*offset*ppm)
                   for line in ordered for offset in (0,.04,.08,.12)):
            diag['reject_view']+=1
            continue
        # Joint support permits one shorter separator, but never invents it.
        if (sum(line['size']>=.18 for line in ordered)<2 or
                any(support(line['a'],line['a']+line['pixel_axis']*.12*ppm)<.65
                    or support(line['a']-line['pixel_axis']*.04*ppm,
                               line['a']-line['pixel_axis']*.12*ppm)>.25
                    for line in ordered)):
            diag['reject_support']+=1
            continue
        score=sum(abs(gap-width) for gap in spacing)+(max(t)-min(t))
        proposals.append((score,ordered,axis))
    slots=[]
    for score,ordered,axis in sorted(proposals,key=lambda item:item[0]):
        centers=[(a['mouth']+b['mouth'])/2+axis*depth/2
                 for a,b in zip(ordered,ordered[1:])]
        # Competing triples made from the same painted edges are alternatives,
        # not additional bays. Keep the best complete pair together.
        if any(math.hypot(p[0]-s['x'],p[1]-s['y'])<.12 for p in centers for s in slots):
            continue
        group=diag['accepted_pairs']+1
        for i,(a,b) in enumerate(zip(ordered,ordered[1:])):
            center=centers[i]
            slots.append(dict(x=float(center[0]),y=float(center[1]),
                              yaw=math.atan2(axis[1],axis[0]),kind='perpendicular',
                              width=width,length=depth,evidence='adjacent_three_lines',
                              bottom_inferred=True,adjacent_group=group,
                              entrance=[a['mouth'].tolist(),b['mouth'].tolist()],
                              observed_sides=[[list(metric(*line['a'])),list(metric(*line['b']))]
                                              for line in (a,b)],
                              shared_separator=[list(metric(*ordered[1]['a'])),
                                                list(metric(*ordered[1]['b']))]))
        diag['accepted_pairs']+=1
        if len(slots)>=20:
            break
    return slots,diag
