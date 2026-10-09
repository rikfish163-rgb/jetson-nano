"""Two-bay topology: shared back, outer sides, dashed entrance.

Dimensions supplied by the user: 38 cm per bay along the back/entrance,
45 cm depth, seven entrance dashes of nominal length 5 cm. Dash gaps are
not assumed; partly occluded dashes are permitted, but must support both bays.
"""
from __future__ import division
import itertools
import math
import cv2
import numpy as np


def detect_layout(groups,mask,valid,metric,ppm,cfg):
    diag=dict(dash_components=0,back_candidates=0,reject_sides=0,
              reject_view=0,reject_support=0,reject_dashes=0,accepted_pairs=0,matched_dashes=0)
    width,depth=cfg['slots']['P4']['width'],cfg['slots']['P4']['length']
    total=2*width
    origin=np.array(metric(0.,0.))
    matrix=np.column_stack((np.array(metric(1.,0.))-origin,
                            np.array(metric(0.,1.))-origin))
    inverse=np.linalg.inv(matrix)
    def pixel(p):
        return np.dot(inverse,np.array(p)-origin)
    h,w=mask.shape
    dilation=cv2.dilate(mask,np.ones((5,5),np.uint8))
    def visible(p):
        x,y=np.round(pixel(p)).astype(int)
        r=max(1,int(.0125*ppm))
        return (r<=x<w-r and r<=y<h-r and
                (valid is None or bool(np.all(valid[y-r:y+r+1,x-r:x+r+1]))))
    def support(a,b):
        pts=np.array([pixel(a+(b-a)*i/24) for i in range(25)]).astype(int)
        return sum(0<=x<w and 0<=y<h and bool(dilation[y,x]) for x,y in pts)/25.
    # The main Hough stage deliberately ignores 5 cm marks. Extract them as
    # separate components so entrance evidence is not lost by that threshold.
    dashes=[]
    for contour in cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]:
        rect=cv2.minAreaRect(contour)
        short,long=sorted(rect[1])
        if not .03<=long/ppm<=.075 or short/ppm<.003:
            continue
        box=cv2.boxPoints(rect)
        edge=max((box[(i+1)%4]-box[i] for i in range(4)),key=np.linalg.norm)
        p=np.array(metric(*rect[0]))
        tip=np.array(metric(*(np.array(rect[0])+edge)))
        axis=tip-p
        if np.linalg.norm(axis)<=0:
            continue
        dashes.append(dict(point=p,axis=axis/np.linalg.norm(axis),elongated=long>1.3*short))
    diag['dash_components']=len(dashes)
    lines=[]
    for a,b,unused in groups:
        a,b=np.array(metric(*a)),np.array(metric(*b))
        size=float(np.linalg.norm(b-a))
        if size>0:
            lines.append(dict(a=a,b=b,mid=(a+b)/2,size=size,axis=(b-a)/size))
    proposals=[]
    for back in lines:
        if (not total-.08<=back['size']<=total+.12 or
                abs(back['axis'][0])<abs(back['axis'][1]) or abs(back['mid'][1])<depth):
            continue
        diag['back_candidates']+=1
        along=back['axis'] if back['axis'][0]>0 else -back['axis']
        into=np.array([-along[1],along[0]])
        if into[1]*back['mid'][1]<0:
            into=-into
        bottom=float(np.dot(back['mid'],into))
        sides=[]
        for side in lines:
            if side is back or not .18<=side['size']<=depth+.10:
                continue
            if abs(np.dot(side['axis'],into))<math.cos(math.radians(12)):
                continue
            ends=sorted((side['a'],side['b']),key=lambda p:float(np.dot(p,into)))
            if abs(float(np.dot(ends[1],into))-bottom)>.06:
                continue
            sides.append((float(np.dot(side['mid'],along)),side))
        found=False
        for first,last in itertools.combinations(sorted(sides,key=lambda row:row[0]),2):
            lo,sa=first;hi,sb=last
            if abs((hi-lo)-total)>.06:
                continue
            back_ends=sorted(float(np.dot(p,along)) for p in (back['a'],back['b']))
            if back_ends[0]>lo+.04 or back_ends[1]<hi-.04:
                continue
            mouth=bottom-depth
            corners=[along*q+into*t for q in (lo,hi) for t in (mouth,bottom)]
            # Require observed outer entrance/back corners in this branch.
            # Other detectors handle cropped-back or short-side views.
            if not all(visible(p) for p in corners):
                diag['reject_view']+=1
                continue
            if (support(sa['a'],sa['b'])<.65 or support(sb['a'],sb['b'])<.65 or
                    support(along*lo+into*bottom,along*hi+into*bottom)<.65):
                diag['reject_support']+=1
                continue
            marks=[]
            for dash in dashes:
                p=dash['point'];q=float(np.dot(p,along))
                if not lo-.025<=q<=hi+.025 or abs(float(np.dot(p,into))-mouth)>.04:
                    continue
                if dash['elongated'] and abs(np.dot(dash['axis'],along))<math.cos(math.radians(20)):
                    continue
                if visible(p) and not any(abs(q-other[0])<.03 for other in marks):
                    marks.append((q,p))
            mid=(lo+hi)/2
            if (not 3<=len(marks)<=7 or
                    not any(q<mid-.03 for q,p in marks) or
                    not any(q>mid+.03 for q,p in marks)):
                diag['reject_dashes']+=1
                continue
            score=abs((hi-lo)-total)+abs(back['size']-total)
            proposals.append((score,along,into,lo,hi,bottom,sa,sb,marks))
            found=True
        if not found:
            diag['reject_sides']+=1
    out=[]
    for score,along,into,lo,hi,bottom,sa,sb,marks in sorted(proposals,key=lambda row:row[0]):
        # Both centers share one measured back and one inferred dividing axis.
        centers=[along*(lo+(hi-lo)*fraction)+into*(bottom-depth/2) for fraction in (.25,.75)]
        if any(math.hypot(p[0]-s['x'],p[1]-s['y'])<.12 for p in centers for s in out):
            continue
        group='layout_%d'%(diag['accepted_pairs']+1)
        for center,side in zip(centers,(sa,sb)):
            out.append(dict(x=float(center[0]),y=float(center[1]),
                            yaw=math.atan2(into[1],into[0]),kind='perpendicular',
                            length=depth,width=width,evidence='common_back_dashed_mouth',
                            adjacent_group=group,bottom_inferred=False,geometry_inferred=True,
                            divider_inferred=True,
                            entrance_dashes=[p.tolist() for q,p in marks],
                            observed_sides=[[side['a'].tolist(),side['b'].tolist()]],
                            observed_back=[(along*lo+into*bottom).tolist(),
                                           (along*hi+into*bottom).tolist()]))
        diag['accepted_pairs']+=1
        diag['matched_dashes']=max(diag['matched_dashes'],len(marks))
        if len(out)>=20:
            break
    return out,diag
