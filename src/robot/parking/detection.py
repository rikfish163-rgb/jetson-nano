"""Partial perpendicular bays anchored to an observed end line (no ROS)."""
from __future__ import division
import math
import cv2
import numpy as np
from robot.parking.entrance import from_entrance
from robot.parking.joint import adjacent_bays
from robot.parking.layout import detect_layout


def detect_bays(mask, metric, ppm, cfg, valid_mask=None):
    diag=dict(raw_segments=0,merged_segments=0,pairs=0,reject_parallel=0,
              reject_width=0,reject_short=0,reject_anchor=0,accepted=0,
              accepted_entrance=0,entrance_rejections={})
    raw=cv2.HoughLinesP(mask,1,np.pi/180,20,minLineLength=int(ppm*.10),maxLineGap=int(ppm*.025))
    if raw is None:
        return [],diag
    diag['raw_segments']=len(raw)
    groups=[]
    # Longest first prevents a short noisy fragment from becoming the line axis.
    rows=sorted((r[0] for r in raw),key=lambda r:(r[2]-r[0])**2+(r[3]-r[1])**2,reverse=True)[:80]
    for row in rows:
        a,b=np.array(row[:2],dtype=float),np.array(row[2:],dtype=float)
        size=np.linalg.norm(b-a)
        if size<ppm*.10:
            continue
        v=(b-a)/size
        merged=False
        for g in groups:
            p,q,u=g
            if abs(float(np.dot(u,v)))<math.cos(math.radians(6)):
                continue
            normal=np.array([-u[1],u[0]])
            if max(abs(float(np.dot(x-p,normal))) for x in (a,b))>ppm*.018:
                continue
            lo,hi=sorted(float(np.dot(x-p,u)) for x in (a,b))
            span=float(np.dot(q-p,u))
            if lo>span+ppm*.08 or hi< -ppm*.08:
                continue
            g[0],g[1]=p+u*min(0,lo),p+u*max(span,hi)
            merged=True
            break
        if not merged and len(groups)<32:
            groups.append([a,b,v])
    diag['merged_segments']=len(groups)
    profile=cfg['slots']['P4']
    length,width=profile['length']*ppm,profile['width']*ppm
    tol=min(.06,cfg['white']['dimension_tolerance'])*ppm
    minimum=cfg.get('parking_visible_side_m',.18)*ppm
    dilation=cv2.dilate(mask,np.ones((5,5),np.uint8))
    def coverage(a,b):
        pts=np.array([a+(b-a)*i/24 for i in range(25)]).astype(int)
        return sum(0<=p[0]<mask.shape[1] and 0<=p[1]<mask.shape[0] and
                   bool(dilation[p[1],p[0]]) for p in pts)/25.
    found,joint=adjacent_bays(groups,mask,valid_mask,metric,ppm,cfg)
    diag['joint']=joint
    diag['accepted_joint']=len(found)
    layout,layout_diag=detect_layout(groups,mask,valid_mask,metric,ppm,cfg)
    diag['layout']=layout_diag
    # The specified back/entrance topology takes precedence for its region.
    if layout:
        found=[s for s in found if not any(math.hypot(s['x']-p['x'],s['y']-p['y'])<.20 for p in layout)]
        found=layout+found
    def already_found(x,y):
        for s in found:
            dx,dy=x-s['x'],y-s['y']
            if math.hypot(dx,dy)<.12:
                return True
            if s.get('adjacent_group') is not None:
                co,si=math.cos(s['yaw']),math.sin(s['yaw'])
                if (abs(dx*co+dy*si)<s['length']/2 and
                        abs(-dx*si+dy*co)<s['width']*.45):
                    return True
        return False
    for i,(a,b,u) in enumerate(groups):
        n=np.array([-u[1],u[0]])
        alo,ahi=sorted(float(np.dot(x,u)) for x in (a,b))
        na=float(np.dot((a+b)/2,n))
        for c,d,v in groups[i+1:]:
            diag['pairs']+=1
            if abs(float(np.dot(u,v)))<math.cos(math.radians(12)):
                diag['reject_parallel']+=1
                continue
            nc=float(np.dot((c+d)/2,n))
            if abs(abs(nc-na)-width)>tol:
                diag['reject_width']+=1
                continue
            clo,chi=sorted(float(np.dot(x,u)) for x in (c,d))
            if min(ahi,chi)-max(alo,clo)<minimum:
                diag['reject_short']+=1
                continue
            anchored=False
            for e,f,w in groups:
                if abs(float(np.dot(u,w)))>math.sin(math.radians(12)):
                    continue
                nlo,nhi=sorted(float(np.dot(x,n)) for x in (e,f))
                if nlo>min(na,nc)+ppm*.04 or nhi<max(na,nc)-ppm*.04:
                    continue
                t=float(np.dot((e+f)/2,u))
                # Only a terminating end line anchors depth, not a road marking
                # crossing the middle of two arbitrarily long parallel lines.
                direction=0
                if max(abs(t-alo),abs(t-clo))<=ppm*.06 and min(ahi,chi)-t>=minimum:
                    direction=1
                elif max(abs(t-ahi),abs(t-chi))<=ppm*.06 and t-max(alo,clo)>=minimum:
                    direction=-1
                if not direction:
                    continue
                extent=min(ahi,chi)-t if direction>0 else t-max(alo,clo)
                if extent>length+2*tol:
                    continue
                back=[u*t+n*z for z in (na,nc)]
                tips=[u*(t+direction*min(extent,length))+n*z for z in (na,nc)]
                if min(coverage(back[0],back[1]),coverage(back[0],tips[0]),coverage(back[1],tips[1]))<.5:
                    continue
                anchored=True
                center=u*(t+direction*length/2)+n*(na+nc)/2
                x,y=metric(*center)
                p2=metric(*(center+u*ppm))
                yaw=math.atan2(p2[1]-y,p2[0]-x)
                co,si=math.cos(yaw),math.sin(yaw)
                # An observed terminating edge resolves the depth ambiguity of
                # three parallel fragments. Replace its inferred alternative.
                found[:]=[s for s in found if not (s.get('bottom_inferred') and
                    abs(co*(s['x']-x)+si*(s['y']-y))<profile['length'] and
                    abs(-si*(s['x']-x)+co*(s['y']-y))<profile['width']*.45)]
                if not already_found(x,y):
                    found.append(dict(x=x,y=y,yaw=yaw,kind='perpendicular',
                                      length=profile['length'],width=profile['width']))
                break
            if not anchored:
                candidate, reason = from_entrance(a,b,c,d,u,n,na,nc,metric,
                                                  ppm,mask,valid_mask,cfg,coverage)
                if candidate is not None:
                    anchored = True
                    if not already_found(candidate['x'],candidate['y']):
                        found.append(candidate)
                        diag['accepted_entrance']+=1
                else:
                    rejected=diag['entrance_rejections']
                    rejected[reason]=rejected.get(reason,0)+1
                    diag['reject_anchor']+=1
    found=found[:20]
    diag['accepted']=len(found)
    return found,diag
