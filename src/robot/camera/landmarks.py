"""Measured 1 m / 0.10 m blue triplet, in one fixed visual frame.

No horizontal endpoint offset is assumed. Partial segments constrain their
supporting lines, not an invented visible-segment midpoint or length scale.
"""
from __future__ import division
import math
import itertools
import cv2
import numpy as np
from robot.common.geometry import world
from robot.common.geometry import wrap


def axis_angle(angle):
    return (angle+math.pi/2)%math.pi-math.pi/2


def description(segment):
    a,b=np.asarray(segment,dtype=float)
    delta=b-a
    length=float(np.linalg.norm(delta))
    direction=delta/length
    normal=np.array([-direction[1],direction[0]])
    return (a+b)/2,direction,normal,length


def merge_segments(segments, max_length=1.08):
    """Merge overlapping Hough edges/fragments, not separate painted markers."""
    result=[]
    for segment in segments:
        mid,direction,normal,length=description(segment)
        if not .18<=length<=max_length: continue
        for index,old in enumerate(result):
            center,axis,n,unused=description(old)
            if abs(np.dot(direction,n))>math.sin(math.radians(5)): continue
            if abs(np.dot(mid-center,n))>.035: continue
            lo,hi=sorted(np.dot(old-center,axis))
            x0,x1=sorted(np.dot(segment-center,axis))
            if x0>hi+.08 or lo>x1+.08: continue
            lo,hi=min(lo,x0),max(hi,x1)
            if hi-lo>max_length: continue
            center=center+n*np.dot(mid-center,n)/2
            result[index]=np.array([center+axis*lo,center+axis*hi])
            break
        else:
            result.append(np.asarray(segment,dtype=float))
    return result[:24]


def image_segments(mask,metric,ppm,max_length=1.08):
    lines=cv2.HoughLinesP(mask,1,np.pi/360,30,
                         minLineLength=int(ppm*.20),maxLineGap=int(ppm*.04))
    if lines is None: return []
    rows=sorted(lines[:,0],key=lambda r:(r[2]-r[0])**2+(r[3]-r[1])**2,reverse=True)[:100]
    return merge_segments([metric(np.asarray(row,float).reshape(2,2)) for row in rows],max_length)


class BlueLandmarks(object):
    def __init__(self,cfg):
        self.length=float(cfg.get('uturn_blue_line_length_m',1.))
        self.gap=float(cfg.get('uturn_blue_end_gap_m',.10))
        if not np.isfinite(self.length+self.gap) or not .8<=self.length<=1.2 or not .02<=self.gap<=.25:
            raise ValueError('invalid blue triplet dimensions')
        self.separation=self.length+2*self.gap
        self.history=[]
        self.landmarks=None
        self.corrected=False
        self.corrected_at=-1.
        self.reason='blue_wait_triplet'

    def acquire(self,segments):
        candidates=[]
        for first,second in itertools.combinations(range(len(segments)),2):
            m,u,n,length=description(segments[first])
            q,v,unused,other_length=description(segments[second])
            if abs(np.dot(v,n))>math.sin(math.radians(4)): continue
            if abs(abs(np.dot(q-m,n))-self.separation)>.05: continue
            # The middle segment lies between two lines, with .10 m gaps.
            center=(np.dot(m,n)+np.dot(q,n))/2
            for third in range(len(segments)):
                if third in (first,second): continue
                a,b=segments[third]
                mid,direction,unused,l=description(segments[third])
                if abs(np.dot(direction,u))>math.sin(math.radians(4)): continue
                ends=np.dot(np.array([a,b]),n)-center
                if min(ends)<-self.length/2-.05 or max(ends)>self.length/2+.05: continue
                if l>self.length+.06: continue
                # Vertical marker must be next to an end of each horizontal,
                # not through their middles; gap size along this axis is unknown.
                valid=True
                directions=[]
                for idx in (first,second):
                    along=np.dot(segments[idx]-mid,u)
                    if min(along)<-.08 and max(along)>.08: valid=False
                    directions.append(float(np.mean(along)))
                if directions[0]*directions[1]<=0: valid=False
                if valid: candidates.append((first,second,third))
        # Several distinct patterns require identity information we do not have.
        unique={tuple(sorted(c)) for c in candidates}
        if len(unique)!=1:
            self.reason='blue_triplet_ambiguous' if unique else 'blue_wait_triplet'
            return
        ids=next(iter(unique))
        first,second,third=next(c for c in candidates if tuple(sorted(c))==ids)
        m,u,n,unused=description(segments[first])
        q,v,unused,unused_length=description(segments[second])
        center=(np.dot(m,n)+np.dot(q,n))/2
        # Enforce the supplied separation, not a noisy apparent gap. Horizontal
        # endpoint positions remain observed; their unsupplied offset is unused.
        self.landmarks=[]
        for index in (first,second):
            normal_coordinate=float(np.mean(np.dot(segments[index],n)))
            sign=1 if normal_coordinate>center else -1
            self.landmarks.append(np.array([u*np.dot(p,u)+n*(center+sign*self.separation/2)
                                           for p in segments[index]]))
        mid=segments[third].mean(axis=0)
        mid=u*np.dot(mid,u)+n*center
        self.landmarks.append(np.array([mid-n*self.length/2,mid+n*self.length/2]))
        self.reason='blue_triplet_locked'

    def update(self,segments,pose,stamp):
        self.corrected=False
        segments=merge_segments(segments,self.length+.08)
        if self.landmarks is None:
            observed=[np.asarray([world(pose,p) for p in s]) for s in segments]
            self.history=[(t,s) for t,s in self.history if 0<=stamp-t<=2.]
            self.history.extend((stamp,s) for s in observed)
            self.history=self.history[-120:]
            self.acquire(merge_segments([s for unused,s in self.history],self.length+.08))
            return pose
        associations={}
        for segment in segments:
            m,u,n,length=description(segment)
            wm=np.asarray(world(pose,m))
            angle=math.atan2(u[1],u[0])+pose[2]
            matches=[]
            for index,landmark in enumerate(self.landmarks):
                lm,lu,ln,unused=description(landmark)
                error=axis_angle(math.atan2(lu[1],lu[0])-angle)
                if abs(error)>math.radians(10) or abs(np.dot(wm-lm,ln))>.12: continue
                if abs(np.dot(wm-lm,lu))>self.length+.1: continue
                matches.append((index,error))
            if len(matches)==1:
                index,error=matches[0]
                associations.setdefault(index,[]).append((m,error))
        if len(associations)<2:
            self.reason='blue_need_nonparallel_lines'
            return pose
        normals=[]; centers=[]; angles=[]; mids=[]
        for index,rows in associations.items():
            lm,lu,ln,unused=description(self.landmarks[index])
            normals.append(ln); centers.append(float(np.dot(ln,lm)))
            mids.append(np.median([row[0] for row in rows],axis=0))
            angles.append(float(np.median([row[1] for row in rows])))
        normals=np.asarray(normals)
        if np.linalg.svd(normals,compute_uv=False)[-1]<.5:
            self.reason='blue_parallel_only'
            return pose
        correction=float(np.median(angles))
        if max(abs(a-correction) for a in angles)>math.radians(3):
            self.reason='blue_heading_inconsistent'
            return pose
        yaw=wrap(pose[2]+correction)
        rotated=[world((0,0,yaw),m) for m in mids]
        rhs=np.asarray(centers)-np.sum(normals*np.asarray(rotated),axis=1)
        xy=np.linalg.lstsq(normals,rhs,rcond=-1)[0]
        if np.linalg.norm(xy-np.asarray(pose[:2]))>.12 or max(abs(np.dot(normals,xy)-rhs))>.025:
            self.reason='blue_correction_rejected'
            return pose
        self.corrected=True
        self.corrected_at=stamp
        self.reason='blue_pose_corrected'
        return float(xy[0]),float(xy[1]),yaw
