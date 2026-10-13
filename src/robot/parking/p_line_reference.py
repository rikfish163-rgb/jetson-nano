"""Lock a P image bearing, track its closest transverse stripe, detect loss."""
from __future__ import division
import numpy as np


class PLineReference(object):
    def __init__(self,width=640,height=360):
        self.width,self.height=width,height
        self.last_stamp=-float('inf')
        self.p=None
        self.line=None
        self.line_stamp=None
        self.votes=0
        self.locked=False
        self.missing=0
        self.missing_since=None
        self.ended=False

    def observe_p(self,sign,stamp):
        if sign is None or not 0 <= stamp-sign['stamp'] <= .8:
            return
        x,y,w,h=sign['sign_bounds']
        if self.p is None:
            # A board clipped at an image edge cannot establish its bearing.
            if x <= 2 or x+w >= self.width-2:
                return
        else:
            if sign['stamp'] <= self.p['stamp']:
                return
            a=self.p['sign_bounds']
            previous_id=self.p.get('track_id')
            current_id=sign.get('track_id')
            same_id=(previous_id is not None and previous_id==current_id)
            close=abs(x+w/2.-a[0]-a[2]/2.) <= max(45.,a[2]*1.5)
            if self.locked and not same_id and not close:
                return
            if not self.locked and not same_id and not close:
                # A fresh different P replaces an unconfirmed candidate.
                # Drop its old geometry so the latest board gets a new line.
                self.line,self.line_stamp=None,None
                self.votes=0
        self.p=dict(sign)

    @staticmethod
    def intercept(line):
        a,b=np.asarray(line,float)
        return float(a[0]-a[1]*(b[0]-a[0])/(b[1]-a[1]))

    def matches(self,line,stamp):
        if self.line is None or not 0 < stamp-self.line_stamp <= 1.25:
            return False
        line=np.asarray(line,float)
        delta=line[1]-line[0]
        old=self.line[1]-self.line[0]
        if abs(delta[1]) < .06:
            return False
        overlap=min(max(line[:,1]),max(self.line[:,1]))-max(min(line[:,1]),min(self.line[:,1]))
        cosine=abs(np.dot(delta,old))/(np.linalg.norm(delta)*np.linalg.norm(old))
        error=abs(self.intercept(line)-self.intercept(self.line))
        return overlap >= .06 and cosine >= .90 and error <= min(.25,.07+.4*(stamp-self.line_stamp))

    def update(self,stamp,lines,image_lines,sign,anchor=None):
        if stamp <= self.last_stamp:
            return self.result(False,'DUPLICATE_FRAME')
        self.last_stamp=stamp
        if self.ended:
            # This is a latched approach event. Reversing can bring the stripe
            # back into view, but must not emit ended=True/visible=True.
            return self.result(False,'P_LINE_DISAPPEARED')
        self.observe_p(sign,stamp)
        candidates=[np.asarray(line,float) for line in lines]
        selected=None
        if self.line is not None:
            matches=[line for line in candidates if self.matches(line,stamp)]
            if matches:
                best=min(matches,key=lambda line:abs(self.intercept(line)-self.intercept(self.line)))
                same=[line for line in matches if abs(self.intercept(line)-self.intercept(best)) <= .04]
                selected=max(same,key=lambda line:np.linalg.norm(line[1]-line[0]))
        if selected is None and not self.locked:
            # A missed frame does not erase earlier positive observations.
            # Continue detection/association on every fresh image.
            if self.p is not None and 0 <= stamp-self.p['stamp'] <= .8:
                if anchor is None:
                    x,y,w,h=self.p['sign_bounds']
                    anchor=np.array([x+w/2.,y+h],float)
                ranked=[]
                for line,pixels in zip(candidates,image_lines):
                    pixels=np.asarray(pixels,float)
                    if not np.all(np.isfinite(pixels)) or np.linalg.norm(line[1]-line[0]) < .12:
                        continue
                    if not min(pixels[:,0])-20 <= anchor[0] <= max(pixels[:,0])+20:
                        continue
                    if min(pixels[:,1]) < anchor[1]-10:
                        continue
                    a,b=pixels
                    delta=b-a
                    t=np.clip(np.dot(anchor-a,delta)/np.dot(delta,delta),0,1)
                    ranked.append((float(np.linalg.norm(anchor-(a+t*delta))),line))
                ranked.sort(key=lambda item:item[0])
                if ranked and (len(ranked)==1 or ranked[1][0]-ranked[0][0] >= 10):
                    selected=ranked[0][1]
        if selected is not None:
            self.line,self.line_stamp=selected,stamp
            self.votes+=1
            self.locked=self.locked or self.votes >= 3
            self.missing,self.missing_since=0,None
            return self.result(True,'TRACKING_P_LINE' if self.locked else 'CONFIRMING_P_LINE')
        if self.locked:
            if self.missing_since is None:self.missing_since=stamp
            self.missing+=1
            self.ended=self.ended or (self.missing >= 2 and stamp-self.missing_since >= .1)
            return self.result(False,'P_LINE_DISAPPEARED' if self.ended else 'P_LINE_MISSING_ONCE')
        return self.result(False,'WAIT_P_LINE' if self.p is not None else 'WAIT_TARGET_P')

    def result(self,visible,reason):
        return dict(locked=self.locked,visible=visible,ended=self.ended,reason=reason,
                    p_track_id=self.p.get('track_id') if self.p is not None else None,
                    p_bounds=self.p['sign_bounds'] if self.p is not None else None,
                    line=self.line.tolist() if self.line is not None else None,
                    last_seen_stamp=self.line_stamp,missing_frames=self.missing,
                    lock_frames=self.votes)
