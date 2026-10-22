"""Temporal identity for stationary inspection only, never an actuator source."""
from __future__ import division
import collections
import copy
import math


def angle_difference(a,b):
    # A line axis has pi-periodicity; legacy cross-line candidates may flip it.
    return .5*math.atan2(math.sin(2*(a-b)),math.cos(2*(a-b)))


class BayTracker(object):
    def __init__(self):
        self.tracks=[]
        self.last_stamp=None
        self.next_id=1

    def snapshot(self,now,allow_observed=True):
        self.tracks=[t for t in self.tracks if 0<=now-t['last_seen']<=1.0]
        out=[]
        for t in self.tracks:
            row=copy.deepcopy(t['row'])
            observed=(allow_observed and t['last_seen']==self.last_stamp and
                      now-t['last_seen']<=.5)
            row.update(id=t['id'],confirmed=t['confirmed'],observed=observed,
                       relative_bay=t['relative_bay'],observations=sum(t['votes']),
                       observation_age_s=now-t['last_seen'],
                       tracking_state='observed' if observed else 'temporarily_missing')
            if not observed:
                row.update(occupancy='UNKNOWN',coverage=None)
            out.append(row)
        return out

    def update(self,candidates,stamp,now):
        self.snapshot(now)
        if stamp is None or not 0<=now-stamp<=1.25:
            return self.snapshot(now,False)
        if self.last_stamp is not None and stamp<=self.last_stamp:
            # Re-publishing or polling one source image cannot confirm a bay.
            return self.snapshot(now,stamp==self.last_stamp)
        self.last_stamp=stamp
        incoming=[copy.deepcopy(c) for c in candidates]
        for c in incoming:
            c.setdefault('pose',[c['x'],c['y'],c['yaw']])
        edges=[]
        for i,t in enumerate(self.tracks):
            a=t['row']
            for j,b in enumerate(incoming):
                distance=math.hypot(a['pose'][0]-b['pose'][0],a['pose'][1]-b['pose'][1])
                angle=abs(angle_difference(a['pose'][2],b['pose'][2]))
                if (distance<=.08 and angle<=math.radians(10) and
                        abs(a['width']-b['width'])<=.04 and abs(a['length']-b['length'])<=.04):
                    edges.append((distance+angle*.1,i,j))
        assignments={}
        used=set()
        for cost,i,j in sorted(edges):
            if i not in assignments and j not in used:
                assignments[i]=j;used.add(j)
        seen_groups=collections.defaultdict(list)
        for i,t in enumerate(self.tracks):
            seen=i in assignments
            t['votes'].append(seen)
            if not seen:
                continue
            row=incoming[assignments[i]]
            previous=t['row']['pose'];current=row['pose']
            row['pose']=[previous[0]+.35*(current[0]-previous[0]),
                         previous[1]+.35*(current[1]-previous[1]),
                         previous[2]+.35*angle_difference(current[2],previous[2])]
            row['x'],row['y'],row['yaw']=row['pose']
            t.update(row=row,last_seen=stamp,confirmed=t['confirmed'] or sum(t['votes'])>=3)
            if row.get('adjacent_group') is not None:
                seen_groups[row['adjacent_group']].append(t)
        for j,row in enumerate(incoming):
            if j in used:
                continue
            t=dict(id='bay_%d'%self.next_id,row=row,last_seen=stamp,
                   votes=collections.deque([True],maxlen=5),confirmed=False,
                   relative_bay='unassigned')
            self.next_id+=1;self.tracks.append(t)
            if row.get('adjacent_group') is not None:
                seen_groups[row['adjacent_group']].append(t)
        for group in seen_groups.values():
            if len(group)==2 and all(t['confirmed'] for t in group):
                near,far=sorted(group,key=lambda t:t['row']['pose'][0])
                if (near['relative_bay'] in ('unassigned','near') and
                        far['relative_bay'] in ('unassigned','far')):
                    near['relative_bay']='near';far['relative_bay']='far'
        return self.snapshot(now)
