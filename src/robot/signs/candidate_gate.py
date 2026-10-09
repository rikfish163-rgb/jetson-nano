"""Forward/right route candidates and source-frame identity voting (no ROS)."""
from __future__ import division
import math

ROUTE_LABELS=('LEFT','RIGHT','STRAIGHT','UTURN','PARKING')

def normalize(label):
    label=label.upper()
    return 'PARKING' if label=='PARK' else label

def validate_settings(settings):
    if not isinstance(settings,dict):raise ValueError('sign_route_gate must be a mapping')
    if type(settings.get('enabled',False)) is not bool:
        raise ValueError('sign_route_gate.enabled must be boolean')
    for key,default,low,high in (('left_center_ratio',.30,0,.5),
            ('min_height_ratio',.04,.01,.125),('small_confidence',.90,.8,1.),
            ('max_gap_s',.75,.1,1.5)):
        value=settings.get(key,default)
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not low<=value<=high:
            raise ValueError('invalid sign_route_gate.'+key)
    for key,default in (('normal_frames',2),('small_frames',3)):
        value=settings.get(key,default)
        if isinstance(value,bool) or not isinstance(value,(int,float)) or value!=int(value) or not 2<=value<=8:
            raise ValueError('invalid sign_route_gate.'+key)
    if settings.get('small_frames',3)<settings.get('normal_frames',2):
        raise ValueError('small target requires at least normal_frames')


class RouteSignGate(object):
    def __init__(self,settings,threshold=.8,normal_height=.125):
        validate_settings(settings)
        self.settings=settings
        self.threshold,self.normal_height=threshold,normal_height
        self.track=None
        self.next_id=0

    def allowed(self,label,bounds,shape):
        if normalize(label) not in ROUTE_LABELS:return True
        if bounds is None:return False
        return (bounds[0]+bounds[2]/2.)/shape[1]>=self.settings.get('left_center_ratio',.30)

    def filter(self,detections,shape):
        return [d for d in detections if self.allowed(d['label'],d['bounds'],shape)]

    def evaluate(self,label,confidence,bounds,shape,stamp):
        label=normalize(label)
        diagnostics=dict(enabled=True,left_center_ratio=self.settings.get('left_center_ratio',.30),
                         frames=0,track_id=None)
        if label not in ROUTE_LABELS:
            self.track=None
            accepted=label if label in ('RED','GREEN') and confidence>=self.threshold else ''
            return accepted,'signal_passthrough' if accepted else 'no_candidate',diagnostics
        if not self.allowed(label,bounds,shape):
            self.track=None
            return '','outside_route_region',diagnostics
        x,y,w,h=bounds
        height=h/float(shape[0]);center=(x+w/2.,y+h/2.)
        small=height<self.normal_height
        required=int(self.settings.get('small_frames',3) if small else self.settings.get('normal_frames',2))
        threshold=max(self.threshold,self.settings.get('small_confidence',.90)) if small else self.threshold
        diagnostics.update(center_ratio=center[0]/shape[1],small_target=small,
            required_frames=required,confidence_threshold=threshold,
            min_height_ratio=self.settings.get('min_height_ratio',.04))
        if height<self.settings.get('min_height_ratio',.04) or w/float(shape[1])<.01:
            self.track=None
            return '','too_small',diagnostics
        if confidence<threshold:
            self.track=None
            return '','low_confidence',diagnostics
        old=self.track
        same=False
        if old is not None and old['label']==label and old['shape']==tuple(shape[:2]):
            dt=stamp-old['stamp']
            if dt<=0:
                diagnostics.update(track_id=old['id'],frames=old['count'])
                return '','duplicate_source_frame',diagnostics
            ox,oy,ow,oh=old['bounds']
            oc=(ox+ow/2.,oy+oh/2.)
            area_ratio=w*h/float(ow*oh)
            # Allow smooth perspective growth, but never add a distant same-class board's votes.
            same=(dt<=self.settings.get('max_gap_s',.75) and .35<=area_ratio<=2.8 and
                  abs(center[0]-oc[0])<=max(.04*shape[1],1.5*max(w,ow)) and
                  abs(center[1]-oc[1])<=max(.04*shape[0],1.5*max(h,oh)))
        if not same:
            self.next_id+=1
            self.track=dict(id=self.next_id,count=0,label=label)
        self.track.update(bounds=list(bounds),shape=tuple(shape[:2]),stamp=stamp,
                          count=self.track['count']+1)
        diagnostics.update(track_id=self.track['id'],frames=self.track['count'])
        ready=self.track['count']>=required
        return (label if ready else ''),'route_confirmed' if ready else 'candidate_tracking',diagnostics
