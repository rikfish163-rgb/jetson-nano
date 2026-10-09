"""Forward visual parking from paired bay sides and an observed bottom edge."""
from __future__ import division
import math
from robot.common.geometry import local, world
from robot.parking.entry import WhiteParking


class ForwardParking(WhiteParking):

    def __init__(self,cfg,pose,now):
        self.cfg,self.pose,self.start,self.started=cfg,pose,pose,now
        self.stamp=-1.;self.bottom=None;self.bottom_stamp=-1.;self.bottom_source=None
        self.view=None;self.phase='APPROACH';self.reason=None;self.status=None
        self.debug={};self.slot=None;self.sign_anchor=None;self.single=None
        self.associate=cfg.get('parking_sign_association',False)
        self.sides_behind_sign_rejected=0
        self.bottom_target_aligned=True
        self.bottom_pose=None;self.motion_sample=None;self.motion_ratios=[]
        self.motion_scale=1.;self.motion_speed=None
        self.tracked_view=None;self.tracked_view_stamp=-1.
        self.bottom_only=False;self.bottom_candidate=None

    def observe_bottom_only(self, cross, stamp):
        """Confirm a measured transverse stripe inside the target corridor."""
        if not self.bottom_only or self.bottom is not None:
            return
        if self.tracked_view is not None:
            corridor = tuple(self.tracked_view['center'])+(self.tracked_view['theta'],)
        else:
            if self.associate and not self.bottom_target_aligned:
                self.bottom_candidate=None
                return
            corridor = self.pose
        candidates=[]
        for unused_a, unused_b, line in cross:
            a,b=[local(corridor,p) for p in line]
            dx,dy=b[0]-a[0],b[1]-a[1]
            low,high=sorted((a[1],b[1]))
            if not .30<=high-low<=.60 or not low<=-.12<.12<=high:
                continue
            if abs(dx/dy)>.35 or abs((low+high)/2.)>.10:
                continue
            x=a[0]-a[1]*dx/dy
            if self.tracked_view is not None:
                far_end=max(local(corridor,p)[0]
                            for side in self.tracked_view['sides'] for p in side)
                if x<far_end-.25:
                    continue  # A transverse stripe near the remembered mouth is not the bottom.
            center=world(corridor,(x,0.))
            forward=local(self.pose,center)[0]
            if not .05<forward<=1.2:
                continue
            if self.tracked_view is None and self.sign_anchor is not None:
                ax=local(self.pose,self.sign_anchor)[0]
                if not -.10<=ax-forward<=.45:
                    continue
            candidates.append((forward,line,center,math.atan(dx/dy)+corridor[2]))
        if not candidates:
            self.bottom_candidate=None
            return
        unused_forward,line,center,angle=min(candidates,key=lambda row:row[0])
        previous=self.bottom_candidate
        consistent=(previous is not None and
            0<stamp-previous['stamp']<=self.cfg.get('ground_timeout',1.25) and
            math.hypot(center[0]-previous['center'][0],center[1]-previous['center'][1])<=.10 and
            abs((angle-previous['angle']+math.pi)%(2*math.pi)-math.pi)<=math.radians(10))
        self.bottom_candidate=dict(stamp=stamp,center=center,angle=angle,
            frames=previous['frames']+1 if consistent else 1)
        if self.bottom_candidate['frames']>=2:
            self.remember_bottom(line,stamp,'standalone_bottom')

    def remember_bottom(self,line,stamp,source):
        if source!='tracked_bottom':
            # A newly associated line cannot calibrate travel against another.
            self.motion_sample=None;self.motion_ratios=[];self.motion_scale=1.
        self.bottom,self.bottom_stamp,self.bottom_source=line,stamp,source
        self.bottom_pose=tuple(self.pose)
        remaining=WhiteParking.remaining(self)
        if remaining is None:return
        sample=self.motion_sample
        if sample is not None:
            origin,previous,previous_stamp=sample
            heading=(self.pose[2]-origin[2]+math.pi)%(2*math.pi)-math.pi
            progress=local(origin,self.pose)[0]
            if abs(heading)>.08 or stamp-previous_stamp>3.:
                self.motion_sample=None
            elif progress>=.08:
                ratio=(previous-remaining)/progress
                if 0<=ratio<=1.5:
                    self.motion_ratios=(self.motion_ratios+[ratio])[-3:]
                    self.motion_scale=sorted(self.motion_ratios)[len(self.motion_ratios)//2]
                self.motion_sample=None
        if self.motion_sample is None:
            self.motion_sample=(tuple(self.pose),remaining,stamp)

    def remaining(self):
        # Only extrapolation after the last measured line uses the locally
        # observed travel ratio. Do not alter global odometry or other actions.
        pose=self.pose
        if self.bottom_pose is not None:
            origin=self.bottom_pose
            pose=(origin[0]+(pose[0]-origin[0])*self.motion_scale,
                  origin[1]+(pose[1]-origin[1])*self.motion_scale,pose[2])
        return WhiteParking.remaining(self,pose=pose)

    def observe(self,lines,stamp,pose=None):
        if stamp<=self.stamp:return
        if pose is not None:self.pose=tuple(pose)
        self.lines=lines;self.stamp=stamp;self.view=None;self.single=None
        self.sides_behind_sign_rejected=0
        anchor=local(self.pose,self.sign_anchor) if self.sign_anchor is not None else None
        if self.associate and anchor is None:return
        tracking=(self.associate and self.phase=='CENTER' and self.tracked_view is not None
                  and 0<=stamp-self.tracked_view_stamp<=self.cfg.get('ground_timeout',1.25))
        # A bottom stripe may be shared by several adjacent bays. Seeing the
        # stripe below P is not our target while still aimed at its neighbour.
        width=self.cfg.get('slots',{}).get('P4',{}).get('width',.38)
        self.bottom_target_aligned=(not self.associate or abs(anchor[1])<=width/2.+.04)
        sides=[];cross=[]
        for line in lines:
            a,b=sorted(local(self.pose,p) for p in line)
            dx,dy=b[0]-a[0],b[1]-a[1]
            if math.hypot(dx,dy)<.18:continue
            if dx>.18 and abs(dy/dx)<math.tan(math.radians(35)):
                slope,intercept=dy/dx,a[1]-a[0]*dy/dx
                if self.associate and not tracking:
                    # Only observed paint BEFORE P can bound its approach bay.
                    # A background segment cannot be extended backwards to
                    # invent a missing side in front of the board.
                    end=min(b[0],anchor[0])
                    if end-max(a[0],.05)<.18:
                        self.sides_behind_sign_rejected+=1
                        continue
                    b=(end,slope*end+intercept)
                sides.append((a,b,slope,intercept))
            elif abs(dy)>.18 and abs(dx/dy)<.35:cross.append((a,b,line))
        if self.bottom is not None:
            # P initializes identity. Thereafter track the same measured stripe
            # even when dead-reckoned P has drifted behind the visible bottom.
            a,b=[local(self.pose,p) for p in self.bottom]
            dx,dy=b[0]-a[0],b[1]-a[1]
            matches=[]
            if abs(dy)>.18 and abs(dx/dy)<.5:
                slope=dx/dy;expected=a[0]-a[1]*slope
                for a,b,line in cross:
                    low,high=sorted((a[1],b[1]))
                    if high-low<.30 or not low<=-.12<.12<=high:continue
                    k=(b[0]-a[0])/(b[1]-a[1]);x=a[0]-a[1]*k
                    error=abs(x-expected)
                    if .05<x<1.8 and error<=.12 and abs(math.atan(k)-math.atan(slope))<=.15:
                        matches.append((error,line))
            if matches:
                self.remember_bottom(min(matches,key=lambda row:row[0])[1],stamp,'tracked_bottom')
        if self.associate and self.bottom_target_aligned and self.bottom_stamp!=stamp:
            # Near the bottom the side rails can leave the image or fragment.
            # The retained P position still identifies the target corridor;
            # do not require a complete side pair to retain the bottom target.
            ax,ay=anchor
            bottoms=[]
            for a,b,line in cross:
                low,high=sorted((a[1],b[1]))
                if high-low<.30 or not low<=ay-.12<ay+.12<=high:continue
                bx=a[0]+(ay-a[1])*(b[0]-a[0])/(b[1]-a[1])
                # Bottom is immediately before the board. Reject the distant
                # bay mouth and unrelated cross-lines behind the P stand.
                if not .05<bx<=1.2 or not 0<=ax-bx<=.25:continue
                bottoms.append((abs(ax-bx),line))
            if bottoms:
                self.remember_bottom(min(bottoms,key=lambda row:row[0])[1],stamp,'p_near_cross')
                # Keep extracting visible sides for centering on the approach.
        candidates=[]
        for i,left in enumerate(sides):
            for right in sides[i+1:]:
                lo=max(left[0][0],right[0][0],.05)
                hi=min(left[1][0],right[1][0],1.8)
                if hi-lo<.18 or abs(math.atan(left[2])-math.atan(right[2]))>math.radians(10):continue
                x=(lo+hi)/2
                yl,yr=left[2]*x+left[3],right[2]*x+right[3]
                width=abs(yl-yr);center=(yl+yr)/2
                if not .30<=width<=.50 or abs(center)>(1.0 if self.associate else .30):continue
                score=lo
                if tracking:
                    # After P identifies the bay, follow fresh measurements of
                    # that pair. A drifting command-estimated P must not veto
                    # its visible rails. Never switch to a neighbouring pair.
                    previous=self.tracked_view
                    theta=previous['theta']-self.pose[2]
                    rx,ry=local(self.pose,previous['center'])
                    slope=(left[2]+right[2])/2.
                    heading=(math.atan(slope)-theta+math.pi)%(2*math.pi)-math.pi
                    lateral=-(x-rx)*math.sin(theta)+(center-ry)*math.cos(theta)
                    if (abs(heading)>math.radians(10) or abs(lateral)>.08
                            or abs(width-previous['width'])>.06):continue
                    if self.bottom is not None:
                        if lo>=max(local(self.pose,q)[0] for q in self.bottom):continue
                    score=abs(lateral)+abs(width-previous['width'])
                elif self.associate:
                    ax,ay=anchor
                    edges=sorted((left[2]*ax+left[3],right[2]*ax+right[3]))
                    # Board stands between the sides, close to their far end.
                    if not edges[0]-.04<=ay<=edges[1]+.04:continue
                    if max(abs(side[1][0]-ax) for side in (left,right))>.45:continue
                    score=abs(hi-ax)+abs((edges[0]+edges[1])/2.-ay)
                candidates.append((abs(center),score,lo,hi,left,right,width))
        if self.associate and self.phase=='APPROACH':
            matches=[]
            ax,ay=anchor
            for side in sides:
                offset=side[2]*ax+side[3]-ay
                if .12<=abs(offset)<=.29 and abs(side[1][0]-ax)<=.45:
                    x=(side[0][0]+side[1][0])/2.
                    # Follow the bay side at half-bay clearance, not on top of paint.
                    y=side[2]*x+side[3]-(.19 if offset>0 else -.19)
                    matches.append((abs(side[1][0]-ax),side,x,y))
            if matches:
                _,side,x,y=min(matches,key=lambda row:row[0])
                self.single=dict(theta=self.pose[2]+math.atan(side[2]),center=world(self.pose,(x,y)),
                    side=[world(self.pose,p) for p in side[:2]])
        if not candidates:
            self.observe_bottom_only(cross,stamp)
            return
        # Production ranks association with P; unassociated legacy mode ranks
        # forward proximity. Never substitute a nearer, unrelated rectangle.
        _,_,lo,hi,left,right,width=min(candidates,key=lambda p:(p[1],p[0]))
        slope=(left[2]+right[2])/2
        x=(lo+hi)/2;y=((left[2]+right[2])*x+left[3]+right[3])/2
        self.view=dict(theta=self.pose[2]+math.atan(slope),center=world(self.pose,(x,y)),width=width,
            sides=[[world(self.pose,p) for p in side[:2]] for side in (left,right)])
        self.tracked_view=self.view;self.tracked_view_stamp=stamp
        # Bottom must join both far endpoints, never the near mouth.
        bottoms=[]
        for a,b,line in (cross if self.bottom_target_aligned else []):
            bx=(a[0]+b[0])/2
            if abs(bx-left[1][0])>.10 or abs(bx-right[1][0])>.10:continue
            ends=sorted((a[1],b[1]));ys=sorted((left[2]*bx+left[3],right[2]*bx+right[3]))
            if max(abs(ends[j]-ys[j]) for j in (0,1))>.08:continue
            bottoms.append((bx,line))
        if bottoms and self.bottom_stamp!=stamp:
            self.remember_bottom(min(bottoms,key=lambda p:p[0])[1],stamp,'paired_far_end')

    def command(self,now,pose):
        self.pose=tuple(pose);self.reason=None
        self.debug=dict(phase=self.phase,line_age_s=now-self.stamp,bottom_seen=self.bottom is not None,
            bottom_source=self.bottom_source,
            bottom_local=[local(self.pose,p) for p in self.bottom] if self.bottom is not None else None,
            sign_anchor_local=local(self.pose,self.sign_anchor) if self.sign_anchor is not None else None,
            sides_behind_sign_rejected=self.sides_behind_sign_rejected,
            bottom_target_aligned=self.bottom_target_aligned,
            sign_association=self.associate,
            bottom_only_search=self.bottom_only and self.view is None,
            bottom_candidate_frames=(self.bottom_candidate or {}).get('frames',0))
        if self.status=='FINISHED':
            self.reason='parking_at_bottom_clearance';return (0,0.)
        if not 0<=now-self.stamp<=self.cfg.get('ground_timeout',1.25):
            self.reason='parking_front_stale';return (0,0.)
        speed=self.cfg.get('parking_entry_speed_raw',12)
        if self.bottom is not None:
            # Reuse the front-bumper calculation, including the nearer corner
            # at an oblique line. Detection alone says nothing about arrival.
            remaining=self.remaining()
            clearance=self.cfg.get('parking_bottom_clearance_m',.04)
            self.debug.update(remaining_m=remaining,target_clearance_m=clearance,
                bumper_clearance_m=None if remaining is None else remaining+clearance,
                bottom_age_s=now-self.bottom_stamp,
                visual_motion_scale=self.motion_scale,visual_motion_samples=len(self.motion_ratios),
                distance_source='live_bottom' if self.bottom_stamp==self.stamp else 'measured_bottom_plus_pose')
            if remaining is None:
                self.reason='parking_bottom_geometry_invalid';return (0,0.)
            if remaining<=0:
                self.status='FINISHED';self.phase='FINISHED';self.reason='parking_at_bottom_clearance'
                self.debug['phase']=self.phase
                return (0,0.)
            # Retain the measured world line when it enters the camera's near
            # blind region; require fresh image callbacks above throughout.
            if remaining<=self.cfg.get('parking_final_max_m',.30):
                speed=min(speed,self.cfg.get('parking_final_speed_raw',8))
        if self.motion_speed!=speed:
            # A different raw speed can have a different deadband/gain. Wait
            # for new visual travel evidence rather than transferring its ratio.
            self.motion_speed=speed;self.motion_sample=None;self.motion_ratios=[]
            self.motion_scale=1.
        if self.view is None:
            if self.bottom_only and self.bottom is not None:
                self.reason='parking_follow_bottom_line'
                return (speed,0.)
            if self.single is not None and self.phase=='APPROACH':
                self.reason='parking_follow_sign_side'
                self.debug['side_local']=[local(self.pose,p) for p in self.single['side']]
                return (speed,self.limited_steer(self.single))
            self.reason='parking_missing_white_straight'
            return (speed,0.)
        self.phase='CENTER';self.debug.update(phase=self.phase,width_m=self.view['width'])
        self.debug['pair_local']=[[local(self.pose,p) for p in side] for side in self.view['sides']]
        self.reason='parking_center_between_sides'
        return (speed,self.limited_steer(self.view))

    def limited_steer(self,view):
        steer=self.steer(view['theta'],view['center'])
        limit=6.0/self.cfg['steering_raw_limit']*self.cfg.get('steering_command_scale_rad',self.cfg['max_steer'])
        return max(-limit,min(limit,steer))
