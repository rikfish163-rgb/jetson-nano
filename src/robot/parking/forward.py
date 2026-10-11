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
        self.left_reference=None
        self.bottom_only=False;self.bottom_candidate=None
        self.bottom_stop_enabled=True
        self.bottom_reacquire_candidate=None
        self.bottom_detection_reason='not_observed'
        self.bottom_cross_candidates=0
        self.finish_reason=None
        self.reset_bottom_loss()
        self.independent_bottom=False
        self.lock_target_bay=False
        self.bay_width=cfg.get('parking_bay_width_m',.38)

    def target_corridor(self):
        reference=self.tracked_view or self.left_reference
        if reference is not None:
            return tuple(reference['center'])+(reference['theta'],)
        if self.sign_anchor is not None:
            return tuple(self.sign_anchor)+(self.pose[2],)
        return self.pose

    def observe_bottom_only(self, cross, stamp):
        """Confirm a measured transverse stripe inside the target corridor."""
        if not self.bottom_only or self.bottom is not None:
            return
        if self.lock_target_bay:
            if self.sign_anchor is None:return
            corridor=self.target_corridor()
        elif self.tracked_view is not None and not self.independent_bottom:
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
            if self.lock_target_bay:
                width=self.bay_width
                if max(abs(low+width/2.),abs(high-width/2.))>.08:
                    continue
            x=a[0]-a[1]*dx/dy
            if self.tracked_view is not None and not self.independent_bottom:
                far_end=max(local(corridor,p)[0]
                            for side in self.tracked_view['sides'] for p in side)
                if x<far_end-.25:
                    continue  # A transverse stripe near the remembered mouth is not the bottom.
            center=world(corridor,(x,0.))
            forward=local(self.pose,center)[0]
            if not .05<forward<=1.2:
                continue
            if (self.tracked_view is None or self.lock_target_bay) and self.sign_anchor is not None:
                ax=local(self.pose,self.sign_anchor)[0]
                # Rear P marks the back of a 45 cm bay. Allowing 45 cm here
                # would also admit its mouth as the stopping line.
                maximum=.25 if self.lock_target_bay else .45
                if not -.10<=ax-forward<=maximum:
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
            self.remember_bottom(line,stamp,
                'observed_bottom' if self.independent_bottom else 'standalone_bottom')

    def remember_bottom(self,line,stamp,source):
        self.bottom_detection_reason=source
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

    def observe_left_side(self, lines, stamp, side_sign=1):
        """Infer the bay center half a width inward from either target edge."""
        self.single = None
        reference = self.left_reference
        unanchored = getattr(self, 'allow_unanchored_side', False)
        if (reference is not None and not self.lock_target_bay and
                not 0 <= stamp-reference['stamp'] <= self.cfg.get('ground_timeout',1.25)):
            if not unanchored:
                return
            reference = self.left_reference = None
        anchor = local(self.pose,self.sign_anchor) if self.sign_anchor is not None else None
        if reference is None and (not self.associate or anchor is None) and not unanchored:
            return
        width = self.bay_width
        candidates = []
        for line in lines:
            a,b = sorted(local(self.pose,p) for p in line)
            dx,dy = b[0]-a[0],b[1]-a[1]
            minimum = .04 if reference is not None else .18
            if dx < minimum or abs(math.atan2(dy,dx)) > math.radians(35):
                continue
            theta = math.atan2(dy,dx)
            slope,intercept = dy/dx,a[1]-a[0]*dy/dx
            lo,hi = max(.05,a[0]),min(1.8,b[0])
            if self.bottom is not None:
                ba,bb = [local(self.pose,p) for p in self.bottom]
                if abs(bb[1]-ba[1]) < .05:continue
                k = (bb[0]-ba[0])/(bb[1]-ba[1])
                if abs(1-k*slope) < .1:continue
                crossing = (ba[0]-k*ba[1]+k*intercept)/(1-k*slope)
                hi = min(hi,crossing+.01)
            if reference is not None:
                heading = (self.pose[2]+theta-reference['theta']+math.pi)%(2*math.pi)-math.pi
                rx,ry = local(self.pose,reference['center'])
                error = -math.sin(theta)*(a[0]-rx)+math.cos(theta)*(a[1]-ry)-side_sign*width/2.
                if abs(heading)>math.radians(10) or abs(error)>.08:continue
                score = abs(error)+.1*abs(heading)
            elif anchor is not None:
                ax,ay = anchor
                # Signed distance distinguishes the requested target edge
                # from its opposite edge and neighbouring road paint.
                offset = side_sign*(-math.sin(theta)*(a[0]-ax)+math.cos(theta)*(a[1]-ay))
                if not .12<=offset<=.29 or abs(b[0]-ax)>.45:continue
                hi = min(hi,ax)
                score = abs(offset-width/2.)+abs(b[0]-ax)
            else:
                # Rear entry has no front P anchor. Acquire the measured
                # preferred side at the same half-bay clearance; the rear
                # adapter maps this left-side calculation to rear-view right.
                offset = side_sign*(-math.sin(theta)*a[0]+math.cos(theta)*a[1])
                if not .12<=offset<=.29:
                    continue
                score = abs(offset-width/2.)+.1*lo
            if hi-lo < minimum:continue
            x = (lo+hi)/2.; y = slope*x+intercept
            center = world(self.pose,(x+side_sign*math.sin(theta)*width/2.,
                                      y-side_sign*math.cos(theta)*width/2.))
            side = [world(self.pose,(q,slope*q+intercept)) for q in (lo,hi)]
            candidates.append((score,dict(theta=self.pose[2]+theta,center=center,
                                         side=side,stamp=stamp,
                                         source='left_side' if side_sign>0 else 'right_side')))
        if candidates:
            self.single = min(candidates,key=lambda row:row[0])[1]
            self.left_reference = self.single

    def reset_bottom_loss(self,after=-1.):
        self.bottom_loss_after=after
        self.bottom_loss_origin=None;self.bottom_loss_last_pose=None
        self.bottom_loss_travel=0.

    def observe(self,lines,stamp,pose=None):
        if stamp<=self.stamp:return
        self.observe_geometry(lines,stamp,pose)
        if not self.bottom_only or self.status=='FINISHED':return
        if self.bottom_stamp==stamp:
            self.reset_bottom_loss(self.bottom_loss_after)
        elif (self.bottom is not None and self.bottom_stamp>self.bottom_loss_after
              and self.bottom_loss_origin is None):
            # A new image without the previously acquired bottom starts the
            # final 15 cm at that image's pose, not at P or the last sighting.
            self.bottom_loss_origin=tuple(self.pose)
            self.bottom_loss_last_pose=tuple(self.pose)
            self.bottom_loss_travel=0.

    def observe_geometry(self,lines,stamp,pose=None):
        if stamp<=self.stamp:return
        if pose is not None:self.pose=tuple(pose)
        self.stamp=stamp;self.view=None;self.single=None
        self.bottom_detection_reason='no_matching_bottom'
        self.bottom_cross_candidates=0
        self.sides_behind_sign_rejected=0
        anchor=local(self.pose,self.sign_anchor) if self.sign_anchor is not None else None
        if self.associate and anchor is None:
            self.bottom_detection_reason='missing_sign_anchor'
            return
        # Only a measured pair may constrain the identity of another pair.
        # A P-inferred single edge may have its left/right identity reversed;
        # its half-width centre must never veto stronger two-edge evidence.
        previous=self.tracked_view
        tracking=(self.associate and previous is not None and
                  (self.lock_target_bay or ((self.phase=='CENTER' or self.bottom_only) and
                   0<=stamp-previous.get('stamp',self.tracked_view_stamp)<=self.cfg.get('ground_timeout',1.25))))
        # A bottom stripe may be shared by several adjacent bays. Seeing the
        # stripe below P is not our target while still aimed at its neighbour.
        width=self.bay_width
        self.bottom_target_aligned=(not self.associate or abs(anchor[1])<=width/2.+.04)
        sides=[];cross=[]
        cross_minimum=.08 if self.bottom is not None and not self.lock_target_bay else .18
        for line in lines:
            a,b=sorted(local(self.pose,p) for p in line)
            dx,dy=b[0]-a[0],b[1]-a[1]
            if math.hypot(dx,dy)<cross_minimum:continue
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
                    # P depth is approximate. Keep the measured far endpoint:
                    # clipping it to P makes a real closing stripe look detached.
                sides.append((a,b,slope,intercept))
            elif abs(dy)>cross_minimum and abs(dx/dy)<.35:cross.append((a,b,line))
        self.bottom_cross_candidates=len(cross)
        if not cross:self.bottom_detection_reason='no_transverse_segments'
        # Hough returns short duplicates of long rails. A short fragment ending
        # near approximate P depth must not win over the same fully seen edge.
        complete=[]
        for side in sides:
            redundant=False
            for other in sides:
                if other[1][0]-other[0][0]<=side[1][0]-side[0][0]+.02:continue
                if other[0][0]>side[0][0]+.015 or other[1][0]<side[1][0]-.015:continue
                if abs(math.atan(other[2])-math.atan(side[2]))>math.radians(5):continue
                x=(side[0][0]+side[1][0])/2.
                if abs((other[2]-side[2])*x+other[3]-side[3])<.025:
                    redundant=True;break
            if not redundant:complete.append(side)
        sides=complete
        if self.independent_bottom:
            # Rear stopping is acquired from the transverse paint itself,
            # with two-frame confirmation even while both side rails exist.
            self.observe_bottom_only(cross,stamp)
        if self.bottom is not None and self.bottom_stamp!=stamp:
            # P initializes identity. Thereafter track the same measured stripe
            # even when dead-reckoned P has drifted behind the visible bottom.
            a,b=[local(self.pose,p) for p in self.bottom]
            dx,dy=b[0]-a[0],b[1]-a[1]
            matches=[];farther=[]
            if abs(dy)>cross_minimum and abs(dx/dy)<.5:
                slope=dx/dy;expected=a[0]-a[1]*slope
                old_low,old_high=sorted((a[1],b[1]))
                for a,b,line in cross:
                    low,high=sorted((a[1],b[1]))
                    if self.lock_target_bay:
                        # Match the locked stripe's finite lateral extent, not
                        # a neighbour or a broad road stripe at the same depth.
                        corridor=self.target_corridor()
                        ends=sorted(local(corridor,p)[1] for p in line)
                        old=sorted(local(corridor,p)[1] for p in self.bottom)
                        if max(abs(ends[j]-old[j]) for j in (0,1))>.10:continue
                    elif min(high,old_high)-max(low,old_low)<min(.18,.8*(high-low),.8*(old_high-old_low)):
                        # Track the observed stripe's overlap, not whether it
                        # straddles the car. A valid paired bottom can be only
                        # short or off centre while the car is aligning.
                        continue
                    k=(b[0]-a[0])/(b[1]-a[1]);x=a[0]-a[1]*k
                    error=abs(x-expected)
                    angle_error=abs(math.atan(k)-math.atan(slope))
                    if .05<x<1.8 and error<=.12 and angle_error<=math.radians(10):
                        matches.append((error,line))
                    elif (.05<x<1.8 and .12<x-expected<=.45 and
                          angle_error<=math.radians(10)):
                        # Command odometry may have advanced past the actual
                        # stripe. Only recover farther paint (never a nearer
                        # mouth), with the same finite overlap and two frames.
                        farther.append((error,line,world(self.pose,(x,0.)),
                                        self.pose[2]-math.atan(k)))
            if matches:
                self.remember_bottom(min(matches,key=lambda row:row[0])[1],stamp,'tracked_bottom')
                self.bottom_reacquire_candidate=None
            elif farther:
                unused_error,line,center,angle=min(farther,key=lambda row:row[0])
                previous_bottom=self.bottom_reacquire_candidate
                consistent=(previous_bottom is not None and
                    0<stamp-previous_bottom['stamp']<=self.cfg.get('ground_timeout',1.25) and
                    math.hypot(center[0]-previous_bottom['center'][0],
                               center[1]-previous_bottom['center'][1])<=.10 and
                    abs((angle-previous_bottom['angle']+math.pi)%(2*math.pi)-math.pi)<=math.radians(10))
                self.bottom_reacquire_candidate=dict(stamp=stamp,center=center,angle=angle)
                if consistent:
                    self.remember_bottom(line,stamp,'reacquired_bottom')
                    self.bottom_reacquire_candidate=None
            else:
                self.bottom_reacquire_candidate=None
        if self.associate and not self.independent_bottom and self.bottom_target_aligned and self.bottom_stamp!=stamp:
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
                if self.lock_target_bay and self.bottom is not None:
                    hi=min(hi,max(local(self.pose,q)[0] for q in self.bottom))
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
                    theta=previous['theta']-self.pose[2]
                    rx,ry=local(self.pose,previous['center'])
                    slope=(left[2]+right[2])/2.
                    heading=(math.atan(slope)-theta+math.pi)%(2*math.pi)-math.pi
                    lateral=-(x-rx)*math.sin(theta)+(center-ry)*math.cos(theta)
                    previous_width=previous.get('width',self.bay_width)
                    if (abs(heading)>math.radians(10) or abs(lateral)>.08
                            or abs(width-previous_width)>.06):continue
                    if self.bottom is not None:
                        if lo>=max(local(self.pose,q)[0] for q in self.bottom):continue
                    score=abs(lateral)+abs(width-previous_width)
                elif self.associate:
                    ax,ay=anchor
                    edges=sorted((left[2]*ax+left[3],right[2]*ax+right[3]))
                    # A P stand may sit beside a bay, as in the recorded course.
                    # Keep width/parallel/end checks and rank by P proximity;
                    # do not discard a real pair merely because P is outside it.
                    slope=(left[2]+right[2])/2.
                    # Single-edge acquisition permits P up to 29 cm outside
                    # an edge. Do not reject both measured sides more strictly
                    # than that weaker fallback (normal distance to the rails).
                    margin=max(self.bay_width/2.,.29)*math.sqrt(1+slope*slope)
                    if not edges[0]-margin<=ay<=edges[1]+margin:continue
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
                    y=side[2]*x+side[3]-(self.bay_width/2. if offset>0 else -self.bay_width/2.)
                    matches.append((abs(side[1][0]-ax),side,x,y))
            if matches:
                _,side,x,y=min(matches,key=lambda row:row[0])
                self.single=dict(theta=self.pose[2]+math.atan(side[2]),center=world(self.pose,(x,y)),
                    side=[world(self.pose,p) for p in side[:2]])
        if not candidates:
            if not self.independent_bottom:
                self.observe_bottom_only(cross,stamp)
            if self.bottom_only:
                self.observe_left_side(lines,stamp)
                if self.single is None:
                    self.observe_left_side(lines,stamp,side_sign=-1)
            return
        # Production ranks association with P; unassociated legacy mode ranks
        # forward proximity. Never substitute a nearer, unrelated rectangle.
        _,_,lo,hi,left,right,width=min(candidates,key=lambda p:(p[1],p[0]))
        slope=(left[2]+right[2])/2
        x=(lo+hi)/2;y=((left[2]+right[2])*x+left[3]+right[3])/2
        self.view=dict(theta=self.pose[2]+math.atan(slope),center=world(self.pose,(x,y)),width=width,
            sides=[[world(self.pose,p) for p in side[:2]] for side in (left,right)])
        self.tracked_view=self.view;self.tracked_view_stamp=stamp
        if self.bottom_only:
            self.left_reference=dict(theta=self.view['theta'],center=self.view['center'],stamp=stamp)
        if self.independent_bottom:
            return  # Side endpoints never acquire/reassign the rear bottom.
        # The paired bay, rather than the board beside it, establishes whether
        # this car is in the target corridor. Preserve the adjacent-bay guard.
        self.bottom_target_aligned=(abs((left[3]+right[3])/2.)/
            math.sqrt(1+slope*slope)<=width/2.+.04)
        # Bottom must join both far endpoints, never the near mouth.
        bottoms=[]
        for a,b,line in (cross if self.bottom_target_aligned else []):
            k=(b[0]-a[0])/(b[1]-a[1]);intercept=a[0]-k*a[1]
            ends=sorted((a[1],b[1]));joined=[]
            for side in (left,right):
                denominator=1-side[2]*k
                if abs(denominator)<.1:break
                by=(side[2]*intercept+side[3])/denominator
                bx=intercept+k*by
                if not ends[0]-.08<=by<=ends[1]+.08 or abs(bx-side[1][0])>.10:break
                # A shared bottom may continue into adjacent bays. Retain only
                # its actually observed portion inside these measured rails.
                by=max(ends[0],min(ends[1],by))
                joined.append(world(self.pose,(intercept+k*by,by)))
            if len(joined)!=2 or math.hypot(joined[1][0]-joined[0][0],joined[1][1]-joined[0][1])<.18:continue
            bottoms.append(((a[0]+b[0])/2.,joined))
        if bottoms:
            source=('tracked_bottom' if self.bottom_stamp==stamp and
                    self.bottom_source=='tracked_bottom' else 'paired_far_end')
            self.remember_bottom(min(bottoms,key=lambda p:p[0])[1],stamp,source)

    def command(self,now,pose):
        self.pose=tuple(pose);self.reason=None
        self.debug=dict(phase=self.phase,line_age_s=now-self.stamp,bottom_seen=self.bottom is not None,
            bottom_detection_reason=self.bottom_detection_reason,
            bottom_cross_candidates=self.bottom_cross_candidates,
            bottom_measurement_stamp=self.bottom_stamp,
            bottom_source=self.bottom_source,
            bottom_local=[local(self.pose,p) for p in self.bottom] if self.bottom is not None else None,
            sign_anchor_local=local(self.pose,self.sign_anchor) if self.sign_anchor is not None else None,
            sides_behind_sign_rejected=self.sides_behind_sign_rejected,
            bottom_target_aligned=self.bottom_target_aligned,
            sign_association=self.associate,
            bottom_only_search=(self.bottom_only and self.view is None and
                not (self.single is not None and self.single.get('source') in ('left_side','right_side'))),
            left_seen=self.view is not None or (self.single is not None and self.single.get('source')=='left_side'),
            right_seen=self.view is not None or (self.single is not None and self.single.get('source')=='right_side'),
            steering_reference='none',
            bottom_visible_current_frame=self.bottom is not None and self.bottom_stamp==self.stamp,
            bottom_loss_active=self.bottom_loss_origin is not None,
            bottom_loss_travel_m=self.bottom_loss_travel,
            bottom_loss_remaining_m=max(0.,.15-self.bottom_loss_travel) if self.bottom_loss_origin is not None else None,
            bottom_candidate_frames=(self.bottom_candidate or {}).get('frames',0))
        if self.status=='FINISHED':
            self.reason=self.finish_reason or 'parking_at_bottom_clearance';return (0,0.)
        loss_active=self.bottom_stop_enabled and self.bottom_only and self.bottom_loss_origin is not None
        if loss_active:
            previous=self.bottom_loss_last_pose
            self.bottom_loss_travel+=math.hypot(pose[0]-previous[0],pose[1]-previous[1])
            self.bottom_loss_last_pose=tuple(pose)
            self.debug.update(bottom_loss_travel_m=self.bottom_loss_travel,
                bottom_loss_remaining_m=max(0.,.15-self.bottom_loss_travel))
            if self.bottom_loss_travel>=.15-1e-9:
                self.status=self.phase='FINISHED'
                self.reason=self.finish_reason='parking_bottom_lost_15cm'
                self.debug['phase']=self.phase
                return (0,0.)
        # Explicit S keeps following measured world geometry with the current
        # pose when camera updates pause; other forward modes require freshness.
        fresh = 0<=now-self.stamp<=self.cfg.get('ground_timeout',1.25)
        if now<self.stamp or (not self.bottom_only and not fresh):
            self.reason='parking_front_stale';return (0,0.)
        speed=self.cfg.get('parking_entry_speed_raw',12)
        if self.bottom is not None:
            # Reuse the front-bumper calculation, including the nearer corner
            # at an oblique line. Detection alone says nothing about arrival.
            remaining=self.remaining()
            clearance=self.cfg.get('parking_bottom_clearance_m',.04)
            measured=WhiteParking.remaining(self,pose=self.bottom_pose or self.pose)
            bottom_fresh=(self.bottom_stamp==self.stamp and
                0<=now-self.bottom_stamp<=self.cfg.get('ground_timeout',1.25))
            arrived=bottom_fresh and measured is not None and measured<=0
            self.debug.update(remaining_m=remaining,target_clearance_m=clearance,
                bumper_clearance_m=None if remaining is None else remaining+clearance,
                measured_bumper_clearance_m=None if measured is None else measured+clearance,
                bottom_arrival_confirmed=arrived,
                bottom_age_s=now-self.bottom_stamp,
                visual_motion_scale=self.motion_scale,visual_motion_samples=len(self.motion_ratios),
                distance_source='live_bottom' if fresh and self.bottom_stamp==self.stamp else 'measured_bottom_plus_pose')
            if remaining is None and not loss_active and self.bottom_stop_enabled:
                self.reason='parking_bottom_geometry_invalid';return (0,0.)
            if arrived and self.bottom_stop_enabled:
                self.status='FINISHED';self.phase='FINISHED';self.reason='parking_at_bottom_clearance'
                self.debug['phase']=self.phase
                return (0,0.)
            # Model distance may slow the approach, but cannot prove arrival.
            # Only a recent image measuring the bumper at the target can finish.
            if loss_active or (remaining is not None and remaining<=self.cfg.get('parking_final_max_m',.30)):
                speed=min(speed,self.cfg.get('parking_final_speed_raw',8))
        if self.motion_speed!=speed:
            # A different raw speed can have a different deadband/gain. Wait
            # for new visual travel evidence rather than transferring its ratio.
            self.motion_speed=speed;self.motion_sample=None;self.motion_ratios=[]
            self.motion_scale=1.
        if self.view is None:
            if self.bottom_only and self.single is not None and self.single.get('source') in ('left_side','right_side'):
                self.reason='parking_follow_'+self.single['source']
                self.debug.update(steering_reference=self.single['source'],
                    side_local=[local(self.pose,p) for p in self.single['side']])
                return (speed,self.limited_steer(self.single))
            if self.bottom_only and self.bottom is not None:
                self.reason='parking_follow_bottom_line'
                self.debug['steering_reference']='bottom_only_straight'
                return (speed,0.)
            if self.single is not None and self.phase=='APPROACH':
                self.reason='parking_follow_sign_side'
                self.debug['side_local']=[local(self.pose,p) for p in self.single['side']]
                return (speed,self.limited_steer(self.single))
            self.reason='parking_missing_white_straight'
            return (speed,0.)
        self.phase='CENTER';self.debug.update(phase=self.phase,width_m=self.view['width'])
        self.debug['steering_reference']='both_sides'
        self.debug['pair_local']=[[local(self.pose,p) for p in side] for side in self.view['sides']]
        self.reason='parking_center_between_sides'
        return (speed,self.limited_steer(self.view))

    def limited_steer(self,view,direction=1):
        # Only the calibrated chassis range remains; no parking-specific cap.
        heading,lateral=self.errors(view['theta'],view['center'])
        steer=self.steer(view['theta'],view['center'],direction=direction)
        self.debug.update(centerline_local=local(self.pose,view['center']),
            centerline_heading_error_deg=math.degrees(heading),
            centerline_lateral_error_m=lateral,steering_command_rad=steer)
        return steer
