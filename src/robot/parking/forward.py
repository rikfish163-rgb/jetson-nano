"""Forward visual parking from paired bay sides and an observed bottom edge."""
from __future__ import division
import math
from robot.common.geometry import distance, local, world, wrap
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
        self.bottom_reference_travel=0.;self.bottom_reference_last_pose=None
        self.bottom_depth_jump_rejected=0
        self.tracked_view=None;self.tracked_view_stamp=-1.
        self.left_reference=None
        self.bottom_only=False;self.bottom_candidate=None
        self.bottom_stop_enabled=True
        self.bottom_reacquire_candidate=None
        self.bottom_detection_reason='not_observed'
        self.bottom_cross_candidates=0
        self.bottom_rejection_counts={}
        self.bottom_corridor_source=None
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
        reference=self.tracked_view
        if self.independent_bottom:
            # Match the current image, in the same order used for steering.
            # A retained pair must not override a newly measured target edge.
            current_single=(self.single if self.single is not None and
                self.single.get('stamp')==stamp and distance(*self.single['side'])>=.18 else None)
            reference=self.view or current_single or self.tracked_view
            self.bottom_corridor_source=('current_pair' if self.view is not None else
                'current_side' if current_single is not None else
                'retained_pair' if reference is not None else None)
        if self.independent_bottom and self.lock_target_bay and reference is None:
            # Rear P alone does not identify a ground stripe. Establish the
            # target bay from a measured pair or a P-associated long side.
            self.bottom_candidate=None
            self.bottom_detection_reason='bottom_waiting_bay_sides'
            return
        if self.lock_target_bay:
            if self.sign_anchor is None:return
            corridor=(tuple(reference['center'])+(reference['theta'],)
                      if reference is not None else self.target_corridor())
        elif self.tracked_view is not None and not self.independent_bottom:
            corridor = tuple(self.tracked_view['center'])+(self.tracked_view['theta'],)
        else:
            if self.associate and not self.bottom_target_aligned:
                self.bottom_candidate=None
                return
            corridor = self.pose
        candidates=[];rejections=[]
        for unused_a, unused_b, line in cross:
            a,b=[local(corridor,p) for p in line]
            dx,dy=b[0]-a[0],b[1]-a[1]
            low,high=sorted((a[1],b[1]))
            if not .30<=high-low<=.60 or not low<=-.12<.12<=high:
                rejections.append('width_or_span')
                continue
            if abs(dx/dy)>.35 or abs((low+high)/2.)>.10:
                rejections.append('angle_or_center')
                continue
            if self.lock_target_bay and not self.independent_bottom:
                width=self.bay_width
                if max(abs(low+width/2.),abs(high-width/2.))>.08:
                    continue
            x=a[0]-a[1]*dx/dy
            if reference is not None:
                sides=reference.get('sides') or [reference['side']]
                far_end=max(local(corridor,p)[0]
                            for side in sides for p in side)
                # Use front S's tolerance for fragmented/extended side ends.
                if x<far_end-.25:
                    rejections.append('before_side_far_end')
                    continue  # A transverse stripe near the remembered mouth is not the bottom.
            center=world(corridor,(x,0.))
            forward=local(self.pose,center)[0]
            if not .05<forward<=1.2:
                rejections.append('outside_view_depth')
                continue
            if (not self.independent_bottom and
                    (self.tracked_view is None or self.lock_target_bay) and
                    self.sign_anchor is not None):
                ax=local(self.pose,self.sign_anchor)[0]
                # Legacy front association only. Rear bottom depth is measured
                # from the stripe, independently of the locked P depth.
                maximum=.25 if self.lock_target_bay else .45
                if not -.10<=ax-forward<=maximum:
                    continue
            candidates.append((forward,line,center,math.atan(dx/dy)+corridor[2]))
        self.bottom_rejection_counts=dict((reason,rejections.count(reason)) for reason in set(rejections))
        if not candidates:
            self.bottom_candidate=None
            if rejections:
                self.bottom_detection_reason='bottom_rejected_'+max(sorted(set(rejections)),key=rejections.count)
            return
        unused_forward,line,center,angle=min(candidates,key=lambda row:row[0])
        previous=self.bottom_candidate
        consistent=False
        if previous is not None and 0<stamp-previous['stamp']<=self.cfg.get('ground_timeout',1.25):
            comparisons=[(center,angle,previous['center'],previous['angle'])]
            if (self.independent_bottom and getattr(self,'camera_steering_feedback',False)
                    and 'observation_pose' in previous):
                # As with rear steering, command odometry must not break two
                # agreeing camera measurements by moving their world poses.
                comparisons.append((local(self.pose,center),angle-self.pose[2],
                    local(previous['observation_pose'],previous['center']),
                    previous['angle']-previous['observation_pose'][2]))
            consistent=any(math.hypot(a[0]-b[0],a[1]-b[1])<=.10 and
                abs(wrap(aa-ba))<=math.radians(10) for a,aa,b,ba in comparisons)
        self.bottom_candidate=dict(stamp=stamp,center=center,angle=angle,
            observation_pose=tuple(self.pose),
            frames=previous['frames']+1 if consistent else 1)
        self.bottom_detection_reason='bottom_confirming'
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
        if getattr(self, 'bottom_reference', None) is not None:
            # A fresh rear image resets the remaining distance. Use the same
            # signed net travel as front_axle_blue; never learn another gain.
            self.bottom_reference_travel=0.
            self.bottom_reference_last_pose=tuple(self.pose)
            self.motion_sample=None;self.motion_ratios=[];self.motion_scale=1.
            return
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

    def measured_remaining(self, pose):
        reference = getattr(self, 'bottom_reference', None)
        if reference is None:
            return WhiteParking.remaining(self, pose=pose)
        if self.bottom is None:
            return None
        a,b = [local(pose,p) for p in self.bottom]
        dx,dy = b[0]-a[0],b[1]-a[1]
        if abs(dy)<.05 or abs(dx/dy)>.35:
            return None
        depth = a[0]-a[1]*dx/dy
        return depth-reference['depth_m']+reference['remaining_m']

    @staticmethod
    def reference_step(previous, pose):
        # Same sign and midpoint-heading calculation as mission.set_pose's
        # front_axle_blue counter. This pose faces into the rear bay.
        dx,dy=pose[0]-previous[0],pose[1]-previous[1]
        heading=previous[2]+wrap(pose[2]-previous[2])/2.
        step=math.hypot(dx,dy)
        return -step if dx*math.cos(heading)+dy*math.sin(heading)<0 else step

    def advance_bottom_reference(self, pose):
        if getattr(self, 'bottom_reference', None) is None or self.bottom_pose is None:
            return
        previous=self.bottom_reference_last_pose or self.bottom_pose
        self.bottom_reference_travel+=self.reference_step(previous,pose)
        self.bottom_reference_last_pose=tuple(pose)

    def remaining(self):
        if getattr(self, 'bottom_reference', None) is not None:
            origin=self.bottom_pose or self.pose
            measured=self.measured_remaining(origin)
            if measured is None:return None
            previous=self.bottom_reference_last_pose or origin
            travel=self.bottom_reference_travel+self.reference_step(previous,self.pose)
            return measured-travel
        # Only extrapolation after the last measured line uses the locally
        # observed travel ratio. Do not alter global odometry or other actions.
        pose=self.pose
        if self.bottom_pose is not None:
            origin=self.bottom_pose
            pose=(origin[0]+(pose[0]-origin[0])*self.motion_scale,
                  origin[1]+(pose[1]-origin[1])*self.motion_scale,pose[2])
        return self.measured_remaining(pose)

    def side_reaches_anchor(self, end, anchor_x):
        far=getattr(self,'side_view_far_m',None)
        return (abs(end-anchor_x)<=.45 or
                (far is not None and anchor_x>far and end>=far-.12 and end<=far+.02))

    def steering_pose(self, reference):
        if getattr(self,'camera_steering_feedback',False):
            return reference.get('observation_pose',self.pose)
        return self.pose

    def association_poses(self, reference):
        # Match moving-car observations in the CURRENT frame, as front S does.
        # Command odometry is only an estimate: unchanged rear image geometry
        # may also match the last camera frame when that estimate has drifted.
        poses=[self.pose]
        if getattr(self,'camera_steering_feedback',False) and 'observation_pose' in reference:
            poses.append(reference['observation_pose'])
        return poses

    def far_cropped_side(self, end):
        far=getattr(self,'side_view_far_m',None)
        return far is not None and abs(end-far)<=.025

    def observe_bottom_image(self, cross, stamp):
        """Track rear paint between camera frames independently of model drift."""
        if (not getattr(self,'camera_steering_feedback',False) or self.bottom is None or
                self.bottom_pose is None or
                not 0<stamp-self.bottom_stamp<=self.cfg.get('ground_timeout',1.25)):
            return
        a,b=sorted((local(self.bottom_pose,p) for p in self.bottom),key=lambda p:p[1])
        if b[1]-a[1]<.08:return
        slope=(b[0]-a[0])/(b[1]-a[1])
        expected=a[0]-a[1]*slope
        matches=[]
        for unused_a,unused_b,line in cross:
            ca,cb=sorted((local(self.pose,p) for p in line),key=lambda p:p[1])
            span=cb[1]-ca[1]
            if span<.08:continue
            k=(cb[0]-ca[0])/span
            x=ca[0]-ca[1]*k
            overlap=min(b[1],cb[1])-max(a[1],ca[1])
            if (not .05<x<1.8 or abs(x-expected)>.12 or
                    abs(math.atan(k)-math.atan(slope))>math.radians(10) or
                    overlap<min(.18,.8*span,.8*(b[1]-a[1]))):continue
            # Keep only observed paint overlapping the previous target stripe.
            if self.tracked_view is not None:
                pair=self.tracked_view
                origin=pair.get('observation_pose',self.bottom_pose)
                corridor=local(origin,pair['center'])+(wrap(pair['theta']-origin[2]),)
                pa,pb=sorted((local(corridor,p) for p in (ca,cb)),key=lambda p:p[1])
                half=pair.get('width',self.bay_width)/2.
                lo,hi=max(pa[1],-half),min(pb[1],half)
                if hi-lo<.08:continue
                slope_in_bay=(pb[0]-pa[0])/(pb[1]-pa[1])
                points=[world(corridor,(pa[0]+(y-pa[1])*slope_in_bay,y)) for y in (lo,hi)]
            else:
                lo,hi=max(ca[1],a[1]-.10),min(cb[1],b[1]+.10)
                points=[(x+k*y,y) for y in (lo,hi)]
            clipped=[world(self.pose,p) for p in points]
            matches.append((abs(x-expected),clipped))
        if matches:
            self.remember_bottom(min(matches,key=lambda row:row[0])[1],stamp,'tracked_bottom')
            self.bottom_reacquire_candidate=None

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
                matches=[]
                for reference_pose in self.association_poses(reference):
                    heading = (reference_pose[2]+theta-reference['theta']+math.pi)%(2*math.pi)-math.pi
                    rx,ry = local(reference_pose,reference['center'])
                    error = -math.sin(theta)*(a[0]-rx)+math.cos(theta)*(a[1]-ry)-side_sign*width/2.
                    if abs(heading)<=math.radians(10) and abs(error)<=.08:
                        matches.append(abs(error)+.1*abs(heading))
                if not matches:continue
                score = min(matches)
            elif anchor is not None:
                ax,ay = anchor
                # Signed distance distinguishes the requested target edge
                # from its opposite edge and neighbouring road paint.
                offset = side_sign*(-math.sin(theta)*(a[0]-ax)+math.cos(theta)*(a[1]-ay))
                if not .12<=offset<=.29 or not self.side_reaches_anchor(b[0],ax):continue
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
                                         observation_pose=tuple(self.pose),
                                         source='left_side' if side_sign>0 else 'right_side')))
        if candidates:
            if self.independent_bottom:
                # A few pixels of Hough angle noise on a 4 cm fragment must
                # not outrank a visible long rail of the same confirmed bay.
                long_sides=[row for row in candidates if distance(*row[1]['side'])>=.18]
                candidates=long_sides or candidates
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
        if pose is not None:
            self.advance_bottom_reference(pose)
            self.pose=tuple(pose)
        self.stamp=stamp;self.view=None;self.single=None
        self.bottom_detection_reason='no_matching_bottom'
        self.bottom_cross_candidates=0
        self.bottom_rejection_counts={}
        self.bottom_corridor_source=None
        self.bottom_depth_jump_rejected=0
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
        cross_minimum=.08 if self.bottom is not None else .18
        for line in lines:
            a,b=sorted(local(self.pose,p) for p in line)
            dx,dy=b[0]-a[0],b[1]-a[1]
            minimum_side=.08 if self.independent_bottom and self.far_cropped_side(b[0]) else .18
            if math.hypot(dx,dy)<min(cross_minimum,minimum_side):continue
            if dx>minimum_side and abs(dy/dx)<math.tan(math.radians(35)):
                slope,intercept=dy/dx,a[1]-a[0]*dy/dx
                if self.associate and not tracking:
                    # Only observed paint BEFORE P can bound its approach bay.
                    # A background segment cannot be extended backwards to
                    # invent a missing side in front of the board.
                    end=min(b[0],anchor[0])
                    if end-max(a[0],.05)<minimum_side:
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
        self.observe_bottom_image(cross,stamp)
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
                        ca,cb=sorted((local(corridor,p) for p in line),key=lambda p:p[1])
                        ends=[ca[1],cb[1]]
                        if self.independent_bottom and self.tracked_view is not None:
                            # A continuous stripe may extend into a neighbour.
                            # Keep its observed overlap with our measured bay;
                            # a neighbour-only piece has no such overlap.
                            half=self.tracked_view.get('width',self.bay_width)/2.
                            clipped_low=max(ends[0],-half)
                            clipped_high=min(ends[1],half)
                            if clipped_high-clipped_low<.08:continue
                            kx=(cb[0]-ca[0])/(cb[1]-ca[1])
                            line=[world(corridor,(ca[0]+(y-ca[1])*kx,y))
                                  for y in (clipped_low,clipped_high)]
                            a,b=[local(self.pose,p) for p in line]
                            low,high=sorted((a[1],b[1]))
                            ends=[clipped_low,clipped_high]
                        old=sorted(local(corridor,p)[1] for p in self.bottom)
                        overlap=min(ends[1],old[1])-max(ends[0],old[0])
                        # Bound by the bay, not the last short fragment, so a
                        # full stripe can return without admitting a neighbour.
                        if (ends[0]<-self.bay_width/2.-.10 or ends[1]>self.bay_width/2.+.10 or
                                overlap<min(.18,.8*(ends[1]-ends[0]),.8*(old[1]-old[0]))):continue
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
                    elif (getattr(self,'bottom_reference',None) is None and
                          .05<x<1.8 and .12<x-expected<=.45 and
                          angle_error<=math.radians(10)):
                        # Command odometry may have advanced past the actual
                        # stripe. Only recover farther paint (never a nearer
                        # mouth), with the same finite overlap and two frames.
                        farther.append((error,line,world(self.pose,(x,0.)),
                                        self.pose[2]-math.atan(k)))
                    elif (getattr(self,'bottom_reference',None) is not None and
                          .05<x<1.8 and x-expected>.12 and
                          angle_error<=math.radians(10)):
                        # Keep the confirmed rear stripe. A farther cross-line
                        # must not restart the final remaining-distance budget.
                        self.bottom_depth_jump_rejected+=1
                        self.bottom_detection_reason='bottom_track_depth_jump'
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
                if (self.bottom_reacquire_candidate or {}).get('source')!='paired':
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
        self.debug['pair_tracking_rejected']=0
        for i,left in enumerate(sides):
            for right in sides[i+1:]:
                lo=max(left[0][0],right[0][0],.05)
                hi=min(left[1][0],right[1][0],1.8)
                stale_rear_bottom=(self.independent_bottom and
                    getattr(self,'bottom_reference',None) is not None and self.bottom_stamp!=stamp)
                if self.lock_target_bay and self.bottom is not None and not stale_rear_bottom:
                    hi=min(hi,max(local(self.pose,q)[0] for q in self.bottom))
                minimum_overlap=(.08 if self.independent_bottom and
                    any(self.far_cropped_side(side[1][0]) for side in (left,right)) else .18)
                if hi-lo<minimum_overlap or abs(math.atan(left[2])-math.atan(right[2]))>math.radians(10):continue
                x=(lo+hi)/2
                yl,yr=left[2]*x+left[3],right[2]*x+right[3]
                width=abs(yl-yr);center=(yl+yr)/2
                if not .30<=width<=.50 or abs(center)>(1.0 if self.associate else .30):continue
                score=lo
                if tracking:
                    # After P identifies the bay, follow fresh measurements of
                    # that pair. A drifting command-estimated P must not veto
                    # its visible rails. Never switch to a neighbouring pair.
                    slope=(left[2]+right[2])/2.
                    previous_width=previous.get('width',self.bay_width)
                    matches=[]
                    for reference_pose in self.association_poses(previous):
                        theta=previous['theta']-reference_pose[2]
                        rx,ry=local(reference_pose,previous['center'])
                        heading=(math.atan(slope)-theta+math.pi)%(2*math.pi)-math.pi
                        lateral=-(x-rx)*math.sin(theta)+(center-ry)*math.cos(theta)
                        if (abs(heading)<=math.radians(10) and abs(lateral)<=.08
                                and abs(width-previous_width)<=.06):
                            matches.append(abs(lateral)+abs(width-previous_width))
                    if not matches:
                        self.debug['pair_tracking_rejected']+=1
                        continue
                    if self.bottom is not None and not stale_rear_bottom:
                        if lo>=max(local(self.pose,q)[0] for q in self.bottom):continue
                    score=min(matches)
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
                    if not all(self.side_reaches_anchor(side[1][0],ax) for side in (left,right)):continue
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
            if (self.bottom_reacquire_candidate or {}).get('source')=='paired':
                self.bottom_reacquire_candidate=None
            if not self.independent_bottom:self.observe_bottom_only(cross,stamp)
            if self.bottom_only:
                reference=self.left_reference
                self.observe_left_side(lines,stamp)
                preferred=self.single
                if preferred is None or (self.independent_bottom and distance(*preferred['side'])<.18):
                    # Test the opposite edge against the same bay reference,
                    # not against the short fragment just selected above.
                    self.left_reference=reference
                    self.observe_left_side(lines,stamp,side_sign=-1)
                    if preferred is not None and (self.single is None or distance(*self.single['side'])<.18):
                        self.single=self.left_reference=preferred
            if self.independent_bottom:self.observe_bottom_only(cross,stamp)
            return
        # Production ranks association with P; unassociated legacy mode ranks
        # forward proximity. Never substitute a nearer, unrelated rectangle.
        _,_,lo,hi,left,right,width=min(candidates,key=lambda p:(p[1],p[0]))
        slope=(left[2]+right[2])/2
        x=(lo+hi)/2;y=((left[2]+right[2])*x+left[3]+right[3])/2
        self.view=dict(theta=self.pose[2]+math.atan(slope),center=world(self.pose,(x,y)),width=width,
            observation_pose=tuple(self.pose),
            sides=[[world(self.pose,p) for p in side[:2]] for side in (left,right)])
        self.tracked_view=self.view;self.tracked_view_stamp=stamp
        if self.bottom_only:
            self.left_reference=dict(theta=self.view['theta'],center=self.view['center'],stamp=stamp,
                                     observation_pose=tuple(self.pose))
        if (self.independent_bottom and self.bottom is not None and
                getattr(self,'bottom_reference',None) is None):
            return  # Preserve a confirmed rear stripe; do not switch to a nearer mouth.
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
            selected=min(bottoms,key=lambda p:p[0])[1]
            if self.independent_bottom and self.bottom is not None and self.bottom_stamp!=stamp:
                # Strong current evidence (both target sides AND their closing
                # stripe) can repair a stale prediction after a visual gap.
                # Two distinct camera frames must agree; an isolated farther
                # stripe still cannot reset the remaining distance.
                a,b=[local(self.pose,p) for p in selected]
                k=(b[0]-a[0])/(b[1]-a[1])
                center=((a[0]+b[0])/2.,(a[1]+b[1])/2.)
                previous=self.bottom_reacquire_candidate
                consistent=(previous is not None and previous.get('source')=='paired' and
                    0<stamp-previous['stamp']<=self.cfg.get('ground_timeout',1.25) and
                    distance(center,previous['center'])<=.10 and
                    abs(math.atan(k)-previous['angle'])<=math.radians(10))
                self.bottom_reacquire_candidate=dict(source='paired',stamp=stamp,
                    center=center,angle=math.atan(k))
                if not consistent:
                    self.bottom_detection_reason='paired_bottom_reconfirming'
                    return
                self.remember_bottom(selected,stamp,'paired_bottom_reacquired')
                self.bottom_reacquire_candidate=None
                return
            source=('tracked_bottom' if self.bottom_stamp==stamp and
                    self.bottom_source=='tracked_bottom' else 'paired_far_end')
            self.remember_bottom(selected,stamp,source)
        elif self.independent_bottom:
            if (self.bottom_reacquire_candidate or {}).get('source')=='paired':
                self.bottom_reacquire_candidate=None
            # Pair selection comes first: a mouth stripe must never acquire a
            # bottom and then clip away the very sides that disprove it.
            self.observe_bottom_only(cross,stamp)

    def command(self,now,pose):
        self.advance_bottom_reference(pose)
        self.pose=tuple(pose);self.reason=None
        self.debug=dict(phase=self.phase,line_age_s=now-self.stamp,bottom_seen=self.bottom is not None,
            bottom_detection_reason=self.bottom_detection_reason,
            bottom_cross_candidates=self.bottom_cross_candidates,
            bottom_rejection_counts=dict(self.bottom_rejection_counts),
            bottom_corridor_source=self.bottom_corridor_source,
            bottom_depth_jump_rejected=self.bottom_depth_jump_rejected,
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
        reference = getattr(self, 'bottom_reference', None)
        loss_active=(reference is None and self.bottom_stop_enabled and
                     self.bottom_only and self.bottom_loss_origin is not None)
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
        # Explicit S retains geometry between images. Rear command-model
        # steering uses the observation pose; other forward modes require freshness.
        fresh = 0<=now-self.stamp<=self.cfg.get('ground_timeout',1.25)
        if now<self.stamp or (not self.bottom_only and not fresh):
            self.reason='parking_front_stale';return (0,0.)
        speed=self.cfg.get('parking_entry_speed_raw',12)
        if self.bottom is not None:
            # Reuse the front-bumper calculation, including the nearer corner
            # at an oblique line. Detection alone says nothing about arrival.
            remaining=self.remaining()
            clearance=self.cfg.get('parking_bottom_clearance_m',.04)
            measured=self.measured_remaining(self.bottom_pose or self.pose)
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
            if reference is not None:
                # Every accepted image resets the distance. Between images,
                # signed motion consumes that distance (away motion adds it).
                # A missed image never restarts a fixed post-loss allowance.
                near_exit=(measured is not None and measured<=reference['remaining_m']+1e-9)
                reconfirming=((self.bottom_reacquire_candidate or {}).get('source')=='paired' and
                              self.bottom_reacquire_candidate['stamp']==self.stamp)
                arrived = (not reconfirming and remaining is not None and remaining<=1e-9 and
                           ((bottom_fresh and measured is not None and measured<=0) or near_exit))
                self.debug.update(bottom_reference_depth_m=reference['depth_m'],
                    bottom_reference_remaining_m=reference['remaining_m'],
                    bottom_reference_net_travel_m=self.bottom_reference_travel,
                    bottom_reference_measured_remaining_m=measured,
                    bottom_extrapolation_can_finish=near_exit and not reconfirming,
                    bottom_waiting_visual_confirmation=not arrived and remaining is not None and remaining<=1e-9,
                    visual_motion_scale=1.,visual_motion_samples=0,
                    bottom_arrival_confirmed=arrived,
                    bottom_loss_active=self.bottom_loss_origin is not None,
                    bottom_loss_remaining_m=max(0.,remaining) if remaining is not None else None,
                    bottom_loss_travel_m=None,
                    distance_source='live_bottom_reference' if bottom_fresh else 'bottom_reference_plus_pose',
                    target_clearance_m=None,bumper_clearance_m=None,
                    measured_bumper_clearance_m=None)
            if remaining is None and not loss_active and self.bottom_stop_enabled:
                self.reason='parking_bottom_geometry_invalid';return (0,0.)
            if arrived and self.bottom_stop_enabled:
                self.status='FINISHED';self.phase='FINISHED';self.reason='parking_at_bottom_clearance'
                if reference is not None:
                    self.reason=self.finish_reason='parking_rear_visual_reference_reached'
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
                self.debug['side_local']=[local(self.steering_pose(self.single),p) for p in self.single['side']]
                return (speed,self.limited_steer(self.single))
            self.reason='parking_missing_white_straight'
            return (speed,0.)
        self.phase='CENTER';self.debug.update(phase=self.phase,width_m=self.view['width'])
        self.debug['steering_reference']='both_sides'
        self.debug['pair_local']=[[local(self.steering_pose(self.view),p) for p in side] for side in self.view['sides']]
        self.reason='parking_center_between_sides'
        return (speed,self.limited_steer(self.view))

    def limited_steer(self,view,direction=1):
        # Only the calibrated chassis range remains; no parking-specific cap.
        pose=self.steering_pose(view)
        heading,lateral=self.errors(view['theta'],view['center'],pose=pose)
        steer=self.steer(view['theta'],view['center'],direction=direction,pose=pose)
        self.debug.update(centerline_local=local(pose,view['center']),
            steering_pose_source='rear_image' if getattr(self,'camera_steering_feedback',False) else 'current_pose',
            centerline_heading_error_deg=math.degrees(heading),
            centerline_lateral_error_m=lateral,steering_command_rad=steer)
        return steer
