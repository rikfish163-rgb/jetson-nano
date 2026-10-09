"""Identify the right-side terminal bay line in each image. No optical flow."""
from __future__ import division
import cv2
import numpy as np
from robot.camera.vision import GroundDetector
from robot.camera.landmarks import image_segments
from robot.common.config import read_mapping


def load_reference_config(base_path, projection_path=None):
    cfg=read_mapping(base_path)
    if projection_path:
        cfg['front_camera']['H']=read_mapping(projection_path)['front_camera']['H']
    return cfg


def paint_mask(bev, white_cfg, ppm):
    hsv=cv2.cvtColor(cv2.GaussianBlur(bev,(3,3),0),cv2.COLOR_BGR2HSV)
    white=cv2.inRange(hsv,(0,0,white_cfg['v_min']),(179,white_cfg['s_max'],255))
    # Bright reflected floor is not a continuing painted rail. Require
    # contrast above its local background, at a scale wider than the tape.
    kernel=max(3,int(round(ppm*.07))|1)
    contrast=cv2.morphologyEx(hsv[:,:,2],cv2.MORPH_TOPHAT,np.ones((kernel,kernel),np.uint8))
    return cv2.bitwise_and(white,cv2.inRange(contrast,25,255))


def terminal_lines(segments, white, valid, metric, ppm, diagnostic=None):
    """Require a 36 cm end, an outer rail behind it and visible empty ground ahead.

    A shared divider or a rail clipped by the image cannot prove P1's front end.
    Numbering assumes this is the configured P3->P2->P1 right-side bay row.
    """
    origin_x,origin_y=metric(0,0)
    def support(points, mask):
        points=np.asarray(points)
        u=np.rint((origin_y-points[:,1])*ppm).astype(int)
        v=np.rint((origin_x-points[:,0])*ppm).astype(int)
        inside=(u>=0)&(u<mask.shape[1])&(v>=0)&(v<mask.shape[0])
        values=np.zeros(len(points),bool)
        values[inside]=mask[v[inside],u[inside]]>0
        return float(values.mean())
    paint=cv2.dilate(white,np.ones((5,5),np.uint8))
    def centreline(a,b):
        # Hough may choose opposite edges of thick tape. Fit the actual paint
        # inside the segment, excluding its corner junctions, before extending
        # the line from the right-hand bay to the vehicle's front-axle centre.
        length=np.linalg.norm(b-a);axis=(b-a)/length
        normal=np.array([-axis[1],axis[0]])
        pixels=np.column_stack(((origin_y-np.array([a[1],b[1]]))*ppm,
                                (origin_x-np.array([a[0],b[0]]))*ppm))
        lo=np.maximum(0,np.floor(pixels.min(axis=0)-.03*ppm).astype(int))
        hi=np.minimum([white.shape[1],white.shape[0]],
                      np.ceil(pixels.max(axis=0)+.03*ppm).astype(int)+1)
        if np.any(hi<=lo):return a,b
        rows,cols=np.nonzero(white[lo[1]:hi[1],lo[0]:hi[0]])
        points=np.column_stack((origin_x-(rows+lo[1])/ppm,
                                origin_y-(cols+lo[0])/ppm))
        along=np.dot(points-a,axis)
        keep=(along>=.04)&(along<=length-.04)&(abs(np.dot(points-a,normal))<=.025)
        if keep.sum()<12 or np.ptp(along[keep])<.18:return a,b
        vx,vy,x,y=cv2.fitLine(points[keep].astype(np.float32),cv2.DIST_L1,0,.001,.001).reshape(4)
        if abs(vy)<1e-6 or abs(vx)>.15*abs(vy):return a,b
        return (np.array([x+(a[1]-y)*vx/vy,a[1]]),
                np.array([x+(b[1]-y)*vx/vy,b[1]]))
    candidates=[]
    for segment in segments:
        a,b=sorted((np.asarray(p,float) for p in segment),key=lambda p:p[1],reverse=True)
        dx,dy=b-a
        length=np.linalg.norm(b-a)
        if abs(dx)>.15*abs(dy):continue
        if length>0:
            a,b=centreline(a,b);dx,dy=b-a;length=np.linalg.norm(b-a)
        if abs(dy)>0:
            # Hough can merge or truncate the end. Measure its full width
            # at the separate road-side edge, never fit the desired width.
            matches=[]
            for p,q in segments:
                p,q=sorted((np.asarray(p,float),np.asarray(q,float)),key=lambda point:point[0])
                vx,vy=q-p
                if vx<.2 or abs(vy)>.22*vx:continue
                slope=vy/vx
                y=(p[1]+slope*(a[0]-p[0])-slope*dx*a[1]/dy)/(1-slope*dx/dy)
                x=a[0]+(y-a[1])*dx/dy
                if abs(y-a[1])<=.15 and p[0]-.03<=x<=q[0]-.15:
                    matches.append((abs(y-a[1]),np.array([x,y])))
            if not matches:continue
            a=min(matches,key=lambda item:item[0])[1];dx,dy=b-a;length=np.linalg.norm(b-a)
        if (not .20<=length<=.80 or abs(dx)>.15*abs(dy) or
                not -.70<=a[1]<=-.10 or b[1]<-1.): continue
        full_end=[a+(b-a)*t for t in np.linspace(0,1,24)]
        if support(full_end,valid)<.95 or support(full_end,paint)<.8:continue
        # Check actual support behind the outer (deeper) corner.
        rail=None
        for start,end in segments:
            start,end=sorted((np.asarray(start,float),np.asarray(end,float)),key=lambda p:p[0])
            vx,vy=end-start
            if vx<.20 or abs(vy)>.22*vx: continue
            y_at_end=start[1]+(b[0]-start[0])*vy/vx
            if (abs(y_at_end-b[1])<.04 and start[0]<=b[0]-.2 and
                    b[0]-.06<=end[0]<=b[0]+.15):
                rail=(y_at_end,vy/vx);break
        if rail is None: continue
        outer_y,slope=rail
        behind=[(b[0]-x,outer_y-x*slope) for x in np.linspace(.04,.20,14)]
        if support(behind,valid)<.9 or support(behind,paint)<.8: continue
        # The road-side endpoint can join the road curve/dashes. Only the
        # OUTER bay rail must end; its absence needs observed ground support.
        ahead=[(b[0]+x,outer_y+x*slope+y)
               for x in np.linspace(.08,.18,6) for y in (-.015,0,.015)]
        if support(ahead,valid)<.95 or support(ahead,paint)>.15: continue
        line=[tuple(a),tuple(b)]
        if not .28<=length<=.44:
            if diagnostic is not None:
                diagnostic.setdefault('width_mismatches',[]).append(dict(line=line,width_m=float(length)))
            continue
        if not any(abs(old[0][0]-a[0])<.04 and abs(old[0][1]-a[1])<.04 for old in candidates):
            candidates.append(line)
    return candidates


class ReferenceVision(object):
    def __init__(self,cfg):
        self.cfg=cfg
        # Half-resolution metric canvas: scale H and every metric canvas
        # parameter together. Input calibration / physical scale is unchanged.
        camera=dict(cfg['front_camera'])
        camera['H']=np.dot(np.diag([.5,.5,1.]),np.asarray(camera['H']).reshape(3,3)).reshape(-1).tolist()
        for key in ('bev_width','bev_height','origin_u','origin_v','pixels_per_m'):
            camera[key]=camera[key]*.5
        for key in ('bev_width','bev_height'): camera[key]=int(camera[key])
        # Extend the line-search canvas to 3 m without changing projection scale.
        shift_v=3.*camera['pixels_per_m']-camera['origin_v']
        shift=np.array([[1,0,0],[0,1,shift_v],[0,0,1]],float)
        camera['H']=np.dot(shift,np.asarray(camera['H']).reshape(3,3)).reshape(-1).tolist()
        camera['origin_v']+=shift_v
        detector_cfg=dict(cfg,front_camera=camera)
        self.detector=GroundDetector(detector_cfg)
        camera=dict(camera);camera['origin_u']+=360
        self.camera=camera
        self.last_stamp=-1.

    def observe(self,frame,stamp):
        if stamp<=self.last_stamp: return None
        self.last_stamp=stamp
        result=dict(source='p1_reference_camera',stamp=stamp,frame='base_link',
                    wheelbase_m=float(self.cfg['wheelbase']),p1_lines=[])
        bev=self.detector.bev(frame,parking=True)
        # Suppress MJPEG floor texture before looking for finite paint segments.
        white=paint_mask(bev,self.cfg['white'],self.camera['pixels_per_m'])
        ppm=self.camera['pixels_per_m']
        left=int(self.camera['origin_u']+.08*ppm)
        right=int(self.camera['origin_u']+.95*ppm)
        row_end=int(self.camera['origin_v']-.5*ppm)
        def points_metric(points):
            return np.column_stack(((self.camera['origin_v']-points[:,1])/ppm,
                                     (self.camera['origin_u']-left-points[:,0])/ppm))
        segments=image_segments(white[:row_end,left:right],points_metric,ppm,2.5)
        metric=lambda u,v:self.detector.metric(u-360,v)
        diagnostic={}
        lines=terminal_lines(segments,white,self.detector.parking_valid,metric,ppm,diagnostic)
        rejected=diagnostic.get('width_mismatches',[])
        result.update(p1_lines=lines,rejected_lines=rejected,
                      reason='p1_terminal_seen' if lines else
                      ('p1_terminal_width_mismatch' if rejected else 'searching_p1_terminal'))
        for line,color in ([(line,(0,0,255)) for line in lines]+
                           [(row['line'],(0,255,255)) for row in rejected]):
            c=self.camera
            pts=[(int(c['origin_u']-y*c['pixels_per_m']),int(c['origin_v']-x*c['pixels_per_m']))
                 for x,y in line]
            cv2.line(bev,pts[0],pts[1],color,4)
        return result,bev
