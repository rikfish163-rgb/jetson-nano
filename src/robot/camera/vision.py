"""Metric blue tape and white U/rectangle bay detection, no ROS imports."""
from __future__ import division
import math
import cv2
import numpy as np
from robot.parking.detection import detect_bays
from robot.camera.landmarks import image_segments


_COVERAGE_STEPS = np.arange(31,dtype=np.float64)[:,None]


class GroundDetector(object):
    def __init__(self, cfg):
        self.cfg = cfg
        c = cfg['front_camera']
        self.c = c
        K = np.array(c['K'],dtype=np.float64).reshape(3,3)
        D = np.array(c['D'],dtype=np.float64)
        size = (c['width'],c['height'])
        newK,_ = cv2.getOptimalNewCameraMatrix(K,D,size,1.0,size)
        self.maps = cv2.initUndistortRectifyMap(K,D,None,newK,size,cv2.CV_16SC2)
        self.H = np.array(c['H'],dtype=np.float64).reshape(3,3)
        self.parking_valid = None

    def bev(self, frame, parking=False):
        if frame.shape[:2] != (self.c['height'],self.c['width']):
            raise ValueError('camera size differs from calibration')
        rectified = cv2.remap(frame,self.maps[0],self.maps[1],cv2.INTER_LINEAR)
        if parking:
            # Same calibrated metric transform, wider canvas for off-road bays.
            # Translation changes only the canvas origin, never the calibration.
            offset = 360
            shift = np.array([[1,0,offset],[0,1,0],[0,0,1]],dtype=np.float64)
            if self.parking_valid is None:
                valid = np.full(frame.shape[:2],255,dtype=np.uint8)
                valid = cv2.remap(valid,self.maps[0],self.maps[1],cv2.INTER_LINEAR)
                valid = cv2.warpPerspective(valid,np.dot(shift,self.H),
                                            (self.c['bev_width']+2*offset,
                                             int(self.c['origin_v'])))
                self.parking_valid = (valid >= 254).astype(np.uint8)
            return cv2.warpPerspective(rectified,np.dot(shift,self.H),
                                       (self.c['bev_width']+2*offset, int(self.c['origin_v'])))
        return cv2.warpPerspective(rectified,self.H,(self.c['bev_width'],self.c['bev_height']))

    def detect(self, bev, rear=False, include_slots=True):
        hsv = cv2.cvtColor(bev,cv2.COLOR_BGR2HSV)
        b,w = self.cfg['blue'],self.cfg['white']
        blue = cv2.inRange(hsv,np.array([b['h_min'],b['s_min'],b['v_min']]),np.array([b['h_max'],255,255]))
        white = cv2.inRange(hsv,np.array([0,0,w['v_min']]),np.array([179,w['s_max'],255]))
        ppm = self.cfg['rear_bev']['pixels_per_m'] if rear else self.c['pixels_per_m']
        markers = []
        blue_lines = []
        for contour in cv2.findContours(blue,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]:
            rect = cv2.minAreaRect(contour)
            sides = sorted(rect[1])
            thick,length = sides[0]/ppm,sides[1]/ppm
            if not b['thickness_min'] <= thick <= b['thickness_max'] or not b['length_min'] <= length <= b['length_max']:
                continue
            box = cv2.boxPoints(rect)
            edges = [(np.linalg.norm(box[(i+1)%4]-box[i]),box[(i+1)%4]-box[i]) for i in range(4)]
            delta = max(edges,key=lambda e:e[0])[1]
            angle = abs(math.atan2(delta[1],delta[0]))
            angle = min(angle,abs(math.pi-angle))
            x,y = self.metric(rect[0][0],rect[0][1],rear)
            a = self.metric(rect[0][0]-delta[0]/2,rect[0][1]-delta[1]/2,rear)
            z = self.metric(rect[0][0]+delta[0]/2,rect[0][1]+delta[1]/2,rear)
            dx,dy = z[0]-a[0],z[1]-a[1]
            if dy<0: dx,dy=-dx,-dy
            if length>=b['long_min']:
                blue_lines.append(dict(x=x,y=y,yaw=math.atan2(-dx,dy),length=length))
            # Front junction detection intentionally accepts oblique tape.
            # Keep the rear-camera filter and measured yaw for other consumers.
            if rear and angle > b['angle_tolerance']:
                continue
            x,y = self.metric(rect[0][0],rect[0][1],rear)
            markers.append(dict(x=x,y=y,length=length,kind='junction' if length >= b['long_min'] else 'tick'))
        if not rear:
            # Connected corners and shadow fragments have misleading contour
            # boxes. Recover straight segments while retaining paint-width
            # and length gates, including the short scoring-tick rejection.
            segments=image_segments(blue,lambda pts:np.array(
                [self.metric(u,v) for u,v in pts]),ppm,b['length_max'])
            for a,z in segments:
                delta=z-a
                length=float(np.linalg.norm(delta))
                if length < b['long_min']:continue
                normal=np.array([-delta[1],delta[0]])/length
                centers=a+np.linspace(.05,.95,41)[:,None]*delta
                offsets=np.arange(-int(ppm*.08),int(ppm*.08)+1)/ppm
                samples=centers[:,None,:]+offsets[None,:,None]*normal
                u=np.rint(self.c['origin_u']-samples[:,:,1]*ppm).astype(int)
                v=np.rint(self.c['origin_v']-samples[:,:,0]*ppm).astype(int)
                valid=(u>=0)&(u<blue.shape[1])&(v>=0)&(v<blue.shape[0])
                paint=np.zeros(u.shape,dtype=bool)
                paint[valid]=blue[v[valid],u[valid]]>0
                widths=np.count_nonzero(paint,axis=1)/ppm
                if (np.mean(widths>=b['thickness_min']) < .50 or
                        not b['thickness_min'] <= np.median(widths) <= b['thickness_max']):
                    continue
                dx,dy=delta
                if dy<0:dx,dy=-dx,-dy
                center=(a+z)/2
                blue_lines.append(dict(x=float(center[0]),y=float(center[1]),
                                       yaw=math.atan2(-dx,dy),length=length))
            merged=[]
            for line in sorted(blue_lines,key=lambda row:row['length'],reverse=True):
                duplicate=False
                for old in merged:
                    angle=(line['yaw']-old['yaw']+math.pi/2)%math.pi-math.pi/2
                    dx,dy=line['x']-old['x'],line['y']-old['y']
                    normal_distance=abs(dx*math.cos(old['yaw'])+dy*math.sin(old['yaw']))
                    along=abs(-dx*math.sin(old['yaw'])+dy*math.cos(old['yaw']))
                    if (abs(angle)<math.radians(5) and normal_distance<.035 and
                            along <= (line['length']+old['length'])/2+.04):
                        duplicate=True;break
                if not duplicate:merged.append(line)
            blue_lines=merged
            markers=[m for m in markers if m['kind']=='tick']+[
                dict(x=m['x'],y=m['y'],length=m['length'],kind='junction') for m in merged]
        slots = self.detect_slots(white,rear) if include_slots else []
        return dict(markers=markers,slots=slots,blue_lines=blue_lines),blue,white

    def metric(self,u,v,rear=False):
        if rear:
            r = self.cfg['rear_bev']
            return (r['axle_x']-(r['origin_v']-v)/r['pixels_per_m'],
                    r['y_sign']*(u-r['origin_u'])/r['pixels_per_m'])
        c = self.c
        return ((c['origin_v']-v)/c['pixels_per_m'],(c['origin_u']-u)/c['pixels_per_m'])

    def detect_slots(self,white,rear=False,u_offset=0):
        ppm = self.cfg['rear_bev']['pixels_per_m'] if rear else self.c['pixels_per_m']
        if not rear and self.cfg.get('parking_mode')=='forward_white':
            valid = self.parking_valid
            if valid is not None and valid.shape != white.shape:
                valid = None
            slots,self.slot_diagnostic=detect_bays(white,lambda u,v:self.metric(u-u_offset,v),ppm,self.cfg,valid)
            return slots
        return self._slots(white,lambda u,v:self.metric(u-u_offset,v,rear),ppm)

    def parking_lines(self, white, u_offset=0):
        """Finite front-camera segments for tracking a selected bay at ground_hz."""
        ppm = self.c['pixels_per_m']
        rows = cv2.HoughLinesP(white,1,np.pi/180,25,
                              minLineLength=int(ppm*.12),maxLineGap=int(ppm*.025))
        if rows is None:
            return []
        return [[self.metric(a-u_offset,b),self.metric(c-u_offset,d)]
                for a,b,c,d in (row[0] for row in rows[:80])]

    def _slots(self, mask, metric, ppm):
        """Pair parallel long white edges and verify a closing edge.

        A missing mouth edge is expected; missing side/back edges do NOT invent
        a bay. Initial identity is configured by same-frame spatial rank.
        """
        cfg = self.cfg['white']
        lines = cv2.HoughLinesP(mask,1,np.pi/180,35,minLineLength=int(ppm*0.25),maxLineGap=int(ppm*0.035))
        if lines is None:
            return []
        segments = []
        for row in lines[:100]:
            a = np.array(row[0][:2],dtype=float); b = np.array(row[0][2:],dtype=float)
            size = np.linalg.norm(b-a)
            if size:
                segments.append((a,b,(b-a)/size,size))
        dilated = cv2.dilate(mask,np.ones((5,5),np.uint8))
        def coverage(a,b):
            # Keep the old truncation-to-int sampling points, but let NumPy
            # gather all 31 pixels at once.  The Python generator dominated
            # Nano CPU time when many Hough segment pairs survived geometry.
            points = (a+(b-a)*_COVERAGE_STEPS/30.0).astype(int)
            good = (points[:,0] >= 0)&(points[:,0] < mask.shape[1])&(points[:,1] >= 0)&(points[:,1] < mask.shape[0])
            points = points[good]
            return float(np.count_nonzero(dilated[points[:,1],points[:,0]]))/31.0
        found = []
        # One model per bay kind; no dependence on model label order for geometry.
        profiles = [self.cfg['slots']['P1'],self.cfg['slots']['P4']]
        for i,(a,b,u,la) in enumerate(segments):
            normal = np.array([-u[1],u[0]])
            for c,d,v,lb in segments[i+1:]:
                if abs(float(np.dot(u,v))) < math.cos(0.12):
                    continue
                sep = abs(float(np.dot(c-a,normal)))/ppm
                for profile in profiles:
                    length,width = profile['length']*ppm,profile['width']*ppm
                    tol = cfg['dimension_tolerance']*ppm
                    if abs(sep*ppm-width) > tol or min(la,lb) < length-tol:
                        continue
                    aproj = sorted((float(np.dot(a,u)),float(np.dot(b,u))))
                    bproj = sorted((float(np.dot(c,u)),float(np.dot(d,u))))
                    low,high = max(aproj[0],bproj[0]),min(aproj[1],bproj[1])
                    if high-low < length-tol or high-low > length+2*tol:
                        continue
                    mid = (low+high)/2
                    na,nc = float(np.dot(a,normal)),float(np.dot(c,normal))
                    ends = [u*(mid+s*length/2)+normal*n for s in (-1,1) for n in (na,nc)]
                    # Hough often emits several nearly identical segments for
                    # the same physical edge.  The previous implementation
                    # paid for four 31-point coverage scans before discarding
                    # such duplicates; reject them as soon as their measured
                    # center is known.  ``found`` still contains only fully
                    # validated bays, so output order and selection are
                    # unchanged.
                    center = u*mid+normal*(na+nc)/2
                    x,y = metric(*center)
                    if any(math.hypot(x-s['x'],y-s['y']) < 0.10 and s['kind'] == profile['kind'] for s in found):
                        continue
                    sides_ok = min(coverage(ends[0],ends[2]),coverage(ends[1],ends[3])) >= cfg['edge_coverage']
                    back_ok = max(coverage(ends[0],ends[1]),coverage(ends[2],ends[3])) >= cfg['edge_coverage']
                    if not sides_ok or not back_ok:
                        continue
                    p2 = metric(*(center+u*ppm))
                    yaw = math.atan2(p2[1]-y,p2[0]-x)
                    found.append(dict(x=x,y=y,yaw=yaw,kind=profile['kind'],length=profile['length'],width=profile['width']))
        return found
