"""Approximate the P board's ground position using its known physical width.

The elevated board center is a camera ray, not a ground homography point.
The ground calibration supplies the camera pose; box width supplies depth.
"""
import cv2
import numpy as np


class ParkingSignProjector(object):
    def __init__(self, cfg, rear_calibration=None):
        self.direction=1 if rear_calibration is None else -1
        c=cfg['front_camera'] if rear_calibration is None else rear_calibration
        self.width,self.height=(c['width'],c['height']) if self.direction==1 else (c['image_width'],c['image_height'])
        self.k=np.asarray(c['K'] if self.direction==1 else c['camera_matrix'],dtype=float).reshape(3,3)
        self.d=np.asarray(c['D'] if self.direction==1 else c['distortion_coefficients'],dtype=float)
        self.sign_width=cfg.get('sign_width_m',.195)
        size=(self.width,self.height)
        if self.direction==1:
            nk=cv2.getOptimalNewCameraMatrix(self.k,self.d,size,1.,size)[0]
            metric=np.array([[0,-c['pixels_per_m'],c['origin_u']],
                             [-c['pixels_per_m'],0,c['origin_v']],[0,0,1.]])
            homography=c['H']
        else:
            # Invert exactly GroundDetector.metric(rear=True), including the
            # deployed BEV mirror and rear-axle offset. Output is body x/y.
            r=cfg['rear_bev'];ppm=r['pixels_per_m']
            nk=np.asarray(c['rectified_camera_matrix'],dtype=float)
            metric=np.array([[0,ppm/r['y_sign'],r['origin_u']],
                             [ppm,0,r['origin_v']-r['axle_x']*ppm],[0,0,1.]])
            homography=c['homography_rectified_to_bev']
        a=np.dot(np.linalg.inv(nk),np.dot(np.linalg.inv(np.array(homography).reshape(3,3)),metric))
        scale=(np.linalg.norm(a[:,0])+np.linalg.norm(a[:,1]))/2.
        # Choose positive optical depth on the viewed side of the vehicle,
        # not at the axle (which can lie behind the camera's optical plane).
        if self.direction*a[2,0]+a[2,2]<0:scale=-scale
        r=np.column_stack((a[:,0]/scale,a[:,1]/scale,np.cross(a[:,0],a[:,1])/scale**2))
        u,s,v=np.linalg.svd(r)
        self.rotation=np.dot(u,v);self.translation=a[:,2]/scale

    def position(self, bounds):
        if bounds is None or len(bounds)!=4:return None
        x,y,w,h=[float(v) for v in bounds]
        if not np.isfinite([x,y,w,h]).all():return None
        # A clipped board has an artificially narrow width and false depth.
        if w<16 or h<=0 or (self.direction==1 and h<16) or x<=1 or y<=1 or x+w>=self.width-1 or y+h>=self.height-1:return None
        points=np.array([[[x,y+h/2.]],[[x+w,y+h/2.]],[[x+w/2.,y+h/2.]]])
        uv=cv2.undistortPoints(points,self.k,self.d).reshape(3,2)
        span=uv[1,0]-uv[0,0]
        if span<=0:return None
        depth=self.sign_width/span
        camera=depth*np.array([uv[2,0],uv[2,1],1.])
        p=np.dot(self.rotation.T,camera-self.translation)
        if not np.isfinite(p).all() or not .1<self.direction*p[0]<3.5:return None
        return (float(p[0]),float(p[1]))
