"""Approximate the P board's ground position using its known physical width.

The elevated board center is a camera ray, not a ground homography point.
The ground calibration supplies the camera pose; box width supplies depth.
"""
import cv2
import numpy as np


class ParkingSignProjector(object):
    def __init__(self, cfg):
        c=cfg['front_camera']
        self.width,self.height=c['width'],c['height']
        self.k=np.asarray(c['K'],dtype=float).reshape(3,3)
        self.d=np.asarray(c['D'],dtype=float)
        self.sign_width=cfg.get('sign_width_m',.195)
        size=(self.width,self.height)
        nk=cv2.getOptimalNewCameraMatrix(self.k,self.d,size,1.,size)[0]
        metric=np.array([[0,-c['pixels_per_m'],c['origin_u']],
                         [-c['pixels_per_m'],0,c['origin_v']],[0,0,1.]])
        a=np.dot(np.linalg.inv(nk),np.dot(np.linalg.inv(np.array(c['H']).reshape(3,3)),metric))
        scale=(np.linalg.norm(a[:,0])+np.linalg.norm(a[:,1]))/2.
        # Choose positive depth in FRONT of the vehicle, not at the axle
        # (which can be behind the camera's optical plane).
        if a[2,0]+a[2,2]<0:scale=-scale
        r=np.column_stack((a[:,0]/scale,a[:,1]/scale,np.cross(a[:,0],a[:,1])/scale**2))
        u,s,v=np.linalg.svd(r)
        self.rotation=np.dot(u,v);self.translation=a[:,2]/scale

    def position(self, bounds):
        if bounds is None or len(bounds)!=4:return None
        x,y,w,h=[float(v) for v in bounds]
        if not np.isfinite([x,y,w,h]).all():return None
        # A clipped board has an artificially narrow width and false depth.
        if w<16 or h<16 or x<=1 or y<=1 or x+w>=self.width-1 or y+h>=self.height-1:return None
        points=np.array([[[x,y+h/2.]],[[x+w,y+h/2.]],[[x+w/2.,y+h/2.]]])
        uv=cv2.undistortPoints(points,self.k,self.d).reshape(3,2)
        span=uv[1,0]-uv[0,0]
        if span<=0:return None
        depth=self.sign_width/span
        camera=depth*np.array([uv[2,0],uv[2,1],1.])
        p=np.dot(self.rotation.T,camera-self.translation)
        if not np.isfinite(p).all() or not .1<p[0]<3.5:return None
        return (float(p[0]),float(p[1]))
