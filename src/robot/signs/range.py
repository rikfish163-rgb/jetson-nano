"""Approximate optical depth of a known-width sign, never odometry distance."""
import math
import cv2
import numpy as np

class SignRange(object):
    def __init__(self, calibration, width_m, maximum_m):
        self.width_m,self.maximum_m=width_m,maximum_m
        self.k=np.asarray(calibration['camera_matrix']['data'],dtype=float).reshape(3,3)
        self.d=np.asarray(calibration['distortion_coefficients']['data'],dtype=float)
        self.width=float(calibration['image_width'])
        self.height=float(calibration['image_height'])
        if (not np.isfinite(self.k).all() or not np.isfinite(self.d).all() or
                self.k[0,0]<=0 or self.k[1,1]<=0 or self.width<=0 or self.height<=0):
            raise ValueError('invalid sign range calibration')

    def evaluate(self, bounds, shape):
        if bounds is None:return False,None,'no_candidate'
        x,y,w,h=bounds
        height,width=shape[:2]
        if w<=0 or h<=0 or x<=0 or y<=0 or x+w>=width or y+h>=height:
            return False,None,'clipped_candidate'
        k=self.k.copy();k[0,:]*=width/self.width;k[1,:]*=height/self.height
        points=np.asarray([[[x,y+h/2.0]],[[x+w,y+h/2.0]]],dtype=np.float64)
        points=cv2.undistortPoints(points,k,self.d).reshape(2,2)
        span=abs(points[1,0]-points[0,0])
        if not np.isfinite(span) or span<=0:return False,None,'invalid_geometry'
        depth=float(self.width_m/span)
        if not math.isinf(depth) and not math.isnan(depth) and depth>0:
            return depth<=self.maximum_m,depth,'in_range' if depth<=self.maximum_m else 'too_far'
        return False,None,'invalid_geometry'
