"""Exclude an elevated P board using its raw-image box, never ground depth."""
import cv2
import numpy as np
import yaml


class RearSignMask(object):
    def __init__(self, path):
        with open(path) as stream:
            c=yaml.safe_load(stream)
        size=(c['image_width'],c['image_height'])
        maps=cv2.initUndistortRectifyMap(np.array(c['camera_matrix']),
            np.array(c['distortion_coefficients']),None,
            np.array(c['rectified_camera_matrix']),size,cv2.CV_32FC1)
        h=np.array(c['homography_rectified_to_bev'])
        out=(c['bev_width'],c['bev_height'])
        # Inverse sampling stays well-defined even when the raw box crosses
        # the horizon; transforming its four corners would wrap across infinity.
        self.x,self.y=[cv2.warpPerspective(m,h,out,borderValue=-10000) for m in maps]

    def apply(self, white, bounds):
        if not isinstance(bounds,(list,tuple)) or len(bounds)!=4:
            return white
        try:x,y,w,h=[float(v) for v in bounds]
        except (ValueError,TypeError):return white
        if not np.isfinite([x,y,w,h]).all() or min(w,h)<=0:return white
        if white.shape!=self.x.shape:return white
        inside=((self.x>=x-3)&(self.x<=x+w+3)&(self.y>=y-3)&(self.y<=y+h+3))
        result=white.copy()
        result[inside]=0
        return result
