"""Full-frame board candidates. Python 2/3, no ROS or vehicle commands.

Area ranks valid candidates; it is not a calibrated distance measurement.
"""
import cv2
import numpy as np


def board_candidates(frame):
    if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
        return []
    height,width=frame.shape[:2]
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    ranges=[('red',(0,60,40),(12,255,255)),
            ('red',(165,60,40),(179,255,255)),
            ('green',(30,60,40),(95,255,255)),
            ('blue',(95,60,30),(145,255,255))]
    masks={}
    for color,lower,upper in ranges:
        mask=cv2.inRange(hsv,np.array(lower),np.array(upper))
        masks[color]=cv2.bitwise_or(masks[color],mask) if color in masks else mask
    result=[]
    for color,mask in masks.items():
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
        contours=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        for contour in contours:
            area=cv2.contourArea(contour)
            x,y,w,h=cv2.boundingRect(contour)
            aspect=float(w)/max(h,1)
            valid_aspect=(.5 <= aspect <= 1.45) if color=='blue' else (.25 <= aspect <= 2.0)
            if area < height*width*.002 or not valid_aspect:
                continue
            if y+h*.5 >= height*.72:
                continue
            if color=='blue':
                # A blue base or floor patch is not enough: require an interior
                # light glyph. Outer borders and white regions outside the face
                # do not count. This is a candidate filter, not a classifier.
                filled=np.zeros((h,w),np.uint8)
                shifted=contour-np.array([[[x,y]]],dtype=contour.dtype)
                cv2.drawContours(filled,[cv2.convexHull(shifted)],-1,255,-1)
                margin=max(1,int(min(w,h)*.06))
                inner=cv2.erode(filled,np.ones((margin*2+1,margin*2+1),np.uint8))
                light=cv2.inRange(hsv[y:y+h,x:x+w],np.array([0,0,125]),np.array([179,100,255]))
                glyph=cv2.countNonZero(cv2.bitwise_and(inner,light))
                if glyph < max(8,cv2.countNonZero(inner)*.015):
                    continue
            px=max(2,int(w*.10));py=max(2,int(h*.10))
            crop=frame[max(0,y-py):min(height,y+h+py),max(0,x-px):min(width,x+w+px)].copy()
            result.append(dict(crop=crop,bounds=(x,y,w,h),color=color,area=area))
    return sorted(result,key=lambda c:c['area'],reverse=True)


def extract_board_roi(frame,return_bounds=False):
    candidates=board_candidates(frame)
    if not candidates:
        return None
    item=candidates[0]
    return (item['crop'],item['bounds']) if return_bounds else item['crop']
