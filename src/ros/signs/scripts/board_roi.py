"""Full-frame board candidates. Python 2/3, no ROS or vehicle commands.

Search every image position. Area ranks valid candidates; it is not distance.
"""
import cv2
import numpy as np


def board_candidates(frame, diagnostics=None):
    if diagnostics is not None:
        diagnostics.update(rejected_counts={}, regions=[], candidate_count=0)
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
            # A pixel floor rejects speckles without excluding small boards
            # merely because the camera image is large. Do not mask the bottom:
            # a nearby sign can occupy any part of the image.
            reason = ('too_small' if min(w,h)<12 or area<72 else
                      'aspect_ratio' if not valid_aspect else None)
            if reason:
                _record(diagnostics, color, (x,y,w,h), area, reason)
                continue
            if color=='blue':
                # Tiny tapered fragments are ambiguous even with white paint.
                # Do not impose this on large/partly occluded sign faces.
                if max(w,h) <= 32 and area/float(w*h) < .55:
                    _record(diagnostics, color, (x,y,w,h), area, 'sparse_shape')
                    continue
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
                    _record(diagnostics, color, (x,y,w,h), area, 'no_light_glyph')
                    continue
            px=max(2,int(w*.10));py=max(2,int(h*.10))
            crop=frame[max(0,y-py):min(height,y+h+py),max(0,x-px):min(width,x+w+px)].copy()
            result.append(dict(crop=crop,bounds=(x,y,w,h),color=color,area=area))
            _record(diagnostics, color, (x,y,w,h), area, 'candidate')
    if diagnostics is not None:
        diagnostics['candidate_count'] = len(result)
    return sorted(result,key=lambda c:c['area'],reverse=True)


def _record(diagnostics, color, bounds, area, reason):
    if diagnostics is None:
        return
    if reason != 'candidate':
        counts=diagnostics['rejected_counts']
        counts[reason]=counts.get(reason,0)+1
    # Bound diagnostic size on cluttered frames; exclude speckle rectangles.
    if reason != 'too_small':
        regions=diagnostics['regions']
        regions.append(dict(color=color,bounds=bounds,area=area,reason=reason))
        regions.sort(key=lambda row:row['area'],reverse=True)
        del regions[24:]


def extract_board_roi(frame,return_bounds=False,diagnostics=None):
    candidates=board_candidates(frame,diagnostics)
    if not candidates:
        return None
    item=candidates[0]
    return (item['crop'],item['bounds']) if return_bounds else item['crop']
