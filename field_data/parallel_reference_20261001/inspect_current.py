from __future__ import print_function,division
import os,glob,json
import cv2,numpy as np,yaml
from robot.parallel_parking.reference_vision import ReferenceVision,paint_mask
from robot.camera.landmarks import image_segments
cv2.setNumThreads(1)
cfg=yaml.safe_load(open('src/robot/config/competition.yaml'))
directory='field_data/parallel_reference_20261001/current'
vision=ReferenceVision(cfg)
for i,path in enumerate(sorted(glob.glob(directory+'/frame[0-9][0-9].png'))):
    frame=cv2.imread(path)
    result,bev=vision.observe(frame,1.+i*.2)
    print(os.path.basename(path),json.dumps(result))
    if bev is None:continue
    c=vision.camera;ppm=c['pixels_per_m']
    white=paint_mask(bev,cfg['white'],ppm)
    left=int(c['origin_u']+.08*ppm);right=int(c['origin_u']+.95*ppm)
    metric=lambda pts:np.column_stack(((c['origin_v']-pts[:,1])/ppm,(c['origin_u']-left-pts[:,0])/ppm))
    segs=image_segments(white[:int(c['origin_v']-.5*ppm),left:right],metric,ppm,2.5)
    for n,s in enumerate(segs):
        a,b=s;d=b-a
        print(n,np.round(s,3).tolist(),'len',round(np.linalg.norm(d),3),'slope',round(d[0]/d[1],3) if abs(d[1])>.001 else None)
        pts=[(int(c['origin_u']-p[1]*ppm),int(c['origin_v']-p[0]*ppm)) for p in s]
        cv2.line(bev,pts[0],pts[1],(0,0,255),1);cv2.putText(bev,str(n),pts[0],cv2.FONT_HERSHEY_SIMPLEX,.4,(0,255,0),1)
    cv2.imwrite(path.replace('.png','_bev.jpg'),bev)
    cv2.imwrite(path.replace('.png','_white.png'),white)
