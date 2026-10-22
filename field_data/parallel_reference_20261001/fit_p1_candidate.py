"""Offline candidate only. Four bay corners are fit data, not validation."""
from __future__ import print_function,division
import json,copy
import cv2,numpy as np,yaml
from robot.parallel_parking.reference_vision import ReferenceVision,paint_mask
from robot.camera.landmarks import image_segments
cv2.setNumThreads(1)
directory='field_data/parallel_reference_20261001'
cfg=yaml.safe_load(open('src/robot/config/competition.yaml'))
# Rectified image line-centre intersections, ordered front-mouth/front-outer/
# rear-mouth/rear-outer. Pixel uncertainty must be checked independently.
source=np.float32([[392.,164.4],[452.1,164.4],[454.6,198.],[588.4,198.5]])
ground=np.float32([[1.60,-.29],[1.60,-.65],[.90,-.29],[.90,-.65]])
c=cfg['front_camera']
dest=np.column_stack((c['origin_u']-ground[:,1]*c['pixels_per_m'],
                      c['origin_v']-ground[:,0]*c['pixels_per_m'])).astype(np.float32)
H=cv2.getPerspectiveTransform(source,dest)
candidate=copy.deepcopy(cfg);candidate['front_camera']['H']=H.reshape(-1).tolist()
image=cv2.imread(directory+'/second_position/clear_front.png')
seed=ReferenceVision(candidate);observed,seed_bev=seed.observe(image,1.)
camera=seed.camera;ppm=camera['pixels_per_m']
white=paint_mask(seed_bev,cfg['white'],ppm)
left=int(camera['origin_u']+.08*ppm);right=int(camera['origin_u']+.95*ppm)
metric=lambda pts:np.column_stack(((camera['origin_v']-pts[:,1])/ppm,
                                  (camera['origin_u']-left-pts[:,0])/ppm))
segments=image_segments(white[:int(camera['origin_v']-.5*ppm),left:right],metric,ppm,2.5)
def intersect(first,second):
    a,b=np.asarray(first);p,q=np.asarray(second)
    t=np.linalg.solve(np.column_stack((b-a,p-q)),p-a)[0]
    return a+t*(b-a)
front=np.asarray(observed['p1_lines'][0])
longitudinal=[s for s in segments if abs(s[1][1]-s[0][1])<.15*abs(s[1][0]-s[0][0])]
outer=max((s for s in longitudinal if np.mean(s[:,1])<-.5),key=lambda s:abs(s[1][0]-s[0][0]))
mouth=max((s for s in longitudinal if -.4<np.mean(s[:,1])<-.2),key=lambda s:abs(s[1][0]-s[0][0]))
rear=min((s for s in segments if abs(s[1][0]-s[0][0])<.15*abs(s[1][1]-s[0][1])),
         key=lambda s:abs(np.mean(s[:,0])-.9))
corners=np.asarray([intersect(front,mouth),intersect(front,outer),
                    intersect(rear,mouth),intersect(rear,outer)])
pixels=np.column_stack((camera['origin_u']-360-corners[:,1]*ppm,
                        camera['origin_v']-corners[:,0]*ppm))
source=cv2.perspectiveTransform(pixels.reshape(-1,1,2),np.linalg.inv(seed.detector.H)).reshape(-1,2).astype(np.float32)
H=cv2.getPerspectiveTransform(source,dest)
candidate['front_camera']['H']=H.reshape(-1).tolist()
with open(directory+'/p1_projection_candidate.yaml','w') as out:
    yaml.safe_dump(dict(front_camera=dict(H=H.reshape(-1).tolist())),out,default_flow_style=False)
baseline=np.float64([[419.4128991331301,175.9786734407722],
                    [525.9556452104007,176.0647142351618]])
bev=cv2.perspectiveTransform(baseline.reshape(-1,1,2),H).reshape(-1,2)
line=np.column_stack(((c['origin_v']-bev[:,1])/c['pixels_per_m'],
                     (c['origin_u']-bev[:,0])/c['pixels_per_m']))
a,b=line;gap=float(a[0]-a[1]*(b[0]-a[0])/(b[1]-a[1])-cfg['wheelbase'])
report=dict(applied=False,source_image='second_position/clear_front.png',
    source_rectified_points_px=source.tolist(),ground_points_m=ground.tolist(),
    assumed_parallel_to_bay=True,bay_length_m=.70,bay_width_m=.36,
    user_front_axle_gap_m=1.34,user_mouth_lateral_m=.29,
    validation_image='current/frame05.png',validation_expected_gap_m=.88,
    validation_gap_m=gap,validation_width_m=float(np.linalg.norm(b-a)),
    validation_passed=abs(gap-.88)<=.02 and abs(np.linalg.norm(b-a)-.36)<=.03,
    note='The 88 cm measurement must correspond to this saved validation image; pose correspondence has not been rechecked.')
with open(directory+'/candidate_validation.json','w') as out:json.dump(report,out,indent=2)
print(json.dumps(report))
vision=ReferenceVision(candidate)
result,preview=vision.observe(image,1.)
print(json.dumps(result))
cv2.imwrite(directory+'/second_position/candidate_bev.png',preview)
