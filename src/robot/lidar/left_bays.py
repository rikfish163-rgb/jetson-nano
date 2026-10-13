"""P4/P5 point counts above/below a single left-side sign board.

Coordinates remain in laser_link, X forward / Y left. No pose integration,
motion publisher, camera mode, or parking state is needed by this detector.
"""
from __future__ import division
import math


DEFAULT_BOXES = dict(P4=[.90,1.20,1.00,1.30],P5=[.50,.80,1.00,1.30])
DEFAULT_REFERENCE_CENTERS = dict(P4=[1.00,1.43],P5=[.61,1.44])


def _sign_lines(points,centers,tolerance):
    # Line fitting only locates a board; no second board or pair test is needed.
    # Seed only near the two reference positions. Grow the complete connected
    # segment so clipping at the 20 cm boundary cannot move its midpoint.
    # Spatial grouping also avoids mixing two surfaces at adjacent scan angles.
    remaining=[p for p in points if p[1]>0]
    groups=[]
    while remaining:
        seed=next((i for i,p in enumerate(remaining) if any(
            math.hypot(p[0]-c[0],p[1]-c[1])<=tolerance for c in centers.values())),None)
        if seed is None:break
        group=[remaining.pop(seed)]
        for p in group:
            rest=[]
            for q in remaining:
                if math.hypot(p[0]-q[0],p[1]-q[1])<=.10:group.append(q)
                else:rest.append(q)
            remaining=rest
        groups.append(group)
    lines=[]
    for group in groups:
        if len(group)<8:continue
        cx=sum(p[0] for p in group)/len(group)
        cy=sum(p[1] for p in group)/len(group)
        xx=sum((p[0]-cx)**2 for p in group)
        yy=sum((p[1]-cy)**2 for p in group)
        xy=sum((p[0]-cx)*(p[1]-cy) for p in group)
        angle=.5*math.atan2(2*xy,xx-yy)
        ux,uy=math.cos(angle),math.sin(angle)
        along=[(x-cx)*ux+(y-cy)*uy for x,y in group]
        rms=math.sqrt(sum((-(x-cx)*uy+(y-cy)*ux)**2 for x,y in group)/len(group))
        if max(along)-min(along)<.12 or rms>.02 or ux<abs(uy):continue
        mid=(min(along)+max(along))/2
        midpoint=[cx+mid*ux,cy+mid*uy]
        error,name=min((math.hypot(midpoint[0]-center[0],midpoint[1]-center[1]),name)
                       for name,center in centers.items())
        if error>tolerance:continue
        lines.append(dict(mid=midpoint,axis=[ux,uy],count=len(group),
                          matched_bay=name,match_error_m=error))
    return lines


def _relative_box(points,line,name,gap):
    # Above the board is P4; below it is P5. The occupied bay is the other bay.
    offset=gap if name=='P4' else -gap
    x0,x1=offset-.10,offset+.20
    y0,y1=-.43,-.13
    mx,my=line['mid'];ux,uy=line['axis']
    count=0
    for x,y in points:
        dx,dy=x-mx,y-my
        if x0<=dx*ux+dy*uy<=x1 and y0<=-dx*uy+dy*ux<=y1:count+=1
    corners=[[mx+x*ux-y*uy,my+x*uy+y*ux]
             for x,y in ((x0,y0),(x1,y0),(x1,y1),(x0,y1))]
    return dict(name=name,count=count,corners=corners,reference=line)


def measure(points, cfg=None):
    cfg=cfg or {}
    lines=_sign_lines(points,cfg.get('reference_centers',DEFAULT_REFERENCE_CENTERS),
                      cfg.get('reference_match_radius_m',.20))
    if lines:
        # Each board independently supports either arrangement. When both boards
        # are present, retain each bay's strongest observation without pairing.
        boxes=[max((_relative_box(points,line,name,cfg.get('bay_spacing_m',.40))
                    for line in lines),key=lambda box:(box['count'],box['reference']['count']))
               for name in ('P4','P5')]
        supported=[box for box in boxes if box['count']>0]
        if supported:
            anchor=max(supported,key=lambda box:box['count'])['reference']
            # An unrelated long wall must not place the empty companion box.
            boxes=[box if box['count'] else _relative_box(points,anchor,box['name'],
                   cfg.get('bay_spacing_m',.40)) for box in boxes]
            return dict(valid=True,reason='single_sign_boxes',
                        counts=dict((box['name'],box['count']) for box in boxes),boxes=boxes)
    # No obstacle-supported board: distant/irrelevant lines must not hide
    # the calibrated point-count fallback, just like a missing board must not.
    boxes=[];counts={}
    for name in ('P4','P5'):
        x0,x1,y0,y1=cfg.get('boxes',DEFAULT_BOXES)[name]
        count=sum(1 for x,y in points if x0<=x<=x1 and y0<=y<=y1)
        corners=[[x0,y0],[x1,y0],[x1,y1],[x0,y1]]
        counts[name]=count
        boxes.append(dict(name=name,count=count,corners=corners))
    return dict(valid=True,reason='calibrated_boxes',counts=counts,boxes=boxes)


def from_scan(scan,cfg):
    points=[]
    for i,r in enumerate(scan.ranges):
        if not math.isnan(r) and not math.isinf(r) and scan.range_min<=r<scan.range_max:
            a=scan.angle_min+i*scan.increment
            points.append((r*math.cos(a),r*math.sin(a)))
    return measure(points,cfg)


def arm(sign_at,sign_stamp):
    return dict(sign_at=sign_at,sign_stamp=sign_stamp,stamp=-1.,frames=0,
                streak=0,case=None,measurement=None,occupied_evidence=None)


def observe(cache,measurement,stamp,now,cfg):
    if cache is None or stamp<=max(cache['stamp'],cache['sign_stamp']):return
    timeout=cfg.get('max_age_s',1.5)
    if not 0<=now-stamp<=timeout:return
    counts=measurement['counts'];threshold=cfg.get('min_points',1)
    case=None
    if measurement['valid']:
        a,b=counts['P4']>=threshold,counts['P5']>=threshold
        case='both_occupied' if a and b else 'P4' if a else 'P5' if b else 'empty'
    previous=(cache['case']==case and 0<stamp-cache['stamp']<=timeout)
    cache.update(stamp=stamp,frames=cache['frames']+1,case=case,
                 streak=cache['streak']+1 if previous else 1,measurement=measurement)
    if case in ('P4','P5','both_occupied'):
        cache['occupied_evidence']=dict(case=case,stamp=stamp,measurement=measurement)


def decide(cache,now,sign_at,cfg):
    result=dict(target=None,status='no_fresh_evidence',sign_at=sign_at)
    if (cache is None or cache['sign_at']!=sign_at or
            not 0<=now-cache['stamp']<=cfg.get('max_age_s',1.5)):return result
    result.update(scan_stamp=cache['stamp'],frames=cache['frames'],streak=cache['streak'],
                  measurement=cache['measurement'])
    evidence=cache.get('occupied_evidence')
    case=evidence['case'] if evidence is not None else cache['case']
    if evidence is not None:
        result.update(evidence_stamp=evidence['stamp'],measurement=evidence['measurement'],
                      latest_measurement=cache['measurement'])
    if case is None:
        result['status']='unconfirmed';return result
    result['status']=case
    if case in ('P4','P5','both_occupied'):
        target='P5' if case=='P4' else 'P4'
        result.update(status='selected',occupied=case,target=target,
            entry_s=cfg.get('p5_entry_s',0.) if target=='P5' else cfg.get('p4_entry_s',1.8),
            turn_s=cfg.get('turn_s',3.6),straight_s=cfg.get('straight_s',1.6),
            speed_raw=30,turn_steering_raw=22,settle_s=.7)
    return result
