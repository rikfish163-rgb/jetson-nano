"""Reuse road lane steering while the parking reference decides when to stop."""
import copy
from robot.common.contracts import decode,number,encode_command
from robot.master.controller import Controller


class LaneFollower(object):
    def __init__(self,cfg,speed):
        self.cfg=copy.deepcopy(cfg)
        self.cfg['speed_raw']['lane']=speed
        self.cfg['speed_raw']['gap']=speed
        self.cfg['lane_curve_speed_raw']=speed
        self.core=Controller(self.cfg)

    def observe(self,raw,now):
        data,stamp=decode(raw,now,self.cfg['sensor_timeout'])
        if data.get('frame')!=self.cfg['lane_frame']:
            raise ValueError('lane frame mismatch')
        confidence=number(data.get('confidence'))
        if not 0<=confidence<=1:raise ValueError('invalid lane confidence')
        rows=data.get('points')
        if not isinstance(rows,list) or len(rows)>1000:
            raise ValueError('invalid lane points')
        points=[]
        for row in rows:
            if not isinstance(row,list) or len(row)!=2:raise ValueError('invalid lane point')
            x,y=map(number,row)
            if not 0<x<=8 or abs(y)>8 or (points and x<=points[-1][0]):
                raise ValueError('unordered or unbounded lane path')
            points.append((x,y))
        # Observations are already in the current vehicle frame. Each fresh
        # path replaces the previous one; parking floor pose is independent.
        self.core.observe_lane(points,confidence,stamp)

    def command(self,now):
        speed,angle=self.core.lane_command(now)
        self.core.issued_steer=angle
        result=encode_command(speed,angle,self.cfg,0)
        return result['speed_raw'],result['steering_raw']
