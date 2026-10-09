"""Shared RAW-to-motion model; lane steering gain is a separate parameter."""
from __future__ import division
import math


def validate_speed_points(points):
    if not isinstance(points,list) or not points:
        raise ValueError('invalid chassis speed points')
    previous_raw=previous_velocity=0.
    for point in points:
        if not isinstance(point,(list,tuple)) or len(point)!=2:
            raise ValueError('invalid chassis speed point')
        raw,velocity=point
        if any(isinstance(v,bool) or not isinstance(v,(int,float)) for v in point):
            raise ValueError('invalid chassis speed point values')
        if not previous_raw<raw<=100 or not 0<velocity<=3 or velocity<previous_velocity:
            raise ValueError('invalid chassis speed point range/order')
        previous_raw,previous_velocity=raw,velocity


def speed_from_points(speed,points):
    """Measured m/s magnitude; interpolate within range, scale endpoints outside."""
    raw=abs(speed)
    if raw<=points[0][0]:return raw*points[0][1]/points[0][0]
    for (a,va),(b,vb) in zip(points,points[1:]):
        if raw<=b:return va+(vb-va)*(raw-a)/(b-a)
    return raw*points[-1][1]/points[-1][0]


def validate(cfg):
    table = cfg.get('chassis_calibration')
    if table is None:
        return
    if not isinstance(table,dict):
        raise ValueError('invalid chassis calibration table')
    sign = table.get('reverse_steering_sign',1)
    if isinstance(sign,bool) or sign not in (-1,1):
        raise ValueError('invalid chassis reverse steering sign')
    for key in ('forward_left_radius','forward_right_radius',
                'reverse_left_radius','reverse_right_radius',
                'forward_mps_per_raw','reverse_mps_per_raw'):
        value = table.get(key)
        if isinstance(value,bool) or not isinstance(value,(int,float)):
            raise ValueError('invalid chassis calibration '+key)
        upper = 5. if 'radius' in key else .1
        if not 0 < value <= upper:
            raise ValueError('invalid chassis calibration '+key)
    if 'speed_points' in table:
        points=table['speed_points']
        if not isinstance(points,dict) or set(points)!=set(('forward','reverse')):
            raise ValueError('chassis speed points require forward and reverse')
        for rows in points.values():validate_speed_points(rows)


def limit(cfg, gear, steer):
    table = cfg.get('chassis_calibration')
    if table is None:
        return cfg['max_steer']
    side = steer if gear > 0 else steer*table.get('reverse_steering_sign',1)
    key = ('forward_' if gear > 0 else 'reverse_')+('left_radius' if side >= 0 else 'right_radius')
    return math.atan(cfg['wheelbase']/table[key])


def command_angle(cfg, gear, steer):
    fraction = max(-1.,min(1.,steer/limit(cfg,gear,steer)))
    sign = 1 if gear > 0 else cfg.get('chassis_calibration',{}).get('reverse_steering_sign',1)
    return sign*fraction*cfg.get('steering_command_scale_rad',cfg['max_steer'])


def raw_angle(cfg, speed, steering_raw):
    gear = 1 if speed >= 0 else -1
    fraction = steering_raw/float(cfg['steering_sign']*cfg['steering_raw_limit'])
    fraction = max(-1.,min(1.,fraction))
    sign = 1 if gear > 0 else cfg.get('chassis_calibration',{}).get('reverse_steering_sign',1)
    side = sign*fraction
    return side*limit(cfg,gear,side)


def speed_gain(cfg, speed):
    gear = 'forward' if speed >= 0 else 'reverse'
    table = cfg.get('chassis_calibration')
    if table is not None and 'speed_points' in table:
        points=table['speed_points'][gear]
        return speed_from_points(speed,points)/abs(speed) if speed else points[0][1]/points[0][0]
    return table[gear+'_mps_per_raw'] if table is not None else cfg['raw_to_mps'][gear]
