"""U-turn-only empirical full-lock model. Intermediate angles are interpolated."""
import math


def validate(cfg):
    table=cfg.get('uturn_calibration')
    if table is None:return
    if not isinstance(table,dict):raise ValueError('invalid uturn calibration table')
    sign=table.get('reverse_steering_sign',1)
    if isinstance(sign,bool) or sign not in (-1,1):
        raise ValueError('invalid uturn reverse steering sign')
    for key in ('forward_left_radius','forward_right_radius',
                'reverse_left_radius','reverse_right_radius',
                'forward_mps_per_raw','reverse_mps_per_raw'):
        value=table.get(key)
        if isinstance(value,bool) or not isinstance(value,(int,float)):
            raise ValueError('invalid uturn calibration '+key)
        upper=5. if 'radius' in key else .1
        if not 0 < value <= upper:raise ValueError('invalid uturn calibration '+key)


def limit(cfg,gear,steer):
    table=cfg.get('uturn_calibration')
    if table is None:return cfg['max_steer']
    # Map model curvature to RAW side; reverse travel already reverses yaw.
    raw_side=steer if gear>0 else steer*table.get('reverse_steering_sign',1)
    key=('forward_' if gear>0 else 'reverse_')+('left_radius' if raw_side>=0 else 'right_radius')
    return math.atan(cfg['wheelbase']/table[key])


def command_angle(cfg,gear,steer):
    fraction=max(-1.,min(1.,steer/limit(cfg,gear,steer)))
    sign=1 if gear>0 else cfg.get('uturn_calibration',{}).get('reverse_steering_sign',1)
    return sign*fraction*cfg.get('steering_command_scale_rad',cfg['max_steer'])


def trial_active(cfg, state):
    return (state == 'UTURN' and cfg.get('uturn_trial_enabled',False) and
            cfg.get('uturn_calibration') is not None)


def raw_angle(cfg, speed, steering_raw):
    """Inverse chassis mapping; speed is logical RAW, steering is wire RAW."""
    gear = 1 if speed >= 0 else -1
    fraction = steering_raw / float(cfg['steering_sign']*cfg['steering_raw_limit'])
    fraction = max(-1.,min(1.,fraction))
    sign = 1 if gear > 0 else cfg.get('uturn_calibration',{}).get('reverse_steering_sign',1)
    physical_side = sign*fraction
    return physical_side*limit(cfg,gear,physical_side)


def speed_gain(cfg, speed):
    gear = 'forward' if speed >= 0 else 'reverse'
    table = cfg.get('uturn_calibration')
    return table[gear+'_mps_per_raw'] if table is not None else cfg['raw_to_mps'][gear]
