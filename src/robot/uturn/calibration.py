"""Optional U-turn calibration override of the shared chassis model."""
from robot.motion import calibration as chassis


def _config(cfg):
    if cfg.get('uturn_calibration') is not None:
        return dict(cfg,chassis_calibration=cfg['uturn_calibration'])
    return cfg


def validate(cfg):
    chassis.validate(_config(cfg))


def limit(cfg,gear,steer):
    return chassis.limit(_config(cfg),gear,steer)


def command_angle(cfg,gear,steer):
    return chassis.command_angle(_config(cfg),gear,steer)


def trial_active(cfg, state):
    return (state == 'UTURN' and cfg.get('uturn_trial_enabled',False) and
            (cfg.get('uturn_calibration') is not None or cfg.get('chassis_calibration') is not None))


def raw_angle(cfg, speed, steering_raw):
    """Inverse chassis mapping; speed is logical RAW, steering is wire RAW."""
    return chassis.raw_angle(_config(cfg),speed,steering_raw)


def speed_gain(cfg, speed):
    return chassis.speed_gain(_config(cfg),speed)
