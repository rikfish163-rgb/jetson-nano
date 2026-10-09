"""Scoped lidar exceptions; completing one category never disables the device."""


def explicit_parking_lidar_disabled(cfg, state, action):
    return (state == 'PARKING' and action == 'PARKING' and
            cfg.get('parking_slot') in ('P4', 'P5') and
            cfg.get('parking_mode') == 'forward_center' and
            cfg.get('parking_entry_style') in ('S', 'T') and
            not cfg.get('parking_lidar_enabled', True))


def straight_lidar_consumed(cfg, state, action, completed):
    return (cfg.get('straight_lidar_once', False) and completed and
            ((action is None and state in ('LANE', 'GAP', 'WAIT_OBSTACLE')) or
             (action == 'STRAIGHT' and state in
              ('BLUE_APPROACH', 'BLUE_STOP', 'MANEUVER', 'REACQUIRE'))))


def lidar_guard_disabled_reason(cfg, state, action, completed=False):
    if explicit_parking_lidar_disabled(cfg, state, action):
        return 'parking_lidar_disabled'
    if straight_lidar_consumed(cfg, state, action, completed):
        return 'straight_lidar_consumed'
    if (not cfg.get('lidar_enabled', True) and
            (action != 'PARKING' or cfg.get('parking_mode') == 'forward_center')):
        return 'config_disabled'
    return None
