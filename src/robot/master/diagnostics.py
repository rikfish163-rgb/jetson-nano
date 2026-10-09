"""Read-only graph readiness. A missing control chain cannot report PASS."""
from robot.common.lidar_policy import lidar_guard_disabled_reason


def sensor_issues(status, cfg):
    required = dict(lane=cfg.get('sensor_timeout', .5),
                    front_ground=cfg.get('ground_timeout', .8))
    disabled = lidar_guard_disabled_reason(cfg, status.get('state'),
        status.get('action'), status.get('timed_bypass_completed', False))
    if disabled not in ('parking_lidar_disabled', 'straight_lidar_consumed') and (cfg.get('lidar_enabled', True) or (cfg.get('parking_enabled', True) and cfg.get('parking_mode')!='forward_center')):
        required['scan'] = cfg.get('sensor_timeout', .5)
    if status.get('state')=='UTURN' or (status.get('state')=='PARKING' and cfg.get('parking_mode')!='forward_center'):
        required['rear_ground'] = cfg.get('ground_timeout', .8)
    ages = status.get('source_ages', {})
    return ['sensor stale or missing: '+name for name, timeout in sorted(required.items())
            if not isinstance(ages.get(name), (int, float)) or
            not 0 <= ages[name] <= timeout]


def graph_issues(publishers, types, cfg, live):
    required = {
        cfg.get('lane_observation_topic', '/vision/lane_observation'): 'std_msgs/String',
        '/competition/sign': 'std_msgs/String',
        '/competition/ground': 'std_msgs/String',
        '/competition/status': 'std_msgs/String',
        '/control/cmd' if live else '/competition/control_preview': 'std_msgs/String',
    }
    if cfg.get('lidar_enabled', True) or (cfg.get('parking_enabled', True) and cfg.get('parking_mode')!='forward_center'):
        required[cfg.get('scan_topic', '/scan')] = 'sensor_msgs/LaserScan'
    if live:
        required['/ackermann_cmd'] = 'ackermann_msgs/AckermannDriveStamped'
    issues = []
    for topic, expected in sorted(required.items()):
        owners = publishers.get(topic, [])
        if not owners:
            issues.append('missing publisher: '+topic)
        elif len(owners) != 1:
            issues.append('multiple publishers: '+topic)
        if owners and types.get(topic) != expected:
            issues.append('incompatible message type: '+topic)
    for topic in ('/control/cmd', '/ackermann_cmd'):
        if len(publishers.get(topic, [])) > 1:
            issues.append('multiple control owners: '+topic)
    return issues
