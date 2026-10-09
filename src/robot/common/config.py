"""Load team-owned overrides in the same order as stack.launch."""
import copy
import os
import yaml


def read_mapping(path):
    with open(path) as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError('configuration must be a mapping: '+path)
    return value


def merge(base, override):
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def check_override(owner, override, catalog):
    allowed = set(catalog['modules'][owner]['parameters'])
    for key, value in override.items():
        paths = [key+'.'+child for child in value] if key == 'speed_raw' and isinstance(value, dict) else [key]
        for path in paths:
            if path not in allowed:
                raise ValueError('%s cannot override parameter %s' % (owner, path))


def load_config(config_dir, base_path=None, maneuver_path=None):
    catalog = read_mapping(os.path.join(config_dir, 'workspace_modules.yaml'))
    cfg = read_mapping(base_path or os.path.join(config_dir, 'competition.yaml'))
    cfg = merge(cfg, read_mapping(maneuver_path or os.path.join(config_dir, 'maneuvers.yaml')))
    for owner in sorted(catalog['modules']):
        override = read_mapping(os.path.join(os.path.dirname(config_dir), 'master' if owner == 'mission' else owner, 'config.yaml'))
        check_override(owner, override, catalog)
        cfg = merge(cfg, override)
    cfg['blue_stop_test'] = read_mapping(os.path.join(os.path.dirname(config_dir),'turn','blue_stop_test.yaml'))
    return cfg
