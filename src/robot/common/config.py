"""Read runtime YAML mappings."""
import yaml


def read_mapping(path):
    with open(path) as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError('configuration must be a mapping: '+path)
    return value
