#!/usr/bin/env python2
"""Resolve one-file vehicle settings without starting nodes during inspection."""
from __future__ import print_function
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def scalar(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if value is None or isinstance(value, (dict, list)):
        raise ValueError('launch arguments must be scalar values')
    return str(value)


def resolve(root, argv):
    package = os.path.join(root, 'src', 'robot')
    full = ET.parse(os.path.join(package, 'launch', 'full.launch')).getroot()
    allowed = set(row.get('name') for row in full.findall('arg'))
    aliases = {'--S': 'parking_entry_style:=S', '--T': 'parking_entry_style:=T',
               '--no-bypass': 'timed_bypass_enabled:=false',
               '--follow-lane-start': 'startup_follow_lane:=true'}
    overrides, options = {}, []
    for token in argv:
        token = aliases.get(token, token)
        if token in ('--dump-params', '--nodes'):
            options.append(token)
            continue
        if ':=' not in token:
            raise ValueError('expected name:=value, got '+token)
        key, value = token.split(':=', 1)
        if key not in allowed:
            raise ValueError('unknown launch argument: '+key)
        overrides[key] = value
    path = overrides.get('vehicle_tuning', os.path.join(package, 'config', 'vehicle_tuning.yaml'))
    with open(path) as stream:
        tuning = yaml.safe_load(stream)
    if not isinstance(tuning, dict):
        raise ValueError('vehicle tuning must be a mapping')
    startup = tuning.get('launch', {})
    if not isinstance(startup, dict):
        raise ValueError('launch section must be a mapping')
    values = {}
    stack = ET.parse(os.path.join(package, 'launch', 'stack.launch')).getroot()
    mapped = set()
    # Reuse the launch file's existing arg-to-controller mapping, including
    # nested keys such as speed_raw/lane. No duplicate tuning table here.
    for row in stack.findall('param'):
        name = row.get('name', '')
        match = re.match(r'^\$\(arg ([a-zA-Z0-9_]+)\)$', row.get('value', ''))
        if not name.startswith('/competition/config/') or not match:
            continue
        key = match.group(1)
        if key not in allowed:
            continue
        mapped.add(key)
        value = tuning
        for part in name[len('/competition/config/'):].split('/'):
            if not isinstance(value, dict) or part not in value:
                break
            value = value[part]
        else:
            values[key] = scalar(value)
    for key, value in startup.items():
        if key not in allowed or key in mapped or key == 'vehicle_tuning':
            raise ValueError('invalid or duplicated launch setting: '+key)
        values[key] = scalar(value)
    values['vehicle_tuning'] = os.path.abspath(path)
    values.update(overrides)
    return options, values


def main(argv):
    try:
        options, values = resolve(ROOT, argv)
        # Inspection bypasses hardware checks and never starts ROS nodes.
        if not options:
            model = values.get('sign_model', '')
            if not (os.path.isfile(model) and os.path.getsize(model)):
                raise ValueError('TensorRT model is missing: '+model)
            if subprocess.call(['pgrep', '-f', '/usb_cam_node|/robot/master/main.py|/robot/master/controller_node.py'],
                               stdout=open(os.devnull, 'w')) == 0:
                raise ValueError('camera/controller already running; stop the previous run first')
        command = ['roslaunch'] + options + ['robocup_competition', 'full.launch']
        command += [key+':='+value for key, value in sorted(values.items())]
        os.execvp(command[0], command)
    except (ValueError, IOError, yaml.YAMLError) as error:
        print(str(error), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
