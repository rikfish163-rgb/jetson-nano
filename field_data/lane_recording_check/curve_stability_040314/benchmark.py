"""No ROS publishers: compare identical raw scans and collision guards."""
from __future__ import print_function
import imp
import json
import math
import os
import time
from robot.common.config import load_config
from robot.master.controller import Controller
from robot.lidar.scan import Scan

root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
backup = os.path.join(root, 'field_data/code_backups/20260930_curve_stability/src/robot')
old_obstacle = imp.load_source('old_obstacle_bench', os.path.join(backup, 'obstacle/controller.py'))
old_geometry = imp.load_source('old_geometry_bench', os.path.join(backup, 'common/geometry.py'))
old_obstacle.collision = old_geometry.collision
cfg = load_config(os.path.join(root, 'src/robot/config'))
cfg.update(wait_green=False, lidar_enabled=True, steering_command_scale_rad=.03)
cfg['lidar'].update(shape_filter=True, shape_mode='line_reject')
c = Controller(cfg)
new_obstacle = c._runtime.modules['obstacle']
n = 3000
ranges = [1.5+.3*math.sin(i*.017) for i in range(n)]
c.scan = Scan(ranges, -math.pi, 2*math.pi/n, .02, 8, (0,0,0), cfg['lidar'], 1)
results = {}
for label, module in (('before', old_obstacle), ('after', new_obstacle)):
    c._runtime.modules['obstacle'] = module
    for unused in range(3): c.checked_command((20,-.03), 1, False)
    started = time.time()
    commands = [c.checked_command((20,-.03), 1, False) for unused in range(30)]
    results[label] = dict(average_ms=(time.time()-started)*1000/30,
                         commands=sorted(set(commands)))
assert results['before']['commands'] == results['after']['commands']
c._runtime.modules['obstacle'] = new_obstacle
c.close()
print(json.dumps(results, indent=2))
