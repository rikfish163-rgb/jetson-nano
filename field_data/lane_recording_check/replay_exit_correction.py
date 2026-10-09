"""No publishers: check real recorded paths after the skewed-exit correction."""
from __future__ import print_function
import glob
import json
import os
from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.master.controller import Controller

ROOT = '/home/nano/robocup_ws'
OUT = os.environ.get('LANE_REPLAY_OUTPUT', ROOT + '/field_data/lane_recording_check/exit_correction')
REQUIRE_FULL_RIGHT = os.environ.get('LANE_REPLAY_REQUIRE_FULL_RIGHT') == '1'
if not os.path.isdir(OUT):
    os.makedirs(OUT)
report = dict(method='Fixed recorded vehicle-frame observations, replay pose zero. Recorded external safety stops are preserved; no simulated new trajectory.', runs=[], known_cases=[])
LANE_REASONS = ('tracking_lane', 'tracking_gap', 'gap_limit_wait_for_lane',
                'lane_stream_stale', 'no_initial_lane', 'lane_behind_vehicle')
known = {'20260929_230116_1410': [1790694132.4520724],
         '20260929_232406_4508': [1790695509.9184332, 1790695510.1848896,
                                  1790695537.3839903, 1790695537.451351]}
for run in sorted(glob.glob(ROOT + '/field_data/lane_runs/*')):
    source = run + '/target.jsonl'
    if not os.path.isfile(source):
        continue
    name = os.path.basename(run)
    cfg = load_config(ROOT + '/src/robot/config')
    cfg.update(wait_green=False, lidar_enabled=False, steering_command_scale_rad=.03)
    core = Controller(cfg)
    counts = dict(run=name, ticks=0, locks=0, releases=0, guarded=0,
                  positive_while_locked=0, stops=0, false_stop_window_ticks=0,
                  false_stop_window_stops=0, moving_locked_not_full=0,
                  external_stops_preserved=0)
    stream = open(OUT + '/' + name + '.jsonl', 'w')
    seen = set()
    for line in open(source):
        row = json.loads(line)['data']
        if row['state'] not in ('LANE', 'GAP'):
            core.lane_curve_lock = None
            core.gap_origin = None
            core.state = 'LANE'
            continue
        selection = row.get('selection')
        core.lane = [tuple(p) for p in selection['path']] if selection else []
        core.lane_stamp = row['source_stamp']
        core.lane_target = None
        before = core.lane_curve_lock is not None
        external_stop = row['command']['speed_raw'] == 0 and row['reason'] not in LANE_REASONS
        if external_stop:
            # The camera recording cannot reconstruct lidar/estop inputs.
            # Keep those recorded safety decisions instead of bypassing them.
            speed = 0
            steer = row['command']['steering_raw'] * cfg['steering_command_scale_rad'] / (cfg['steering_raw_limit'] * cfg['steering_sign'])
        else:
            speed, steer = core.lane_command(row['stamp'])
        core.issued_steer = steer
        raw = encode_command(speed, steer, cfg, 0)['steering_raw']
        locked = core.lane_curve_lock is not None
        guarded = bool(core.lane_target and core.lane_target['selection'] == 'right_curve_hold')
        result = dict(stamp=row['stamp'], source_stamp=row['source_stamp'],
                      original_raw=row['command']['steering_raw'], raw=raw,
                      original_speed=row['command']['speed_raw'], speed=speed,
                      state=core.state, guarded=guarded,
                      external_stop_reason=row['reason'] if external_stop else None,
                      lock=dict(core.lane_curve_lock) if locked else None)
        stream.write(json.dumps(result) + '\n')
        counts['ticks'] += 1
        counts['locks'] += locked and not before
        counts['releases'] += before and not locked
        counts['guarded'] += guarded
        counts['positive_while_locked'] += locked and raw > 0
        counts['moving_locked_not_full'] += locked and speed != 0 and raw != -cfg['steering_raw_limit'] * cfg['steering_sign']
        counts['stops'] += speed == 0
        counts['external_stops_preserved'] += external_stop
        if name == '20260930_011909_18373' and 1790702407.157068 <= row['stamp'] < 1790702419.298211:
            counts['false_stop_window_ticks'] += 1
            counts['false_stop_window_stops'] += speed == 0
        for stamp in known.get(name, []):
            if abs(row['source_stamp'] - stamp) < 1e-6 and stamp not in seen:
                seen.add(stamp)
                case = dict(result, run=name)
                report['known_cases'].append(case)
    stream.close()
    core.close()
    if counts['ticks']:
        report['runs'].append(counts)
        print(json.dumps(counts))
with open(OUT + '/summary.json', 'w') as output:
    json.dump(report, output, indent=2)
assert all(r['positive_while_locked'] == 0 for r in report['runs'])
if REQUIRE_FULL_RIGHT:
    assert all(r['moving_locked_not_full'] == 0 for r in report['runs'])
latest = next(r for r in report['runs'] if r['run'] == '20260930_011909_18373')
assert latest['false_stop_window_ticks'] > 200
assert latest['false_stop_window_stops'] == 0
assert len(report['known_cases']) == 5
assert all(case['raw'] <= 0 for case in report['known_cases'])
