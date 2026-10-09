"""Read recorded commands and uploaded console text; never start ROS nodes."""
import collections
import json
import pathlib
import statistics

ROOT = pathlib.Path(__file__).resolve().parents[3]
RUN = ROOT / 'field_data/lane_runs/20260930_043930_8446'
CONSOLE = pathlib.Path('/home/hetaisheng/.codex/attachments/60106a2d-ac50-4692-9abb-e2da03c96c50/Pasted text.txt')


def rows(name):
    return [json.loads(line) for line in (RUN / name).read_text().splitlines()]


targets = rows('target.jsonl')
start = next(row['data']['stamp'] for row in targets
             if row['data']['command']['speed_raw'] > 0)
targets = [row['data'] for row in targets if row['data']['stamp'] >= start]
seconds = collections.defaultdict(float)
episodes = []
active = None
for current, following in zip(targets, targets[1:]):
    seconds[current['reason']] += following['stamp'] - current['stamp']
    if current['command']['speed_raw'] == 0:
        if active is None:
            active = {'start': current['stamp'], 'reasons': set()}
        active['reasons'].add(current['reason'])
        active['end'] = following['stamp']
    elif active is not None:
        episodes.append(active)
        active = None
if active is not None:
    episodes.append(active)
for episode in episodes:
    episode['reasons'] = sorted(episode['reasons'])

console = CONSOLE.read_text()
states = []
invalid_states = 0
for line in console.splitlines():
    if 'competition_state {' not in line:
        continue
    try:
        states.append(json.JSONDecoder().raw_decode(line.split('competition_state ', 1)[1])[0])
    except ValueError:
        invalid_states += 1
lidar_ms = sorted(state['lidar_processing_ms'] for state in states
                  if state.get('lidar_processing_ms') is not None)
chassis = [row for row in rows('chassis.jsonl') if row['received'] >= start]
result = {
    'run': str(RUN),
    'first_positive_command': start,
    'last_recorded_command': targets[-1]['stamp'],
    'driving_interval_s': targets[-1]['stamp'] - start,
    'controller_zero_speed_episodes': len(episodes),
    'controller_zero_speed_s': sum(row['end'] - row['start'] for row in episodes),
    'seconds_by_reason': dict(seconds),
    'episodes': episodes,
    'recorded_chassis_status_count': len(chassis),
    'recorded_chassis_zero_speed_status_count': sum(row['data']['speed_raw'] == 0 for row in chassis),
    'recorded_chassis_timeout_count': sum(row['data']['timed_out'] for row in chassis),
    'recorded_serial_closed_count': sum(not row['data']['serial_open'] for row in chassis),
    'console_bridge_timeout_warning_count': console.count('Central command timed out after'),
    'console_state_count': len(states),
    'invalid_console_state_count': invalid_states,
    'sampled_lidar_processing_ms': {
        'median': statistics.median(lidar_ms),
        'max': max(lidar_ms),
    },
    'limits': ['No raw scan arrival history or actual wheel speed measurements.',
               'Command intervals are sampled records, not measured physical stop durations.',
               'Console states are sparse; their stamp gaps do not measure raw lidar frequency.'],
}
output = pathlib.Path(__file__).with_name('analysis.json')
output.write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps({key: value for key, value in result.items() if key != 'episodes'}, indent=2))
