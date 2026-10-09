#!/usr/bin/env python2
"""List exact inference images and the controller events from the same run."""
from __future__ import print_function
import argparse
import glob
import json
import os


def read_events(directory):
    session_path = os.path.join(directory, '_session.json')
    if not os.path.exists(session_path):
        return []
    with open(session_path) as stream:
        session = json.load(stream)
    log_dir = session.get('ros_log_dir', '')
    paths = [os.path.join(log_dir, 'rosout.log')]
    if not os.path.isfile(paths[0]):
        paths = glob.glob(os.path.join(log_dir, 'competition_controller-*.log'))
    result = []
    for path in paths:
        if not os.path.isfile(path):
            continue
        with open(path) as stream:
            for line in stream:
                for kind in ('competition_sign_decision', 'competition_control_event'):
                    marker = kind+' '
                    if marker in line:
                        try:
                            result.append((kind, json.loads(line.split(marker, 1)[1])))
                        except ValueError:
                            pass
    return sorted(result, key=lambda event: event[1]['stamp'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', help='capture session directory (default: latest)')
    parser.add_argument('--label', help='filter accepted label, top prediction or YOLO candidates')
    parser.add_argument('--limit', type=int, default=20)
    args = parser.parse_args()
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
    directory = args.directory
    if not directory:
        sessions = [p for p in glob.glob(os.path.join(root, 'field_data/sign_capture/*'))
                    if os.path.isdir(p)]
        if not sessions:
            parser.exit(0, 'No captures yet. Restart the vehicle with sign_capture:=true.\n')
        directory = max(sessions, key=os.path.getmtime)
    if args.limit < 1 or not os.path.isdir(directory):
        parser.error('Provide an existing directory and --limit >= 1')
    events = read_events(directory)
    decisions = dict(('%.9f' % d['stamp'], d) for kind, d in events
                     if kind == 'competition_sign_decision')
    rows = []
    for path in glob.glob(os.path.join(directory, '*.json')):
        if os.path.basename(path).startswith('_'):
            continue
        try:
            with open(path) as stream:
                info = json.load(stream)
            labels = [info.get('label', ''), info.get('top_label', '')]
            labels += [d.get('label', '') for d in info.get('detections', [])]
            if args.label and args.label.upper() not in [label.upper() for label in labels]:
                continue
            rows.append((info['stamp'], path, info))
        except (ValueError, KeyError, IOError) as exc:
            print('Unreadable metadata: %s (%s)' % (path, exc))
    rows.sort()
    print('Directory: '+os.path.abspath(directory))
    print('Captures: %d matching; showing last %d' % (len(rows), args.limit))
    for stamp, path, info in rows[-args.limit:]:
        decision = decisions.get('%.9f' % stamp, {}).get('decision', '--')
        print('%.9f label=%s top=%s confidence=%.4f gate=%s decision=%s' % (
            stamp, info.get('label') or 'REJECTED', info.get('top_label', ''),
            info.get('confidence', 0), info.get('range_reason', ''), decision))
        print('  frame: '+path[:-5]+'_frame.png')
        print('  metadata: '+path)
    if events:
        print('Recent controller events (source stamps; -- means no exact decision event):')
        for kind, event in events[-args.limit:]:
            keys = ('stamp', 'state', 'reason', 'command', 'label', 'confidence',
                    'decision', 'pending', 'action')
            print(kind+' '+json.dumps(dict((key, event[key]) for key in keys
                                           if key in event), sort_keys=True))
    else:
        print('Controller log unavailable; metadata describes detector output only.')


if __name__ == '__main__':
    main()
