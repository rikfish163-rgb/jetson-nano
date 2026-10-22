#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Independent P4 forward entry: front axle starts on the blue line."""
from __future__ import print_function
import argparse
import json
import os
import sys
import parking_line_stop_test as shared
from robot.common.config import read_mapping
from p4_forward_core import ForwardRun, validate


def arguments(argv=None):
    cli = sys.argv[1:] if argv is None else list(argv)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=os.path.join(shared.ROOT, 'src/robot/config/p4_forward_test.yaml'))
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--execute', action='store_true', help='run one physical trial')
    modes.add_argument('--observe', action='store_true', help='camera diagnostics only')
    parser.add_argument('--image-topic', default='/front/usb_cam/image_raw')
    chosen = parser.parse_args(cli)
    cfg = validate(read_mapping(chosen.config))
    # Reuse only the established camera/base/command lifecycle. Its road/side
    # settings do not control this state machine or this vision adapter.
    args = shared.arguments([])
    args.execute, args.observe = chosen.execute, chosen.observe
    args.image_topic, args.config = chosen.image_topic, chosen.config
    args.countdown, args.camera_timeout = cfg['countdown_s'], cfg['camera_timeout_s']
    args.lost_frames = cfg['lost_frames']
    args.p4_config = cfg
    args.config = os.path.realpath(args.config)
    return args


def main(argv=None):
    try:
        args = arguments(argv)
        print('P4 ONLY: manually place FRONT AXLE on blue line, nose toward P4.')
        print('Config: '+args.config)
        print(json.dumps(args.p4_config, sort_keys=True, indent=2))
        print('PREPARE -> STOP/SEARCH SIDES -> SIDE_ALIGN -> ALIGN BOTTOM -> STOP/VERIFY -> FINAL -> STOP.')
        print('SPACE / x / q / Ctrl+C = STOP. No sign recognition or auto avoidance.')
        if not args.execute and not args.observe:
            print('PREVIEW ONLY: no ROS node and no vehicle command.')
            return 0
        from p4_forward_vision import BayVision, NoRoadVision
        return shared.run(args, task_factory=lambda unused: ForwardRun(args.p4_config),
                          node_name='p4_forward_test', status_topic='/parking_p4_forward/status',
                          lane_vision_factory=NoRoadVision, line_vision_factory=BayVision,
                          success_reasons=('P4_COMPLETE',))
    except (ValueError, RuntimeError, IOError, KeyboardInterrupt) as exc:
        print('STOP / NOT STARTED: %s' % exc, file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
