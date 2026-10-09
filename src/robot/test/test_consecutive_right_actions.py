"""Two consecutive RIGHT signs own two distinct blue-triggered maneuvers."""
import os
import unittest

import robot
from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.master.controller import Controller


class ConsecutiveRightActionTests(unittest.TestCase):
    def test_same_label_without_a_visibility_gap_runs_at_two_distinct_junctions(self):
        cfg = load_config(os.path.join(os.path.dirname(robot.__file__), 'config'))
        cfg.update(wait_green=False, lidar_enabled=False, intersection_wait_s=0.,
                   steering_command_scale_rad=.03, right_timed_enabled=True,
                   right_timed_reverse_s=.1, right_timed_turn_s=.2,
                   right_timed_exit_reverse_s=.1, exit_frames=2,
                   left_reference_stable_s=.1)
        cfg['blue_stop_test'].update(forward_seconds=.1)
        c = Controller(cfg)
        self.addCleanup(c.close)
        now = 1.
        triggers = []
        for junction in range(2):
            # Place a fresh camera view at the next distinct junction. The
            # label remains RIGHT; this fixture introduces no visibility gap.
            c.set_pose((2.*junction, 0., 0.), now)
            for unused in range(cfg.get('direction_sign_votes', 2)):
                now += .1
                c.observe_sign('RIGHT', .99, now, now)
            self.assertEqual(c.pending, 'RIGHT')
            phases = {}
            started = False
            for unused in range(120):
                now += .1
                c.observe_sign('RIGHT', .99, now, now)
                c.observe_lane([(.25, 0.), (.45, 0.), (.65, 0.)], .99, now)
                cam = cfg['front_camera']
                x = (cam['origin_v']-.72*(cam['bev_height']-1))/cam['pixels_per_m']
                c.observe_ground(dict(source='front', part='markers', slots=[],
                    markers=[dict(kind='junction', x=x, y=0., length=.8)],
                    blue_lines=[dict(x=x, y=0., yaw=0., length=.8)]), now)
                command = c.tick(now)
                if junction == 1 and c.state == 'BLUE_APPROACH':
                    self.assertLessEqual(abs(command[0]), 16)
                if c.right_lock is not None and command[0]:
                    started = True
                    phases[c.right_lock['phase']] = encode_command(command[0], command[1], cfg, 0)
                if started and c.last_completed_action == 'RIGHT' and c.action is None:
                    break
            self.assertEqual(c.state, 'LANE', c.reason)
            self.assertIsNone(c.action)
            self.assertEqual(c.last_completed_action, 'RIGHT')
            self.assertEqual(phases['TIMED_TURN']['steering_raw'], -22)
            self.assertLess(phases['TIMED_REVERSE']['speed_raw'], 0)
            self.assertLess(phases['TIMED_EXIT_REVERSE']['speed_raw'], 0)
            self.assertEqual(phases['TIMED_TURN']['speed_raw'],
                             cfg.get('right_timed_turn_speed_raw', 30))
            self.assertEqual(c.right_completed_count, junction + 1)
            triggers.append(c.last_blue_trigger['stamp'])
        self.assertGreater(triggers[1], triggers[0])


if __name__ == '__main__':
    unittest.main()
