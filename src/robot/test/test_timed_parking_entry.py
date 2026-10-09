"""P4/P5 T-entry is blue-triggered, guarded and exactly one second long."""
import copy
import unittest

from test_core import CONFIG
from robot.common.contracts import encode_command
from robot.parking.entry import TimedRightEntry


class TimedParkingEntryTests(unittest.TestCase):
    def config(self):
        cfg = copy.deepcopy(CONFIG)
        cfg.update(parking_mode='forward_center', parking_slot='P4',
                   parking_entry_style='T',
                   parking_entry_speed_raw=12,
                   steering_command_scale_rad=.03)
        return cfg

    def task(self):
        return TimedRightEntry(self.config(), 10.0)

    def raw(self, task, now):
        command = task.command(now)
        return encode_command(command[0], command[1], task.cfg, 0)

    def test_starts_right_full_lock_and_finishes_after_one_second(self):
        task = self.task()
        first = self.raw(task, 10.0)
        self.assertEqual((first['speed_raw'], first['steering_raw']), (12, -22))
        middle = self.raw(task, 10.2)
        self.assertEqual((middle['speed_raw'], middle['steering_raw']), (12, -22))
        self.assertEqual(task.phase, 'TIMED_RIGHT')
        self.raw(task, 10.4)
        self.raw(task, 10.6)
        self.raw(task, 10.8)
        self.assertEqual(task.command(11.0), (0, 0))
        self.assertEqual(task.phase, 'FINISHED')
        self.assertEqual(task.reason, 'parking_t_entry_complete')
        self.assertEqual(task.command(11.2), (0, 0))

    def test_short_scheduler_delay_is_bounded_and_counts_valid_time(self):
        task = self.task()
        self.raw(task, 10.0)
        self.assertEqual(self.raw(task, 10.295)['speed_raw'], 12)
        self.assertEqual(task.phase, 'TIMED_RIGHT')
        self.raw(task, 10.545)
        self.raw(task, 10.795)
        self.raw(task, 11.045)
        self.assertEqual(task.command(11.295), (0, 0))

    def test_control_gap_over_half_second_faults_before_another_motion_command(self):
        task = self.task()
        self.raw(task, 10.0)
        self.assertEqual(task.command(10.501), (0, 0))
        self.assertEqual(task.phase, 'FAULT')
        self.assertEqual(task.reason, 'parking_t_entry_control_gap')

    def test_first_delayed_nonzero_command_starts_the_one_second_clock(self):
        task = self.task()
        self.assertEqual(self.raw(task, 10.295)['speed_raw'], 12)
        self.raw(task, 10.545)
        self.raw(task, 10.795)
        self.raw(task, 11.045)
        self.assertEqual(task.command(11.295), (0, 0))

    def test_backward_clock_faults(self):
        task = self.task()
        self.raw(task, 10.0)
        self.assertEqual(task.command(9.99), (0, 0))
        self.assertEqual(task.phase, 'FAULT')


if __name__ == '__main__':
    unittest.main()
