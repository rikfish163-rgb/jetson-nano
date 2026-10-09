import unittest

from robot.obstacle.lidar_left_pass_core import LeftPass, clusters


def target(x, y, radius=.05):
    return dict(x=x, y=y, radius=radius, points=[(x, y)])


class LeftPassTest(unittest.TestCase):
    def test_left_pass_tracks_scan_distance_and_stops_after_target(self):
        control = LeftPass(clearance=.20)
        observations = [(1., 0.), (.8, -.1), (.6, -.2), (.4, -.35),
                        (.2, -.42), (0., -.42), (-.2, -.42), (-.4, -.4)]
        commands = [control.step([target(x, y)], [(x, y)]) for x, y in observations]
        self.assertGreater(commands[0][1], 0)
        self.assertTrue(all(row[0] > 0 for row in commands[:-1]))
        self.assertEqual(commands[-1], (0, 0, 'passed_target_stop'))

    def test_missing_target_and_collision_stop(self):
        control = LeftPass()
        self.assertEqual(control.step([], [])[0], 0)
        control.step([target(1., 0.)], [(1., 0.)])
        self.assertEqual(control.step([], [])[2], 'target_missing_stop')
        control = LeftPass()
        self.assertEqual(control.step([target(.8, 0.)], [(.10, 0.)])[2],
                         'predicted_collision_stop')

    def test_clusters_reject_sparse_returns(self):
        ranges = [float('inf')]*20
        for i in (8, 9, 10, 11):
            ranges[i] = .8
        self.assertEqual(len(clusters(ranges, -.2, .02, .05, 3.)), 1)
        self.assertEqual(clusters([float('inf'), .8, float('inf')],
                                  -.2, .02, .05, 3.), [])

    def test_clearance_changes_left_steering_and_side_guard(self):
        close = LeftPass(clearance=.10)
        far = LeftPass(clearance=.30)
        obstacle = target(1., 0.)
        self.assertGreater(far.step([obstacle], [(1., 0.)])[1],
                           close.step([obstacle], [(1., 0.)])[1])
        obstacle = target(.2, -.30)
        far.target = target(.4, -.3)
        self.assertEqual(far.step([obstacle], [(.2, -.30)])[2],
                         'side_clearance_stop')


if __name__ == '__main__':
    unittest.main()
