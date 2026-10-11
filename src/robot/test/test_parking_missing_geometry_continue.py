"""Missing S bay lines do not issue a wait/stop command."""
import copy
import unittest
from test_core import CONFIG
from robot.parking.entry import ExplicitStraightEntry


class MissingGeometryContinueTests(unittest.TestCase):
    def test_initial_missing_and_lost_pair_keep_entry_speed_then_reacquire(self):
        cfg=copy.deepcopy(CONFIG)
        cfg.update(parking_p_reverse_s=0.,parking_entry_speed_raw=16)
        entry=ExplicitStraightEntry(cfg,(0.,0.,0.),1.)
        entry.observe([],1.1,(0.,0.,0.))
        self.assertEqual(entry.command(1.1,(0.,0.,0.)),(16,0.))
        self.assertFalse(entry.debug['geometry_ready'])
        lines=[[(.4,-.11),(1.,-.11)],[(.4,.27),(1.,.27)]]
        entry.observe(lines,1.2,(0.,0.,0.))
        self.assertGreater(entry.command(1.2,(0.,0.,0.))[1],0)
        entry.observe([],1.3,(0.,0.,0.))
        self.assertEqual(entry.command(1.3,(0.,0.,0.)),(16,0.))
        self.assertFalse(entry.debug['geometry_ready'])
        self.assertEqual(entry.reason,'parking_missing_white_straight')
        entry.observe(lines,1.4,(0.,0.,0.))
        self.assertGreater(entry.command(1.4,(0.,0.,0.))[1],0)
        self.assertTrue(entry.debug['geometry_ready'])


if __name__=='__main__':unittest.main()
