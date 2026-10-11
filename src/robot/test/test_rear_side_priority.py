"""Either edge of the P bay can steer; blue must not override its correction."""
import copy
import unittest
from test_core import CONFIG
from robot.common.geometry import bicycle
from robot.parking.reverse_entry import ReverseStraightEntry


class RearSidePriorityTests(unittest.TestCase):
    def entry(self,offset):
        cfg=copy.deepcopy(CONFIG)
        cfg.update(parking_mode='forward_center',parking_entry_style='S',
                   steering_command_scale_rad=.03)
        e=ReverseStraightEntry(cfg,(0.,0.,0.),10.,wait_for_sign=True)
        e.observe_sign('PARKING',.99,10.1,10.2,anchor=(-.9,offset))
        return e

    def test_either_measured_side_moves_rear_towards_bay(self):
        for offset in (-.06,.06):
            for side in (-.19,.19):
                e=self.entry(offset)
                e.observe([[(-.3,offset+side),(-.85,offset+side)]],10.3,(0.,0.,0.))
                speed,steer=e.command(10.5,(0.,0.,0.))
                self.assertLess(speed,0)
                moved=bicycle((0.,0.,0.),-.1,steer,e.cfg['wheelbase'])
                self.assertGreater(moved[1]*offset,0.)
                self.assertLess(e.debug['rear_vehicle_lateral_m']*offset,0.)
                self.assertEqual(e.reason,'parking_follow_right_side' if side>0 else 'parking_follow_left_side')

    def test_blue_cannot_reverse_or_replace_side_correction(self):
        for offset in (-.025,.025):
            e=self.entry(offset)
            lines=[[(-.3,y),(-.85,y)] for y in (offset-.19,offset+.19)]
            e.observe(lines,10.3,(0.,0.,0.))
            expected=e.command(10.5,(0.,0.,0.))[1]
            for source,x in (('front',.6),('rear',-.6)):
                for stamp in (10.6,10.7):
                    e.observe_blue([dict(point=(x,0.),yaw=.20 if offset>0 else -.20,length=.7)],
                                   stamp,(0.,0.,0.),source=source)
                actual=e.command(10.8,(0.,0.,0.))[1]
                self.assertEqual(actual,expected)
                self.assertFalse(e.debug['blue_used'])
                self.assertEqual(e.debug['blue_reject_reason'],'bay_edges_priority')


if __name__=='__main__':unittest.main()
