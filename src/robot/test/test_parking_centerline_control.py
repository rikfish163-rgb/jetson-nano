"""Measured bay center tracking for either side and either motion direction."""
import copy
import math
import unittest
from test_core import CONFIG
from robot.common.geometry import local
from robot.parking.entry import ExplicitStraightEntry
from robot.parking.reverse_entry import ReverseStraightEntry


class ParkingCenterlineTests(unittest.TestCase):
    def cfg(self, retreat=0.):
        cfg=copy.deepcopy(CONFIG)
        cfg.update(parking_p_reverse_s=retreat,parking_entry_speed_raw=16,
                   steering_command_scale_rad=.03,parking_sign_association=True)
        return cfg

    def lines(self, center=.08):
        return [[(.4,center+.19),(1.,center+.19)],
                [(.4,center-.19),(1.,center-.19)]]

    def test_pair_left_and_right_infer_same_center_and_steering(self):
        for center in (-.12,.12):
            commands=[]
            lines=self.lines(center)
            for observation,source in ((lines,'both_sides'),(lines[:1],'left_side'),
                                       (lines[1:],'right_side')):
                entry=ExplicitStraightEntry(self.cfg(),(0.,0.,0.),1.,(1.,center))
                entry.observe(observation,1.1,(0.,0.,0.))
                commands.append(entry.command(1.1,(0.,0.,0.)))
                ref=entry.view or entry.single
                self.assertIsNotNone(ref)
                self.assertAlmostEqual(ref['center'][1],center)
                self.assertEqual(entry.debug['steering_reference'],source)
                self.assertGreater(abs(commands[-1][1]),6./22*.03)
                self.assertLessEqual(abs(commands[-1][1]),.03)
            self.assertEqual(commands[0],commands[1])
            self.assertEqual(commands[0],commands[2])

    def test_pair_to_either_short_side_keeps_center_correction_with_bottom(self):
        for index,source in ((0,'left_side'),(1,'right_side')):
            entry=ExplicitStraightEntry(self.cfg(),(0.,0.,0.),1.,(1.,.08))
            lines=self.lines()
            bottom=[(1.,-.11),(1.,.27)]
            entry.observe(lines+[bottom],1.1,(0.,0.,0.))
            entry.command(1.1,(0.,0.,0.))
            y=lines[index][0][1]
            entry.observe([[(.9,y),(1.,y)],bottom],1.2,(0.,0.,0.))
            speed,steer=entry.command(1.2,(0.,0.,0.))
            self.assertEqual(speed,16)
            self.assertGreater(steer,0.)
            self.assertEqual(entry.debug['steering_reference'],source)

    def test_real_pair_overrides_wrong_single_side_identity(self):
        # Construct the reported failure: a P beside the right rail makes
        # that rail look like a left edge when it is the only visible line.
        for rear in (False,True):
            for anchor_y in (-.16,-.24):
                pose=(0.,0.,0.)
                if rear:
                    entry=ReverseStraightEntry(self.cfg(),pose,1.,wait_for_sign=True)
                    entry.observe_sign('PARKING',.99,1.1,1.1,anchor=(-1.,anchor_y))
                else:
                    entry=ExplicitStraightEntry(self.cfg(),pose,1.,(1.,anchor_y))
                direction=-1 if rear else 1
                right=[(direction*.4,0.),(direction*1.,0.)]
                left=[(direction*.4,.38),(direction*1.,.38)]
                entry.observe([right],1.5,pose)
                entry.command(1.6,pose)
                self.assertIsNone(entry.view)
                self.assertAlmostEqual(local(entry.inner.pose,entry.single['center'])[1],-.19)
                entry.observe([right,left],1.7,pose)
                entry.command(1.7,pose)
                self.assertIsNotNone(entry.view)
                self.assertEqual(entry.debug['steering_reference'],'both_sides')
                self.assertAlmostEqual(local(entry.inner.pose,entry.view['center'])[1],.19)
                # Losing the actual left rail now identifies the remaining
                # rail from the pair, rather than reinterpreting it using P.
                entry.observe([right],1.8,pose)
                entry.command(1.8,pose)
                self.assertEqual(entry.inner.single['source'],'right_side')
                self.assertAlmostEqual(local(entry.inner.pose,entry.single['center'])[1],.19)

    def test_single_to_pair_upgrade_keeps_width_and_parallel_checks(self):
        for y,end_y in ((.10,.10),(.60,.60),(.38,.58)):
            entry=ExplicitStraightEntry(self.cfg(),(0.,0.,0.),1.,(1.,-.16))
            right=[(.4,0.),(1.,0.)]
            entry.observe([right],1.1,(0.,0.,0.))
            entry.command(1.1,(0.,0.,0.))
            entry.observe([right,[(.4,y),(1.,end_y)]],1.2,(0.,0.,0.))
            self.assertIsNone(entry.view)

    def test_recorded_board_beside_bay_keeps_both_sides_during_retreat(self):
        # 20261012_015853: P projects 12.9 cm outside the right edge.
        anchor=(1.564797519959767,-.03517184211047111)
        lines=[[(1.1125,.3975),(1.5,.4575)],
               [(1.1775,.045),(1.4975,.085)]]
        entry=ExplicitStraightEntry(self.cfg(3.),(0.,0.,0.),1.,anchor)
        entry.command(1.1,(0.,0.,0.))
        entry.observe(lines,1.5,(0.,0.,0.))
        speed,steer=entry.command(1.6,(0.,0.,0.))
        self.assertEqual(speed,-16)
        self.assertEqual(entry.debug['steering_reference'],'both_sides')
        self.assertAlmostEqual(entry.view['width'],.3673387096774194)
        self.assertGreater(entry.view['center'][1],.2)
        # The old left-only fallback inferred a center on the opposite side.
        self.assertLess(abs(steer),.03)

    def test_front_retreat_uses_lines_and_reverses_heading_feedback(self):
        for heading in (-.08,.08):
            entry=ExplicitStraightEntry(self.cfg(3.),(0.,0.,heading),1.,(1.,0.))
            entry.command(1.1,(0.,0.,heading))
            entry.observe(self.lines(0.),1.5,(0.,0.,heading))
            speed,steer=entry.command(1.6,(0.,0.,heading))
            self.assertEqual(speed,-16)
            self.assertEqual(entry.debug['steering_reference'],'both_sides')
            # Reverse motion must reduce yaw error, not increase it.
            next_heading=heading-.1*math.tan(steer/.03*entry.cfg['max_steer'])/entry.cfg['wheelbase']
            self.assertLess(abs(next_heading),abs(heading))

    def test_rear_entry_and_retreat_have_correct_physical_steering(self):
        for retreat in (0.,3.):
            pose=(0.,0.,.08)
            entry=ReverseStraightEntry(self.cfg(retreat),pose,1.,wait_for_sign=True)
            entry.observe_sign('PARKING',.99,1.1,1.1,anchor=(-1.,0.))
            entry.command(1.5,pose)
            lines=[[(-x,y) for x,y in line] for line in self.lines(0.)]
            entry.observe(lines,1.9,pose)
            speed,steer=entry.command(2.,pose)
            self.assertEqual(speed,16 if retreat else -16)
            next_heading=pose[2]+(.1 if retreat else -.1)*math.tan(steer/.03*entry.cfg['max_steer'])/entry.cfg['wheelbase']
            self.assertLess(abs(next_heading),abs(pose[2]))
            self.assertGreater(abs(steer),10./22*.03)


if __name__=='__main__':unittest.main()
