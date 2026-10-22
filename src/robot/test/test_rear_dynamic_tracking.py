"""Moving-car regressions against front S; no ROS or chassis commands."""
import copy
import math
import unittest
from test_core import CONFIG
from robot.common.geometry import world
from robot.parking.entry import ExplicitStraightEntry
from robot.parking.reverse_entry import ReverseStraightEntry


class RearDynamicTrackingTests(unittest.TestCase):
    def cfg(self, retreat=0.):
        cfg=copy.deepcopy(CONFIG)
        cfg.update(parking_p_reverse_s=retreat,steering_command_scale_rad=.03,
                   pose_mode='command_model',parking_entry_speed_raw=16)
        return cfg

    def rear(self, anchor=(-.95,.03), retreat=0.):
        rear=ReverseStraightEntry(self.cfg(retreat),(0.,0.,0.),10.,wait_for_sign=True)
        rear.observe_sign('PARKING',.99,10.1,10.1,anchor=anchor)
        return rear

    def rails(self, center=.03):
        return [[(-.3,y),(-.9,y)] for y in (center-.19,center+.19)]

    def test_same_bay_during_real_yaw_change_matches_front_s(self):
        for retreat in (0.,3.):
            for yaw in (-.22,.22):
                rear=self.rear(retreat=retreat)
                front=ExplicitStraightEntry(self.cfg(retreat),(0.,0.,0.),10.,(.95,.03))
                lines=self.rails()
                front_lines=[[(-x,y) for x,y in line] for line in lines]
                for entry,geometry in ((rear,lines),(front,front_lines)):
                    entry.command(10.4,(0.,0.,0.))
                    entry.observe(geometry,10.5,(0.,0.,0.))
                    entry.command(10.8,(0.,0.,0.))
                pose=(.04,0.,yaw)
                front_pose=(-pose[0],pose[1],-pose[2])
                # Fixed ground rails, new image after actual translation/yaw.
                rear.observe(lines,11.1,pose)
                front.observe(front_lines,11.1,front_pose)
                self.assertIsNotNone(front.view)
                self.assertIsNotNone(rear.view)
                a=front.command(11.1,front_pose)
                b=rear.command(11.1,pose)
                self.assertEqual(a[0],-b[0])
                self.assertAlmostEqual(a[1],b[1])
                self.assertEqual(rear.debug['steering_reference'],'both_sides')

    def test_measured_single_after_real_turn_keeps_the_confirmed_bay(self):
        for yaw in (-.22,.22):
            rear=self.rear()
            lines=self.rails()
            rear.observe(lines,10.2,(0.,0.,0.))
            rear.command(10.5,(0.,0.,0.))
            pose=(.04,0.,yaw)
            rear.observe(lines[1:],10.6,pose)
            self.assertIsNotNone(rear.single)
            self.assertAlmostEqual(rear.single['center'][1],-.03)

    def test_far_cropped_pair_survives_retreat_before_leaving_image(self):
        rear=self.rear(anchor=(-2.4,.03))
        lines=[[(-.40,-.16),(-1.05,-.16)],
               [(-.84,.22),(-1.05,.22)]]
        rear.observe(lines,10.2,(0.,0.,0.))
        self.assertIsNotNone(rear.view)
        pose=(.1,0.,0.)
        # Both rails still visible: far rail has 11 cm left at the BEV edge.
        observed=[[(-.50,-.16),(-1.05,-.16)],
                  [(-.94,.22),(-1.05,.22)]]
        rear.observe([[world(pose,p) for p in side] for side in observed],10.6,pose)
        self.assertIsNotNone(rear.view)
        rear.command(10.7,pose)
        self.assertEqual(rear.debug['steering_reference'],'both_sides')

    def test_short_near_fragments_do_not_count_as_two_far_cropped_rails(self):
        rear=self.rear(anchor=(-.7,.03))
        rear.observe([[(-.4,y),(-.51,y)] for y in (-.16,.22)],10.2,(0.,0.,0.))
        self.assertIsNone(rear.view)

    def test_long_measured_side_wins_over_short_hough_angle_fragments(self):
        for slope in (-.08,.08):
            rear=self.rear(anchor=(-.95,0.))
            rear.observe(self.rails(0.),10.2,(0.,0.,0.))
            rear.command(10.5,(0.,0.,0.))
            long_side=[(-.3,.22),(-.9,.22)]
            short=[(-x,.19*math.sqrt(1+slope*slope)+slope*(x-.6))
                   for x in (.65,.70)]
            rear.observe([short,long_side],10.6,(0.,0.,0.))
            rear.command(10.7,(0.,0.,0.))
            self.assertIsNotNone(rear.single)
            side=rear.single['side']
            self.assertGreater(math.hypot(side[1][0]-side[0][0],side[1][1]-side[0][1]),.5)
            self.assertAlmostEqual(rear.debug['centerline_heading_error_deg'],0.)

    def test_long_opposite_side_wins_over_preferred_side_fragment(self):
        rear=self.rear(anchor=(-.95,0.))
        rear.observe(self.rails(0.),10.2,(0.,0.,0.))
        rear.command(10.5,(0.,0.,0.))
        rear.observe([[(-.65,.19),(-.70,.19)],
                      [(-.3,-.22),(-.9,-.22)]],10.6,(0.,0.,0.))
        speed,steer=rear.command(10.7,(0.,0.,0.))
        self.assertEqual(rear.debug['steering_reference'],'left_side')
        self.assertLess(steer,0.)


if __name__=='__main__':unittest.main()
