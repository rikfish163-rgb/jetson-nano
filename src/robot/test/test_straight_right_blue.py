"""Straight tracks a parallel path 30 cm left of the right blue tape."""
import math
import unittest
import test_direction_single_frame as fixtures
from robot.common.contracts import encode_command
from blue_test_helpers import enter_blue_action, observe_direction


class RightBlueStraightTests(unittest.TestCase):
    def core(self):
        c = fixtures.DirectionSingleFrameTests.__dict__['core'](self)
        c.cfg.update(steering_command_scale_rad=.1, blue_default_straight=False)
        return c

    def ground(self, c, now, lines=(), trigger=False):
        c.observe_ground(dict(source='front', part='markers', slots=[],
            markers=[dict(kind='junction', x=.3, y=0)] if trigger else [],
            blue_lines=list(lines)), now)

    def line(self, angle=.15, x=.6, y=None):
        if y is None:
            y=(-.3+x*math.sin(angle))/math.cos(angle) if abs(math.cos(angle))>.01 else -.3
        return dict(x=x, y=y, yaw=angle-math.pi/2, length=.7)

    def test_parallel_line_distance_corrects_both_lateral_directions(self):
        for y,side in ((-.45,-1),(-.15,1),(-.30,0)):
            c=self.core();self.start(c)
            self.ground(c,1.2,[self.line(angle=0,y=y)])
            speed,steer=c.tick(1.2)
            self.assertEqual(speed,c.cfg['straight_speed_raw'])
            if side:self.assertGreater(steer*side,0)
            else:self.assertAlmostEqual(steer,0)

    def test_offset_is_perpendicular_and_independent_of_segment_center(self):
        commands=[]
        for x in (.5,1.1):
            c=self.core();self.start(c)
            self.ground(c,1.2,[self.line(angle=.2,x=x)])
            commands.append(c.tick(1.2)[1])
            self.assertAlmostEqual(c.straight_search['right_blue']['distance_m'],.3)
        self.assertAlmostEqual(commands[0],commands[1])

    def test_straight_entry_clears_consumed_sign_votes_immediately(self):
        c=self.core();observe_direction(c,'STRAIGHT',.99,.7)
        c.sign_label='STRAIGHT';c.sign_count=2
        c.direction_window=[('RIGHT',.6)]
        c.next_direction='RIGHT';c.next_direction_at=.6
        enter_blue_action(c,'STRAIGHT',1.1)
        self.assertIsNone(c.pending)
        self.assertEqual(c.pending_at,0.)
        self.assertEqual(c.sign_label,'')
        self.assertEqual(c.sign_count,0)
        self.assertEqual(c.direction_window,[])
        self.assertIsNone(c.next_direction)
        self.assertEqual(c.action,'STRAIGHT')

    def test_right_turn_exit_straight_also_tracks_right_blue(self):
        c=self.core();self.start(c);c.action_source='right_blue'
        self.ground(c,1.2,[self.line(angle=0,y=-.45)])
        self.assertLess(c.tick(1.2)[1],0)

    def test_bicycle_motion_reduces_offset_error_within_straight_segment(self):
        from robot.common.geometry import local,bicycle
        from robot.common.contracts import command_to_model_steering
        for initial_distance in (.15,.45):
            c=self.core();self.start(c)
            for i in range(200):
                t=1.2+i*.05
                x,y=local(c.pose,(c.pose[0]+.8,-initial_distance))
                self.ground(c,t,[self.line(angle=-c.pose[2],x=x,y=y)])
                c.observe_lane([(.3,0),(.6,0)],.99,t)
                speed,steer=c.tick(t)
                if c.action is None:break
                c.set_pose(bicycle(c.pose,speed*c.cfg['raw_to_mps']['forward']*.05,
                    command_to_model_steering(steer,c.cfg),c.cfg['wheelbase']),t+.05)
            self.assertIsNone(c.action)
            self.assertLess(abs(initial_distance+c.pose[1]-.30),.06)

    def start(self, c):
        observe_direction(c,'STRAIGHT', .99, .7)
        enter_blue_action(c,'STRAIGHT',1.1)
        self.assertEqual(c.straight_search['phase'],'STRAIGHT_DISTANCE')

    def test_both_correction_directions_keep_speed_and_bound_raw_steering(self):
        for angle in (-.3, .3):
            c = self.core(); self.start(c)
            self.ground(c, 1.2, [self.line(angle)])
            speed, steer = c.tick(1.2)
            self.assertEqual(speed, c.cfg['straight_speed_raw'])
            self.assertGreater(steer*angle, 0)
            self.assertLessEqual(abs(encode_command(speed, steer, c.cfg, 0)['steering_raw']), 6)
            self.assertEqual(c.reason, 'straight_align_right_blue')

    def test_missing_stale_left_and_transverse_lines_keep_zero_steering(self):
        for lines, delay in (([], 0), ([self.line(y=.3)], 0),
                ([self.line(angle=math.pi/2)], 0), ([self.line()], .6),
                ([self.line(angle=0)], 0)):
            c = self.core(); self.start(c)
            self.ground(c, 1.2, lines)
            self.assertEqual(c.tick(1.2+delay), (c.cfg['straight_speed_raw'], 0.))

    def test_loss_clears_previous_correction(self):
        c = self.core(); self.start(c)
        self.ground(c, 1.2, [self.line()]); self.assertNotEqual(c.tick(1.2)[1], 0)
        self.ground(c, 1.3)
        self.assertEqual(c.tick(1.3), (c.cfg['straight_speed_raw'], 0.))

    def test_correction_uses_local_heading_and_does_not_delay_lane_handoff(self):
        c = self.core(); c.set_pose((2, 3, math.pi/2), .9); self.start(c)
        origin=c.pose
        c.set_pose((2, origin[1]+1.249, math.pi/2), 1.2)
        self.ground(c, 1.2, [self.line()])
        self.assertGreater(c.tick(1.2)[1], 0)
        c.set_pose((2, origin[1]+1.251, math.pi/2), 1.3)
        self.ground(c, 1.3, [self.line()])
        c.observe_lane([(.3, 0), (.6, 0)], .99, 1.3)
        c.tick(1.3)
        for t in (1.4,1.5):
            c.observe_lane([(.3,0),(.6,0)],.99,t)
            command=c.tick(t)
        self.assertAlmostEqual(command[1], 0)
        self.assertEqual(c.state, 'LANE')

    def test_reversed_endpoints_same_heading_and_red_still_stops(self):
        c = self.core(); self.start(c)
        line = self.line()
        self.ground(c, 1.2, [line]); before = c.tick(1.2)
        line['yaw'] += math.pi
        self.ground(c, 1.3, [line])
        self.assertAlmostEqual(c.tick(1.3)[1], before[1])
        c.red = True
        self.assertEqual(c.tick(1.3), (0, 0.))

    def test_sign_still_waits_for_blue_and_green_does_not_use_side_line(self):
        c = self.core(); observe_direction(c,'STRAIGHT', .99, 1.)
        self.ground(c, 1.1, [self.line()]); c.tick(1.1)
        self.assertIsNone(c.straight_search)
        c = self.core(); c.state = 'WAIT_GREEN'
        c.observe_sign('GREEN', .99, 1., 1.)
        self.ground(c, 1.1, [self.line()])
        self.assertEqual(c.tick(1.1), (c.cfg['speed_raw']['lane'], 0.))


if __name__ == '__main__':
    unittest.main()
