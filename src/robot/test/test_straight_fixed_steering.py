"""Fixed steering applies only to the post-blue signed STRAIGHT segment."""
import os
import unittest
from robot.common.config import load_config
from robot.common.contracts import encode_command, validate_config
from robot.master.controller import Controller
from blue_test_helpers import enter_blue_action, observe_direction


class FixedStraightTests(unittest.TestCase):
    def core(self):
        cfg=load_config(os.path.join(os.path.dirname(os.path.dirname(__file__)),'config'))
        cfg.update(wait_green=False,lidar_enabled=False,steering_command_scale_rad=.03,
                   straight_steering_raw=-2)
        validate_config(cfg)
        c=Controller(cfg);self.addCleanup(c.close)
        return c

    def start(self,c):
        observe_direction(c,'STRAIGHT',.99,.6)
        enter_blue_action(c,'STRAIGHT',1.)
        self.assertEqual(c.action_source,'sign')

    def test_fixed_raw_with_and_without_opposing_blue_reference(self):
        for steering_sign in (1,-1):
            for lines in ([],[dict(x=.6,y=-.15,yaw=-1.57079632679,length=.7)]):
                c=self.core();c.cfg['steering_sign']=steering_sign
                self.start(c)
                c.observe_ground(dict(source='front',part='markers',markers=[],slots=[],blue_lines=lines),1.1)
                command=c.tick(1.1)
                self.assertEqual(encode_command(command[0],command[1],c.cfg,0)['steering_raw'],-2)
                self.assertEqual(c.reason,'straight_fixed_steer')
                self.assertIsNone(c.straight_search['right_blue'])

    def test_distance_completion_returns_to_normal_lane_command(self):
        c=self.core();self.start(c)
        c.pose=(c.cfg.get('straight_distance',1.25)+.01,0.,0.)
        c.front_marker_stamp=1.1
        c.lane_command=lambda now:(30,.009)
        self.assertEqual(c.tick(1.1),(30,.009))
        self.assertEqual(c.state,'LANE')

    def parking_straight(self):
        c=self.core()
        c.cfg.update(parking_enabled=True,parking_mode='forward_center',
                     parking_entry_style='S',parking_slot='P4',
                     straight_speed_raw=30,parking_entry_speed_raw=16,
                     parking_p_reverse_s=0.)
        self.start(c)
        return c

    def confirm_parking(self,c):
        start=c.route_action_started+c.cfg['sign_timeout']+.1
        for i in range(c.cfg.get('parking_sign_votes',3)):
            now=start+i*.1
            c.observe_sign('PARKING',.99,now,now)
        self.assertEqual(c.next_direction,'PARKING')
        c.front_marker_stamp=now
        return now

    def test_confirmed_p_interrupts_straight_and_centers_until_blue_net_distance(self):
        c=self.parking_straight()
        c.front_marker_stamp=1.1
        self.assertEqual(c.tick(1.1)[0],30)
        now=self.confirm_parking(c)
        c.set_pose((.2,0.,0.),now)
        origin=dict(c.parking_straight_travel)
        self.assertEqual(c.tick(now),(0,0.))
        self.assertEqual(c.state,'PARKING')
        self.assertEqual(c.action,'PARKING')
        self.assertIsNone(c.straight_search)
        self.assertIsNone(c.next_direction)
        self.assertIsNone(c.pending)
        self.assertEqual(c.parking_straight_travel['stamp'],origin['stamp'])
        cross_x=c.last_blue_trigger['point_world'][0]-c.cfg['wheelbase']
        self.assertAlmostEqual(c.parking_straight_travel['travelled_m'],.2-cross_x)
        lines=[[(.4,-.19),(1.,-.19)],[(.4,.19),(1.,.19)],
               [(1.,-.19),(1.,.19)]]
        c.observe_ground(dict(source='front',part='parking_lines',lines=lines),now+.1)
        self.assertEqual(c.tick(now+.1)[0],16)
        self.assertEqual(c.parking_entry.debug['steering_reference'],'both_sides')
        self.assertIsNotNone(c.parking_entry.bottom)
        entry=c.parking_entry
        c.observe_sign('',0.,now+.2,now+.2)
        c.tick(now+.2)
        self.assertIs(c.parking_entry,entry)
        stop_x=cross_x+c.cfg['parking_s_blue_distance_m']
        c.set_pose((stop_x,0.,0.),now+.3)
        self.assertEqual(c.tick(now+.3),(0,0.))
        self.assertEqual(c.state,'FINISHED')
        self.assertEqual(c.reason,'parking_blue_net_distance_reached')

    def test_unconfirmed_p_does_not_slow_straight(self):
        c=self.parking_straight()
        now=c.route_action_started+c.cfg['sign_timeout']+.1
        c.observe_sign('PARKING',.99,now,now)
        self.assertIsNone(c.next_direction)
        c.front_marker_stamp=now
        self.assertEqual(c.tick(now)[0],30)

    def test_direct_p_handoff_preserves_projected_sign_anchor(self):
        c=self.parking_straight()
        now=self.confirm_parking(c)
        c.parking_sign=dict(stamp=now,point=(1.,.04))
        c.tick(now)
        self.assertEqual(c.state,'PARKING')
        self.assertEqual(c.parking_entry.sign_anchor,(1.,.04))
        c.observe_ground(dict(source='front',part='parking_lines',
                             lines=[[(.4,.23),(1.,.23)]]),now+.1)
        self.assertEqual(c.tick(now+.1)[0],16)
        self.assertEqual(c.parking_entry.debug['steering_reference'],'left_side')

    def test_p_handoff_does_not_burst_back_to_lane_speed(self):
        c=self.parking_straight()
        now=self.confirm_parking(c)
        c.set_pose((c.cfg['straight_distance'],0.,0.),now)
        def unexpected_lane(stamp):
            self.fail('confirmed P must bypass the lane handoff')
        c.lane_command=unexpected_lane
        self.assertEqual(c.tick(now),(0,0.))
        self.assertEqual(c.state,'PARKING')

    def test_right_exit_straight_does_not_use_sign_trim(self):
        c=self.core();self.start(c);c.action_source='right_blue'
        c.front_marker_stamp=1.1
        self.assertEqual(c.tick(1.1)[1],0.)

    def test_stale_perception_still_stops(self):
        c=self.core();self.start(c)
        self.assertEqual(c.tick(3.)[0],0)
        self.assertEqual(c.reason,'straight_front_stale')

    def test_green_startup_is_independent_of_straight_trim(self):
        commands=[]
        for raw in (-2,0,2):
            c=self.core();c.cfg['straight_steering_raw']=raw
            c.begin_startup(1.)
            c.front_marker_stamp=1.1
            commands.append(c.tick(1.1))
        self.assertEqual(commands[0],commands[1])
        self.assertEqual(commands[1],commands[2])

    def test_invalid_raw_rejected(self):
        for value in (23,-23,.5,float('nan')):
            c=self.core();c.cfg['straight_steering_raw']=value
            with self.assertRaises(ValueError):validate_config(c.cfg)


if __name__=='__main__':unittest.main()
