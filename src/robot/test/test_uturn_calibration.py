import math
import os
import unittest
import yaml
from robot.uturn.calibration import limit
from robot.uturn.calibration import command_angle
from robot.uturn.calibration import validate
from robot.uturn.planner import plan_uturn
from robot.common.geometry import bicycle
from robot.common.geometry import distance
from robot.common.geometry import wrap
from robot.uturn.relative import UturnFollower
from robot.common.contracts import encode_command


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        root=os.path.join(os.path.dirname(__file__),'../config')
        self.cfg=yaml.safe_load(open(os.path.join(root,'competition.yaml')))
        self.cfg.update(yaml.safe_load(open(os.path.join(root,'uturn_vision.yaml'))))
        self.cfg['steering_command_scale_rad']=.1

    def test_forward_wheel_endpoints_and_command_mapping(self):
        # Historical reverse lateral signs conflict with the direct yaw test;
        # they must not be treated as validated endpoint ground truth.
        for gear,side,x,y in [(1,1,.355,.523),(1,-1,.53,-.42)]:
            angle=gear*side*limit(self.cfg,gear,gear*side)
            radius=self.cfg['wheelbase']/abs(math.tan(angle))
            # Forward: front wheel on steering side. Reverse: rear wheel on
            # steering side. The observed displacement is not the axle center.
            wheel_x=self.cfg['wheelbase'] if gear>0 else 0.
            wheel_y=side*.12
            heading=wrap(2*math.atan2(y,x+2*wheel_x))
            length=radius*abs(heading)
            end=bicycle((0,0,0),gear*length,angle,self.cfg['wheelbase'])
            observed_x=end[0]+wheel_x*(math.cos(end[2])-1)-wheel_y*math.sin(end[2])
            observed_y=end[1]+wheel_x*math.sin(end[2])+wheel_y*(math.cos(end[2])-1)
            self.assertAlmostEqual(observed_x,x,places=5)
            self.assertAlmostEqual(observed_y,y,places=5)
            self.assertAlmostEqual(command_angle(self.cfg,gear,angle),side*.1)
            raw=encode_command(gear*26,command_angle(self.cfg,gear,angle),self.cfg,0)
            self.assertEqual(raw['steering_raw'],side*22*self.cfg['steering_sign'])

    def test_reverse_left_turns_nose_right(self):
        # User's isolated test: start north, reverse with RAW +22, finish NE.
        angle=limit(self.cfg,-1,1)
        command=command_angle(self.cfg,-1,angle)
        self.assertEqual(encode_command(-26,command,self.cfg,0)['steering_raw'],22)
        end=bicycle((0,0,0),-.2,angle,self.cfg['wheelbase'])
        self.assertLess(end[2],0)

    def test_bad_radius_rejected(self):
        self.cfg['uturn_calibration']['reverse_right_radius']=-1
        with self.assertRaises(ValueError):validate(self.cfg)

    def test_bad_table_rejected(self):
        self.cfg['uturn_calibration']=[]
        with self.assertRaises(ValueError):validate(self.cfg)

    def test_search_uses_direction_specific_motion(self):
        path,reason=plan_uturn((0,0,0),'RIGHT',self.cfg,
            lambda q:-.5<=q[0]<=1.6 and -.27<=q[1]<=.87)
        self.assertTrue(path,reason)
        self.assertIn(-1,[p[3] for p in path])
        for a,b in zip(path,path[1:]):
            steer=b[4];gear=b[3]
            self.assertLessEqual(abs(steer),limit(self.cfg,gear,steer)+1e-9)
            chord=distance(a,b)
            k=math.tan(steer)/self.cfg['wheelbase']
            length=2*math.asin(min(1.,chord*abs(k)/2))/abs(k) if abs(k)>1e-9 else chord
            expected=bicycle(a[:3],gear*length,steer,self.cfg['wheelbase'])
            self.assertLess(distance(expected,b),1e-6)
            self.assertLess(abs(wrap(expected[2]-b[2])),1e-6)

    def test_reverse_follower_does_not_exceed_weak_right_lock(self):
        f=UturnFollower([(0,0,0,-1,0),(-.2,-.2,0,-1,0)],self.cfg)
        speed,angle=f.command((0,0,0),0)
        self.assertLess(speed,0)
        self.assertLess(angle,0)
        self.assertLessEqual(abs(angle),limit(self.cfg,-1,-1))
        self.assertLess(command_angle(self.cfg,-1,angle),0)

    def test_shift_right_with_asymmetric_model(self):
        path,reason=plan_uturn((0,0,0),'LEFT',self.cfg,
            lambda q:-.5<=q[0]<=1.6 and -.87<=q[1]<=.27)
        self.assertTrue(path,reason)
        self.assertLess(distance(path[-1],(0,-.6)),.045)
        self.assertLess(abs(wrap(path[-1][2]-math.pi)),math.radians(10))


if __name__=='__main__':unittest.main()
